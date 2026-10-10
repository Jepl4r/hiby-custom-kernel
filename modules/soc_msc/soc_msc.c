// SPDX-License-Identifier: GPL-2.0
//
// soc_msc -- the two MMC/SD controllers of the X1600 as MMC hosts: msc0, the
// SDIO bus of the Wi-Fi chip, and msc1, the microSD slot.
//
// Based on the Ingenic SDK's drivers/mmc/host/ingenic_mmc.c, Copyright (C)
// 2012 Ingenic Semiconductor Co., Ltd., written by Large Dipper
// <ykli@ingenic.com> and modified by qipengzhen <aric.pzqi@ingenic.com>.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock soc_msc.sh passes them). For each controller N:
//
//   mscN_is_enable          1: register the controller
//   mscN_voltage_min/max    the card voltage range, in mV
//   mscN_bus_width          1, 4 or 8 data lines
//   mscN_rm_method          dontcare, nonremovable, removable (a card-detect
//                           pin) or manual (jzmmc_manual_detect(), for the
//                           Wi-Fi chip)
//   mscN_cd_method          non-removable, broken-cd (polled) or anything else
//   mscN_speed              sd_card, sdio or emmc: the timings offered
//   mscN_max_frequency      the bus clock limit, in Hz
//   mscN_cap_*, mscN_full_pwr_cycle, mscN_keep_power_in_suspend,
//   mscN_enable_sdio_wakeup host capabilities for the MMC core
//   mscN_dsr                the driver stage register value (0: none)
//   mscN_pio_mode           move all data through the FIFO, without DMA
//   mscN_sdio_clk           keep the bus clock running between commands
//   mscN_cd, mscN_wp, mscN_pwr, mscN_rst, mscN_sdr
//                           pins by name ("PB22", -1 for none), each with
//                           its mscN_*_enable_level (0: active low)
//   mscN_pwr_regulator      a regulator for the card power ("-1": none,
//                           the pwr pin is used)
//
// and wifi_power_on, wifi_reg_on: pins driven to their *_level at load.
//
// Requests are served from the controller's threaded interrupt: a command
// and its response, then the data by descriptor DMA (straight from the
// scatterlist, up to 128 segments) or, for a single segment below 64 bytes,
// through the FIFO. A timer catches a request that never completes. The
// card-detect pin is debounced by 200 ms; a card found is announced to the
// core 1 s later. The bus clock pin is a GPIO input whenever the controller
// clocks are off.
//
// /sys/devices/platform/md_ingenic,mmc.N/present reads Y or N; on a
// nonremovable host INSERT or REMOVE written there sets it.

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <asm/addrspace.h>
#include <soc/gpio.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern int gpio_set_func(int gpio, int func);
extern char *gpio_to_str(int gpio, char *buf);

// Exported by the kernel (module_drivers/drivers/mmc/host/ingenic_sdio.c).
extern int ingenic_sdio_wlan_init(struct device *dev, int index);

// Exported for the Wi-Fi power driver.
int jzmmc_manual_detect(int index, int on);
int jzmmc_clk_ctrl(int index, int on);
int jzmmc_of_parse_voltage(int index, u32 *mask);

// --- registers --------------------------------------------------------------

#define MSC_CTRL		0x000
#define MSC_STAT		0x004
#define MSC_CLKRT		0x008
#define MSC_CMDAT		0x00c
#define MSC_RESTO		0x010
#define MSC_RDTO		0x014
#define MSC_BLKLEN		0x018
#define MSC_NOB			0x01c
#define MSC_SNOB		0x020
#define MSC_IMASK		0x024
#define MSC_IFLG		0x028
#define MSC_CMD			0x02c
#define MSC_ARG			0x030
#define MSC_RES			0x034
#define MSC_RXFIFO		0x038
#define MSC_TXFIFO		0x03c
#define MSC_LPM			0x040
#define MSC_DMAC		0x044
#define MSC_DMANDA		0x048
#define MSC_DMADA		0x04c
#define MSC_DMALEN		0x050
#define MSC_DMACMD		0x054
#define MSC_CTRL2		0x058
#define MSC_RTCNT		0x05c
#define MSC_DEBUG		0x0fc

#define CTRL_RESET		BIT(3)
#define CTRL_START_OP		BIT(2)
#define CTRL_CLOCK_START	(2 << 0)

#define STAT_AUTO_CMD12_DONE	BIT(31)
#define STAT_IS_RESETTING	BIT(15)
#define STAT_PRG_DONE		BIT(13)
#define STAT_DATA_TRAN_DONE	BIT(12)
#define STAT_END_CMD_RES	BIT(11)
#define STAT_DATA_FIFO_FULL	BIT(7)
#define STAT_DATA_FIFO_EMPTY	BIT(6)
#define STAT_CRC_READ_ERROR	BIT(4)
// CRC_RES_ERR, CRC_READ_ERROR, CRC_WRITE_ERROR, TIME_OUT_RES, TIME_OUT_READ
#define ERROR_STAT		0x3f

#define CMDAT_AUTO_CMD12	BIT(16)
#define CMDAT_BUS_WIDTH_MASK	(3 << 9)
#define CMDAT_BUS_WIDTH_1BIT	(0 << 9)
#define CMDAT_BUS_WIDTH_4BIT	(2 << 9)
#define CMDAT_BUS_WIDTH_8BIT	(3 << 9)
#define CMDAT_INIT		BIT(7)
#define CMDAT_BUSY		BIT(6)
#define CMDAT_WRITE_READ	BIT(4)
#define CMDAT_DATA_EN		BIT(3)
#define CMDAT_RESPONSE_MASK	(7 << 0)
#define CMDAT_RESPONSE_R1	(1 << 0)
#define CMDAT_RESPONSE_R2	(2 << 0)
#define CMDAT_RESPONSE_R3	(3 << 0)

// IMASK and IFLG share their bit positions.
#define IRQ_DMA_DATA_DONE	BIT(31)
#define IRQ_WR_ALL_DONE		BIT(23)
#define IRQ_DMAEND		BIT(16)
#define IRQ_AUTO_CMD12_DONE	BIT(15)
#define IRQ_CRC_RES_ERR		BIT(12)
#define IRQ_CRC_READ_ERR	BIT(11)
#define IRQ_CRC_WRITE_ERR	BIT(10)
#define IRQ_TIMEOUT_RES		BIT(9)
#define IRQ_TIMEOUT_READ	BIT(8)
#define IRQ_SDIO		BIT(7)
#define IRQ_END_CMD_RES		BIT(2)
#define IRQ_PRG_DONE		BIT(1)
#define IRQ_DATA_TRAN_DONE	BIT(0)
#define ERROR_IFLG		(IRQ_CRC_RES_ERR | IRQ_CRC_READ_ERR | IRQ_CRC_WRITE_ERR | \
				 IRQ_TIMEOUT_RES | IRQ_TIMEOUT_READ)

#define LPM_DRV_SEL_RISING	(2 << 30)	// sample at the rising clock edge
#define LPM_LPM			BIT(0)

#define DMAC_INCR_64		(2 << 2)
#define DMAC_ALIGNEN		BIT(4)
#define DMAC_AOFST_SHF		5
#define DMAC_DMAEN		BIT(0)

#define DMACMD_ENDI		BIT(1)
#define DMACMD_LINK		BIT(0)

// --- driver -----------------------------------------------------------------

#define TIMEOUT_PERIOD		3000	// ms between request timeout checks
#define PIO_THRESHOLD		64	// a single segment below this goes by PIO
#define CLK_RATE		24000000
#define MAX_SEGS		128
#define MSC_PINS		6	// clk, cmd, d0..d3

enum {
	DONTCARE = 0,
	NONREMOVABLE,
	REMOVABLE,
	MANUAL,
};

enum {
	EVENT_CMD_COMPLETE = 0,
	EVENT_TRANS_COMPLETE,
	EVENT_DMA_COMPLETE,
	EVENT_DATA_COMPLETE,
	EVENT_STOP_COMPLETE,
	EVENT_ERROR,
};

enum ingenic_mmc_state {
	STATE_IDLE = 0,
	STATE_WAITING_RESP,
	STATE_WAITING_DATA,
	STATE_SENDING_STOP,
	STATE_ERROR,
};

// Bits of ingenic_mmc_host.flags.
#define INGENIC_MMC_CARD_PRESENT	0
#define INGENIC_MMC_CARD_NEED_INIT	1
#define INGENIC_MMC_USE_PIO		2

#define LOW_ENABLE		0

struct sdma_desc {
	volatile u32 nda;
	volatile u32 da;
	volatile u32 len;
	volatile u32 dcmd;
};

struct desc_hd {
	struct sdma_desc *dma_desc;
	dma_addr_t dma_desc_phys_addr;
	struct desc_hd *next;
};

struct ingenic_mmc_pin {
	short num;
	short enable_level;
};

struct card_gpio {
	struct ingenic_mmc_pin wp;
	struct ingenic_mmc_pin cd;
	struct ingenic_mmc_pin pwr;
	struct ingenic_mmc_pin rst;
};

struct ingenic_mmc_pdata {
	unsigned short removal;		// DONTCARE, NONREMOVABLE, REMOVABLE, MANUAL
	unsigned short sdio_clk;
	unsigned int ocr_avail;
	unsigned int pm_flags;
	struct card_gpio *gpio;
	const char *pwr_regulator;
	unsigned int pio_mode;
};

struct ingenic_mmc_host {
	void __iomem *iomem;
	struct device *dev;
	struct clk *clk_cgu;
	struct clk *clk_gate;
	struct regulator *power;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_host *mmc;
	struct timer_list detect_timer;
	struct timer_list request_timer;
	struct list_head list;		// on manual_list, for MANUAL hosts
	struct ingenic_mmc_pdata *pdata;
	struct desc_hd decshds[MAX_SEGS];
	enum ingenic_mmc_state state;
	spinlock_t lock;		// IMASK read-modify-write
	unsigned long pending_events;
	unsigned long flags;
	unsigned int cmdat;
	unsigned int cmdat_def;
	unsigned int index;
	unsigned int double_enter;
	int timeout_cnt;
	int irq;
	int cd_irq;			// the card-detect interrupt, or -1
	bool irq_disabled;		// the controller interrupt, off without a card
	bool power_enabled;
	bool dma_mapped;		// the data scatterlist is mapped for DMA
	atomic_t clk_count;		// clock enables not yet disabled
	int bus_pins;			// bus pins requested at probe
	unsigned long desc_page;
	char labels[4][12];		// the cd, wp, rst and pwr pin names
};

// One controller's parameters and pins.
struct msc_pin {
	int gpio;
	int func;
	const char *name;
};

struct msc_config {
	int is_enable;
	int id;
	int voltage_min;
	int voltage_max;
	int bus_width;
	int max_frequency;
	char rm_method[24];
	char cd_method[24];
	char speed[24];
	bool cap_power_off_card;
	bool cap_mmc_hw_reset;
	bool cap_sdio_irq;
	bool full_pwr_cycle;
	bool keep_power_in_suspend;
	bool enable_sdio_wakeup;
	int dsr;
	bool pio_mode;
	bool enable_autocmd12;		// accepted, not used
	bool enable_cpm_rx_tuning;	// accepted, not used
	bool enable_cpm_tx_tuning;	// accepted, not used
	bool sdio_clk;
	int rst, rst_enable_level;
	int wp, wp_enable_level;
	int pwr, pwr_enable_level;
	int cd, cd_enable_level;
	int sdr, sdr_enable_level;
	const struct msc_pin *pins;
	char pwr_regulator[24];
};

static const struct msc_pin jz_msc0_pin[MSC_PINS] = {
	{ GPIO_PB(12), GPIO_FUNC_0, "msc0_clk" },
	{ GPIO_PB(13), GPIO_FUNC_0, "msc0_cmd" },
	{ GPIO_PB(14), GPIO_FUNC_0, "msc0_d0" },
	{ GPIO_PB(15), GPIO_FUNC_0, "msc0_d1" },
	{ GPIO_PB(16), GPIO_FUNC_0, "msc0_d2" },
	{ GPIO_PB(17), GPIO_FUNC_0, "msc0_d3" },
};

static const struct msc_pin jz_msc1_pin[MSC_PINS] = {
	{ GPIO_PD(0), GPIO_FUNC_0, "msc1_clk" },
	{ GPIO_PD(1), GPIO_FUNC_0, "msc1_cmd" },
	{ GPIO_PD(2), GPIO_FUNC_0, "msc1_d0" },
	{ GPIO_PD(3), GPIO_FUNC_0, "msc1_d1" },
	{ GPIO_PD(4), GPIO_FUNC_0, "msc1_d2" },
	{ GPIO_PD(5), GPIO_FUNC_0, "msc1_d3" },
};

static struct msc_config jzmsc_gpio[2] = {
	{ .id = 0, .pins = jz_msc0_pin },
	{ .id = 1, .pins = jz_msc1_pin },
};

static int wifi_power_on = -1;
static int wifi_power_on_level = -1;
static int wifi_reg_on = -1;
static int wifi_reg_on_level = -1;

module_param_cb(wifi_power_on, &param_gpio_ops, &wifi_power_on, 0644);
module_param(wifi_power_on_level, int, 0644);
module_param_cb(wifi_reg_on, &param_gpio_ops, &wifi_reg_on, 0644);
module_param(wifi_reg_on_level, int, 0644);

#define MSC_PARAMS(n)									\
	module_param_named(msc##n##_is_enable, jzmsc_gpio[n].is_enable, int, 0644);	\
	module_param_named(msc##n##_voltage_min, jzmsc_gpio[n].voltage_min, int, 0644);	\
	module_param_named(msc##n##_voltage_max, jzmsc_gpio[n].voltage_max, int, 0644);	\
	module_param_named(msc##n##_bus_width, jzmsc_gpio[n].bus_width, int, 0644);	\
	module_param_string(msc##n##_rm_method, jzmsc_gpio[n].rm_method, 24, 0644);	\
	module_param_string(msc##n##_cd_method, jzmsc_gpio[n].cd_method, 24, 0644);	\
	module_param_string(msc##n##_speed, jzmsc_gpio[n].speed, 24, 0644);		\
	module_param_named(msc##n##_max_frequency, jzmsc_gpio[n].max_frequency, int, 0644); \
	module_param_named(msc##n##_cap_power_off_card, jzmsc_gpio[n].cap_power_off_card, bool, 0644); \
	module_param_named(msc##n##_cap_mmc_hw_reset, jzmsc_gpio[n].cap_mmc_hw_reset, bool, 0644); \
	module_param_named(msc##n##_cap_sdio_irq, jzmsc_gpio[n].cap_sdio_irq, bool, 0644); \
	module_param_named(msc##n##_full_pwr_cycle, jzmsc_gpio[n].full_pwr_cycle, bool, 0644); \
	module_param_named(msc##n##_keep_power_in_suspend, jzmsc_gpio[n].keep_power_in_suspend, bool, 0644); \
	module_param_named(msc##n##_enable_sdio_wakeup, jzmsc_gpio[n].enable_sdio_wakeup, bool, 0644); \
	module_param_named(msc##n##_dsr, jzmsc_gpio[n].dsr, int, 0644);		\
	module_param_named(msc##n##_pio_mode, jzmsc_gpio[n].pio_mode, bool, 0644);	\
	module_param_named(msc##n##_enable_autocmd12, jzmsc_gpio[n].enable_autocmd12, bool, 0644); \
	module_param_named(msc##n##_enable_cpm_rx_tuning, jzmsc_gpio[n].enable_cpm_rx_tuning, bool, 0644); \
	module_param_named(msc##n##_enable_cpm_tx_tuning, jzmsc_gpio[n].enable_cpm_tx_tuning, bool, 0644); \
	module_param_named(msc##n##_sdio_clk, jzmsc_gpio[n].sdio_clk, bool, 0644);	\
	module_param_cb(msc##n##_rst, &param_gpio_ops, &jzmsc_gpio[n].rst, 0644);	\
	module_param_cb(msc##n##_wp, &param_gpio_ops, &jzmsc_gpio[n].wp, 0644);		\
	module_param_cb(msc##n##_pwr, &param_gpio_ops, &jzmsc_gpio[n].pwr, 0644);	\
	module_param_string(msc##n##_pwr_regulator, jzmsc_gpio[n].pwr_regulator, 24, 0644); \
	module_param_cb(msc##n##_cd, &param_gpio_ops, &jzmsc_gpio[n].cd, 0644);		\
	module_param_cb(msc##n##_sdr, &param_gpio_ops, &jzmsc_gpio[n].sdr, 0644);	\
	module_param_named(msc##n##_rst_enable_level, jzmsc_gpio[n].rst_enable_level, int, 0644); \
	module_param_named(msc##n##_wp_enable_level, jzmsc_gpio[n].wp_enable_level, int, 0644); \
	module_param_named(msc##n##_pwr_enable_level, jzmsc_gpio[n].pwr_enable_level, int, 0644); \
	module_param_named(msc##n##_cd_enable_level, jzmsc_gpio[n].cd_enable_level, int, 0644); \
	module_param_named(msc##n##_sdr_enable_level, jzmsc_gpio[n].sdr_enable_level, int, 0644)

MSC_PARAMS(0);
MSC_PARAMS(1);

static LIST_HEAD(manual_list);

#define msc_readl(host, reg)		__raw_readl((host)->iomem + MSC_##reg)
#define msc_writel(host, reg, value)	__raw_writel((value), (host)->iomem + MSC_##reg)

#define ingenic_mmc_check_pending(host, event)	test_and_clear_bit(event, &(host)->pending_events)
#define ingenic_mmc_set_pending(host, event)	set_bit(event, &(host)->pending_events)
#define is_pio_mode(host)		((host)->flags & (1 << INGENIC_MMC_USE_PIO))
#define enable_pio_mode(host)		((host)->flags |= (1 << INGENIC_MMC_USE_PIO))
#define disable_pio_mode(host)		((host)->flags &= ~(1 << INGENIC_MMC_USE_PIO))

static void ingenic_mmc_dump_reg(struct ingenic_mmc_host *host)
{
	dev_info(host->dev, "\nREG dump:\n"
		 "\tCTRL2\t= 0x%08X\n"
		 "\tSTAT\t= 0x%08X\n"
		 "\tCLKRT\t= 0x%08X\n"
		 "\tCMDAT\t= 0x%08X\n"
		 "\tRESTO\t= 0x%08X\n"
		 "\tRDTO\t= 0x%08X\n"
		 "\tBLKLEN\t= 0x%08X\n"
		 "\tNOB\t= 0x%08X\n"
		 "\tSNOB\t= 0x%08X\n"
		 "\tIMASK\t= 0x%08X\n"
		 "\tIFLG\t= 0x%08X\n"
		 "\tCMD\t= 0x%08X\n"
		 "\tARG\t= 0x%08X\n"
		 "\tRES\t= 0x%08X\n"
		 "\tLPM\t= 0x%08X\n"
		 "\tDMAC\t= 0x%08X\n"
		 "\tDMANDA\t= 0x%08X\n"
		 "\tDMADA\t= 0x%08X\n"
		 "\tDMALEN\t= 0x%08X\n"
		 "\tDMACMD\t= 0x%08X\n"
		 "\tRTCNT\t= 0x%08X\n"
		 "\tDEBUG\t= 0x%08X\n",
		 msc_readl(host, CTRL2), msc_readl(host, STAT), msc_readl(host, CLKRT),
		 msc_readl(host, CMDAT), msc_readl(host, RESTO), msc_readl(host, RDTO),
		 msc_readl(host, BLKLEN), msc_readl(host, NOB), msc_readl(host, SNOB),
		 msc_readl(host, IMASK), msc_readl(host, IFLG), msc_readl(host, CMD),
		 msc_readl(host, ARG), msc_readl(host, RES), msc_readl(host, LPM),
		 msc_readl(host, DMAC), msc_readl(host, DMANDA), msc_readl(host, DMADA),
		 msc_readl(host, DMALEN), msc_readl(host, DMACMD), msc_readl(host, RTCNT),
		 msc_readl(host, DEBUG));
}

// --- small helpers ----------------------------------------------------------

// Unmasks interrupt sources (a set IMASK bit masks).
static inline void enable_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	unsigned long imsk;

	spin_lock_bh(&host->lock);
	imsk = msc_readl(host, IMASK);
	imsk &= ~bits;
	msc_writel(host, IMASK, imsk);
	spin_unlock_bh(&host->lock);
}

static inline void clear_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	msc_writel(host, IFLG, bits);
}

static inline void disable_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	unsigned long imsk;

	spin_lock_bh(&host->lock);
	imsk = msc_readl(host, IMASK);
	imsk |= bits;
	msc_writel(host, IMASK, imsk);
	spin_unlock_bh(&host->lock);
}

// Resets the controller, keeping the clock divider, with every interrupt
// masked and cleared.
static inline void ingenic_mmc_reset(struct ingenic_mmc_host *host)
{
	unsigned int clkrt = msc_readl(host, CLKRT);
	unsigned int cnt = 100 * 1000 * 1000;

	msc_writel(host, CTRL, CTRL_RESET);
	msc_writel(host, CTRL, msc_readl(host, CTRL) & ~CTRL_RESET);

	while ((msc_readl(host, STAT) & STAT_IS_RESETTING) && --cnt)
		;
	WARN_ON(!cnt);

	if (host->pdata->sdio_clk)
		msc_writel(host, CTRL, CTRL_CLOCK_START);
	else
		msc_writel(host, LPM, LPM_LPM);

	msc_writel(host, IMASK, 0xffffffff);
	msc_writel(host, IFLG, 0xffffffff);
	msc_writel(host, CLKRT, clkrt);
}

static inline void ingenic_mmc_stop_dma(struct ingenic_mmc_host *host)
{
	dev_warn(host->dev, "%s\n", __func__);

	// The DMA cannot be stopped in the middle of a transfer: this only
	// disables it once it is out of DMA requests.
	msc_writel(host, DMAC, 0);
}

static inline int request_need_stop(struct mmc_request *mrq)
{
	return mrq->stop ? 1 : 0;
}

// The controller clocks, and the bus clock pin with them: back to the
// controller when they go on, a GPIO input when they go off. Each call
// enables or disables once; clk_count keeps the balance for remove.
static void ingenic_mmc_clk_onoff(struct ingenic_mmc_host *host, unsigned int on)
{
	const struct msc_pin *clk_pin = &jzmsc_gpio[host->index].pins[0];

	if (on) {
		gpio_set_func(clk_pin->gpio, clk_pin->func);
		clk_prepare_enable(host->clk_cgu);
		clk_prepare_enable(host->clk_gate);
		atomic_inc(&host->clk_count);
	} else {
		clk_disable_unprepare(host->clk_cgu);
		clk_disable_unprepare(host->clk_gate);
		gpio_set_func(clk_pin->gpio, GPIO_INPUT);
		atomic_dec(&host->clk_count);
	}
}

static inline int check_error_status(struct ingenic_mmc_host *host, unsigned int status)
{
	if (status & ERROR_STAT) {
		dev_err(host->dev, "Error status->0x%08X: cmd=%d, state=%d\n",
			status, host->cmd->opcode, host->state);
		return -1;
	}
	return 0;
}

static int ingenic_mmc_polling_status(struct ingenic_mmc_host *host, unsigned int status)
{
	unsigned int cnt = 100 * 1000 * 1000;

	while (!(msc_readl(host, STAT) & (status | ERROR_STAT)) &&
	       test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags) && --cnt)
		;

	if (unlikely(!cnt)) {
		dev_err(host->dev, "polling status(0x%08X) time out, op=%d, status=0x%08X\n",
			status, host->cmd->opcode, msc_readl(host, STAT));
		return -1;
	}
	if (unlikely(!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags))) {
		dev_err(host->dev, "card remove while polling" "status(0x%08X), op=%d\n",
			status, host->cmd->opcode);
		return -1;
	}
	if (msc_readl(host, STAT) & ERROR_STAT) {
		dev_err(host->dev, "polling status(0x%08X) error, op=%d, status=0x%08X\n",
			status, host->cmd->opcode, msc_readl(host, STAT));
		return -1;
	}
	return 0;
}

static void send_stop_command(struct ingenic_mmc_host *host)
{
	struct mmc_command *stop_cmd = host->mrq->stop;

	msc_writel(host, CMD, stop_cmd->opcode);
	msc_writel(host, ARG, stop_cmd->arg);
	msc_writel(host, CMDAT, CMDAT_BUSY | CMDAT_RESPONSE_R1);
	msc_writel(host, RESTO, 0xff);
	msc_writel(host, CTRL, CTRL_START_OP);

	if (ingenic_mmc_polling_status(host, STAT_END_CMD_RES))
		stop_cmd->error = -EIO;
}

// The response comes out of the 16-bit RES FIFO.
static void ingenic_mmc_command_done(struct ingenic_mmc_host *host, struct mmc_command *cmd)
{
	unsigned long res;

	if ((host->cmdat & CMDAT_RESPONSE_MASK) == CMDAT_RESPONSE_R2) {
		int i;

		res = msc_readl(host, RES);
		for (i = 0; i < 4; i++) {
			cmd->resp[i] = res << 24;
			res = msc_readl(host, RES);
			cmd->resp[i] |= res << 8;
			res = msc_readl(host, RES);
			cmd->resp[i] |= res >> 8;
		}
	} else {
		res = msc_readl(host, RES);
		cmd->resp[0] = res << 24;
		res = msc_readl(host, RES);
		cmd->resp[0] |= res << 8;
		res = msc_readl(host, RES);
		cmd->resp[0] |= res & 0xff;
	}

	clear_msc_irq(host, IRQ_END_CMD_RES);
}

static enum dma_data_direction data_dir(struct mmc_data *data)
{
	return data->flags & MMC_DATA_WRITE ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
}

// Gives the scatterlist of a DMA request back to the CPU. A request that
// ended in an error leaves its mapping marked: the next request with data
// maps its own.
static void ingenic_mmc_unmap(struct ingenic_mmc_host *host)
{
	if (!is_pio_mode(host) && host->dma_mapped && host->data) {
		dma_unmap_sg(host->dev, host->data->sg, host->data->sg_len, data_dir(host->data));
		host->dma_mapped = 0;
	}
}

static void ingenic_mmc_data_done(struct ingenic_mmc_host *host)
{
	struct mmc_data *data = host->data;

	if (data->error == 0) {
		data->bytes_xfered = data->blocks * data->blksz;
	} else {
		ingenic_mmc_stop_dma(host);
		data->bytes_xfered = 0;
		dev_err(host->dev, "error when request done\n");
	}

	ingenic_mmc_unmap(host);
	del_timer_sync(&host->request_timer);
	mmc_request_done(host->mmc, host->mrq);
}

// --- state machine ----------------------------------------------------------

// Moves a request on, from the interrupt thread.
static void ingenic_mmc_state_machine(struct ingenic_mmc_host *host, unsigned int status)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *data = host->data;

	WARN_ON(host->double_enter++);
start:
	switch (host->state) {
	case STATE_IDLE:
		dev_warn(host->dev, "WARN: enter state machine with IDLE\n");
		break;

	case STATE_WAITING_RESP:
		if (!ingenic_mmc_check_pending(host, EVENT_CMD_COMPLETE))
			break;
		if (unlikely(check_error_status(host, status) != 0)) {
			host->state = STATE_ERROR;
			clear_msc_irq(host, IRQ_CRC_RES_ERR | IRQ_TIMEOUT_RES | IRQ_END_CMD_RES);
			goto start;
		}
		ingenic_mmc_command_done(host, mrq->cmd);
		if (!data) {
			host->state = STATE_IDLE;
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
			break;
		}
		host->state = STATE_WAITING_DATA;
		break;

	case STATE_WAITING_DATA:
		if (!ingenic_mmc_check_pending(host, EVENT_DATA_COMPLETE))
			break;
		if (unlikely(check_error_status(host, status) != 0)) {
			clear_msc_irq(host, IRQ_DATA_TRAN_DONE | IRQ_CRC_READ_ERR |
				      IRQ_CRC_WRITE_ERR | IRQ_TIMEOUT_READ);
			if (request_need_stop(host->mrq))
				send_stop_command(host);
			host->state = STATE_ERROR;
			goto start;
		}

		if (request_need_stop(host->mrq)) {
			if (likely(msc_readl(host, STAT) & STAT_AUTO_CMD12_DONE)) {
				disable_msc_irq(host, IRQ_AUTO_CMD12_DONE);
				clear_msc_irq(host, IRQ_AUTO_CMD12_DONE);
				host->state = STATE_IDLE;
				ingenic_mmc_data_done(host);
			} else {
				enable_msc_irq(host, IRQ_AUTO_CMD12_DONE);
				if (msc_readl(host, STAT) & STAT_AUTO_CMD12_DONE) {
					disable_msc_irq(host, IRQ_AUTO_CMD12_DONE);
					clear_msc_irq(host, IRQ_AUTO_CMD12_DONE);
					host->state = STATE_IDLE;
					ingenic_mmc_data_done(host);
				} else {
					host->state = STATE_SENDING_STOP;
				}
			}
		} else {
			host->state = STATE_IDLE;
			ingenic_mmc_data_done(host);
		}
		break;

	case STATE_SENDING_STOP:
		if (!ingenic_mmc_check_pending(host, EVENT_STOP_COMPLETE))
			break;
		host->state = STATE_IDLE;
		ingenic_mmc_data_done(host);
		break;

	case STATE_ERROR:
		host->cmd->error = -1;
		if (data)
			data->bytes_xfered = 0;
		del_timer_sync(&host->request_timer);
		host->state = STATE_IDLE;
		mmc_request_done(host->mmc, host->mrq);
		break;
	}

	host->double_enter--;
}

static irqreturn_t ingenic_mmc_thread_handle(int irq, void *dev_id)
{
	struct ingenic_mmc_host *host = dev_id;
	unsigned int iflg, imask, pending, status;

start:
	iflg = msc_readl(host, IFLG);
	imask = msc_readl(host, IMASK);
	pending = iflg & ~imask;
	status = msc_readl(host, STAT);

	if (!pending) {
		goto out;
	} else if (pending & IRQ_SDIO) {
		mmc_signal_sdio_irq(host->mmc);
		goto out;
	} else if (pending & ERROR_IFLG) {
		unsigned int mask = ERROR_IFLG;

		if (host->state == STATE_WAITING_RESP)
			mask |= IRQ_END_CMD_RES;
		else if (host->state == STATE_WAITING_DATA)
			mask |= IRQ_WR_ALL_DONE | IRQ_DMA_DATA_DONE;

		clear_msc_irq(host, mask);
		disable_msc_irq(host, mask);

		// A CMD53 CRC error turns up now and then at 50 MHz: the
		// request is handed back for one retry.
		if (host->cmd->opcode == 53 && (status & STAT_CRC_READ_ERROR)) {
			dev_err(host->dev, "cmd53 crc error, retry.\n");
			host->cmd->error = -1;
			host->cmd->retries = 1;
			host->data->bytes_xfered = 0;
			del_timer_sync(&host->request_timer);
			host->state = STATE_IDLE;
			mmc_request_done(host->mmc, host->mrq);
			goto out;
		}
		host->state = STATE_ERROR;
		ingenic_mmc_state_machine(host, status);
		goto out;
	} else if (pending & IRQ_END_CMD_RES) {
		ingenic_mmc_set_pending(host, EVENT_CMD_COMPLETE);
		disable_msc_irq(host, IRQ_END_CMD_RES | IRQ_CRC_RES_ERR | IRQ_TIMEOUT_RES);
		ingenic_mmc_state_machine(host, status);
	} else if (pending & IRQ_WR_ALL_DONE) {
		ingenic_mmc_set_pending(host, EVENT_DATA_COMPLETE);
		clear_msc_irq(host, IRQ_WR_ALL_DONE | IRQ_DMAEND | IRQ_DATA_TRAN_DONE | IRQ_PRG_DONE);
		disable_msc_irq(host, IRQ_WR_ALL_DONE | IRQ_CRC_WRITE_ERR);
		ingenic_mmc_state_machine(host, status);
	} else if (pending & IRQ_DMA_DATA_DONE) {
		ingenic_mmc_set_pending(host, EVENT_DATA_COMPLETE);
		clear_msc_irq(host, IRQ_DATA_TRAN_DONE | IRQ_DMAEND | IRQ_DMA_DATA_DONE);
		disable_msc_irq(host, IRQ_DMA_DATA_DONE | IRQ_CRC_READ_ERR);
		ingenic_mmc_state_machine(host, status);
	} else if (pending & IRQ_AUTO_CMD12_DONE) {
		ingenic_mmc_set_pending(host, EVENT_STOP_COMPLETE);
		clear_msc_irq(host, IRQ_AUTO_CMD12_DONE);
		disable_msc_irq(host, IRQ_AUTO_CMD12_DONE);
		ingenic_mmc_state_machine(host, status);
	} else {
		dev_warn(host->dev, "state-%d: Nothing happens?!\n", host->state);
	}

	// The status moved on meanwhile: handled now rather than by one more
	// interrupt.
	if (status != msc_readl(host, STAT))
		goto start;
out:
	return IRQ_HANDLED;
}

// --- DMA --------------------------------------------------------------------

static inline void sg_to_desc(struct scatterlist *sgentry, struct desc_hd *dhd)
{
	dhd->dma_desc->da = sg_phys(sgentry);
	dhd->dma_desc->len = sg_dma_len(sgentry);
	dhd->dma_desc->dcmd = DMACMD_LINK;
}

// Maps the scatterlist and chains one descriptor per segment. It stays
// mapped until the request ends.
static void ingenic_mmc_submit_dma(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	struct scatterlist *sgentry;
	struct desc_hd *dhd = &host->decshds[0];
	unsigned int i;

	dma_map_sg(host->dev, data->sg, data->sg_len, data_dir(data));

	for_each_sg(data->sg, sgentry, data->sg_len, i) {
		sg_to_desc(sgentry, dhd);
		if (data->sg_len - i > 1) {
			if (unlikely(!dhd->next)) {
				dev_err(host->dev, "dhd->next == NULL\n");
			} else {
				dhd->dma_desc->nda = dhd->next->dma_desc_phys_addr;
				dhd = dhd->next;
			}
		}
	}
	host->dma_mapped = 1;

	dhd->dma_desc->dcmd |= DMACMD_ENDI;
	dhd->dma_desc->dcmd &= ~DMACMD_LINK;
}

static inline void ingenic_mmc_dma_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	dma_addr_t dma_addr = sg_phys(data->sg);
	unsigned int dma_len = sg_dma_len(data->sg);
	unsigned int dmac;

	BUG_ON(!dma_len);
	dmac = DMAC_INCR_64 | DMAC_DMAEN;

	if ((dma_addr & 0x3) || (dma_len & 0x3)) {
		dmac |= DMAC_ALIGNEN;
		if (dma_addr & 0x3)
			dmac |= (dma_addr % 4) << DMAC_AOFST_SHF;
	}
	msc_writel(host, DMANDA, host->decshds[0].dma_desc_phys_addr);
	msc_writel(host, DMAC, dmac);
}

// --- PIO --------------------------------------------------------------------

static int wait_cmd_response(struct ingenic_mmc_host *host)
{
	if (ingenic_mmc_polling_status(host, STAT_END_CMD_RES) < 0) {
		dev_err(host->dev, "PIO mode: command response error\n");
		return -1;
	}
	msc_writel(host, IFLG, IRQ_END_CMD_RES);
	return 0;
}

static void do_pio_read(struct ingenic_mmc_host *host, unsigned int *addr, unsigned int cnt)
{
	unsigned int status = 0;
	unsigned int i;

	for (i = 0; i < cnt / 4; i++) {
		while (((status = msc_readl(host, STAT)) & STAT_DATA_FIFO_EMPTY) &&
		       test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags))
			;

		if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
			host->data->error = -ENOMEDIUM;
			dev_err(host->dev, "PIO mode: card remove while reading\n");
			return;
		}
		if (check_error_status(host, status)) {
			host->data->error = -1;
			return;
		}
		*addr++ = msc_readl(host, RXFIFO);
	}

	// The last 1, 2 or 3 bytes.
	if (cnt & 3) {
		u32 n = cnt & 3;
		u32 data = msc_readl(host, RXFIFO);
		u8 *p = (u8 *)addr;

		while (n--) {
			*p++ = data;
			data >>= 8;
		}
	}
}

static void do_pio_write(struct ingenic_mmc_host *host, unsigned int *addr, unsigned int cnt)
{
	unsigned int status = 0;
	unsigned int i;

	for (i = 0; i < cnt / 4; i++) {
		while (((status = msc_readl(host, STAT)) & STAT_DATA_FIFO_FULL) &&
		       test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags))
			;

		if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
			host->data->error = -ENOMEDIUM;
			dev_err(host->dev, "PIO mode: card remove while writing\n");
			break;
		}
		if (check_error_status(host, status)) {
			host->data->error = -1;
			return;
		}
		msc_writel(host, TXFIFO, *addr++);
	}

	// The last 1, 2 or 3 bytes.
	if (cnt & 3) {
		u32 data = 0;
		u8 *p = (u8 *)addr;

		for (i = 0; i < (cnt & 3); i++)
			data |= *p++ << (8 * i);

		msc_writel(host, TXFIFO, data);
	}
}

static inline void pio_trans_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	unsigned int *addr = sg_virt(data->sg);
	unsigned int cnt = sg_dma_len(data->sg);

	if (data->flags & MMC_DATA_WRITE)
		do_pio_write(host, addr, cnt);
	else
		do_pio_read(host, addr, cnt);
}

static void pio_trans_done(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	if (host->mrq->stop) {
		if (ingenic_mmc_polling_status(host, STAT_AUTO_CMD12_DONE) < 0)
			data->error = -EIO;
	}

	if (data->flags & MMC_DATA_WRITE) {
		if (ingenic_mmc_polling_status(host, STAT_PRG_DONE) < 0)
			data->error = -EIO;
		clear_msc_irq(host, IRQ_PRG_DONE);
	} else {
		if (ingenic_mmc_polling_status(host, STAT_DATA_TRAN_DONE) < 0)
			data->error = -EIO;
		clear_msc_irq(host, IRQ_DATA_TRAN_DONE);
	}
}

// --- requests ---------------------------------------------------------------

static void ingenic_mmc_data_pre(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	unsigned long cmdat, imsk;

	msc_writel(host, RDTO, 0xffffff);
	msc_writel(host, NOB, data->blocks);
	msc_writel(host, BLKLEN, data->blksz);
	cmdat = CMDAT_DATA_EN;
	msc_writel(host, CMDAT, CMDAT_DATA_EN);

	if (data->flags & MMC_DATA_WRITE) {
		cmdat |= CMDAT_WRITE_READ;
		imsk = IRQ_WR_ALL_DONE | IRQ_CRC_WRITE_ERR;
	} else if (data->flags & MMC_DATA_READ) {
		cmdat &= ~CMDAT_WRITE_READ;
		imsk = IRQ_DMA_DATA_DONE | IRQ_TIMEOUT_READ | IRQ_CRC_READ_ERR;
	} else {
		dev_err(host->dev, "data direction confused\n");
		BUG();
	}
	host->cmdat |= cmdat;

	if (!is_pio_mode(host)) {
		ingenic_mmc_submit_dma(host, data);
		clear_msc_irq(host, IRQ_PRG_DONE);
		enable_msc_irq(host, imsk);
	}
}

static void ingenic_mmc_data_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	if (is_pio_mode(host)) {
		pio_trans_start(host, data);
		pio_trans_done(host, data);
		del_timer_sync(&host->request_timer);
		if (!host->pdata->pio_mode)
			disable_pio_mode(host);
		mmc_request_done(host->mmc, host->mrq);
	} else {
		ingenic_mmc_dma_start(host, data);
	}
}

static void ingenic_mmc_command_start(struct ingenic_mmc_host *host, struct mmc_command *cmd)
{
	unsigned long cmdat = 0;

	if (cmd->flags & MMC_RSP_BUSY)
		cmdat |= CMDAT_BUSY;
	if (request_need_stop(host->mrq))
		cmdat |= CMDAT_AUTO_CMD12;

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_R1:	// also R5, R6, R7
	case MMC_RSP_R1B:
		cmdat |= CMDAT_RESPONSE_R1;
		break;
	case MMC_RSP_R2:
		cmdat |= CMDAT_RESPONSE_R2;
		break;
	case MMC_RSP_R3:	// also R4
		cmdat |= CMDAT_RESPONSE_R3;
		break;
	default:
		break;
	}
	host->cmdat |= cmdat;

	if (!is_pio_mode(host)) {
		enable_msc_irq(host, IRQ_TIMEOUT_RES | IRQ_END_CMD_RES);
		host->state = STATE_WAITING_RESP;
	}

	msc_writel(host, CMD, cmd->opcode);
	msc_writel(host, ARG, cmd->arg);
	msc_writel(host, CMDAT, host->cmdat);
	msc_writel(host, CTRL, CTRL_START_OP);

	if (is_pio_mode(host)) {
		if (wait_cmd_response(host) < 0) {
			cmd->error = -ETIMEDOUT;
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
			return;
		}
		ingenic_mmc_command_done(host, host->cmd);
		if (!host->data) {
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
		}
	}
}

static void ingenic_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	if (host->state != STATE_IDLE && mrq->data) {
		dev_warn(host->dev, "operate in non-idle state\n");
		WARN_ON(1);
	}

	host->mrq = mrq;
	host->data = mrq->data;
	host->cmd = mrq->cmd;

	host->cmdat = host->cmdat_def;
	if (host->data) {
		if (host->data->sg_len == 1 && sg_dma_len(host->data->sg) < PIO_THRESHOLD)
			enable_pio_mode(host);
		ingenic_mmc_data_pre(host, host->data);
	}

	// The request ends in mmc_request_done() unless something such as the
	// supply dropping stops the card answering: the timer catches that.
	host->timeout_cnt = 0;
	mod_timer(&host->request_timer, jiffies + msecs_to_jiffies(TIMEOUT_PERIOD));
	ingenic_mmc_command_start(host, host->cmd);
	if (host->data)
		ingenic_mmc_data_start(host, host->data);

	if (unlikely(test_and_clear_bit(INGENIC_MMC_CARD_NEED_INIT, &host->flags)))
		host->cmdat_def &= ~CMDAT_INIT;
}

static void ingenic_mmc_request_timeout(unsigned long arg)
{
	struct ingenic_mmc_host *host = (struct ingenic_mmc_host *)arg;
	unsigned int status = msc_readl(host, STAT);
	int state = host->state;

	if (host->timeout_cnt++ < 3000 / TIMEOUT_PERIOD) {
		dev_warn(host->dev, "timeout %dms op:%d %s sz:%d state:%d STAT:0x%08X DMALEN:0x%08X blks:%d/%d clk:%s\n",
			 host->timeout_cnt * TIMEOUT_PERIOD, host->cmd->opcode,
			 host->data ? (host->data->flags & MMC_DATA_WRITE ? "w" : "r") : "",
			 host->data ? host->data->blocks << 9 : 0, state, status,
			 msc_readl(host, DMALEN), msc_readl(host, SNOB), msc_readl(host, NOB),
			 __clk_is_enabled(host->clk_cgu) ? "enable" : "disable");
		mod_timer(&host->request_timer, jiffies + msecs_to_jiffies(TIMEOUT_PERIOD));
		return;
	}

	dev_err(host->dev, "request time out, op=%d arg=0x%08X, sz:%dB state=%d, status=0x%08X, pending=0x%08X, nr_desc=%d\n",
		host->cmd->opcode, host->cmd->arg, host->data ? (int)(host->data->blocks << 9) : -1,
		state, status, (u32)host->pending_events, host->data ? host->data->sg_len : 0);
	ingenic_mmc_dump_reg(host);

	if (host->data && host->decshds[0].dma_desc) {
		int i;

		dev_err(host->dev, "Descriptor dump:\n");
		for (i = 0; i < MAX_SEGS; i++) {
			struct sdma_desc *desc = host->decshds[i].dma_desc;

			dev_err(host->dev, "\t%03d\t nda=%08X da=%08X len=%08X dcmd=%08X\n",
				i, desc->nda, desc->da, desc->len, desc->dcmd);
		}
		dev_err(host->dev, "\n");
	}

	if (host->mrq) {
		if (request_need_stop(host->mrq))
			send_stop_command(host);
		ingenic_mmc_unmap(host);
		host->cmd->error = -ENOMEDIUM;
		host->state = STATE_IDLE;
		mmc_request_done(host->mmc, host->mrq);
	}
}

// --- card detection ---------------------------------------------------------

static irqreturn_t ingenic_mmc_detect_handler(int irq, void *dev_id)
{
	struct ingenic_mmc_host *host = dev_id;

	disable_irq_nosync(irq);
	mod_timer(&host->detect_timer, jiffies + msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

// 1 when the pin is at its enable level, 0 when not, -1 without a pin.
static int get_pin_status(struct ingenic_mmc_pin *pin)
{
	int val;

	if (!gpio_is_valid(pin->num))
		return -1;
	val = gpio_get_value(pin->num);
	if (pin->enable_level == LOW_ENABLE)
		return !val;
	return val;
}

static void set_pin_status(struct ingenic_mmc_pin *pin, int enable)
{
	if (!gpio_is_valid(pin->num))
		return;
	if (pin->enable_level == LOW_ENABLE)
		enable = !enable;
	gpio_set_value(pin->num, enable);
}

// The card-detect pin settled: a card that came or went (or came and went
// again meanwhile) is reported to the core.
static void ingenic_mmc_detect(unsigned long arg)
{
	struct ingenic_mmc_host *host = (struct ingenic_mmc_host *)arg;
	bool present, present_old;

	present = get_pin_status(&host->pdata->gpio->cd);
	present_old = test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);

	if (present != present_old || (present_old && host->mmc->card)) {
		if (present && present_old)
			dev_warn(host->dev, "rapidly remove\n");
		else
			dev_notice(host->dev, "card %s, state=%d\n",
				   present ? "inserted" : "removed", host->state);

		if (!present || present_old) {
			clear_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
			if (!host->irq_disabled) {
				disable_irq_nosync(host->irq);
				host->irq_disabled = true;
			}
			ingenic_mmc_reset(host);

			if (host->mrq && host->state > STATE_IDLE) {
				host->cmd->error = -ENOMEDIUM;
				if (host->data) {
					host->data->bytes_xfered = 0;
					ingenic_mmc_stop_dma(host);
				}
				del_timer_sync(&host->request_timer);
				mmc_request_done(host->mmc, host->mrq);
				host->state = STATE_IDLE;
			}
			mmc_detect_change(host->mmc, 0);
		} else {
			if (host->irq_disabled) {
				enable_irq(host->irq);
				host->irq_disabled = false;
			}
			set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
			ingenic_mmc_clk_onoff(host, 1);
			mmc_detect_change(host->mmc, msecs_to_jiffies(1000));
		}

		if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags))
			ingenic_mmc_clk_onoff(host, 0);
	}

	enable_irq(gpio_to_irq(host->pdata->gpio->cd.num));
}

static struct ingenic_mmc_host *manual_host(int index)
{
	struct ingenic_mmc_host *host;

	list_for_each_entry(host, &manual_list, list)
		if ((int)host->index == index)
			return host;
	return NULL;
}

// Inserts (on) or removes the card of a MANUAL host by hand: the Wi-Fi
// power driver calls it once the chip is powered, or before it goes off.
int jzmmc_manual_detect(int index, int on)
{
	struct ingenic_mmc_host *host = manual_host(index);

	if (!host) {
		pr_err("no manual card detect\n");
		return -1;
	}

	if (on) {
		set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		ingenic_mmc_clk_onoff(host, 1);
		mmc_detect_change(host->mmc, 0);
	} else {
		clear_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		mmc_detect_change(host->mmc, 0);
		ingenic_mmc_clk_onoff(host, 0);
	}
	return 0;
}
EXPORT_SYMBOL(jzmmc_manual_detect);

// The clocks of a MANUAL host, on or off.
int jzmmc_clk_ctrl(int index, int on)
{
	struct ingenic_mmc_host *host = manual_host(index);

	if (!host) {
		pr_err("no manual card detect\n");
		return -1;
	}
	ingenic_mmc_clk_onoff(host, on);
	return 0;
}
EXPORT_SYMBOL(jzmmc_clk_ctrl);

// --- host operations --------------------------------------------------------

static inline void ingenic_mmc_power_on(struct ingenic_mmc_host *host)
{
	if (host->power && !IS_ERR(host->power)) {
		if (!host->power_enabled && !regulator_enable(host->power))
			host->power_enabled = true;
	} else if (host->pdata->gpio) {
		set_pin_status(&host->pdata->gpio->pwr, 1);
	}
}

static inline void ingenic_mmc_power_off(struct ingenic_mmc_host *host)
{
	if (host->power && !IS_ERR(host->power)) {
		if (host->power_enabled && !regulator_disable(host->power))
			host->power_enabled = false;
	} else if (host->pdata->gpio) {
		set_pin_status(&host->pdata->gpio->pwr, 0);
	}
}

static int ingenic_mmc_get_read_only(struct mmc_host *mmc)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	if (host->pdata->gpio)
		ret = get_pin_status(&host->pdata->gpio->wp);
	return ret < 0 ? 0 : ret;
}

static int ingenic_mmc_get_card_detect(struct mmc_host *mmc)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);
	int ret = -1;

	if (host->pdata->removal == NONREMOVABLE || host->pdata->removal == MANUAL)
		return test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);

	if (host->pdata->gpio)
		ret = get_pin_status(&host->pdata->gpio->cd);
	return ret < 0 ? 1 : ret;
}

static void ingenic_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_1BIT;
		break;
	case MMC_BUS_WIDTH_4:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_4BIT;
		break;
	case MMC_BUS_WIDTH_8:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_8BIT;
		break;
	}

	if (ios->clock) {
		unsigned int clk_want = ios->clock;
		unsigned int clk_set, clkrt = 0, lpm = 0;

		// The rate changes with the clocks off; below 3 MHz the source
		// stays at 24 MHz and CLKRT divides it.
		ingenic_mmc_clk_onoff(host, 0);
		if (clk_want > 3000000)
			clk_set_rate(host->clk_cgu, ios->clock);
		else
			clk_set_rate(host->clk_cgu, CLK_RATE);
		clk_set = clk_get_rate(host->clk_cgu);

		while (clk_want < clk_set) {
			clkrt++;
			clk_set >>= 1;
		}

		if (clk_want > 3000000 && clkrt) {
			dev_err(host->dev, "CLKRT must be set to 0 when MSC works during normal r/w: "
				"ios->clock=%d clk_want=%d clk_set=%d clkrt=%X,\n",
				ios->clock, clk_want, clk_set, clkrt);
			WARN_ON(1);
		}
		if (clkrt > 7) {
			dev_err(host->dev, "invalid value of CLKRT: "
				"ios->clock=%d clk_want=%d clk_set=%d clkrt=%X,\n",
				ios->clock, clk_want, clk_set, clkrt);
			WARN_ON(1);
			return;
		}

		ingenic_mmc_clk_onoff(host, 1);
		msc_writel(host, CLKRT, clkrt);

		if (clk_set > 25000000)
			lpm = LPM_DRV_SEL_RISING;

		if (host->pdata->sdio_clk) {
			msc_writel(host, LPM, lpm);
			msc_writel(host, CTRL, CTRL_CLOCK_START);
		} else {
			lpm |= LPM_LPM;
			msc_writel(host, LPM, lpm);
		}
	}

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		host->cmdat_def |= CMDAT_INIT;
		set_bit(INGENIC_MMC_CARD_NEED_INIT, &host->flags);
		ingenic_mmc_power_on(host);
		break;
	case MMC_POWER_OFF:
		ingenic_mmc_power_off(host);
		break;
	default:
		break;
	}
}

static void ingenic_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	if (enable) {
		enable_msc_irq(host, IRQ_SDIO);
	} else {
		clear_msc_irq(host, IRQ_SDIO);
		disable_msc_irq(host, IRQ_SDIO);
	}
}

static const struct mmc_host_ops ingenic_mmc_ops = {
	.request		= ingenic_mmc_request,
	.set_ios		= ingenic_mmc_set_ios,
	.get_ro			= ingenic_mmc_get_read_only,
	.get_cd			= ingenic_mmc_get_card_detect,
	.enable_sdio_irq	= ingenic_mmc_enable_sdio_irq,
};

// --- sysfs ------------------------------------------------------------------

static ssize_t ingenic_mmc_present_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_mmc_host *host = dev_get_drvdata(dev);

	if (test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags))
		return sprintf(buf, "Y\n");
	return sprintf(buf, "N\n");
}

static ssize_t ingenic_mmc_present_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct ingenic_mmc_host *host = dev_get_drvdata(dev);

	if (!buf || host->pdata->removal != NONREMOVABLE) {
		dev_err(host->dev, "can't set present\n");
		return count;
	}

	if (strncmp(buf, "INSERT", 6) == 0) {
		dev_info(host->dev, "card insert via sysfs\n");
		set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		mmc_detect_change(host->mmc, 0);
	} else if (strncmp(buf, "REMOVE", 6) == 0) {
		dev_info(host->dev, "card remove via sysfs\n");
		clear_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		mmc_detect_change(host->mmc, 0);
		ingenic_mmc_reset(host);
	} else {
		dev_err(host->dev, "set present error, the argument can't be recognised\n");
	}
	return count;
}

static DEVICE_ATTR(present, S_IWUSR | S_IRUSR, ingenic_mmc_present_show, ingenic_mmc_present_store);

static struct attribute *ingenic_mmc_attributes[] = {
	&dev_attr_present.attr,
	NULL
};

static const struct attribute_group ingenic_mmc_attr_group = {
	.attrs = ingenic_mmc_attributes,
};

// --- probe and remove -------------------------------------------------------

// The card voltage range of controller index, added to *mask.
int jzmmc_of_parse_voltage(int index, u32 *mask)
{
	u32 ocr;

	if (index < 0 || index >= (int)ARRAY_SIZE(jzmsc_gpio))
		return -EINVAL;
	ocr = mmc_vddrange_to_ocrmask(jzmsc_gpio[index].voltage_min, jzmsc_gpio[index].voltage_max);
	if (!ocr) {
		pr_err("%s: voltage-range  is invalid\n", __func__);
		return -EINVAL;
	}
	*mask |= ocr;
	return 0;
}
EXPORT_SYMBOL(jzmmc_of_parse_voltage);

// The host capabilities from the parameters, as mmc_of_parse() takes them
// from a device tree.
static int ingenic_mmc_parse(struct mmc_host *mmc, const struct msc_config *cfg)
{
	bool ro_gpio = false;

	if (!mmc->parent)
		return 0;

	switch (cfg->bus_width) {
	case 8:
		mmc->caps |= MMC_CAP_8_BIT_DATA;
		// fall through - a host that does 8 bits also does 4
	case 4:
		mmc->caps |= MMC_CAP_4_BIT_DATA;
		break;
	case 1:
		break;
	default:
		dev_err(mmc->parent, "Invalid \"bus-width\" value %u!\n", cfg->bus_width);
		return -EINVAL;
	}

	mmc->f_max = cfg->max_frequency;

	if (!strcmp(cfg->cd_method, "non-removable")) {
		mmc->caps |= MMC_CAP_NONREMOVABLE;
	} else {
		if (!strcmp(cfg->cd_method, "broken-cd"))
			mmc->caps |= MMC_CAP_NEEDS_POLL;
		if (cfg->cd >= 0)
			dev_info(mmc->parent, "Got CD GPIO\n");
		else
			mmc->caps2 |= MMC_CAP2_CD_ACTIVE_HIGH;
	}

	if (cfg->wp >= 0) {
		dev_info(mmc->parent, "Got WP GPIO\n");
		ro_gpio = true;
	}
	if (cfg->wp != -1)
		mmc->caps2 |= MMC_CAP2_NO_WRITE_PROTECT;
	if ((cfg->wp_enable_level != 0) != ro_gpio)
		mmc->caps2 |= MMC_CAP2_RO_ACTIVE_HIGH;

	if (!strcmp(cfg->speed, "sd_card"))
		mmc->caps |= MMC_CAP_SD_HIGHSPEED;
	if (!strcmp(cfg->speed, "sdio") || cfg->sdr >= 0)
		mmc->caps |= MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_SDR50 |
			     MMC_CAP_UHS_SDR104 | MMC_CAP_UHS_DDR50;
	if (!strcmp(cfg->speed, "emmc")) {
		mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_1_8V_DDR;
		mmc->caps2 |= MMC_CAP2_HS200_1_8V_SDR;
	}

	if (cfg->cap_power_off_card)
		mmc->caps |= MMC_CAP_POWER_OFF_CARD;
	if (cfg->cap_mmc_hw_reset)
		mmc->caps |= MMC_CAP_HW_RESET;
	if (cfg->cap_sdio_irq)
		mmc->caps |= MMC_CAP_SDIO_IRQ;
	if (cfg->full_pwr_cycle)
		mmc->caps2 |= MMC_CAP2_FULL_PWR_CYCLE;
	if (cfg->keep_power_in_suspend)
		mmc->pm_caps |= MMC_PM_KEEP_POWER;
	if (cfg->enable_sdio_wakeup)
		mmc->pm_caps |= MMC_PM_WAKE_SDIO_IRQ;

	mmc->dsr = cfg->dsr;
	mmc->dsr_req = cfg->dsr != 0;
	if (mmc->dsr_req && (mmc->dsr & ~0xffff)) {
		dev_err(mmc->parent, "device tree specified broken value for DSR: 0x%x, ignoring\n",
			mmc->dsr);
		mmc->dsr_req = 0;
	}
	return 0;
}

static int ingenic_mmc_dma_init(struct ingenic_mmc_host *host)
{
	struct sdma_desc *next_desc;
	void __iomem *desc;
	int i;

	host->desc_page = get_zeroed_page(GFP_KERNEL);
	if (!host->desc_page) {
		dev_err(host->dev, "get DMA descriptor memory error\n");
		return -ENODEV;
	}
	// Zeroed through the cache, used uncached: the dirty lines go out now
	// rather than over the descriptors later.
	dma_cache_wback_inv(host->desc_page, PAGE_SIZE);

	desc = devm_ioremap_nocache(host->dev, virt_to_phys((void *)host->desc_page), PAGE_SIZE);
	if (!desc) {
		dev_err(host->dev, "remap descriptor memory error\n");
		free_pages(host->desc_page, 0);
		host->desc_page = 0;
		return -ENODEV;
	}

	next_desc = (struct sdma_desc __force *)desc;
	for (i = 0; i < MAX_SEGS; i++) {
		struct desc_hd *dhd = &host->decshds[i];

		dhd->dma_desc = next_desc;
		dhd->dma_desc_phys_addr = CPHYSADDR((unsigned long)next_desc);
		dhd->next = dhd + 1;
		next_desc++;
	}
	host->decshds[MAX_SEGS - 1].next = NULL;
	return 0;
}

static void ingenic_mmc_init_gpio(struct ingenic_mmc_pin *pin, const char *name, int dir)
{
	if (gpio_is_valid(pin->num) && gpio_request_one(pin->num, dir, name)) {
		pr_info("%s no detect pin available\n", name);
		pin->num = -EBUSY;
	}
}

static void ingenic_mmc_gpio_deinit(struct ingenic_mmc_host *host)
{
	struct card_gpio *card_gpio = host->pdata->gpio;

	if (gpio_is_valid(card_gpio->cd.num))
		gpio_free(card_gpio->cd.num);
	if (gpio_is_valid(card_gpio->wp.num))
		gpio_free(card_gpio->wp.num);
	if (gpio_is_valid(card_gpio->pwr.num))
		gpio_free(card_gpio->pwr.num);
	if (gpio_is_valid(card_gpio->rst.num))
		gpio_free(card_gpio->rst.num);
}

// Requests the slot's pins, and sets up card detection for its removal
// method. The power pin starts with the card off.
static int ingenic_mmc_gpio_init(struct ingenic_mmc_host *host)
{
	struct card_gpio *card_gpio = host->pdata->gpio;
	int index = host->index;
	int ret;

	if (card_gpio->cd.num >= 0) {
		snprintf(host->labels[0], sizeof(host->labels[0]), "msc%d_cd", index);
		ingenic_mmc_init_gpio(&card_gpio->cd, host->labels[0], GPIOF_DIR_IN);
	}
	if (card_gpio->wp.num >= 0) {
		snprintf(host->labels[1], sizeof(host->labels[1]), "msc%d_wp", index);
		ingenic_mmc_init_gpio(&card_gpio->wp, host->labels[1], GPIOF_DIR_IN);
	}
	if (card_gpio->rst.num >= 0) {
		snprintf(host->labels[2], sizeof(host->labels[2]), "msc%d_rst", index);
		ingenic_mmc_init_gpio(&card_gpio->rst, host->labels[2], GPIOF_DIR_OUT);
	}
	if (card_gpio->pwr.num >= 0) {
		snprintf(host->labels[3], sizeof(host->labels[3]), "msc%d_pwr", index);
		ingenic_mmc_init_gpio(&card_gpio->pwr, host->labels[3],
				      card_gpio->pwr.enable_level ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH);
	}

	switch (host->pdata->removal) {
	case NONREMOVABLE:
		break;
	case REMOVABLE:
		if (!gpio_is_valid(card_gpio->cd.num)) {
			set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
			break;
		}
		setup_timer(&host->detect_timer, ingenic_mmc_detect, (unsigned long)host);
		ret = devm_request_irq(host->dev, gpio_to_irq(card_gpio->cd.num), ingenic_mmc_detect_handler,
				       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "mmc-insert-detect", host);
		if (ret) {
			dev_err(host->dev, "request detect irq-%d fail\n", gpio_to_irq(card_gpio->cd.num));
			return ret;
		}
		host->cd_irq = gpio_to_irq(card_gpio->cd.num);

		// The first look at the slot, from the timer as after an edge.
		if (!timer_pending(&host->detect_timer)) {
			disable_irq_nosync(gpio_to_irq(card_gpio->cd.num));
			mod_timer(&host->detect_timer, jiffies);
		}
		break;
	case MANUAL:
		list_add(&host->list, &manual_list);
		break;
	default:
		set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		break;
	}
	return 0;
}

// Stops the card-detect interrupt and its timer for good.
static void ingenic_mmc_stop_detect(struct ingenic_mmc_host *host)
{
	if (host->cd_irq < 0)
		return;
	del_timer_sync(&host->detect_timer);
	devm_free_irq(host->dev, host->cd_irq, host);
	del_timer_sync(&host->detect_timer);
	host->cd_irq = -1;
}

static void ingenic_mmc_free_bus_pins(struct ingenic_mmc_host *host)
{
	int i;

	for (i = 0; i < host->bus_pins; i++)
		gpio_free(jzmsc_gpio[host->index].pins[i].gpio);
	host->bus_pins = 0;
}

static int mmc_ingenic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc_config *cfg;
	struct ingenic_mmc_pdata *pdata;
	struct card_gpio *card_gpio;
	struct ingenic_mmc_host *host;
	struct mmc_host *mmc;
	struct resource *regs;
	char clk_cgu_name[16], clk_gate_name[16];
	int bus_pins, i, ret;

	if (pdev->id < 0 || pdev->id >= (int)ARRAY_SIZE(jzmsc_gpio))
		return -ENODEV;
	cfg = &jzmsc_gpio[pdev->id];

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	card_gpio = devm_kzalloc(dev, sizeof(*card_gpio), GFP_KERNEL);
	if (!pdata || !card_gpio)
		return -ENOMEM;

	// The bus pins: clock, command and the data lines in use.
	for (i = 0; i < cfg->bus_width + 2 && i < MSC_PINS; i++) {
		const struct msc_pin *pin = &cfg->pins[i];

		if (gpio_request(pin->gpio, pin->name) < 0) {
			char buf[16];

			pr_err("MSC failed to request %s %s!\n", gpio_to_str(pin->gpio, buf), pin->name);
			pr_err("msc%d gpio request failed\n", pdev->id);
			break;
		}
		gpio_set_func(pin->gpio, pin->func);
	}
	bus_pins = i;

	card_gpio->rst.num = cfg->rst;
	card_gpio->rst.enable_level = cfg->rst_enable_level;
	card_gpio->wp.num = cfg->wp;
	card_gpio->wp.enable_level = cfg->wp_enable_level;
	card_gpio->pwr.num = cfg->pwr;
	card_gpio->pwr.enable_level = cfg->pwr_enable_level;
	card_gpio->cd.num = cfg->cd;
	card_gpio->cd.enable_level = cfg->cd_enable_level;
	pdata->gpio = card_gpio;
	if (cfg->pio_mode)
		pdata->pio_mode = 1;
	pdata->sdio_clk = cfg->sdio_clk;
	if (!strcmp(cfg->rm_method, "dontcare"))
		pdata->removal = DONTCARE;
	if (!strcmp(cfg->rm_method, "nonremovable"))
		pdata->removal = NONREMOVABLE;
	if (!strcmp(cfg->rm_method, "removable"))
		pdata->removal = REMOVABLE;
	if (!strcmp(cfg->rm_method, "manual"))
		pdata->removal = MANUAL;
	pdata->pwr_regulator = cfg->pwr_regulator;
	jzmmc_of_parse_voltage(pdev->id, &pdata->ocr_avail);

	mmc = mmc_alloc_host(sizeof(struct ingenic_mmc_host), dev);
	if (!mmc) {
		dev_err(dev, "mmc probe error\n");
		ret = -ENOMEM;
		goto err_pins;
	}
	host = mmc_priv(mmc);
	host->index = pdev->id;
	host->bus_pins = bus_pins;
	host->cd_irq = -1;

	sprintf(clk_cgu_name, "div_msc%d", host->index);
	sprintf(clk_gate_name, "gate_msc%d", host->index);
	printk("%s %s\n", clk_cgu_name, clk_gate_name);
	host->clk_cgu = devm_clk_get(dev, clk_cgu_name);
	if (IS_ERR_OR_NULL(host->clk_cgu)) {
		dev_err(dev, "Failed to Get MSC clk!\n");
		ret = host->clk_cgu ? PTR_ERR(host->clk_cgu) : -ENOENT;
		goto err_free_host;
	}
	host->clk_gate = devm_clk_get(dev, clk_gate_name);
	if (IS_ERR_OR_NULL(host->clk_gate)) {
		dev_err(dev, "Failed to Get PWC MSC clk!\n");
		ret = host->clk_gate ? PTR_ERR(host->clk_gate) : -ENOENT;
		goto err_free_host;
	}

	clk_set_rate(host->clk_cgu, CLK_RATE);
	if (clk_get_rate(host->clk_cgu) > CLK_RATE) {
		dev_err(dev, "Failed to Set MSC clk %ld!\n", clk_get_rate(host->clk_cgu));
		ret = -EINVAL;
		goto err_free_host;
	}
	ingenic_mmc_clk_onoff(host, 1);

	host->dev = dev;
	host->pdata = pdata;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->iomem = devm_ioremap_resource(dev, regs);
	if (IS_ERR(host->iomem)) {
		dev_err(dev, "Err iomem resource\n");
		ret = PTR_ERR(host->iomem);
		goto err_clk;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		dev_err(dev, "No irq resource\n");
		ret = host->irq;
		goto err_clk;
	}

	if (strlen(pdata->pwr_regulator) && strcmp(pdata->pwr_regulator, "-1")) {
		host->power = regulator_get(host->dev, pdata->pwr_regulator);
		if (!host->power || IS_ERR(host->power))
			dev_warn(host->dev, "mmc %s regulator missing\n", pdata->pwr_regulator);
	} else {
		host->power = NULL;
	}

	if (pdata->pio_mode)
		set_bit(INGENIC_MMC_USE_PIO, &host->flags);
	if (!test_bit(INGENIC_MMC_USE_PIO, &host->flags)) {
		ret = ingenic_mmc_dma_init(host);
		if (ret < 0)
			goto err_regulator;
	}

	spin_lock_init(&host->lock);
	if (pdata->sdio_clk)
		ingenic_sdio_wlan_init(dev, host->index);

	ingenic_mmc_reset(host);
	host->cmdat_def = 0;	// RTRG and TTRG at 16, 1-bit bus
	ret = devm_request_threaded_irq(dev, host->irq, NULL, ingenic_mmc_thread_handle,
					IRQF_ONESHOT, dev_name(dev), host);
	if (ret < 0)
		goto err_desc;

	mmc->ops = &ingenic_mmc_ops;
	mmc->f_min = 200000;
	mmc->ocr_avail = pdata->ocr_avail;
	mmc->pm_flags |= pdata->pm_flags;
	mmc->max_segs = MAX_SEGS;
	mmc->max_blk_count = 4096;
	mmc->max_req_size = 4096 * 512;
	mmc->max_blk_size = 512;
	mmc->max_seg_size = mmc->max_req_size;
	host->mmc = mmc;
	setup_timer(&host->request_timer, ingenic_mmc_request_timeout, (unsigned long)host);
	ingenic_mmc_parse(mmc, cfg);
	ret = mmc_add_host(mmc);
	if (ret)
		goto err_irq;

	platform_set_drvdata(pdev, host);
	ret = ingenic_mmc_gpio_init(host);
	if (ret < 0)
		goto err_remove;

	ret = sysfs_create_group(&dev->kobj, &ingenic_mmc_attr_group);
	if (ret < 0)
		goto err_remove;

	dev_info(host->dev, "register success!\n");
	return 0;

err_remove:
	if (host->pdata->removal == MANUAL)
		list_del(&host->list);
	ingenic_mmc_stop_detect(host);
	mmc_remove_host(mmc);
	ingenic_mmc_gpio_deinit(host);
	platform_set_drvdata(pdev, NULL);
err_irq:
	devm_free_irq(dev, host->irq, host);
	del_timer_sync(&host->request_timer);
err_desc:
	if (host->desc_page) {
		free_pages(host->desc_page, 0);
		host->desc_page = 0;
	}
err_regulator:
	if (host->power && !IS_ERR(host->power))
		regulator_put(host->power);
err_clk:
	while (atomic_read(&host->clk_count) > 0)
		ingenic_mmc_clk_onoff(host, 0);
err_free_host:
	mmc_free_host(mmc);
	dev_err(dev, "mmc probe error\n");
err_pins:
	for (i = 0; i < bus_pins; i++)
		gpio_free(cfg->pins[i].gpio);
	return ret;
}

// Undoes the probe: the card-detect interrupt and the sysfs file go first,
// then the core lets go of the card, then the controller interrupt, the
// timer, the clocks and the pins, and the host last.
static int mmc_ingenic_remove(struct platform_device *pdev)
{
	struct ingenic_mmc_host *host = platform_get_drvdata(pdev);
	struct mmc_host *mmc = host->mmc;

	if (host->pdata->removal == MANUAL)
		list_del(&host->list);
	sysfs_remove_group(&pdev->dev.kobj, &ingenic_mmc_attr_group);
	ingenic_mmc_stop_detect(host);
	mmc_remove_host(mmc);
	ingenic_mmc_power_off(host);

	devm_free_irq(&pdev->dev, host->irq, host);
	del_timer_sync(&host->request_timer);
	ingenic_mmc_gpio_deinit(host);
	if (host->desc_page)
		free_pages(host->desc_page, 0);
	if (host->power && !IS_ERR(host->power))
		regulator_put(host->power);
	while (atomic_read(&host->clk_count) > 0)
		ingenic_mmc_clk_onoff(host, 0);
	ingenic_mmc_free_bus_pins(host);

	platform_set_drvdata(pdev, NULL);
	mmc_free_host(mmc);
	return 0;
}

// At shutdown a memory card (not an SDIO one) is held in reset, or removed
// from the core, so no request reaches it any more.
static void mmc_ingenic_shutdown(struct platform_device *pdev)
{
	struct ingenic_mmc_host *host = platform_get_drvdata(pdev);
	struct card_gpio *card_gpio = host->pdata->gpio;

	if (host->mmc->card && !mmc_card_sdio(host->mmc->card)) {
		if (gpio_is_valid(card_gpio->rst.num))
			gpio_direction_output(card_gpio->rst.num, 0);
		else
			mmc_remove_host(host->mmc);
	}
}

#ifdef CONFIG_PM_SLEEP
static int mmc_ingenic_suspend(struct device *dev)
{
	struct ingenic_mmc_host *host = dev_get_drvdata(dev);

	ingenic_mmc_clk_onoff(host, 0);
	return 0;
}

static int mmc_ingenic_resume(struct device *dev)
{
	struct ingenic_mmc_host *host = dev_get_drvdata(dev);

	ingenic_mmc_clk_onoff(host, 1);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mmc_ingenic_pm_ops, mmc_ingenic_suspend, mmc_ingenic_resume);

static const struct of_device_id mmc_ingenic_of_match[] = {
	{ .compatible = "md_ingenic,mmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mmc_ingenic_of_match);

static struct platform_driver mmc_ingenic_driver = {
	.driver = {
		.name = "md_ingenic,mmc",
		.owner = THIS_MODULE,
		.pm = &mmc_ingenic_pm_ops,
		.of_match_table = of_match_ptr(mmc_ingenic_of_match),
	},
	.probe = mmc_ingenic_probe,
	.remove = mmc_ingenic_remove,
	.shutdown = mmc_ingenic_shutdown,
};

// --- the two controllers ----------------------------------------------------

static u64 msc_dmamask = DMA_BIT_MASK(32);

static void msc_dev_release(struct device *dev)
{
}

static struct resource msc0_resources[] = {
	{ .start = 0x13450000, .end = 0x1345ffff, .flags = IORESOURCE_MEM },
	{ .start = 45, .end = 45, .flags = IORESOURCE_IRQ },
};

static struct resource msc1_resources[] = {
	{ .start = 0x13460000, .end = 0x1346ffff, .flags = IORESOURCE_MEM },
	{ .start = 44, .end = 44, .flags = IORESOURCE_IRQ },
};

static struct platform_device msc0_device = {
	.name = "md_ingenic,mmc",
	.id = 0,
	.dev = {
		.dma_mask = &msc_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.release = msc_dev_release,
	},
	.num_resources = ARRAY_SIZE(msc0_resources),
	.resource = msc0_resources,
};

static struct platform_device msc1_device = {
	.name = "md_ingenic,mmc",
	.id = 1,
	.dev = {
		.dma_mask = &msc_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.release = msc_dev_release,
	},
	.num_resources = ARRAY_SIZE(msc1_resources),
	.resource = msc1_resources,
};

static bool msc0_registered, msc1_registered;

static int __init msc_init(void)
{
	int ret;

	if (wifi_power_on != -1)
		gpio_set_func(wifi_power_on, wifi_power_on_level ? GPIO_OUTPUT1 : GPIO_OUTPUT0);
	if (wifi_reg_on != -1)
		gpio_set_func(wifi_reg_on, wifi_reg_on_level ? GPIO_OUTPUT1 : GPIO_OUTPUT0);

	if (jzmsc_gpio[0].is_enable) {
		ret = platform_device_register(&msc0_device);
		if (ret) {
			pr_err("msc0: Failed to register msc dev: %d\n", ret);
			return ret;
		}
		msc0_registered = true;
	}
	if (jzmsc_gpio[1].is_enable) {
		ret = platform_device_register(&msc1_device);
		if (ret) {
			pr_err("msc1: Failed to register msc dev: %d\n", ret);
			goto err_devices;
		}
		msc1_registered = true;
	}

	ret = platform_driver_register(&mmc_ingenic_driver);
	if (ret)
		goto err_devices;
	return 0;

err_devices:
	if (msc1_registered)
		platform_device_unregister(&msc1_device);
	if (msc0_registered)
		platform_device_unregister(&msc0_device);
	msc0_registered = msc1_registered = false;
	return ret;
}
module_init(msc_init);

static void __exit msc_exit(void)
{
	if (msc0_registered)
		platform_device_unregister(&msc0_device);
	if (msc1_registered)
		platform_device_unregister(&msc1_device);
	platform_driver_unregister(&mmc_ingenic_driver);
}
module_exit(msc_exit);

MODULE_DESCRIPTION("Multimedia Card Interface driver, MMC version 1.2");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20170222");
