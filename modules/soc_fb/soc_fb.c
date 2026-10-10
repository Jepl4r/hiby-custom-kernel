// SPDX-License-Identifier: GPL-2.0
//
// soc_fb -- the X1600 LCD controller as a framebuffer (/dev/fb0) for the
// panel module of the board, which describes its panel with
// jzfb_register_lcd(): lcd_st7701_sbtc033001.ko on the R3 Pro II,
// lcd_lg35583.ko on the R1.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock soc_fb.sh passes lcd_is_inited=0 frame_num=2
// pan_display_sync=1):
//
//   frame_num          frames in the buffer; FBIOPAN_DISPLAY picks the one
//                      on screen
//   pan_display_sync   nonzero: a pan returns only after the frame
//                      interrupt, on smart panels too (a pan on an RGB
//                      panel waits for it anyway)
//   lcd_is_inited      nonzero: the panel is left as the boot loader set it
//                      up: no power_on and no set-up commands, and a smart
//                      panel keeps the set-up clock and is not switched to
//                      streaming frames
//
// Loading touches neither the controller nor the panel: both are set up at
// the first FB_BLANK_UNBLANK. Parallel RGB (TFT) panels and smart (MCU: 8080, 6800 or SPI) panels are driven; both boards
// use RGB. The frames come from rmem_manager.ko. The controller fetches them
// through two descriptors, used in turn by the pans, each pointing at itself
// so that the frame repeats until the next pan.
//
// slcd_read_data_only() reads bytes back from a smart panel by driving its
// bus through the GPIO functions.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/addrspace.h>
#include <asm/barrier.h>
#include <soc/gpio.h>

// Exported by utils.ko.
extern void __assert(const char *expr, const char *file, int line, const char *func);
extern int gpio_set_func(int gpio, int func);
extern int gpio_port_set_func(int port, unsigned int pins, int func);
extern u64 local_clock_us(void);

// Exported by rmem_manager.ko.
extern unsigned long rmem_alloc_aligned(int size, int align);
extern void rmem_free(unsigned long addr, int size);

#define JZFB_ASSERT(x) \
	do { if (!(x)) __assert(#x, __FILE__, __LINE__, __func__); } while (0)

// A registration the module cannot work with.
#define JZFB_CHECK(cond) \
	do { if (cond) panic("fb: failed to check: %s\n", #cond); } while (0)

// --- registers --------------------------------------------------------------

#define LCDC_PHYS		0x13050000
#define LCDC_REG(off)		((void __iomem *)CKSEG1ADDR(LCDC_PHYS + (off)))
#define LCDC_FRM_DESC		0x1000		// physical address of the descriptor
#define LCDC_FRM_START		0x1004		// write 1: fetch it
#define LCDC_CTRL		0x2000
#define  LCDC_CTRL_QUICK_STOP	BIT(1)
#define  LCDC_CTRL_GEN_STOP	BIT(4)		// stop at the end of the frame
#define LCDC_CLR_ST		0x2008		// write 1: clear the status bit
#define LCDC_INT_EN		0x200c
#define LCDC_ST			0x2010
#define  LCDC_ST_STOPPED	BIT(7)		// a GEN_STOP is done
#define  LCDC_ST_UNDERRUN	BIT(8)
#define  LCDC_ST_FRM_DONE	BIT(17)		// the descriptor is fetched
#define LCDC_CFG2		0x2014		// bits 5:4 set at enable
#define LCDC_COM_CFG		0x8000
#define  COM_CFG_IF_MASK	0x3		// 1: TFT, 2: smart panel
#define  COM_CFG_DITHER_EN	BIT(4)
#define  COM_CFG_BIT5		BIT(5)
#define  COM_CFG_BIT6		BIT(6)
#define  COM_CFG_DITHER_SHIFT	16		// 2 bits a colour: R 21:20, G 19:18, B 17:16
#define  COM_CFG_DITHER_MASK	(0x3f << COM_CFG_DITHER_SHIFT)
#define LCDC_TFT_HSYNC		0x9000		// hsync_len 27:16, total 11:0
#define LCDC_TFT_VSYNC		0x9004
#define LCDC_TFT_HDE		0x9008		// first active pixel 27:16, end 11:0
#define LCDC_TFT_VDE		0x900c
#define LCDC_TFT_CFG		0x9010
#define LCDC_SLCD_CFG		0xa000
#define  SLCD_CFG_FRM_EN	BIT(26)		// stream the frame
#define LCDC_SLCD_WR_DUTY	0xa004
#define LCDC_SLCD_TIMING	0xa008
#define LCDC_SLCD_FRM_SIZE	0xa00c		// yres 31:16, xres 15:0
#define LCDC_SLCD_SLOW_TIME	0xa010
#define LCDC_SLCD_FIFO		0xa014
#define  SLCD_FIFO_CMD		BIT(31)
#define  SLCD_FIFO_DATA		BIT(30)
#define  SLCD_FIFO_VALUE	0xffffff
#define LCDC_SLCD_ST		0xa018
#define  SLCD_ST_BUSY		BIT(0)

#define TIMING_MASK		0xfff

#define CPM_PHYS		0x10000000
#define CPM_REG(off)		((void __iomem *)CKSEG1ADDR(CPM_PHYS + (off)))
#define CPM_LPCDR		0x64		// LCD pixel clock divider

#define LCDC_IRQ		39

// The GPIO function of the controller pins on port A.
#define TFT_PIN_FUNC		GPIO_FUNC_0
#define SLCD_PIN_FUNC		(GPIO_PULL_HIZ | GPIO_FUNC_1)

// Pins on port A: the smart panel lines past its data lines from PA0, and
// the RGB pixel clock.
#define SLCD_PIN_DC		25
#define SLCD_PIN_WR		26
#define SLCD_PIN_TE		27
#define TFT_PIN_PCLK		24

// The descriptor waits: 300 ms at HZ=100.
#define JZFB_WAIT_JIFFIES	30
#define STOP_WAIT_MS		100
#define SLCD_BUSY_TRIES		10001
#define SLCD_BUSY_PAN_US	10000
#define SLCD_BUSY_READ_US	100000

// --- the panel description --------------------------------------------------

// The pixel format in memory.
enum fb_fmt {
	FB_FMT_RGB555,
	FB_FMT_RGB565,
	FB_FMT_RGB888,		// 32 bits a pixel
	FB_FMT_ARGB8888,
};

enum lcd_mode {
	LCD_MODE_TFT_24BIT,	// parallel RGB, by out_format
	LCD_MODE_TFT_18BIT,
	LCD_MODE_TFT_2,		// not driven
	LCD_MODE_TFT_3,		// not driven
	LCD_MODE_TFT_8BIT,	// 8-bit serial RGB
	LCD_MODE_TFT_8BIT_B,
	LCD_MODE_SLCD_6800,	// smart panels from here
	LCD_MODE_SLCD_8080,
	LCD_MODE_SLCD_SPI_3,	// SPI from here
	LCD_MODE_SLCD_SPI_4,
};

#define LCD_MODE_IS_TFT(m)	((m) < LCD_MODE_SLCD_6800)

// The pixel format on the pins.
enum out_format {
	OUT_RGB565,
	OUT_RGB666,
	OUT_RGB888,
	OUT_RGB444,
	OUT_RGB555,
};

// The bus width of a smart panel, data_width and cmd_width.
enum slcd_width {
	SLCD_WIDTH_8,
	SLCD_WIDTH_9,
	SLCD_WIDTH_16,
};

enum slcd_cmd_type {
	SLCD_CMD_DATA,
	SLCD_CMD_COMMAND,
	SLCD_CMD_SLEEP_US,
};

struct slcd_cmd {
	unsigned int type;	// enum slcd_cmd_type
	unsigned int value;
};

// The layout the panel modules fill; offsets are part of the interface.
struct lcdc_data {
	const char *name;
	unsigned int refresh;		// Hz; 0: 40
	unsigned int xres;
	unsigned int yres;
	unsigned int pixclock;		// Hz; 0: from the timings
	unsigned int left_margin;
	unsigned int right_margin;
	unsigned int upper_margin;
	unsigned int lower_margin;
	unsigned int hsync_len;
	unsigned int vsync_len;
	unsigned int fb_fmt;		// enum fb_fmt
	unsigned int lcd_mode;		// enum lcd_mode
	unsigned int out_format;	// enum out_format
	unsigned int color_even;	// TFT: colour order of even lines
	unsigned int color_odd;		// TFT: colour order of odd lines
	unsigned int pix_clk_active;	// TFT: 1 rising edge
	unsigned int de_active_level;	// TFT: 1 high
	unsigned int hsync_active_level;
	unsigned int vsync_active_level;
	unsigned int lpcdr_bit26;	// TFT: 1 sets bit 26 of CPM_LPCDR
	unsigned int slcd_init_pixclock; // smart: Hz for the set-up commands
	unsigned int reserved_88;
	int rd_gpio;			// smart: RD pin, for slcd_read_data_only()
	unsigned int write_mem_cmd;	// smart: sent after the set-up commands
	unsigned int data_width;	// smart: enum slcd_width
	unsigned int cmd_width;		// smart: enum slcd_width
	unsigned int wr_level;		// smart: 1 WR idles high
	unsigned int rd_level;		// smart: 1 RD idles high
	unsigned int dc_level;		// smart: 1 DC high for commands
	unsigned int slcd_bit6;		// smart: 1 sets bit 6 of SLCD_CFG
	unsigned int te_mode;		// smart: 2 the TE pin is used
	unsigned int slcd_bit10;	// smart: 1 sets bit 10 of SLCD_CFG
	unsigned int rdy_used;		// smart: bit 0, a RDY pin (none on the X1600)
	unsigned int height_mm;
	unsigned int width_mm;
	const struct slcd_cmd *init_cmds; // smart: the set-up commands
	unsigned int init_cmd_count;
	int (*power_on)(void *unused);
	int (*power_off)(void *unused);
	int (*set_rotate)(u16 rotate);	// JZFB_SET_ROTATE; NULL: none
};

// --- state ------------------------------------------------------------------

// A frame descriptor as the controller reads it.
struct lcdc_desc {
	u32 next;		// physical, this descriptor again
	u32 addr;		// physical, the frame
	u32 stride;		// pixels
	u32 cfg;		// format 22:19, bit 0 for smart panels
	u32 ctrl;
};

#define DESC_SIZE		64
#define DESC_CFG_FMT_SHIFT	19
#define DESC_CFG_FMT_MASK	(0xf << DESC_CFG_FMT_SHIFT)
#define DESC_CFG_SLCD		BIT(0)
#define DESC_CTRL_BIT17		BIT(17)
#define DESC_SYNC_LEN		20	// the words above

// The frame buffer as the fb ops see it.
struct jzfb_mem {
	unsigned long vaddr;
	unsigned int fb_fmt;
	unsigned int xres;
	unsigned int yres;
	unsigned int line_length;
	unsigned int frame_size;
	unsigned int frame_num;
};

// What lcdc_srdma_init() writes into the descriptors.
struct jzfb_srdma {
	unsigned int fb_fmt;
	unsigned long mem;
	unsigned int is_tft;
	unsigned int xres;
};

// fb_info->par.
struct jzfb_par {
	struct fb_info *fb;
	int enabled;			// unblanked
	int frame_num;			// the parameter
	unsigned long mem;
	unsigned int mem_size;
	struct jzfb_mem m;
	struct fb_videomode mode;
	struct jzfb_srdma srdma;
};

// The pan state, set by the pans and the interrupt.
enum jzfb_state {
	JZFB_IDLE,
	JZFB_FETCHING,
	JZFB_FRAME_DONE,
	JZFB_STOPPED,
};

struct jzfb {
	struct lcdc_desc *desc[2];
	struct clk *clk;		// gate_lcd
	struct clk *pclk;		// div_lcd
	int irq;
	struct mutex lock;
	spinlock_t irq_lock;
	wait_queue_head_t wait_queue;
	struct lcdc_data *pdata;
	int is_open;
	int enabled;			// enable count
	int is_stopping;		// waiting for LCDC_ST_STOPPED
	int frame_idx;
	int state;			// enum jzfb_state
	int pan_display_sync;		// the parameter
	struct jzfb_par par;
};

// Global as in the vendor module.
struct jzfb jzfb;

static int is_wait_stop_cnt;
static u16 display_rotate_set;
static u16 display_rotate;
static int lcd_is_inited;
static int spi_send_delay;		// us after each smart panel word
static u32 m_pins;			// port A pins requested

module_param(lcd_is_inited, int, 0644);
module_param_named(pan_display_sync, jzfb.pan_display_sync, int, 0644);
module_param_named(frame_num, jzfb.par.frame_num, int, 0644);

// JZFB_SET_ROTATE: an int, passed to the set_rotate of the panel.
#define JZFB_SET_ROTATE		_IOW('F', 0x62, int)

static const char slcd_names[28][9] = {
	"SLCD_d0", "SLCD_d1", "SLCD_d2", "SLCD_d3", "SLCD_d4", "SLCD_d5",
	"SLCD_d6", "SLCD_d7", "SLCD_d8", "SLCD_d9", "SLCD_d10", "SLCD_d11",
	"SLCD_d12", "SLCD_d13", "SLCD_d14", "SLCD_d15",
	[23] = "SLCD_cs", [25] = "SLCD_dc", [26] = "SLCD_wr", [27] = "SLCD_te",
};

static const char tft_names[28][10] = {
	"TFT_d0", "TFT_d1", "TFT_d2", "TFT_d3", "TFT_d4", "TFT_d5", "TFT_d6",
	"TFT_d7", "TFT_d8", "TFT_d9", "TFT_d10", "TFT_d11", "TFT_d12",
	"TFT_d13", "TFT_d14", "TFT_d15", "TFT_d16", "TFT_d17", "TFT_d18",
	"TFT_d19", "TFT_d20", "TFT_d21", "TFT_d22", "TFT_d23", "TFT_pclk",
	"TFT_vsync", "TFT_hsync", "TFT_de",
};

// Exported.
int jzfb_register_lcd(struct lcdc_data *pdata);
void jzfb_unregister_lcd(struct lcdc_data *pdata);
int slcd_read_data_only(int cmd, int count, unsigned char *buf);

// Not static in the vendor module; kept so.
void cpm_write(unsigned int off, unsigned int val);
unsigned int cpm_read(unsigned int off);
void lcd_write(unsigned int off, unsigned int val);
unsigned int lcd_read(unsigned int off);
void lcd_set_bit(unsigned int off, unsigned int start, unsigned int end, unsigned int val);
unsigned int lcd_get_bit(unsigned int off, unsigned int start, unsigned int end);
void lcd_genernal_stop_display(void);
void lcd_quick_stop_display(void);
void lcd_start_simple_read(void);
void lcdc_alloc_desc(void);
void lcdc_srdma_init(void);
void lcdc_pan_display(struct jzfb_par *par, unsigned int frame);

// --- register helpers -------------------------------------------------------

void cpm_write(unsigned int off, unsigned int val)
{
	writel(val, CPM_REG(off));
}

unsigned int cpm_read(unsigned int off)
{
	return readl(CPM_REG(off));
}

void lcd_write(unsigned int off, unsigned int val)
{
	writel(val, LCDC_REG(off));
}

unsigned int lcd_read(unsigned int off)
{
	return readl(LCDC_REG(off));
}

// The mask of bits start to end (inclusive). Shift counts are taken modulo
// 32, as the vendor module's: bits 0 to 31 give an empty mask.
static unsigned int lcd_bit_mask(unsigned int start, unsigned int end)
{
	return ((1u << ((end - start + 1) & 31)) - 1) << (start & 31);
}

// Bits start to end of the register to val.
void lcd_set_bit(unsigned int off, unsigned int start, unsigned int end, unsigned int val)
{
	unsigned int v = lcd_read(off);
	unsigned int mask = lcd_bit_mask(start, end);

	lcd_write(off, (v & ~mask) | ((val << (start & 31)) & mask));
}

unsigned int lcd_get_bit(unsigned int off, unsigned int start, unsigned int end)
{
	return (lcd_read(off) & lcd_bit_mask(start, end)) >> (start & 31);
}

void lcd_genernal_stop_display(void)
{
	lcd_write(LCDC_CTRL, LCDC_CTRL_GEN_STOP);
}

void lcd_quick_stop_display(void)
{
	lcd_write(LCDC_CTRL, LCDC_CTRL_QUICK_STOP);
}

void lcd_start_simple_read(void)
{
	lcd_write(LCDC_FRM_START, 1);
}

// --- smart panel bus --------------------------------------------------------

// 1 if the bus is still busy after us microseconds.
static int slcd_wait_busy_us(unsigned int us)
{
	u64 start = local_clock_us();

	while (readl(LCDC_REG(LCDC_SLCD_ST)) & SLCD_ST_BUSY) {
		if (local_clock_us() - start >= us)
			return 1;
		usleep_range(500, 500);
	}
	return 0;
}

// Nonzero if the bus is still busy after SLCD_BUSY_TRIES reads.
static int slcd_wait_busy(void)
{
	unsigned int count = SLCD_BUSY_TRIES;
	int busy = readl(LCDC_REG(LCDC_SLCD_ST)) & SLCD_ST_BUSY;

	while (--count && busy)
		busy = readl(LCDC_REG(LCDC_SLCD_ST)) & SLCD_ST_BUSY;
	return busy;
}

static void slcd_send_word(u32 kind, unsigned int value)
{
	if (slcd_wait_busy())
		panic("lcdc busy\n");
	writel(kind | (value & SLCD_FIFO_VALUE), LCDC_REG(LCDC_SLCD_FIFO));
	if (spi_send_delay)
		udelay(spi_send_delay);
}

static void slcd_send_cmd(unsigned int cmd)
{
	slcd_send_word(SLCD_FIFO_CMD, cmd);
}

static void slcd_send_data(unsigned int data)
{
	slcd_send_word(SLCD_FIFO_DATA, data);
}

// DC to the command (cmd 1) or data (cmd 0) level, WR idle; then the RD pin
// idle.
static int slcd_init_gpio_status(int cmd, struct lcdc_data *pdata)
{
	int low = pdata->dc_level == 1 ? !cmd : cmd;

	gpio_port_set_func(GPIO_PORT_A, BIT(SLCD_PIN_DC), low ? GPIO_OUTPUT0 : GPIO_OUTPUT1);
	gpio_port_set_func(GPIO_PORT_A, BIT(SLCD_PIN_WR),
			   pdata->wr_level == 1 ? GPIO_OUTPUT1 : GPIO_OUTPUT0);
	if (pdata->rd_gpio <= 0) {
		printk(KERN_ERR "please set rd gpio\n");
		return -1;
	}
	gpio_set_func(pdata->rd_gpio, pdata->rd_level == 1 ? GPIO_OUTPUT1 : GPIO_OUTPUT0);
	udelay(1);
	return 0;
}

// Sends the command byte cmd on PA0-PA7 and reads count bytes into buf, one
// RD pulse each. The pins go back to the controller afterwards.
int slcd_read_data_only(int cmd, int count, unsigned char *buf)
{
	struct lcdc_data *pdata;
	unsigned char *p;
	int i;

	if (jzfb.enabled && slcd_wait_busy_us(SLCD_BUSY_READ_US)) {
		printk(KERN_ERR "slcd is busy\n");
		return -1;
	}
	if (slcd_init_gpio_status(1, jzfb.pdata) < 0) {
		printk(KERN_ERR "not set rd gpio\n");
		return -1;
	}

	pdata = jzfb.pdata;
	gpio_port_set_func(GPIO_PORT_A, 0xff, GPIO_OUTPUT0);
	gpio_port_set_func(GPIO_PORT_A, BIT(SLCD_PIN_WR),
			   pdata->wr_level == 1 ? GPIO_OUTPUT0 : GPIO_OUTPUT1);
	udelay(1);
	for (i = 0; i < 8; i++)
		gpio_set_value(i, (cmd >> i) & 1);
	gpio_port_set_func(GPIO_PORT_A, BIT(SLCD_PIN_WR),
			   pdata->wr_level == 1 ? GPIO_OUTPUT1 : GPIO_OUTPUT0);
	udelay(1);
	slcd_init_gpio_status(0, jzfb.pdata);

	for (p = buf; p - buf < count; p++) {
		unsigned int v = 0;

		pdata = jzfb.pdata;
		gpio_port_set_func(GPIO_PORT_A, 0xff, GPIO_INPUT);
		gpio_set_func(pdata->rd_gpio, pdata->rd_level == 1 ? GPIO_OUTPUT0 : GPIO_OUTPUT1);
		udelay(1);
		for (i = 0; i < 8; i++)
			v |= gpio_get_value(i) << i;
		gpio_set_func(pdata->rd_gpio, pdata->rd_level == 1 ? GPIO_OUTPUT1 : GPIO_OUTPUT0);
		udelay(1);
		*p = v;
	}

	gpio_port_set_func(GPIO_PORT_A, 0xff | BIT(SLCD_PIN_DC) | BIT(SLCD_PIN_WR), SLCD_PIN_FUNC);
	return 0;
}
EXPORT_SYMBOL(slcd_read_data_only);

// --- pins -------------------------------------------------------------------

static void jzfb_release_pins(void)
{
	int i;

	for (i = 0; i < 32; i++)
		if (m_pins & BIT(i))
			gpio_free(i);
	m_pins = 0;
}

static int tft_request_pins(u32 pins)
{
	int i, ret;

	for (i = 0; i < 32; i++) {
		if (!(pins & BIT(i)))
			continue;
		ret = gpio_request(i, tft_names[i]);
		if (ret) {
			printk(KERN_ERR "jzfb: failed to request GPIO_PA%d\n", i);
			jzfb_release_pins();
			return ret;
		}
		m_pins |= BIT(i);
	}
	return 0;
}

static void tft_init_pins(u32 pins)
{
	if (!tft_request_pins(pins))
		gpio_port_set_func(GPIO_PORT_A, pins, TFT_PIN_FUNC);
}

static int slcd_init_gpio(u32 pins, int rdy, int te)
{
	int i, ret;

	if (rdy) {
		printk(KERN_ERR "jzfb: x1600 no rdy pin\n");
		return -ENODEV;
	}
	if (te)
		pins |= BIT(SLCD_PIN_TE);
	pins |= BIT(SLCD_PIN_DC) | BIT(SLCD_PIN_WR);

	for (i = 0; i < 32; i++) {
		if (!(pins & BIT(i)))
			continue;
		ret = gpio_request(i, slcd_names[i]);
		if (ret) {
			printk(KERN_ERR "jzfb: failed to request GPIO_PA%d\n", i);
			jzfb_release_pins();
			return ret;
		}
		m_pins |= BIT(i);
	}
	gpio_port_set_func(GPIO_PORT_A, pins, SLCD_PIN_FUNC);
	return 0;
}

// The wider of the two smart panel bus widths.
static unsigned int slcd_bus_width(struct lcdc_data *pdata)
{
	return max(pdata->data_width, pdata->cmd_width);
}

static void jzfb_init_gpio(struct lcdc_data *pdata)
{
	unsigned int of = pdata->out_format;
	int rdy = pdata->rdy_used & 1;
	int te = pdata->te_mode == 2;

	switch (pdata->lcd_mode) {
	case LCD_MODE_TFT_24BIT:
		if (of == OUT_RGB444)
			tft_init_pins(0x0ff0f0f0);
		if (of == OUT_RGB555)
			tft_init_pins(0x0ff8f8f8);
		if (of == OUT_RGB565)
			tft_init_pins(0x0ff8fcf8);
		if (of == OUT_RGB666)
			tft_init_pins(0x0ffcfcfc);
		if (of == OUT_RGB888)
			tft_init_pins(0x0fffffff);
		break;
	case LCD_MODE_TFT_18BIT:
		if (of == OUT_RGB666)
			tft_init_pins(0x0f03ffff);
		else
			printk(KERN_ERR "lcdc: tft 18bits only support out_format_rgb666!\n");
		break;
	case LCD_MODE_TFT_8BIT:
	case LCD_MODE_TFT_8BIT_B:
		if (of == OUT_RGB888)
			tft_init_pins(0x0f0000ff);
		if (of == OUT_RGB666)
			tft_init_pins(0x0f0000fc);
		break;
	case LCD_MODE_SLCD_6800:
	case LCD_MODE_SLCD_8080:
		switch (slcd_bus_width(pdata)) {
		case SLCD_WIDTH_8:
			slcd_init_gpio(0xff, rdy, te);
			break;
		case SLCD_WIDTH_9:
			slcd_init_gpio(0x1ff, rdy, te);
			break;
		case SLCD_WIDTH_16:
			slcd_init_gpio(0xffff, rdy, te);
			break;
		}
		break;
	case LCD_MODE_SLCD_SPI_3:
	case LCD_MODE_SLCD_SPI_4:
		slcd_init_gpio(BIT(0), rdy, te);
		break;
	default:
		printk(KERN_ERR "This mode not implemented(init_gpio): %d\n", pdata->lcd_mode);
		break;
	}
}

// The pixel clock (or the smart panel lines) driven low as a GPIO, and the
// pins given back.
static void jzfb_deinit_gpio(struct lcdc_data *pdata)
{
	int i;

	switch (pdata->lcd_mode) {
	case LCD_MODE_TFT_24BIT:
		gpio_set_func(GPIO_PA(TFT_PIN_PCLK), GPIO_OUTPUT0);
		jzfb_release_pins();
		break;
	case LCD_MODE_TFT_8BIT:
	case LCD_MODE_TFT_8BIT_B:
		if (pdata->out_format == OUT_RGB888 || pdata->out_format == OUT_RGB666) {
			gpio_set_func(GPIO_PA(TFT_PIN_PCLK), GPIO_OUTPUT0);
			jzfb_release_pins();
		}
		break;
	case LCD_MODE_SLCD_6800:
	case LCD_MODE_SLCD_8080:
		switch (slcd_bus_width(pdata)) {
		case SLCD_WIDTH_8:
			for (i = 0; i < 8; i++)
				gpio_set_func(GPIO_PA(i), GPIO_OUTPUT0);
			for (i = 24; i < 28; i++)
				gpio_set_func(GPIO_PA(i), GPIO_OUTPUT0);
			jzfb_release_pins();
			break;
		case SLCD_WIDTH_9:
		case SLCD_WIDTH_16:
			gpio_set_func(GPIO_PA(SLCD_PIN_TE), GPIO_OUTPUT0);
			jzfb_release_pins();
			break;
		}
		break;
	default:
		printk(KERN_ERR "This mode not implemented(deinit_gpio): %d \n", pdata->lcd_mode);
		break;
	}
}

// --- clocks -----------------------------------------------------------------

// Bus cycles a pixel takes on a smart panel; the clock is 2 * cycles + 1
// times the pixel rate.
static unsigned int slcd_pixclock_cycle(struct lcdc_data *pdata)
{
	switch (pdata->data_width) {
	case SLCD_WIDTH_8:
		if (pdata->out_format == OUT_RGB565)
			return 2;
		if (pdata->out_format == OUT_RGB888)
			return 3;
		break;
	case SLCD_WIDTH_9:
		return 2;
	case SLCD_WIDTH_16:
		return 1;
	}
	__assert("cycle", __FILE__, __LINE__, __func__);
	return 0;
}

// Bits a pixel takes on an SPI panel; the clock is twice that times the
// pixel rate.
static unsigned int slcd_spi_pixclock_cycle(struct lcdc_data *pdata)
{
	switch (pdata->data_width) {
	case SLCD_WIDTH_8:
		if (pdata->out_format == OUT_RGB565)
			return 16;
		if (pdata->out_format == OUT_RGB888)
			return 24;
		break;
	case SLCD_WIDTH_9:
		if (pdata->out_format == OUT_RGB666)
			return 24;
		break;
	case SLCD_WIDTH_16:
		return 16;
	}
	__assert("cycle", __FILE__, __LINE__, __func__);
	return 0;
}

// pixclock (and slcd_init_pixclock for smart panels) when the panel leaves
// them at 0.
static void auto_calculate_pixel_clock(struct lcdc_data *pdata)
{
	unsigned int h, v, clk;

	if (!pdata->refresh)
		pdata->refresh = 40;

	if (LCD_MODE_IS_TFT(pdata->lcd_mode)) {
		h = pdata->hsync_len + pdata->left_margin + pdata->xres + pdata->right_margin;
		v = pdata->vsync_len + pdata->upper_margin + pdata->yres + pdata->lower_margin;
		if (pdata->lcd_mode == LCD_MODE_TFT_8BIT)
			clk = ((v - pdata->yres) * h + (2 * pdata->xres + h) * pdata->yres) *
			      pdata->refresh;
		else
			clk = h * v * pdata->refresh;
		if (!pdata->pixclock)
			pdata->pixclock = clk;
		return;
	}

	if (!pdata->pixclock) {
		pdata->pixclock = pdata->xres * pdata->yres * pdata->refresh;
		if (pdata->lcd_mode >= LCD_MODE_SLCD_SPI_3)
			pdata->pixclock *= 2 * slcd_spi_pixclock_cycle(pdata);
		else
			pdata->pixclock *= 2 * slcd_pixclock_cycle(pdata) + 1;
	}
	if (!pdata->slcd_init_pixclock)
		pdata->slcd_init_pixclock = pdata->xres * pdata->yres * 3;
}

// Microseconds an SPI word takes at pixclock, rounded up.
static int calculate_spi_mode_delay_time(struct lcdc_data *pdata)
{
	int bits, t;

	switch (pdata->data_width) {
	case SLCD_WIDTH_8:
		bits = 8;
		break;
	case SLCD_WIDTH_9:
		bits = 9;
		break;
	case SLCD_WIDTH_16:
		bits = 16;
		break;
	default:
		__assert("cycle", __FILE__, __LINE__, __func__);
		bits = 0;
		break;
	}
	t = bits * 1000000;
	return t / (int)pdata->pixclock + (t % (int)pdata->pixclock ? 1 : 0);
}

// --- the controller ---------------------------------------------------------

// Interrupts, status and the common configuration: the interface, and
// dithering for 32-bit frames on a narrower output.
static void lcdc_init_common(struct lcdc_data *pdata)
{
	unsigned int dither_en = 0, dither = 0;
	u32 v;

	writel(LCDC_ST_FRM_DONE | LCDC_ST_UNDERRUN | LCDC_ST_STOPPED, LCDC_REG(LCDC_INT_EN));
	writel(readl(LCDC_REG(LCDC_ST)), LCDC_REG(LCDC_CLR_ST));
	writel(readl(LCDC_REG(LCDC_CFG2)) | 0x30, LCDC_REG(LCDC_CFG2));

	if (pdata->fb_fmt == FB_FMT_RGB888 || pdata->fb_fmt == FB_FMT_ARGB8888) {
		switch (pdata->out_format) {
		case OUT_RGB888:
			break;
		case OUT_RGB444:
			dither_en = 1;
			dither = 0x3f;
			break;
		case OUT_RGB555:
			dither_en = 1;
			dither = 0x2a;
			break;
		case OUT_RGB565:
			dither_en = 1;
			dither = 0x26;
			break;
		case OUT_RGB666:
			dither_en = 1;
			dither = 0x15;
			break;
		default:
			dither_en = 1;
			break;
		}
	}

	v = readl(LCDC_REG(LCDC_COM_CFG));
	v = (v & ~COM_CFG_DITHER_EN) | (dither_en ? COM_CFG_DITHER_EN : 0);
	v = (v & ~COM_CFG_DITHER_MASK) | ((dither << COM_CFG_DITHER_SHIFT) & COM_CFG_DITHER_MASK);
	v |= COM_CFG_BIT6 | COM_CFG_BIT5;
	v = (v & ~COM_CFG_IF_MASK) | (LCD_MODE_IS_TFT(pdata->lcd_mode) ? 1 : 2);
	writel(v, LCDC_REG(LCDC_COM_CFG));
}

static u32 timing(unsigned int start, unsigned int end)
{
	return ((start << 16) & (TIMING_MASK << 16)) | (end & TIMING_MASK);
}

static void lcdc_init_tft(struct lcdc_data *pdata)
{
	unsigned int hde = pdata->hsync_len + pdata->left_margin;
	unsigned int vde = pdata->vsync_len + pdata->upper_margin;
	u32 v;

	writel(timing(pdata->hsync_len, hde + pdata->xres + pdata->right_margin),
	       LCDC_REG(LCDC_TFT_HSYNC));
	writel(timing(pdata->vsync_len, vde + pdata->yres + pdata->lower_margin),
	       LCDC_REG(LCDC_TFT_VSYNC));
	writel(timing(hde, hde + pdata->xres), LCDC_REG(LCDC_TFT_HDE));
	writel(timing(vde, vde + pdata->yres), LCDC_REG(LCDC_TFT_VDE));

	v = 0;
	if (!pdata->pix_clk_active)
		v |= BIT(10);
	if (!pdata->de_active_level)
		v |= BIT(9);
	if (!pdata->vsync_active_level)
		v |= BIT(7);
	if (!pdata->hsync_active_level)
		v |= BIT(8);
	v |= (pdata->color_even << 19) & (0x7 << 19);
	v |= (pdata->color_odd << 16) & (0x7 << 16);
	v |= pdata->lcd_mode & 0x7;
	writel(v, LCDC_REG(LCDC_TFT_CFG));

	v = readl(CPM_REG(CPM_LPCDR));
	v = (v & ~BIT(26)) | (pdata->lpcdr_bit26 == 1 ? BIT(26) : 0);
	writel(v, CPM_REG(CPM_LPCDR));
}

static void lcdc_init_slcd(struct lcdc_data *pdata)
{
	unsigned int mode, of;
	u32 v;

	switch (pdata->lcd_mode) {
	case LCD_MODE_SLCD_6800:
		mode = 1;
		break;
	case LCD_MODE_SLCD_8080:
		mode = 2;
		break;
	case LCD_MODE_SLCD_SPI_3:
		mode = 4;
		break;
	case LCD_MODE_SLCD_SPI_4:
		mode = 5;
		break;
	default:
		mode = 2;
		break;
	}

	of = pdata->out_format;
	if (of == OUT_RGB444)
		of = 1;
	else if (of == OUT_RGB555)
		panic("slcd outformat can't be 555\n");

	v = (mode << 23) & (0x7 << 23);
	v |= (of << 21) & (0x3 << 21);
	v |= BIT(20);
	if (pdata->te_mode == 2)
		v |= BIT(18);
	if (pdata->rdy_used & 1)
		v |= BIT(17);
	if (pdata->slcd_bit10 == 1)
		v |= BIT(10);
	if (pdata->dc_level == 1)
		v |= BIT(9);
	if (pdata->wr_level == 1)
		v |= BIT(8);
	if (pdata->slcd_bit6 == 1)
		v |= BIT(6);
	v |= (pdata->data_width << 3) & (0x7 << 3);
	v |= pdata->cmd_width & 0x7;
	writel(v, LCDC_REG(LCDC_SLCD_CFG));
	writel(0, LCDC_REG(LCDC_SLCD_WR_DUTY));
	writel(0, LCDC_REG(LCDC_SLCD_TIMING));
	writel((pdata->yres << 16) | (pdata->xres & 0xffff), LCDC_REG(LCDC_SLCD_FRM_SIZE));
	writel(0, LCDC_REG(LCDC_SLCD_SLOW_TIME));
}

// The set-up commands of a smart panel.
static void slcd_send_init_cmds(struct lcdc_data *pdata)
{
	const struct slcd_cmd *cmd = pdata->init_cmds;
	unsigned int count = pdata->init_cmd_count;
	unsigned int i;

	for (i = 0; i < count; i++, cmd++) {
		switch (cmd->type) {
		case SLCD_CMD_COMMAND:
			slcd_send_cmd(cmd->value);
			break;
		case SLCD_CMD_DATA:
			slcd_send_data(cmd->value);
			break;
		case SLCD_CMD_SLEEP_US:
			usleep_range(cmd->value, cmd->value);
			break;
		default:
			panic("why this type: %d\n", cmd->type);
		}
	}
	if (slcd_wait_busy())
		panic("lcdc busy\n");
}

// Called with jzfb.lock held.
static void enable_fb(void)
{
	struct lcdc_data *pdata;
	unsigned long rate;

	if (jzfb.enabled++)
		return;

	auto_calculate_pixel_clock(jzfb.pdata);
	pdata = jzfb.pdata;
	rate = pdata->pixclock;
	if (!LCD_MODE_IS_TFT(pdata->lcd_mode) && pdata->slcd_init_pixclock)
		rate = pdata->slcd_init_pixclock;
	clk_prepare_enable(jzfb.clk);
	clk_prepare_enable(jzfb.pclk);
	clk_set_rate(jzfb.pclk, rate);

	pdata = jzfb.pdata;
	if (pdata->lcd_mode < LCD_MODE_SLCD_SPI_3)
		spi_send_delay = 0;
	else
		spi_send_delay = calculate_spi_mode_delay_time(pdata);

	pdata = jzfb.pdata;
	jzfb_init_gpio(pdata);
	lcdc_init_common(pdata);
	if (LCD_MODE_IS_TFT(pdata->lcd_mode))
		lcdc_init_tft(pdata);
	else
		lcdc_init_slcd(pdata);

	jzfb.state = JZFB_IDLE;
	if (lcd_is_inited)
		return;

	jzfb.pdata->power_on(NULL);
	slcd_send_init_cmds(jzfb.pdata);

	pdata = jzfb.pdata;
	if (pdata->pixclock != rate)
		clk_set_rate(jzfb.pclk, pdata->pixclock);
	if (!LCD_MODE_IS_TFT(pdata->lcd_mode)) {
		slcd_send_cmd(pdata->write_mem_cmd);
		writel(readl(LCDC_REG(LCDC_SLCD_CFG)) | SLCD_CFG_FRM_EN, LCDC_REG(LCDC_SLCD_CFG));
	}
}

// Called with jzfb.lock held.
static void disable_fb(void)
{
	if (--jzfb.enabled)
		return;

	jzfb.is_stopping = 1;
	writel(LCDC_CTRL_GEN_STOP, LCDC_REG(LCDC_CTRL));
	while (READ_ONCE(jzfb.is_stopping)) {
		if (++is_wait_stop_cnt >= STOP_WAIT_MS) {
			printk("is_wait_stop_cnt >= 100\n");
			break;
		}
		msleep(1);
	}
	is_wait_stop_cnt = 0;

	jzfb.pdata->power_off(NULL);
	jzfb_deinit_gpio(jzfb.pdata);
	clk_disable_unprepare(jzfb.clk);
	clk_disable_unprepare(jzfb.pclk);
}

static irqreturn_t jzfb_irq_handler(int irq, void *dev_id)
{
	unsigned long st = readl(LCDC_REG(LCDC_ST));

	if (st & LCDC_ST_FRM_DONE) {
		writel(LCDC_ST_FRM_DONE, LCDC_REG(LCDC_CLR_ST));
		jzfb.state = JZFB_FRAME_DONE;
		wake_up_all(&jzfb.wait_queue);
	} else if (st & LCDC_ST_STOPPED) {
		writel(LCDC_ST_STOPPED, LCDC_REG(LCDC_CLR_ST));
		jzfb.state = JZFB_STOPPED;
		jzfb.is_stopping = 0;
		wake_up_all(&jzfb.wait_queue);
	} else if (st & LCDC_ST_UNDERRUN) {
		printk(KERN_ERR "err: lcd underrun\n");
		writel(LCDC_ST_UNDERRUN, LCDC_REG(LCDC_CLR_ST));
	} else {
		printk(KERN_ERR "lcd: why this irq: %lx\n", st);
		writel(st, LCDC_REG(LCDC_CLR_ST));
	}
	return IRQ_HANDLED;
}

// --- descriptors and panning ------------------------------------------------

static unsigned long m_dma_alloc_coherent(int size, int align)
{
	unsigned long mem = rmem_alloc_aligned(size, align);

	JZFB_ASSERT(mem);
	return mem;
}

void lcdc_alloc_desc(void)
{
	int i;

	for (i = 0; i < 2; i++)
		jzfb.desc[i] = (struct lcdc_desc *)m_dma_alloc_coherent(DESC_SIZE, DESC_SIZE);
}

void lcdc_srdma_init(void)
{
	struct jzfb_srdma *s = &jzfb.par.srdma;
	struct lcdc_desc *desc;
	unsigned int fmt;
	int i;

	for (i = 0; i < 2; i++) {
		switch (s->fb_fmt) {
		case FB_FMT_RGB555:
			fmt = 0;
			break;
		case FB_FMT_RGB565:
			fmt = 2;
			break;
		case FB_FMT_RGB888:
		case FB_FMT_ARGB8888:
			fmt = 4;
			break;
		default:
			panic("format err:%d\n", s->fb_fmt);
		}

		desc = jzfb.desc[i];
		desc->next = virt_to_phys(desc);
		desc->cfg = ((fmt << DESC_CFG_FMT_SHIFT) & DESC_CFG_FMT_MASK) |
			    (s->is_tft ? 0 : DESC_CFG_SLCD);
		desc->ctrl = DESC_CTRL_BIT17;
		desc->stride = s->xres;
		desc->addr = virt_to_phys((void *)s->mem);
		dma_cache_sync(NULL, desc, DESC_SYNC_LEN, DMA_TO_DEVICE);
	}
}

// Shows frame: the next descriptor gets its address and is started; a
// started fetch is waited for first on smart panels and afterwards on RGB
// ones.
void lcdc_pan_display(struct jzfb_par *par, unsigned int frame)
{
	struct lcdc_desc *desc;
	unsigned long flags;
	int ret = 0;

	iob();			// the frame writes out of the write buffer
	mutex_lock(&jzfb.lock);
	jzfb.frame_idx++;
	desc = jzfb.desc[jzfb.frame_idx % 2];
	desc->addr = virt_to_phys((void *)(par->srdma.mem + frame * par->m.frame_size));

	if (LCD_MODE_IS_TFT(jzfb.pdata->lcd_mode)) {
		spin_lock_irqsave(&jzfb.irq_lock, flags);
		jzfb.state = JZFB_FETCHING;
		writel(virt_to_phys(desc), LCDC_REG(LCDC_FRM_DESC));
		writel(1, LCDC_REG(LCDC_FRM_START));
		spin_unlock_irqrestore(&jzfb.irq_lock, flags);
		wait_event_timeout(jzfb.wait_queue, jzfb.state == JZFB_FRAME_DONE, JZFB_WAIT_JIFFIES);
		if (jzfb.state != JZFB_FRAME_DONE) {
			printk(KERN_EMERG "jzfb: tft pan display wait timeout %d\n", jzfb.state);
			ret = -1;
		}
	} else {
		if (jzfb.state != JZFB_IDLE) {
			wait_event_timeout(jzfb.wait_queue, jzfb.state == JZFB_FRAME_DONE,
					   JZFB_WAIT_JIFFIES);
			if (jzfb.state != JZFB_FRAME_DONE) {
				printk(KERN_EMERG "jzfb: slcd pan display wait timeout\n");
				ret = -1;
				goto out;
			}
		}
		slcd_wait_busy_us(SLCD_BUSY_PAN_US);
		jzfb.state = JZFB_FETCHING;
		writel(virt_to_phys(desc), LCDC_REG(LCDC_FRM_DESC));
		writel(1, LCDC_REG(LCDC_FRM_START));
	}
out:
	mutex_unlock(&jzfb.lock);

	if (!ret && jzfb.pan_display_sync) {
		wait_event_timeout(jzfb.wait_queue, jzfb.state == JZFB_FRAME_DONE, JZFB_WAIT_JIFFIES);
		if (jzfb.state != JZFB_FRAME_DONE)
			printk(KERN_EMERG "jzfb: sync lcd pan display wait timeout\n");
	}
}

// --- fb ops -----------------------------------------------------------------

static void init_var_info(struct jzfb_mem *m, struct fb_var_screeninfo *var)
{
	struct lcdc_data *pdata = jzfb.pdata;

	var->xres = m->xres;
	var->yres = m->yres;
	var->xres_virtual = m->xres;
	var->yres_virtual = m->yres * m->frame_num;
	var->xoffset = 0;
	var->yoffset = 0;
	var->height = pdata->height_mm;
	var->width = pdata->width_mm;
	var->pixclock = pdata->pixclock;
	var->left_margin = pdata->left_margin;
	var->right_margin = pdata->right_margin;
	var->upper_margin = pdata->upper_margin;
	var->lower_margin = pdata->lower_margin;
	var->hsync_len = pdata->hsync_len;
	var->vsync_len = pdata->vsync_len;
	var->sync = 0;
	var->vmode = 0;
	var->bits_per_pixel = pdata->fb_fmt == FB_FMT_RGB888 ||
			      pdata->fb_fmt == FB_FMT_ARGB8888 ? 32 : 16;

	switch (pdata->fb_fmt) {
	case FB_FMT_RGB555:
		var->red = (struct fb_bitfield){ 10, 5, 0 };
		var->green = (struct fb_bitfield){ 5, 5, 0 };
		var->blue = (struct fb_bitfield){ 0, 5, 0 };
		var->transp = (struct fb_bitfield){ 0, 0, 0 };
		break;
	case FB_FMT_RGB565:
		var->red = (struct fb_bitfield){ 11, 5, 0 };
		var->green = (struct fb_bitfield){ 5, 6, 0 };
		var->blue = (struct fb_bitfield){ 0, 5, 0 };
		var->transp = (struct fb_bitfield){ 0, 0, 0 };
		break;
	case FB_FMT_RGB888:
		var->red = (struct fb_bitfield){ 16, 8, 0 };
		var->green = (struct fb_bitfield){ 8, 8, 0 };
		var->blue = (struct fb_bitfield){ 0, 8, 0 };
		var->transp = (struct fb_bitfield){ 0, 0, 0 };
		break;
	case FB_FMT_ARGB8888:
		var->red = (struct fb_bitfield){ 16, 8, 0 };
		var->green = (struct fb_bitfield){ 8, 8, 0 };
		var->blue = (struct fb_bitfield){ 0, 8, 0 };
		var->transp = (struct fb_bitfield){ 24, 8, 0 };
		break;
	default:
		JZFB_ASSERT(0);
		break;
	}
}

// Only the registered resolution; the rest of var is set to the panel.
static int jzfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct jzfb_par *par = info->par;

	if (var->xres != par->m.xres || var->yres != par->m.yres)
		return -EINVAL;
	init_var_info(&par->m, var);
	return 0;
}

static int jzfb_set_par(struct fb_info *info)
{
	return jzfb_check_var(&info->var, info);
}

static int jzfb_open(struct fb_info *info, int user)
{
	mutex_lock(&jzfb.lock);
	jzfb.is_open++;
	mutex_unlock(&jzfb.lock);
	return 0;
}

static int jzfb_release(struct fb_info *info, int user)
{
	mutex_lock(&jzfb.lock);
	jzfb.is_open--;
	mutex_unlock(&jzfb.lock);
	return 0;
}

static int jzfb_blank(int blank_mode, struct fb_info *info)
{
	struct jzfb_par *par = info->par;

	mutex_lock(&jzfb.lock);
	if (blank_mode == FB_BLANK_UNBLANK) {
		if (!par->enabled) {
			enable_fb();
			par->enabled = 1;
		}
	} else if (par->enabled) {
		disable_fb();
		par->enabled = 0;
	}
	mutex_unlock(&jzfb.lock);
	return 0;
}

static int jzfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct jzfb_par *par = info->par;
	unsigned int frame;

	if (var->xoffset != info->var.xoffset) {
		printk(KERN_ERR "jzfb: No support for X panning for now\n");
		return -EINVAL;
	}
	frame = var->yoffset / var->yres;
	if (frame >= par->m.frame_num) {
		printk(KERN_ERR "jzfb: yoffset is out of framebuffer: %d\n", var->yoffset);
		return -EINVAL;
	}
	if (!par->enabled)
		return -EBUSY;

	lcdc_pan_display(par, frame);
	return 0;
}

// Any other command does nothing and returns 0.
static int jzfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int (*set_rotate)(u16 rotate);
	int value;

	if (cmd != JZFB_SET_ROTATE)
		return 0;

	if (copy_from_user(&value, (void __user *)arg, sizeof(value)))
		return -EFAULT;
	display_rotate_set = value;
	if (display_rotate == (u16)value)
		return 0;
	set_rotate = jzfb.pdata->set_rotate;
	if (set_rotate && !set_rotate(value))
		display_rotate = display_rotate_set;
	return 0;
}

// The frames, write-through cached.
static int jzfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct jzfb_par *par = info->par;
	struct fb_info *fb = par->fb;
	unsigned long len = fb->fix.smem_len;
	unsigned long off, size;

	if (!len)
		return -ENOMEM;
	off = vma->vm_pgoff << PAGE_SHIFT;
	size = vma->vm_end - vma->vm_start;
	if (off + size > PAGE_ALIGN(len))
		return -EINVAL;

	off += fb->fix.smem_start & PAGE_MASK;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_WT_WA;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static struct fb_ops jzfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = jzfb_open,
	.fb_release = jzfb_release,
	.fb_check_var = jzfb_check_var,
	.fb_set_par = jzfb_set_par,
	.fb_blank = jzfb_blank,
	.fb_pan_display = jzfb_pan_display,
	.fb_ioctl = jzfb_ioctl,
	.fb_mmap = jzfb_mmap,
};

// --- registration -----------------------------------------------------------

// Nonzero for a smart panel bus and pixel format that do not go together.
static int check_scld_fmt(struct lcdc_data *pdata)
{
	if (LCD_MODE_IS_TFT(pdata->lcd_mode))
		return 0;
	if (pdata->lcd_mode > LCD_MODE_SLCD_SPI_4)
		return 1;

	switch (pdata->data_width) {
	case SLCD_WIDTH_8:
		return pdata->out_format != OUT_RGB565 && pdata->out_format != OUT_RGB888;
	case SLCD_WIDTH_9:
		return pdata->out_format != OUT_RGB666;
	case SLCD_WIDTH_16:
		return pdata->out_format != OUT_RGB565;
	}
	return 0;
}

static void init_one_fb(void)
{
	struct lcdc_data *pdata = jzfb.pdata;
	struct jzfb_par *par = &jzfb.par;
	struct jzfb_mem *m = &par->m;
	struct fb_videomode *mode = &par->mode;
	struct fb_info *fb;
	int ret;

	fb = framebuffer_alloc(0, NULL);
	JZFB_ASSERT(fb);
	fb->par = par;
	par->fb = fb;

	mode->name = pdata->name;
	mode->refresh = pdata->refresh;
	mode->xres = m->xres;
	mode->yres = m->yres;
	mode->pixclock = pdata->pixclock;
	mode->left_margin = pdata->left_margin;
	mode->right_margin = pdata->right_margin;
	mode->upper_margin = pdata->upper_margin;
	mode->lower_margin = pdata->lower_margin;
	mode->hsync_len = pdata->hsync_len;
	mode->vsync_len = pdata->vsync_len;
	mode->sync = 0;
	mode->vmode = 0;
	mode->flag = 0;
	fb_videomode_to_modelist(mode, 1, &fb->modelist);

	strcpy(fb->fix.id, "jzfb");
	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.xpanstep = 0;
	fb->fix.ypanstep = 1;
	fb->fix.ywrapstep = 0;
	fb->fix.line_length = m->line_length;
	fb->fix.smem_start = virt_to_phys((void *)m->vaddr);
	fb->fix.smem_len = m->frame_num * m->frame_size;
	fb->fix.mmio_start = 0;
	fb->fix.mmio_len = 0;
	fb->fix.accel = FB_ACCEL_NONE;
	init_var_info(m, &fb->var);

	fb->fbops = &jzfb_ops;
	fb->flags = FBINFO_DEFAULT;
	fb->screen_base = (void __iomem *)m->vaddr;
	fb->screen_size = m->frame_size;

	ret = register_framebuffer(fb);
	JZFB_ASSERT(!ret);
}

static void release_one_fb(void)
{
	int ret;

	ret = unregister_framebuffer(jzfb.par.fb);
	JZFB_ASSERT(!ret);
	framebuffer_release(jzfb.par.fb);
}

// The panel; the framebuffer appears with it. One at a time: a second one,
// or a description the module cannot drive, panics.
int jzfb_register_lcd(struct lcdc_data *pdata)
{
	struct jzfb_par *par = &jzfb.par;
	struct jzfb_mem *m = &par->m;
	unsigned int bytes, size;

	mutex_lock(&jzfb.lock);
	JZFB_CHECK(jzfb.pdata != NULL);
	JZFB_CHECK(pdata == NULL);
	JZFB_CHECK(pdata->name == NULL);
	JZFB_CHECK(pdata->power_on == NULL);
	JZFB_CHECK(pdata->power_off == NULL);
	JZFB_CHECK(pdata->xres < 32 || pdata->xres >= 2048);
	JZFB_CHECK(pdata->yres < 32 || pdata->yres >= 2048);
	JZFB_CHECK(pdata->fb_fmt > FB_FMT_ARGB8888);
	JZFB_CHECK(check_scld_fmt(pdata));

	auto_calculate_pixel_clock(pdata);
	jzfb.pdata = pdata;
	lcdc_alloc_desc();

	m->fb_fmt = pdata->fb_fmt;
	bytes = m->fb_fmt == FB_FMT_RGB888 || m->fb_fmt == FB_FMT_ARGB8888 ? 4 : 2;
	m->xres = pdata->xres;
	m->frame_num = par->frame_num;
	m->yres = pdata->yres;
	m->line_length = ALIGN(m->xres * bytes, 8);
	m->frame_size = m->yres * m->line_length;
	size = PAGE_ALIGN(m->frame_size * par->frame_num);
	par->mem = m_dma_alloc_coherent(size, PAGE_SIZE);
	par->mem_size = size;
	memset((void *)par->mem, 0, size);
	m->vaddr = par->mem;

	par->srdma.mem = m->vaddr;
	par->srdma.is_tft = LCD_MODE_IS_TFT(jzfb.pdata->lcd_mode);
	par->srdma.fb_fmt = m->fb_fmt;
	par->srdma.xres = m->xres;
	lcdc_srdma_init();

	init_one_fb();
	mutex_unlock(&jzfb.lock);
	return 0;
}
EXPORT_SYMBOL(jzfb_register_lcd);

// Takes the framebuffer down; it must not be open.
void jzfb_unregister_lcd(struct lcdc_data *pdata)
{
	mutex_lock(&jzfb.lock);
	JZFB_ASSERT(pdata == jzfb.pdata);
	JZFB_ASSERT(!jzfb.is_open);
	if (jzfb.enabled) {
		jzfb.enabled = 1;
		disable_fb();
	}
	if (jzfb.par.frame_num)
		rmem_free(jzfb.par.mem, ALIGN(jzfb.par.mem_size, 64));
	rmem_free((unsigned long)jzfb.desc[0], DESC_SIZE);
	rmem_free((unsigned long)jzfb.desc[1], DESC_SIZE);
	release_one_fb();
	jzfb_release_pins();
	jzfb.pdata = NULL;
	mutex_unlock(&jzfb.lock);
}
EXPORT_SYMBOL(jzfb_unregister_lcd);

// --- module -----------------------------------------------------------------

static int __init jzfb_module_init(void)
{
	int ret;

	mutex_init(&jzfb.lock);
	spin_lock_init(&jzfb.irq_lock);
	init_waitqueue_head(&jzfb.wait_queue);
	jzfb.irq = LCDC_IRQ;

	jzfb.clk = clk_get(NULL, "gate_lcd");
	JZFB_ASSERT(!IS_ERR(jzfb.clk));
	jzfb.pclk = clk_get(NULL, "div_lcd");
	JZFB_ASSERT(!IS_ERR(jzfb.pclk));

	ret = request_irq(jzfb.irq, jzfb_irq_handler, IRQF_SHARED, "jzfb", &jzfb);
	JZFB_ASSERT(!ret);
	return 0;
}
module_init(jzfb_module_init);

// Panics while a panel is registered.
static void __exit jzfb_module_exit(void)
{
	JZFB_ASSERT(!jzfb.pdata);
	clk_put(jzfb.clk);
	clk_put(jzfb.pclk);
	free_irq(jzfb.irq, &jzfb);
}
module_exit(jzfb_module_exit);

MODULE_DESCRIPTION("Ingenic Soc FB driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
