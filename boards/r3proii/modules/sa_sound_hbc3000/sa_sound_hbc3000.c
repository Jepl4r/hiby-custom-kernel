// SPDX-License-Identifier: GPL-2.0
//
// sa_sound_hbc3000 -- the HBC3000 of the HiBy R3 Pro II: a Gowin GW1N FPGA
// between the I2S controller and the CS43198 DACs. It makes the audio clocks
// from its two crystals (22.5792 and 24.576 MHz), is the I2S master, unpacks
// DoP into DSD for the DACs, and drives the analogue switches of the outputs
// on its GPO pins.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock sa_sound_hbc3000.sh passes them) and the same
// functions for the sound card:
//
//   hbc3000_enable()          powers the FPGA and loads its configuration
//   hbc3000_disable()         powers it off
//   hbc3000_start(), _stop()  the run line (TDO, used as reset)
//   hbc3000_set_samplerate()  clocks and mode for a rate: 0 PCM, 2 DoP
//   hbc3000_set_codec_type()  which DAC's clock table to use
//   hbc3000_set_spdif_mode()  the S/PDIF clock table instead
//   hbc3000_set_gpo()         one GPO pin
//   hbc3000_set_power_down()  the crystals and the module power-down bits
//
// The configuration is HiBy's and is not part of this source: it is loaded
// with request_firmware() from hbc3000.fw, the vendor module's FS_data array
// (34304 bytes) taken out of the stock firmware. It goes in over JTAG into
// the FPGA's SRAM, so it is lost whenever the FPGA loses power; the user code
// it sets tells whether it is already there.
//
// Once configured, the JTAG pins carry a 3-wire SPI: TDI data, TCK clock, TMS
// select, 12-bit words MSB first. Bits 11-10 pick the register:
//
//   0  clock word: bit 9 crystal (1: 24.576 MHz), bits 8-7 clock mode,
//      bits 6-3 bit clock divider, bits 2-0 mode (0 PCM, 1 S/PDIF, 2 DoP)
//   1  GPO group 0, one bit per pin
//   2  GPO group 1
//   3  misc: bit 9 module power-down, bits 8-7 crystal power-downs,
//      bit 6 set after loading, bit 5 clock output off
//
// Loaded after utils.ko, which provides the pin names such as "PA30".

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;

#define HBC3000_FIRMWARE	"hbc3000.fw"

// JTAG IDCODEs of the FPGAs the configuration is made for.
#define IDCODE_GW1N_1		0x0900281b
#define IDCODE_GW1N_4		0x0100381b
#define IDCODE_GW1N_9C		0x1100481b	// bit 12 ignored

// Gowin JTAG instructions.
#define JTAG_IDCODE		0x11
#define JTAG_USERCODE		0x13
#define JTAG_CONFIG_ENABLE	0x15
#define JTAG_ERASE_SRAM		0x05
#define JTAG_CONFIG_DISABLE	0x3a
#define JTAG_REPROGRAM		0x3c
#define JTAG_WRITE_SRAM		0x17
#define JTAG_STATUS		0x41
#define JTAG_NOOP		0x02

// SPI registers, bits 11-10 of a word.
#define REG_GPO0		0x400
#define REG_GPO1		0x800
#define REG_MISC		0xc00

#define MISC_MODULE_PD		BIT(9)
#define MISC_XTAL1_PD		BIT(8)
#define MISC_XTAL2_PD		BIT(7)
#define MISC_LOADED		BIT(6)
#define MISC_CLK_OFF		BIT(5)

#define PROGRAM_TRIES		3

// The DACs the clock tables are made for (hbc3000_set_codec_type()).
enum {
	CODEC_ES9XXX = 0,
	CODEC_PCM1792 = 1,
	CODEC_AK4497 = 2,
	CODEC_CS43198 = 4,
	CODEC_AK4499 = 5,
	CODEC_BD34301 = 6,
	CODEC_AK4191 = 7,
	CODEC_ES9018K2M = 8,
	CODEC_AK4493 = 9,
};

static int hbc3000_jtag_tck_gpio = -1;
static int hbc3000_jtag_tms_gpio = -1;
static int hbc3000_jtag_tdo_gpio = -1;
static int hbc3000_jtag_tdi_gpio = -1;
static int hbc3000_power_gpio = -1;
static int hbc3000_xtal_delay_power_down;
static int hbc3000_share_spi_cs1 = -1;
static int hbc3000_jtag_tdo_as_reset;
static int hbc3000_jtag_tdo_as_share_spi_select;
static int hbc3000_share_spi = -1;
static int hbc3000_jtag_select = -1;
static int hbc3000_jtag_io_as_spi_io = -1;

module_param_cb(hbc3000_jtag_tck_gpio, &param_gpio_ops, &hbc3000_jtag_tck_gpio, 0644);
module_param_cb(hbc3000_jtag_tms_gpio, &param_gpio_ops, &hbc3000_jtag_tms_gpio, 0644);
module_param_cb(hbc3000_jtag_tdo_gpio, &param_gpio_ops, &hbc3000_jtag_tdo_gpio, 0644);
module_param_cb(hbc3000_jtag_tdi_gpio, &param_gpio_ops, &hbc3000_jtag_tdi_gpio, 0644);
module_param_cb(hbc3000_power_gpio, &param_gpio_ops, &hbc3000_power_gpio, 0644);
module_param(hbc3000_xtal_delay_power_down, int, 0644);
module_param(hbc3000_share_spi_cs1, int, 0644);
module_param(hbc3000_jtag_tdo_as_reset, int, 0644);
module_param(hbc3000_jtag_tdo_as_share_spi_select, int, 0644);
module_param(hbc3000_share_spi, int, 0644);
module_param_cb(hbc3000_jtag_select, &param_gpio_ops, &hbc3000_jtag_select, 0644);
module_param(hbc3000_jtag_io_as_spi_io, int, 0644);

struct hbc3000 {
	struct mutex spi_mutex;
	int tdi, tdo, tms, tck;		// JTAG
	int jtag_io_as_spi_io;		// SPI on the JTAG pins
	int share_spi;			// SPI shared with another device
	int tdo_as_share_spi_select;
	int tdo_as_reset;		// TDO is the run line
	int share_spi_select;
	int share_spi_cs1;
	int jtag_select;		// a mux in front of the JTAG pins
	int cs, clk, sdo;		// SPI
	int reset;			// the run line
	int power;			// the FPGA's supply
	int held[12];			// the pins requested
	int nheld;

	const u8 *fw;			// the configuration, a multiple of 256 bytes
	size_t fw_size;
	u32 usercode;			// set by the configuration; 0: unknown
};

static struct hbc3000 *g_hbc3000;
static u16 hbc3000_misc_cfg;
static u16 hbc3000_gpo_val[2];
static bool hbc3000_spdif_enabled;
static bool hbc3000_enabled;
static int codec_dsd_mode;

static u8 codec_type = 3;
static u8 hbc3000_osc_type = 1;		// 1: 22.5792 and 24.576 MHz crystals
static u32 hbc3000_id_code = IDCODE_GW1N_1;
static int current_samplerate = 44100;

// The JTAG lines.
static int TMS, TCK, TDI, TDO;

// The FPGA pins of the two GPO groups, for the two parts.
static const u8 hbc3000_gpo1_pins_gw1n_9c[] = { 0x29 };
static const u8 hbc3000_gpo0_pins_gw1n_9c[] = { 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x1b, 0x27, 0x28 };
static const u8 hbc3000_gpo1_pins_gw1n_1[] = { 0x1b, 0x1c, 0x1d, 0x1e, 0x20, 0x21, 0x22, 0x23 };
static const u8 hbc3000_gpo0_pins_gw1n_1[] = { 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12 };

// One clock table entry: the rate and the fields of the clock word.
struct hbc3000_rate {
	u32 rate;
	u8 reg;		// bits 11-10, always 0
	u8 xtal;	// bit 9
	u8 mode;	// bits 8-7
	u8 div;		// bits 6-3: 0 /1, 1 /2, 2 /4, 3 /6, 4 /8, 5 x2, 6 /12, 7 x4
};

static const struct hbc3000_rate spdif_table_22M[] = {
	{ 32000, 0, 1, 0, 1 }, { 44100, 0, 0, 0, 1 }, { 48000, 0, 1, 0, 1 },
	{ 88200, 0, 0, 0, 0 }, { 96000, 0, 1, 0, 0 }, { 176400, 0, 0, 0, 5 },
	{ 192000, 0, 1, 0, 5 }, { 352800, 0, 0, 0, 7 }, { 384000, 0, 1, 0, 7 },
	{ 705600, 0, 0, 0, 7 }, { 768000, 0, 1, 0, 7 },
};

static const struct hbc3000_rate spdif_table[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 1, 4 }, { 48000, 0, 1, 1, 4 },
	{ 88200, 0, 0, 0, 2 }, { 96000, 0, 1, 0, 2 }, { 176400, 0, 0, 0, 1 },
	{ 192000, 0, 1, 0, 1 },
};

static const struct hbc3000_rate dsd_i2s_table_ak4493[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 1, 4 }, { 48000, 0, 1, 1, 4 },
	{ 64000, 0, 1, 0, 3 }, { 88200, 0, 0, 0, 2 }, { 96000, 0, 1, 0, 2 },
	{ 176400, 0, 0, 0, 1 }, { 192000, 0, 1, 0, 1 }, { 352800, 0, 0, 1, 0 },
	{ 384000, 0, 1, 1, 0 }, { 705600, 0, 0, 0, 5 }, { 768000, 0, 1, 2, 5 },
	{ 2822400, 0, 0, 0, 4 }, { 5644800, 0, 0, 0, 2 }, { 11289600, 0, 0, 0, 1 },
	{ 22579200, 0, 0, 3, 0 }, { 45158400, 0, 0, 3, 5 },
};

static const struct hbc3000_rate dsd_i2s_table_es9xxx_22M[] = {
	{ 32000, 0, 1, 1, 3 }, { 44100, 0, 0, 1, 2 }, { 48000, 0, 1, 1, 2 },
	{ 64000, 0, 1, 1, 1 }, { 88200, 0, 0, 1, 1 }, { 96000, 0, 1, 1, 1 },
	{ 176400, 0, 0, 1, 0 }, { 192000, 0, 1, 1, 0 }, { 352800, 0, 0, 3, 5 },
	{ 384000, 0, 1, 3, 5 }, { 705600, 0, 0, 3, 7 }, { 768000, 0, 1, 3, 7 },
	{ 2822400, 0, 0, 1, 2 }, { 5644800, 0, 0, 1, 1 }, { 11289600, 0, 0, 1, 0 },
	{ 22579200, 0, 0, 3, 5 },
};

static const struct hbc3000_rate dsd_i2s_table_es9018k2m[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 0, 4 }, { 48000, 0, 1, 0, 4 },
	{ 64000, 0, 1, 0, 3 }, { 88200, 0, 0, 0, 2 }, { 96000, 0, 1, 0, 2 },
	{ 176400, 0, 0, 0, 1 }, { 192000, 0, 1, 0, 1 }, { 352800, 0, 0, 0, 0 },
	{ 384000, 0, 1, 0, 0 }, { 705600, 0, 0, 0, 5 }, { 768000, 0, 1, 0, 5 },
	{ 2822400, 0, 0, 0, 4 }, { 5644800, 0, 0, 0, 2 }, { 11289600, 0, 0, 0, 1 },
	{ 22579200, 0, 0, 3, 0 },
};

static const struct hbc3000_rate dsd_i2s_table_es9xxx[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 1, 4 }, { 48000, 0, 1, 1, 4 },
	{ 64000, 0, 1, 0, 3 }, { 88200, 0, 0, 0, 2 }, { 96000, 0, 1, 0, 2 },
	{ 176400, 0, 0, 0, 1 }, { 192000, 0, 1, 0, 1 }, { 352800, 0, 0, 3, 0 },
	{ 384000, 0, 1, 3, 0 }, { 705600, 0, 0, 3, 5 }, { 768000, 0, 1, 3, 5 },
	{ 2822400, 0, 0, 0, 4 }, { 5644800, 0, 0, 0, 2 }, { 11289600, 0, 0, 0, 1 },
	{ 22579200, 0, 0, 3, 0 }, { 45158400, 0, 0, 3, 5 },
};

static const struct hbc3000_rate dsd_i2s_table_cs43198[] = {
	{ 32000, 0, 1, 1, 6 }, { 44100, 0, 0, 1, 4 }, { 48000, 0, 1, 1, 4 },
	{ 64000, 0, 1, 1, 3 }, { 88200, 0, 0, 1, 2 }, { 96000, 0, 1, 1, 2 },
	{ 176400, 0, 0, 1, 1 }, { 192000, 0, 1, 1, 1 }, { 352800, 0, 0, 1, 0 },
	{ 384000, 0, 1, 1, 0 }, { 705600, 0, 0, 1, 5 }, { 768000, 0, 1, 1, 5 },
	{ 2822400, 0, 0, 0, 4 }, { 5644800, 0, 0, 0, 2 }, { 11289600, 0, 0, 0, 1 },
	{ 22579200, 0, 0, 0, 0 },
};

static const struct hbc3000_rate dsd_i2s_table_bd34301[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 2, 4 }, { 48000, 0, 1, 2, 4 },
	{ 64000, 0, 1, 1, 3 }, { 88200, 0, 0, 1, 2 }, { 96000, 0, 1, 1, 2 },
	{ 176400, 0, 0, 1, 1 }, { 192000, 0, 1, 1, 1 }, { 352800, 0, 0, 1, 0 },
	{ 384000, 0, 1, 1, 0 }, { 2822400, 0, 0, 2, 4 }, { 5644800, 0, 0, 2, 2 },
	{ 11289600, 0, 0, 1, 1 },
};

static const struct hbc3000_rate dsd_i2s_table_ak4191[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 2, 4 }, { 48000, 0, 1, 2, 4 },
	{ 64000, 0, 1, 2, 3 }, { 88200, 0, 0, 2, 2 }, { 96000, 0, 1, 2, 2 },
	{ 176400, 0, 0, 2, 1 }, { 192000, 0, 1, 2, 1 }, { 352800, 0, 0, 2, 0 },
	{ 384000, 0, 1, 2, 0 }, { 705600, 0, 0, 2, 5 }, { 768000, 0, 1, 2, 5 },
	{ 1411200, 0, 0, 2, 7 }, { 1536000, 0, 1, 2, 7 }, { 2822400, 0, 0, 2, 4 },
	{ 5644800, 0, 0, 2, 2 }, { 11289600, 0, 0, 1, 1 }, { 22579200, 0, 0, 2, 0 },
	{ 45158400, 0, 0, 2, 5 },
};

static const struct hbc3000_rate dsd_i2s_table_ak4499_22M[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 2, 4 }, { 48000, 0, 1, 2, 4 },
	{ 64000, 0, 1, 1, 3 }, { 88200, 0, 0, 1, 2 }, { 96000, 0, 1, 1, 2 },
	{ 176400, 0, 0, 0, 1 }, { 192000, 0, 1, 0, 1 }, { 352800, 0, 0, 1, 0 },
	{ 384000, 0, 1, 1, 0 }, { 2822400, 0, 0, 1, 4 }, { 5644800, 0, 0, 1, 2 },
	{ 11289600, 0, 0, 1, 1 },
};

static const struct hbc3000_rate dsd_i2s_table_ak4499[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 2, 4 }, { 48000, 0, 1, 2, 4 },
	{ 64000, 0, 1, 1, 3 }, { 88200, 0, 0, 1, 2 }, { 96000, 0, 1, 1, 2 },
	{ 176400, 0, 0, 0, 1 }, { 192000, 0, 1, 0, 1 }, { 352800, 0, 0, 1, 0 },
	{ 384000, 0, 1, 1, 0 }, { 2822400, 0, 0, 1, 4 }, { 5644800, 0, 0, 1, 2 },
	{ 11289600, 0, 0, 1, 1 },
};

static const struct hbc3000_rate dsd_i2s_table_ak4497[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 2, 4 }, { 48000, 0, 1, 2, 4 },
	{ 64000, 0, 1, 1, 3 }, { 88200, 0, 0, 1, 2 }, { 96000, 0, 1, 1, 2 },
	{ 176400, 0, 0, 0, 1 }, { 192000, 0, 1, 0, 1 }, { 352800, 0, 0, 1, 0 },
	{ 384000, 0, 1, 1, 0 }, { 2822400, 0, 0, 1, 4 }, { 5644800, 0, 0, 1, 2 },
	{ 11289600, 0, 0, 1, 1 },
};

static const struct hbc3000_rate dsd_i2s_table_pcm1792[] = {
	{ 32000, 0, 1, 2, 6 }, { 44100, 0, 0, 2, 4 }, { 48000, 0, 1, 2, 4 },
	{ 64000, 0, 1, 2, 3 }, { 88200, 0, 0, 2, 2 }, { 96000, 0, 1, 2, 2 },
	{ 176400, 0, 0, 1, 1 }, { 192000, 0, 1, 1, 1 }, { 352800, 0, 0, 0, 0 },
	{ 384000, 0, 1, 0, 0 }, { 2822400, 0, 0, 2, 4 }, { 5644800, 0, 0, 2, 2 },
	{ 11289600, 0, 0, 1, 1 },
};

// ---- JTAG, bit-banged ----

static void jtag_tms(int tms)
{
	gpio_set_value(TCK, 0);
	gpio_set_value(TMS, tms);
	gpio_set_value(TCK, 1);
}

static void idle_to_shift_dr(void)
{
	jtag_tms(1);
	jtag_tms(0);
	jtag_tms(0);
}

static void run_test(int clocks)
{
	int i;

	for (i = 0; i < clocks; i++) {
		gpio_set_value(TCK, 0);
		gpio_set_value(TCK, 1);
	}
}

static void exit1_to_idle(void)
{
	jtag_tms(1);
	jtag_tms(0);
	run_test(1);
}

// Test-Logic-Reset, then Run-Test/Idle.
static void jtag_reset(void)
{
	int i;

	gpio_set_value(TMS, 1);
	for (i = 0; i < 8; i++) {
		gpio_set_value(TCK, 0);
		gpio_set_value(TCK, 1);
	}
	jtag_tms(0);
}

// An 8-bit instruction, LSB first, from Run-Test/Idle back to it.
static void jtag_write_inst(u8 inst)
{
	int i;

	jtag_tms(1);
	jtag_tms(1);
	jtag_tms(0);
	jtag_tms(0);
	udelay(300);
	gpio_set_value(TMS, 0);
	for (i = 0; i < 8; i++) {
		gpio_set_value(TCK, 0);
		if (i == 7)
			gpio_set_value(TMS, 1);
		gpio_set_value(TDI, (inst >> i) & 1);
		gpio_set_value(TCK, 1);
	}
	udelay(500);
	exit1_to_idle();
	udelay(300);
	run_test(7);
}

// A 32-bit register after an instruction, LSB first.
static u32 jtag_read_code(u8 inst)
{
	u32 code = 0;
	int i;

	jtag_write_inst(inst);
	idle_to_shift_dr();
	for (i = 0; i < 32; i++) {
		gpio_set_value(TCK, 0);
		if (i == 31)
			gpio_set_value(TMS, 1);
		gpio_set_value(TCK, 1);
		if (gpio_get_value(TDO))
			code |= 1u << i;
	}
	exit1_to_idle();
	return code;
}

static bool hbc3000_id_known(u32 id)
{
	return id == IDCODE_GW1N_1 || id == IDCODE_GW1N_4 || (id & ~0x1000) == IDCODE_GW1N_9C;
}

static void hbc3000_set_id_code(u32 id)
{
	if (hbc3000_id_known(id))
		hbc3000_id_code = id;
}

// Loads the configuration into the FPGA's SRAM, unless its user code shows it
// is there already. 0 when the FPGA is configured.
static int hbc3000_jtag_program_ram(struct hbc3000 *h)
{
	u32 id, code;
	int last, bit;
	size_t i;
	u8 b;

	jtag_reset();
	id = jtag_read_code(JTAG_IDCODE);
	printk("hbc3000 id code:0x%x\n", id);
	if (!hbc3000_id_known(id)) {
		printk("id code error\n");
		return -1;
	}
	hbc3000_set_id_code(id);

	code = jtag_read_code(JTAG_USERCODE);
	if (h->usercode && code == h->usercode) {
		printk("pre check user code ok:%x\n", code);
		return 0;
	}
	printk("pre check user code:%x\n", code);

	jtag_reset();
	jtag_write_inst(JTAG_REPROGRAM);
	jtag_write_inst(JTAG_REPROGRAM);
	jtag_write_inst(JTAG_CONFIG_ENABLE);
	jtag_write_inst(JTAG_ERASE_SRAM);
	jtag_write_inst(JTAG_ERASE_SRAM);
	jtag_write_inst(JTAG_CONFIG_DISABLE);
	jtag_write_inst(JTAG_CONFIG_ENABLE);
	jtag_write_inst(JTAG_WRITE_SRAM);
	idle_to_shift_dr();
	udelay(500);

	// MSB first; TDI is only written when the bit changes, and once at the
	// start of every 256-byte page.
	gpio_set_value(TMS, 0);
	last = -1;
	for (i = 0; i < h->fw_size; i++) {
		if (!(i % 256))
			last = -1;
		b = h->fw[i];
		for (bit = 0; bit < 8; bit++) {
			gpio_set_value(TCK, 0);
			if ((b & 0x80) != last) {
				last = b & 0x80;
				gpio_set_value(TDI, last ? 1 : 0);
			}
			gpio_set_value(TCK, 1);
			b <<= 1;
		}
	}
	udelay(500);
	jtag_tms(1);
	jtag_tms(1);
	jtag_tms(0);
	jtag_write_inst(JTAG_CONFIG_DISABLE);
	jtag_write_inst(JTAG_NOOP);
	jtag_reset();

	code = jtag_read_code(JTAG_USERCODE);
	if (!h->usercode || code == h->usercode) {
		printk("user code ok:%x\n", code);
		return 0;
	}
	printk("user code error:%x, should be:%x%x\n", code, h->usercode >> 8 & 0xff,
	       h->usercode & 0xff);
	printk("status code: %x\n", jtag_read_code(JTAG_STATUS));
	// The GW1N-1 reports a user code that does not match and works.
	return id == IDCODE_GW1N_1 ? 0 : -1;
}

// ---- The SPI ----

// One 12-bit word, MSB first: data changes with the clock low.
static void hbc3000_spi_write_lock_or_unlock(u16 word, bool lock)
{
	struct hbc3000 *h = g_hbc3000;
	int i;

	if (lock)
		mutex_lock(&h->spi_mutex);
	if (h->share_spi_select >= 0)
		gpio_set_value(h->share_spi_select, 0);
	gpio_set_value(h->sdo, 0);
	gpio_set_value(h->clk, 0);
	gpio_set_value(h->cs, 0);
	for (i = 0; i < 12; i++) {
		gpio_set_value(h->clk, 0);
		gpio_set_value(h->sdo, (word & 0x800) ? 1 : 0);
		word <<= 1;
		gpio_set_value(h->clk, 1);
	}
	gpio_set_value(h->sdo, 0);
	gpio_set_value(h->cs, 1);
	if (lock)
		mutex_unlock(&h->spi_mutex);
}

static void hbc3000_spi_write(u16 word)
{
	hbc3000_spi_write_lock_or_unlock(word, true);
}

static void hbc3000_write_misc(void)
{
	hbc3000_spi_write(REG_MISC | hbc3000_misc_cfg);
}

static void hbc3000_restore_gpo(void)
{
	hbc3000_spi_write(REG_GPO0 | hbc3000_gpo_val[0]);
	hbc3000_spi_write(REG_GPO1 | hbc3000_gpo_val[1]);
}

// ---- Pins and power ----

// JTAG for loading (0), SPI and the run line afterwards (1).
static void hbc3000_set_jtag_io_func(int func)
{
	struct hbc3000 *h = g_hbc3000;

	if (func == 1) {
		if (h->jtag_select >= 0)
			gpio_set_value(h->jtag_select, 1);
		if (h->tdo_as_share_spi_select)
			gpio_direction_output(h->share_spi_select, 0);
		if (h->tdo_as_reset)
			gpio_direction_output(h->reset, 0);
	} else {
		if (h->jtag_select >= 0)
			gpio_set_value(h->jtag_select, 0);
		if (h->tdo_as_share_spi_select || h->tdo_as_reset)
			gpio_direction_input(h->tdo);
	}
}

static void hbc3000_power_enable(struct hbc3000 *h, bool on)
{
	if (h->power >= 0)
		gpio_set_value(h->power, on);
}

// Requests a pin, as an output unless out is negative. A pin that cannot be
// had is logged and skipped, as in the vendor module; the ones held are
// remembered for remove.
static void hbc3000_request(struct hbc3000 *h, int gpio, const char *label, int out,
			    const char *err)
{
	if (gpio_request(gpio, label)) {
		printk(err, gpio);
		return;
	}
	if (out < 0)
		gpio_direction_input(gpio);
	else
		gpio_direction_output(gpio, out);
	h->held[h->nheld++] = gpio;
}

static void hbc3000_gpio_init(struct hbc3000 *h)
{
	printk(KERN_INFO "hbc3000 gpio init\n");
	if (h->jtag_select >= 0)
		hbc3000_request(h, h->jtag_select, "GPIO_HM100_JTAG_SELECT", 0,
				KERN_INFO "GPIO_HM100_JTAG_SELECT error\n");

	printk(KERN_INFO "hbc3000 jtag gpio init\n");
	hbc3000_request(h, h->tdi, "JTAG_GPIO_PORT_TDI", 0, KERN_INFO "JTAG_GPIO_PORT_TDI error\n");
	hbc3000_request(h, h->tdo, "JTAG_GPIO_PORT_TDO", -1, KERN_INFO "JTAG_GPIO_PORT_TDO error\n");
	hbc3000_request(h, h->tms, "JTAG_GPIO_PORT_TMS", 0, KERN_INFO "JTAG_GPIO_PORT_TMS error\n");
	hbc3000_request(h, h->tck, "JTAG_GPIO_PORT_TCK", 0, KERN_INFO "JTAG_GPIO_PORT_TCK error\n");

	hbc3000_request(h, h->power, "GPIO_HBC3000_PIN_POWER", 0,
			KERN_INFO "GPIO_HBC3000_PIN_POWER %d error\n");
	if (!h->jtag_io_as_spi_io) {
		hbc3000_request(h, h->sdo, "GPIO_HBC3000_PIN_SDO", 0,
				KERN_INFO "GPIO_HBC3000_PIN_SDO error\n");
		hbc3000_request(h, h->clk, "GPIO_HBC3000_PIN_CLK", 0,
				KERN_INFO "GPIO_HBC3000_PIN_CLK error\n");
		hbc3000_request(h, h->cs, "GPIO_HBC3000_PIN_CS", 1,
				KERN_INFO "GPIO_HBC3000_PIN_CS error\n");
	} else {
		h->sdo = h->tdi;
		h->clk = h->tck;
		h->cs = h->tms;
	}
	if (!h->tdo_as_reset)
		hbc3000_request(h, h->reset, "GPIO_HBC3000_PIN_RESET", 0,
				KERN_INFO "GPIO_HBC3000_PIN_RESET error\n");
	else
		h->reset = h->tdo;

	if (h->share_spi) {
		if (!h->tdo_as_share_spi_select && h->share_spi_select >= 0)
			hbc3000_request(h, h->share_spi_select, "share_spi_select", 0,
					KERN_INFO "share_spi_select %d error\n");
		if (h->share_spi_cs1 >= 0)
			hbc3000_request(h, h->share_spi_cs1, "share_spi_cs1", 1,
					KERN_INFO "share_spi_cs1 %d error\n");
	}
}

// ---- Exported to the sound card ----

int hbc3000_start(void)
{
	if (!g_hbc3000)
		return -ENODEV;
	gpio_set_value(g_hbc3000->reset, 1);
	printk(KERN_INFO "hbc3000_start\n");
	return 0;
}
EXPORT_SYMBOL_GPL(hbc3000_start);

int hbc3000_stop(void)
{
	if (!g_hbc3000)
		return -ENODEV;
	gpio_set_value(g_hbc3000->reset, 0);
	printk(KERN_INFO "hbc3000_stop\n");
	return 0;
}
EXPORT_SYMBOL_GPL(hbc3000_stop);

int hbc3000_set_codec_type(int type)
{
	if ((unsigned int)type < 10 || (unsigned int)(type - 100) < 3)
		codec_type = type;
	printk(KERN_INFO "code type is %d\n", codec_type);
	return codec_type;
}
EXPORT_SYMBOL_GPL(hbc3000_set_codec_type);

int hbc3000_set_spdif_mode(int spdif)
{
	printk(KERN_INFO "%s:%d\n", __func__, spdif);
	hbc3000_spdif_enabled = spdif;
	return 0;
}
EXPORT_SYMBOL_GPL(hbc3000_set_spdif_mode);

// The clock table of the configured DAC, or S/PDIF's; mode is the low bits
// of the clock word.
static const struct hbc3000_rate *hbc3000_rate_table(int *n, int *mode)
{
	if (hbc3000_spdif_enabled) {
		*mode = 1;
		if (hbc3000_osc_type == 1) {
			*n = ARRAY_SIZE(spdif_table);
			return spdif_table;
		}
		*n = ARRAY_SIZE(spdif_table_22M);
		return spdif_table_22M;
	}

	*mode = codec_dsd_mode;
	switch (codec_type) {
	case CODEC_PCM1792:
		*n = ARRAY_SIZE(dsd_i2s_table_pcm1792);
		return dsd_i2s_table_pcm1792;
	case CODEC_AK4497:
		*n = ARRAY_SIZE(dsd_i2s_table_ak4497);
		return dsd_i2s_table_ak4497;
	case CODEC_CS43198:
		*n = ARRAY_SIZE(dsd_i2s_table_cs43198);
		return dsd_i2s_table_cs43198;
	case CODEC_AK4499:
		if (hbc3000_osc_type == 1) {
			*n = ARRAY_SIZE(dsd_i2s_table_ak4499);
			return dsd_i2s_table_ak4499;
		}
		*n = ARRAY_SIZE(dsd_i2s_table_ak4499_22M);
		return dsd_i2s_table_ak4499_22M;
	case CODEC_BD34301:
		*n = ARRAY_SIZE(dsd_i2s_table_bd34301);
		return dsd_i2s_table_bd34301;
	case CODEC_AK4191:
		*n = ARRAY_SIZE(dsd_i2s_table_ak4191);
		return dsd_i2s_table_ak4191;
	case CODEC_ES9018K2M:
		*n = ARRAY_SIZE(dsd_i2s_table_es9018k2m);
		return dsd_i2s_table_es9018k2m;
	case CODEC_AK4493:
		*n = ARRAY_SIZE(dsd_i2s_table_ak4493);
		return dsd_i2s_table_ak4493;
	case 100:
	case 102:
		*n = ARRAY_SIZE(dsd_i2s_table_es9xxx);
		return dsd_i2s_table_es9xxx;
	default:
		if (hbc3000_osc_type == 1) {
			*n = ARRAY_SIZE(dsd_i2s_table_es9xxx);
			return dsd_i2s_table_es9xxx;
		}
		*n = ARRAY_SIZE(dsd_i2s_table_es9xxx_22M);
		return dsd_i2s_table_es9xxx_22M;
	}
}

// Stops the FPGA and sets its clocks for a rate; mode 2 unpacks DoP. Returns
// the crystal used (1: 24.576 MHz), or -1 for a rate the table lacks.
int hbc3000_set_samplerate(int rate, int mode)
{
	const struct hbc3000_rate *t;
	int n, i, low;
	u16 word;

	current_samplerate = rate;
	codec_dsd_mode = mode;
	printk(KERN_INFO "%s codec_type=%d samplerate=%d spdif=%d codec_dsd_mode=%d\n",
	       __func__, codec_type, rate, hbc3000_spdif_enabled, mode);

	t = hbc3000_rate_table(&n, &low);
	for (i = 0; i < n && t[i].rate != rate; i++)
		;
	if (i == n) {
		printk(KERN_INFO "%s samplerate=%d failed\n", __func__, rate);
		return -1;
	}
	t += i;
	word = (t->reg & 3) << 10 | t->xtal << 9 | (t->mode & 3) << 7 | (t->div & 15) << 3 | (low & 7);

	if (!g_hbc3000)
		return -ENODEV;
	hbc3000_stop();
	// Both crystals on while switching; the sound card powers the unused one
	// down once playback runs.
	if (hbc3000_xtal_delay_power_down) {
		hbc3000_misc_cfg &= ~(MISC_XTAL1_PD | MISC_XTAL2_PD);
		hbc3000_write_misc();
	}
	hbc3000_spi_write(word);
	if (!hbc3000_xtal_delay_power_down) {
		hbc3000_misc_cfg &= ~(MISC_XTAL1_PD | MISC_XTAL2_PD);
		hbc3000_misc_cfg |= t->xtal ? MISC_XTAL1_PD : MISC_XTAL2_PD;
		hbc3000_write_misc();
	}
	printk(KERN_INFO "%s samplerate:%d 0x%04x End\n", __func__, t->rate, word);
	return t->xtal;
}
EXPORT_SYMBOL_GPL(hbc3000_set_samplerate);

// Power-down bits of the misc register: 0 the module, 1 and 2 the crystals,
// 3 bit 6, 4 all four.
int hbc3000_set_power_down(int module, int down)
{
	static const u16 bits[] = {
		MISC_MODULE_PD, MISC_XTAL1_PD, MISC_XTAL2_PD, MISC_LOADED,
		MISC_MODULE_PD | MISC_XTAL1_PD | MISC_XTAL2_PD | MISC_LOADED,
	};
	struct hbc3000 *h = g_hbc3000;

	printk(KERN_INFO "%s: module: %d power_down: %d\n", __func__, module, down);
	if (!h)
		return -ENODEV;
	mutex_lock(&h->spi_mutex);
	if ((unsigned int)module >= ARRAY_SIZE(bits)) {
		mutex_unlock(&h->spi_mutex);
		return -1;
	}
	if (down)
		hbc3000_misc_cfg |= bits[module];
	else
		hbc3000_misc_cfg &= ~bits[module];
	hbc3000_spi_write_lock_or_unlock(REG_MISC | hbc3000_misc_cfg, false);
	mutex_unlock(&h->spi_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(hbc3000_set_power_down);

// One GPO, by FPGA pin; a pin in neither group is ignored.
int hbc3000_set_gpo(int gpo, int value)
{
	const u8 *g0, *g1;
	int n0, n1, i;

	if (hbc3000_id_code == IDCODE_GW1N_9C) {
		g0 = hbc3000_gpo0_pins_gw1n_9c;
		n0 = ARRAY_SIZE(hbc3000_gpo0_pins_gw1n_9c);
		g1 = hbc3000_gpo1_pins_gw1n_9c;
		n1 = ARRAY_SIZE(hbc3000_gpo1_pins_gw1n_9c);
	} else {
		g0 = hbc3000_gpo0_pins_gw1n_1;
		n0 = ARRAY_SIZE(hbc3000_gpo0_pins_gw1n_1);
		g1 = hbc3000_gpo1_pins_gw1n_1;
		n1 = ARRAY_SIZE(hbc3000_gpo1_pins_gw1n_1);
	}

	for (i = 0; i < n0; i++) {
		if (gpo == g0[i]) {
			hbc3000_gpo_val[0] = (hbc3000_gpo_val[0] & ~(1 << i)) | value << i;
			if (g_hbc3000)
				hbc3000_spi_write(REG_GPO0 | hbc3000_gpo_val[0]);
			return 0;
		}
	}
	for (i = 0; i < n1; i++) {
		if (gpo == g1[i]) {
			hbc3000_gpo_val[1] = (hbc3000_gpo_val[1] & ~(1 << i)) | value << i;
			if (g_hbc3000)
				hbc3000_spi_write(REG_GPO1 | hbc3000_gpo_val[1]);
			return 0;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(hbc3000_set_gpo);

// Powers the FPGA (or, already on, takes it out of power-down), loads the
// configuration, up to three times with a power cycle between them, then
// restores the GPOs and the clocks.
int hbc3000_enable(void)
{
	struct hbc3000 *h = g_hbc3000;
	int i, ret;

	if (!h)
		return -ENODEV;

	if (hbc3000_enabled) {
		hbc3000_misc_cfg &= ~(MISC_XTAL2_PD | MISC_XTAL1_PD | MISC_MODULE_PD | MISC_CLK_OFF);
		hbc3000_misc_cfg |= MISC_LOADED;
		hbc3000_write_misc();
		printk(KERN_INFO "hbc3000_enabled\n");
	} else {
		hbc3000_power_enable(h, true);
		mdelay(50);
	}

	mutex_lock(&h->spi_mutex);
	for (i = 0; i < PROGRAM_TRIES; i++) {
		printk(KERN_INFO "hbc3000_program_ram %d\n", i);
		hbc3000_set_jtag_io_func(0);
		ret = hbc3000_jtag_program_ram(h);
		hbc3000_set_jtag_io_func(1);
		if (!ret)
			break;
		hbc3000_power_enable(h, false);
		msleep(1000);
		hbc3000_power_enable(h, true);
		mdelay(50);
	}
	mutex_unlock(&h->spi_mutex);

	if (i == PROGRAM_TRIES) {
		hbc3000_power_enable(h, false);
		printk(KERN_INFO "hbc3000_program_ram failed\n");
		return 0;
	}

	hbc3000_restore_gpo();
	hbc3000_misc_cfg &= ~(MISC_XTAL2_PD | MISC_XTAL1_PD | MISC_MODULE_PD);
	hbc3000_misc_cfg |= MISC_LOADED;
	hbc3000_write_misc();
	hbc3000_start();
	udelay(1000);
	hbc3000_stop();
	hbc3000_enabled = true;
	printk(KERN_INFO "hbc3000_enabled\n");
	return hbc3000_set_samplerate(current_samplerate, codec_dsd_mode);
}
EXPORT_SYMBOL_GPL(hbc3000_enable);

int hbc3000_disable(void)
{
	if (!g_hbc3000)
		return -ENODEV;
	hbc3000_power_enable(g_hbc3000, false);
	hbc3000_enabled = false;
	printk(KERN_INFO "hbc3000_disable\n");
	return 0;
}
EXPORT_SYMBOL_GPL(hbc3000_disable);

// ---- The device ----

// The user code is set by the configuration's 0x0a command near its end:
// "0a 00 00 00" and the 32-bit code, followed by 0xff padding.
static u32 hbc3000_fw_usercode(const u8 *fw, size_t size)
{
	size_t p, from = size > 1024 ? size - 1024 : 0;

	for (p = size - 8; size >= 8 && p >= from && p < size; p--) {
		if (fw[p] == 0x0a && !fw[p + 1] && !fw[p + 2] && !fw[p + 3] &&
		    (p + 8 == size || fw[p + 8] == 0xff))
			return fw[p + 4] << 24 | fw[p + 5] << 16 | fw[p + 6] << 8 | fw[p + 7];
		if (!p)
			break;
	}
	return 0;
}

static int hbc3000_load_firmware(struct device *dev, struct hbc3000 *h)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, HBC3000_FIRMWARE, dev);
	if (ret) {
		dev_err(dev, "no %s (%d): it is FS_data of the stock sa_sound_hbc3000.ko\n",
			HBC3000_FIRMWARE, ret);
		return ret;
	}
	if (!fw->size || fw->size % 256) {
		dev_err(dev, "%s: %zu bytes, not whole 256-byte pages\n", HBC3000_FIRMWARE, fw->size);
		release_firmware(fw);
		return -EINVAL;
	}
	h->fw = devm_kmemdup(dev, fw->data, fw->size, GFP_KERNEL);
	h->fw_size = fw->size;
	release_firmware(fw);
	if (!h->fw)
		return -ENOMEM;
	h->usercode = hbc3000_fw_usercode(h->fw, h->fw_size);
	if (!h->usercode)
		dev_warn(dev, "%s has no user code: loaded at every enable\n", HBC3000_FIRMWARE);
	return 0;
}

static int hbc3000_probe(struct platform_device *pdev)
{
	struct hbc3000 *h;
	int ret;

	printk(KERN_INFO "hbc3000_probe.\n");
	hbc3000_misc_cfg = 0;

	h = devm_kzalloc(&pdev->dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
	h->tdi = hbc3000_jtag_tdi_gpio;
	h->tdo = hbc3000_jtag_tdo_gpio;
	h->tms = hbc3000_jtag_tms_gpio;
	h->tck = hbc3000_jtag_tck_gpio;
	h->jtag_io_as_spi_io = hbc3000_jtag_io_as_spi_io;
	h->share_spi = hbc3000_share_spi;
	h->tdo_as_share_spi_select = hbc3000_jtag_tdo_as_share_spi_select;
	h->tdo_as_reset = hbc3000_jtag_tdo_as_reset;
	h->share_spi_select = -1;
	h->share_spi_cs1 = hbc3000_share_spi_cs1;
	h->jtag_select = hbc3000_jtag_select;
	h->cs = h->clk = h->sdo = h->reset = -1;
	h->power = hbc3000_power_gpio;
	if (h->tdo_as_share_spi_select && h->tdo_as_reset)
		printk(KERN_ERR "jtag tdo only working with one function.\n");

	ret = hbc3000_load_firmware(&pdev->dev, h);
	if (ret)
		return ret;

	mutex_init(&h->spi_mutex);
	hbc3000_gpio_init(h);
	TDO = h->tdo;
	TDI = h->tdi;
	TCK = h->tck;
	TMS = h->tms;
	g_hbc3000 = h;
	platform_set_drvdata(pdev, h);

	// Loaded once, then off until the sound card picks an output.
	hbc3000_enable();
	hbc3000_stop();
	hbc3000_disable();
	printk(KERN_INFO "hbc3000_probe finish.\n");
	return 0;
}

static int hbc3000_remove(struct platform_device *pdev)
{
	struct hbc3000 *h = platform_get_drvdata(pdev);

	hbc3000_power_enable(h, false);
	hbc3000_enabled = false;
	g_hbc3000 = NULL;
	while (h->nheld)
		gpio_free(h->held[--h->nheld]);
	return 0;
}

// The FPGA loses its power, and with it its configuration, in a suspend;
// hbc3000_enable() at the next output change powers and loads it again.
static int hbc3000_suspend(struct device *dev)
{
	struct hbc3000 *h = g_hbc3000;

	printk(KERN_INFO "%s\n", __func__);
	hbc3000_power_enable(h, false);
	hbc3000_enabled = false;
	gpio_set_value(h->sdo, 0);
	gpio_set_value(h->clk, 0);
	gpio_set_value(h->cs, 0);
	return 0;
}

static int hbc3000_resume(struct device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static int hbc3000_poweroff(struct device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	hbc3000_power_enable(g_hbc3000, false);
	return 0;
}

static const struct dev_pm_ops pm_ops = {
	.suspend = hbc3000_suspend,
	.resume = hbc3000_resume,
	.poweroff = hbc3000_poweroff,
};

static struct platform_driver hbc3000_driver = {
	.probe = hbc3000_probe,
	.remove = hbc3000_remove,
	.driver = {
		.name = "hbc3000",
		.owner = THIS_MODULE,
		.pm = &pm_ops,
	},
};

static void sa_sound_hbc3000_release(struct device *dev)
{
}

static struct platform_device sa_sound_hbc3000_dev = {
	.name = "hbc3000",
	.id = -1,
	.dev = {
		.release = sa_sound_hbc3000_release,
	},
};

static int __init hbc3000_driver_init(void)
{
	int ret;

	printk(KERN_INFO "hbc3000_driver_init\n");
	ret = platform_device_register(&sa_sound_hbc3000_dev);
	if (ret) {
		printk(KERN_ERR "hbc3000 device init error.\n");
		return ret;
	}
	ret = platform_driver_register(&hbc3000_driver);
	if (ret) {
		printk(KERN_INFO "hbc3000_driver_init failed\n");
		platform_device_unregister(&sa_sound_hbc3000_dev);
		return ret;
	}
	printk(KERN_INFO "hbc3000_driver_init finish\n");
	return 0;
}
module_init(hbc3000_driver_init);

static void __exit hbc3000_driver_exit(void)
{
	printk(KERN_INFO "hbc3000_driver_exit\n");
	platform_driver_unregister(&hbc3000_driver);
	platform_device_unregister(&sa_sound_hbc3000_dev);
}
module_exit(hbc3000_driver_exit);

MODULE_DESCRIPTION("HBC3000 FPGA of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
