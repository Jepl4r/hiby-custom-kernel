// SPDX-License-Identifier: GPL-2.0
//
// lcd_lg35583 -- the 480x800 RGB panel of the HiBy R1, registered with
// soc_fb.ko as its panel. Two panels are fitted: an LG35583, set up over a
// bit-banged SPI with 16-bit words, and an ST7701S, set up with 9-bit words;
// ADC channel 3 tells them apart at load (below 1000 mV: the LG35583).
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock lcd_lg35583.sh passes them):
//
//   gpio_spi_cs, gpio_spi_sck, gpio_spi_mosi   the SPI pins (required)
//   gpio_lcd_rst          reset, active low (-1: none)
//   gpio_lcd_power_en     power switch, active high (-1: none)
//   vcc_regulator_name, vccio_regulator_name   the panel supplies ("" or
//                         "-1": none)
//   spi_bus_num           not read
//
// soc_fb calls power_on when the screen turns on and power_off when it turns
// off. The first power_on creates /proc/lcd_reg: writing "c700 XX" (hex)
// sets the value the LG35583's register C700 gets at the next power_on;
// reading prints the register and its value to the kernel log.
//
// Unloading leaves the panel registered: jzfb_unregister_lcd() would take the
// framebuffer down with it.
//
// Loaded after utils.ko, soc_adc.ko and soc_fb.ko.

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/uaccess.h>

// The panel description soc_fb.ko takes. Values not named here are 0.
struct lcdc_data {
	const char *name;
	unsigned int refresh;
	unsigned int xres;
	unsigned int yres;
	unsigned int pixclock;		// 0: from refresh and the timings
	unsigned int left_margin;
	unsigned int right_margin;
	unsigned int upper_margin;
	unsigned int lower_margin;
	unsigned int hsync_len;
	unsigned int vsync_len;
	unsigned int fb_fmt;		// 1: RGB565 in memory
	unsigned int lcd_mode;		// 0: parallel TFT
	unsigned int out_format;	// 1: 18-bit RGB666 on the pins
	unsigned int color_even;
	unsigned int color_odd;
	unsigned int pix_clk_active;	// 1: data on the rising edge
	unsigned int de_active_level;	// 1: high
	unsigned int hsync_active_level;
	unsigned int vsync_active_level;
	unsigned int smart_lcd[18];	// smart (MCU) panels only
	int (*power_on)(void *unused);
	int (*power_off)(void *unused);
	unsigned int reserved;
};

// Exported by soc_fb.ko.
extern int jzfb_register_lcd(struct lcdc_data *pdata);

// Exported by soc_adc.ko.
extern void adc_enable(void);
extern void adc_disable(void);
extern int adc_read_channel_voltage(int channel);

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern char *gpio_to_str(int gpio, char *buf);

#define LCD_ID_CHANNEL		3
#define LCD_ID_ST7701S_MV	1000	// at or above: the ST7701S

static int spi_bus_num = -1;
static int gpio_spi_mosi = -1;
static int gpio_spi_sck = -1;
static int gpio_spi_cs = -1;
static int gpio_lcd_rst = -1;
static char *vccio_regulator_name = "";
static char *vcc_regulator_name = "";
static int gpio_lcd_power_en = -1;

module_param(spi_bus_num, int, 0644);
module_param_cb(gpio_spi_mosi, &param_gpio_ops, &gpio_spi_mosi, 0644);
module_param_cb(gpio_spi_sck, &param_gpio_ops, &gpio_spi_sck, 0644);
module_param_cb(gpio_spi_cs, &param_gpio_ops, &gpio_spi_cs, 0644);
module_param_cb(gpio_lcd_rst, &param_gpio_ops, &gpio_lcd_rst, 0644);
module_param(vccio_regulator_name, charp, 0644);
module_param(vcc_regulator_name, charp, 0644);
module_param_cb(gpio_lcd_power_en, &param_gpio_ops, &gpio_lcd_power_en, 0644);

static struct regulator *vccio_regulator;
static struct regulator *vcc_regulator;
static int lcd_id_vol;		// mV on the ID channel
static int lcd_id;		// 0: LG35583, 1: ST7701S
static struct proc_dir_entry *lcd_reg_entry;
static bool lcd_reg_done;	// creation tried

// The LG35583 register /proc/lcd_reg sets, and its value.
static struct {
	u16 reg;
	u8 val;
} reg_map = { 0xc700, 0x46 };

// One 16-bit word, MSB first, for the LG35583: the panel samples MOSI on the
// rising edge of SCK.
static void m_spi_write(u16 val)
{
	int i;

	gpio_set_value(gpio_spi_sck, 0);
	gpio_set_value(gpio_spi_cs, 0);
	udelay(1);

	for (i = 0; i < 16; i++) {
		gpio_set_value(gpio_spi_mosi, (val & 0x8000) ? 1 : 0);
		val <<= 1;
		gpio_set_value(gpio_spi_sck, 1);
		gpio_set_value(gpio_spi_sck, 0);
	}

	udelay(1);
	gpio_set_value(gpio_spi_cs, 1);
	gpio_set_value(gpio_spi_mosi, 0);
	udelay(1);
}

// One 9-bit word, MSB first, for the ST7701S: bit 8 is 0 for a command, 1
// for data.
static void m_spi_write_addr_8(u16 val)
{
	int i;

	gpio_set_value(gpio_spi_sck, 0);
	gpio_set_value(gpio_spi_cs, 0);
	udelay(1);

	for (i = 0; i < 9; i++) {
		gpio_set_value(gpio_spi_mosi, (val & 0x100) ? 1 : 0);
		val <<= 1;
		gpio_set_value(gpio_spi_sck, 0);
		udelay(1);
		gpio_set_value(gpio_spi_sck, 1);
		udelay(1);
	}

	udelay(1);
	gpio_set_value(gpio_spi_cs, 1);
	gpio_set_value(gpio_spi_mosi, 0);
	udelay(1);
}

// An LG35583 register address: high byte, then low byte. Data follows as
// 0x40xx words.
#define LG_ADDR_HI	0x2000
#define LG_ADDR_LO	0x0000
#define LG_DATA		0x4000

static void spi_writecomm(u16 reg)
{
	m_spi_write(LG_ADDR_HI | (reg >> 8));
	m_spi_write(LG_ADDR_LO | (reg & 0xff));
}

struct lg_write {
	u16 reg;	// 0: a delay of val ms
	s16 val;	// -1: no data
};

#define LG_REG(r, v)	{ (r), (v) }
#define LG_CMD(r)	{ (r), -1 }
#define LG_DELAY(ms)	{ 0, (ms) }

// The LG35583 set-up, from reset to Display On (0x2900) and Memory Write
// (0x2c00). C700 takes the value of reg_map.
static const struct lg_write lg35583_init_seq[] = {
	LG_REG(0xb100, 0x06),
	LG_REG(0xb101, 0x01),
	LG_REG(0xb102, 0x54),
	LG_REG(0xb104, 0x07),
	LG_REG(0xb105, 0x06),
	LG_REG(0xb106, 0x06),
	LG_REG(0xb107, 0x4a),
	LG_REG(0xb108, 0x21),
	LG_REG(0xb109, 0x08),
	LG_REG(0xf101, 0xc0),
	LG_REG(0xf103, 0x01),
	LG_DELAY(5),
	LG_REG(0xb400, 0x01),
	LG_REG(0xb600, 0x10),
	LG_REG(0x3a00, 0x77),
	LG_REG(0x3b00, 0x03),
	LG_REG(0x5e04, 0x02),
	LG_REG(0x3600, 0x08),
	LG_DELAY(5),
	LG_REG(0xe000, 0x00),
	LG_REG(0xe001, 0x2f),
	LG_REG(0xe002, 0x3f),
	LG_REG(0xe003, 0x1f),
	LG_REG(0xe004, 0x21),
	LG_REG(0xe005, 0x2c),
	LG_REG(0xe006, 0x2c),
	LG_REG(0xe007, 0x61),
	LG_REG(0xe008, 0x1c),
	LG_REG(0xe009, 0x18),
	LG_REG(0xe00a, 0xbc),
	LG_REG(0xe00b, 0x27),
	LG_REG(0xe00c, 0x59),
	LG_REG(0xe00d, 0x6a),
	LG_REG(0xe00e, 0xbb),
	LG_REG(0xe00f, 0xbe),
	LG_REG(0xe010, 0x3e),
	LG_REG(0xe011, 0x48),
	LG_REG(0xe100, 0x00),
	LG_REG(0xe101, 0x32),
	LG_REG(0xe102, 0x45),
	LG_REG(0xe103, 0x26),
	LG_REG(0xe104, 0x21),
	LG_REG(0xe105, 0x2c),
	LG_REG(0xe106, 0x5c),
	LG_REG(0xe107, 0x63),
	LG_REG(0xe108, 0x1c),
	LG_REG(0xe109, 0x19),
	LG_REG(0xe10a, 0xd3),
	LG_REG(0xe10b, 0x28),
	LG_REG(0xe10c, 0x59),
	LG_REG(0xe10d, 0x6b),
	LG_REG(0xe10e, 0xdb),
	LG_REG(0xe10f, 0xde),
	LG_REG(0xe110, 0x5f),
	LG_REG(0xe111, 0x6b),
	LG_REG(0xe200, 0x7f),
	LG_REG(0xe201, 0x7d),
	LG_REG(0xe202, 0x86),
	LG_REG(0xe203, 0x89),
	LG_REG(0xe204, 0x10),
	LG_REG(0xe205, 0x1e),
	LG_REG(0xe206, 0x41),
	LG_REG(0xe207, 0x77),
	LG_REG(0xe208, 0x0d),
	LG_REG(0xe209, 0x1d),
	LG_REG(0xe20a, 0xc8),
	LG_REG(0xe20b, 0x30),
	LG_REG(0xe20c, 0x5f),
	LG_REG(0xe20d, 0x71),
	LG_REG(0xe20e, 0xc3),
	LG_REG(0xe20f, 0xbe),
	LG_REG(0xe210, 0x3e),
	LG_REG(0xe211, 0x48),
	LG_REG(0xe300, 0x7f),
	LG_REG(0xe301, 0x7f),
	LG_REG(0xe302, 0x86),
	LG_REG(0xe303, 0x93),
	LG_REG(0xe304, 0x10),
	LG_REG(0xe305, 0x1e),
	LG_REG(0xe306, 0x41),
	LG_REG(0xe307, 0x89),
	LG_REG(0xe308, 0x0d),
	LG_REG(0xe309, 0x1d),
	LG_REG(0xe30a, 0xe0),
	LG_REG(0xe30b, 0x30),
	LG_REG(0xe30c, 0x5f),
	LG_REG(0xe30d, 0x70),
	LG_REG(0xe30e, 0xe4),
	LG_REG(0xe30f, 0xdf),
	LG_REG(0xe310, 0x5f),
	LG_REG(0xe311, 0x6b),
	LG_REG(0xe400, 0x00),
	LG_REG(0xe401, 0x43),
	LG_REG(0xe402, 0x6f),
	LG_REG(0xe403, 0x60),
	LG_REG(0xe404, 0x21),
	LG_REG(0xe405, 0x2c),
	LG_REG(0xe406, 0x5a),
	LG_REG(0xe407, 0x6d),
	LG_REG(0xe408, 0x1c),
	LG_REG(0xe409, 0x18),
	LG_REG(0xe40a, 0xc6),
	LG_REG(0xe40b, 0x27),
	LG_REG(0xe40c, 0x59),
	LG_REG(0xe40d, 0x6a),
	LG_REG(0xe40e, 0xbb),
	LG_REG(0xe40f, 0xbe),
	LG_REG(0xe410, 0x3e),
	LG_REG(0xe411, 0x48),
	LG_REG(0xe500, 0x00),
	LG_REG(0xe501, 0x46),
	LG_REG(0xe502, 0x75),
	LG_REG(0xe503, 0x67),
	LG_REG(0xe504, 0x21),
	LG_REG(0xe505, 0x2c),
	LG_REG(0xe506, 0x5a),
	LG_REG(0xe507, 0x7e),
	LG_REG(0xe508, 0x1c),
	LG_REG(0xe509, 0x19),
	LG_REG(0xe50a, 0xdd),
	LG_REG(0xe50b, 0x28),
	LG_REG(0xe50c, 0x59),
	LG_REG(0xe50d, 0x6b),
	LG_REG(0xe50e, 0xdb),
	LG_REG(0xe50f, 0xde),
	LG_REG(0xe510, 0x5f),
	LG_REG(0xe511, 0x6b),
	LG_DELAY(5),
	LG_REG(0xc200, 0x70),
	LG_REG(0xc202, 0x70),
	LG_REG(0xc203, 0x44),
	LG_DELAY(20),
	LG_REG(0xc200, 0x51),
	LG_REG(0xc201, 0x06),
	LG_REG(0xc202, 0x51),
	LG_REG(0xc203, 0x66),
	LG_DELAY(20),
	LG_REG(0xc100, 0x45),
	LG_DELAY(5),
	LG_REG(0xc700, 0x46),
	LG_REG(0xc000, 0xa3),
	LG_REG(0xc002, 0x8a),
	LG_DELAY(5),
	LG_CMD(0x1100),
	LG_DELAY(130),
	LG_CMD(0x2900),
	LG_CMD(0x2c00),
};

#define CMD(c)		(c)
#define DAT(d)		(0x100 | (d))
#define DELAY(ms)	(0x8000 | (ms))

// The ST7701S set-up, from reset to Display On (0x29).
static const u16 st7701s_init_seq[] = {
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x13),
	CMD(0xef), DAT(0x08),
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x10),
	CMD(0xc0), DAT(0x63), DAT(0x00),
	CMD(0xc1), DAT(0x09), DAT(0x0c),
	CMD(0xc2), DAT(0x07), DAT(0x08),
	CMD(0xcc), DAT(0x30),
	CMD(0xb0), DAT(0x00), DAT(0x0d), DAT(0x14), DAT(0x0d), DAT(0x11),
		 DAT(0x07), DAT(0x04), DAT(0x08), DAT(0x08), DAT(0x20),
		 DAT(0x05), DAT(0x14), DAT(0x12), DAT(0x25), DAT(0x2d),
		 DAT(0x1c),
	CMD(0xb1), DAT(0x00), DAT(0x0c), DAT(0x14), DAT(0x0d), DAT(0x11),
		 DAT(0x06), DAT(0x03), DAT(0x08), DAT(0x08), DAT(0x1f),
		 DAT(0x05), DAT(0x14), DAT(0x12), DAT(0x25), DAT(0x2e),
		 DAT(0x1c),
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x11),
	CMD(0xb0), DAT(0x58),
	CMD(0xb1), DAT(0x4a),
	CMD(0xb2), DAT(0x87),
	CMD(0xb3), DAT(0x80),
	CMD(0xb5), DAT(0x4c),
	CMD(0xb7), DAT(0x8a),
	CMD(0xb8), DAT(0x21),
	CMD(0xc0), DAT(0x03),
	CMD(0xc1), DAT(0x78),
	CMD(0xc2), DAT(0x78),
	CMD(0xd0), DAT(0x88),
	CMD(0xe0), DAT(0x00), DAT(0x00), DAT(0x02),
	CMD(0xe1), DAT(0x01), DAT(0xa0), DAT(0x03), DAT(0xa0), DAT(0x02),
		 DAT(0xa0), DAT(0x04), DAT(0xa0), DAT(0x00), DAT(0x44),
		 DAT(0x44),
	CMD(0xe2), DAT(0x00), DAT(0x00), DAT(0x00), DAT(0x00), DAT(0x00),
		 DAT(0x00), DAT(0x00), DAT(0x00), DAT(0x00), DAT(0x00),
		 DAT(0x00), DAT(0x00),
	CMD(0xe3), DAT(0x00), DAT(0x00), DAT(0x33), DAT(0x33),
	CMD(0xe4), DAT(0x44), DAT(0x44),
	CMD(0xe5), DAT(0x01), DAT(0x26), DAT(0xa0), DAT(0xa0), DAT(0x03),
		 DAT(0x28), DAT(0xa0), DAT(0xa0), DAT(0x05), DAT(0x2a),
		 DAT(0xa0), DAT(0xa0), DAT(0x07), DAT(0x2c), DAT(0xa0),
		 DAT(0xa0),
	CMD(0xe6), DAT(0x00), DAT(0x00), DAT(0x33), DAT(0x33),
	CMD(0xe7), DAT(0x44), DAT(0x44),
	CMD(0xe8), DAT(0x02), DAT(0x26), DAT(0xa0), DAT(0xa0), DAT(0x04),
		 DAT(0x28), DAT(0xa0), DAT(0xa0), DAT(0x06), DAT(0x2a),
		 DAT(0xa0), DAT(0xa0), DAT(0x08), DAT(0x2c), DAT(0xa0),
		 DAT(0xa0),
	CMD(0xeb), DAT(0x00), DAT(0x00), DAT(0xe4), DAT(0xe4), DAT(0x44),
		 DAT(0x00), DAT(0x40),
	CMD(0xed), DAT(0xff), DAT(0xf7), DAT(0x65), DAT(0x4f), DAT(0x0b),
		 DAT(0xa1), DAT(0xcf), DAT(0xff), DAT(0xff), DAT(0xfc),
		 DAT(0x1a), DAT(0xb0), DAT(0xf4), DAT(0x56), DAT(0x7f),
		 DAT(0xff),
	CMD(0xef), DAT(0x08), DAT(0x08), DAT(0x08), DAT(0x45), DAT(0x3f),
		 DAT(0x54),
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x13),
	CMD(0xe8), DAT(0x00), DAT(0x0e),
	CMD(0x3a), DAT(0x66),
	CMD(0x36), DAT(0x08),
	CMD(0x11),
	DELAY(120),
	CMD(0xe8), DAT(0x00), DAT(0x0c),
	DELAY(20),
	CMD(0xe8), DAT(0x00), DAT(0x00),
	CMD(0xe6), DAT(0x16), DAT(0x7c),
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x00),
	CMD(0x29),
};

static ssize_t config_read_proc(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	printk("reg[%04x] = [%02x]\n", reg_map.reg, reg_map.val);
	return 0;
}

// "REG VAL" in hex; a REG other than reg_map's, or a line without both, is
// ignored.
static ssize_t config_write_proc(struct file *file, const char __user *buf, size_t count,
				 loff_t *ppos)
{
	char kbuf[128];
	unsigned int reg, val;
	size_t n = min(count, sizeof(kbuf) - 1);

	if (copy_from_user(kbuf, buf, n))
		return -EFAULT;
	kbuf[n] = '\0';

	if (sscanf(kbuf, "%x %x", &reg, &val) == 2 && (u16)reg == reg_map.reg) {
		reg_map.val = val;
		printk("set reg[%04x] = [%02x]\n", (u16)reg, (u8)val);
	}
	return count;
}

static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = config_read_proc,
	.write = config_write_proc,
};

static void lg35583_init_panel(void)
{
	const struct lg_write *w;
	u8 val;

	for (w = lg35583_init_seq; w < lg35583_init_seq + ARRAY_SIZE(lg35583_init_seq); w++) {
		if (!w->reg) {
			usleep_range(w->val * 1000, w->val * 1000);
			continue;
		}
		spi_writecomm(w->reg);
		if (w->val < 0)
			continue;
		val = w->reg == reg_map.reg ? reg_map.val : w->val;
		m_spi_write(LG_DATA | val);
	}
}

static void st7701s_init_panel(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st7701s_init_seq); i++) {
		u16 v = st7701s_init_seq[i];

		if (v & 0x8000)
			usleep_range((v & 0x7fff) * 1000, (v & 0x7fff) * 1000);
		else
			m_spi_write_addr_8(v);
	}
}

static int lg35583_power_on(void *unused)
{
	gpio_set_value(gpio_spi_cs, 1);
	if (gpio_lcd_power_en >= 0)
		gpio_set_value(gpio_lcd_power_en, 1);

	if (vccio_regulator) {
		regulator_enable(vccio_regulator);
		usleep_range(5000, 5000);
	}
	if (vcc_regulator) {
		regulator_enable(vcc_regulator);
		usleep_range(5000, 5000);
	}

	if (gpio_lcd_rst >= 0) {
		gpio_set_value(gpio_lcd_rst, 1);
		usleep_range(5000, 5000);
	}
	if (gpio_lcd_rst >= 0) {
		gpio_set_value(gpio_lcd_rst, 0);
		usleep_range(50000, 50000);
	}
	if (gpio_lcd_rst >= 0) {
		gpio_set_value(gpio_lcd_rst, 1);
		usleep_range(30000, 30000);
	}

	if (lcd_id)
		st7701s_init_panel();
	else
		lg35583_init_panel();

	if (!lcd_reg_done) {
		lcd_reg_entry = proc_create_data("lcd_reg", 0666, NULL, &config_proc_ops, NULL);
		if (lcd_reg_entry)
			printk("create proc entry %s success", "lcd_reg");
		else
			printk("create_proc_entry %s failed\n", "lcd_reg");
		lcd_reg_done = true;
	}
	return 0;
}

static int lg35583_power_off(void *unused)
{
	if (!lcd_id) {
		spi_writecomm(0x2800);		// Display Off
		usleep_range(5000, 5000);
		spi_writecomm(0x1000);		// Sleep In
		usleep_range(5000, 5000);
	}

	if (gpio_lcd_rst >= 0)
		gpio_set_value(gpio_lcd_rst, 0);
	if (vccio_regulator)
		regulator_disable(vccio_regulator);
	if (vcc_regulator)
		regulator_disable(vcc_regulator);
	if (gpio_lcd_power_en >= 0)
		gpio_set_value(gpio_lcd_power_en, 0);

	gpio_set_value(gpio_spi_mosi, 0);
	gpio_set_value(gpio_spi_sck, 0);
	gpio_set_value(gpio_spi_cs, 0);

	return 0;
}

struct lcdc_data lcdc_data = {
	.name = "lg35583",
	.refresh = 62,
	.xres = 480,
	.yres = 800,
	.left_margin = 24,
	.right_margin = 24,
	.upper_margin = 8,
	.lower_margin = 14,
	.hsync_len = 24,
	.vsync_len = 5,
	.fb_fmt = 1,
	.lcd_mode = 0,
	.out_format = 1,
	.color_even = 2,
	.color_odd = 2,
	.pix_clk_active = 1,
	.de_active_level = 1,
	.hsync_active_level = 1,
	.vsync_active_level = 1,
	.power_on = lg35583_power_on,
	.power_off = lg35583_power_off,
};

static bool regulator_named(const char *name)
{
	return strlen(name) && strcmp(name, "-1");
}

static int request_pin(int gpio, const char *label)
{
	char pin[24];
	int ret;

	ret = gpio_request(gpio, label);
	if (ret)
		printk(KERN_ERR "lg35583: failed to request: %s\n", gpio_to_str(gpio, pin));
	return ret;
}

static int __init lg35583_init(void)
{
	int ret;

	if (gpio_spi_cs < 0 || gpio_spi_sck < 0 || gpio_spi_mosi < 0) {
		printk(KERN_ERR "lg35583: gpio_spi_cs, gpio_spi_sck, gpio_spi_mosi is not defined!\n");
		return -EINVAL;
	}

	if (gpio_lcd_power_en >= 0) {
		ret = request_pin(gpio_lcd_power_en, "lcd_power_en");
		if (ret)
			return ret;
		gpio_direction_output(gpio_lcd_power_en, 0);
	}

	// The supplies stay off until power_on.
	if (regulator_named(vcc_regulator_name)) {
		vcc_regulator = regulator_get(NULL, vcc_regulator_name);
		if (IS_ERR(vcc_regulator)) {
			printk(KERN_ERR "regulator_get vcc error!\n");
			vcc_regulator = NULL;
			ret = -1;
			goto err_power_en;
		}
		regulator_force_disable(vcc_regulator);
	}
	if (regulator_named(vccio_regulator_name)) {
		vccio_regulator = regulator_get(NULL, vccio_regulator_name);
		if (IS_ERR(vccio_regulator)) {
			printk(KERN_ERR "regulator_get vccio error!\n");
			vccio_regulator = NULL;
			ret = -1;
			goto err_regulators;
		}
		regulator_force_disable(vccio_regulator);
	}

	if (gpio_lcd_rst >= 0) {
		ret = request_pin(gpio_lcd_rst, "lcd_rst");
		if (ret)
			goto err_regulators;
		gpio_direction_output(gpio_lcd_rst, 0);
	}
	ret = request_pin(gpio_spi_cs, "lcd_spi_cs");
	if (ret)
		goto err_rst;
	ret = request_pin(gpio_spi_sck, "lcd_spi_sck");
	if (ret)
		goto err_cs;
	ret = request_pin(gpio_spi_mosi, "lcd_spi_mosi");
	if (ret)
		goto err_sck;

	adc_enable();
	lcd_id_vol = adc_read_channel_voltage(LCD_ID_CHANNEL);
	adc_disable();
	lcd_id = lcd_id_vol >= LCD_ID_ST7701S_MV;
	printk("LCD: lcd_id_vol:%d lcd_id:%d\n", lcd_id_vol, lcd_id);

	gpio_direction_output(gpio_spi_cs, 1);
	gpio_direction_output(gpio_spi_sck, 0);
	gpio_direction_output(gpio_spi_mosi, 0);

	jzfb_register_lcd(&lcdc_data);
	return 0;

err_sck:
	gpio_free(gpio_spi_sck);
err_cs:
	gpio_free(gpio_spi_cs);
err_rst:
	if (gpio_lcd_rst >= 0)
		gpio_free(gpio_lcd_rst);
err_regulators:
	if (vcc_regulator) {
		regulator_put(vcc_regulator);
		vcc_regulator = NULL;
	}
	if (vccio_regulator) {
		regulator_put(vccio_regulator);
		vccio_regulator = NULL;
	}
err_power_en:
	if (gpio_lcd_power_en >= 0)
		gpio_free(gpio_lcd_power_en);
	return ret;
}
module_init(lg35583_init);

static void __exit lg35583_exit(void)
{
	proc_remove(lcd_reg_entry);
	gpio_free(gpio_spi_mosi);
	gpio_free(gpio_spi_sck);
	gpio_free(gpio_spi_cs);
	if (gpio_lcd_rst >= 0)
		gpio_free(gpio_lcd_rst);
	if (gpio_lcd_power_en >= 0)
		gpio_free(gpio_lcd_power_en);
	if (vcc_regulator) {
		regulator_put(vcc_regulator);
		vcc_regulator = NULL;
	}
	if (vccio_regulator) {
		regulator_put(vccio_regulator);
		vccio_regulator = NULL;
	}
}
module_exit(lg35583_exit);

MODULE_DESCRIPTION("lg35583 lcd panel driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
