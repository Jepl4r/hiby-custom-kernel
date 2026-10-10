// SPDX-License-Identifier: GPL-2.0
//
// lcd_st7701_sbtc033001 -- the 480x720 RGB panel of the HiBy R3 Pro II, an
// ST7701S set up over a bit-banged 3-wire SPI and registered with soc_fb.ko
// as its panel.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock lcd_st7701_sbtc033001.sh passes them):
//
//   gpio_spi_cs, gpio_spi_sck, gpio_spi_mosi   the SPI pins (required)
//   gpio_lcd_rst          reset, active low (-1: none)
//   gpio_lcd_power_en     power switch, active high (-1: none)
//   vcc_regulator_name, vccio_regulator_name   the panel supplies ("" or
//                         "-1": none)
//   spi_bus_num           not read
//
// soc_fb calls power_on when the screen turns on and power_off when it turns
// off. Unloading leaves the panel registered: jzfb_unregister_lcd() would
// take the framebuffer down with it.
//
// Loaded after utils.ko and soc_fb.ko.

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>

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

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern char *gpio_to_str(int gpio, char *buf);

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

// One 9-bit word, MSB first: bit 8 is 0 for a command, 1 for data. The
// panel samples MOSI on the rising edge of SCK.
static void m_spi_write(u16 val)
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

#define CMD(c)	(c)
#define DAT(d)	(0x100 | (d))

// The panel set-up, up to Sleep Out (0x11). Display On (0x29) follows after
// the 120 ms the panel needs to leave sleep.
static const u16 st7701s_init_seq[] = {
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x13),
	CMD(0xef), DAT(0x08),
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x10),
	CMD(0xc0), DAT(0x59), DAT(0x00),
	CMD(0xc1), DAT(0x10), DAT(0x0c),
	CMD(0xc2), DAT(0x17), DAT(0x0a),
	CMD(0xc3), DAT(0x02),
	CMD(0xcc), DAT(0x10),
	CMD(0xb0), DAT(0x0f), DAT(0x15), DAT(0x1d), DAT(0x10), DAT(0x13),
		 DAT(0x07), DAT(0x0b), DAT(0x08), DAT(0x08), DAT(0x26),
		 DAT(0x05), DAT(0x11), DAT(0x0f), DAT(0x2b), DAT(0x32),
		 DAT(0x1f),
	CMD(0xb1), DAT(0x0f), DAT(0x1c), DAT(0x21), DAT(0x0b), DAT(0x0f),
		 DAT(0x05), DAT(0x0c), DAT(0x09), DAT(0x08), DAT(0x24),
		 DAT(0x04), DAT(0x13), DAT(0x11), DAT(0x28), DAT(0x30),
		 DAT(0x1f),
	CMD(0xff), DAT(0x77), DAT(0x01), DAT(0x00), DAT(0x00), DAT(0x11),
	CMD(0xb0), DAT(0x4d),
	CMD(0xb1), DAT(0x4b),
	CMD(0xb2), DAT(0x81),
	CMD(0xb3), DAT(0x80),
	CMD(0xb5), DAT(0x4e),
	CMD(0xb7), DAT(0x85),
	CMD(0xb8), DAT(0x20),
	CMD(0xc1), DAT(0x78),
	CMD(0xc2), DAT(0x78),
	CMD(0xd0), DAT(0x88),
	CMD(0xc0), DAT(0x07),
	CMD(0xe0), DAT(0x00), DAT(0x00), DAT(0x02),
	CMD(0xe1), DAT(0x06), DAT(0x30), DAT(0x08), DAT(0x30), DAT(0x05),
		 DAT(0x30), DAT(0x07), DAT(0x30), DAT(0x00), DAT(0x33),
		 DAT(0x33),
	CMD(0xe2), DAT(0x22), DAT(0x22), DAT(0x33), DAT(0x33), DAT(0xe4),
		 DAT(0x00), DAT(0x00), DAT(0x00), DAT(0xe4), DAT(0x00),
		 DAT(0x00), DAT(0x00),
	CMD(0xe3), DAT(0x00), DAT(0x00), DAT(0x22), DAT(0x22),
	CMD(0xe4), DAT(0x44), DAT(0x44),
	CMD(0xe5), DAT(0x0d), DAT(0xe3), DAT(0x30), DAT(0xe6), DAT(0x0f),
		 DAT(0xe5), DAT(0x30), DAT(0xe6), DAT(0x09), DAT(0xdf),
		 DAT(0x30), DAT(0xe6), DAT(0x0b), DAT(0xe1), DAT(0x30),
		 DAT(0xe6),
	CMD(0xe6), DAT(0x00), DAT(0x00), DAT(0x22), DAT(0x22),
	CMD(0xe7), DAT(0x44), DAT(0x44),
	CMD(0xe8), DAT(0x0c), DAT(0xe2), DAT(0x30), DAT(0xe6), DAT(0x0e),
		 DAT(0xe4), DAT(0x30), DAT(0xe6), DAT(0x08), DAT(0xde),
		 DAT(0x30), DAT(0xe6), DAT(0x0a), DAT(0xe0), DAT(0x30),
		 DAT(0xe6),
	CMD(0xe9), DAT(0x36), DAT(0x01),
	CMD(0xeb), DAT(0x00), DAT(0x01), DAT(0xe4), DAT(0xe4), DAT(0x44),
		 DAT(0x88), DAT(0x40),
	CMD(0xed), DAT(0xff), DAT(0x56), DAT(0x7f), DAT(0xab), DAT(0x01),
		 DAT(0x42), DAT(0xcf), DAT(0xff), DAT(0xff), DAT(0xfc),
		 DAT(0x24), DAT(0x10), DAT(0xba), DAT(0xf7), DAT(0x65),
		 DAT(0xff),
	CMD(0xef), DAT(0x10), DAT(0x0d), DAT(0x04), DAT(0x08), DAT(0x3f),
		 DAT(0x1f),
	CMD(0x35), DAT(0x00),
	CMD(0x3a), DAT(0x66),
	CMD(0x11),
};

static int st7701s_power_on(void *unused)
{
	int i;

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

	for (i = 0; i < ARRAY_SIZE(st7701s_init_seq); i++)
		m_spi_write(st7701s_init_seq[i]);
	usleep_range(120000, 120000);
	m_spi_write(CMD(0x29));

	return 0;
}

static int st7701s_power_off(void *unused)
{
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

static struct lcdc_data lcdc_data = {
	.name = "st7701",
	.refresh = 62,
	.xres = 480,
	.yres = 720,
	.left_margin = 40,
	.right_margin = 80,
	.upper_margin = 40,
	.lower_margin = 40,
	.hsync_len = 10,
	.vsync_len = 10,
	.fb_fmt = 1,
	.lcd_mode = 0,
	.out_format = 1,
	.pix_clk_active = 1,
	.de_active_level = 1,
	.hsync_active_level = 1,
	.vsync_active_level = 1,
	.power_on = st7701s_power_on,
	.power_off = st7701s_power_off,
};

static bool regulator_named(const char *name)
{
	return strlen(name) && strcmp(name, "-1");
}

static int request_pin(int gpio, const char *label, const char *fmt)
{
	char pin[24];
	int ret;

	ret = gpio_request(gpio, label);
	if (ret)
		printk(fmt, gpio_to_str(gpio, pin));
	return ret;
}

static int __init st7701s_init(void)
{
	static const char err_fmt[] = KERN_ERR "st7701s: failed to request: %s\n";
	int ret;

	if (gpio_spi_cs < 0 || gpio_spi_sck < 0 || gpio_spi_mosi < 0) {
		printk(KERN_ERR "st7701s: gpio_spi_cs, gpio_spi_sck, gpio_spi_mosi is not defined!\n");
		return -EINVAL;
	}

	if (gpio_lcd_power_en >= 0) {
		ret = request_pin(gpio_lcd_power_en, "lcd_power_en", err_fmt);
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
		ret = request_pin(gpio_lcd_rst, "lcd_rst", err_fmt);
		if (ret)
			goto err_regulators;
		gpio_direction_output(gpio_lcd_rst, 0);
	}
	ret = request_pin(gpio_spi_cs, "lcd_spi_cs", err_fmt);
	if (ret)
		goto err_rst;
	ret = request_pin(gpio_spi_sck, "lcd_spi_sck", err_fmt);
	if (ret)
		goto err_cs;
	ret = request_pin(gpio_spi_mosi, "lcd_spi_mosi", "st7701s: failed to request: %s\n");
	if (ret)
		goto err_sck;
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
module_init(st7701s_init);

static void __exit st7701s_exit(void)
{
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
module_exit(st7701s_exit);

MODULE_DESCRIPTION("ST7701S panel (SBTC033001) of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
