// SPDX-License-Identifier: GPL-2.0
//
// pwm_backlight -- the display backlight of the HiBy X1600 players, driven by
// a PWM channel of soc_pwm.ko and an optional power GPIO.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock pwm_backlight.sh passes them):
//
//   backlight_dev_name       name of the backlight class device
//   pwm_gpio                 PWM output pin, which picks the channel
//   pwm_active_level         1: the level sets the high time
//   pwm_freq                 Hz
//   max_brightness           brightness steps
//   default_brightness       brightness at load
//   power_gpio               backlight power enable; -1: none
//   power_gpio_vaild_level   level of power_gpio while lit
//
// Brightness 0, a blanked framebuffer or a powered-down device turn the PWM
// to 0 and power_gpio off.
//
// Loaded after soc_pwm.ko, which provides the pwm2_* calls, and utils.ko.

#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>

// The channel configuration pwm2_config() reads, laid out as soc_pwm.ko
// expects it.
struct pwm2_config {
	int reserved;			// 1, not read by pwm2_config()
	int idle_level;			// output level while the channel is idle
	int exact_levels;		// nonzero: the period is a whole number of levels
	unsigned long freq;		// Hz
	unsigned long levels;		// the range of pwm2_set_level(), below 0xffff
};

// Exported by soc_pwm.ko: a channel is an index, negative on failure.
extern int pwm2_request(int gpio, const char *name);
extern int pwm2_config(int pwm, struct pwm2_config *config);
extern int pwm2_set_level(int pwm, unsigned int level);
extern int pwm2_release(int pwm);

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern char *gpio_to_str(int gpio, char *buf);

static struct {
	int pwm;
	int pwm_gpio;
	int pwm_active_level;
	int power_gpio;
	int power_gpio_vaild_level;
	int default_brightness;
	char *backlight_dev_name;
	struct backlight_device *bd;
	struct pwm2_config config;
} backlight_data = {
	.config = {
		.reserved = 1,
		.exact_levels = 1,
		.freq = 1000000,
		.levels = 300,
	},
};

module_param_named(max_brightness, backlight_data.config.levels, ulong, 0644);
module_param_named(pwm_freq, backlight_data.config.freq, ulong, 0644);
module_param_named(pwm_active_level, backlight_data.pwm_active_level, int, 0644);
module_param_named(power_gpio_vaild_level, backlight_data.power_gpio_vaild_level, int, 0644);
module_param_cb(power_gpio, &param_gpio_ops, &backlight_data.power_gpio, 0644);
module_param_named(backlight_dev_name, backlight_data.backlight_dev_name, charp, 0644);
module_param_named(default_brightness, backlight_data.default_brightness, int, 0644);
module_param_cb(pwm_gpio, &param_gpio_ops, &backlight_data.pwm_gpio, 0644);

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK || bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (brightness) {
		pwm2_set_level(backlight_data.pwm, brightness);
		if (backlight_data.power_gpio >= 0)
			gpio_direction_output(backlight_data.power_gpio,
					      backlight_data.power_gpio_vaild_level);
	} else {
		pwm2_set_level(backlight_data.pwm, 0);
		if (backlight_data.power_gpio >= 0)
			gpio_direction_output(backlight_data.power_gpio,
					      !backlight_data.power_gpio_vaild_level);
	}
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops pwm_backlight_backlight_ops = {
	.update_status = pwm_backlight_update_status,
	.get_brightness = pwm_backlight_get_brightness,
};

static int __init pwm_backlight_drv_init(void)
{
	struct backlight_properties props;
	struct backlight_device *bd;
	char pin[24];
	int ret;

	if (backlight_data.power_gpio >= 0) {
		ret = gpio_request(backlight_data.power_gpio, backlight_data.backlight_dev_name);
		if (ret) {
			pr_err("pwm_backlight: failed to request %s: %s\n",
			       backlight_data.backlight_dev_name,
			       gpio_to_str(backlight_data.power_gpio, pin));
			return ret;
		}
	}

	backlight_data.pwm = pwm2_request(backlight_data.pwm_gpio, backlight_data.backlight_dev_name);
	backlight_data.config.idle_level = !backlight_data.pwm_active_level;
	pwm2_config(backlight_data.pwm, &backlight_data.config);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = backlight_data.config.levels;
	bd = backlight_device_register(backlight_data.backlight_dev_name, NULL, NULL,
				       &pwm_backlight_backlight_ops, &props);
	if (IS_ERR(bd)) {
		pr_err("pwm_backlight %s failed to register backlight\n",
		       backlight_data.backlight_dev_name);
		if (backlight_data.power_gpio >= 0)
			gpio_free(backlight_data.power_gpio);
		return PTR_ERR(bd);
	}

	bd->props.brightness = backlight_data.default_brightness;
	backlight_data.bd = bd;
	backlight_update_status(bd);
	return 0;
}

static void __exit pwm_backlight_drv_exit(void)
{
	backlight_device_unregister(backlight_data.bd);
	pwm2_set_level(backlight_data.pwm, 0);
	pwm2_release(backlight_data.pwm);
	if (backlight_data.power_gpio >= 0)
		gpio_free(backlight_data.power_gpio);
}

module_init(pwm_backlight_drv_init);
module_exit(pwm_backlight_drv_exit);

MODULE_DESCRIPTION("pwm_backlight driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
