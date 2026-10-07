// SPDX-License-Identifier: GPL-2.0
//
// leds_sgm31324_add -- the RGB LED of the HiBy R3 Pro II, an SGMICRO
// SGM31324 on I2C (address 0x30), plus an optional white LED on a pin.
//
// A drop-in replacement for the vendor module of the same name. It is set up
// through its one parameter, sgm31324, written as words (the stock
// leds_sgm31324_add.sh writes them one line at a time):
//
//   i2c_bus_num=N            bus of the controller
//   enable_gpio=PIN          the controller's enable pin; -1: none
//   enable_active_level=L
//   wled_gpio=PIN            the white LED; -1: none
//   wled_active_level=L
//   wled_pattern_id=N        the pattern that lights the white LED instead
//   alloc=N                  room for N patterns, once
//   regs=HEX                 a pattern: the controller's registers 0-9, two
//                            hex digits each; numbered from 1 in order
//   register                 registers the I2C device with what was set
//
// Reading the parameter lists the patterns as "regs=" lines.
//
// The LED is /sys/class/leds/sgm31324-leds; its led_pattern (0644) reads and
// selects the pattern: 0 is off, N writes pattern N to registers 0-9. The
// white LED's pattern turns the controller off and the pin on. Suspend and
// shutdown turn everything off, resume restores the pattern.
//
// Loaded after utils.ko, which provides i2c_register_device(), pin names such
// as "PF11" and the word splitting.

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>

// Exported by utils.ko.
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);
extern char **str_to_words(const char *str, int *count);
extern void str_free_words(char **words);
extern int str_to_gpio(const char *str);

#define SGM31324_REGS		10
#define SGM31324_ADDR		0x30
#define SGM31324_PINS		512	// pin numbers below this are pins

struct sgm31324_platform_data {
	int enable_gpio;
	int enable_active_level;
	int wled_gpio;
	int wled_active_level;
	int wled_pattern_id;
	int pattern_cnt;
	u8 (*patterns)[SGM31324_REGS];
};

// What register sends to the driver: the board info, its platform data and
// the patterns, in one allocation.
struct sgm31324_data {
	struct i2c_board_info info;
	struct sgm31324_platform_data pdata;
	u8 patterns[0][SGM31324_REGS];
};

struct sgm31324_led {
	struct i2c_client *client;
	struct sgm31324_platform_data *pdata;
	struct led_classdev cdev;
	unsigned long pattern;
};

static const u8 led_off_pattern[SGM31324_REGS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x06 };

static struct sgm31324_data *gp_sgm31324_data;
static int g_sgm31324_total_cnt;		// patterns allocated
static int g_sgm31324_pattern_cnt;		// patterns written since the last register
static int g_sgm31324_save_cnt;		// patterns handed to the last register
static int g_sgm31324_wled_active_level;
static int g_sgm31324_enable_active_level;
static int g_sgm31324_wled_pattern_id = -1;
static int g_sgm31324_wled_gpio = -1;
static int g_sgm31324_enable_gpio = -1;
static int g_sgm31324_i2c_bus_num = -1;
static struct i2c_client *gp_sgm31324_i2c_dev;

// The value of "key=..." in w, or NULL for another word.
static const char *sgm31324_arg(const char *w, const char *key)
{
	return strncmp(w, key, strlen(key)) ? NULL : w + strlen(key);
}

static int sgm31324_num(const char *w, const char *v, long *n)
{
	if (kstrtol(v, 0, n)) {
		pr_err("%s is not a valid num\n", w);
		return -EINVAL;
	}
	return 0;
}

static int sgm31324_gpio(const char *w, const char *v, int *gpio)
{
	int g = str_to_gpio(v);

	if (g < -1) {
		pr_err("%s is not a gpio\n", w);
		return -EINVAL;
	}
	*gpio = g;
	return 0;
}

// Two hex digits per register, spaces allowed between them; missing
// registers are 0.
static int sgm31324_regs(const char *v, u8 *regs)
{
	char digits[3];
	int len = strlen(v), used, i;
	unsigned int byte;

	for (i = 0; i < SGM31324_REGS; i++) {
		if (len <= 0) {
			for (; i < SGM31324_REGS; i++)
				regs[i] = 0;
			break;
		}
		if (sscanf(v, "%2s%n ", digits, &used) != 1 || sscanf(digits, "%2X", &byte) != 1) {
			pr_err("pattern number error, MUST BE HEX NUMBER!\n");
			return -EINVAL;
		}
		regs[i] = byte;
		v += used;
		len -= used;
	}
	return 0;
}

static int sgm31324_register(void)
{
	struct sgm31324_data *d = gp_sgm31324_data;

	if (g_sgm31324_i2c_bus_num < 0 || !g_sgm31324_pattern_cnt) {
		pr_err("can not register, i2c bus num is not set, or no pattern is set!\n");
		return -ENODEV;
	}
	d->pdata.pattern_cnt = g_sgm31324_pattern_cnt;
	d->pdata.enable_gpio = g_sgm31324_enable_gpio;
	d->pdata.patterns = d->patterns;
	d->pdata.enable_active_level = g_sgm31324_enable_active_level;
	d->pdata.wled_gpio = g_sgm31324_wled_gpio;
	d->pdata.wled_active_level = g_sgm31324_wled_active_level;
	d->pdata.wled_pattern_id = g_sgm31324_wled_pattern_id;
	strcpy(d->info.type, "sgm31324");
	d->info.addr = SGM31324_ADDR;
	d->info.platform_data = &d->pdata;
	g_sgm31324_save_cnt = g_sgm31324_pattern_cnt;
	g_sgm31324_pattern_cnt = 0;
	g_sgm31324_total_cnt = 0;
	gp_sgm31324_i2c_dev = i2c_register_device(&d->info, g_sgm31324_i2c_bus_num);
	if (!gp_sgm31324_i2c_dev) {
		pr_err("Failed to register sgm31324 i2c device\n");
		return -EINVAL;
	}
	return 0;
}

static int param_sgm31324_set(const char *val, const struct kernel_param *kp)
{
	bool do_alloc = false, do_regs = false, do_register = false;
	u8 regs[SGM31324_REGS];
	unsigned long count;
	const char *v, *w;
	char **words;
	int n, i, ret = 0;
	long num;

	words = str_to_words(val, &n);
	for (i = 0; i < n; i++) {
		w = words[i];
		if ((v = sgm31324_arg(w, "i2c_bus_num="))) {
			ret = sgm31324_num(w, v, &num);
			if (!ret)
				g_sgm31324_i2c_bus_num = num;
		} else if ((v = sgm31324_arg(w, "enable_gpio="))) {
			ret = sgm31324_gpio(w, v, &g_sgm31324_enable_gpio);
		} else if ((v = sgm31324_arg(w, "enable_active_level="))) {
			ret = sgm31324_num(w, v, &num);
			if (!ret)
				g_sgm31324_enable_active_level = num;
		} else if ((v = sgm31324_arg(w, "wled_gpio="))) {
			ret = sgm31324_gpio(w, v, &g_sgm31324_wled_gpio);
		} else if ((v = sgm31324_arg(w, "wled_active_level="))) {
			ret = sgm31324_num(w, v, &num);
			if (!ret)
				g_sgm31324_wled_active_level = num;
		} else if ((v = sgm31324_arg(w, "wled_pattern_id="))) {
			ret = sgm31324_num(w, v, &num);
			if (!ret)
				g_sgm31324_wled_pattern_id = num;
		} else if ((v = sgm31324_arg(w, "alloc="))) {
			if (kstrtoul(v, 0, &count)) {
				pr_err("%s is not a valid num\n", w);
				ret = -EINVAL;
			} else if (!count) {
				ret = -EINVAL;
			} else if (g_sgm31324_total_cnt) {
				pr_err("new alloc %ld can not be done, old patterns not register!\n", count);
				ret = -EINVAL;
			} else {
				g_sgm31324_total_cnt = count;
				do_alloc = true;
			}
		} else if ((v = sgm31324_arg(w, "regs="))) {
			ret = sgm31324_regs(v, regs);
			do_regs = true;
		} else if (!strcmp(w, "register")) {
			do_register = true;
			break;
		} else {
			pr_err("%s can not be parse!\n", w);
			ret = -EINVAL;
		}
		if (ret) {
			str_free_words(words);
			return -EINVAL;
		}
	}
	str_free_words(words);

	if (do_alloc) {
		gp_sgm31324_data = kzalloc(sizeof(*gp_sgm31324_data) +
					   g_sgm31324_total_cnt * SGM31324_REGS, GFP_KERNEL);
		if (!gp_sgm31324_data) {
			pr_err("failed to alloc %d keys\n", g_sgm31324_total_cnt);
			g_sgm31324_total_cnt = 0;
		}
		return 0;
	}
	if (do_register) {
		ret = sgm31324_register();
		if (ret)
			return ret;
	}
	if (do_regs) {
		if (g_sgm31324_pattern_cnt >= g_sgm31324_total_cnt) {
			pr_err("too many patterns, total:%d!\n", g_sgm31324_total_cnt);
			return -ERANGE;
		}
		memcpy(gp_sgm31324_data->patterns[g_sgm31324_pattern_cnt++], regs, SGM31324_REGS);
	}
	return 0;
}

static int param_sgm31324_get(char *buffer, const struct kernel_param *kp)
{
	int n = g_sgm31324_pattern_cnt ? g_sgm31324_pattern_cnt : g_sgm31324_save_cnt;
	char *p = buffer;
	int i, j;

	for (i = 0; i < n; i++) {
		p += sprintf(p, "regs=");
		for (j = 0; j < SGM31324_REGS; j++)
			p += sprintf(p, "%02x ", gp_sgm31324_data->patterns[i][j]);
		p += sprintf(p, "\n");
	}
	return p - buffer;
}

static const struct kernel_param_ops param_sgm31324_ops = {
	.set = param_sgm31324_set,
	.get = param_sgm31324_get,
};
module_param_cb(sgm31324, &param_sgm31324_ops, NULL, 0644);

static void sgm31324_set_pin(int gpio, int level)
{
	if ((unsigned int)gpio < SGM31324_PINS)
		gpio_set_value(gpio, level);
}

static int sgm31324_run_predef_led_pattern(struct sgm31324_led *led, unsigned long id)
{
	struct sgm31324_platform_data *pdata = led->pdata;
	const u8 *regs = led_off_pattern;
	int i;

	if (!id) {
		sgm31324_set_pin(pdata->enable_gpio, !pdata->enable_active_level);
		sgm31324_set_pin(pdata->wled_gpio, !pdata->wled_active_level);
	} else if ((unsigned int)pdata->wled_gpio < SGM31324_PINS && id == pdata->wled_pattern_id) {
		sgm31324_set_pin(pdata->wled_gpio, pdata->wled_active_level);
		sgm31324_set_pin(pdata->enable_gpio, !pdata->enable_active_level);
	} else {
		sgm31324_set_pin(pdata->wled_gpio, !pdata->wled_active_level);
		sgm31324_set_pin(pdata->enable_gpio, pdata->enable_active_level);
		if (!pdata->patterns) {
			dev_err(&led->client->dev, "invalid pattern data\n");
			return -EINVAL;
		}
		regs = pdata->patterns[id - 1];
	}

	for (i = 0; i < SGM31324_REGS; i++)
		i2c_smbus_write_byte_data(led->client, i, regs[i]);
	return 0;
}

// The LED has no brightness of its own; led_pattern sets it.
static void sgm31324_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
}

static ssize_t sgm31324_show_pattern(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct sgm31324_led *led = container_of(cdev, struct sgm31324_led, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", (int)led->pattern);
}

static ssize_t sgm31324_store_pattern(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct sgm31324_led *led = container_of(cdev, struct sgm31324_led, cdev);
	struct sgm31324_platform_data *pdata = led->pdata;
	unsigned long id;
	int ret;

	ret = kstrtoul(buf, 0, &id);
	if (ret)
		return ret;
	if (id > pdata->pattern_cnt || !pdata->patterns)
		return -EINVAL;
	if (id != led->pattern) {
		ret = sgm31324_run_predef_led_pattern(led, id);
		if (ret)
			return ret;
		led->pattern = id;
	}
	return count;
}

static DEVICE_ATTR(led_pattern, 0644, sgm31324_show_pattern, sgm31324_store_pattern);

static struct attribute *sgm31324_attrs[] = {
	&dev_attr_led_pattern.attr,
	NULL,
};

static const struct attribute_group sgm31324_led_attr_group = {
	.attrs = sgm31324_attrs,
};

static int sgm31324_request(struct device *dev, int gpio, const char *label, int level)
{
	int ret;

	if ((unsigned int)gpio >= SGM31324_PINS)
		return 0;
	ret = gpio_request(gpio, label);
	if (ret < 0) {
		dev_err(dev, "could not acquire enable gpio (err=%d)\n", ret);
		return ret;
	}
	gpio_direction_output(gpio, level);
	return 0;
}

static void sgm31324_free(struct sgm31324_platform_data *pdata)
{
	if ((unsigned int)pdata->enable_gpio < SGM31324_PINS)
		gpio_free(pdata->enable_gpio);
	if ((unsigned int)pdata->wled_gpio < SGM31324_PINS)
		gpio_free(pdata->wled_gpio);
}

static int sgm31324_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sgm31324_platform_data *pdata = dev_get_platdata(&client->dev);
	struct device *dev = &client->dev;
	struct sgm31324_led *led;
	int ret;

	if (!pdata) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	// Both start off.
	ret = sgm31324_request(dev, pdata->enable_gpio, "sgm31324_enable", !pdata->enable_active_level);
	if (ret)
		return ret;
	ret = sgm31324_request(dev, pdata->wled_gpio, "sgm31324_wled", !pdata->wled_active_level);
	if (ret)
		goto err_pins;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led) {
		ret = -ENOMEM;
		goto err_pins;
	}
	led->pdata = pdata;
	led->client = client;
	led->pattern = 0;
	led->cdev.name = "sgm31324-leds";
	led->cdev.brightness_set = sgm31324_led_brightness_set;
	led->cdev.max_brightness = LED_FULL;
	led->cdev.brightness = LED_OFF;
	i2c_set_clientdata(client, led);

	ret = led_classdev_register(dev, &led->cdev);
	if (ret < 0)
		goto err_pins;
	ret = sysfs_create_group(&led->cdev.dev->kobj, &sgm31324_led_attr_group);
	if (ret) {
		dev_err(dev, "led sysfs err: %d\n", ret);
		led_classdev_unregister(&led->cdev);
		goto err_pins;
	}
	return 0;

err_pins:
	sgm31324_free(pdata);
	return ret;
}

static int sgm31324_remove(struct i2c_client *client)
{
	struct sgm31324_led *led = i2c_get_clientdata(client);

	led_classdev_unregister(&led->cdev);
	sgm31324_free(led->pdata);
	return 0;
}

static void sgm31324_shutdown(struct i2c_client *client)
{
	sgm31324_run_predef_led_pattern(i2c_get_clientdata(client), 0);
}

static int sgm31324_suspend(struct device *dev)
{
	sgm31324_run_predef_led_pattern(dev_get_drvdata(dev), 0);
	return 0;
}

static int sgm31324_resume(struct device *dev)
{
	struct sgm31324_led *led = dev_get_drvdata(dev);

	sgm31324_run_predef_led_pattern(led, led->pattern);
	return 0;
}

static const struct dev_pm_ops pm_ops = {
	.suspend = sgm31324_suspend,
	.resume = sgm31324_resume,
};

static const struct i2c_device_id sgm31324_id[] = {
	{ "sgm31324", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sgm31324_id);

static const struct of_device_id of_sgm31324_leds_match[] = {
	{ .compatible = "sgm,sgm31324" },
	{ }
};
MODULE_DEVICE_TABLE(of, of_sgm31324_leds_match);

static struct i2c_driver sgm31324_i2c_driver = {
	.probe = sgm31324_probe,
	.remove = sgm31324_remove,
	.shutdown = sgm31324_shutdown,
	.id_table = sgm31324_id,
	.driver = {
		.name = "sgm31324",
		.of_match_table = of_sgm31324_leds_match,
		.pm = &pm_ops,
	},
};

static int __init sgm31324_init(void)
{
	int ret = i2c_add_driver(&sgm31324_i2c_driver);

	if (ret)
		pr_err("Failed to register sgm31324 I2C driver\n");
	return ret;
}

static void __exit sgm31324_exit(void)
{
	if (gp_sgm31324_i2c_dev)
		i2c_unregister_device(gp_sgm31324_i2c_dev);
	i2c_del_driver(&sgm31324_i2c_driver);
	kfree(gp_sgm31324_data);
}

module_init(sgm31324_init);
module_exit(sgm31324_exit);

MODULE_DESCRIPTION("SGMICRO SGM31324 LED of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
