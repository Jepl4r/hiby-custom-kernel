// SPDX-License-Identifier: GPL-2.0
//
// leds_pwm_add -- the PWM LEDs of the HiBy R1 (red and blue), as LED class
// devices driven by channels of soc_pwm.ko.
//
// Based on drivers/leds/leds-pwm.c, Copyright (C) 2009 Luotao Fu
// <l.fu@pengutronix.de>, with the PWM calls of soc_pwm.ko and the LEDs
// described by module parameters instead of platform data or a device tree.
//
// A drop-in replacement for the vendor module of the same name. Up to five
// LEDs, n = 0..4, with the same parameters (the stock leds_pwm_add.sh passes
// them):
//
//   led_pwmN_gpio             PWM output pin, which picks the channel;
//                             -1: no LED N
//   led_pwmN_name             name under /sys/class/leds
//   led_pwmN_trigger          default trigger
//   led_pwmN_max_brightness   brightness steps, also the PWM levels
//   led_pwmN_period_ns        PWM period; 0: 10 kHz
//
// Loaded after soc_pwm.ko, which provides the pwm2_* calls, and utils.ko.

#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

// The channel configuration pwm2_config() reads, laid out as soc_pwm.ko
// expects it.
struct pwm2_config {
	int reserved;			// not read by pwm2_config()
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

#define LED_PWM_MAX		5
#define LED_PWM_NAME_LEN	32
#define LED_PWM_DEFAULT_FREQ	10000

struct led_pwm_param {
	int gpio;
	char name[LED_PWM_NAME_LEN];
	char trigger[LED_PWM_NAME_LEN];
	int max_brightness;
	int period_ns;
};

static struct led_pwm_param leds[LED_PWM_MAX] = {
	{ .gpio = -1 }, { .gpio = -1 }, { .gpio = -1 }, { .gpio = -1 }, { .gpio = -1 },
};

#define LED_PWM_PARAMS(n)								\
	module_param_cb(led_pwm##n##_gpio, &param_gpio_ops, &leds[n].gpio, 0644);	\
	module_param_string(led_pwm##n##_name, leds[n].name, LED_PWM_NAME_LEN, 0644);	\
	module_param_string(led_pwm##n##_trigger, leds[n].trigger, LED_PWM_NAME_LEN, 0644); \
	module_param_named(led_pwm##n##_max_brightness, leds[n].max_brightness, int, 0644); \
	module_param_named(led_pwm##n##_period_ns, leds[n].period_ns, int, 0644)

LED_PWM_PARAMS(0);
LED_PWM_PARAMS(1);
LED_PWM_PARAMS(2);
LED_PWM_PARAMS(3);
LED_PWM_PARAMS(4);

struct led_pwm_data {
	struct led_classdev cdev;
	int pwm;
	struct work_struct work;
	unsigned int duty;
	struct pwm2_config config;
};

struct led_pwm_priv {
	int num_leds;
	struct led_pwm_data leds[0];
};

// brightness_set may be called in atomic context, the level is set from a
// work item.
static void led_pwm_work(struct work_struct *work)
{
	struct led_pwm_data *led_dat = container_of(work, struct led_pwm_data, work);

	pwm2_set_level(led_dat->pwm, led_dat->duty);
}

static void led_pwm_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct led_pwm_data *led_dat = container_of(led_cdev, struct led_pwm_data, cdev);

	led_dat->duty = brightness;
	schedule_work(&led_dat->work);
}

static void led_pwm_cleanup(struct led_pwm_priv *priv)
{
	while (priv->num_leds--) {
		struct led_pwm_data *led_dat = &priv->leds[priv->num_leds];

		led_classdev_unregister(&led_dat->cdev);
		cancel_work_sync(&led_dat->work);
		pwm2_release(led_dat->pwm);
	}
}

static int led_pwm_add(struct device *dev, struct led_pwm_priv *priv,
		       struct led_pwm_param *led)
{
	struct led_pwm_data *led_dat = &priv->leds[priv->num_leds];
	unsigned int period = led->period_ns;
	unsigned long freq;
	int ret;

	led_dat->cdev.name = led->name;
	led_dat->cdev.default_trigger = led->trigger;
	led_dat->cdev.brightness_set = led_pwm_set;
	led_dat->cdev.brightness = LED_OFF;
	led_dat->cdev.max_brightness = led->max_brightness;
	led_dat->cdev.flags = LED_CORE_SUSPENDRESUME;

	led_dat->pwm = pwm2_request(led->gpio, led->name);
	if (led_dat->pwm < 0) {
		pr_err("unable to request PWM for %s: %d\n", led->name, led_dat->pwm);
		return led_dat->pwm;
	}

	freq = period ? NSEC_PER_SEC / period : 0;
	led_dat->config.freq = freq ? freq : LED_PWM_DEFAULT_FREQ;
	led_dat->config.levels = led->max_brightness;
	led_dat->config.idle_level = 0;
	pwm2_config(led_dat->pwm, &led_dat->config);

	INIT_WORK(&led_dat->work, led_pwm_work);

	ret = led_classdev_register(dev, &led_dat->cdev);
	if (ret) {
		dev_err(dev, "failed to register PWM led for %s: %d\n", led->name, ret);
		pwm2_release(led_dat->pwm);
		return ret;
	}
	priv->num_leds++;
	return 0;
}

static int led_pwm_probe(struct platform_device *pdev)
{
	struct led_pwm_priv *priv;
	int i, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv) +
			    LED_PWM_MAX * sizeof(struct led_pwm_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	for (i = 0; i < LED_PWM_MAX; i++) {
		if (leds[i].gpio == -1)
			continue;
		ret = led_pwm_add(&pdev->dev, priv, &leds[i]);
		if (ret) {
			led_pwm_cleanup(priv);
			return ret;
		}
	}

	platform_set_drvdata(pdev, priv);
	return 0;
}

static int led_pwm_remove(struct platform_device *pdev)
{
	led_pwm_cleanup(platform_get_drvdata(pdev));
	return 0;
}

static void leds_pwm_dev_release(struct device *dev)
{
}

static struct platform_device leds_pwm_dev = {
	.name = "leds_pwm2",
	.id = -1,
	.dev = {
		.release = leds_pwm_dev_release,
	},
};

static struct platform_driver led_pwm_driver = {
	.probe = led_pwm_probe,
	.remove = led_pwm_remove,
	.driver = {
		.name = "leds_pwm2",
	},
};

static int __init leds_pwm_dev_init(void)
{
	int ret;

	ret = platform_device_register(&leds_pwm_dev);
	if (ret) {
		pr_err("leds pwm device init error.\n");
		return ret;
	}
	return platform_driver_register(&led_pwm_driver);
}

static void __exit leds_pwm_dev_exit(void)
{
	platform_device_unregister(&leds_pwm_dev);
	platform_driver_unregister(&led_pwm_driver);
}

module_init(leds_pwm_dev_init);
module_exit(leds_pwm_dev_exit);

MODULE_ALIAS("led:pwm");
MODULE_DESCRIPTION("PWM LEDs of the HiBy R1");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
