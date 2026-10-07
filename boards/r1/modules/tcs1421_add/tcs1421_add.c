// SPDX-License-Identifier: GPL-2.0
//
// tcs1421_add -- the TCS1421 USB Type-C port controller of the HiBy R1, set
// through its two configuration pins.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock tcs1421.sh passes them):
//
//   tcs1421_cfg0_gpio   CFG0 pin; -1: none
//   tcs1421_cfg1_gpio   CFG1 pin; -1: none
//
// The port role is read and written as a word on
// /sys/devices/platform/tcs1421/tcs1421_cfg (0600):
//
//   word        CFG0 CFG1
//   Sink          0    0
//   Source        1    0
//   StrongDRP     0    1
//   NormalDRP     1    1   also any other word; the role at probe
//
// Suspend switches the pins to Source without changing the word, resume
// applies the word again.
//
// Loaded after utils.ko, which parses pin names such as "PA09".

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;

#define TCS1421_MODE_LEN	20

struct tcs1421_plat_data {
	int cfg0_gpio;
	int cfg1_gpio;
};

struct tcs1421_priv {
	char mode[TCS1421_MODE_LEN];
	int cfg0_gpio;
	int cfg1_gpio;
};

static struct tcs1421_plat_data tcs1421_plat_data = {
	.cfg0_gpio = -1,
	.cfg1_gpio = -1,
};

module_param_cb(tcs1421_cfg0_gpio, &param_gpio_ops, &tcs1421_plat_data.cfg0_gpio, 0644);
module_param_cb(tcs1421_cfg1_gpio, &param_gpio_ops, &tcs1421_plat_data.cfg1_gpio, 0644);

static const struct {
	const char *name;
	int cfg0;
	int cfg1;
} tcs1421_modes[] = {
	{ "Sink", 0, 0 },
	{ "Source", 1, 0 },
	{ "StrongDRP", 0, 1 },
	{ "NormalDRP", 1, 1 },
};

// Drives the pins for the word; save also keeps it as the current role.
static void tcs1421_set_cfg(struct tcs1421_priv *priv, const char *mode, int save)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tcs1421_modes) - 1; i++)
		if (!strcmp(mode, tcs1421_modes[i].name))
			break;
	if (save)
		strcpy(priv->mode, tcs1421_modes[i].name);
	if (priv->cfg0_gpio >= 0)
		gpio_set_value(priv->cfg0_gpio, tcs1421_modes[i].cfg0);
	if (priv->cfg1_gpio >= 0)
		gpio_set_value(priv->cfg1_gpio, tcs1421_modes[i].cfg1);
}

static ssize_t tcs1421_cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tcs1421_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%s", priv->mode);
}

static ssize_t tcs1421_cfg_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct tcs1421_priv *priv = dev_get_drvdata(dev);
	char mode[TCS1421_MODE_LEN];

	if (sscanf(buf, "%19s", mode) != 1)
		return -EINVAL;
	tcs1421_set_cfg(priv, mode, 1);
	return count;
}

static DEVICE_ATTR(tcs1421_cfg, 0600, tcs1421_cfg_show, tcs1421_cfg_store);

static int tcs1421_suspend(struct device *dev)
{
	struct tcs1421_priv *priv = dev_get_drvdata(dev);

	printk("tcs1421_suspend\n");
	tcs1421_set_cfg(priv, "Source", 0);
	return 0;
}

static int tcs1421_resume(struct device *dev)
{
	struct tcs1421_priv *priv = dev_get_drvdata(dev);

	printk("tcs1421_resume\n");
	tcs1421_set_cfg(priv, priv->mode, 0);
	return 0;
}

static SIMPLE_DEV_PM_OPS(tcs1421_pm_ops, tcs1421_suspend, tcs1421_resume);

static int tcs1421_request(int gpio, const char *label)
{
	int ret;

	if (gpio < 0)
		return 0;
	ret = gpio_request(gpio, label);
	if (ret < 0) {
		printk("failed to request GPIO %d: %d\n", gpio, ret);
		return ret;
	}
	gpio_direction_output(gpio, 0);
	return 0;
}

static void tcs1421_free(struct tcs1421_priv *priv)
{
	if (priv->cfg0_gpio >= 0)
		gpio_free(priv->cfg0_gpio);
	if (priv->cfg1_gpio >= 0)
		gpio_free(priv->cfg1_gpio);
}

static int tcs1421_probe(struct platform_device *pdev)
{
	struct tcs1421_plat_data *pdata = dev_get_platdata(&pdev->dev);
	struct tcs1421_priv *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		printk("priv_data: malloc faile\n");
		return -ENOMEM;
	}
	priv->cfg0_gpio = pdata->cfg0_gpio;
	priv->cfg1_gpio = pdata->cfg1_gpio;

	ret = tcs1421_request(priv->cfg0_gpio, "tcs1421_cfg0");
	if (ret)
		goto err_free;
	ret = tcs1421_request(priv->cfg1_gpio, "tcs1421_cfg1");
	if (ret) {
		if (priv->cfg0_gpio >= 0)
			gpio_free(priv->cfg0_gpio);
		goto err_free;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_tcs1421_cfg);
	if (ret) {
		printk("device_create_file error\n");
		tcs1421_free(priv);
		goto err_free;
	}

	tcs1421_set_cfg(priv, "NormalDRP", 1);
	platform_set_drvdata(pdev, priv);
	return 0;

err_free:
	kfree(priv);
	return ret;
}

static int tcs1421_remove(struct platform_device *pdev)
{
	struct tcs1421_priv *priv = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_tcs1421_cfg);
	tcs1421_free(priv);
	kfree(priv);
	return 0;
}

static void tcs1421_dev_release(struct device *dev)
{
}

static struct platform_device tcs1421_dev = {
	.name = "tcs1421",
	.id = -1,
	.dev = {
		.platform_data = &tcs1421_plat_data,
		.release = tcs1421_dev_release,
	},
};

static struct platform_driver tcs1421_driver = {
	.probe = tcs1421_probe,
	.remove = tcs1421_remove,
	.driver = {
		.name = "tcs1421",
		.pm = &tcs1421_pm_ops,
	},
};

static int __init tcs1421_dev_init(void)
{
	int ret;

	ret = platform_device_register(&tcs1421_dev);
	if (ret) {
		pr_err("tcs1421 device init error.\n");
		return ret;
	}
	return platform_driver_register(&tcs1421_driver);
}

static void __exit tcs1421_dev_exit(void)
{
	platform_device_unregister(&tcs1421_dev);
	platform_driver_unregister(&tcs1421_driver);
}

module_init(tcs1421_dev_init);
module_exit(tcs1421_dev_exit);

MODULE_ALIAS("usb:tcs1421");
MODULE_DESCRIPTION("TCS1421 Type-C port controller of the HiBy R1");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
