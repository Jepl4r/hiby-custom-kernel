// SPDX-License-Identifier: GPL-2.0
//
// sa_config_module -- /dev/sa-config, through which HiBy's userspace asks
// the kernel which player it runs on.
//
// A drop-in replacement for the vendor module of the same name. Two ioctls,
// each copying a string to the caller's buffer without its terminating NUL:
//
//   SA_CONFIG_GET_MODEL      the model, SA_CONFIG_MODEL ("r1", "r3proii")
//   SA_CONFIG_GET_UPT_NAME   the firmware file name, the model + ".upt"
//
// SA_CONFIG_MODEL comes from the Makefile (HIBY_MODEL, the board's folder in
// boards/), so the one source builds the module of every board.

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#ifndef SA_CONFIG_MODEL
#error "SA_CONFIG_MODEL is not set"
#endif

#define SA_CONFIG_GET_MODEL	0xffff00f0
#define SA_CONFIG_GET_UPT_NAME	0xffff00f1

struct sa_config {
	struct device *dev;
	struct miscdevice misc;
	struct mutex lock;
	int open_count;
};

static struct sa_config *config;

static int sa_config_open(struct inode *inode, struct file *file)
{
	struct sa_config *c = container_of(file->private_data, struct sa_config, misc);

	mutex_lock(&c->lock);
	c->open_count++;
	mutex_unlock(&c->lock);
	return 0;
}

static int sa_config_release(struct inode *inode, struct file *file)
{
	struct sa_config *c = container_of(file->private_data, struct sa_config, misc);

	mutex_lock(&c->lock);
	c->open_count--;
	mutex_unlock(&c->lock);
	return 0;
}

static long sa_config_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char name[256];

	switch (cmd) {
	case SA_CONFIG_GET_MODEL:
		if (copy_to_user((void __user *)arg, SA_CONFIG_MODEL, strlen(SA_CONFIG_MODEL)))
			return -EFAULT;
		return 0;
	case SA_CONFIG_GET_UPT_NAME:
		strcpy(name, SA_CONFIG_MODEL);
		strcat(name, ".upt");
		if (copy_to_user((void __user *)arg, name, strlen(name)))
			return -EFAULT;
		return 0;
	default:
		printk("no support other cmd\n");
		return -1;
	}
}

static const struct file_operations sa_config_misc_fops = {
	.unlocked_ioctl = sa_config_ioctl,
	.open = sa_config_open,
	.release = sa_config_release,
};

static int sa_config_probe(struct platform_device *pdev)
{
	int ret;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		printk("config: malloc faile\n");
		return -ENOMEM;
	}
	config->dev = &pdev->dev;
	config->misc.minor = MISC_DYNAMIC_MINOR;
	config->misc.name = "sa-config";
	config->misc.fops = &sa_config_misc_fops;

	ret = misc_register(&config->misc);
	if (ret < 0) {
		dev_err(config->dev, "misc_register failed\n");
		return ret;
	}
	platform_set_drvdata(pdev, config);
	mutex_init(&config->lock);
	return 0;
}

static int sa_config_remove(struct platform_device *pdev)
{
	struct sa_config *c = platform_get_drvdata(pdev);

	misc_deregister(&c->misc);
	kfree(c);
	return 0;
}

static int sa_config_suspend(struct device *dev)
{
	return 0;
}

static int sa_config_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops sa_config_pm_ops = {
	.suspend = sa_config_suspend,
	.resume = sa_config_resume,
};

static void sa_config_dev_release(struct device *dev)
{
}

static struct platform_device sa_config_device = {
	.name = "sa-config",
	.id = -1,
	.dev = {
		.release = sa_config_dev_release,
	},
};

static struct platform_driver sa_config_driver = {
	.probe = sa_config_probe,
	.remove = sa_config_remove,
	.driver = {
		.name = "sa-config",
		.owner = THIS_MODULE,
		.pm = &sa_config_pm_ops,
	},
};

static int __init sa_config_init(void)
{
	int ret;

	ret = platform_device_register(&sa_config_device);
	if (ret) {
		pr_err("sa_config device init error.\n");
		return ret;
	}
	return platform_driver_register(&sa_config_driver);
}

static void __exit sa_config_exit(void)
{
	platform_device_unregister(&sa_config_device);
	platform_driver_unregister(&sa_config_driver);
}

module_init(sa_config_init);
module_exit(sa_config_exit);

MODULE_VERSION("20160830");
MODULE_DESCRIPTION("HiBy player model for userspace (/dev/sa-config)");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
