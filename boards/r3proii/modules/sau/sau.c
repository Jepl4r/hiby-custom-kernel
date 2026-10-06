// SPDX-License-Identifier: GPL-2.0
//
// sau -- /sys/devices/platform/sa_information/sa_verification, a word that
// HiBy's player and its bluealsa write and read back to check that they run
// on a HiBy kernel.
//
// A drop-in replacement for the vendor module of the same name (HiBy R3 Pro
// II). Writing "bt", "wifi", "smartaction" or "sau" (a prefix of the write is
// enough) stores that word; anything else leaves the stored word as it is.
// Reading answers by the stored word:
//
//   "bt"    "/main.conf" while the SAU flag is set, else nothing
//   "sau"   "ok" while the SAU flag is set, else "fail"
//   other   nothing
//
// The word is "smartaction" from load, and the SAU flag is set; other modules
// may change the flag with sa_set_sau_verify().

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>

int sa_get_sau_verify(void);
void sa_set_sau_verify(int verify);

static int sa_sau_verify = 1;
static char sa_verification[16];

void sa_set_sau_verify(int verify)
{
	sa_sau_verify = verify;
}
EXPORT_SYMBOL(sa_set_sau_verify);

int sa_get_sau_verify(void)
{
	return sa_sau_verify;
}

static void store_word(const char *buf, size_t n)
{
	strncpy(sa_verification, buf, n);
	sa_verification[n] = '\0';
}

static ssize_t set_sa_verification(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (!strncmp(buf, "bt", 2))
		store_word(buf, 2);
	else if (!strncmp(buf, "wifi", 4))
		store_word(buf, 4);
	else if (!strncmp(buf, "smartaction", 11))
		store_word(buf, 11);
	else if (!strncmp(buf, "sau", 3))
		store_word(buf, 3);
	printk("set_sa_verification %s\n", buf);
	return count;
}

static ssize_t get_sa_verification(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	if (!strcmp(sa_verification, "bt")) {
		if (sa_sau_verify)
			strcpy(buf, "/main.conf");
	} else if (!strcmp(sa_verification, "sau")) {
		strcpy(buf, sa_sau_verify ? "ok" : "fail");
	}
	printk("get_sa_verification %s\n", buf);
	return strlen(buf);
}

static DEVICE_ATTR(sa_verification, 0644, get_sa_verification, set_sa_verification);

static struct attribute *sa_information_attributes[] = {
	&dev_attr_sa_verification.attr,
	NULL,
};

static const struct attribute_group sa_information_group = {
	.attrs = sa_information_attributes,
};

static int sa_information_probe(struct platform_device *pdev)
{
	int ret;

	ret = sysfs_create_group(&pdev->dev.kobj, &sa_information_group);
	if (ret < 0)
		return ret;
	return 0;
}

static int sa_information_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &sa_information_group);
	return 0;
}

static void sa_information_dev_release(struct device *dev)
{
}

static struct platform_device sa_information_device = {
	.name = "sa_information",
	.id = -1,
	.dev = {
		.release = sa_information_dev_release,
	},
};

static struct platform_driver sa_information_driver = {
	.probe = sa_information_probe,
	.remove = sa_information_remove,
	.driver = {
		.name = "sa_information",
		.owner = THIS_MODULE,
	},
};

static int __init sa_information_init(void)
{
	int ret;

	ret = platform_device_register(&sa_information_device);
	if (ret)
		return ret;
	ret = platform_driver_register(&sa_information_driver);
	if (ret)
		return ret;
	strcpy(sa_verification, "smartaction");
	printk("%s success\n", __func__);
	return 0;
}

static void __exit sa_information_exit(void)
{
	printk("%s\n", __func__);
	platform_driver_unregister(&sa_information_driver);
	platform_device_unregister(&sa_information_device);
}

module_init(sa_information_init);
module_exit(sa_information_exit);

MODULE_DESCRIPTION("SmartAction Information");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
