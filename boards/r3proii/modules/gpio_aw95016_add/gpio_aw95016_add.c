// SPDX-License-Identifier: GPL-2.0
//
// gpio_aw95016_add -- registers the AW95016 GPIO expander of the HiBy
// R3 Pro II on I2C, address 0x20, for the pca953x driver of patch 0010.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock gpio_aw95016_add.sh passes them all):
//
//   i2c_bus_num   bus of the expander
//   gpio_base     first of its 16 GPIOs (160: port F)
//   irq_base      first of their interrupts
//   rst_gpio      RSTN, driven high at load; -1: none
//   int_gpio      accepted and unused, as by the vendor module: the expander
//                 is registered without an interrupt
//
// Loaded after utils.ko, which provides i2c_register_device() and the parsing
// of pin names such as "PA09".

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_data/pca953x.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);

static int i2c_bus_num = -1;
static int rst_gpio = -1;
static int int_gpio = -1;
static int gpio_base = 160;
static int irq_base = 160;

module_param(i2c_bus_num, int, 0644);
module_param_cb(rst_gpio, &param_gpio_ops, &rst_gpio, 0644);
module_param_cb(int_gpio, &param_gpio_ops, &int_gpio, 0644);
module_param(irq_base, int, 0644);
module_param(gpio_base, int, 0644);

static const char *const names[] = {
	"aw95016-0", "aw95016-1", "aw95016-2", "aw95016-3",
	"aw95016-4", "aw95016-5", "aw95016-6", "aw95016-7",
	"aw95016-8", "aw95016-9", "aw95016-10", "aw95016-11",
	"aw95016-12", "aw95016-13", "aw95016-14", "aw95016-15",
};

static struct pca953x_platform_data platform_data = {
	.names = names,
};

static struct i2c_board_info aw95016_board_info = {
	I2C_BOARD_INFO("aw95016", 0x20),
};

static struct i2c_client *aw95016_dev;
static bool rst_requested;

static int __init aw95016_dev_init(void)
{
	printk("aw95016_dev_init\n");
	platform_data.gpio_base = gpio_base;
	platform_data.invert = 0;
	platform_data.irq_base = irq_base;

	if (rst_gpio != -1) {
		if (gpio_request(rst_gpio, "aw95016_rst") < 0) {
			printk("aw95016_dev_init request gpio[%d] fail\n", rst_gpio);
			return -1;
		}
		rst_requested = true;
		gpio_direction_output(rst_gpio, 1);
	}

	aw95016_board_info.platform_data = &platform_data;
	aw95016_board_info.irq = 0;
	aw95016_dev = i2c_register_device(&aw95016_board_info, i2c_bus_num);
	if (!aw95016_dev)
		return -EINVAL;
	printk("aw95016_dev_init finish\n");
	return 0;
}

static void __exit aw95016_dev_exit(void)
{
	i2c_unregister_device(aw95016_dev);
	if (rst_requested)
		gpio_free(rst_gpio);
}

module_init(aw95016_dev_init);
module_exit(aw95016_dev_exit);

MODULE_ALIAS("gpio:aw95016");
MODULE_DESCRIPTION("AW95016 GPIO expander of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
