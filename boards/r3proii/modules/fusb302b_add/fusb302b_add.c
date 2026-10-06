// SPDX-License-Identifier: GPL-2.0
//
// fusb302b_add -- registers the FUSB302B Type-C controller of the HiBy
// R3 Pro II on I2C, address 0x22, as "typec_fusb302" with its board data.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock fusb302b.sh passes them all):
//
//   i2c_bus_num    bus of the controller
//   int_gpio       INT_N, the controller's interrupt
//   otg_id_gpio    the line that tells the USB controller it is host
//   charger_psy    power supply that follows the port's role
//   microvolt, microamp, microwatt   the most the port asks for as a sink
//
// The driver is the one of patch 0004, which takes this board data as
// struct fusb302_platform_data.
//
// Loaded after utils.ko, which provides i2c_register_device() and the parsing
// of pin names such as "PB21".

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/usb/fusb302.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);

static int i2c_bus_num = -1;
static int int_gpio = -1;
static int otg_id_gpio = -1;
static char charger_psy[20];

static struct fusb302_platform_data platdata = {
	.max_sink_microvolt = 5000000,
	.max_sink_microamp = 3000000,
	.max_sink_microwatt = 10000000,
};

module_param_cb(microwatt, &param_ops_int, &platdata.max_sink_microwatt, 0644);
__MODULE_PARM_TYPE(microwatt, "int");
module_param_cb(microamp, &param_ops_int, &platdata.max_sink_microamp, 0644);
__MODULE_PARM_TYPE(microamp, "int");
module_param_cb(microvolt, &param_ops_int, &platdata.max_sink_microvolt, 0644);
__MODULE_PARM_TYPE(microvolt, "int");
module_param_string(charger_psy, charger_psy, sizeof(charger_psy), 0644);
module_param(i2c_bus_num, int, 0644);
module_param_cb(otg_id_gpio, &param_gpio_ops, &otg_id_gpio, 0644);
module_param_cb(int_gpio, &param_gpio_ops, &int_gpio, 0644);

static struct i2c_board_info fusb302_board_info = {
	I2C_BOARD_INFO("typec_fusb302", 0x22),
	.platform_data = &platdata,
};

static struct i2c_client *fusb302b_dev;

static int __init fusb302b_dev_init(void)
{
	printk("fusb302b_dev_init\n");
	strcpy(platdata.charger_psy, charger_psy);
	platdata.otg_id_gpio = otg_id_gpio;
	fusb302_board_info.irq = gpio_to_irq(int_gpio);

	fusb302b_dev = i2c_register_device(&fusb302_board_info, i2c_bus_num);
	return fusb302b_dev ? 0 : -EINVAL;
}

static void __exit fusb302b_dev_exit(void)
{
	i2c_unregister_device(fusb302b_dev);
}

module_init(fusb302b_dev_init);
module_exit(fusb302b_dev_exit);

MODULE_ALIAS("usb:fusb302b");
MODULE_DESCRIPTION("FUSB302B Type-C controller of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
