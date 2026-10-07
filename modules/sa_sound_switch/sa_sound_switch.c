// SPDX-License-Identifier: GPL-2.0
//
// sa_sound_switch -- what is plugged into the outputs of the HiBy X1600
// players, as switch class devices (/sys/class/switch/NAME/state).
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock sa_sound_switch.sh passes them). Four outputs, each
// read from a pin or from an ADC channel of soc_adc.ko:
//
//   output      switch      parameters sass_OUTPUT_*
//   headset     headset     det_gpio, en_level        a pin; its level when
//   balance     balance                               plugged is en_level
//   lineout     lineout     adc_channel, adc_min,     an ADC channel; plugged
//   balancelo   balancelo   adc_max, adc_range_state  is range_state when
//                                                     min <= mV < max, the
//                                                     other way round outside
//
// A pin below 512 is used; otherwise a channel of 0 or more; otherwise the
// output is left out. sass_debounce_time is the poll interval in ms: a state
// is reported once it has been read three times in a row.
//
// /sys/devices/platform/sa_sound_switch/:
//   enable      "on" starts polling, anything else stops it; reads 1 or 0
//   switch_on   write a switch name: report it plugged
//   switch_off  write a switch name: report it unplugged
//
// get_switch_status(name) returns the last state read for a switch, for
// sa_earpods_adc.ko.
//
// Loaded after utils.ko and soc_adc.ko.

#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/switch.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

// Exported by soc_adc.ko.
extern void adc_enable(void);
extern void adc_disable(void);
extern int adc_read_channel_voltage(int channel);

// Exported by utils.ko: sets a pin's function, with the encoding of
// enum gpio_function in the X1600 SoC headers.
extern const struct kernel_param_ops param_gpio_ops;
extern int gpio_set_func(int gpio, int func);
#define GPIO_INPUT_NO_PULL	0x8106

#define SASS_PINS		512	// pin numbers below this are pins
#define SASS_DEBOUNCE_READS	3

struct sass_item {
	const char *label;		// in the log and as the pin's label
	const char *name;		// the switch
	struct switch_dev sdev;
	int state;			// as last read
	int reported;			// as last reported
	int raw;			// pin level or mV
	int reads;			// reads in a row that differ from reported
	bool adc;			// read from an ADC channel, not a pin
	bool registered;
	int line;			// the pin or the channel
	int adc_min;
	int adc_max;
	int on_level;			// en_level for a pin, range_state for a channel
};

struct sass {
	struct sass_item *items;
	int count;
	struct delayed_work work;
	unsigned long debounce;		// jiffies
	struct sass_item item[0];
};

static struct sass *gp_switch;
static int g_sass_enable;

#define SASS_OUTPUT(o, gpio_default)							\
	static int sass_##o##_det_gpio = gpio_default;					\
	static int sass_##o##_en_level = -1;						\
	static int sass_##o##_adc_channel = -1;						\
	static int sass_##o##_adc_min = -1;						\
	static int sass_##o##_adc_max = -1;						\
	static int sass_##o##_adc_range_state;						\
	module_param_cb(sass_##o##_det_gpio, &param_gpio_ops, &sass_##o##_det_gpio, 0644);	\
	module_param(sass_##o##_en_level, int, 0644);					\
	module_param(sass_##o##_adc_channel, int, 0644);				\
	module_param(sass_##o##_adc_min, int, 0644);					\
	module_param(sass_##o##_adc_max, int, 0644);					\
	module_param(sass_##o##_adc_range_state, int, 0644)

static int sass_debounce_time = 200;
module_param(sass_debounce_time, int, 0644);
SASS_OUTPUT(headset, -1);
SASS_OUTPUT(balance, -1);
SASS_OUTPUT(lineout, -1);
SASS_OUTPUT(balancelo, -1);

int get_switch_status(const char *name)
{
	int i;

	if (!gp_switch || !name)
		return -1;
	for (i = 0; i < gp_switch->count; i++)
		if (!strcmp(gp_switch->items[i].name, name))
			return gp_switch->items[i].state;
	return 0;
}
EXPORT_SYMBOL(get_switch_status);

static ssize_t switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->name);
}

static void switch_work_schedule(struct work_struct *work)
{
	struct sass *sw = container_of(to_delayed_work(work), struct sass, work);
	struct sass_item *it;
	int i, in_range;

	if (!g_sass_enable)
		return;

	for (i = 0; i < sw->count; i++) {
		it = &sw->items[i];
		if (it->adc)
			it->raw = adc_read_channel_voltage(it->line);
		else
			it->raw = gpio_get_value(it->line);
	}

	for (i = 0; i < sw->count; i++) {
		it = &sw->items[i];
		if (it->adc) {
			in_range = it->raw >= it->adc_min && it->raw < it->adc_max;
			it->state = in_range ? it->on_level : it->on_level == 0;
		} else {
			it->state = it->raw ? it->on_level : it->on_level == 0;
		}
		if (it->state == it->reported) {
			it->reads = 0;
		} else if (++it->reads >= SASS_DEBOUNCE_READS) {
			switch_set_state(&it->sdev, it->state);
			it->reported = it->state;
		}
	}

	queue_delayed_work(system_wq, &sw->work, sw->debounce);
}

static ssize_t switch_get_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_sass_enable);
}

static ssize_t switch_set_enable(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct sass *sw;

	if (strncmp(buf, "on", 2)) {
		g_sass_enable = 0;
	} else if (!g_sass_enable) {
		g_sass_enable = 1;
		sw = dev_get_drvdata(dev);
		queue_delayed_work(system_wq, &sw->work, sw->debounce);
	}
	return count;
}

static void switch_force(struct device *dev, const char *buf, int state)
{
	struct sass *sw = dev_get_drvdata(dev);
	struct sass_item *it;
	int i;

	for (i = 0; i < sw->count; i++) {
		it = &sw->items[i];
		if (!strncmp(it->name, buf, strlen(it->name))) {
			it->state = state;
			it->reported = state;
			switch_set_state(&it->sdev, state);
		}
	}
}

static ssize_t switch_set_switch_on(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	switch_force(dev, buf, 1);
	return count;
}

static ssize_t switch_set_switch_off(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	switch_force(dev, buf, 0);
	return count;
}

static DEVICE_ATTR(enable, 0644, switch_get_enable, switch_set_enable);
static DEVICE_ATTR(switch_on, 0200, NULL, switch_set_switch_on);
static DEVICE_ATTR(switch_off, 0200, NULL, switch_set_switch_off);

static struct attribute *sa_switch_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_switch_on.attr,
	&dev_attr_switch_off.attr,
	NULL,
};

static const struct attribute_group sa_switch_group = {
	.attrs = sa_switch_attributes,
};

static int switch_probe(struct platform_device *pdev)
{
	struct sass *sw = dev_get_platdata(&pdev->dev);
	struct sass_item *it;
	int i, ret;

	if (!sw->count) {
		printk("[SASS]NO item probe!\n");
		return -EINVAL;
	}
	platform_set_drvdata(pdev, sw);

	ret = sysfs_create_group(&pdev->dev.kobj, &sa_switch_group);
	if (ret < 0) {
		printk("[SASS]failed to create files\n");
		return ret;
	}

	for (i = 0; i < sw->count; i++) {
		it = &sw->items[i];
		it->sdev.name = it->name;
		it->sdev.print_name = switch_print_name;
		if (!it->adc) {
			if (gpio_request(it->line, it->label) < 0) {
				printk("[SASS]failed to init switch item %s\n", it->label);
				continue;
			}
			gpio_direction_input(it->line);
			gpio_set_func(it->line, GPIO_INPUT_NO_PULL);
		} else {
			adc_enable();
		}
		if (switch_dev_register(&it->sdev) < 0) {
			printk("[SASS]failed to regist switch device %s\n", it->name);
			printk("[SASS]failed to init switch item %s\n", it->label);
			continue;
		}
		it->registered = true;
		printk("[SASS]success to init switch item %s\n", it->label);
	}

	sw->debounce = msecs_to_jiffies(sass_debounce_time);
	INIT_DELAYED_WORK(&sw->work, switch_work_schedule);
	g_sass_enable = 1;
	queue_delayed_work(system_wq, &sw->work, sw->debounce);
	printk("[SASS]switch_probe OK\n");
	return 0;
}

static int switch_remove(struct platform_device *pdev)
{
	struct sass *sw = platform_get_drvdata(pdev);
	struct sass_item *it;
	int i;

	cancel_delayed_work_sync(&sw->work);
	for (i = 0; i < sw->count; i++) {
		it = &sw->items[i];
		if (it->registered)
			switch_dev_unregister(&it->sdev);
		it->registered = false;
		if (!it->adc)
			gpio_free(it->line);
		else
			adc_disable();
	}
	sysfs_remove_group(&pdev->dev.kobj, &sa_switch_group);
	g_sass_enable = 0;
	return 0;
}

static void switch_shutdown(struct platform_device *pdev)
{
	g_sass_enable = 0;
}

static int switch_suspend(struct device *dev)
{
	struct sass *sw = dev_get_drvdata(dev);

	printk("[SASS]switch_suspend\n");
	cancel_delayed_work_sync(&sw->work);
	adc_disable();
	return 0;
}

static int switch_resume(struct device *dev)
{
	struct sass *sw = dev_get_drvdata(dev);

	printk("[SASS]switch_resume\n");
	adc_enable();
	queue_delayed_work(system_wq, &sw->work, sw->debounce);
	return 0;
}

static SIMPLE_DEV_PM_OPS(switch_pm_ops, switch_suspend, switch_resume);

static void switch_dev_release(struct device *dev)
{
}

static struct platform_device switch_dev = {
	.name = "sa_sound_switch",
	.id = -1,
	.dev = {
		.release = switch_dev_release,
	},
};

static struct platform_driver switch_drv = {
	.probe = switch_probe,
	.remove = switch_remove,
	.shutdown = switch_shutdown,
	.driver = {
		.name = "sa_sound_switch",
		.pm = &switch_pm_ops,
	},
};

// Fills the next item from an output's parameters; false when the output has
// neither a pin nor a channel.
static bool sass_add(struct sass_item *it, const char *label, const char *name, int gpio,
		     int en_level, int channel, int min, int max, int range_state)
{
	it->label = label;
	it->name = name;
	if ((unsigned int)gpio < SASS_PINS) {
		it->line = gpio;
		it->on_level = en_level;
		return true;
	}
	if (channel < 0)
		return false;
	it->adc = true;
	it->line = channel;
	it->adc_min = min;
	it->adc_max = max;
	it->on_level = range_state;
	return true;
}

#define SASS_ADD(o, label)									\
	sass_add(&sw->item[count], label, #o, sass_##o##_det_gpio, sass_##o##_en_level,	\
		 sass_##o##_adc_channel, sass_##o##_adc_min, sass_##o##_adc_max,		\
		 sass_##o##_adc_range_state)

#define SASS_USED(o)	((unsigned int)sass_##o##_det_gpio < SASS_PINS || sass_##o##_adc_channel >= 0)

static int __init sass_init(void)
{
	struct sass *sw;
	int n, count = 0, ret;

	n = SASS_USED(headset) + SASS_USED(balance) + SASS_USED(lineout) + SASS_USED(balancelo);
	if (!n) {
		printk("[SASS]NO detect items\n");
		return -EINVAL;
	}
	printk("[SASS]%d detect items\n", n);

	sw = kzalloc(sizeof(*sw) + n * sizeof(struct sass_item), GFP_KERNEL);
	gp_switch = sw;
	if (!sw) {
		printk("[SASS]failed to alloc %d switch\n", n);
		return -ENOMEM;
	}

	if (SASS_ADD(headset, "headphone"))
		count++;
	if (SASS_ADD(balance, "balance"))
		count++;
	if (SASS_ADD(lineout, "lineout"))
		count++;
	if (SASS_ADD(balancelo, "balancelo"))
		count++;
	sw->items = sw->item;
	sw->count = count;
	switch_dev.dev.platform_data = sw;

	ret = platform_device_register(&switch_dev);
	if (ret) {
		printk("[SASS]failed to register switch device\n");
		gp_switch = NULL;
		kfree(sw);
		return ret;
	}
	return platform_driver_register(&switch_drv);
}

static void __exit sass_exit(void)
{
	platform_device_unregister(&switch_dev);
	platform_driver_unregister(&switch_drv);
	if (gp_switch)
		kfree(gp_switch);
	gp_switch = NULL;
}

module_init(sass_init);
module_exit(sass_exit);

MODULE_DESCRIPTION("Output detection of the HiBy X1600 players");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
