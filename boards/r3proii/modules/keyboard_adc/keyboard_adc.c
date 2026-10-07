// SPDX-License-Identifier: GPL-2.0
//
// keyboard_adc -- the keys of the HiBy R3 Pro II on a resistor ladder, read
// as a voltage on an ADC channel of soc_adc.ko and reported as the input
// device "jz adc keyboard".
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock keyboard_adc.sh passes them):
//
//   adc_channel          ADC channel of the ladder
//   adc_init_value       voltage with no key pressed, mV (not read)
//   adc_deviation        a key matches within this many mV of its value
//   adc_key_detectime    poll interval, ms
//   keyN_code, keyN_value  N = 1..8: key code and its voltage in mV; the
//                        first code of -1 ends the table
//   debug                1: print every reading
//
// While the input device is open a worker polls the channel; a reading in
// no key's window is no key. A change of key reports the old key's release
// and the new key's press.
//
// Loaded after soc_adc.ko, which provides the adc_* calls.

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

// Exported by soc_adc.ko.
extern void adc_enable(void);
extern void adc_disable(void);
extern int adc_read_channel_voltage(int channel);

#define ADC_KEY_MAX	8

struct adc_key_info {
	int code;
	int value;
};

struct adc_keys_data {
	int adc_init_value;
	int adc_deviation;
	int adc_key_detectime;
	struct adc_key_info *key_info;
};

// The entry past key 8 ends the table when all eight codes are set.
static struct adc_key_info adc_key_info[ADC_KEY_MAX + 1] = {
	[0] = { .code = -1 },
	[ADC_KEY_MAX] = { .code = -1 },
};

static struct adc_keys_data data = {
	.adc_init_value = 1800,
	.key_info = adc_key_info,
};

static int debug;

static void adc_keys_dev_release(struct device *dev)
{
}

static struct platform_device adc_keys_dev = {
	.name = "adc_key",
	.dev = {
		.platform_data = &data,
		.release = adc_keys_dev_release,
	},
};

#define ADC_KEY_PARAMS(n)								\
	module_param_named(key##n##_code, adc_key_info[n - 1].code, int, 0644);	\
	module_param_named(key##n##_value, adc_key_info[n - 1].value, int, 0644)

ADC_KEY_PARAMS(1);
ADC_KEY_PARAMS(2);
ADC_KEY_PARAMS(3);
ADC_KEY_PARAMS(4);
ADC_KEY_PARAMS(5);
ADC_KEY_PARAMS(6);
ADC_KEY_PARAMS(7);
ADC_KEY_PARAMS(8);
module_param_named(adc_init_value, data.adc_init_value, int, 0644);
// The channel is the platform device's id.
module_param_named(adc_channel, adc_keys_dev.id, int, 0644);
module_param_named(adc_deviation, data.adc_deviation, int, 0644);
module_param_named(adc_key_detectime, data.adc_key_detectime, int, 0644);
module_param(debug, int, 0644);

struct adc_keyboard {
	int channel;
	int opened;
	struct input_dev *input;
	struct workqueue_struct *wq;
	struct work_struct work;
	struct adc_keys_data *pdata;
};

// The code of the key whose window holds mv, 0 for none.
static int adc_key_code(struct adc_keys_data *pdata, int mv)
{
	struct adc_key_info *info = pdata->key_info;

	do {
		if (info->value - pdata->adc_deviation < mv &&
		    mv < info->value + pdata->adc_deviation)
			return info->code;
		info++;
	} while (info->code != -1);
	return 0;
}

static void adc_keyboard_work(struct work_struct *work)
{
	struct adc_keyboard *kbd = container_of(work, struct adc_keyboard, work);
	unsigned long interval = kbd->pdata->adc_key_detectime * 1000;
	int last = 0;
	int code, mv;

	adc_enable();
	while (kbd->opened) {
		mv = adc_read_channel_voltage(kbd->channel);
		if (debug)
			printk(KERN_EMERG "adc_val = %d\n", mv);
		code = adc_key_code(kbd->pdata, mv);
		if (code != last) {
			if (last) {
				input_report_key(kbd->input, last, 0);
				input_sync(kbd->input);
			}
			if (code) {
				input_report_key(kbd->input, code, 1);
				input_sync(kbd->input);
			}
		}
		usleep_range(interval, interval);
		last = code;
	}
}

static int adc_keyboard_open(struct input_dev *input)
{
	struct adc_keyboard *kbd = input_get_drvdata(input);

	if (!kbd) {
		pr_err("Failed input_get_drvdata\n");
		return -1;
	}
	if (kbd->opened)
		return 0;

	kbd->wq = create_singlethread_workqueue("aux_workqueue");
	if (!kbd->wq) {
		pr_err("Failed to create test_workqueue\n");
		return -1;
	}
	kbd->opened = 1;
	INIT_WORK(&kbd->work, adc_keyboard_work);
	queue_work(kbd->wq, &kbd->work);
	return 0;
}

static void adc_keyboard_close(struct input_dev *input)
{
	struct adc_keyboard *kbd = input_get_drvdata(input);

	if (!kbd->opened)
		return;
	// The worker sees opened at 0 and returns; destroy_workqueue waits for it.
	kbd->opened = 0;
	destroy_workqueue(kbd->wq);
	adc_disable();
}

static int keyboard_suspend(struct device *dev)
{
	struct adc_keyboard *kbd = dev_get_drvdata(dev);

	printk("keyboard_suspend\n");
	adc_keyboard_close(kbd->input);
	return 0;
}

static int keyboard_resume(struct device *dev)
{
	struct adc_keyboard *kbd = dev_get_drvdata(dev);

	printk("keyboard_resume\n");
	adc_keyboard_open(kbd->input);
	return 0;
}

static SIMPLE_DEV_PM_OPS(keyboard_pm_ops, keyboard_suspend, keyboard_resume);

static int keyboard_probe(struct platform_device *pdev)
{
	struct adc_keys_data *pdata = dev_get_platdata(&pdev->dev);
	struct adc_key_info *info;
	struct adc_keyboard *kbd;
	struct input_dev *input;
	int ret;

	kbd = kzalloc(sizeof(*kbd), GFP_KERNEL);
	if (!kbd) {
		pr_err("Failed to allocate driver structre\n");
		return -ENOMEM;
	}

	input = input_allocate_device();
	kbd->input = input;
	if (!input) {
		pr_err("jz adc keyboard drvier allocate memory failed!\n");
		kfree(kbd);
		return -ENOMEM;
	}

	input->name = "jz adc keyboard";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0005;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	input->open = adc_keyboard_open;
	input->close = adc_keyboard_close;
	input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);

	kbd->opened = 0;
	kbd->channel = pdev->id;
	kbd->pdata = pdata;

	info = pdata->key_info;
	do {
		set_bit(info->code, input->keybit);
		info++;
	} while (info->code != -1);

	input_set_drvdata(input, kbd);
	platform_set_drvdata(pdev, kbd);

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		kfree(kbd);
		return ret;
	}
	printk("jz adc keyboard driver has been initialized successfully!\n");
	return 0;
}

static int keyboard_remove(struct platform_device *pdev)
{
	struct adc_keyboard *kbd = platform_get_drvdata(pdev);

	input_unregister_device(kbd->input);
	kfree(kbd);
	return 0;
}

static struct platform_driver adc_keys_drv = {
	.probe = keyboard_probe,
	.remove = keyboard_remove,
	.driver = {
		.name = "adc_key",
		.pm = &keyboard_pm_ops,
	},
};

static int __init adc_keyboard_drv_init(void)
{
	platform_device_register(&adc_keys_dev);
	platform_driver_register(&adc_keys_drv);
	return 0;
}

static void __exit adc_keyboard_drv_exit(void)
{
	platform_device_unregister(&adc_keys_dev);
	platform_driver_unregister(&adc_keys_drv);
}

module_init(adc_keyboard_drv_init);
module_exit(adc_keyboard_drv_exit);

MODULE_DESCRIPTION("ADC keys of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
