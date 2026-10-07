// SPDX-License-Identifier: GPL-2.0
//
// keyboard_adc_multifunc -- the keys of the HiBy R1 on a resistor ladder,
// read as a voltage on an ADC channel of soc_adc.ko and reported as the
// input device "jz adc keyboard", with an optional second code per key.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock keyboard_adc_multifunc.sh passes them):
//
//   adc_channel          ADC channel of the ladder
//   adc_init_value       voltage with no key pressed, mV (not read)
//   adc_deviation        a key matches within this many mV of its value
//   adc_key_detectime    poll interval, ms
//   keyN_code, keyN_reuse_code, keyN_multifunc_enable, keyN_value
//                        N = 1..8: key code, second code, 1 for a
//                        multi-function key, its voltage in mV; the first
//                        code of -1 ends the table
//   debug                1: print every reading
//
// While the input device is open a worker polls the channel. A plain key
// reports a press when it is pressed and a release when it is let go. A
// multi-function key reports nothing at once: a second press within ten poll
// intervals reports reuse_code, otherwise the timer reports code, each as a
// press and a release.
//
// /sys/devices/platform/adc_key.0/key_config (0644):
//   read   "key N: enable=E code=C reuse_code=R" for every key
//   write  "<key> <enable> <swap>": enable or disable multi-function on key N
//          (from 0); a swap that differs from the last one written exchanges
//          that key's code and reuse_code
//
// Loaded after soc_adc.ko, which provides the adc_* calls.

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

// Exported by soc_adc.ko.
extern void adc_enable(void);
extern void adc_disable(void);
extern int adc_read_channel_voltage(int channel);

#define ADC_KEY_MAX	8

struct adc_key_info {
	int code;
	int reuse_code;
	int multifunc_enable;
	int value;
};

struct adc_keys_data {
	int adc_init_value;
	int adc_deviation;
	int adc_key_detectime;
	struct adc_key_info *key_info;
	unsigned int key_count;		// keys before the first code of -1
	u8 swapped;			// the swap flag last written to key_config
};

// The entry past key 8 ends the table when all eight codes are set.
static struct adc_key_info adc_key_info[ADC_KEY_MAX + 1] = {
	{ -1, -1, 0, 0 },
	{ -1, -1, 0, 300 },
	{ -1, -1, 0, 600 },
	{ -1, -1, 0, 900 },
	{ -1, -1, 0, 1200 },
	{ -1, -1, 0, 1500 },
	{ -1, -1, 0, 1800 },
	{ -1, -1, 0, 1800 },
	{ -1, -1, 0, 1800 },
};

static struct adc_keys_data data = {
	.adc_init_value = 1800,
	.adc_deviation = 50,
	.adc_key_detectime = 100,
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

#define ADC_KEY_PARAMS(n)									\
	module_param_named(key##n##_code, adc_key_info[n - 1].code, int, 0644);			\
	module_param_named(key##n##_reuse_code, adc_key_info[n - 1].reuse_code, int, 0644);	\
	module_param_named(key##n##_multifunc_enable, adc_key_info[n - 1].multifunc_enable, int, 0644); \
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
	spinlock_t lock;		// click_pending and click_key
	struct timer_list timer;	// the end of a multi-function key's click window
	int click_pending;		// a multi-function key was pressed once
	int click_key;			// its index
};

static void report_key_value(struct input_dev *input, int code, int value)
{
	input_report_key(input, code, value);
	input_sync(input);
}

// The index of the key whose window holds mv, -1 for none.
static int adc_key_index(struct adc_keys_data *pdata, int mv)
{
	struct adc_key_info *info = pdata->key_info;
	int i = 0;

	do {
		if (info[i].value - pdata->adc_deviation < mv &&
		    mv < info[i].value + pdata->adc_deviation)
			return i;
		i++;
	} while (info[i].code != -1);
	return -1;
}

// A single click of a multi-function key.
static void multi_click_timer_handler(unsigned long arg)
{
	struct adc_keyboard *kbd = (struct adc_keyboard *)arg;
	int code;

	spin_lock(&kbd->lock);
	if (kbd->click_pending == 1) {
		code = kbd->pdata->key_info[kbd->click_key].code;
		report_key_value(kbd->input, code, 1);
		report_key_value(kbd->input, code, 0);
		kbd->click_pending = 0;
	}
	spin_unlock(&kbd->lock);
}

static void adc_keyboard_work(struct work_struct *work)
{
	struct adc_keyboard *kbd = container_of(work, struct adc_keyboard, work);
	struct adc_keys_data *pdata = kbd->pdata;
	struct adc_key_info *info = pdata->key_info;
	unsigned int click_ms = pdata->adc_key_detectime * 10;
	unsigned long interval = pdata->adc_key_detectime * 1000;
	int last = 0, last_idx = 0;
	int code, idx, mv;

	adc_enable();
	while (kbd->opened) {
		mv = adc_read_channel_voltage(kbd->channel);
		if (debug)
			printk(KERN_EMERG "adc_val = %d\n", mv);
		idx = adc_key_index(pdata, mv);
		code = idx < 0 ? 0 : info[idx].code;

		if (code != last) {
			spin_lock_bh(&kbd->lock);
			if (kbd->click_pending == 1 && code == info[kbd->click_key].code) {
				// The second press of a multi-function key.
				del_timer(&kbd->timer);
				report_key_value(kbd->input, info[kbd->click_key].reuse_code, 1);
				report_key_value(kbd->input, info[kbd->click_key].reuse_code, 0);
				kbd->click_pending = 0;
				spin_unlock_bh(&kbd->lock);
				usleep_range(interval, interval);
				code = 0;
				spin_lock_bh(&kbd->lock);
			}
			if (last && !info[last_idx].multifunc_enable)
				report_key_value(kbd->input, last, 0);
			if (code) {
				if (!info[idx].multifunc_enable) {
					report_key_value(kbd->input, code, 1);
				} else if (!kbd->click_pending) {
					kbd->click_pending = 1;
					kbd->click_key = idx;
					mod_timer(&kbd->timer, jiffies + msecs_to_jiffies(click_ms));
				}
			}
			spin_unlock_bh(&kbd->lock);
			last_idx = idx;
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
	del_timer_sync(&kbd->timer);
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

static ssize_t key_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adc_keyboard *kbd = dev_get_drvdata(dev);
	struct adc_keys_data *pdata = kbd->pdata;
	struct adc_key_info *info;
	unsigned int i;
	ssize_t len = 0;

	spin_lock_bh(&kbd->lock);
	for (i = 0; i < pdata->key_count; i++) {
		info = &pdata->key_info[i];
		len += scnprintf(buf + len, PAGE_SIZE - len, "key %d: enable=%d code=%u reuse_code=%u\n",
				 i, info->multifunc_enable, info->code, info->reuse_code);
	}
	spin_unlock_bh(&kbd->lock);
	return len;
}

static ssize_t key_config_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct adc_keyboard *kbd = dev_get_drvdata(dev);
	struct adc_keys_data *pdata = kbd->pdata;
	struct adc_key_info *info;
	unsigned int key, enable, swap;
	int code;

	if (sscanf(buf, "%u %u %u", &key, &enable, &swap) != 3) {
		dev_err(dev, "Invalid input format. Use: echo \"<key_num> <enable> <swap>\" > key_config\n");
		return -EINVAL;
	}
	if (key >= pdata->key_count) {
		dev_err(dev, "Invalid key_num %u. Max is %d\n", key, pdata->key_count - 1);
		return -EINVAL;
	}
	if (enable > 1 || swap > 1) {
		dev_err(dev, "Invalid enable/swap_flag value. Use 0 or 1.\n");
		return -EINVAL;
	}

	spin_lock_bh(&kbd->lock);
	info = &pdata->key_info[key];
	info->multifunc_enable = enable;
	dev_info(dev, "Key %u: multi-function set to %d\n", key, enable);
	if (pdata->swapped != swap) {
		code = info->code;
		info->code = info->reuse_code;
		info->reuse_code = code;
		dev_info(dev, "Key %u: codes swapped. New code=%u, reuse_code=%u\n",
			 key, info->code, info->reuse_code);
		pdata->swapped = swap;
	}
	if (!enable && kbd->click_pending == 1 && kbd->click_key == key) {
		del_timer(&kbd->timer);
		kbd->click_pending = 0;
	}
	spin_unlock_bh(&kbd->lock);
	return count;
}

static DEVICE_ATTR(key_config, 0644, key_config_show, key_config_store);

static int keyboard_probe(struct platform_device *pdev)
{
	struct adc_keys_data *pdata = dev_get_platdata(&pdev->dev);
	struct adc_keyboard *kbd;
	struct input_dev *input;
	unsigned int i;
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

	i = 0;
	do {
		set_bit(pdata->key_info[i].code, input->keybit);
		if (pdata->key_info[i].reuse_code != -1)
			set_bit(pdata->key_info[i].reuse_code, input->keybit);
		i++;
	} while (pdata->key_info[i].code != -1);

	spin_lock_init(&kbd->lock);
	setup_timer(&kbd->timer, multi_click_timer_handler, (unsigned long)kbd);
	kbd->click_key = 0;
	pdata->key_count = i;

	input_set_drvdata(input, kbd);
	platform_set_drvdata(pdev, kbd);

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		kfree(kbd);
		return ret;
	}
	if (device_create_file(&pdev->dev, &dev_attr_key_config))
		dev_err(&pdev->dev, "Failed to create sysfs file key_config\n");
	printk("jz adc keyboard driver has been initialized successfully!\n");
	return 0;
}

static int keyboard_remove(struct platform_device *pdev)
{
	struct adc_keyboard *kbd = platform_get_drvdata(pdev);

	input_unregister_device(kbd->input);
	device_remove_file(&pdev->dev, &dev_attr_key_config);
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

MODULE_DESCRIPTION("ADC keys of the HiBy R1");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
