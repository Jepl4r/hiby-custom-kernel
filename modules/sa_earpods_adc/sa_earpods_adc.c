// SPDX-License-Identifier: GPL-2.0
//
// sa_earpods_adc -- the buttons of a wired headset remote (Apple EarPods
// style), read as a voltage on an ADC channel of soc_adc.ko and reported as
// the input device "earpods_adc".
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock sa_earpods_adc.sh passes them):
//
//   sea_adc_channel        ADC channel of the microphone line
//   sea_sdev_name          switch of sa_sound_switch.ko that says a headset
//                          is plugged in; "-1": always plugged in
//   sea_pwr_name           regulator that powers the microphone; "-1": none
//   sea_play_pause_min/max, sea_volume_up_min/max, sea_volume_down_min/max
//                          mV windows of the three buttons, min <= mV < max;
//                          the volume buttons only when both windows are set
//   sea_poll_ms            poll interval
//   sea_play_pause_press_hold_ms         a held centre button becomes a long
//   sea_play_pause_press_hold_repeat_ms  press after this, and repeats every
//   sea_play_pause_release_reset_ms      this; clicks end after this gap
//
// The centre button: one, two or three clicks report KEY_PLAYPAUSE,
// KEY_NEXTSONG or KEY_PREVIOUSSONG; held after one or two clicks it reports
// KEY_FASTFORWARD or KEY_REWIND, repeated, until released. The volume
// buttons report KEY_VOLUMEUP and KEY_VOLUMEDOWN as pressed and released.
//
// Polling runs while /sys/devices/platform/earpods_adc/earpods_adc/
// earpods_adc_sw is "on" (it reads 1 or 0). After a headset is plugged in,
// readings in the centre button's window are ignored for up to ten polls.
//
// Loaded after soc_adc.ko and sa_sound_switch.ko.

#include <linux/device.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

// Exported by soc_adc.ko.
extern void adc_enable(void);
extern void adc_disable(void);
extern int adc_read_channel_voltage(int channel);

// Exported by sa_sound_switch.ko.
extern int get_switch_status(const char *name);

#define EARPODS_SETTLE_POLLS	10
#define EARPODS_START_DELAY	50	// jiffies, from "on" or resume to the first poll

enum earpods_state {
	EARPODS_SETTLING = 1,		// just plugged in
	EARPODS_READY = 2,
};

struct earpods_adc_platform_data {
	int adc_channel;
	char *sdev_name;
	char *pwr_name;
	int play_pause_min;
	int play_pause_max;
	int volume_up_min;
	int volume_up_max;
	int volume_down_min;
	int volume_down_max;
	int poll_ms;
	int press_hold_ms;
	int press_hold_repeat_ms;
	int release_reset_ms;
};

static struct earpods_adc_platform_data earpods_adc_pdata = {
	.adc_channel = -1,
	.play_pause_min = -1,
	.play_pause_max = -1,
	.volume_up_min = -1,
	.volume_up_max = -1,
	.volume_down_min = -1,
	.volume_down_max = -1,
	.poll_ms = 10,
	.press_hold_ms = 500,
	.press_hold_repeat_ms = 200,
	.release_reset_ms = 500,
};

module_param_named(sea_adc_channel, earpods_adc_pdata.adc_channel, int, 0644);
module_param_named(sea_sdev_name, earpods_adc_pdata.sdev_name, charp, 0644);
module_param_named(sea_pwr_name, earpods_adc_pdata.pwr_name, charp, 0644);
module_param_named(sea_play_pause_min, earpods_adc_pdata.play_pause_min, int, 0644);
module_param_named(sea_play_pause_max, earpods_adc_pdata.play_pause_max, int, 0644);
module_param_named(sea_volume_up_min, earpods_adc_pdata.volume_up_min, int, 0644);
module_param_named(sea_volume_up_max, earpods_adc_pdata.volume_up_max, int, 0644);
module_param_named(sea_volume_down_min, earpods_adc_pdata.volume_down_min, int, 0644);
module_param_named(sea_volume_down_max, earpods_adc_pdata.volume_down_max, int, 0644);
module_param_named(sea_poll_ms, earpods_adc_pdata.poll_ms, int, 0644);
module_param_named(sea_play_pause_press_hold_ms, earpods_adc_pdata.press_hold_ms, int, 0644);
module_param_named(sea_play_pause_press_hold_repeat_ms, earpods_adc_pdata.press_hold_repeat_ms, int, 0644);
module_param_named(sea_play_pause_release_reset_ms, earpods_adc_pdata.release_reset_ms, int, 0644);

struct earpods_key {
	int hold_polls;			// centre button: polls until a long press
	int repeat_polls;		// polls between long-press repeats
	int reset_polls;		// polls of silence that end the clicks
	int min;
	int max;
	int last;			// pressed at the previous poll
	int code;			// volume: the key; centre: the long press
	int presses;
	int releases;
	int held;			// polls held
	int repeat;
	int idle;			// polls since the last release
	void (*process)(struct earpods_key *key, struct input_dev *input, int pressed);
};

struct earpods_adc {
	int channel;
	int poll_ms;
	int state;
	int settle;			// polls left to ignore the centre button
	bool reg_on;
	struct regulator *reg;
	bool suspended;
	struct input_dev *input;
	struct delayed_work work;
	int nkeys;
	struct earpods_key keys[3];
};

static int gi_earpods_adc_sw;

static void earpods_report(struct input_dev *input, int code, int value)
{
	input_report_key(input, code, value);
	input_sync(input);
}

static void earpods_volume_key_process(struct earpods_key *key, struct input_dev *input, int pressed)
{
	if (pressed) {
		if (++key->presses == 1)
			earpods_report(input, key->code, 1);
	} else if (key->presses) {
		earpods_report(input, key->code, 0);
		key->presses = 0;
	}
}

static void earpods_play_pause_key_process(struct earpods_key *key, struct input_dev *input, int pressed)
{
	int code;

	if (key->last != pressed) {
		if (pressed)
			key->presses++;
		else
			key->releases++;
		key->idle = 0;
		if (pressed)
			goto clicks;
	} else if (pressed) {
		key->held++;
		if (key->held == key->hold_polls)
			key->repeat = key->repeat_polls;
		else if (key->held > key->hold_polls)
			key->repeat++;
		if (key->repeat >= key->repeat_polls) {
			key->code = key->presses == 1 ? KEY_FASTFORWARD :
				    key->presses == 2 ? KEY_REWIND : 0;
			if (key->code)
				earpods_report(input, key->code, 1);
			key->repeat = 0;
		}
		goto clicks;
	}

	// Released: the end of a long press.
	if (key->held >= key->hold_polls) {
		earpods_report(input, key->code, 0);
		key->last = 0;
		key->code = 0;
		key->presses = 0;
		key->releases = 0;
		key->held = 0;
		key->idle = 0;
	}

clicks:
	if (key->presses > 0 && key->presses == key->releases && ++key->idle > key->reset_polls) {
		switch (key->presses) {
		case 1:
			code = KEY_PLAYPAUSE;
			break;
		case 2:
			code = KEY_NEXTSONG;
			break;
		case 3:
			code = KEY_PREVIOUSSONG;
			break;
		default:
			code = 0;
			break;
		}
		if (code) {
			earpods_report(input, code, 1);
			earpods_report(input, code, 0);
		}
		key->code = 0;
		key->presses = 0;
		key->releases = 0;
		key->held = 0;
		key->idle = 0;
	}
	key->last = pressed;
}

static void earpods_mic_power(struct earpods_adc *ea, bool on)
{
	if (!ea->reg || ea->reg_on == on)
		return;
	if (on) {
		if (!regulator_is_enabled(ea->reg))
			regulator_enable(ea->reg);
	} else {
		if (regulator_is_enabled(ea->reg))
			regulator_disable(ea->reg);
	}
	ea->reg_on = on;
}

static void earpods_release_all(struct earpods_adc *ea)
{
	int i;

	for (i = 0; i < ea->nkeys; i++)
		if (ea->keys[i].presses)
			ea->keys[i].process(&ea->keys[i], ea->input, 0);
}

static void earpods_unplugged(struct earpods_adc *ea)
{
	struct earpods_key *key;
	int i;

	earpods_mic_power(ea, false);
	for (i = 0; i < ea->nkeys; i++) {
		key = &ea->keys[i];
		if (key->presses)
			key->process(key, ea->input, 0);
		key->last = 0;
		key->code = 0;
		key->presses = 0;
		key->releases = 0;
		key->held = 0;
		key->idle = 0;
	}
	ea->keys[1].code = KEY_VOLUMEUP;
	ea->keys[2].code = KEY_VOLUMEDOWN;
	ea->settle = EARPODS_SETTLE_POLLS;
	ea->state = EARPODS_SETTLING;
}

static void earpods_adc_work(struct work_struct *work)
{
	struct earpods_adc *ea = container_of(to_delayed_work(work), struct earpods_adc, work);
	const char *sdev = earpods_adc_pdata.sdev_name;
	struct earpods_key *key;
	int i, mv;

	if (sdev && strcmp(sdev, "-1") && !get_switch_status(sdev)) {
		earpods_unplugged(ea);
		goto next;
	}

	earpods_mic_power(ea, true);
	if (ea->settle) {
		mv = adc_read_channel_voltage(ea->channel);
		if (mv >= ea->keys[0].min && mv < ea->keys[0].max) {
			ea->settle--;
			ea->state = EARPODS_SETTLING;
		} else {
			ea->settle = 0;
			ea->state = EARPODS_READY;
		}
	} else if (ea->state == EARPODS_READY) {
		mv = adc_read_channel_voltage(ea->channel);
		if (mv < 0)
			goto next;
		for (i = 0; i < ea->nkeys; i++) {
			key = &ea->keys[i];
			if (mv >= key->min && mv < key->max) {
				key->process(key, ea->input, 1);
				break;
			}
		}
		if (i == ea->nkeys)
			earpods_release_all(ea);
	}

next:
	if (gi_earpods_adc_sw)
		queue_delayed_work(system_wq, &ea->work, msecs_to_jiffies(ea->poll_ms));
}

static ssize_t get_earpods_adc_sw(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gi_earpods_adc_sw);
}

static ssize_t set_earpods_adc_sw(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct earpods_adc *ea = dev_get_drvdata(dev);

	if (!strncmp(buf, "on", 2)) {
		gi_earpods_adc_sw = 1;
		queue_delayed_work(system_wq, &ea->work, EARPODS_START_DELAY);
	} else {
		earpods_mic_power(ea, false);
		cancel_delayed_work(&ea->work);
		gi_earpods_adc_sw = 0;
		earpods_release_all(ea);
	}
	return count;
}

static DEVICE_ATTR(earpods_adc_sw, 0644, get_earpods_adc_sw, set_earpods_adc_sw);

static struct attribute *earpods_adc_attributes[] = {
	&dev_attr_earpods_adc_sw.attr,
	NULL,
};

static const struct attribute_group earpods_adc_group = {
	.name = "earpods_adc",
	.attrs = earpods_adc_attributes,
};

// A time in polls, rounded to the nearest.
static int earpods_polls(int ms, int poll_ms)
{
	return (poll_ms / 2 + ms) / poll_ms;
}

static int earpods_adc_probe(struct platform_device *pdev)
{
	struct earpods_adc_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct earpods_adc *ea;
	struct input_dev *input;
	struct earpods_key *key;
	int ret;

	ea = kzalloc(sizeof(*ea), GFP_KERNEL);
	if (!ea)
		return -ENOMEM;
	input = input_allocate_device();
	if (!input) {
		kfree(ea);
		return -ENOMEM;
	}

	input->dev.parent = &pdev->dev;
	input->name = pdev->name;
	input->phys = "earpods-adc/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0002;
	input->id.version = 0x0100;

	key = &ea->keys[0];
	key->min = pdata->play_pause_min;
	key->max = pdata->play_pause_max;
	key->process = earpods_play_pause_key_process;
	key->hold_polls = earpods_polls(pdata->press_hold_ms, pdata->poll_ms);
	key->repeat_polls = earpods_polls(pdata->press_hold_repeat_ms, pdata->poll_ms);
	key->reset_polls = earpods_polls(pdata->release_reset_ms, pdata->poll_ms);
	ea->nkeys = 1;
	input_set_capability(input, EV_KEY, KEY_NEXTSONG);
	input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);
	input_set_capability(input, EV_KEY, KEY_PLAYPAUSE);
	input_set_capability(input, EV_KEY, KEY_FASTFORWARD);
	input_set_capability(input, EV_KEY, KEY_REWIND);

	if (pdata->volume_up_min >= 0 && pdata->volume_up_max >= pdata->volume_up_min &&
	    pdata->volume_down_min > 0 && pdata->volume_down_max > pdata->volume_down_min) {
		key = &ea->keys[1];
		key->min = pdata->volume_up_min;
		key->max = pdata->volume_up_max;
		key->code = KEY_VOLUMEUP;
		key->process = earpods_volume_key_process;
		key = &ea->keys[2];
		key->min = pdata->volume_down_min;
		key->max = pdata->volume_down_max;
		key->code = KEY_VOLUMEDOWN;
		key->process = earpods_volume_key_process;
		ea->nkeys = 3;
		input_set_capability(input, EV_KEY, KEY_VOLUMEDOWN);
		input_set_capability(input, EV_KEY, KEY_VOLUMEUP);
	}

	if (pdata->pwr_name && strlen(pdata->pwr_name) && strcmp(pdata->pwr_name, "-1")) {
		ea->reg = regulator_get(&pdev->dev, pdata->pwr_name);
		if (IS_ERR(ea->reg)) {
			dev_warn(&pdev->dev, "Earpods adc %s regulator missing\n", pdata->pwr_name);
			ea->reg = NULL;
		} else {
			regulator_enable(ea->reg);
			ea->reg_on = true;
		}
	}
	ea->settle = EARPODS_SETTLE_POLLS;
	ea->state = EARPODS_SETTLING;

	ret = input_register_device(input);
	if (ret) {
		printk("earpods-adc: Unable to register input device: %d\n", ret);
		input_free_device(input);
		goto err_reg;
	}
	ret = sysfs_create_group(&pdev->dev.kobj, &earpods_adc_group);
	if (ret < 0) {
		printk("earpods-adc: Unable to create sysfs group: %d\n", ret);
		input_unregister_device(input);
		goto err_reg;
	}

	ea->channel = pdata->adc_channel;
	ea->input = input;
	ea->poll_ms = pdata->poll_ms;
	platform_set_drvdata(pdev, ea);
	INIT_DELAYED_WORK(&ea->work, earpods_adc_work);
	adc_enable();
	return 0;

err_reg:
	if (ea->reg) {
		if (ea->reg_on)
			regulator_disable(ea->reg);
		regulator_put(ea->reg);
	}
	kfree(ea);
	return ret;
}

static int earpods_adc_remove(struct platform_device *pdev)
{
	struct earpods_adc *ea = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&ea->work);
	sysfs_remove_group(&pdev->dev.kobj, &earpods_adc_group);
	adc_disable();
	device_init_wakeup(&pdev->dev, false);
	input_unregister_device(ea->input);
	if (ea->reg) {
		if (ea->reg_on)
			regulator_disable(ea->reg);
		regulator_put(ea->reg);
	}
	kfree(ea);
	return 0;
}

static int earpods_adc_suspend(struct device *dev)
{
	struct earpods_adc *ea = dev_get_drvdata(dev);

	printk("earpods_adc_suspend\n");
	cancel_delayed_work_sync(&ea->work);
	adc_disable();
	ea->suspended = true;
	return 0;
}

static int earpods_adc_resume(struct device *dev)
{
	struct earpods_adc *ea = dev_get_drvdata(dev);

	printk("earpods_adc_resume\n");
	adc_enable();
	ea->suspended = false;
	queue_delayed_work(system_wq, &ea->work, EARPODS_START_DELAY);
	return 0;
}

static const struct dev_pm_ops earpods_adc_pm_ops = {
	.suspend = earpods_adc_suspend,
	.resume = earpods_adc_resume,
};

static void earpods_adc_dev_release(struct device *dev)
{
}

static struct platform_device earpods_adc_dev = {
	.name = "earpods_adc",
	.id = -1,
	.dev = {
		.platform_data = &earpods_adc_pdata,
		.release = earpods_adc_dev_release,
	},
};

static struct platform_driver earpods_adc_drv = {
	.probe = earpods_adc_probe,
	.remove = earpods_adc_remove,
	.driver = {
		.name = "earpods_adc",
		.pm = &earpods_adc_pm_ops,
	},
};

static int __init earpods_adc_init(void)
{
	struct earpods_adc_platform_data *p = &earpods_adc_pdata;
	int ret;

	if (p->adc_channel < 0 || p->play_pause_min < 0 || p->play_pause_max < 0 ||
	    p->play_pause_max < p->play_pause_min || p->poll_ms <= 0 ||
	    p->press_hold_ms < p->poll_ms || p->press_hold_repeat_ms < p->poll_ms ||
	    p->release_reset_ms < p->poll_ms) {
		printk("earpods_adc_init: params error!\n");
		return -EINVAL;
	}

	ret = platform_device_register(&earpods_adc_dev);
	if (ret) {
		printk("failed to register earpods adc device\n");
		return ret;
	}
	return platform_driver_register(&earpods_adc_drv);
}

static void __exit earpods_adc_exit(void)
{
	platform_device_unregister(&earpods_adc_dev);
	platform_driver_unregister(&earpods_adc_drv);
}

module_init(earpods_adc_init);
module_exit(earpods_adc_exit);

MODULE_ALIAS("Earpods-adc");
MODULE_DESCRIPTION("Headset remote buttons on the ADC for the HiBy X1600 players");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
