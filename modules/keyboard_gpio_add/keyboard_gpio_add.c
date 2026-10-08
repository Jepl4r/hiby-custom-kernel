// SPDX-License-Identifier: GPL-2.0
//
// keyboard_gpio_add -- the keys on pins of the HiBy R3 Pro II and R1 (power,
// and next or previous track), as the input device md-gpio-keys.
//
// Based on drivers/input/keyboard/gpio_keys.c, Copyright 2005 Phil Blundell,
// Copyright 2010, 2011 David Jander <david@protonic.nl>, in its timer and
// work version, with the keys described by a module parameter instead of
// board code; the pins also get a pull (active low) or none (active high).
//
// A drop-in replacement for the vendor module of the same name. It is set up
// through its one parameter, keyboard, written one line at a time (the
// stock keyboard_gpio_add.sh writes them):
//
//   alloc=N                  room for N keys, once
//   gpio=PIN key_code=N tag=NAME active_level=0|1 [wakeup=y|n]
//                            a key; without wakeup= it wakes the system
//   register                 registers the device md-gpio-keys with the keys
//
// Reading the parameter lists the keys, one line each.
//
// Each key is debounced for 20 ms. The device has the sysfs files keys,
// switches, disabled_keys and disabled_switches of gpio_keys.c.
//
// Loaded after utils.ko, which provides gpio_set_func(), pin names such as
// "PC31" and the word splitting.

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

// Exported by utils.ko.
extern int gpio_set_func(int gpio, int func);
extern char *gpio_to_str(int gpio, char *buf);
extern char **str_to_words(const char *str, int *count);
extern void str_free_words(char **words);
extern int str_to_gpio(const char *str);

#define GPIO_PULL_HIZ		0x8000	// gpio_set_func(): no pull
#define GPIO_PULL		0xa000	// gpio_set_func(): pull

#define KEY_DEBOUNCE_MS		20

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	// ms
	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct input_dev *input;
	struct mutex disable_lock;
	struct gpio_button_data data[0];
};

// What register sends to the driver: the device, its platform data and the
// keys, in one allocation.
struct keyboard_data {
	struct platform_device pdev;
	struct gpio_keys_platform_data pdata;
	struct gpio_keys_button buttons[0];
};

static struct keyboard_data *keyboard_data;	// being filled by key lines
static struct keyboard_data *keyboard_dev;	// registered
static int keyboard_total_cnt;			// keys allocated
static int keyboard_bnt_cnt;			// keys written since the last register
static int keyboard_save_cnt;			// keys handed to the last register

static void keyboard_release(struct device *dev)
{
	struct keyboard_data *d = container_of(dev, struct keyboard_data, pdev.dev);
	int i;

	for (i = 0; i < d->pdata.nbuttons; i++)
		kfree(d->buttons[i].desc);
	kfree(d);
}

static int keyboard_register(void)
{
	struct keyboard_data *d = keyboard_data;
	int ret;

	if (!keyboard_bnt_cnt) {
		pr_err("can not register, no key is set!\n");
		return -ENODEV;
	}
	d->pdev.name = "md-gpio-keys";
	d->pdev.id = -1;
	d->pdev.dev.platform_data = &d->pdata;
	d->pdev.dev.release = keyboard_release;
	d->pdata.buttons = d->buttons;
	d->pdata.nbuttons = keyboard_bnt_cnt;
	keyboard_save_cnt = keyboard_bnt_cnt;
	keyboard_total_cnt = 0;
	keyboard_bnt_cnt = 0;
	keyboard_data = NULL;
	ret = platform_device_register(&d->pdev);
	if (ret) {
		platform_device_put(&d->pdev);
		keyboard_save_cnt = 0;
		return ret;
	}
	keyboard_dev = d;
	return 0;
}

// The value of "key=..." in w, or NULL for another word.
static const char *keyboard_arg(const char *w, const char *key)
{
	return strncmp(w, key, strlen(key)) ? NULL : w + strlen(key);
}

static int param_keyboard_set(const char *val, const struct kernel_param *kp)
{
	int gpio = -1, active_low = -1, wakeup = -1;
	bool do_alloc = false, do_register = false;
	struct gpio_keys_button *b;
	unsigned long code = -1, n;
	const char *v, *w;
	char *tag = NULL;
	char **words;
	int count, i, ret = 0;

	words = str_to_words(val, &count);
	for (i = 0; i < count && !ret; i++) {
		w = words[i];
		if ((v = keyboard_arg(w, "alloc="))) {
			if (kstrtoul(v, 0, &n)) {
				pr_err("%s is not a valid num\n", w);
				ret = -EINVAL;
			} else if (!n) {
				ret = -EINVAL;
			} else if (keyboard_total_cnt) {
				pr_err("new alloc %ld can not be done, old keys not register!\n", n);
				ret = -EINVAL;
			} else {
				keyboard_total_cnt = n;
				do_alloc = true;
			}
		} else if (!strcmp(w, "register")) {
			do_register = true;
			break;
		} else if ((v = keyboard_arg(w, "gpio="))) {
			gpio = str_to_gpio(v);
			if (gpio < -1) {
				pr_err("%s is not a gpio\n", w);
				ret = -EINVAL;
			}
		} else if ((v = keyboard_arg(w, "key_code="))) {
			if (kstrtoul(v, 0, &code)) {
				pr_err("%s is not a valid num\n", w);
				ret = -EINVAL;
			}
		} else if ((v = keyboard_arg(w, "tag="))) {
			kfree(tag);
			tag = kstrdup(v, GFP_KERNEL);
		} else if ((v = keyboard_arg(w, "active_level="))) {
			if (!strcmp(v, "0")) {
				active_low = 1;
			} else if (!strcmp(v, "1")) {
				active_low = 0;
			} else {
				pr_err("%s active_level only support '0' or '1'\n", w);
				ret = -EINVAL;
			}
		} else if ((v = keyboard_arg(w, "wakeup="))) {
			wakeup = !strcmp(v, "y");
		} else {
			pr_err("%s can not be parse!\n", w);
			ret = -EINVAL;
		}
	}
	str_free_words(words);
	if (ret)
		goto out;

	if (do_alloc) {
		keyboard_data = kzalloc(sizeof(*keyboard_data) +
					keyboard_total_cnt * sizeof(struct gpio_keys_button), GFP_KERNEL);
		if (!keyboard_data) {
			pr_err("failed to alloc %d keys\n", keyboard_total_cnt);
			keyboard_total_cnt = 0;
		}
		goto out;
	}
	if (do_register) {
		ret = keyboard_register();
		goto out;
	}

	if (gpio == -1) {
		pr_err("gpio must define!\n");
		ret = -EINVAL;
	} else if (active_low == -1) {
		pr_err("activel_level must define!\n");
		ret = -EINVAL;
	} else if (code == -1) {
		pr_err("key_code must define!\n");
		ret = -EINVAL;
	} else if (keyboard_bnt_cnt >= keyboard_total_cnt) {
		pr_err("too many gpio keys, total:%d!\n", keyboard_total_cnt);
		ret = -ERANGE;
	} else {
		b = &keyboard_data->buttons[keyboard_bnt_cnt++];
		b->code = code;
		b->gpio = gpio;
		b->active_low = active_low;
		b->desc = tag;
		b->wakeup = wakeup;
		b->debounce_interval = KEY_DEBOUNCE_MS;
		return 0;
	}
out:
	kfree(tag);
	return ret;
}

static int param_keyboard_get(char *buffer, const struct kernel_param *kp)
{
	struct keyboard_data *d = keyboard_bnt_cnt ? keyboard_data : keyboard_dev;
	int n = keyboard_bnt_cnt ? keyboard_bnt_cnt : keyboard_save_cnt;
	struct gpio_keys_button *b;
	char *p = buffer;
	char pin[16];
	int i;

	for (i = 0; i < n; i++) {
		b = &d->buttons[i];
		p += sprintf(p, "gpio=%s code=%d tag=%s active_level=%s wakeup=%s\n",
			     gpio_to_str(b->gpio, pin), b->code, b->desc,
			     b->active_low ? "low" : "high", b->wakeup ? "true" : "false");
	}
	return p - buffer;
}

static const struct kernel_param_ops param_keyboard_ops = {
	.set = param_keyboard_set,
	.get = param_keyboard_get,
};
module_param_cb(keyboard, &param_keyboard_ops, NULL, 0644);

// The input event code count of a type, for the sysfs bitmaps.
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		disable_irq(bdata->irq);
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);
		bdata->disabled = true;
	}
}

static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

// The codes of the buttons of a type that can be disabled, or only those
// that are disabled, as a list of ranges.
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata, char *buf,
					  unsigned int type, bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;
		if (only_disabled && !bdata->disabled)
			continue;
		__set_bit(bdata->button->code, bits);
	}

	ret = scnprintf(buf, PAGE_SIZE - 1, "%*pbl", n_events, bits);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);
	return ret;
}

// Disables the buttons of a type whose codes are in the list, enables the
// others; refused if a listed button cannot be disabled.
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata, const char *buf,
					   unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;
		if (test_bit(bdata->button->code, bits) && !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);
	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;
		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}
	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)					\
static ssize_t gpio_keys_show_##name(struct device *dev,			\
				     struct device_attribute *attr, char *buf)	\
{										\
	struct platform_device *pdev = to_platform_device(dev);			\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);		\
										\
	return gpio_keys_attr_show_helper(ddata, buf, type, only_disabled);	\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)						\
static ssize_t gpio_keys_store_##name(struct device *dev,			\
				      struct device_attribute *attr,		\
				      const char *buf, size_t count)		\
{										\
	struct platform_device *pdev = to_platform_device(dev);			\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);		\
	ssize_t error;								\
										\
	error = gpio_keys_attr_store_helper(ddata, buf, type);			\
	if (error)								\
		return error;							\
	return count;								\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO, gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO, gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;

	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		input_event(input, type, button->code, !!state);
	}
	input_sync(input);
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata = container_of(work, struct gpio_button_data, work);

	gpio_keys_gpio_report_event(bdata);
	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static void gpio_keys_gpio_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;

	schedule_work(&bdata->work);
}

// A pin edge: the state is read once it has been stable for the debounce
// time.
static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);
	if (bdata->timer_debounce)
		mod_timer(&bdata->timer, jiffies + msecs_to_jiffies(bdata->timer_debounce));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, bdata->button->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

// A key on an interrupt only: pressed at the interrupt, released after the
// debounce time.
static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

		input_event(input, EV_KEY, button->code, 1);
		input_sync(input);

		if (!bdata->timer_debounce) {
			input_event(input, EV_KEY, button->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer, jiffies + msecs_to_jiffies(bdata->timer_debounce));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}

static int gpio_keys_setup_key(struct platform_device *pdev, struct input_dev *input,
			       struct gpio_button_data *bdata,
			       const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (gpio_is_valid(button->gpio)) {
		error = gpio_request_one(button->gpio, GPIOF_IN, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n", button->gpio, error);
			return error;
		}

		if (button->debounce_interval) {
			error = gpio_set_debounce(button->gpio, button->debounce_interval * 1000);
			// A timer when the pin has no debounce of its own.
			if (error < 0)
				bdata->timer_debounce = button->debounce_interval;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}
		bdata->irq = irq;

		INIT_WORK(&bdata->work, gpio_keys_gpio_work_func);
		setup_timer(&bdata->timer, gpio_keys_gpio_timer, (unsigned long)bdata);

		gpio_set_func(button->gpio, button->active_low ? GPIO_PULL : GPIO_PULL_HIZ);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	} else {
		if (!button->irq) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->timer_debounce = button->debounce_interval;
		setup_timer(&bdata->timer, gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}

	input_set_capability(input, button->type ?: EV_KEY, button->code);

	// Shared unless userspace may disable it.
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n", bdata->irq, error);
		goto fail;
	}

	return 0;

fail:
	if (gpio_is_valid(button->gpio))
		gpio_free(button->gpio);
	return error;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (gpio_is_valid(bdata->button->gpio))
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	// The current state of the keys on pins.
	gpio_keys_report_state(ddata);
	return 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

#ifdef CONFIG_OF
// The keys of a device tree node, one child each.
static struct gpio_keys_platform_data *gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = kzalloc(sizeof(*pdata) + nbuttons * sizeof(*button), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);

	i = 0;
	for_each_child_of_node(node, pp) {
		enum of_gpio_flags flags;
		int gpio;

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->nbuttons--;
			dev_warn(dev, "Found button without gpios\n");
			continue;
		}

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (gpio < 0) {
			error = gpio;
			if (error != -EPROBE_DEFER)
				dev_err(dev, "Failed to get gpio flags, error: %d\n", error);
			goto err_free_pdata;
		}

		button = &pdata->buttons[i++];

		button->gpio = gpio;
		button->active_low = flags & OF_GPIO_ACTIVE_LOW;

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n", button->gpio);
			error = -EINVAL;
			goto err_free_pdata;
		}

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = !!of_get_property(pp, "gpio-key,wakeup", NULL);

		if (of_property_read_u32(pp, "debounce-interval", &button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0) {
		error = -EINVAL;
		goto err_free_pdata;
	}

	return pdata;

err_free_pdata:
	kfree(pdata);
	return ERR_PTR(error);
}

static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "md-gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);
#else
static inline struct gpio_keys_platform_data *gpio_keys_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}
#endif

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->timer_debounce)
		del_timer_sync(&bdata->timer);
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->button->gpio))
		gpio_free(bdata->button->gpio);
}

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data), GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "md-gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	// Autorepeat of the input core.
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n", error);
		goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n", error);
		goto fail3;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
fail1:
	input_free_device(input);
	kfree(ddata);
	// Allocated above when it came from the device tree.
	if (!dev_get_platdata(&pdev->dev))
		kfree(pdata);

	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->pdata->nbuttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);

	// Allocated by the probe when it came from the device tree.
	if (!dev_get_platdata(&pdev->dev))
		kfree(ddata->pdata);

	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];

			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_keys_close(input);
		mutex_unlock(&input->mutex);
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];

			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;

	gpio_keys_report_state(ddata);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "md-gpio-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

// Also takes away the device registered through the parameter, and the keys
// written but not registered.
static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
	if (keyboard_dev)
		platform_device_unregister(&keyboard_dev->pdev);
	if (keyboard_data) {
		while (keyboard_bnt_cnt)
			kfree(keyboard_data->buttons[--keyboard_bnt_cnt].desc);
		kfree(keyboard_data);
	}
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_ALIAS("platform:md-gpio-keys");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
