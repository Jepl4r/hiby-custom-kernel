// SPDX-License-Identifier: GPL-2.0
//
// cst8xx_touch -- the Hynitron CST8xx touch panel of the HiBy R1, as the
// input device "hyn_ts" (multi-touch protocol A).
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock cst8xx_touch.sh passes them):
//
//   cst_i2c_bus_num, cst_i2c_addr   where the panel answers
//   cst_reset_gpio, cst_irq_gpio    reset (active low) and interrupt pins
//   cst_regulator_name              the panel supply ("": none)
//   cst_max_touch_number            contacts reported, 1 or 2 (the panel
//                                   tracks two)
//   cst_x_coords_*, cst_y_coords_*, cst_power_en_*   not read; positions
//                                   go out as the panel reports them, on a
//                                   720x1280 range
//
// The panel interrupts on each frame; a contact is reported on every other
// frame it is down. When the screen blanks the panel goes to sleep and its
// supply off; when it unblanks a worker powers it, resets it and waits for
// its chip ID before taking interrupts again.
//
// Loaded after utils.ko, which provides the pin parameters.

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;

#define HYN_TS_NAME		"hyn_ts"

#define HYN_REG_DATA		0x00	// the touch frame
#define HYN_REG_POWER_MODE	0xa5
#define HYN_REG_FW_VER		0xa6
#define HYN_REG_VENDOR_ID	0xa8
#define HYN_REG_CHIP_ID		0xaa

#define HYN_POWER_HIBERNATE	0x03

// Frame: register, gesture, number of contacts (bits 3..0), then 6 bytes a
// contact: event (bits 7..6) and X 11..8, X 7..0, ID (bits 7..4) and Y 11..8,
// Y 7..0, weight, area (bits 7..4).
#define HYN_POINT_LEN		6
#define HYN_POINTS_OFF		3
#define HYN_MAX_ID		2

#define HYN_EVENT_UP		1	// any other event: the contact is down

#define HYN_X_MAX		720
#define HYN_Y_MAX		1280
#define HYN_MAX_TOUCH		2

#define HYN_VDD_MIN_UV		2800000
#define HYN_VDD_MAX_UV		3300000
#define HYN_I2C_UV		1800000

struct hyn_ts_event {
	int x;
	int y;
	int p;
	int flag;
	int id;
	int area;
	int down;	// frames since it was last reported
};

struct hyn_ts_data {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *input_dev;
	struct workqueue_struct *ts_workqueue;
	struct work_struct resume_work;
	struct mutex report_mutex;
	struct mutex bus_lock;
	spinlock_t irq_lock;
	int irq;
	int reset_gpio;
	int irq_gpio;
	int max_touch_number;
	u8 ic_id;
	u8 fw_ver;
	u8 vendor_id;
	bool suspended;
	bool irq_disabled;
	struct hyn_ts_event *events;
	u8 *point_buf;
	int pnt_buf_size;
	int touchs;		// contacts reported in the last frame
	int point_num;
	int touch_points;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct notifier_block fb_notif;
};

static int cst_i2c_bus_num;
static int cst_i2c_addr = -1;
static int cst_x_coords_min;
static int cst_y_coords_min;
static int cst_x_coords_max;
static int cst_y_coords_max;
static int cst_x_coords_flip;
static int cst_y_coords_flip;
static int cst_x_y_coords_exchange;
static int cst_reset_gpio = -1;
static int cst_irq_gpio = -1;
static int cst_power_en_gpio = -1;
static int cst_power_en_level = -1;
static int cst_max_touch_number;
static char *cst_regulator_name = "";

module_param(cst_regulator_name, charp, 0644);
module_param(cst_x_y_coords_exchange, int, 0644);
module_param(cst_y_coords_flip, int, 0644);
module_param(cst_x_coords_flip, int, 0644);
module_param(cst_max_touch_number, int, 0644);
module_param(cst_power_en_level, int, 0644);
module_param_cb(cst_power_en_gpio, &param_gpio_ops, &cst_power_en_gpio, 0644);
module_param_cb(cst_irq_gpio, &param_gpio_ops, &cst_irq_gpio, 0644);
module_param_cb(cst_reset_gpio, &param_gpio_ops, &cst_reset_gpio, 0644);
module_param(cst_y_coords_max, int, 0644);
module_param(cst_x_coords_max, int, 0644);
module_param(cst_y_coords_min, int, 0644);
module_param(cst_x_coords_min, int, 0644);
module_param(cst_i2c_addr, int, 0644);
module_param(cst_i2c_bus_num, int, 0644);

static struct i2c_board_info hi_info = {
	I2C_BOARD_INFO("i2c-cst8xx", 0x15),
};

static struct i2c_client *hi_client;
static struct hyn_ts_data *hyn_data;

static int hi_i2c_read(u8 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2] = {
		{
			.addr = hi_client->addr,
			.flags = hi_client->flags & I2C_M_TEN,
			.len = 1,
			.buf = &reg,
		}, {
			.addr = hi_client->addr,
			.flags = (hi_client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	return i2c_transfer(hi_client->adapter, msgs, 2);
}

static int hi_i2c_write(u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };

	return i2c_master_send(hi_client, buf, sizeof(buf));
}

static int hyn_read(u8 reg, u8 *buf, int len)
{
	int ret;

	mutex_lock(&hyn_data->bus_lock);
	ret = hi_i2c_read(reg, buf, len);
	mutex_unlock(&hyn_data->bus_lock);
	return ret;
}

// Pulses reset, then waits delay_ms for the panel to start.
static void hyn_reset_proc(int delay_ms)
{
	gpio_direction_output(hyn_data->reset_gpio, 0);
	mdelay(20);
	gpio_direction_output(hyn_data->reset_gpio, 1);
	mdelay(delay_ms);
}

static void hyn_irq_disable(struct hyn_ts_data *ts)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->irq_lock, flags);
	if (!ts->irq_disabled) {
		disable_irq_nosync(ts->irq);
		ts->irq_disabled = true;
	}
	spin_unlock_irqrestore(&ts->irq_lock, flags);
}

static void hyn_irq_enable(struct hyn_ts_data *ts)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->irq_lock, flags);
	if (ts->irq_disabled) {
		enable_irq(ts->irq);
		ts->irq_disabled = false;
	}
	spin_unlock_irqrestore(&ts->irq_lock, flags);
}

static bool hyn_regulator_ok(struct regulator *reg)
{
	return reg && !IS_ERR(reg);
}

static void hyn_power_on(struct hyn_ts_data *ts)
{
	if (hyn_regulator_ok(ts->vdd))
		regulator_enable(ts->vdd);
	if (hyn_regulator_ok(ts->vcc_i2c))
		regulator_enable(ts->vcc_i2c);
}

// Supplies off and reset held.
static void hyn_power_off(struct hyn_ts_data *ts)
{
	if (hyn_regulator_ok(ts->vcc_i2c))
		regulator_disable(ts->vcc_i2c);
	if (hyn_regulator_ok(ts->vdd))
		regulator_disable(ts->vdd);
	gpio_direction_output(ts->reset_gpio, 0);
}

static void hyn_power_source_exit(struct hyn_ts_data *ts)
{
	hyn_power_off(ts);
	if (hyn_regulator_ok(ts->vdd)) {
		if (regulator_count_voltages(ts->vdd) > 0)
			regulator_set_voltage(ts->vdd, 0, HYN_VDD_MAX_UV);
		regulator_put(ts->vdd);
	}
	if (hyn_regulator_ok(ts->vcc_i2c)) {
		if (regulator_count_voltages(ts->vcc_i2c) > 0)
			regulator_set_voltage(ts->vcc_i2c, 0, HYN_I2C_UV);
		regulator_put(ts->vcc_i2c);
	}
}

// The supply named by cst_regulator_name, and "vcc_i2c" when there is one.
// With no supply named, nothing is switched.
static int hyn_power_source_init(struct hyn_ts_data *ts)
{
	int ret;

	ts->vdd = regulator_get(ts->dev, cst_regulator_name);
	if (!hyn_regulator_ok(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		printk(KERN_ERR "[HYN_TS/E]%s:get vdd regulator failed,ret=%d\n", __func__, ret);
		if (!ts->vdd)
			return 0;
		ts->vdd = NULL;
		return ret;
	}
	if (regulator_count_voltages(ts->vdd) > 0) {
		ret = regulator_set_voltage(ts->vdd, HYN_VDD_MIN_UV, HYN_VDD_MAX_UV);
		if (ret) {
			printk(KERN_ERR "[HYN_TS/E]%s:vdd regulator set_vtg failed ret=%d\n", __func__, ret);
			regulator_put(ts->vdd);
			ts->vdd = NULL;
			return ret;
		}
	}

	ts->vcc_i2c = regulator_get(ts->dev, "vcc_i2c");
	if (hyn_regulator_ok(ts->vcc_i2c) && regulator_count_voltages(ts->vcc_i2c) > 0) {
		ret = regulator_set_voltage(ts->vcc_i2c, HYN_I2C_UV, HYN_I2C_UV);
		if (ret) {
			printk(KERN_ERR "[HYN_TS/E]%s:vcc_i2c regulator set_vtg failed,ret=%d\n", __func__, ret);
			regulator_put(ts->vcc_i2c);
			ts->vcc_i2c = NULL;
		}
	}

	hyn_power_on(ts);
	return 0;
}

static void hyn_report_event(struct hyn_ts_data *ts, struct hyn_ts_event *ev)
{
	struct input_dev *input = ts->input_dev;

	input_report_abs(input, ABS_MT_TRACKING_ID, ev->id);
	if (ev->p <= 0)
		ev->p = 0x3f;
	input_report_abs(input, ABS_MT_PRESSURE, ev->p);
	if (ev->area <= 0)
		ev->area = 9;
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, ev->area);
	input_report_abs(input, ABS_MT_POSITION_X, ev->x);
	input_report_abs(input, ABS_MT_POSITION_Y, ev->y);
	input_mt_sync(input);
}

static irqreturn_t hyn_irq_handler(int irq, void *data)
{
	struct hyn_ts_data *ts = hyn_data;
	struct input_dev *input = ts->input_dev;
	u8 *buf = ts->point_buf;
	struct hyn_ts_event *ev;
	int max_touch = ts->max_touch_number;
	int i, ret, touchs = 0;
	bool report = false;

	memset(buf, 0xff, ts->pnt_buf_size);
	buf[0] = HYN_REG_DATA;
	ret = hyn_read(buf[0], buf, ts->pnt_buf_size);
	if (ret < 0) {
		printk(KERN_ERR "[HYN_TS/E]%s:read touchdata failed, ret:%d\n", "hyn_read_touchdata", ret);
		return IRQ_HANDLED;
	}

	ts->point_num = 0;
	ts->touch_points = buf[2] & 0x0f;
	if (ts->touch_points > max_touch)
		return IRQ_HANDLED;

	for (i = 0; i < max_touch; i++) {
		u8 *p = buf + HYN_POINTS_OFF + HYN_POINT_LEN * i;

		if ((p[2] >> 4) >= HYN_MAX_ID)
			break;
		ev = &ts->events[i];
		ev->x = ((p[0] & 0x0f) << 8) + p[1];
		ev->y = ((p[2] & 0x0f) << 8) + p[3];
		ev->flag = p[0] >> 6;
		ev->id = p[2] >> 4;
		ev->area = p[5] >> 4;
		ev->p = p[4];
		ts->point_num++;
	}

	if (!ts->point_num) {
		for (i = 0; i < max_touch; i++)
			ts->events[i].down = 0;
		return IRQ_HANDLED;
	}

	for (i = 0; i < ts->point_num; i++) {
		ev = &ts->events[i];
		if ((ev->flag & ~2) == 0) {
			if (ev->down == 0) {
				hyn_report_event(ts, ev);
				touchs++;
			}
			if (++ev->down >= 2)
				ev->down = 0;
		} else {
			ev->down = 0;
		}
		report = true;
	}

	if (ts->touchs && !touchs)
		report = true;
	ts->touchs = touchs;

	if (report) {
		if (!ts->touch_points) {
			input_report_key(input, BTN_TOUCH, 0);
			input_mt_sync(input);
		} else {
			input_report_key(input, BTN_TOUCH, 1);
		}
	}
	input_sync(input);
	return IRQ_HANDLED;
}

static void hyn_ts_suspend(struct hyn_ts_data *ts)
{
	int ret;

	if (ts->suspended) {
		printk(KERN_INFO "[HYN_TS/I]%s:Already in suspend state\n", __func__);
		return;
	}

	hyn_irq_disable(ts);

	mutex_lock(&ts->bus_lock);
	ret = hi_i2c_write(HYN_REG_POWER_MODE, HYN_POWER_HIBERNATE);
	mutex_unlock(&ts->bus_lock);
	if (ret < 0)
		printk(KERN_ERR "[HYN_TS/E]%s:set TP to sleep mode fail, ret=%d\n", __func__, ret);

	hyn_power_off(ts);
	ts->suspended = true;
}

// Releases every contact, powers the panel and waits for its chip ID.
static void hyn_resume_work(struct work_struct *work)
{
	struct hyn_ts_data *ts = hyn_data;
	struct input_dev *input = ts->input_dev;
	u8 id;
	int i, ret;

	if (!ts->suspended)
		return;

	mutex_lock(&ts->report_mutex);
	input_mt_sync(input);
	input_report_key(input, BTN_TOUCH, 0);
	input_sync(input);
	ts->touchs = 0;
	mutex_unlock(&ts->report_mutex);

	hyn_power_on(ts);
	hyn_reset_proc(200);

	for (i = 0; i < 5; i++) {
		id = 0;
		ret = hyn_read(HYN_REG_CHIP_ID, &id, 1);
		if (ret >= 0 && id == ts->ic_id)
			break;
		msleep(200);
	}

	hyn_irq_enable(ts);
	ts->suspended = false;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int blank;

	if (event != FB_EVENT_BLANK)
		return 0;
	blank = *(int *)evdata->data;
	if (blank == FB_BLANK_UNBLANK)
		queue_work(hyn_data->ts_workqueue, &hyn_data->resume_work);
	else if (blank == FB_BLANK_POWERDOWN)
		hyn_ts_suspend(hyn_data);
	return 0;
}

static int hyn_input_init(struct hyn_ts_data *ts)
{
	struct input_dev *input;
	int ret;

	input = input_allocate_device();
	if (!input) {
		printk(KERN_ERR "[HYN_TS/E]%s:Failed to allocate memory for input device\n", __func__);
		return -ENOMEM;
	}

	input->name = HYN_TS_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = ts->dev;
	input_set_drvdata(input, ts);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 15, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, HYN_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, HYN_Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	ret = input_register_device(input);
	if (ret) {
		printk(KERN_ERR "[HYN_TS/E]%s:Input device registration failed\n", __func__);
		input_set_drvdata(input, NULL);
		input_free_device(input);
		return ret;
	}
	ts->input_dev = input;
	return 0;
}

static int hyn_report_buffer_init(struct hyn_ts_data *ts)
{
	int n = ts->max_touch_number;

	ts->pnt_buf_size = HYN_POINTS_OFF + HYN_POINT_LEN * n;
	ts->point_buf = kzalloc(ts->pnt_buf_size + 1, GFP_KERNEL);
	if (!ts->point_buf) {
		printk(KERN_ERR "[HYN_TS/E]%s:failed to alloc memory for point buf\n", __func__);
		return -ENOMEM;
	}
	ts->events = kzalloc(n * sizeof(*ts->events), GFP_KERNEL);
	if (!ts->events) {
		printk(KERN_ERR "[HYN_TS/E]%s:failed to alloc memory for point events\n", __func__);
		kfree(ts->point_buf);
		ts->point_buf = NULL;
		return -ENOMEM;
	}
	return 0;
}

// Chip ID, firmware version and vendor ID; 0xff, 0 and 0xff when the panel
// does not answer within 1 s.
static void hyn_get_ic_information(struct hyn_ts_data *ts)
{
	u8 id[3] = { 0 };
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = hyn_read(HYN_REG_CHIP_ID, &id[0], 1);
		ret |= hyn_read(HYN_REG_FW_VER, &id[1], 1);
		ret |= hyn_read(HYN_REG_VENDOR_ID, &id[2], 1);
		if (ret >= 0) {
			ts->ic_id = id[0];
			ts->fw_ver = id[1];
			ts->vendor_id = id[2];
			return;
		}
		msleep(200);
	}

	printk(KERN_INFO "[HYN_TS/I]%s:read chip_ID fail !!!\n", __func__);
	ts->ic_id = 0xff;
	ts->fw_ver = 0;
	ts->vendor_id = 0xff;
}

static int i2cdev_init(void)
{
	struct i2c_adapter *adapter;

	hi_info.addr = cst_i2c_addr;
	adapter = i2c_get_adapter(cst_i2c_bus_num);
	if (!adapter) {
		dev_err(NULL, "i2c_get_adapter error()!\n");
		return -1;
	}
	hi_client = i2c_new_device(adapter, &hi_info);
	i2c_put_adapter(adapter);
	if (!hi_client) {
		dev_err(NULL, "i2c_new_device error()!\n");
		return -1;
	}
	return 0;
}

static void i2cdev_exit(void)
{
	if (hi_client)
		i2c_unregister_device(hi_client);
}

static int __init hyn_ts_init(void)
{
	struct hyn_ts_data *ts_data;
	int ret;

	printk("%s load touch driver\n", __func__);
	if (i2cdev_init()) {
		dev_err(NULL, " i2cdev_init fail!\n");
		return -1;
	}

	ts_data = kzalloc(sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data) {
		printk(KERN_ERR "[HYN_TS/E]%s:allocate memory for hyn_data fail\n", "hyn_ts_probe");
		ret = -ENOMEM;
		goto err_client;
	}
	hyn_data = ts_data;
	ts_data->client = hi_client;
	ts_data->dev = &hi_client->dev;
	ts_data->max_touch_number = clamp(cst_max_touch_number, 1, HYN_MAX_TOUCH);
	ts_data->reset_gpio = cst_reset_gpio;
	ts_data->irq_gpio = cst_irq_gpio;
	spin_lock_init(&ts_data->irq_lock);

	gpio_request(ts_data->reset_gpio, "tp_reset");
	gpio_request(ts_data->irq_gpio, "tp_irq");

	ts_data->ts_workqueue = create_singlethread_workqueue("hyn_wq");
	if (!ts_data->ts_workqueue)
		printk(KERN_ERR "[HYN_TS/E]%s:create fts workqueue fail\n", "hyn_ts_probe_entry");
	mutex_init(&ts_data->report_mutex);
	mutex_init(&ts_data->bus_lock);

	ret = hyn_input_init(ts_data);
	if (ret) {
		printk(KERN_ERR "[HYN_TS/E]%s:input initialize fail\n", "hyn_ts_probe_entry");
		goto err_gpio;
	}

	ret = hyn_report_buffer_init(ts_data);
	if (ret) {
		printk(KERN_ERR "[HYN_TS/E]%s:report buffer init fail\n", "hyn_ts_probe_entry");
		goto err_input;
	}

	ret = hyn_power_source_init(ts_data);
	if (ret) {
		printk(KERN_ERR "[HYN_TS/E]%s:fail to get power(regulator)\n", "hyn_ts_probe_entry");
		goto err_power;
	}

	hyn_reset_proc(200);
	hyn_get_ic_information(ts_data);

	ts_data->irq = gpiod_to_irq(gpio_to_desc(ts_data->irq_gpio));
	printk(KERN_INFO "[HYN_TS/I]%s:== irq:%d, flag:%x\n", "hyn_irq_registration", ts_data->irq,
	       IRQF_TRIGGER_FALLING | IRQF_ONESHOT);
	ret = request_threaded_irq(ts_data->irq, NULL, hyn_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   HYN_TS_NAME, ts_data);
	if (ret) {
		printk(KERN_ERR "[HYN_TS/E]%s:request irq failed\n", "hyn_ts_probe_entry");
		goto err_power;
	}

	if (ts_data->ts_workqueue)
		INIT_WORK(&ts_data->resume_work, hyn_resume_work);
	ts_data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts_data->fb_notif);
	if (ret)
		printk(KERN_ERR "[HYN_TS/E]%s:[FB]Unable to register fb_notifier: %d\n", "hyn_ts_probe_entry", ret);

	printk(KERN_INFO "[HYN_TS/I]%s:Touch Screen(I2C BUS) driver prboe successfully\n", "hyn_ts_probe");
	return 0;

err_power:
	hyn_power_source_exit(ts_data);
	kfree(ts_data->point_buf);
	kfree(ts_data->events);
err_input:
	input_unregister_device(ts_data->input_dev);
err_gpio:
	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);
	gpio_free(ts_data->irq_gpio);
	gpio_free(ts_data->reset_gpio);
	printk(KERN_ERR "[HYN_TS/E]%s:Touch Screen(I2C BUS) driver probe fail\n", "hyn_ts_probe");
	kfree(ts_data);
	hyn_data = NULL;
err_client:
	i2cdev_exit();
	printk(KERN_ERR "[HYN_TS/E]%s:Focaltech touch screen driver init failed!\n", __func__);
	return ret;
}
module_init(hyn_ts_init);

static void __exit hyn_ts_exit(void)
{
	struct hyn_ts_data *ts = hyn_data;

	free_irq(ts->irq, ts);
	if (fb_unregister_client(&ts->fb_notif))
		printk(KERN_ERR "[HYN_TS/E]%s:Error occurred while unregistering fb_notifier.\n",
		       "hyn_ts_remove_entry");
	input_unregister_device(ts->input_dev);
	i2cdev_exit();
	if (ts->ts_workqueue)
		destroy_workqueue(ts->ts_workqueue);
	hyn_power_source_exit(ts);
	gpio_free(ts->irq_gpio);
	gpio_free(ts->reset_gpio);
	kfree(ts->point_buf);
	kfree(ts->events);
	kfree(ts);
}
module_exit(hyn_ts_exit);

MODULE_DESCRIPTION("CST8xx touch panel of the HiBy R1");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL v2");
