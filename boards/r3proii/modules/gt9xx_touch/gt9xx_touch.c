// SPDX-License-Identifier: GPL-2.0
//
// gt9xx_touch -- the Goodix GT9xx touch panel of the HiBy R3 Pro II (a
// GT967), as the input device "goodix-ts" (multi-touch protocol A).
//
// A drop-in replacement for the vendor module of the same name, based on
// Goodix's GT9xx driver 2.4, with the same parameters (the stock
// gt9xx_touch.sh passes them):
//
//   gtp_i2c_bus_num                 I2C bus of the panel (address 0x14)
//   gtp_reset_gpio, gtp_irq_gpio    reset and interrupt pins
//   gtp_power_on_gpio               requested and held, not driven
//   gtp_regulator_name              the panel supply ("" or "-1": none)
//   gtp_x_coords_max, gtp_y_coords_max   the resolution written into the
//                                   panel configuration and the range of
//                                   the reported positions
//   gtp_x_coords_flip, gtp_y_coords_flip, gtp_x_y_coords_exchange
//                                   position transforms
//   gtp_max_touch_number            contacts reported, at most 10
//   gtp_x_coords_min, gtp_y_coords_min   not read
//
// At probe the panel is reset with the address strap on its interrupt pin
// and sent the configuration below. Each interrupt queues a worker that reads
// the frame at 0x814E and reports every contact in it.
//
// gesture_sw on the I2C device switches gesture wake: with it on, a blank
// screen puts the panel in doze mode instead of switching it off, and a
// gesture or a double tap is reported as KEY_POWER. Without it, a blank
// screen frees the interrupt and cuts the supply; unblanking resets the panel,
// sends the configuration again and takes the interrupt back.
//
// /proc/gt9xx_config shows the configuration sent and the one the panel
// holds; writing up to 240 bytes to it sends them as the configuration.
//
// Loaded after utils.ko, which provides the pin parameters and
// i2c_register_device().

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);

#define GTP_DRIVER_VERSION	"V2.4.0.1<2016/10/26>"
#define GTP_I2C_NAME		"Goodix-TS"
#define GTP_I2C_ADDR		0x14

#define GTP_ADDR_LENGTH		2
#define GTP_CONFIG_MIN_LENGTH	186
#define GTP_CONFIG_MAX_LENGTH	240
#define GTP_MAX_TOUCH		10
#define GTP_POLL_TIME		10	// ms, polling mode
#define GTP_INT_TRIGGER		1	// falling edge

#define FAIL			0
#define SUCCESS			1

#define GTP_READ_COOR_ADDR	0x814E	// status, then 8 bytes a contact
#define GTP_REG_DOZE_GESTURE	0x814B	// the gesture that woke the panel
#define GTP_REG_SLEEP		0x8040	// command
#define GTP_REG_DOZE_FLAG	0x8046
#define GTP_REG_SENSOR_ID	0x814A
#define GTP_REG_CONFIG_DATA	0x8047
#define GTP_REG_VERSION		0x8140
#define GTP_REG_FW_STATE	0x41E4	// 0xBE: the firmware runs

#define GTP_CMD_DOZE		8

// Positions in the configuration, counted from its register address.
#define RESOLUTION_LOC		3
#define TRIGGER_LOC		8

#define GTP_STATUS_READY	0x80	// status byte: a frame is ready
#define GTP_STATUS_TOUCHES	0x0f

enum doze {
	DOZE_DISABLED,
	DOZE_ENABLED,	// the panel is in doze mode, waiting for a gesture
	DOZE_WAKEUP,	// a gesture woke it
};

#define GTP_INFO(fmt, arg...)	printk("<<-GTP-INFO->> " fmt "\n", ##arg)
#define GTP_ERROR(fmt, arg...)	printk("<<-GTP-ERROR->> " fmt "\n", ##arg)

struct goodix_ts_data {
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct hrtimer timer;		// polling mode, without an interrupt
	struct delayed_work work;	// reads and reports a frame
	struct delayed_work reset_work;	// powers the panel back up at unblank
	s32 irq_is_disable;
	s32 use_irq;
	bool irq_requested;
	bool power_gpio_owned;
	bool rst_gpio_owned;
	u16 abs_x_max;
	u16 abs_y_max;
	u8 int_trigger_type;
	u8 gtp_is_suspend;
	u8 pnl_init_error;		// no valid configuration: none is sent
	struct notifier_block notifier;
};

static int i2c_bus_num;
static int gtp_x_min;
static int gtp_y_min;
static int gtp_x_max;
static int gtp_y_max;
static int gtp_power_on_gpio;
static int gtp_rst_gpio;
static int gtp_int_gpio;
static int gtp_max_touch_number;
static int gtp_x_coords_flip;
static int gtp_y_coords_flip;
static int gtp_x_y_coords_exchange;
static char *gtp_regulator_name = "";

module_param_named(gtp_i2c_bus_num, i2c_bus_num, int, 0644);
module_param_named(gtp_x_coords_min, gtp_x_min, int, 0644);
module_param_named(gtp_y_coords_min, gtp_y_min, int, 0644);
module_param_named(gtp_x_coords_max, gtp_x_max, int, 0644);
module_param_named(gtp_y_coords_max, gtp_y_max, int, 0644);
module_param_cb(gtp_power_on_gpio, &param_gpio_ops, &gtp_power_on_gpio, 0644);
module_param_cb(gtp_reset_gpio, &param_gpio_ops, &gtp_rst_gpio, 0644);
module_param_cb(gtp_irq_gpio, &param_gpio_ops, &gtp_int_gpio, 0644);
module_param(gtp_max_touch_number, int, 0644);
module_param(gtp_x_coords_flip, int, 0644);
module_param(gtp_y_coords_flip, int, 0644);
module_param(gtp_x_y_coords_exchange, int, 0644);
module_param(gtp_regulator_name, charp, 0644);

static struct i2c_client *i2c_connect_client;
static struct workqueue_struct *goodix_wq;
static struct proc_dir_entry *gt91xx_config_proc;
static int doze_status = DOZE_DISABLED;
static int gesture_sw_val;

// The configuration sent to the panel: register address, then the bytes.
static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] = {
	GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff,
};

// The R3 Pro II panel's configuration, 228 bytes from 0x8047: version 0x45,
// 480x720, 5 contacts, falling-edge interrupt. The last byte, the checksum,
// is computed when it is sent.
static const u8 cfg_info_group1[] = {
	0x45, 0xe0, 0x01, 0xd0, 0x02, 0x05, 0x0d, 0x02, 0x01, 0x0a, 0x28, 0x0d,
	0x5a, 0x3c, 0x03, 0xd5, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x05, 0x17,
	0x19, 0x1d, 0x14, 0x85, 0x25, 0x78, 0x27, 0x29, 0x0c, 0x08, 0xb8, 0x08,
	0x60, 0x1b, 0x33, 0x11, 0x14, 0x00, 0x00, 0x00, 0xe0, 0x83, 0x7b, 0x00,
	0x3c, 0x28, 0x00, 0x1e, 0x41, 0x94, 0xd5, 0x02, 0x07, 0x00, 0x00, 0x04,
	0x9a, 0x20, 0x00, 0x86, 0x25, 0x00, 0x72, 0x2c, 0x00, 0x63, 0x33, 0x00,
	0x56, 0x3c, 0x00, 0x56, 0x00, 0x00, 0x00, 0x00, 0x78, 0x50, 0x35, 0xcf,
	0xff, 0x27, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x28, 0x27,
	0x26, 0x25, 0x24, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x0e, 0x0d, 0x29, 0x2a,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xf0, 0xf0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x7f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x01,
};

// IRQ flags by the trigger in the configuration.
static const u8 irq_table[] = {
	IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH,
};

static const char *const slide_direction[] = { "Right", "Down", "Up", "Left" };

static void gtp_reset_guitar(struct i2c_client *client, s32 ms);

// --- interrupt ----------------------------------------------------------------

static void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->irq_lock, flags);
	if (!ts->irq_is_disable) {
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, flags);
}

static void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->irq_lock, flags);
	if (ts->irq_is_disable && ts->irq_requested) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, flags);
}

// The worker is queued with the interrupt off; it turns it back on when the
// frame is done.
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	gtp_irq_disable(ts);
	if (!delayed_work_pending(&ts->work))
		queue_delayed_work(goodix_wq, &ts->work, 0);
	else
		gtp_irq_enable(ts);
	return IRQ_HANDLED;
}

static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

	queue_delayed_work(goodix_wq, &ts->work, 0);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

// Takes the interrupt, off; without it the panel is polled every 16 ms.
static int gtp_request_irq(struct goodix_ts_data *ts)
{
	int ret;

	ret = request_irq(ts->client->irq, goodix_ts_irq_handler, irq_table[ts->int_trigger_type],
			  ts->client->name, ts);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		gpio_direction_input(gtp_int_gpio);
		gpio_free(gtp_int_gpio);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return -1;
	}
	ts->irq_requested = true;
	gtp_irq_disable(ts);
	ts->use_irq = 1;
	return 0;
}

// --- I2C ----------------------------------------------------------------------

// buf: the 2-byte register address, then room for len - 2 bytes. Five tries;
// after that the panel is reset, unless it dozes. Returns i2c_transfer()'s
// result.
static int gtp_i2c_read(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = GTP_ADDR_LENGTH, .buf = buf },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len - GTP_ADDR_LENGTH,
		  .buf = buf + GTP_ADDR_LENGTH },
	};
	int retries, ret;

	for (retries = 5; retries; retries--) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			return ret;
	}
	if (doze_status == DOZE_ENABLED)
		return ret;
	GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.",
		  (buf[0] << 8) | buf[1], len - 2, ret);
	gtp_reset_guitar(client, 10);
	return ret;
}

// buf: the register address, then the len - 2 bytes to write.
static int gtp_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msg = { .addr = client->addr, .flags = 0, .len = len, .buf = buf };
	int retries, ret;

	for (retries = 5; retries; retries--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			return ret;
	}
	if (doze_status == DOZE_ENABLED)
		return ret;
	GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.",
		  (buf[0] << 8) | buf[1], len - 2, ret);
	gtp_reset_guitar(client, 10);
	return ret;
}

// Reads len bytes (at most 14) at addr twice, three times over, until both
// reads agree. Returns SUCCESS or FAIL.
static s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = { 0 };
	u8 confirm_buf[16] = { 0 };
	u8 retry;

	for (retry = 3; retry; retry--) {
		memset(buf, 0xaa, 16);
		buf[0] = addr >> 8;
		buf[1] = addr & 0xff;
		gtp_i2c_read(client, buf, len + 2);
		memset(confirm_buf, 0xab, 16);
		confirm_buf[0] = addr >> 8;
		confirm_buf[1] = addr & 0xff;
		gtp_i2c_read(client, confirm_buf, len + 2);
		if (!memcmp(buf, confirm_buf, len + 2)) {
			memcpy(rxbuf, confirm_buf + 2, len);
			return SUCCESS;
		}
	}
	GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
	return FAIL;
}

static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = { GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff };
	u8 retry = 0;
	s8 ret = -1;

	while (retry++ < 5) {
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0)
			return ret;
		GTP_ERROR("GTP i2c test failed time %d.", retry);
		msleep(10);
	}
	return ret;
}

// --- reset and configuration ------------------------------------------------

// INT stays an input for ms, as the panel expects after a reset.
static void gtp_int_sync(s32 ms)
{
	msleep(ms);
	gpio_direction_input(gtp_int_gpio);
}

// Resets the panel. INT is held during the reset to pick its address, high
// for 0x14 and low for 0x5D, then low for 50 ms before it becomes the
// interrupt again. With gesture wake on, the interrupt is off meanwhile.
static void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (gesture_sw_val)
		gtp_irq_disable(ts);
	GTP_INFO("Guitar reset");
	if (gtp_rst_gpio >= 0)
		gpio_direction_output(gtp_rst_gpio, 0);
	msleep(ms);
	msleep(10);
	if (gtp_rst_gpio >= 0)
		gpio_direction_output(gtp_rst_gpio, 0);
	msleep(10);
	if (gtp_int_gpio >= 0)
		gpio_direction_output(gtp_int_gpio, client->addr == 0x14);
	msleep(2);
	if (gtp_rst_gpio >= 0)
		gpio_direction_output(gtp_rst_gpio, 1);
	msleep(10);
	if (gtp_int_gpio >= 0)
		gpio_direction_output(gtp_int_gpio, 0);
	msleep(50);
	if (gtp_int_gpio >= 0)
		gpio_direction_input(gtp_int_gpio);
	gtp_int_sync(30);
	if (gesture_sw_val) {
		irq_set_irq_type(client->irq, irq_table[ts->int_trigger_type]);
		gtp_irq_enable(ts);
	}
	GTP_INFO("Guitar reset end");
}

// Sends the whole configuration buffer, five tries.
static int gtp_send_cfg(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int retry, ret = 0;

	if (ts->pnl_init_error) {
		GTP_INFO("Error occured in init_panel, no config sent");
		return 0;
	}
	GTP_INFO("Driver send config.");
	for (retry = 0; retry < 5; retry++) {
		ret = gtp_i2c_write(client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0)
			break;
	}
	return ret;
}

static u8 gtp_cfg_checksum(int len)
{
	u8 sum = 0;
	int i;

	for (i = GTP_ADDR_LENGTH; i < len; i++)
		sum += config[i];
	return ~sum + 1;
}

// Checks that the firmware runs and the sensor ID is valid, then sends the
// configuration with the resolution and trigger set. A configuration older
// than the one in the panel goes out with version 0, which the panel takes as
// "replace"; the buffer keeps the real version for the sends that follow.
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
	const u8 *send_cfg_buf[] = { cfg_info_group1 };
	u8 opr_buf[16] = { 0 };
	u8 sensor_id = 0;
	u8 grp_cfg_version = 0;
	bool old_cfg;
	int cfg_len, x_max = gtp_x_max, y_max = gtp_y_max;
	s32 ret;

	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_FW_STATE, opr_buf, 1);
	if (ret == SUCCESS && opr_buf[0] != 0xbe) {
		GTP_ERROR("Firmware error, no config sent!");
		return -1;
	}

	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
	if (ret != SUCCESS) {
		GTP_ERROR("Failed to get sensor_id, No config sent!");
		ts->pnl_init_error = 1;
		return -1;
	}
	if (sensor_id >= 6) {
		GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
		ts->pnl_init_error = 1;
		return -1;
	}
	GTP_INFO("Sensor_ID: %d", sensor_id);

	// One configuration for every sensor.
	cfg_len = sizeof(cfg_info_group1);
	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	sensor_id = 0;
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], cfg_len);
	GTP_INFO("Config group%d used,length: %d", sensor_id, cfg_len);
	if (cfg_len < GTP_CONFIG_MIN_LENGTH) {
		GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!",
			  sensor_id, cfg_len);
		ts->pnl_init_error = 1;
		return -1;
	}

	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, opr_buf, 1);
	if (ret != SUCCESS) {
		GTP_ERROR("Failed to get ic config version!No config sent!");
		return -1;
	}
	grp_cfg_version = config[GTP_ADDR_LENGTH];
	old_cfg = opr_buf[0] < 90 && grp_cfg_version < opr_buf[0];
	if (old_cfg)
		config[GTP_ADDR_LENGTH] = 0x00;

	if (gtp_x_y_coords_exchange) {
		x_max = gtp_y_max;
		y_max = gtp_x_max;
	}
	config[RESOLUTION_LOC] = x_max & 0xff;
	config[RESOLUTION_LOC + 1] = (x_max >> 8) & 0xff;
	config[RESOLUTION_LOC + 2] = y_max & 0xff;
	config[RESOLUTION_LOC + 3] = (y_max >> 8) & 0xff;
	config[TRIGGER_LOC] |= 0x01;
	config[cfg_len] = gtp_cfg_checksum(cfg_len);

	if (!ts->abs_x_max && !ts->abs_y_max) {
		ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
		ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
		ts->int_trigger_type = config[TRIGGER_LOC] & 0x03;
	}

	ret = gtp_send_cfg(ts->client);
	if (ret < 0)
		GTP_ERROR("Send config error.");

	if (old_cfg) {
		config[GTP_ADDR_LENGTH] = grp_cfg_version;
		config[cfg_len] = gtp_cfg_checksum(cfg_len);
	}

	GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max, ts->abs_y_max,
		 ts->int_trigger_type);
	msleep(10);
	return 0;
}

static s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
	u8 buf[8] = { GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff };
	s32 ret;

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}
	if (version)
		*version = (buf[7] << 8) | buf[6];
	if (buf[5] == 0x00)
		GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
	else
		GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
	return ret;
}

// --- gesture wake -------------------------------------------------------------

// Puts the panel in doze mode. Returns the last write's result, or 1 when it
// already dozes.
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
	u8 i2c_control_buf[3] = { GTP_REG_SLEEP >> 8, GTP_REG_SLEEP & 0xff, GTP_CMD_DOZE };
	u8 retry;
	s8 ret = -1;

	if (doze_status == DOZE_ENABLED)
		return SUCCESS;
	for (retry = 5; retry; retry--) {
		i2c_control_buf[0] = GTP_REG_DOZE_FLAG >> 8;
		i2c_control_buf[1] = GTP_REG_DOZE_FLAG & 0xff;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret < 0)
			continue;
		i2c_control_buf[0] = GTP_REG_SLEEP >> 8;
		i2c_control_buf[1] = GTP_REG_SLEEP & 0xff;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			doze_status = DOZE_ENABLED;
			GTP_INFO("Gesture mode enabled.");
			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send gesture cmd failed.");
	return ret;
}

static void gtp_report_power_key(struct goodix_ts_data *ts)
{
	doze_status = DOZE_WAKEUP;
	input_report_key(ts->input_dev, KEY_POWER, 1);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_POWER, 0);
	input_sync(ts->input_dev);
}

// In doze mode: a letter, a slide or a double tap wakes the screen with
// KEY_POWER; anything else puts the panel back in doze mode.
static void gtp_doze_work(struct goodix_ts_data *ts)
{
	u8 doze_buf[3] = { GTP_REG_DOZE_GESTURE >> 8, GTP_REG_DOZE_GESTURE & 0xff };
	u8 g;

	if (gtp_i2c_read(i2c_connect_client, doze_buf, 3) <= 0)
		return;
	g = doze_buf[2];
	if (strchr("abcdeghmoqsvwyz", g) && g) {
		GTP_INFO("Wakeup by gesture(%c), light up the screen!", g);
	} else if (g == '^') {
		GTP_INFO("Wakeup by gesture(^), light up the screen!");
	} else if (g == '>') {
		GTP_INFO("Wakeup by gesture(%c), light up the screen!", g);
	} else if (g == 0xaa || g == 0xab || g == 0xba || g == 0xbb) {
		u8 type = ((g & 0x0f) - 0x0a) + (((g >> 4) & 0x0f) - 0x0a) * 2;

		GTP_INFO("%s slide to light up the screen!", slide_direction[type]);
	} else if (g == 0xcc) {
		GTP_INFO("Double click to light up the screen!");
	} else {
		doze_buf[2] = 0x00;
		gtp_i2c_write(i2c_connect_client, doze_buf, 3);
		gtp_enter_doze(ts);
		return;
	}
	gtp_report_power_key(ts);
	doze_buf[2] = 0x00;
	gtp_i2c_write(i2c_connect_client, doze_buf, 3);
}

// --- touch report -------------------------------------------------------------

// Reads the frame and reports each contact in it, then clears the status for
// the next frame. A frame with more contacts than gtp_max_touch_number
// reports the first ones; when the contacts after the first cannot be read,
// the first one alone.
static void goodix_ts_work_func(struct work_struct *work)
{
	struct goodix_ts_data *ts = container_of(to_delayed_work(work), struct goodix_ts_data, work);
	u8 end_cmd[3] = { GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xff, 0 };
	u8 point_data[GTP_ADDR_LENGTH + 1 + 8 * GTP_MAX_TOUCH] = {
		GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xff,
	};
	static u16 pre_touch;
	int max_touch = clamp(gtp_max_touch_number, 0, GTP_MAX_TOUCH);
	u8 finger, touch_num;
	int ret, i;

	if (doze_status == DOZE_ENABLED) {
		gtp_doze_work(ts);
		goto enable_irq;
	}

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0) {
		GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
		goto enable_irq;
	}
	finger = point_data[GTP_ADDR_LENGTH];
	if (finger == 0x00)
		goto enable_irq;
	if (!(finger & GTP_STATUS_READY))
		goto end_frame;

	touch_num = min_t(int, finger & GTP_STATUS_TOUCHES, max_touch);
	if (touch_num > 1) {
		u8 buf[GTP_ADDR_LENGTH + 8 * (GTP_MAX_TOUCH - 1)] = {
			(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff,
		};

		if (gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)) < 0)
			touch_num = 1;
		else
			memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
	}

	for (i = 0; i < touch_num; i++) {
		const u8 *coor = &point_data[3 + 8 * i];
		int x = coor[1] | (coor[2] << 8);
		int y = coor[3] | (coor[4] << 8);
		int w = coor[5] | (coor[6] << 8);

		if (gtp_x_y_coords_exchange)
			swap(x, y);
		if (gtp_x_coords_flip)
			x = gtp_x_max - x;
		if (gtp_y_coords_flip)
			y = gtp_y_max - y;
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, coor[0]);
		input_mt_sync(ts->input_dev);
	}
	if (!touch_num && pre_touch)
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
	if (touch_num || pre_touch)
		input_sync(ts->input_dev);
	pre_touch = touch_num;

end_frame:
	ret = gtp_i2c_write(ts->client, end_cmd, 3);
	if (ret < 0)
		GTP_INFO("I2C write end_cmd error!");
enable_irq:
	if (ts->use_irq)
		gtp_irq_enable(ts);
}

// --- /proc/gt9xx_config -----------------------------------------------------

#define GTP_CONFIG_DUMP_SIZE	3072

static char *gtp_dump_cfg(char *ptr, const u8 *cfg)
{
	int i;

	for (i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
		ptr += sprintf(ptr, "0x%02X ", cfg[i]);
		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}
	return ptr;
}

static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size,
				       loff_t *ppos)
{
	u8 temp_data[GTP_CONFIG_MAX_LENGTH + 2] = { GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff };
	char *out, *ptr;
	size_t len;
	ssize_t ret;

	if (*ppos)
		return 0;
	out = kmalloc(GTP_CONFIG_DUMP_SIZE, GFP_KERNEL);
	if (!out)
		return -ENOMEM;
	ptr = out;
	ptr += sprintf(ptr, "==== GT9XX config init value====\n");
	ptr = gtp_dump_cfg(ptr, &config[2]);
	ptr += sprintf(ptr, "\n");
	ptr += sprintf(ptr, "==== GT9XX config real value====\n");
	gtp_i2c_read(i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
	ptr = gtp_dump_cfg(ptr, &temp_data[2]);

	len = min_t(size_t, ptr - out, size);
	if (copy_to_user(page, out, len)) {
		ret = -EFAULT;
	} else {
		*ppos += len;
		ret = len;
	}
	kfree(out);
	return ret;
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer,
					size_t count, loff_t *off)
{
	if (count > GTP_CONFIG_MAX_LENGTH) {
		GTP_ERROR("size not match [%d:%zu]\n", GTP_CONFIG_MAX_LENGTH, count);
		return -EFAULT;
	}
	if (copy_from_user(&config[2], buffer, count)) {
		GTP_ERROR("copy from user fail\n");
		return -EFAULT;
	}
	if (gtp_send_cfg(i2c_connect_client) < 0)
		GTP_ERROR("send config failed.");
	return count;
}

static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = gt91xx_config_read_proc,
	.write = gt91xx_config_write_proc,
};

// --- gesture_sw -------------------------------------------------------------

static ssize_t get_gesture_sw(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gesture_sw_val);
}

static ssize_t set_gesture_sw(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	gesture_sw_val = !strncmp(buf, "on", 2);
	return count;
}

static DEVICE_ATTR(gesture_sw, 0644, get_gesture_sw, set_gesture_sw);

// --- power ------------------------------------------------------------------

static bool gtp_has_regulator(void)
{
	return strcmp(gtp_regulator_name, "-1") && strlen(gtp_regulator_name);
}

// Unblank: powers the panel, resets it, configures it and takes the interrupt
// back.
static void goodix_ts_reset_work_func(struct work_struct *work)
{
	struct goodix_ts_data *ts;
	struct regulator *reg;

	if (!i2c_connect_client)
		return;
	ts = i2c_get_clientdata(i2c_connect_client);
	if (gtp_has_regulator()) {
		reg = regulator_get(NULL, gtp_regulator_name);
		if (!IS_ERR_OR_NULL(reg)) {
			if (!regulator_is_enabled(reg))
				regulator_enable(reg);
			regulator_put(reg);
		}
	}
	gtp_reset_guitar(ts->client, 15);
	gtp_init_panel(ts);
	if (ts->use_irq && !ts->irq_requested) {
		gtp_request_irq(ts);
		ts->irq_is_disable = 0;
	}
}

static void goodix_ts_suspend(struct goodix_ts_data *ts)
{
	struct regulator *reg;

	if (ts->gtp_is_suspend)
		return;
	GTP_INFO("System suspend.");
	if (gesture_sw_val) {
		gtp_enter_doze(ts);
		goto done;
	}
	cancel_delayed_work_sync(&ts->reset_work);
	if (ts->use_irq) {
		msleep(10);
		gtp_irq_disable(ts);
		if (ts->irq_requested) {
			free_irq(ts->client->irq, ts);
			ts->irq_requested = false;
		}
		if (gtp_int_gpio >= 0)
			gpio_direction_output(gtp_int_gpio, 0);
	} else {
		hrtimer_cancel(&ts->timer);
	}
	if (gtp_has_regulator()) {
		reg = regulator_get(NULL, gtp_regulator_name);
		if (!IS_ERR_OR_NULL(reg)) {
			if (regulator_is_enabled(reg))
				regulator_disable(reg);
			regulator_put(reg);
		}
	}
	if (gtp_rst_gpio >= 0)
		gpio_direction_output(gtp_rst_gpio, 0);
done:
	ts->gtp_is_suspend = 1;
	msleep(58);
}

static void goodix_ts_resume(struct goodix_ts_data *ts)
{
	u8 retry;
	s8 ret = -1;

	if (!ts->gtp_is_suspend)
		return;
	GTP_INFO("System resume.");
	if (!gesture_sw_val) {
		unsigned long delay = 0;

		if (delayed_work_pending(&ts->reset_work)) {
			cancel_delayed_work_sync(&ts->reset_work);
			delay = 5;
		}
		queue_delayed_work(goodix_wq, &ts->reset_work, delay);
		goto done;
	}

	for (retry = 10; retry; retry--) {
		GTP_INFO("%s", doze_status == DOZE_WAKEUP ? "Gesture wakeup." : "Powerkey wakeup.");
		doze_status = DOZE_DISABLED;
		gtp_reset_guitar(ts->client, 10);
		GTP_INFO("Gtp i2c test");
		ret = gtp_i2c_test(ts->client);
		if (ret > 0) {
			GTP_INFO("GTP wakeup sleep.");
			goto out_doze;
		}
		gtp_reset_guitar(ts->client, 20);
	}
	GTP_ERROR("GTP wakeup sleep failed.");
	if (ret)
		GTP_ERROR("GTP later resume failed.");
out_doze:
	doze_status = DOZE_DISABLED;
done:
	ts->gtp_is_suspend = 0;
}

static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct goodix_ts_data *ts = container_of(noti, struct goodix_ts_data, notifier);
	struct fb_event *ev_data = data;
	int *blank;

	if (!ev_data || !ev_data->data || event != FB_EVENT_BLANK || !ts)
		return 0;
	blank = ev_data->data;
	if (*blank == FB_BLANK_UNBLANK)
		goodix_ts_resume(ts);
	else if (*blank == FB_BLANK_POWERDOWN)
		goodix_ts_suspend(ts);
	return 0;
}

static int gtp_pm_suspend(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);

	if (ts) {
		if (device_may_wakeup(dev))
			irq_set_irq_wake(to_i2c_client(dev)->irq, 1);
		goodix_ts_suspend(ts);
	}
	return 0;
}

static int gtp_pm_resume(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);

	if (ts) {
		if (device_may_wakeup(dev))
			irq_set_irq_wake(to_i2c_client(dev)->irq, 0);
		goodix_ts_resume(ts);
	}
	return 0;
}

static const struct dev_pm_ops gtp_pm_ops = {
	.suspend = gtp_pm_suspend,
	.resume = gtp_pm_resume,
};

// --- probe --------------------------------------------------------------------

// The power, interrupt and reset pins, then a reset. Gives back what it took
// on failure.
static void gtp_free_io_port(struct goodix_ts_data *ts)
{
	if (ts->rst_gpio_owned)
		gpio_free(gtp_rst_gpio);
	if (ts->power_gpio_owned)
		gpio_free(gtp_power_on_gpio);
	ts->rst_gpio_owned = false;
	ts->power_gpio_owned = false;
}

static int gtp_request_io_port(struct goodix_ts_data *ts)
{
	int ret;

	if (gtp_power_on_gpio >= 0) {
		ret = gpio_request(gtp_power_on_gpio, "GTP POWER_ON");
		if (ret < 0)
			GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", gtp_power_on_gpio, ret);
		else
			ts->power_gpio_owned = true;
	}
	ret = gpio_request(gtp_int_gpio, "GTP INT IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", gtp_int_gpio, ret);
		ret = -ENODEV;
		goto err;
	}
	gpio_direction_input(gtp_int_gpio);
	ts->client->irq = gpio_to_irq(gtp_int_gpio);
	if (gtp_rst_gpio >= 0) {
		ret = gpio_request(gtp_rst_gpio, "GTP RST PORT");
		if (ret < 0) {
			GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", gtp_rst_gpio, ret);
			gpio_free(gtp_int_gpio);
			ret = -ENODEV;
			goto err;
		}
		ts->rst_gpio_owned = true;
		gpio_direction_input(gtp_rst_gpio);
	}
	gtp_reset_guitar(ts->client, 20);
	return 0;

err:
	gtp_free_io_port(ts);
	return ret;
}

static int gtp_request_input_dev(struct goodix_ts_data *ts)
{
	struct input_dev *dev;
	int ret;

	dev = input_allocate_device();
	if (!dev) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}
	ts->input_dev = dev;
	dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	__set_bit(INPUT_PROP_DIRECT, dev->propbit);
	input_set_capability(dev, EV_KEY, KEY_POWER);
	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, gtp_x_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, gtp_y_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
	dev->name = "goodix-ts";
	dev->phys = "input/ts";
	dev->id.bustype = BUS_I2C;
	dev->id.vendor = 0xdead;
	dev->id.product = 0xbeef;
	dev->id.version = 10427;
	ret = input_register_device(dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", dev->name);
		input_free_device(dev);
		ts->input_dev = NULL;
	}
	return ret;
}

static void gtp_regulator_off(void)
{
	struct regulator *reg;

	if (!gtp_has_regulator())
		return;
	reg = regulator_get(NULL, gtp_regulator_name);
	if (!IS_ERR_OR_NULL(reg)) {
		if (regulator_is_enabled(reg))
			regulator_disable(reg);
		regulator_put(reg);
	}
}

static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct goodix_ts_data *ts;
	struct regulator *reg;
	u16 version_info;
	int ret;

	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);
	i2c_connect_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}
	spin_lock_init(&ts->irq_lock);
	INIT_DELAYED_WORK(&ts->work, goodix_ts_work_func);
	INIT_DELAYED_WORK(&ts->reset_work, goodix_ts_reset_work_func);

	if (gtp_has_regulator()) {
		reg = regulator_get(NULL, gtp_regulator_name);
		if (!IS_ERR_OR_NULL(reg)) {
			regulator_enable(reg);
			regulator_put(reg);
		}
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = gtp_request_io_port(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		goto err_regulator;
	}

	if (gtp_i2c_test(client) < 0)
		GTP_ERROR("I2C communication ERROR!");
	if (gtp_read_version(client, &version_info) < 0)
		GTP_ERROR("Read version failed.");
	if (gtp_init_panel(ts) < 0) {
		GTP_ERROR("GTP init panel failed.");
		ts->abs_x_max = gtp_x_max;
		ts->abs_y_max = gtp_y_max;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}

	gt91xx_config_proc = proc_create("gt9xx_config", 0666, NULL, &config_proc_ops);
	if (!gt91xx_config_proc)
		GTP_ERROR("create_proc_entry %s failed\n", "gt9xx_config");
	else
		GTP_INFO("create proc entry %s success", "gt9xx_config");

	ret = gtp_request_input_dev(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
		goto err_proc;
	}

	if (gtp_request_irq(ts) < 0)
		GTP_INFO("GTP works in polling mode.");
	else
		GTP_INFO("GTP works in interrupt mode.");
	if (ts->use_irq)
		gtp_irq_enable(ts);

	ts->notifier.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&ts->notifier);
	device_init_wakeup(&client->dev, 1);
	device_create_file(&client->dev, &dev_attr_gesture_sw);
	return 0;

err_proc:
	proc_remove(gt91xx_config_proc);
	gt91xx_config_proc = NULL;
	gpio_free(gtp_int_gpio);
	gtp_free_io_port(ts);
err_regulator:
	gtp_regulator_off();
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	fb_unregister_client(&ts->notifier);
	if (ts->use_irq) {
		gpio_direction_input(gtp_int_gpio);
		gpio_free(gtp_int_gpio);
		if (ts->irq_requested) {
			free_irq(client->irq, ts);
			ts->irq_requested = false;
		}
	} else {
		hrtimer_cancel(&ts->timer);
	}
	cancel_delayed_work_sync(&ts->work);
	cancel_delayed_work_sync(&ts->reset_work);
	gtp_free_io_port(ts);
	GTP_INFO("GTP driver removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	device_remove_file(&client->dev, &dev_attr_gesture_sw);
	return 0;
}

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = goodix_ts_id,
	.driver = {
		.name = GTP_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &gtp_pm_ops,
	},
};

static struct i2c_board_info touch_gt9xx_ts_info = {
	I2C_BOARD_INFO(GTP_I2C_NAME, GTP_I2C_ADDR),
};

static int __init goodix_ts_init(void)
{
	int ret;

	GTP_INFO("GTP driver installing....");
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
	ret = i2c_add_driver(&goodix_ts_driver);
	if (ret)
		goto err_wq;
	i2c_connect_client = i2c_register_device(&touch_gt9xx_ts_info, i2c_bus_num);
	if (!i2c_connect_client) {
		GTP_ERROR("failed to register i2c device\n");
		i2c_del_driver(&goodix_ts_driver);
		ret = -EINVAL;
		goto err_wq;
	}
	return 0;

err_wq:
	destroy_workqueue(goodix_wq);
	goodix_wq = NULL;
	return ret;
}

static void __exit goodix_ts_exit(void)
{
	GTP_INFO("GTP driver exited.");
	proc_remove(gt91xx_config_proc);
	i2c_unregister_device(i2c_connect_client);
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
