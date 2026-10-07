// SPDX-License-Identifier: GPL-2.0
//
// cw2015 -- the CellWise CW2015 fuel gauge, as the power supply "battery":
// capacity, voltage, status, time to empty.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock cw2015.sh passes them):
//
//   i2c_bus_num   I2C bus of the gauge (address 0x62)
//   fuel_gauge    the 64-byte battery profile, as "18,0A,..." (191
//                 characters); any other length leaves the gauge out
//   int_gpio      alert pin; not read
//
// The profile goes into the gauge when the gauge does not hold it. A worker
// reads the gauge every second.
//
// Register 0x4f, the last profile byte, is not compared: it keeps the charge
// level at which the battery last stopped charging (bits 6..0) and whether
// that was short of 100 % (bit 7). While that flag is set, the capacity
// reported is scaled so that this level reads as 100 %.
//
// config_info (write-only, on the I2C device): a new profile as 64 "%02x "
// fields, loaded by the next run of the worker.
//
// Loaded after utils.ko, which provides i2c_register_device() and the pin
// parameter.

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);

#define REG_VCELL		0x02	// 2 bytes, 312/1024 mV per step
#define REG_SOC			0x04	// percent, then 1/256 percent
#define REG_RRT_ALERT		0x06	// 2 bytes: time to empty in minutes, 13 bits
#define REG_CONFIG		0x08
#define REG_MODE		0x0a
#define REG_BATINFO		0x10	// 64 bytes of profile
#define REG_HIGH_SOC		0x4f	// the last profile byte

#define CONFIG_UPDATE_FLG	0x02
#define CONFIG_ATHD_MASK	0xf8
#define MODE_SLEEP		0xc0
#define MODE_NORMAL		0x00
#define MODE_RESTART		0x0f

#define SIZE_BATINFO		64

#define HIGH_SOC_SHORT		0x80	// charging stopped short of 100 %
#define HIGH_SOC_MASK		0x7f

struct cw_bat_platform_data {
	u8 *cw_bat_config_info;
};

struct cw_battery {
	struct i2c_client *client;
	struct workqueue_struct *battery_workqueue;
	struct delayed_work battery_delay_work;
	struct cw_bat_platform_data *plat_data;
	struct power_supply *rk_bat;
	int charger_mode;	// 1 while a charger gives current
	int capacity;		// percent
	int voltage;		// mV
	int status;		// POWER_SUPPLY_STATUS_*
	int time_to_empty;	// minutes
	int change;		// 1: tell user space
};

static int i2c_bus_num = -1;
static int int_gpio = -1;
static char fuel_gauge[200];

module_param_string(fuel_gauge, fuel_gauge, sizeof(fuel_gauge), 0644);
module_param(i2c_bus_num, int, 0644);
module_param_cb(int_gpio, &param_gpio_ops, &int_gpio, 0644);

// Replaced by the fuel_gauge parameter at load.
static u8 cw_bat_config_info[SIZE_BATINFO] = {
	0x18, 0x0a, 0x66, 0x6d, 0x6c, 0x6b, 0x69, 0x67, 0x60, 0x6d, 0x63, 0x57, 0x5e, 0x5a, 0x48, 0x41,
	0x39, 0x2f, 0x27, 0x20, 0x25, 0x2f, 0x41, 0x4f, 0x26, 0x46, 0x0b, 0x85, 0x1f, 0x3e, 0x4f, 0x64,
	0x6f, 0x6e, 0x6f, 0x71, 0x3e, 0x19, 0x7d, 0x21, 0x09, 0x42, 0x17, 0x46, 0x7d, 0x97, 0xa6, 0x23,
	0x4a, 0x82, 0x96, 0xa0, 0x80, 0x9c, 0xc5, 0xcb, 0x2f, 0x00, 0x64, 0xa5, 0xb5, 0x08, 0x40, 0x79,
};

static struct cw_bat_platform_data cw_bat_platdata;

static struct i2c_board_info cw2015_board_info = {
	I2C_BOARD_INFO("cw201x", 0x62),
};

static struct i2c_client *cw2015_dev;
static u8 *config_info;
static bool user_config_update;

static u8 last_charger_high_soc = 96;
static u8 mark_no_charger_full;
static int last_real_soc;
static int suspend_resume_mark;
static int charging_time_loop;
static int charging_zero_loop;
static int reset_loop;

static struct power_supply *chrg_ac_psy;
static struct power_supply *chrg_usb_psy;

static int cw_write(struct i2c_client *client, u8 reg, u8 *buf)
{
	return i2c_smbus_write_i2c_block_data(client, reg, 1, buf);
}

static int cw_read(struct i2c_client *client, u8 reg, u8 *buf)
{
	return i2c_smbus_read_i2c_block_data(client, reg, 1, buf);
}

// Writes the profile, then restarts the gauge on it.
static int cw_update_config_info(struct cw_battery *cw_bat)
{
	u8 reg_val, reset_val;
	int ret, i;

	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0)
		return ret;
	reset_val = reg_val;
	if ((reg_val & MODE_SLEEP) == MODE_SLEEP)
		return -1;

	for (i = 0; i < SIZE_BATINFO; i++) {
		ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info[i]);
		if (ret < 0)
			return ret;
	}

	// The flag that the gauge holds a profile, alert threshold 0.
	reg_val = (reg_val & 0x07) | CONFIG_UPDATE_FLG;
	ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	if (!(reg_val & CONFIG_UPDATE_FLG))
		printk("Error: The new config set fail\n");
	if (reg_val & CONFIG_ATHD_MASK)
		printk("Error: The new ATHD set fail\n");

	reset_val &= ~MODE_RESTART;
	reg_val = reset_val | MODE_RESTART;
	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0)
		return ret;
	msleep(10);
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	msleep(100);
	return 0;
}

// Wakes the gauge, loads the profile if it differs, and waits up to 3.6 s
// for a first capacity.
static int cw_init(struct cw_battery *cw_bat)
{
	u8 reg_val = MODE_NORMAL;
	int ret, i;

	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0)
		return ret;

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	if (reg_val & CONFIG_ATHD_MASK) {
		reg_val &= 0x07;
		ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
		if (ret < 0)
			return ret;
	}
	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	if (!(reg_val & CONFIG_UPDATE_FLG)) {
		ret = cw_update_config_info(cw_bat);
		if (ret < 0) {
			printk("%s : update config fail\n", __func__);
			return ret;
		}
	} else {
		for (i = 0; i < SIZE_BATINFO - 1; i++) {
			ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
			if (ret < 0)
				return ret;
			if (config_info[i] != reg_val)
				break;
		}
		if (i < SIZE_BATINFO - 1) {
			ret = cw_update_config_info(cw_bat);
			if (ret < 0)
				return ret;
		}
	}

	msleep(10);
	for (i = 30; ; ) {
		ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
		if (ret < 0)
			return ret;
		if (reg_val <= 100) {
			last_real_soc = reg_val;
			return 0;
		}
		msleep(120);
		if (--i == 0)
			break;
	}

	reg_val = MODE_SLEEP;
	cw_write(cw_bat->client, REG_MODE, &reg_val);
	return -1;
}

// Power-on reset: sleep, wake, set up again.
static int cw_por(struct cw_battery *cw_bat)
{
	u8 reset_val = MODE_SLEEP;
	int ret;

	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	msleep(10);
	reset_val = MODE_NORMAL;
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	return cw_init(cw_bat);
}

// The capacity to report, or a negative error.
static int cw_get_capacity(struct cw_battery *cw_bat)
{
	u8 reg_val[2], high_soc, new_high;
	int cw_capacity, real_soc, remainder, full, ui_soc, ui_100, ret;

	ret = i2c_smbus_read_i2c_block_data(cw_bat->client, REG_SOC, 2, reg_val);
	if (ret < 0)
		return ret;
	ret = cw_read(cw_bat->client, REG_HIGH_SOC, &high_soc);
	if (ret < 0)
		return ret;

	real_soc = reg_val[0];
	remainder = reg_val[1];

	// A capacity over 100 % for 40 s in a row resets the gauge.
	if (real_soc > 100) {
		if (++reset_loop > 40) {
			cw_por(cw_bat);
			reset_loop = 0;
		}
		return cw_bat->capacity;
	}
	reset_loop = 0;

	// So does 0 % for 30 minutes on a charger.
	if (cw_bat->charger_mode > 0 && real_soc == 0) {
		if (++charging_zero_loop > 1800) {
			cw_por(cw_bat);
			charging_zero_loop = 0;
		}
	} else if (charging_zero_loop) {
		charging_zero_loop = 0;
	}

	full = mark_no_charger_full == 1 ? (high_soc & HIGH_SOC_MASK) : 100;
	ui_100 = (real_soc << 8) + remainder;
	ui_soc = ui_100 * 100 / (full << 8);
	if (ui_soc < 100) {
		// Away from the middle of a percent, a step of one is ignored.
		int frac = ui_100 * 10000 / (full << 8) % 100;

		if (frac < 30 || frac > 70) {
			if (ui_soc >= cw_bat->capacity - 1 && ui_soc <= cw_bat->capacity + 1)
				ui_soc = cw_bat->capacity;
		}
	} else {
		ui_soc = 100;
	}

	// On a charger at 96 % or more: a level that holds for 7 minutes is
	// where charging stops, and is kept in REG_HIGH_SOC.
	if (cw_bat->charger_mode > 0 && real_soc >= 96) {
		if (last_real_soc < real_soc) {
			charging_time_loop = 0;
		} else if (++charging_time_loop > 420) {
			cw_capacity = max(real_soc, last_real_soc);
			last_charger_high_soc = cw_capacity;
			mark_no_charger_full = (u8)cw_capacity != 100;
			new_high = mark_no_charger_full << 7 | cw_capacity;
			charging_time_loop = 0;
			ret = cw_write(cw_bat->client, REG_HIGH_SOC, &new_high);
			if (ret < 0)
				return ret;
		}
		if (last_real_soc == 100) {
			new_high = 100;
			mark_no_charger_full = 0;
			ret = cw_write(cw_bat->client, REG_HIGH_SOC, &new_high);
			if (ret < 0)
				return ret;
		}
	} else {
		charging_time_loop = 0;
	}

	last_real_soc = real_soc;
	if (suspend_resume_mark == 1)
		suspend_resume_mark = 0;
	return ui_soc;
}

// The median of three readings.
static int cw_get_vol(struct cw_battery *cw_bat)
{
	u8 reg_val[2];
	u16 value16, value16_1, value16_2, value16_3;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(cw_bat->client, REG_VCELL, 2, reg_val);
	if (ret < 0)
		return ret;
	value16 = (reg_val[0] << 8) + reg_val[1];

	ret = i2c_smbus_read_i2c_block_data(cw_bat->client, REG_VCELL, 2, reg_val);
	if (ret < 0)
		return ret;
	value16_1 = (reg_val[0] << 8) + reg_val[1];

	ret = i2c_smbus_read_i2c_block_data(cw_bat->client, REG_VCELL, 2, reg_val);
	if (ret < 0)
		return ret;
	value16_2 = (reg_val[0] << 8) + reg_val[1];

	if (value16 > value16_1)
		swap(value16, value16_1);
	if (value16_1 > value16_2)
		value16_1 = value16_2;
	value16_3 = value16 > value16_1 ? value16 : value16_1;

	return value16_3 * 312 / 1024;
}

static int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
	u8 reg_val;
	u16 value16;
	int ret;

	ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
	if (ret < 0)
		return ret;
	value16 = reg_val;
	ret = cw_read(cw_bat->client, REG_RRT_ALERT + 1, &reg_val);
	if (ret < 0)
		return ret;
	value16 = ((value16 << 8) + reg_val) & 0x1fff;
	return value16;
}

static void cw_update_vol(struct cw_battery *cw_bat)
{
	int voltage = cw_get_vol(cw_bat);

	if (voltage < 0 || cw_bat->voltage == voltage)
		return;
	if (abs(cw_bat->voltage - voltage) > 100)
		cw_bat->change = 1;
	cw_bat->voltage = voltage;
}

static void cw_update_time_to_empty(struct cw_battery *cw_bat)
{
	int rrt = cw_get_time_to_empty(cw_bat);

	if (rrt >= 0 && cw_bat->time_to_empty != rrt) {
		cw_bat->time_to_empty = rrt;
		cw_bat->change = 1;
	}
}

static int check_chrg_usb_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->desc->type == POWER_SUPPLY_TYPE_USB) {
		chrg_usb_psy = psy;
		return 1;
	}
	return 0;
}

static int check_chrg_ac_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->desc->type == POWER_SUPPLY_TYPE_MAINS) {
		chrg_ac_psy = psy;
		return 1;
	}
	return 0;
}

// Charging while a USB or mains supply is online and the USB one allows
// current.
static void cw_update_charge_status(struct cw_battery *cw_bat)
{
	union power_supply_propval val;
	int usb_online = 0, ac_online = 0, usb_current = 500;
	int mode;

	if (!chrg_usb_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_usb_psy);
	if (!chrg_ac_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_ac_psy);

	if (chrg_usb_psy) {
		if (!chrg_usb_psy->desc->get_property(chrg_usb_psy, POWER_SUPPLY_PROP_ONLINE, &val))
			usb_online = val.intval;
		if (!chrg_usb_psy->desc->get_property(chrg_usb_psy,
						      POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val))
			usb_current = val.intval;
	}
	if (chrg_ac_psy) {
		if (!chrg_ac_psy->desc->get_property(chrg_ac_psy, POWER_SUPPLY_PROP_ONLINE, &val))
			ac_online = val.intval;
	}

	mode = (usb_online | ac_online) ? usb_current > 0 : 0;
	if (cw_bat->charger_mode != mode) {
		cw_bat->charger_mode = mode;
		cw_bat->change = 1;
	}
}

static void cw_update_status(struct cw_battery *cw_bat)
{
	int status;

	if (cw_bat->charger_mode > 0)
		status = cw_bat->capacity < 100 ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_FULL;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (cw_bat->status != status) {
		cw_bat->status = status;
		cw_bat->change = 1;
	}
}

static void cw_bat_work(struct work_struct *work)
{
	struct cw_battery *cw_bat = container_of(work, struct cw_battery, battery_delay_work.work);
	u8 reg_val;
	int ret, capacity, i;

	if (user_config_update) {
		cw_update_config_info(cw_bat);
		user_config_update = false;
	}

	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0) {
		// No gauge: report a full battery.
		cw_bat->capacity = 100;
		cw_bat->voltage = 8400;
		cw_bat->change = 1;
	} else {
		if ((reg_val & MODE_SLEEP) == MODE_SLEEP) {
			for (i = 0; i < 5; i++)
				if (!cw_por(cw_bat))
					break;
		}

		capacity = cw_get_capacity(cw_bat);
		if (capacity >= 0 && capacity <= 100 && cw_bat->capacity != capacity) {
			cw_bat->capacity = capacity;
			cw_bat->change = 1;
		}

		cw_update_vol(cw_bat);
		cw_update_charge_status(cw_bat);
		cw_update_status(cw_bat);
		cw_update_time_to_empty(cw_bat);
	}

	if (suspend_resume_mark == 1)
		suspend_resume_mark = 0;

	if (cw_bat->change == 1) {
		power_supply_changed(cw_bat->rk_bat);
		cw_bat->change = 0;
	}

	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(1000));
}

static int cw_battery_get_property(struct power_supply *psy, enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct cw_battery *cw_bat = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = cw_bat->capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = cw_bat->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = cw_bat->voltage > 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = cw_bat->voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = cw_bat->time_to_empty;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property cw_battery_properties[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static ssize_t write_config_info(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int val;
	int i;

	for (i = 0; i < SIZE_BATINFO; i++) {
		if (sscanf(buf, "%02x ", &val) == 1) {
			buf += 3;
			config_info[i] = val;
		}
		user_config_update = true;
	}
	return count;
}

static DEVICE_ATTR(config_info, 0200, NULL, write_config_info);

static struct attribute *sysfs_attributes[] = {
	&dev_attr_config_info.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attributes,
};

static int cw2015_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct power_supply_config psy_cfg = { 0 };
	struct power_supply_desc *psy_desc;
	struct cw_battery *cw_bat;
	u8 reg_val[2];
	int ret, loop;

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat)
		return -ENOMEM;
	i2c_set_clientdata(client, cw_bat);

	cw_bat->plat_data = client->dev.platform_data;
	if (!cw_bat->plat_data || !cw_bat->plat_data->cw_bat_config_info) {
		printk("cw_bat config info error\n");
		return -EINVAL;
	}
	config_info = cw_bat->plat_data->cw_bat_config_info;

	cw_bat->client = client;
	cw_bat->capacity = 1;
	cw_bat->voltage = 0;
	cw_bat->status = 0;
	cw_bat->charger_mode = 0;
	cw_bat->change = 0;

	ret = cw_init(cw_bat);
	for (loop = 1; ret && loop < 4; loop++) {
		msleep(200);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		printk("%s : cw2015 init fail!\n", __func__);
		return ret;
	}

	// A fresh profile leaves its own last byte in REG_HIGH_SOC: start from
	// 96 % and no flag.
	ret = cw_read(client, REG_HIGH_SOC, reg_val);
	if (ret < 0)
		goto err_data;
	if (reg_val[0] == config_info[SIZE_BATINFO - 1]) {
		reg_val[0] = 96;
		last_charger_high_soc = 96;
		mark_no_charger_full = 0;
		ret = cw_write(client, REG_HIGH_SOC, reg_val);
		if (ret < 0)
			goto err_data;
	} else {
		last_charger_high_soc = reg_val[0] & HIGH_SOC_MASK;
		mark_no_charger_full = reg_val[0] >> 7;
	}

	ret = i2c_smbus_read_i2c_block_data(client, REG_SOC, 2, reg_val);
	if (ret < 0) {
		printk("%s : cw2015 init data fail!\n", __func__);
		return ret;
	}
	last_real_soc = reg_val[0];
	if (mark_no_charger_full == 1)
		cw_bat->capacity = ((reg_val[0] << 8) + reg_val[1]) * 100 / (last_charger_high_soc << 8);
	else
		cw_bat->capacity = reg_val[0];

	cw_update_vol(cw_bat);
	cw_update_charge_status(cw_bat);
	cw_update_status(cw_bat);
	cw_update_time_to_empty(cw_bat);

	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;
	psy_desc->name = "battery";
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = cw_battery_properties;
	psy_desc->num_properties = ARRAY_SIZE(cw_battery_properties);
	psy_desc->get_property = cw_battery_get_property;

	psy_cfg.drv_data = cw_bat;
	cw_bat->rk_bat = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if (IS_ERR(cw_bat->rk_bat)) {
		ret = PTR_ERR(cw_bat->rk_bat);
		printk(KERN_ERR "failed to register battery: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&client->dev.kobj, &sysfs_group);
	if (ret) {
		printk("sysfs_create_group err\n");
		goto err_psy;
	}

	cw_bat->battery_workqueue = create_singlethread_workqueue("cwfg_gauge");
	if (!cw_bat->battery_workqueue) {
		ret = -ENOMEM;
		goto err_group;
	}
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(50));
	return 0;

err_group:
	sysfs_remove_group(&client->dev.kobj, &sysfs_group);
err_psy:
	power_supply_unregister(cw_bat->rk_bat);
	return ret;
err_data:
	printk("%s : cw2015 init data fail!\n", __func__);
	return -1;
}

static int cw2015_remove(struct i2c_client *client)
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&cw_bat->battery_delay_work);
	destroy_workqueue(cw_bat->battery_workqueue);
	sysfs_remove_group(&client->dev.kobj, &sysfs_group);
	power_supply_unregister(cw_bat->rk_bat);
	return 0;
}

static int cw_bat_suspend(struct device *dev)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	cancel_delayed_work(&cw_bat->battery_delay_work);
	return 0;
}

static int cw_bat_resume(struct device *dev)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	suspend_resume_mark = 1;
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, 1);
	return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
	.suspend = cw_bat_suspend,
	.resume = cw_bat_resume,
};

static const struct i2c_device_id cw2015_id_table[] = {
	{ "cw201x", 0 },
	{ }
};

static struct i2c_driver cw2015_driver = {
	.driver = {
		.name = "cw201x",
		.owner = THIS_MODULE,
		.pm = &cw_bat_pm_ops,
	},
	.probe = cw2015_probe,
	.remove = cw2015_remove,
	.id_table = cw2015_id_table,
};

static int hex2int(char c)
{
	if (c >= 'A' && c <= 'Z')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'z')
		return c - 'a' + 10;
	if (c >= '0' && c <= '9')
		return c - '0';
	return 0;
}

// The profile from fuel_gauge, and the I2C device that carries it.
static int cw2015_dev_init(void)
{
	const char *s = fuel_gauge;
	int i;

	if (strlen(fuel_gauge) != SIZE_BATINFO * 3 - 1) {
		printk("cw2015 battery param str length[%d] is error %s\n", (int)strlen(fuel_gauge), fuel_gauge);
		return -1;
	}
	for (i = 0; i < SIZE_BATINFO; i++, s += 3)
		cw_bat_config_info[i] = hex2int(s[0]) * 16 + hex2int(s[1]);

	cw_bat_platdata.cw_bat_config_info = cw_bat_config_info;
	cw2015_board_info.platform_data = &cw_bat_platdata;
	cw2015_dev = i2c_register_device(&cw2015_board_info, i2c_bus_num);
	return cw2015_dev ? 0 : -EINVAL;
}

static int __init cw215_init(void)
{
	int ret;

	ret = cw2015_dev_init();
	if (ret)
		return ret;
	ret = i2c_add_driver(&cw2015_driver);
	if (ret)
		i2c_unregister_device(cw2015_dev);
	return ret;
}
module_init(cw215_init);

static void __exit cw215_exit(void)
{
	i2c_unregister_device(cw2015_dev);
	i2c_del_driver(&cw2015_driver);
}
module_exit(cw215_exit);

MODULE_DESCRIPTION("CW2015 fuel gauge of the HiBy R1 and R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
