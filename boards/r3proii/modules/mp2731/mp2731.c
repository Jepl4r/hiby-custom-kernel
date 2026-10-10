// SPDX-License-Identifier: GPL-2.0
//
// mp2731 -- the Monolithic Power MP2731 battery charger of the R3 Pro II, as
// the power supply "mp2731-charger" (type USB).
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock mp2731.sh passes them):
//
//   i2c_bus_num          I2C bus of the charger (address 0x4b)
//   int_gpio             interrupt pin of the charger (PC01)
//   otg_enable_gpio      output pin driven high at probe (PF10)
//   charge_enable_gpio   output pin driven low at probe; the stock script
//                        does not set it
//   input_limit          input current limit, uA
//   charge_limit         fast charge current, uA
//   min_sys_voltage      minimum system voltage, uV
//   min_input_voltage    input voltage regulation, uV
//   vbat_target          battery regulation voltage, uV
//   termination_current  charge termination current, uA
//   precharge_current    precharge current, uA
//   otg_max_current      OTG output current limit, uA
//   charge_limit_percent battery level at which charging stops, 100 for
//                        none; not in the stock script, and the same as
//                        the charge_limit_percent file below
//
// The structure is the mainline bq25890 driver: a regmap with one field per
// register bit group, the chip state read on every interrupt, and a power
// supply whose properties come from that state and from the ADC registers.
// On top of it sits a worker that, every 100 ms to 1 s, steps the input and
// charge currents towards what the source can give and writes them, together
// with the charge enable bit, from the properties user space sets:
//
//   step_charging_enabled       0 stops charging and leaves the input path
//                               on (CHG_CONFIG = 0); 1 charges
//   charge_control_limit        fast charge current, uA
//   input_current_limited       input current limit, uA
//   input_suspend               1 caps the input current at 100 mA
//   sdp_current_max             set by a USB driver for a host port:
//                               500 mA of input and charge current
//   usb_otg                     1 puts the charger in OTG (boost) mode
//
// The charger interrupt wakes the system from suspend, so that plugging or
// unplugging the cable in standby wakes the player.
//
// The charge limit, in two files next to the properties:
//
//   charge_limit_percent   the battery level (of the power supply "battery",
//                          the fuel gauge) at which charging stops; it starts
//                          again 3 % below. 50 to 100; 100, the default, is
//                          no limit.
//   charge_limit_held      1 while charging is stopped at the limit
//
// Charging stops with CHG_CONFIG, as step_charging_enabled does, so the input
// keeps supplying the system. While the cable is in and a limit is set, the
// system is kept out of suspend: nothing would watch the level otherwise.
//
// Loaded after utils.ko, which provides i2c_register_device() and the pin
// parameters.

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/usb/phy.h>
#include <linux/workqueue.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);

#define MP2731_MANUFACTURER	"MonolithicPower"
#define MP2731_IRQ_PIN		"mp2731_irq"
#define MP2731_ID		0

#define LIMIT_BATTERY		"battery"	// the fuel gauge, cw2015.ko
#define LIMIT_HYSTERESIS	3		// percent
#define LIMIT_MIN		50		// percent; a lower limit is refused

// The charge_limit_percent parameter: the limit the driver starts with.
static int charge_limit_percent = 100;

// The fields are named after the datasheet where it names them. The rest are
// named after their register and bits; the comments say what the driver does
// with them.
enum mp2731_fields {
	F_EN_HIZ, F_EN_ILIM, F_IIN_LIM,				// REG00
	F_REG_RST, F_VIN_MIN,					// REG01
	F_TSM_DLY, F_NTC_TYPE, F_EN_OTG_NTC, F_EN_CHG_NTC,
	F_TJ_REG, F_NTC_OPT, F_AICO_EN,				// REG02
	F_ADC_START, F_ADC_RATE, F_VIN_DSCHG, F_IIN_DSCHG,	// REG03
	F_BAT_LOADEN, F_STAT_EN, F_CHG_CONFIG, F_VSYS_MIN,
	F_VTRACK,						// REG04
	F_VBATT_PRE, F_ICC,					// REG05
	F_IPRE, F_ITERM,					// REG06
	F_VBATT_REG, F_VRECH,					// REG07
	F_EN_TERM, F_WATCHDOG, F_WD_TR, F_CHG_TMR, F_EN_TIMER,	// REG08
	F_BG_EN,						// REG09
	F_SW_FREQ, F_TMR2X_EN, F_BATFET_DIS, F_SYSRST_SEL,
	F_TDISC_H, F_TDISC_L,					// REG0A
	F_R0B_7_6, F_USB_DET_EN, F_R0B_4, F_R0B_3, F_R0B_2_1,
	F_R0B_0,						// REG0B
	F_VIN_STAT, F_CHG_STAT, F_R0C_2, F_R0C_1, F_R0C_0,	// REG0C
	F_WATCHDOG_FAULT, F_OTG_FAULT, F_R0D_5,
	F_THERMAL_FAULT,	// reported as overheat
	F_BAT_FAULT,		// reported as overvoltage
	F_NTC_FAULT,						// REG0D
	F_ADC_VBATT,						// REG0E, 20 mV
	F_ADC_VSYS,						// REG0F
	F_ADC_VNTC,						// REG10
	F_ADC_VIN,						// REG11, 60 mV
	F_ADC_ICHG,						// REG12, 17.5 mA
	F_ADC_IIN,						// REG13, 13.3 mA
	F_R14_7, F_R14_6,
	F_IIN_LIM_NOW,		// the IIN_LIM in force, read with AICO
								// REG14
	F_R15_7, F_R15_6, F_R15_5,				// REG15
	F_JEITA_VSET, F_JEITA_ISET, F_R16_5, F_R16_4_3, F_R16_2_1,
	F_R16_0,						// REG16
	F_TIMER_FAULT, F_PART_NUM,				// REG17
	F_R18_7, F_R18_3, F_R18_2, F_R18_1,			// REG18

	F_MAX_FIELDS
};

// The values of the init fields, converted to register values.
struct mp2731_init_data {
	u8 iilim;	// input current limit
	u8 icc;		// fast charge current
	u8 vbatt_reg;	// battery regulation voltage
	u8 iterm;	// termination current
	u8 iprechg;	// precharge current
	u8 sysvmin;	// minimum system voltage
	u8 vinmin;	// input voltage regulation
	u8 otgv;	// OTG voltage
	u8 otgi;	// OTG current limit
	u8 boostf;	// "mp,boost-low-freq"; not written to the chip
	u8 ilim_en;	// ILIM pin enable
	u8 treg;	// thermal regulation threshold
};

struct mp2731_state {
	u8 online;		// VIN_STAT neither 0 (no input) nor 7 (OTG)
	u8 vin_stat;
	u8 chrg_status;
	u8 r0c_2;
	u8 r0c_1;
	u8 r0c_0;
	u8 wd_fault;
	u8 otg_fault;
	u8 r0d_5;
	u8 thermal_fault;
	u8 bat_fault;
	u8 ntc_fault;
	u8 timer_fault;
	u8 r14_7;
	u8 r14_6;
	u8 r15_7;
};

// What mp2731.sh fills in through the module parameters, or a board file.
struct mp2731_platform_data {
	u8 boost_low_freq;
	char psy_name[131];		// power supply to follow, may be empty
	int ce_gpio;
	int otg_gpio;
	int input_suspend_current;	// uA, 0 for the default
	int input_limit;		// uA
	int charge_limit;		// uA
	int min_sys_voltage;		// uV
	int min_input_voltage;		// uV
	int vbat_target;		// uV
	int termination_current;	// uA
	int precharge_current;		// uA
	int thermal_threshold;		// degrees C
	int otg_voltage;		// uV
	int otg_max_current;		// uA
};

struct mp2731_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct work_struct irq_work;

	struct usb_phy *usb_phy;
	struct usb_phy *usb3_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;

	struct regmap *rmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];

	char psy_name[128];
	struct notifier_block psy_nb;
	struct delayed_work charge_work;
	struct delayed_work psy_work;

	int typec_power_role;	// stored for user space, nothing else
	int usb_otg;
	int ce_gpio;
	int otg_gpio;
	int sdp_current;	// sdp_current_max, as the worker last took it
	int sdp_current_max;
	int chip_id;
	int charge_limit;	// uA
	int charge_enabled;
	int real_type;		// POWER_SUPPLY_TYPE_*
	int temp_errors;	// ADC reads of the NTC that gave nothing
	int temp;		// last temperature, tenths of degree C
	int aico;		// AICO_EN as written by the worker
	int apply;		// the worker has currents to write
	int retries;
	int ichg;		// charge current the worker aims at, uA
	int iin;		// input current the worker aims at, uA
	int online;		// state.online, as the worker last took it
	int stage;		// worker stage, STAGE_*
	int changed;		// something the worker reads has changed
	int stepped_down;	// a step down of the currents is in progress

	// Values from the parameters or the firmware node, before conversion.
	int vbat_target;	// uV
	int iterm;		// uA
	int iprechg;		// uA
	int otg_voltage;	// uV
	int otg_current;	// uA
	int treg;		// degrees C
	int sysvmin;		// uV
	int vinmin;		// uV
	int input_limit;	// uA
	int input_suspend;
	int input_suspend_current;	// uA

	struct mp2731_init_data init_data;
	struct mp2731_state state;

	struct mutex lock; // protects state and the fields the worker reads

	bool wake_armed;	// enable_irq_wake() done by the last suspend

	int limit_percent;	// charge_limit_percent, 100 for no limit
	bool limit_held;	// charging stopped at the limit
	bool limit_awake;	// limit_ws held
	struct wakeup_source *limit_ws;
};

// The worker stages.
enum {
	STAGE_HOST = 1,		// a USB host port: 500 mA
	STAGE_STEP_DOWN,	// fall 100 mA per run until the input holds
	STAGE_DETECT,		// find out what the source is
	STAGE_IDLE,		// currents written; wait for a change
};

static const struct regmap_range mp2731_readonly_reg_ranges[] = {
	regmap_reg_range(0x0c, 0x14),
	regmap_reg_range(0x17, 0x18),
};

static const struct regmap_access_table mp2731_writeable_regs = {
	.no_ranges = mp2731_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(mp2731_readonly_reg_ranges),
};

static const struct regmap_range mp2731_volatile_reg_ranges[] = {
	regmap_reg_range(0x00, 0x18),
};

static const struct regmap_access_table mp2731_volatile_regs = {
	.yes_ranges = mp2731_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(mp2731_volatile_reg_ranges),
};

static const struct regmap_config mp2731_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x18,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &mp2731_writeable_regs,
	.volatile_table = &mp2731_volatile_regs,
};

static const struct reg_field mp2731_reg_fields[] = {
	// REG00
	[F_EN_HIZ]		= REG_FIELD(0x00, 7, 7),
	[F_EN_ILIM]		= REG_FIELD(0x00, 6, 6),
	[F_IIN_LIM]		= REG_FIELD(0x00, 0, 5),
	// REG01
	[F_REG_RST]		= REG_FIELD(0x01, 7, 7),
	[F_VIN_MIN]		= REG_FIELD(0x01, 0, 6),
	// REG02
	[F_TSM_DLY]		= REG_FIELD(0x02, 7, 7),
	[F_NTC_TYPE]		= REG_FIELD(0x02, 6, 6),
	[F_EN_OTG_NTC]		= REG_FIELD(0x02, 5, 5),
	[F_EN_CHG_NTC]		= REG_FIELD(0x02, 4, 4),
	[F_TJ_REG]		= REG_FIELD(0x02, 2, 3),
	[F_NTC_OPT]		= REG_FIELD(0x02, 1, 1),
	[F_AICO_EN]		= REG_FIELD(0x02, 0, 0),
	// REG03
	[F_ADC_START]		= REG_FIELD(0x03, 7, 7),
	[F_ADC_RATE]		= REG_FIELD(0x03, 6, 6),
	[F_VIN_DSCHG]		= REG_FIELD(0x03, 3, 5),
	[F_IIN_DSCHG]		= REG_FIELD(0x03, 0, 2),
	// REG04
	[F_BAT_LOADEN]		= REG_FIELD(0x04, 7, 7),
	[F_STAT_EN]		= REG_FIELD(0x04, 6, 6),
	[F_CHG_CONFIG]		= REG_FIELD(0x04, 4, 5),
	[F_VSYS_MIN]		= REG_FIELD(0x04, 1, 3),
	[F_VTRACK]		= REG_FIELD(0x04, 0, 0),
	// REG05
	[F_VBATT_PRE]		= REG_FIELD(0x05, 7, 7),
	[F_ICC]			= REG_FIELD(0x05, 0, 6),
	// REG06
	[F_IPRE]		= REG_FIELD(0x06, 4, 7),
	[F_ITERM]		= REG_FIELD(0x06, 0, 3),
	// REG07
	[F_VBATT_REG]		= REG_FIELD(0x07, 1, 7),
	[F_VRECH]		= REG_FIELD(0x07, 0, 0),
	// REG08
	[F_EN_TERM]		= REG_FIELD(0x08, 7, 7),
	[F_WATCHDOG]		= REG_FIELD(0x08, 4, 5),
	[F_WD_TR]		= REG_FIELD(0x08, 3, 3),
	[F_CHG_TMR]		= REG_FIELD(0x08, 1, 2),
	[F_EN_TIMER]		= REG_FIELD(0x08, 0, 0),
	// REG09
	[F_BG_EN]		= REG_FIELD(0x09, 3, 3),
	// REG0A
	[F_SW_FREQ]		= REG_FIELD(0x0a, 7, 7),
	[F_TMR2X_EN]		= REG_FIELD(0x0a, 6, 6),
	[F_BATFET_DIS]		= REG_FIELD(0x0a, 5, 5),
	[F_SYSRST_SEL]		= REG_FIELD(0x0a, 4, 4),
	[F_TDISC_H]		= REG_FIELD(0x0a, 2, 3),
	[F_TDISC_L]		= REG_FIELD(0x0a, 0, 1),
	// REG0B
	[F_R0B_7_6]		= REG_FIELD(0x0b, 6, 7),
	[F_USB_DET_EN]		= REG_FIELD(0x0b, 5, 5),
	[F_R0B_4]		= REG_FIELD(0x0b, 4, 4),
	[F_R0B_3]		= REG_FIELD(0x0b, 3, 3),
	[F_R0B_2_1]		= REG_FIELD(0x0b, 1, 2),
	[F_R0B_0]		= REG_FIELD(0x0b, 0, 0),
	// REG0C
	[F_VIN_STAT]		= REG_FIELD(0x0c, 5, 7),
	[F_CHG_STAT]		= REG_FIELD(0x0c, 3, 4),
	[F_R0C_2]		= REG_FIELD(0x0c, 2, 2),
	[F_R0C_1]		= REG_FIELD(0x0c, 1, 1),
	[F_R0C_0]		= REG_FIELD(0x0c, 0, 0),
	// REG0D
	[F_WATCHDOG_FAULT]	= REG_FIELD(0x0d, 7, 7),
	[F_OTG_FAULT]		= REG_FIELD(0x0d, 6, 6),
	[F_R0D_5]		= REG_FIELD(0x0d, 5, 5),
	[F_THERMAL_FAULT]	= REG_FIELD(0x0d, 4, 4),
	[F_BAT_FAULT]		= REG_FIELD(0x0d, 3, 3),
	[F_NTC_FAULT]		= REG_FIELD(0x0d, 0, 2),
	// REG0E to REG13
	[F_ADC_VBATT]		= REG_FIELD(0x0e, 0, 7),
	[F_ADC_VSYS]		= REG_FIELD(0x0f, 0, 7),
	[F_ADC_VNTC]		= REG_FIELD(0x10, 0, 7),
	[F_ADC_VIN]		= REG_FIELD(0x11, 0, 7),
	[F_ADC_ICHG]		= REG_FIELD(0x12, 0, 7),
	[F_ADC_IIN]		= REG_FIELD(0x13, 0, 7),
	// REG14
	[F_R14_7]		= REG_FIELD(0x14, 7, 7),
	[F_R14_6]		= REG_FIELD(0x14, 6, 6),
	[F_IIN_LIM_NOW]		= REG_FIELD(0x14, 0, 5),
	// REG15
	[F_R15_7]		= REG_FIELD(0x15, 7, 7),
	[F_R15_6]		= REG_FIELD(0x15, 6, 6),
	[F_R15_5]		= REG_FIELD(0x15, 5, 5),
	// REG16
	[F_JEITA_VSET]		= REG_FIELD(0x16, 7, 7),
	[F_JEITA_ISET]		= REG_FIELD(0x16, 6, 6),
	[F_R16_5]		= REG_FIELD(0x16, 5, 5),
	[F_R16_4_3]		= REG_FIELD(0x16, 3, 4),
	[F_R16_2_1]		= REG_FIELD(0x16, 1, 2),
	[F_R16_0]		= REG_FIELD(0x16, 0, 0),
	// REG17
	[F_TIMER_FAULT]		= REG_FIELD(0x17, 7, 7),
	[F_PART_NUM]		= REG_FIELD(0x17, 3, 5),
	// REG18
	[F_R18_7]		= REG_FIELD(0x18, 7, 7),
	[F_R18_3]		= REG_FIELD(0x18, 3, 3),
	[F_R18_2]		= REG_FIELD(0x18, 2, 2),
	[F_R18_1]		= REG_FIELD(0x18, 1, 1),
};

// Most of the val -> idx conversions can be computed, given the minimum,
// maximum and the step between values. For the rest of the conversions there
// are lookup tables.
enum mp2731_table_ids {
	// range tables
	TBL_IILIM,
	TBL_ICC,
	TBL_ITERM,
	TBL_IPRECHG,
	TBL_VBATT_REG,
	TBL_OTGV,
	TBL_SYSVMIN,
	TBL_VINMIN,

	// lookup tables
	TBL_TREG,
	TBL_OTGI,
};

// Thermal regulation threshold, degrees C.
static const u32 mp2731_treg_tbl[] = { 60, 80, 100, 120 };

// OTG current limit, uA.
static const u32 mp2731_ids_chg_tbl[] = {
	500000, 800000, 1100000, 1500000, 1800000, 2100000, 2400000, 3000000
};

struct mp2731_range {
	u32 min;
	u32 max;
	u32 step;
};

struct mp2731_lookup {
	const u32 *tbl;
	u32 size;
};

static const union {
	struct mp2731_range rt;
	struct mp2731_lookup lt;
} mp2731_tables[] = {
	// range tables
	[TBL_IILIM] =		{ .rt = {100000, 3250000, 50000} },	// uA
	[TBL_ICC] =		{ .rt = {320000, 4520000, 40000} },	// uA
	[TBL_ITERM] =		{ .rt = {120000, 720000, 40000} },	// uA
	[TBL_IPRECHG] =		{ .rt = {150000, 750000, 40000} },	// uA
	[TBL_VBATT_REG] =	{ .rt = {3400000, 4670000, 10000} },	// uV
	[TBL_OTGV] =		{ .rt = {4800000, 5500000, 100000} },	// uV
	[TBL_SYSVMIN] =		{ .rt = {3000000, 3750000, 150000} },	// uV
	[TBL_VINMIN] =		{ .rt = {3700000, 15200000, 100000} },	// uV

	// lookup tables
	[TBL_TREG] =	{ .lt = {mp2731_treg_tbl, ARRAY_SIZE(mp2731_treg_tbl)} },
	[TBL_OTGI] =	{ .lt = {mp2731_ids_chg_tbl, ARRAY_SIZE(mp2731_ids_chg_tbl)} },
};

static int mp2731_field_read(struct mp2731_device *mp, enum mp2731_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(mp->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}

static int mp2731_field_write(struct mp2731_device *mp, enum mp2731_fields field_id, u8 val)
{
	return regmap_field_write(mp->rmap_fields[field_id], val);
}

static u8 mp2731_find_idx(u32 value, enum mp2731_table_ids id)
{
	u8 idx;

	if (id >= TBL_TREG) {
		const u32 *tbl = mp2731_tables[id].lt.tbl;
		u32 tbl_size = mp2731_tables[id].lt.size;

		for (idx = 1; idx < tbl_size && tbl[idx] <= value; idx++)
			;
	} else {
		const struct mp2731_range *rtbl = &mp2731_tables[id].rt;
		u8 rtbl_size;

		rtbl_size = (rtbl->max - rtbl->min) / rtbl->step + 1;

		for (idx = 1; idx < rtbl_size && (idx * rtbl->step + rtbl->min <= value); idx++)
			;
	}

	return idx - 1;
}

// CHG_STAT
enum mp2731_status {
	STATUS_NOT_CHARGING,
	STATUS_PRE_CHARGING,
	STATUS_FAST_CHARGING,
	STATUS_TERMINATION_DONE,
};

// The CHG_CONFIG to go back to when OTG ends: charging on or off.
static u8 mp2731_chg_config(struct mp2731_device *mp)
{
	return mp->charge_enabled && !mp->limit_held ? 1 : 0;
}

static int mp2731_get_chip_state(struct mp2731_device *mp, struct mp2731_state *state)
{
	unsigned int i;
	int ret;

	struct {
		enum mp2731_fields id;
		u8 *data;
	} state_fields[] = {
		{F_VIN_STAT,		&state->vin_stat},
		{F_CHG_STAT,		&state->chrg_status},
		{F_R0C_2,		&state->r0c_2},
		{F_R0C_1,		&state->r0c_1},
		{F_R0C_0,		&state->r0c_0},
		{F_R14_7,		&state->r14_7},
		{F_R14_6,		&state->r14_6},
		{F_R15_7,		&state->r15_7},
		{F_WATCHDOG_FAULT,	&state->wd_fault},
		{F_OTG_FAULT,		&state->otg_fault},
		{F_THERMAL_FAULT,	&state->thermal_fault},
		{F_BAT_FAULT,		&state->bat_fault},
		{F_NTC_FAULT,		&state->ntc_fault},
		{F_R0D_5,		&state->r0d_5},
		{F_TIMER_FAULT,		&state->timer_fault},
	};

	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = mp2731_field_read(mp, state_fields[i].id);
		if (ret < 0)
			return ret;

		*state_fields[i].data = ret;
	}

	state->online = state->vin_stat != 0 && state->vin_stat != 7;

	// The safety timer ran out: switching it off and on again restarts it.
	if (state->timer_fault) {
		mp2731_field_write(mp, F_EN_TIMER, 0);
		mp2731_field_write(mp, F_EN_TIMER, 1);
	}

	return 0;
}

static int mp2731_power_supply_get_property(struct power_supply *psy,
					    enum power_supply_property psp,
					    union power_supply_propval *val)
{
	int ret;
	struct mp2731_device *mp = power_supply_get_drvdata(psy);
	struct mp2731_state state;

	mutex_lock(&mp->lock);
	state = mp->state;
	mutex_unlock(&mp->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.online)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_status == STATUS_NOT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == STATUS_PRE_CHARGING ||
			 state.chrg_status == STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = state.online && state.chrg_status == STATUS_TERMINATION_DONE;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = MP2731_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = mp->real_type;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (state.wd_fault)
			val->intval = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
		else if (state.thermal_fault)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (state.bat_fault)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.ntc_fault == 0)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (state.ntc_fault == 2)
			val->intval = POWER_SUPPLY_HEALTH_WARM;
		else if (state.ntc_fault == 3)
			val->intval = POWER_SUPPLY_HEALTH_COOL;
		else if (state.ntc_fault == 5)
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else if (state.ntc_fault == 6)
			val->intval = POWER_SUPPLY_HEALTH_HOT;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mp2731_field_read(mp, F_ADC_ICHG); // measured
		if (ret < 0)
			return ret;

		val->intval = ret * 17500;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = mp2731_tables[TBL_ICC].rt.max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mp2731_field_read(mp, F_ADC_VBATT); // measured
		val->intval = ret > 0 ? ret * 20000 : 4200000;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = mp2731_tables[TBL_VBATT_REG].rt.max;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED:
		ret = mp2731_field_read(mp, F_ADC_VIN); // measured
		if (ret < 0)
			return ret;

		val->intval = ret * 60000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = mp->init_data.iterm * 40000 + 120000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = mp->charge_limit;
		break;

	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
		val->intval = mp->charge_enabled;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		// With AICO on, the input current limit the chip settled on;
		// otherwise the ADC.
		ret = mp2731_field_read(mp, F_AICO_EN);
		if (ret) {
			ret = mp2731_field_read(mp, F_IIN_LIM_NOW);
			if (ret < 0)
				return ret;

			val->intval = ret * 50000 + 100000;
		} else {
			ret = mp2731_field_read(mp, F_ADC_IIN);
			if (ret < 0)
				return ret;

			val->intval = ret * 13300;
		}
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = mp->input_limit;
		break;

	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		val->intval = mp->input_suspend;
		break;

	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
		val->intval = mp->sdp_current_max;
		break;

	case POWER_SUPPLY_PROP_TYPEC_POWER_ROLE:
		val->intval = mp->typec_power_role;
		break;

	case POWER_SUPPLY_PROP_USB_OTG:
		val->intval = mp->usb_otg;
		break;

	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGER_TEMP:
		// Tenths of degree C, from the NTC while charging. Not charging,
		// a fixed 30 C; ten failed reads in a row, a fixed 70 C.
		if (state.chrg_status < STATUS_PRE_CHARGING ||
		    state.chrg_status > STATUS_TERMINATION_DONE) {
			val->intval = 300;
			mp->temp_errors = 0;
			break;
		}

		ret = mp2731_field_read(mp, F_ADC_VNTC);
		if (ret > 0) {
			mp->temp = ((73000 - ret * 392) / 39 * 65 - 10000) / 1000;
			val->intval = mp->temp;
			mp->temp_errors = 0;
		} else {
			val->intval = mp->temp;
			mp->temp_errors++;
		}
		if (mp->temp_errors >= 10)
			val->intval = 700;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int mp2731_power_supply_set_property(struct power_supply *psy,
					    enum power_supply_property psp,
					    const union power_supply_propval *val)
{
	struct mp2731_device *mp = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		mp->charge_limit = val->intval;
		break;

	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
		mp->charge_enabled = !!val->intval;
		break;

	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		mp->input_suspend = !!val->intval;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		mp->input_limit = val->intval;
		break;

	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
		mutex_lock(&mp->lock);
		mp->sdp_current_max = val->intval < 0 ? 500000 : val->intval;
		mp->changed = 1;
		mutex_unlock(&mp->lock);
		return 0;

	case POWER_SUPPLY_PROP_TYPEC_POWER_ROLE:
		mp->typec_power_role = val->intval;
		return 0;

	case POWER_SUPPLY_PROP_USB_OTG:
		mutex_lock(&mp->lock);
		mp->changed = 1;
		mutex_unlock(&mp->lock);

		mp->usb_otg = val->intval;
		if (mp->usb_otg)
			mp2731_field_write(mp, F_CHG_CONFIG, 3);
		else
			mp2731_field_write(mp, F_CHG_CONFIG, mp2731_chg_config(mp));
		return 0;

	default:
		return -EINVAL;
	}

	mutex_lock(&mp->lock);
	mp->changed = 1;
	mutex_unlock(&mp->lock);

	return 0;
}

static int mp2731_power_supply_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_TYPEC_POWER_ROLE:
	case POWER_SUPPLY_PROP_USB_OTG:
		return 1;
	default:
		return 0;
	}
}

// The battery level, -errno when there is none to read.
static int mp2731_battery_capacity(void)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int ret;

	psy = power_supply_get_by_name(LIMIT_BATTERY);
	if (!psy)
		return -ENODEV;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	power_supply_put(psy);
	if (ret < 0)
		return ret;

	return val.intval;
}

// The charge limit, at every run of the worker. Without a cable, or without a
// limit, charging is never held: the bit would survive the cable going out,
// and the next cable would meet a charger that does nothing. Without a
// battery level to judge by, nothing changes.
static void mp2731_charge_limit_update(struct mp2731_device *mp, const struct mp2731_state *state)
{
	int limit = READ_ONCE(mp->limit_percent);
	bool active = limit < 100 && state->online;
	bool held = mp->limit_held;

	if (!active) {
		held = false;
	} else {
		int capacity = mp2731_battery_capacity();

		if (capacity >= 0)
			held = held ? capacity > limit - LIMIT_HYSTERESIS : capacity >= limit;
	}

	if (active != mp->limit_awake) {
		mp->limit_awake = active;
		if (active)
			__pm_stay_awake(mp->limit_ws);
		else
			__pm_relax(mp->limit_ws);
	}

	if (held == mp->limit_held)
		return;

	mp->limit_held = held;
	if (!mp->usb_otg)
		mp2731_field_write(mp, F_CHG_CONFIG, mp2731_chg_config(mp));
	dev_info(mp->dev, "charging %s at the %d%% limit\n", held ? "held off" : "released", limit);
	power_supply_changed(mp->charger);
}

// The worker: decides the input and charge currents and writes them.
//
// At a cable change it starts from STAGE_DETECT:
//   - no input: 100 mA, then idle;
//   - a USB host (sdp_current_max set): 500 mA, STAGE_HOST;
//   - otherwise, the followed power supply's voltage and current limit, if
//     it gives both: its current as input, the power that makes at 4.4 V as
//     charge current, then idle. If it does not answer within 50 runs
//     (5 s): input_current_limited and charge_control_limit, then
//     STAGE_STEP_DOWN.
// STAGE_STEP_DOWN takes 100 mA off both currents per run while there is no
// input, down to 100 mA, and idles once it is back. STAGE_HOST idles while
// the host is there; once it is gone it drops to 100 mA and detects again.
// STAGE_IDLE starts again at the next change.
static void mp2731_charge_update_work(struct work_struct *work)
{
	struct mp2731_device *mp = container_of(to_delayed_work(work), struct mp2731_device, charge_work);
	struct mp2731_state state;
	unsigned int delay = 1000;
	bool changed = false;
	int ret;

	mutex_lock(&mp->lock);
	if (mp->changed) {
		mp->changed = 0;
		changed = true;
		mp->sdp_current = mp->sdp_current_max;
		mp->online = mp->state.online;
	}
	state = mp->state;
	mutex_unlock(&mp->lock);

	// The state in hand disagrees with itself (an input but no charging, or
	// the other way round): read it again.
	if ((state.vin_stat != 0 && state.vin_stat != 7) != (state.chrg_status != 0) &&
	    mp2731_get_chip_state(mp, &state) >= 0) {
		mutex_lock(&mp->lock);
		mp->state = state;
		mutex_unlock(&mp->lock);
	}

	switch (mp->stage) {
	case STAGE_DETECT:
		if (!mp->online) {
			mp->apply = 1;
			mp->iin = 100000;
			mp->ichg = 100000;
			mp->stage = STAGE_IDLE;
			mp->retries = 0;
			delay = 100;
		} else if (mp->sdp_current) {
			mp->apply = 1;
			mp->iin = 500000;
			mp->ichg = 500000;
			mp->stage = STAGE_HOST;
			mp->retries = 0;
			delay = 100;
		} else {
			union power_supply_propval voltage = { 0 }, current_max = { 0 };
			struct power_supply *psy = power_supply_get_by_name(mp->psy_name);

			if (psy) {
				power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &voltage);
				power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &current_max);
				power_supply_put(psy);
			}

			if (voltage.intval && current_max.intval) {
				mp->apply = 1;
				mp->iin = current_max.intval;
				mp->retries = 0;
				mp->ichg = voltage.intval / 1000 * current_max.intval / 4400;
				mp->stage = STAGE_IDLE;
			} else if (++mp->retries < 50) {
				delay = 100;
			} else {
				mp->apply = 1;
				mp->iin = mp->input_limit;
				mp->stepped_down = 1;
				mp->ichg = mp->charge_limit;
				mp->stage = STAGE_STEP_DOWN;
				mp->retries = 0;
			}
		}
		break;

	case STAGE_STEP_DOWN:
		if (mp->sdp_current) {
			mp->apply = 1;
			mp->iin = 500000;
			mp->ichg = 500000;
			mp->stage = STAGE_HOST;
			break;
		}

		if (!state.vin_stat)
			mp->stepped_down = 0;
		if (mp->stepped_down) {
			mp->stage = STAGE_IDLE;
			break;
		}

		mp->stepped_down = 1;
		mp->apply = 1;
		mp->iin -= 100000;
		mp->ichg -= 100000;
		if (mp->iin <= 100000 || mp->ichg <= 100000) {
			mp->iin = 100000;
			mp->ichg = 100000;
			mp->stage = STAGE_IDLE;
		} else {
			mp->stage = STAGE_STEP_DOWN;
			delay = 100;
		}
		break;

	case STAGE_HOST:
		if (mp->sdp_current) {
			mp->stage = STAGE_IDLE;
		} else {
			mp->apply = 1;
			mp->iin = 100000;
			mp->ichg = 100000;
			mp->stage = STAGE_DETECT;
		}
		break;

	case STAGE_IDLE:
		if (!changed)
			break;

		mp->apply = 1;
		if (mp->sdp_current) {
			mp->iin = 500000;
			mp->ichg = 500000;
			mp->stage = STAGE_HOST;
		} else {
			mp->iin = 100000;
			mp->ichg = 100000;
			mp->stage = STAGE_DETECT;
		}
		break;
	}

	if (mp->apply) {
		int limit = mp->input_suspend ? mp->input_suspend_current : mp->input_limit;
		int iin = min(mp->iin, limit);
		int ichg = min(mp->ichg, mp->charge_limit);

		ret = regmap_field_write(mp->rmap_fields[F_IIN_LIM], mp2731_find_idx(iin, TBL_IILIM));
		if (ret < 0)
			pr_err("failed to write F_IILIM ret=%d\n", ret);

		ret = regmap_field_write(mp->rmap_fields[F_ICC], mp2731_find_idx(ichg, TBL_ICC));
		if (ret < 0)
			pr_err("failed to write F_ICC ret=%d\n", ret);

		if (mp->usb_otg) {
			mp2731_field_write(mp, F_CHG_CONFIG, 3);
		} else if (mp2731_chg_config(mp)) {
			mp2731_field_write(mp, F_CHG_CONFIG, 1);
			mp2731_field_write(mp, F_AICO_EN, mp->aico ? 1 : 0);
		} else {
			mp2731_field_write(mp, F_CHG_CONFIG, 0);
		}

		// Above 6 V in, a high-voltage charger.
		mp->real_type = POWER_SUPPLY_TYPE_USB;
		ret = mp2731_field_read(mp, F_ADC_VIN);
		if (ret > 0 && ret * 60000 > 6000000)
			mp->real_type = POWER_SUPPLY_TYPE_USB_HVDCP;

		mp->apply = 0;
	}

	mp2731_charge_limit_update(mp, &state);

	queue_delayed_work(system_wq, &mp->charge_work, msecs_to_jiffies(delay));
}

static void mp2731_notifier_call_update_work(struct work_struct *work)
{
	struct mp2731_device *mp = container_of(to_delayed_work(work), struct mp2731_device, psy_work);

	mutex_lock(&mp->lock);
	mp->changed = 1;
	mutex_unlock(&mp->lock);
}

// A change of the followed power supply counts as a change for the worker.
static int mp2731_notifier_call(struct notifier_block *nb, unsigned long event, void *data)
{
	struct mp2731_device *mp = container_of(nb, struct mp2731_device, psy_nb);
	struct power_supply *psy = data;

	if (strcmp(psy->desc->name, mp->psy_name) == 0 && event == PSY_EVENT_PROP_CHANGED)
		queue_delayed_work(system_wq, &mp->psy_work, 0);

	return NOTIFY_OK;
}

// The interrupt: off until the state has been read, in a work item.
static irqreturn_t mp2731_irq_handler(int irq, void *private)
{
	struct mp2731_device *mp = private;

	// An interrupt that comes while the system is on its way into suspend,
	// before the interrupts are switched to wake-up only, is a wake event
	// too: without this the suspend goes on and the cable is missed.
	if (device_may_wakeup(mp->dev))
		pm_wakeup_event(mp->dev, 0);

	disable_irq_nosync(irq);
	queue_work(system_wq, &mp->irq_work);

	return IRQ_HANDLED;
}

static void mp2731_irq_work(struct work_struct *work)
{
	struct mp2731_device *mp = container_of(work, struct mp2731_device, irq_work);
	struct mp2731_state state, old;

	if (mp2731_get_chip_state(mp, &state) < 0)
		goto out;

	mutex_lock(&mp->lock);
	old = mp->state;
	mutex_unlock(&mp->lock);

	if (old.chrg_status == state.chrg_status && old.online == state.online &&
	    old.vin_stat == state.vin_stat && old.r0c_0 == state.r0c_0 &&
	    old.bat_fault == state.bat_fault && old.otg_fault == state.otg_fault &&
	    old.ntc_fault == state.ntc_fault && old.timer_fault == state.timer_fault)
		goto out;

	mutex_lock(&mp->lock);
	if (mp->state.online != state.online)
		mp->changed = 1;
	mp->state = state;
	mutex_unlock(&mp->lock);

	// The interrupt is requested before the power supply is registered.
	if (!IS_ERR_OR_NULL(mp->charger))
		power_supply_changed(mp->charger);
out:
	enable_irq(mp->client->irq);
}

static int mp2731_chip_reset(struct mp2731_device *mp)
{
	int ret;
	int rst_check_counter = 10;

	ret = mp2731_field_write(mp, F_REG_RST, 1);
	if (ret < 0)
		return ret;

	do {
		ret = mp2731_field_read(mp, F_REG_RST);
		if (ret < 0)
			return ret;

		usleep_range(5, 10);
	} while (ret == 1 && --rst_check_counter);

	if (!rst_check_counter)
		return -ETIMEDOUT;

	return 0;
}

static int mp2731_hw_init(struct mp2731_device *mp)
{
	int ret;
	unsigned int i;
	struct mp2731_state state;

	const struct {
		enum mp2731_fields id;
		u32 value;
	} init_data[] = {
		{F_IIN_LIM,	mp->init_data.iilim},
		{F_ICC,		mp->init_data.icc},
		{F_VBATT_REG,	mp->init_data.vbatt_reg},
		{F_ITERM,	mp->init_data.iterm},
		{F_IPRE,	mp->init_data.iprechg},
		{F_VSYS_MIN,	mp->init_data.sysvmin},
		{F_VIN_MIN,	mp->init_data.vinmin},
		{F_EN_ILIM,	mp->init_data.ilim_en},
		{F_VIN_DSCHG,	mp->init_data.otgv},
		{F_IIN_DSCHG,	mp->init_data.otgi},
		{F_TJ_REG,	mp->init_data.treg},
	};

	// A reset that fails leaves the registers as they are: the writes below
	// set every one that matters.
	ret = mp2731_chip_reset(mp);
	if (ret < 0)
		dev_err(mp->dev, "mp2731_chip_reset ret:%d.\n", ret);

	ret = mp2731_field_write(mp, F_WD_TR, 1);
	if (ret < 0)
		dev_err(mp->dev, "mp2731_field_write F_WD_TR ret:%d.\n", ret);

	// disable the watchdog
	ret = mp2731_field_write(mp, F_WATCHDOG, 0);
	if (ret < 0) {
		dev_err(mp->dev, "mp2731_field_write F_WD ret:%d.\n", ret);
		return ret;
	}

	// initialize currents, voltages and other parameters
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = mp2731_field_write(mp, init_data[i].id, init_data[i].value);
		if (ret < 0) {
			dev_err(mp->dev, "mp2731_field_write ret:%d.\n", ret);
			return ret;
		}
	}

	// The ADC converts continuously once started.
	ret = mp2731_field_write(mp, F_ADC_RATE, 1);
	if (ret < 0) {
		dev_err(mp->dev, "mp2731_field_write F_ADC_RATE ret:%d.\n", ret);
		return ret;
	}

	ret = mp2731_field_write(mp, F_STAT_EN, 0);
	if (ret < 0)
		dev_err(mp->dev, "mp2731_field_write F_STAT_EN ret:%d.\n", ret);

	mp2731_field_write(mp, F_R15_6, 0);
	mp2731_field_write(mp, F_R15_5, 0);

	ret = mp2731_field_write(mp, F_R16_5, 1);
	if (ret < 0)
		return ret;
	ret = mp2731_field_write(mp, F_R16_4_3, 1);
	if (ret < 0)
		return ret;
	ret = mp2731_field_write(mp, F_R16_2_1, 1);
	if (ret < 0)
		return ret;
	ret = mp2731_field_write(mp, F_R16_0, 0);
	if (ret < 0)
		return ret;

	ret = mp2731_field_write(mp, F_USB_DET_EN, 0);
	if (ret < 0)
		return ret;

	ret = mp2731_field_write(mp, F_EN_OTG_NTC, 1);
	if (ret < 0)
		dev_err(mp->dev, "mp2731_field_write F_EN_OTG_NTC ret:%d.\n", ret);

	ret = mp2731_field_write(mp, F_EN_CHG_NTC, 1);
	if (ret < 0)
		dev_err(mp->dev, "mp2731_field_write F_EN_CHG_NTC ret:%d.\n", ret);

	ret = mp2731_get_chip_state(mp, &state);
	if (ret < 0) {
		dev_err(mp->dev, "mp2731_get_chip_state ret:%d.\n", ret);
		return ret;
	}

	mutex_lock(&mp->lock);
	mp->state = state;
	mutex_unlock(&mp->lock);

	return 0;
}

static enum power_supply_property mp2731_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_SDP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPEC_POWER_ROLE,
	POWER_SUPPLY_PROP_USB_OTG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static char *mp2731_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc mp2731_power_supply_desc = {
	.name = "mp2731-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mp2731_power_supply_props,
	.num_properties = ARRAY_SIZE(mp2731_power_supply_props),
	.get_property = mp2731_power_supply_get_property,
	.set_property = mp2731_power_supply_set_property,
	.property_is_writeable = mp2731_power_supply_is_writeable,
};

static struct mp2731_device *mp2731_from_psy_dev(struct device *dev)
{
	return power_supply_get_drvdata(dev_get_drvdata(dev));
}

static ssize_t charge_limit_percent_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", READ_ONCE(mp2731_from_psy_dev(dev)->limit_percent));
}

static ssize_t charge_limit_percent_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct mp2731_device *mp = mp2731_from_psy_dev(dev);
	int val, ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;
	if (val < LIMIT_MIN || val > 100)
		return -EINVAL;

	WRITE_ONCE(mp->limit_percent, val);
	return count;
}
static DEVICE_ATTR_RW(charge_limit_percent);

static ssize_t charge_limit_held_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mp2731_from_psy_dev(dev)->limit_held);
}
static DEVICE_ATTR_RO(charge_limit_held);

static struct attribute *mp2731_limit_attrs[] = {
	&dev_attr_charge_limit_percent.attr,
	&dev_attr_charge_limit_held.attr,
	NULL,
};

static const struct attribute_group mp2731_limit_group = {
	.attrs = mp2731_limit_attrs,
};

static int mp2731_power_supply_init(struct mp2731_device *mp)
{
	struct power_supply_config psy_cfg = { .drv_data = mp, };

	psy_cfg.supplied_to = mp2731_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(mp2731_charger_supplied_to);

	mp->charger = power_supply_register(mp->dev, &mp2731_power_supply_desc, &psy_cfg);

	return PTR_ERR_OR_ZERO(mp->charger);
}

static void mp2731_usb_work(struct work_struct *data)
{
	int ret;
	struct mp2731_device *mp = container_of(data, struct mp2731_device, usb_work);

	switch (mp->usb_event) {
	case USB_EVENT_ID:
		// OTG
		ret = regmap_field_write(mp->rmap_fields[F_CHG_CONFIG], 3);
		if (ret < 0)
			goto error;
		break;

	case USB_EVENT_NONE:
		ret = regmap_field_write(mp->rmap_fields[F_CHG_CONFIG], mp2731_chg_config(mp));
		if (ret < 0)
			goto error;

		if (!IS_ERR_OR_NULL(mp->charger))
			power_supply_changed(mp->charger);
		break;
	}

	return;

error:
	dev_err(mp->dev, "Error switching to boost/charger mode.\n");
}

static int mp2731_usb_notifier(struct notifier_block *nb, unsigned long val, void *priv)
{
	struct mp2731_device *mp = container_of(nb, struct mp2731_device, usb_nb);

	mp->usb_event = val;
	queue_work(system_power_efficient_wq, &mp->usb_work);

	return NOTIFY_OK;
}

// The phy the USB notifier went on, if any.
static struct usb_phy *mp2731_usb_notifier_phy(struct mp2731_device *mp)
{
	if (!IS_ERR_OR_NULL(mp->usb_phy))
		return mp->usb_phy;
	if (!IS_ERR_OR_NULL(mp->usb3_phy))
		return mp->usb3_phy;
	return NULL;
}

static int mp2731_fw_read_u32_props(struct mp2731_device *mp, bool read)
{
	int ret;
	u32 property;
	unsigned int i;
	struct mp2731_init_data *init = &mp->init_data;
	struct {
		char *name;
		bool optional;
		enum mp2731_table_ids tbl_id;
		u8 *conv_data;	// holds the converted value of the property
		int *value;	// the property as it is
	} props[] = {
		// required properties
		{"mp,input-limit-current", false, TBL_IILIM, &init->iilim, &mp->input_limit},
		{"mp,charge-current", false, TBL_ICC, &init->icc, &mp->charge_limit},
		{"mp,battery-regulation-voltage", false, TBL_VBATT_REG, &init->vbatt_reg, &mp->vbat_target},
		{"mp,termination-current", false, TBL_ITERM, &init->iterm, &mp->iterm},
		{"mp,precharge-current", false, TBL_IPRECHG, &init->iprechg, &mp->iprechg},
		{"mp,minimum-sys-voltage", false, TBL_SYSVMIN, &init->sysvmin, &mp->sysvmin},
		{"mp,minimum-input-voltage", false, TBL_VINMIN, &init->vinmin, &mp->vinmin},
		{"mp,otg-voltage", false, TBL_OTGV, &init->otgv, &mp->otg_voltage},
		{"mp,otg-max-current", false, TBL_OTGI, &init->otgi, &mp->otg_current},

		// optional properties
		{"mp,thermal-regulation-threshold", true, TBL_TREG, &init->treg, &mp->treg},
	};

	// initialize data for optional properties
	init->treg = 3; // 120 degrees C

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		if (read) {
			ret = device_property_read_u32(mp->dev, props[i].name, &property);
			if (ret < 0) {
				if (props[i].optional)
					continue;

				return ret;
			}
			*props[i].value = property;
		}

		*props[i].conv_data = mp2731_find_idx(*props[i].value, props[i].tbl_id);
	}

	return 0;
}

// Without platform data: the firmware node.
static int mp2731_fw_probe(struct mp2731_device *mp)
{
	struct device_node *np = mp->client->dev.of_node;
	enum of_gpio_flags flags;
	const char *name;
	u32 val;
	int ret;

	ret = mp2731_fw_read_u32_props(mp, true);
	if (ret < 0)
		return ret;

	mp->init_data.ilim_en = device_property_read_bool(mp->dev, "mp,use-ilim-pin");
	mp->init_data.boostf = device_property_read_bool(mp->dev, "mp,boost-low-freq");

	mp->ce_gpio = of_get_named_gpio_flags(np, "mp,ce-gpio", 0, &flags);

	if (device_property_read_u32(mp->dev, "mp,input-suspend-current", &val) < 0)
		val = 100000;
	mp->input_suspend_current = val;

	if (!device_property_read_string(mp->dev, "pd-name", &name))
		strncpy(mp->psy_name, name, sizeof(mp->psy_name));

	return 0;
}

static void mp2731_pdata_probe(struct mp2731_device *mp, const struct mp2731_platform_data *pdata)
{
	mp->init_data.boostf = pdata->boost_low_freq;
	mp->input_suspend_current = pdata->input_suspend_current ?: 100000;
	if (strlen(pdata->psy_name))
		strncpy(mp->psy_name, pdata->psy_name, sizeof(mp->psy_name));
	mp->ce_gpio = pdata->ce_gpio;
	mp->otg_gpio = pdata->otg_gpio;
	mp->input_limit = pdata->input_limit;
	mp->charge_limit = pdata->charge_limit;
	mp->sysvmin = pdata->min_sys_voltage;
	mp->vinmin = pdata->min_input_voltage;
	mp->vbat_target = pdata->vbat_target;
	mp->iterm = pdata->termination_current;
	mp->iprechg = pdata->precharge_current;
	mp->otg_voltage = pdata->otg_voltage;
	mp->otg_current = pdata->otg_max_current;
	mp->treg = pdata->thermal_threshold;

	mp2731_fw_read_u32_props(mp, false);
}

static int mp2731_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct mp2731_platform_data *pdata;
	struct mp2731_device *mp;
	struct usb_phy *phy;
	int ret;
	unsigned int i;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	mp = devm_kzalloc(dev, sizeof(*mp), GFP_KERNEL);
	if (!mp)
		return -ENOMEM;

	mp->client = client;
	mp->dev = dev;
	mp->limit_percent = clamp(charge_limit_percent, LIMIT_MIN, 100);

	mutex_init(&mp->lock);
	i2c_set_clientdata(client, mp);

	pdata = dev_get_platdata(dev);
	if (pdata) {
		mp2731_pdata_probe(mp, pdata);
	} else {
		ret = mp2731_fw_probe(mp);
		if (ret < 0) {
			dev_err(dev, "Cannot read device properties.\n");
			return ret;
		}
	}

	mp->rmap = devm_regmap_init_i2c(client, &mp2731_regmap_config);
	if (IS_ERR(mp->rmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(mp->rmap);
	}

	for (i = 0; i < ARRAY_SIZE(mp2731_reg_fields); i++) {
		const struct reg_field *reg_fields = mp2731_reg_fields;

		mp->rmap_fields[i] = devm_regmap_field_alloc(dev, mp->rmap, reg_fields[i]);
		if (IS_ERR(mp->rmap_fields[i])) {
			dev_err(dev, "cannot allocate regmap field\n");
			return PTR_ERR(mp->rmap_fields[i]);
		}
	}

	mp->chip_id = mp2731_field_read(mp, F_PART_NUM);
	if (mp->chip_id < 0) {
		dev_err(dev, "Cannot read chip ID.\n");
		return mp->chip_id;
	}

	if (mp->chip_id != MP2731_ID) {
		dev_err(dev, "Chip with ID=%d, not supported!\n", mp->chip_id);
		return -ENODEV;
	}

	ret = mp2731_hw_init(mp);
	if (ret < 0) {
		dev_err(dev, "Cannot initialize the chip ret:%d.\n", ret);
		return ret;
	}

	if (client->irq <= 0) {
		struct gpio_desc *irq = devm_gpiod_get(mp->dev, MP2731_IRQ_PIN, GPIOD_IN);

		if (IS_ERR(irq)) {
			dev_err(mp->dev, "Could not probe irq pin.\n");
			client->irq = PTR_ERR(irq);
		} else {
			client->irq = gpiod_to_irq(irq);
		}
	}

	if (client->irq < 0) {
		dev_err(dev, "No irq resource found.\n");
		return client->irq;
	}

	// OTG reporting, on the first phy there is: one notifier block goes on
	// one chain only.
	INIT_WORK(&mp->usb_work, mp2731_usb_work);
	mp->usb_nb.notifier_call = mp2731_usb_notifier;
	mp->usb_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(mp->usb_phy))
		usb_register_notifier(mp->usb_phy, &mp->usb_nb);
	mp->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	if (!IS_ERR_OR_NULL(mp->usb3_phy) && IS_ERR_OR_NULL(mp->usb_phy))
		usb_register_notifier(mp->usb3_phy, &mp->usb_nb);

	INIT_DELAYED_WORK(&mp->psy_work, mp2731_notifier_call_update_work);
	mp->psy_nb.notifier_call = mp2731_notifier_call;
	ret = power_supply_reg_notifier(&mp->psy_nb);
	if (ret < 0)
		dev_err(mp->dev, "Couldn't register psy notifier rc = %d\n", ret);

	INIT_DELAYED_WORK(&mp->charge_work, mp2731_charge_update_work);
	INIT_WORK(&mp->irq_work, mp2731_irq_work);

	ret = devm_request_threaded_irq(dev, client->irq, NULL, mp2731_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT, MP2731_IRQ_PIN, mp);
	if (ret)
		goto irq_fail;

	ret = mp2731_power_supply_init(mp);
	if (ret < 0) {
		dev_err(dev, "Failed to register power supply\n");
		goto psy_fail;
	}

	if (gpio_is_valid(mp->ce_gpio)) {
		ret = gpio_request(mp->ce_gpio, "ce_gpio");
		if (ret < 0) {
			dev_err(dev, "could not acquire ce gpio (err=%d)\n", ret);
			goto gpio_fail;
		}
		gpiod_direction_output_raw(gpio_to_desc(mp->ce_gpio), 0);
	}

	if (gpio_is_valid(mp->otg_gpio)) {
		ret = gpio_request(mp->otg_gpio, "otg_gpio");
		if (ret < 0) {
			dev_err(dev, "could not acquire otg gpio (err=%d)\n", ret);
			goto ce_fail;
		}
		gpiod_direction_output_raw(gpio_to_desc(mp->otg_gpio), 1);
	}

	// The charge limit is optional: without its files the player can still
	// hold the charge through step_charging_enabled.
	if (sysfs_create_group(&mp->charger->dev.kobj, &mp2731_limit_group))
		dev_err(dev, "could not create the charge limit files\n");
	mp->limit_ws = wakeup_source_register("mp2731-charge-limit");

	mp->charge_enabled = 1;
	regmap_field_write(mp->rmap_fields[F_CHG_CONFIG], 1);
	mp->aico = 1;
	regmap_field_write(mp->rmap_fields[F_AICO_EN], 1);
	mp->real_type = POWER_SUPPLY_TYPE_USB;
	regmap_field_write(mp->rmap_fields[F_SYSRST_SEL], 1);
	regmap_field_write(mp->rmap_fields[F_TDISC_H], 0);
	regmap_field_write(mp->rmap_fields[F_TDISC_L], 0);

	// The cable wakes the system from suspend (see mp2731_suspend()).
	device_init_wakeup(dev, true);

	mp->stage = STAGE_DETECT;
	mp->changed = 1;
	queue_delayed_work(system_wq, &mp->charge_work, 1);

	return 0;

ce_fail:
	if (gpio_is_valid(mp->ce_gpio))
		gpio_free(mp->ce_gpio);
gpio_fail:
	power_supply_unregister(mp->charger);
psy_fail:
	devm_free_irq(dev, client->irq, mp);
	cancel_work_sync(&mp->irq_work);
irq_fail:
	power_supply_unreg_notifier(&mp->psy_nb);
	cancel_delayed_work_sync(&mp->psy_work);
	phy = mp2731_usb_notifier_phy(mp);
	if (phy) {
		usb_unregister_notifier(phy, &mp->usb_nb);
		cancel_work_sync(&mp->usb_work);
	}

	return ret;
}

// The charge enable bit as user space left it, without the limit's hold, and a
// second for the chip to settle before the power goes: a player switched off
// at its limit must not meet the next cable with a charger that does nothing.
static void mp2731_release_otg(struct mp2731_device *mp)
{
	mp->usb_otg = 0;
	mp->limit_held = false;
	mp2731_field_write(mp, F_CHG_CONFIG, mp2731_chg_config(mp));
	mdelay(1000);
}

static int mp2731_remove(struct i2c_client *client)
{
	struct mp2731_device *mp = i2c_get_clientdata(client);
	struct usb_phy *phy;

	device_init_wakeup(mp->dev, false);
	disable_irq(client->irq);
	cancel_work_sync(&mp->irq_work);
	cancel_delayed_work_sync(&mp->charge_work);

	// The USB work calls power_supply_changed(): gone before the supply.
	phy = mp2731_usb_notifier_phy(mp);
	if (phy) {
		usb_unregister_notifier(phy, &mp->usb_nb);
		cancel_work_sync(&mp->usb_work);
	}

	if (mp->limit_awake)
		__pm_relax(mp->limit_ws);
	wakeup_source_unregister(mp->limit_ws);
	sysfs_remove_group(&mp->charger->dev.kobj, &mp2731_limit_group);
	power_supply_unregister(mp->charger);
	power_supply_unreg_notifier(&mp->psy_nb);
	cancel_delayed_work_sync(&mp->psy_work);

	mp2731_release_otg(mp);

	if (gpio_is_valid(mp->otg_gpio))
		gpio_free(mp->otg_gpio);
	if (gpio_is_valid(mp->ce_gpio))
		gpio_free(mp->ce_gpio);

	return 0;
}

static void mp2731_shutdown(struct i2c_client *client)
{
	struct mp2731_device *mp = i2c_get_clientdata(client);

	pr_info("mp2731_shutdown\n");
	// The worker stopped first, so that it cannot hold the charge again.
	cancel_delayed_work_sync(&mp->charge_work);
	mp2731_release_otg(mp);
}

#ifdef CONFIG_PM_SLEEP
static int mp2731_suspend(struct device *dev)
{
	struct mp2731_device *mp = dev_get_drvdata(dev);
	int ret;

	// The ADC is off in suspend; it uses slightly more power.
	ret = regmap_field_write(mp->rmap_fields[F_ADC_START], 0);
	if (ret < 0)
		return ret;

	// The interrupt wakes the system: a cable plugged in or pulled out in
	// suspend.
	if (device_may_wakeup(dev))
		mp->wake_armed = !enable_irq_wake(mp->client->irq);

	return 0;
}

static int mp2731_resume(struct device *dev)
{
	int ret;
	struct mp2731_state state;
	struct mp2731_device *mp = dev_get_drvdata(dev);

	if (mp->wake_armed) {
		disable_irq_wake(mp->client->irq);
		mp->wake_armed = false;
	}

	ret = mp2731_get_chip_state(mp, &state);
	if (ret < 0)
		return ret;

	mutex_lock(&mp->lock);
	mp->state = state;
	mutex_unlock(&mp->lock);

	ret = regmap_field_write(mp->rmap_fields[F_ADC_START], 1);
	if (ret < 0)
		return ret;

	// signal user space, maybe the state changed while suspended
	power_supply_changed(mp->charger);

	return 0;
}
#endif

static const struct dev_pm_ops mp2731_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(mp2731_suspend, mp2731_resume)
};

static const struct acpi_device_id mp2731_acpi_match[] = {
	{"MP2731", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, mp2731_acpi_match);

static const struct of_device_id mp2731_of_match[] = {
	{ .compatible = "mp,mp2731", },
	{ },
};
MODULE_DEVICE_TABLE(of, mp2731_of_match);

static const struct i2c_device_id mp2731_i2c_ids[] = {
	{ "mp2731-charger", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mp2731_i2c_ids);

static struct i2c_driver mp2731_driver = {
	.driver = {
		.name = "mp2731-charger",
		.of_match_table = of_match_ptr(mp2731_of_match),
		.acpi_match_table = ACPI_PTR(mp2731_acpi_match),
		.pm = &mp2731_pm,
	},
	.probe = mp2731_probe,
	.remove = mp2731_remove,
	.shutdown = mp2731_shutdown,
	.id_table = mp2731_i2c_ids,
};

// The device: what mp2731.sh passes, on the bus it names.
static int i2c_bus_num = -1;
static int otg_enable_gpio = -1;
static int charge_enable_gpio = -1;
static int int_gpio = -1;

static struct mp2731_platform_data platdata = {
	.ce_gpio = -1,
	.otg_gpio = -1,
	.input_suspend_current = 100000,
	.input_limit = 500000,
	.charge_limit = 500000,
	.min_sys_voltage = 3500000,
	.min_input_voltage = 4500000,
	.vbat_target = 4350000,
	.termination_current = 120000,
	.precharge_current = 120000,
	.thermal_threshold = 120,
	.otg_voltage = 5000000,
	.otg_max_current = 1000000,
};

module_param(charge_limit_percent, int, 0644);
module_param_cb(int_gpio, &param_gpio_ops, &int_gpio, 0644);
module_param_cb(charge_enable_gpio, &param_gpio_ops, &charge_enable_gpio, 0644);
module_param_cb(otg_enable_gpio, &param_gpio_ops, &otg_enable_gpio, 0644);
module_param(i2c_bus_num, int, 0644);
module_param_named(input_limit, platdata.input_limit, int, 0644);
module_param_named(charge_limit, platdata.charge_limit, int, 0644);
module_param_named(min_sys_voltage, platdata.min_sys_voltage, int, 0644);
module_param_named(min_input_voltage, platdata.min_input_voltage, int, 0644);
module_param_named(vbat_target, platdata.vbat_target, int, 0644);
module_param_named(termination_current, platdata.termination_current, int, 0644);
module_param_named(precharge_current, platdata.precharge_current, int, 0644);
module_param_named(otg_max_current, platdata.otg_max_current, int, 0644);

static struct i2c_board_info mp2731_board_info = {
	I2C_BOARD_INFO("mp2731-charger", 0x4b),
};

static struct i2c_client *mp2731_dev;

static int mp2731_dev_init(void)
{
	platdata.ce_gpio = charge_enable_gpio;
	platdata.otg_gpio = otg_enable_gpio;
	mp2731_board_info.platform_data = &platdata;
	mp2731_board_info.irq = gpiod_to_irq(gpio_to_desc(int_gpio));
	mp2731_dev = i2c_register_device(&mp2731_board_info, i2c_bus_num);
	return mp2731_dev ? 0 : -EINVAL;
}

static int __init mp2731_init(void)
{
	int ret;

	ret = mp2731_dev_init();
	if (ret)
		return ret;
	ret = i2c_add_driver(&mp2731_driver);
	if (ret)
		i2c_unregister_device(mp2731_dev);
	return ret;
}
module_init(mp2731_init);

static void __exit mp2731_exit(void)
{
	i2c_unregister_device(mp2731_dev);
	i2c_del_driver(&mp2731_driver);
}
module_exit(mp2731_exit);

MODULE_DESCRIPTION("MP2731 battery charger of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
