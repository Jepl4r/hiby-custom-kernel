// SPDX-License-Identifier: GPL-2.0
//
// axp2101 -- the X-Powers AXP2101 power management IC of the R1 and the
// R3 Pro II: sixteen regulators, the battery charger and fuel gauge, and
// power off.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock axp2101.sh passes them):
//
//   i2c_bus_num            I2C bus of the PMIC (address 0x34)
//   pmu_int_gpio           interrupt pin (PB23)
//   charge_voltage_limit   end of charge voltage, mV (4200)
//   charge_term_current    charge termination current, mA (100)
//   dcdc3_always_pwmmode   1 keeps DCDC3 in PWM mode while the system runs
//   dcdc1 ... cpusldo      one per regulator, as "regulator_name=NAME
//                          boot_on=enable|disable work_voltage=uV
//                          suspend_volage=uV"; the name is not used
//
// Loading creates the I2C device "axp2101_mfd". Its driver registers the
// interrupt chip and the cells of the PMIC; two of them have drivers here:
//
//   axp2101-regulator      the regulators dcdc1 to dcdc5, rtcldo, rtcldo1,
//                          aldo1 to aldo4, bldo1, bldo2, dldo1, dldo2 and
//                          cpusldo
//   axp2101-power-supply   the power supplies "axp_battery", "usb" and "ac"
//
// The class "axp" has two files: axp_reg (a register number to read, or
// 0xRRVV to write VV to register RR) and debug_mask.
//
// Every voltage this module sets is checked before it reaches the PMIC: a
// regulator parameter that is outside the rail's range, or that no selector
// of the rail gives exactly, is ignored; the charge voltage is one of the
// five the charger has; axp_reg refuses a reserved selector in a voltage
// register. The charge current is capped at 1000 mA and the input current
// limit at 2000 mA, the top of their documented ranges.
//
// Loaded after utils.ko, which provides i2c_register_device() and the pin and
// regulator parameters.

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern const struct kernel_param_ops param_pmu_ops;
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);

// What param_pmu_ops fills in.
struct pmu_param {
	const char *regulator_name;
	int boot_on;
	unsigned int work_voltage;	// uV, 0 for none
	unsigned int suspend_voltage;	// uV
};

// Registers.
#define AXP_STATUS1		0x00	// bit 5 VBUS good, bit 3 battery present
#define AXP_STATUS2		0x01	// bits 6..5 current direction, 2..0 charging state
#define AXP_CHIP_ID		0x03
#define AXP_COMM_CFG		0x10	// bit 3 PWROK low restarts, bit 2 16 s power key off, bit 0 power off
#define AXP_BATFET_CTRL		0x12
#define AXP_IIN_LIM		0x16	// input current limit, bits 2..0
#define AXP_GAUGE_RESET		0x17	// bit 2 resets the fuel gauge
#define AXP_MODULE_EN		0x18	// bit 1 charges the battery
#define AXP_LOW_WARN		0x1a	// battery low warning levels: 7..4 level 1 (5 + N %), 3..0 level 2
#define AXP_PWROFF_EN		0x22	// bit 2 powers off at die over-temperature
#define AXP_VOFF		0x24	// power off voltage, 2.6 V + N * 0.1 V
#define AXP_PWROK_SEQ		0x25	// bits 1..0 PWROK delay
#define AXP_IRQ_LEVEL		0x27	// bits 1..0 power key press that turns the PMIC on
#define AXP_ADC_EN		0x30	// 2 bytes
#define AXP_VBAT_H		0x34	// battery voltage, 14 bits, mV
#define AXP_TS_H		0x36	// TS pin voltage, 14 bits
#define AXP_TDIE_H		0x3c	// die temperature, 2 bytes
#define AXP_TS_CTRL		0x50
#define AXP_TS_VLTF_CHG		0x54	// TS cold threshold while charging
#define AXP_TS_VHTF_CHG		0x55	// TS hot threshold while charging
#define AXP_IPRECHG		0x61	// precharge current, 25 mA steps
#define AXP_ICC			0x62	// constant charge current, bits 4..0
#define AXP_ITERM		0x63	// bit 4 termination on, bits 3..0 current, 25 mA steps
#define AXP_CV			0x64	// end of charge voltage, bits 2..0
#define AXP_BAT_DET		0x68
#define AXP_CHGLED		0x69	// bit 0 CHGLED on, bits 2..1 its function
#define AXP_BTN_CHG_CV		0x6a	// coin cell charge voltage, bits 2..0
#define AXP_DCDC_EN		0x80
#define AXP_DCDC_PWM		0x81	// bit 4 keeps DCDC3 in PWM
#define AXP_DCDC1_V		0x82
#define AXP_DCDC2_V		0x83
#define AXP_DCDC3_V		0x84
#define AXP_DCDC4_V		0x85
#define AXP_DCDC5_V		0x86
#define AXP_LDO_EN0		0x90
#define AXP_LDO_EN1		0x91
#define AXP_ALDO1_V		0x92
#define AXP_ALDO2_V		0x93
#define AXP_ALDO3_V		0x94
#define AXP_ALDO4_V		0x95
#define AXP_BLDO1_V		0x96
#define AXP_BLDO2_V		0x97
#define AXP_CPUSLDO_V		0x98
#define AXP_DLDO1_V		0x99
#define AXP_DLDO2_V		0x9a
#define AXP_BROM		0xa1	// the gauge's battery model, written one byte at a time
#define AXP_GAUGE_CFG		0xa2	// bit 0 opens the model for writing, bit 4 runs it from SRAM
#define AXP_SOC			0xa4	// battery level, %
#define AXP_TTE			0xa6	// time to empty, 2 bytes, minutes
#define AXP_TTF			0xa8	// time to full, 2 bytes, minutes
#define AXP_IRQ_EN		0x40	// 3 registers, 1 enables
#define AXP_IRQ_STATUS		0x48	// 3 registers, 1 clears

#define STATUS1_VBUS_GOOD	BIT(5)
#define STATUS1_BAT_PRESENT	BIT(3)
#define MODULE_EN_CHARGE	BIT(1)
#define GAUGE_CFG_BROM		BIT(0)
#define GAUGE_CFG_SRAM		BIT(4)

#define AXP2101_ID		8	// the variant number of the vendor's AXP20x table

#define AXP2101_CHIP_A		0x47	// REG03h with bits 5..4 cleared
#define AXP2101_CHIP_B		0x4a
#define AXP2101_I2C_ADDR	0x34
#define AXP2101_SUPPLY_NAME	"axp2101-power-supply"

#define AXP_ICC_MAX_MA		1000	// REG62h 16; 17 to 31 are reserved
#define AXP_IIN_MAX_SEL		5	// 2000 mA; 6 and 7 are reserved

// The regulators, in the order of the parameters and of their descriptions.
enum {
	AXP_DCDC1, AXP_DCDC2, AXP_DCDC3, AXP_DCDC4, AXP_DCDC5,
	AXP_RTCLDO, AXP_RTCLDO1,
	AXP_ALDO1, AXP_ALDO2, AXP_ALDO3, AXP_ALDO4,
	AXP_BLDO1, AXP_BLDO2,
	AXP_DLDO1, AXP_DLDO2, AXP_CPUSLDO,
	AXP2101_NR_RAILS,
};

// --- the MFD core ------------------------------------------------------------

struct axp20x_dev {
	struct device *dev;
	int irq;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irqc;
	long variant;
	int nr_cells;
	struct mfd_cell *cells;
	const struct regmap_config *regmap_cfg;
	const struct regmap_irq_chip *regmap_irq_chip;
	void (*dts_parse)(struct axp20x_dev *axp20x);
};

// A device of the board's platform data.
struct axp_dev_info {
	int id;
	const char *name;
	void *platform_data;
	size_t pdata_size;
};

struct axp_platform_data {
	int nsupplies;
	struct axp_dev_info *supplies;
};

int axp20x_match_device(struct axp20x_dev *axp20x);
int axp20x_device_probe(struct axp20x_dev *axp20x, struct axp_platform_data *pdata);
int axp20x_device_remove(struct axp20x_dev *axp20x);

// The platform data of the charger cell.
struct axp_charger_pdata {
	int charge_current;	// mA, while charging
	int shutdown_current;	// mA, set at power off
	int voltage_limit;	// mV, end of charge
	int term_current;	// mA, charge termination
	int pwron_time;		// ms, power key press that turns the PMIC on
	int dcdc3_pwm;		// DCDC3 in PWM mode while the system runs
};

static int axp_debug_mask;
static int axp_reg_addr;
static struct axp20x_dev *axp20x_pm_power_off;
static bool axp_class_registered;

static const struct regmap_range axp2101_writeable_ranges[] = {
	regmap_reg_range(0x00, 0xff),
};

static const struct regmap_range axp2101_volatile_ranges[] = {
	regmap_reg_range(0x00, 0xff),
};

static const struct regmap_access_table axp2101_writeable_table = {
	.yes_ranges = axp2101_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(axp2101_writeable_ranges),
};

static const struct regmap_access_table axp2101_volatile_table = {
	.yes_ranges = axp2101_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(axp2101_volatile_ranges),
};

static const struct regmap_config axp2101_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.wr_table = &axp2101_writeable_table,
	.volatile_table = &axp2101_volatile_table,
	.max_register = 0xff,
	.use_single_rw = true,
	.cache_type = REGCACHE_RBTREE,
};

#define INIT_REGMAP_IRQ(_irq, _off, _mask)		\
	[_irq] = { .reg_offset = (_off), .mask = BIT(_mask) }

// IRQ numbers: register (0x40 + N / 8), bit N % 8.
static const struct regmap_irq axp2101_regmap_irqs[] = {
	INIT_REGMAP_IRQ(0, 0, 0), INIT_REGMAP_IRQ(1, 0, 1), INIT_REGMAP_IRQ(2, 0, 2),
	INIT_REGMAP_IRQ(3, 0, 3), INIT_REGMAP_IRQ(4, 0, 4), INIT_REGMAP_IRQ(5, 0, 5),
	INIT_REGMAP_IRQ(6, 0, 6), INIT_REGMAP_IRQ(7, 0, 7), INIT_REGMAP_IRQ(8, 1, 0),
	INIT_REGMAP_IRQ(9, 1, 1), INIT_REGMAP_IRQ(10, 1, 2), INIT_REGMAP_IRQ(11, 1, 3),
	INIT_REGMAP_IRQ(12, 1, 4), INIT_REGMAP_IRQ(13, 1, 5), INIT_REGMAP_IRQ(14, 1, 6),
	INIT_REGMAP_IRQ(15, 1, 7), INIT_REGMAP_IRQ(16, 2, 0), INIT_REGMAP_IRQ(17, 2, 1),
	INIT_REGMAP_IRQ(18, 2, 2), INIT_REGMAP_IRQ(19, 2, 3), INIT_REGMAP_IRQ(20, 2, 4),
	INIT_REGMAP_IRQ(21, 2, 5), INIT_REGMAP_IRQ(22, 2, 6), INIT_REGMAP_IRQ(23, 2, 7),
};

static const struct regmap_irq_chip axp2101_regmap_irq_chip = {
	.name = "axp2101_irq_chip",
	.status_base = AXP_IRQ_STATUS,
	.ack_base = AXP_IRQ_STATUS,
	.mask_base = AXP_IRQ_EN,
	.mask_invert = true,
	.init_ack_masked = true,
	.irqs = axp2101_regmap_irqs,
	.num_irqs = ARRAY_SIZE(axp2101_regmap_irqs),
	.num_regs = 3,
};

static struct resource axp2101_pek_resources[] = {
	DEFINE_RES_IRQ_NAMED(9, "PEK_DBF"),
	DEFINE_RES_IRQ_NAMED(8, "PEK_DBR"),
};

static struct resource axp2101_power_supply_resources[] = {
	DEFINE_RES_IRQ_NAMED(0, "bat untemp work"),
	DEFINE_RES_IRQ_NAMED(1, "bat ovtemp work"),
	DEFINE_RES_IRQ_NAMED(2, "bat untemp chg"),
	DEFINE_RES_IRQ_NAMED(3, "bat ovtemp chg"),
	DEFINE_RES_IRQ_NAMED(4, "CHG_NEWSOC"),
	DEFINE_RES_IRQ_NAMED(5, "CHG_GWDT"),
	DEFINE_RES_IRQ_NAMED(6, "low warning1"),
	DEFINE_RES_IRQ_NAMED(7, "low warning2"),
	DEFINE_RES_IRQ_NAMED(12, "bat out"),
	DEFINE_RES_IRQ_NAMED(13, "bat in"),
	DEFINE_RES_IRQ_NAMED(15, "usb in"),
	DEFINE_RES_IRQ_NAMED(14, "usb out"),
	DEFINE_RES_IRQ_NAMED(16, "CHG_BOVP"),
	DEFINE_RES_IRQ_NAMED(17, "CHG_CHGTE"),
	DEFINE_RES_IRQ_NAMED(18, "CHG_DOTL1"),
	DEFINE_RES_IRQ_NAMED(19, "charging"),
	DEFINE_RES_IRQ_NAMED(20, "charge over"),
	DEFINE_RES_IRQ_NAMED(21, "CHG_BOCP"),
	DEFINE_RES_IRQ_NAMED(22, "CHG_LDOOC"),
	DEFINE_RES_IRQ_NAMED(23, "CHG_WDEXP"),
};

// A device per regulator for the virtual consumer driver, which this kernel
// does not build: they stay without a driver.
#define AXP_VIRT_CONSUMER(_id, _name)				\
	{							\
		.name = "reg-virt-consumer",			\
		.id = (_id),					\
		.platform_data = _name,				\
		.pdata_size = sizeof(_name),			\
		.of_compatible = "xpower-vregulator," _name,	\
	}

static struct mfd_cell axp2101_cells[] = {
	{
		.name = "axp2101-pek",
		.of_compatible = "x-powers,axp2101-pek",
		.num_resources = ARRAY_SIZE(axp2101_pek_resources),
		.resources = axp2101_pek_resources,
	}, {
		.name = "axp2101-regulator",
	}, {
		.name = AXP2101_SUPPLY_NAME,
		.of_compatible = "x-powers,axp2101-power-supply",
		.num_resources = ARRAY_SIZE(axp2101_power_supply_resources),
		.resources = axp2101_power_supply_resources,
	}, {
		.name = "axp2xx-watchdog",
		.of_compatible = "x-powers,axp2xx-watchdog",
	},
	AXP_VIRT_CONSUMER(1, "dcdc1"),
	AXP_VIRT_CONSUMER(2, "dcdc2"),
	AXP_VIRT_CONSUMER(3, "dcdc3"),
	AXP_VIRT_CONSUMER(4, "dcdc4"),
	AXP_VIRT_CONSUMER(5, "dcdc5"),
	AXP_VIRT_CONSUMER(8, "aldo1"),
	AXP_VIRT_CONSUMER(9, "aldo2"),
	AXP_VIRT_CONSUMER(10, "aldo3"),
	AXP_VIRT_CONSUMER(11, "aldo4"),
	AXP_VIRT_CONSUMER(12, "bldo1"),
	AXP_VIRT_CONSUMER(13, "bldo2"),
	AXP_VIRT_CONSUMER(14, "dldo1"),
	AXP_VIRT_CONSUMER(15, "dldo2"),
};

static bool axp2101_reg_value_valid(unsigned int reg, unsigned int val);

static ssize_t axp_reg_show(struct class *class, struct class_attribute *attr, char *buf)
{
	unsigned int val = 0;

	regmap_read(axp20x_pm_power_off->regmap, axp_reg_addr, &val);
	return sprintf(buf, "REG[0x%x]=0x%x\n", axp_reg_addr, val);
}

// A number below 0x100 selects the register axp_reg reads; 0xRRVV writes VV
// to register RR and selects it.
static ssize_t axp_reg_store(struct class *class, struct class_attribute *attr, const char *buf,
			     size_t count)
{
	unsigned int reg;
	int val, ret;

	ret = kstrtoint(buf, 16, &val);
	if (ret)
		return ret;
	if (val < 0x100) {
		axp_reg_addr = val;
		return count;
	}

	reg = (val >> 8) & 0xff;
	axp_reg_addr = reg;
	if (!axp2101_reg_value_valid(reg, val & 0xff))
		return -EINVAL;
	regmap_write(axp20x_pm_power_off->regmap, reg, val & 0xff);
	return count;
}

static ssize_t debug_mask_show(struct class *class, struct class_attribute *attr, char *buf)
{
	ssize_t len;

	len = scnprintf(buf, PAGE_SIZE, "%s\n", "1: SPLY 2: REGU 4: INT 8: CHG");
	len += scnprintf(buf + len, PAGE_SIZE - len, "debug_mask=%d\n", axp_debug_mask);
	return len;
}

static ssize_t debug_mask_store(struct class *class, struct class_attribute *attr, const char *buf,
				size_t count)
{
	int val, ret;

	ret = kstrtoint(buf, 16, &val);
	if (ret)
		return ret;
	axp_debug_mask = val;
	return count;
}

static struct class_attribute axp_class_attrs[] = {
	__ATTR(axp_reg, 0644, axp_reg_show, axp_reg_store),
	__ATTR(debug_mask, 0644, debug_mask_show, debug_mask_store),
	__ATTR_NULL,
};

static struct class axp_class = {
	.name = "axp",
	.class_attrs = axp_class_attrs,
};

static void axp20x_power_off(void)
{
	regmap_update_bits(axp20x_pm_power_off->regmap, AXP_COMM_CFG, BIT(0), BIT(0));
	msleep(500);
}

// The device has no device tree node: the two properties keep their defaults.
static void axp2101_dts_parse(struct axp20x_dev *axp20x)
{
	struct device_node *np = axp20x->dev->of_node;
	struct regmap *map = axp20x->regmap;
	u32 hot_shutdown;

	regmap_update_bits(map, AXP_COMM_CFG, BIT(3),
			   of_find_property(np, "pmu_powerok_noreset", NULL) ? 0 : BIT(3));
	regmap_update_bits(map, AXP_COMM_CFG, BIT(2), BIT(2));
	regmap_update_bits(map, AXP_COMM_CFG, BIT(3), BIT(3));

	if (of_property_read_u32(np, "pmu_hot_shutdown", &hot_shutdown))
		hot_shutdown = 0;
	regmap_update_bits(map, AXP_PWROFF_EN, BIT(2), hot_shutdown ? BIT(2) : 0);
	regmap_write(map, AXP_VOFF, 0);
	regmap_update_bits(map, AXP_PWROK_SEQ, 0x03, 0);
}

int axp20x_match_device(struct axp20x_dev *axp20x)
{
	axp20x->variant = AXP2101_ID;
	axp20x->nr_cells = ARRAY_SIZE(axp2101_cells);
	axp20x->cells = axp2101_cells;
	axp20x->regmap_cfg = &axp2101_regmap_config;
	axp20x->regmap_irq_chip = &axp2101_regmap_irq_chip;
	axp20x->dts_parse = axp2101_dts_parse;
	dev_info(axp20x->dev, "AXP20x variant %s found\n", "AXP2101");
	return 0;
}
EXPORT_SYMBOL(axp20x_match_device);

int axp20x_device_probe(struct axp20x_dev *axp20x, struct axp_platform_data *pdata)
{
	int i, j, ret;

	if (axp20x->irq) {
		ret = regmap_add_irq_chip(axp20x->regmap, axp20x->irq,
					  IRQF_ONESHOT | IRQF_SHARED, -1,
					  axp20x->regmap_irq_chip, &axp20x->regmap_irqc);
		if (ret) {
			dev_err(axp20x->dev, "failed to add irq chip: %d\n", ret);
			return ret;
		}
	}

	if (axp20x->dts_parse)
		axp20x->dts_parse(axp20x);

	// The charger cell takes its platform data from the board's.
	for (i = 0; i < axp20x->nr_cells; i++)
		if (!strcmp(axp20x->cells[i].name, AXP2101_SUPPLY_NAME))
			break;
	for (j = 0; pdata && j < pdata->nsupplies; j++) {
		if (strcmp(pdata->supplies[j].name, AXP2101_SUPPLY_NAME))
			continue;
		if (i < axp20x->nr_cells) {
			axp20x->cells[i].platform_data = pdata->supplies[j].platform_data;
			axp20x->cells[i].pdata_size = pdata->supplies[j].pdata_size;
		}
		break;
	}

	ret = mfd_add_devices(axp20x->dev, 0, axp20x->cells, axp20x->nr_cells, NULL, 0, NULL);
	if (ret) {
		dev_err(axp20x->dev, "failed to add MFD devices: %d\n", ret);
		if (axp20x->irq)
			regmap_del_irq_chip(axp20x->irq, axp20x->regmap_irqc);
		return ret;
	}

	axp20x_pm_power_off = axp20x;
	ret = class_register(&axp_class);
	if (ret < 0)
		pr_err("axp2101: class axp not registered: %d\n", ret);
	else
		axp_class_registered = true;
	pm_power_off = axp20x_power_off;

	dev_info(axp20x->dev, "AXP20X driver loaded\n");
	return 0;
}
EXPORT_SYMBOL(axp20x_device_probe);

int axp20x_device_remove(struct axp20x_dev *axp20x)
{
	if (axp20x == axp20x_pm_power_off) {
		if (axp_class_registered) {
			class_unregister(&axp_class);
			axp_class_registered = false;
		}
		axp20x_pm_power_off = NULL;
		pm_power_off = NULL;
	}

	mfd_remove_devices(axp20x->dev);
	if (axp20x->irq)
		regmap_del_irq_chip(axp20x->irq, axp20x->regmap_irqc);
	return 0;
}
EXPORT_SYMBOL(axp20x_device_remove);

static int axp20x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct axp20x_dev *axp20x;
	int ret;

	axp20x = devm_kzalloc(&client->dev, sizeof(*axp20x), GFP_KERNEL);
	if (!axp20x)
		return -ENOMEM;

	axp20x->dev = &client->dev;
	axp20x->irq = client->irq;
	i2c_set_clientdata(client, axp20x);

	ret = axp20x_match_device(axp20x);
	if (ret)
		return ret;

	axp20x->regmap = devm_regmap_init_i2c(client, axp20x->regmap_cfg);
	if (IS_ERR(axp20x->regmap)) {
		ret = PTR_ERR(axp20x->regmap);
		dev_err(&client->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	return axp20x_device_probe(axp20x, dev_get_platdata(&client->dev));
}

static int axp20x_i2c_remove(struct i2c_client *client)
{
	return axp20x_device_remove(i2c_get_clientdata(client));
}

static const struct i2c_device_id axp20x_i2c_id[] = {
	{ "axp2101_mfd", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, axp20x_i2c_id);

static const struct of_device_id axp20x_i2c_of_match[] = {
	{ .compatible = "x-powers,axp2101", .data = (void *)AXP2101_ID },
	{ }
};
MODULE_DEVICE_TABLE(of, axp20x_i2c_of_match);

static struct i2c_driver axp2101_i2c_driver = {
	.driver = {
		.name = "axp20x-i2c",
		.of_match_table = of_match_ptr(axp20x_i2c_of_match),
	},
	.probe = axp20x_i2c_probe,
	.remove = axp20x_i2c_remove,
	.id_table = axp20x_i2c_id,
};

// --- the regulators ---------------------------------------------------------

// The ranges the regulators may be set to without a parameter.
#define AXP_RAIL(_name, _min, _max, _ops, _standby_uV, _standby_on)	\
	{								\
		.constraints = {					\
			.name = (_name),				\
			.min_uV = (_min),				\
			.max_uV = (_max),				\
			.valid_ops_mask = (_ops),			\
			.state_standby = {				\
				.uV = (_standby_uV),			\
				.enabled = (_standby_on),		\
			},						\
			.initial_state = PM_SUSPEND_STANDBY,		\
		},							\
	}

#define OPS_VOLTAGE	(REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS)
#define OPS_STATUS	REGULATOR_CHANGE_STATUS

static struct regulator_init_data axp_regl_init_data[AXP2101_NR_RAILS] = {
	[AXP_DCDC1] = AXP_RAIL("dcdc1", 1500000, 3400000, OPS_VOLTAGE, 3300000, 1),
	[AXP_DCDC2] = AXP_RAIL("dcdc2", 500000, 1540000, OPS_VOLTAGE, 1200000, 1),
	[AXP_DCDC3] = AXP_RAIL("dcdc3", 500000, 3400000, OPS_VOLTAGE, 1800000, 1),
	[AXP_DCDC4] = AXP_RAIL("dcdc4", 500000, 1840000, OPS_VOLTAGE, 1800000, 1),
	[AXP_DCDC5] = AXP_RAIL("dcdc5", 1200000, 3700000, OPS_VOLTAGE, 1400000, 1),
	[AXP_RTCLDO] = AXP_RAIL("rtcldo", 1800000, 1800000, OPS_STATUS, 1800000, 1),
	[AXP_RTCLDO1] = AXP_RAIL("rtcldo1", 1800000, 1800000, OPS_STATUS, 1800000, 1),
	[AXP_ALDO1] = AXP_RAIL("aldo1", 500000, 3500000, OPS_VOLTAGE, 2500000, 1),
	[AXP_ALDO2] = AXP_RAIL("aldo2", 500000, 3500000, OPS_VOLTAGE, 3300000, 1),
	[AXP_ALDO3] = AXP_RAIL("aldo3", 500000, 3500000, OPS_VOLTAGE, 3300000, 1),
	[AXP_ALDO4] = AXP_RAIL("aldo4", 500000, 3500000, OPS_VOLTAGE, 3300000, 1),
	[AXP_BLDO1] = AXP_RAIL("bldo1", 500000, 3500000, OPS_VOLTAGE, 2800000, 0),
	[AXP_BLDO2] = AXP_RAIL("bldo2", 500000, 3500000, OPS_VOLTAGE, 3300000, 0),
	[AXP_DLDO1] = AXP_RAIL("dldo1", 500000, 3300000, OPS_VOLTAGE, 1800000, 1),
	[AXP_DLDO2] = AXP_RAIL("dldo2", 500000, 1400000, OPS_VOLTAGE, 1250000, 1),
	[AXP_CPUSLDO] = AXP_RAIL("cpusldo", 500000, 1400000, OPS_VOLTAGE, 1200000, 1),
};

static struct regulator_init_data *axp_regulators_init_data;
static int axp_nregulators_init_data;

static int axp2101_set_voltage_time_sel(struct regulator_dev *rdev, unsigned int old_selector,
					unsigned int new_selector)
{
	return 0;
}

static const struct regulator_ops axp20x_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_time_sel = axp2101_set_voltage_time_sel,
};

static const struct regulator_ops axp20x_ops_range = {
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_time_sel = axp2101_set_voltage_time_sel,
};

static const struct regulator_ops axp20x_ops_fixed = {
	.list_voltage = regulator_list_voltage_linear,
};

static const struct regulator_linear_range axp2101_dcdc2_ranges[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0x46, 10000),
	REGULATOR_LINEAR_RANGE(1220000, 0x47, 0x57, 20000),
};

static const struct regulator_linear_range axp2101_dcdc3_ranges[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0x46, 10000),
	REGULATOR_LINEAR_RANGE(1220000, 0x47, 0x57, 20000),
	REGULATOR_LINEAR_RANGE(1600000, 0x58, 0x6a, 100000),
};

static const struct regulator_linear_range axp2101_dcdc4_ranges[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0x46, 10000),
	REGULATOR_LINEAR_RANGE(1220000, 0x47, 0x66, 20000),
};

static const struct regulator_linear_range axp2101_dcdc5_ranges[] = {
	REGULATOR_LINEAR_RANGE(1400000, 0x00, 0x17, 100000),
	REGULATOR_LINEAR_RANGE(1200000, 0x19, 0x19, 0),
};

#define AXP_DESC_COMMON(_id, _name)				\
	.name = (_name),					\
	.of_match = of_match_ptr(_name),			\
	.regulators_node = of_match_ptr("regulators"),		\
	.type = REGULATOR_VOLTAGE,				\
	.id = (_id),						\
	.owner = THIS_MODULE

#define AXP_DESC(_id, _name, _min, _step, _n, _vreg, _vmask, _ereg, _emask)	\
	[_id] = {							\
		AXP_DESC_COMMON(_id, _name),				\
		.n_voltages = (_n),					\
		.min_uV = (_min),					\
		.uV_step = (_step),					\
		.vsel_reg = (_vreg),					\
		.vsel_mask = (_vmask),					\
		.enable_reg = (_ereg),					\
		.enable_mask = (_emask),				\
		.ops = &axp20x_ops,					\
	}

#define AXP_DESC_RANGES(_id, _name, _ranges, _n, _vreg, _vmask, _ereg, _emask)	\
	[_id] = {							\
		AXP_DESC_COMMON(_id, _name),				\
		.n_voltages = (_n),					\
		.vsel_reg = (_vreg),					\
		.vsel_mask = (_vmask),					\
		.enable_reg = (_ereg),					\
		.enable_mask = (_emask),				\
		.linear_ranges = (_ranges),				\
		.n_linear_ranges = ARRAY_SIZE(_ranges),			\
		.ops = &axp20x_ops_range,				\
	}

#define AXP_DESC_FIXED(_id, _name, _volt)				\
	[_id] = {							\
		AXP_DESC_COMMON(_id, _name),				\
		.n_voltages = 1,					\
		.min_uV = (_volt),					\
		.ops = &axp20x_ops_fixed,				\
	}

// dldo1 stops at 3.3 V, selector 28; 29 to 31 are reserved.
static const struct regulator_desc axp2101_regulators[AXP2101_NR_RAILS] = {
	AXP_DESC(AXP_DCDC1, "dcdc1", 1500000, 100000, 20, AXP_DCDC1_V, 0x1f, AXP_DCDC_EN, BIT(0)),
	AXP_DESC_RANGES(AXP_DCDC2, "dcdc2", axp2101_dcdc2_ranges, 0x58, AXP_DCDC2_V, 0x7f, AXP_DCDC_EN,
			BIT(1)),
	AXP_DESC_RANGES(AXP_DCDC3, "dcdc3", axp2101_dcdc3_ranges, 0x6b, AXP_DCDC3_V, 0x7f, AXP_DCDC_EN,
			BIT(2)),
	AXP_DESC_RANGES(AXP_DCDC4, "dcdc4", axp2101_dcdc4_ranges, 0x67, AXP_DCDC4_V, 0x7f, AXP_DCDC_EN,
			BIT(3)),
	AXP_DESC_RANGES(AXP_DCDC5, "dcdc5", axp2101_dcdc5_ranges, 0x19, AXP_DCDC5_V, 0x1f, AXP_DCDC_EN,
			BIT(4)),
	AXP_DESC_FIXED(AXP_RTCLDO, "rtcldo", 1800000),
	AXP_DESC_FIXED(AXP_RTCLDO1, "rtcldo1", 1800000),
	AXP_DESC(AXP_ALDO1, "aldo1", 500000, 100000, 31, AXP_ALDO1_V, 0x1f, AXP_LDO_EN0, BIT(0)),
	AXP_DESC(AXP_ALDO2, "aldo2", 500000, 100000, 31, AXP_ALDO2_V, 0x1f, AXP_LDO_EN0, BIT(1)),
	AXP_DESC(AXP_ALDO3, "aldo3", 500000, 100000, 31, AXP_ALDO3_V, 0x1f, AXP_LDO_EN0, BIT(2)),
	AXP_DESC(AXP_ALDO4, "aldo4", 500000, 100000, 31, AXP_ALDO4_V, 0x1f, AXP_LDO_EN0, BIT(3)),
	AXP_DESC(AXP_BLDO1, "bldo1", 500000, 100000, 31, AXP_BLDO1_V, 0x1f, AXP_LDO_EN0, BIT(4)),
	AXP_DESC(AXP_BLDO2, "bldo2", 500000, 100000, 31, AXP_BLDO2_V, 0x1f, AXP_LDO_EN0, BIT(5)),
	AXP_DESC(AXP_DLDO1, "dldo1", 500000, 100000, 29, AXP_DLDO1_V, 0x1f, AXP_LDO_EN0, BIT(7)),
	AXP_DESC(AXP_DLDO2, "dldo2", 500000, 50000, 19, AXP_DLDO2_V, 0x1f, AXP_LDO_EN1, BIT(0)),
	AXP_DESC(AXP_CPUSLDO, "cpusldo", 500000, 50000, 19, AXP_CPUSLDO_V, 0x1f, AXP_LDO_EN0, BIT(6)),
};

// Whether a selector is one the regulator has.
static bool axp2101_selector_valid(const struct regulator_desc *desc, unsigned int sel)
{
	int i;

	if (sel >= desc->n_voltages)
		return false;
	if (!desc->n_linear_ranges)
		return true;
	for (i = 0; i < desc->n_linear_ranges; i++)
		if (sel >= desc->linear_ranges[i].min_sel && sel <= desc->linear_ranges[i].max_sel)
			return true;
	return false;
}

// Whether VAL may be written to REG through axp_reg: a voltage register only
// takes a selector its regulator has, the charge voltage one of its five
// values.
static bool axp2101_reg_value_valid(unsigned int reg, unsigned int val)
{
	int i;

	if (reg == AXP_CV) {
		unsigned int cv = val & 0x07;

		return cv >= 1 && cv <= 5;
	}
	for (i = 0; i < AXP2101_NR_RAILS; i++) {
		const struct regulator_desc *desc = &axp2101_regulators[i];

		if (desc->ops == &axp20x_ops_fixed || desc->vsel_reg != reg)
			continue;
		return axp2101_selector_valid(desc, val & desc->vsel_mask);
	}
	return true;
}

static int axp2101_regulator_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = {
		.dev = pdev->dev.parent,
		.regmap = axp20x->regmap,
		.driver_data = axp20x,
	};
	struct regulator_dev *rdev;
	unsigned int i;

	if (axp20x->variant != AXP2101_ID) {
		dev_err(&pdev->dev, "Unsupported AXP variant: %ld\n", axp20x->variant);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(axp2101_regulators); i++) {
		if ((int)i < axp_nregulators_init_data)
			config.init_data = &axp_regulators_init_data[i];
		rdev = devm_regulator_register(&pdev->dev, &axp2101_regulators[i], &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "Failed to register %s\n", axp2101_regulators[i].name);
			return PTR_ERR(rdev);
		}
	}
	return 0;
}

static const struct of_device_id axp_regulator_id_tab[] = {
	{ .compatible = "x-powers,axp2202-regulator" },
	{ }
};

static struct platform_driver axp2101_regulator_driver = {
	.probe = axp2101_regulator_probe,
	.driver = {
		.name = "axp2101-regulator",
		.of_match_table = axp_regulator_id_tab,
	},
};

// --- the charger and fuel gauge ---------------------------------------------

// Fixed values the properties report.
#define BAT_ENERGY_FULL_DESIGN	1600
#define BAT_VOLTAGE_MAX_DESIGN	4350000
#define BAT_VOLTAGE_MIN_DESIGN	3400000
#define BAT_CURRENT_NOW		200000
#define BAT_TEMP_ALERT_MIN	85
#define AC_VOLTAGE_NOW		4600
#define AC_CURRENT_NOW		1200
#define USB_VOLTAGE_NOW		4400
#define LOW_WARN1		15	// percent
#define LOW_WARN2		0

#define PLUG_SETTLE		5	// seconds on VBUS before the charge current is set

// The battery check: what the work does on its next run.
enum {
	CHECK_START,		// requeue
	CHECK_ENABLE,		// reset the gauge, enable charging
	CHECK_RUN,		// follow the battery
};

struct axp210x_info {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;		// charge_blocked and the enable bit
	const u8 *model;		// the gauge's battery model
	unsigned int model_len;
	bool chip_b;			// REG03h 0x4a: battery presence from REG00h

	int charge_current;		// mA, constant_charge_current_max
	int shutdown_current;		// mA
	int voltage_limit;		// mV
	int input_limit;		// mA, charge_control_limit_max
	bool charge_blocked;		// charge_control_limit_max = 100
	bool charging_off;		// "charger" = 0

	u16 vbat;			// mV, the last voltage in range
	u8 soc;				// percent
	u8 warn_level;			// REG1Ah bits 7..4

	struct task_struct *thread;
	u8 usb_online;
	u8 plug_count;			// seconds on VBUS, up to PLUG_SETTLE

	struct power_supply *bat;
	struct power_supply *ac;
	struct power_supply *usb;

	struct delayed_work work;
	int check;			// CHECK_*
	int present;			// a battery is there
	int full;			// charging done for more than a second
	int gauge_ok;			// the gauge has run with its model
};

static struct axp210x_info *axp210x_info;
static int dcdc3_pwm_in_run;

static const u8 axp2101_model[128] = {
	0x01, 0xf5, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x1e, 0x32, 0x01, 0x14, 0x04, 0xd8, 0x04,
	0x74, 0xfd, 0x58, 0x0b, 0xb3, 0x10, 0x3f, 0xfb, 0xc8, 0x00, 0xbe, 0x03, 0x4e, 0x06, 0x3f, 0x06,
	0x02, 0x0a, 0xd3, 0x0f, 0x74, 0x0f, 0x31, 0x09, 0xe5, 0x0e, 0xb9, 0x0e, 0xc0, 0x04, 0xbe, 0x04,
	0xbb, 0x09, 0xb4, 0x0e, 0xa0, 0x0e, 0x92, 0x09, 0x79, 0x0e, 0x4c, 0x0e, 0x27, 0x03, 0xfc, 0x03,
	0xd5, 0x08, 0xbc, 0x0d, 0x9c, 0x0d, 0x55, 0x06, 0xb8, 0x2e, 0x24, 0x2e, 0x2e, 0x24, 0x2e, 0x24,
	0xc5, 0x98, 0x7e, 0x66, 0x4e, 0x44, 0x38, 0x1a, 0x12, 0x0a, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6,
	0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6,
	0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6,
};

// The TS pin as a resistance, ohm, to tenths of a degree Celsius.
static const struct {
	int ohm;
	int temp;
} ohm_to_temp_table[] = {
	{ 74353, -200 }, { 57626, -150 }, { 45067, -100 }, { 32550, -50 }, { 28267, 0 },
	{ 22649, 50 }, { 18280, 100 }, { 14855, 150 }, { 12150, 200 }, { 10000, 250 },
	{ 8278, 300 }, { 6891, 350 }, { 5768, 400 }, { 4852, 450 }, { 4101, 500 },
	{ 3482, 550 }, { 2970, 600 }, { 0, 650 },
};

// Input current limits, mA, of REG16h 0 to 5.
static const unsigned int iin_lim_tbl[] = { 100, 500, 900, 1000, 1500, 2000 };

static enum power_supply_property axp2101_ac_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static enum power_supply_property axp2101_usb_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
};

static enum power_supply_property axp2101_bat_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static char *supply_list[] = {
	"axp_battery",
};

int charger_plugged = 1;
EXPORT_SYMBOL_GPL(charger_plugged);

// LEN registers from REG on; stops at the first error and returns it.
static int axp_read(struct axp210x_info *info, u8 reg, u8 *buf, u8 len)
{
	unsigned int val;
	int i, ret = 0;

	if (!info->regmap)
		return -ENODEV;
	for (i = 0; i < len; i++) {
		ret = regmap_read(info->regmap, reg + i, &val);
		if (ret < 0)
			return ret;
		buf[i] = val;
	}
	return ret;
}

// LEN registers from REG on; stops at the first error, and returns 0.
static int axp_write(struct axp210x_info *info, u8 reg, const u8 *buf, u8 len)
{
	int i;

	if (!info->regmap)
		return -ENODEV;
	for (i = 0; i < len; i++)
		if (regmap_write(info->regmap, reg + i, buf[i]) < 0)
			break;
	return 0;
}

// REG62h: 25 mA steps up to 200 mA, then 100 mA steps.
static void axp_set_charge_current(struct axp210x_info *info, int ma)
{
	unsigned int sel;

	ma = clamp(ma, 0, AXP_ICC_MAX_MA);
	sel = ma <= 200 ? ma / 25 : (ma - 200) / 100 + 8;
	regmap_update_bits(info->regmap, AXP_ICC, 0x1f, sel);
}

static void axp_set_input_current_limit(struct axp210x_info *info, unsigned int ma)
{
	unsigned int sel;

	for (sel = 0; sel < ARRAY_SIZE(iin_lim_tbl); sel++)
		if (ma < iin_lim_tbl[sel])
			break;
	sel = sel ? sel - 1 : 0;
	regmap_update_bits(info->regmap, AXP_IIN_LIM, 0x07, min_t(unsigned int, sel, AXP_IIN_MAX_SEL));
}

// REG64h for an end of charge voltage in mV: the step at or below it, 4.10
// to 4.40 V, and 4.00 V below 4100 mV. From 4600 mV up it is 4.20 V.
static unsigned int axp_cv_sel(unsigned int mv)
{
	if (mv < 4100)
		return 1;
	if (mv < 4200)
		return 2;
	if (mv < 4350)
		return 3;
	if (mv < 4400)
		return 4;
	if (mv < 4600)
		return 5;
	pr_warn("axp2101: charge_voltage_limit %u mV is out of the charger's range, charging to 4200 mV\n",
		mv);
	return 3;
}

// Writes the battery charge enable bit from charge_blocked, by a read and a
// write of REG18h. Called with info->lock held.
static int axp_charge_enable_write(struct axp210x_info *info)
{
	u8 val;
	int ret;

	ret = axp_read(info, AXP_MODULE_EN, &val, 1);
	if (ret < 0)
		return ret;
	if (info->charge_blocked)
		val &= ~MODULE_EN_CHARGE;
	else
		val |= MODULE_EN_CHARGE;
	return regmap_write(info->regmap, AXP_MODULE_EN, val);
}

static int axp210x_reset_gauge(struct axp210x_info *info)
{
	u8 val;
	int ret;

	ret = axp_read(info, AXP_GAUGE_RESET, &val, 1);
	if (ret < 0)
		return ret;
	val |= BIT(2);
	ret = axp_write(info, AXP_GAUGE_RESET, &val, 1);
	if (ret < 0)
		return ret;
	val &= ~BIT(2);
	ret = axp_write(info, AXP_GAUGE_RESET, &val, 1);
	return ret > 0 ? 0 : ret;
}

// Sets the TS pin up as the battery temperature input (REG50h 0x0e), reads
// the gauge's state, and turns CHGLED off.
static int axp210x_reg_update(struct axp210x_info *info)
{
	u8 buf[2] = { 0x0e, 0 };
	int ret;

	ret = axp_write(info, AXP_TS_CTRL, buf, 1);
	if (ret < 0)
		return ret;
	ret = axp_read(info, AXP_GAUGE_CFG, buf, 1);
	if (ret < 0)
		return ret;
	ret = axp_read(info, AXP_VBAT_H, buf, 2);
	if (ret < 0)
		return ret;
	info->vbat = ((buf[0] & 0x3f) << 8) + buf[1];
	ret = axp_read(info, AXP_TDIE_H, buf, 2);
	if (ret < 0)
		return ret;
	ret = axp_read(info, AXP_SOC, buf, 1);
	info->soc = buf[0];
	if (ret < 0)
		return ret;
	ret = axp_read(info, AXP_TTE, buf, 2);
	if (ret < 0)
		return ret;
	ret = axp_read(info, AXP_TTF, buf, 2);
	if (ret < 0)
		return ret;
	ret = axp_read(info, AXP_LOW_WARN, buf, 1);
	info->warn_level = buf[0] >> 4;
	if (ret < 0)
		return ret;

	regmap_update_bits(info->regmap, AXP_CHGLED, BIT(0), 0);
	return 0;
}

// Loads the battery model into the gauge, with charging off; the battery
// check turns it on again.
static int axp210x_model_update(struct axp210x_info *info)
{
	unsigned int i;
	u8 val;
	int ret;

	mutex_lock(&info->lock);
	ret = axp_read(info, AXP_MODULE_EN, &val, 1);
	if (ret >= 0) {
		val &= ~MODULE_EN_CHARGE;
		ret = axp_write(info, AXP_MODULE_EN, &val, 1);
	}
	mutex_unlock(&info->lock);
	if (ret < 0)
		goto err;
	msleep(1000);

	ret = axp210x_reset_gauge(info);
	if (ret < 0)
		goto err;

	ret = axp_read(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;
	val &= ~GAUGE_CFG_BROM;
	ret = axp_write(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;
	val |= GAUGE_CFG_BROM;
	ret = axp_write(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;

	for (i = 0; i < info->model_len; i++)
		ret = axp_write(info, AXP_BROM, &info->model[i], 1);
	if (ret < 0)
		goto err;

	ret = axp_read(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;
	val &= ~GAUGE_CFG_BROM;
	ret = axp_write(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;
	val |= GAUGE_CFG_BROM;
	ret = axp_write(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;

	ret = axp_read(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;
	val = (val & ~GAUGE_CFG_BROM) | GAUGE_CFG_SRAM;
	axp_write(info, AXP_GAUGE_CFG, &val, 1);

	ret = axp_read(info, AXP_GAUGE_CFG, &val, 1);
	if (ret < 0)
		goto err;
	ret = axp210x_reset_gauge(info);
	if (ret >= 0)
		return 0;

err:
	axp_read(info, AXP_GAUGE_CFG, &val, 1);
	val &= ~GAUGE_CFG_BROM;
	axp_write(info, AXP_GAUGE_CFG, &val, 1);
	axp210x_reset_gauge(info);
	return ret;
}

// Sets the gauge and the ADC up; loads the battery model when the gauge does
// not run one.
static int axp210x_init_chip(struct axp210x_info *info)
{
	static const u8 adc_en[2] = { 0x02, 0x08 };
	u8 val;
	int ret;

	if (!info)
		return -ENODEV;

	ret = axp210x_reg_update(info);
	if (ret < 0)
		return ret;

	ret = axp_read(info, AXP_GAUGE_CFG, &val, 1);
	if (ret >= 0 && (val & GAUGE_CFG_SRAM)) {
		info->gauge_ok = 1;
	} else {
		axp_read(info, AXP_GAUGE_CFG, &val, 1);
		val &= ~GAUGE_CFG_BROM;
		axp_write(info, AXP_GAUGE_CFG, &val, 1);
		axp210x_reset_gauge(info);
		info->check = CHECK_START;
		ret = axp210x_model_update(info);
		if (ret < 0)
			return ret;
	}

	ret = axp210x_reg_update(info);
	if (ret < 0)
		return ret;

	ret = axp_write(info, AXP_ADC_EN, adc_en, 2);
	if (ret >= 0) {
		val = 0;
		ret = axp_write(info, AXP_TS_H, &val, 1);
	}
	if (ret >= 0) {
		val = 0x04;
		ret = axp_write(info, AXP_BAT_DET, &val, 1);
	}
	if (ret >= 0) {
		val = 0x2c;
		ret = axp_write(info, AXP_TS_VLTF_CHG, &val, 1);
	}
	if (ret >= 0) {
		val = 0x66;
		ret = axp_write(info, AXP_TS_VHTF_CHG, &val, 1);
	}
	if (ret >= 0)
		regmap_write(info->regmap, AXP_BATFET_CTRL, 0);
	return ret;
}

// Sets *full once the charger has reported done for more than a second.
static void battery_set_full(int *full)
{
	static ktime_t last;

	if (ktime_to_ms(ktime_sub(ktime_get(), last)) > 1000)
		*full = 1;
	last = ktime_get();
}

// The median of three readings of a 2-byte value; HI_MASK is applied to the
// first byte.
static int axp_read_median(struct axp210x_info *info, u8 reg, u8 hi_mask, u16 *out)
{
	u16 v[3];
	u8 buf[2];
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = axp_read(info, reg, buf, 2);
		if (ret < 0)
			return ret;
		v[i] = ((buf[0] & hi_mask) << 8) | buf[1];
	}
	if (v[1] < v[0])
		swap(v[0], v[1]);
	if (v[2] < v[1])
		v[1] = v[2];
	if (v[1] < v[0])
		v[1] = v[0];
	*out = v[1];
	return 0;
}

static int axp_ts_to_temp(int ohm)
{
	unsigned int i;

	if (ohm >= ohm_to_temp_table[0].ohm)
		return -250;
	for (i = 0; i < ARRAY_SIZE(ohm_to_temp_table) - 1; i++) {
		int o0 = ohm_to_temp_table[i].ohm, o1 = ohm_to_temp_table[i + 1].ohm;
		int t0 = ohm_to_temp_table[i].temp, t1 = ohm_to_temp_table[i + 1].temp;

		if (ohm >= o1)
			return (t1 - t0) * (o0 - ohm) / (o0 - o1) + t0;
	}
	return 0;
}

static int axp210x_bat_get_property(struct power_supply *psy, enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct axp210x_info *info = axp210x_info;
	u8 buf[2] = { 0, 0 };
	u16 v;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!info->present) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			return 0;
		}
		ret = axp_read(info, AXP_STATUS2, buf, 1);
		if (ret)
			return ret;
		switch (buf[0] & 0x07) {
		case 0:		// trickle
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case 1:		// precharge
		case 2:		// constant current
		case 3:		// constant voltage
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 4:		// done
			val->intval = info->full ? POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 5:		// not charging: by the current direction
			val->intval = ((buf[0] >> 5) & 0x03) == 1 ? POWER_SUPPLY_STATUS_CHARGING :
								     POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = info->present;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = BAT_VOLTAGE_MAX_DESIGN;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = BAT_VOLTAGE_MIN_DESIGN;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = BAT_CURRENT_NOW;
		return 0;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = BAT_ENERGY_FULL_DESIGN;
		return 0;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		val->intval = BAT_TEMP_ALERT_MIN;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = psy->desc->name;
		return 0;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "X-POWERS";
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		// A reading outside 2 to 8 V gives the last one inside.
		ret = axp_read_median(info, AXP_VBAT_H, 0x3f, &v);
		if (ret < 0)
			return ret;
		if (v >= 2000 && v <= 8000)
			info->vbat = v;
		val->intval = info->vbat * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!info->present || !info->gauge_ok) {
			val->intval = -1;
			return 0;
		}
		axp_read(info, AXP_GAUGE_CFG, buf, 1);
		if (!(buf[0] & GAUGE_CFG_SRAM)) {
			printk(KERN_WARNING "axp2101: something reset the gauge\n\n");
			axp210x_init_chip(info);
			msleep(500);
		}
		ret = axp_read(info, AXP_SOC, buf, 1);
		if (ret < 0)
			return ret;
		info->soc = min_t(u8, buf[0], 100);
		val->intval = info->soc;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = axp_read(info, AXP_TS_H, buf, 2);
		if (ret < 0)
			return ret;
		val->intval = axp_ts_to_temp((((buf[0] & 0x3f) << 8) + buf[1]) * 10);
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		// The register as read: level 1 in bits 7..4, level 2 in 3..0.
		ret = axp_read(info, AXP_LOW_WARN, buf, 1);
		if (ret < 0)
			return ret;
		info->warn_level = buf[0] >> 4;
		val->intval = buf[0];
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (info->soc == 100)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (info->soc > 80)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else if (info->soc > info->warn_level)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else if (info->soc < info->warn_level)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (info->soc < 2)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		return 0;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = axp_read_median(info, AXP_TTE, 0xff, &v);
		if (ret < 0)
			return ret;
		val->intval = v * 60;
		return 0;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = axp_read_median(info, AXP_TTF, 0xff, &v);
		if (ret < 0)
			return ret;
		val->intval = v * 60;
		return 0;
	default:
		return -EINVAL;
	}
}

// CAPACITY_ALERT_MIN: the level 1 warning, 5 + N %, N up to 20.
static int axp210x_bat_set_property(struct power_supply *psy, enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct axp210x_info *info = axp210x_info;
	u8 level = val->intval, reg = 0;
	int ret;

	if (psp != POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN || level > 20)
		return -EINVAL;

	axp_read(info, AXP_LOW_WARN, &reg, 1);
	reg = (reg & 0x0f) | (level << 4);
	ret = axp_write(info, AXP_LOW_WARN, &reg, 1);
	return ret > 0 ? 0 : ret;
}

// No battery property is writeable from sysfs.
static int axp210x_bat_property_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	return 0;
}

static int axp210x_ac_get_property(struct power_supply *psy, enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct axp210x_info *info = axp210x_info;

	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = psy->desc->name;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->usb_online;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = AC_VOLTAGE_NOW;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = AC_CURRENT_NOW;
		return 0;
	default:
		return -EINVAL;
	}
}

static int axp210x_usb_get_property(struct power_supply *psy, enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct axp210x_info *info = axp210x_info;

	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = psy->desc->name;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->usb_online;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = USB_VOLTAGE_NOW;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = info->charge_current;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = info->input_limit;
		return 0;
	default:
		return -EINVAL;
	}
}

// CONSTANT_CHARGE_CURRENT_MAX: the charge current, mA. Up to 500 mA it is set
// at once, above it after PLUG_SETTLE seconds on VBUS.
// CHARGE_CONTROL_LIMIT_MAX: the input current limit, mA, 0 for 2000; 100
// also stops charging, any other value lets it run.
static int axp210x_usb_set_property(struct power_supply *psy, enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct axp210x_info *info = axp210x_info;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		info->charge_current = val->intval;
		if ((unsigned int)val->intval > 500) {
			info->plug_count = 0;
			return 0;
		}
		info->plug_count = PLUG_SETTLE;
		axp_set_charge_current(info, val->intval);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		info->input_limit = val->intval ? val->intval : 2000;
		mutex_lock(&info->lock);
		info->charge_blocked = val->intval == 100;
		regmap_update_bits(info->regmap, AXP_MODULE_EN, MODULE_EN_CHARGE,
				   info->charge_blocked ? 0 : MODULE_EN_CHARGE);
		mutex_unlock(&info->lock);
		axp_set_input_current_limit(info, info->input_limit);
		return 0;
	default:
		return -EINVAL;
	}
}

static int axp210x_usb_property_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX;
}

// "charger": 0 sets the charge current to 0 mA, any other value back to
// constant_charge_current_max.
static ssize_t charger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !axp210x_info->charging_off);
}

static ssize_t charger_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t count)
{
	struct axp210x_info *info = axp210x_info;
	long val;
	int ret;

	ret = kstrtol(buf, 0, &val);
	if (ret)
		return ret;
	info->charging_off = !val;
	axp_set_charge_current(info, val ? info->charge_current : 0);
	return count;
}
static DEVICE_ATTR(charger, 0644, charger_show, charger_store);

// Turns charging on, whatever charge_control_limit_max had it at, at shutdown
// and removal; the monitor thread is stopped by then.
static void axp_charge_release(struct axp210x_info *info)
{
	mutex_lock(&info->lock);
	info->charge_blocked = false;
	axp_charge_enable_write(info);
	mutex_unlock(&info->lock);
}

// Once a second (the first time after 5 s): follows VBUS for the "usb" and
// "ac" supplies, and sets the charge current PLUG_SETTLE seconds after it
// comes.
static int axp2101_monitor(void *data)
{
	struct axp210x_info *info = data;
	long timeout = 500;
	u8 status, online;

	set_freezable();
	while (!kthread_should_stop()) {
		schedule_timeout_interruptible(timeout);
		try_to_freeze();
		timeout = 100;

		if (axp_read(info, AXP_STATUS1, &status, 1) < 0)
			continue;

		online = !!(status & STATUS1_VBUS_GOOD);
		if (online != info->usb_online) {
			info->usb_online = online;
			info->plug_count = 0;
			power_supply_changed(info->ac);
			power_supply_changed(info->usb);
			power_supply_changed(info->bat);
		} else if (online && info->plug_count < PLUG_SETTLE) {
			if (++info->plug_count == PLUG_SETTLE)
				axp_set_charge_current(info, info->charge_current);
		}
	}
	return 0;
}

// The battery check, every 500 ms. CHECK_ENABLE resets the gauge with VBUS
// there and turns charging on. CHECK_RUN follows the charger state and
// whether a battery is there: three battery voltages of 2 V or more in a row
// (chip 0x47), or REG00h bit 3 (chip 0x4a). A battery coming or going resets
// the gauge, turns CHGLED on or off and restarts the second the gauge needs
// before its level is read.
static void axp210x_battery_check(struct work_struct *work)
{
	static int vbat_ok[3] = { 1, 1, 1 };
	static unsigned int vbat_index;
	static ktime_t changed;
	struct axp210x_info *info = container_of(work, struct axp210x_info, work.work);
	u8 buf[2] = { 0, 0 };
	bool present;

	if (info->check == CHECK_START) {
		info->check = CHECK_ENABLE;
		goto requeue;
	}

	if (info->check == CHECK_ENABLE) {
		info->check = CHECK_RUN;
		axp_read(info, AXP_STATUS1, buf, 1);
		if (buf[0] & STATUS1_VBUS_GOOD) {
			if (axp210x_reset_gauge(info) < 0) {
				info->check = CHECK_ENABLE;
				goto requeue;
			}
			msleep(10);
		}
		info->gauge_ok = 1;
		mutex_lock(&info->lock);
		axp_charge_enable_write(info);
		mutex_unlock(&info->lock);
		goto requeue;
	}

	if (axp_read(info, AXP_STATUS2, buf, 1) < 0) {
		printk(KERN_INFO "axp2101: read AXP210X_REG_VBAT error\n\n");
		goto requeue;
	}
	if ((buf[0] & 0x07) == 4)
		battery_set_full(&info->full);
	else
		info->full = 0;

	if (info->chip_b) {
		if (axp_read(info, AXP_STATUS1, buf, 1) < 0) {
			printk(KERN_INFO "axp2101: read AXP210X_COMM_STAT0 error\n\n");
			goto requeue;
		}
		present = buf[0] & STATUS1_BAT_PRESENT;
	} else {
		if (axp_read(info, AXP_VBAT_H, buf, 2) < 0) {
			printk(KERN_INFO "axp2101: read AXP210X_REG_VBAT error\n\n");
			goto requeue;
		}
		vbat_ok[vbat_index] = (((buf[0] & 0x3f) << 8) | buf[1]) >= 2000;
		vbat_index = (vbat_index + 1) % 3;
		present = vbat_ok[0] && vbat_ok[1] && vbat_ok[2];
	}

	if (present != !!info->present) {
		info->present = present;
		if (!present)
			info->gauge_ok = 0;
		axp210x_reset_gauge(info);
		changed = ktime_get();
		regmap_update_bits(info->regmap, AXP_CHGLED, BIT(0), present ? BIT(0) : 0);
		power_supply_changed(info->bat);
	}

	if (info->present && ktime_to_ms(ktime_sub(ktime_get(), changed)) > 1000)
		info->gauge_ok = 1;

requeue:
	queue_delayed_work(system_wq, &info->work, 50);
}

static const struct axp_charger_pdata axp_charger_defaults = {
	.charge_current = 300,
	.shutdown_current = 1000,
	.voltage_limit = 4350,
};

static int axp2101_power_supplies_register(struct axp210x_info *info)
{
	struct power_supply_config psy_cfg = {
		.of_node = info->dev->of_node,
		.drv_data = info,
	};
	struct power_supply_desc *desc;
	int ret;

	desc = devm_kzalloc(info->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	desc->name = "axp_battery";
	desc->type = POWER_SUPPLY_TYPE_BATTERY;
	desc->properties = axp2101_bat_props;
	desc->num_properties = ARRAY_SIZE(axp2101_bat_props);
	desc->get_property = axp210x_bat_get_property;
	desc->set_property = axp210x_bat_set_property;
	desc->property_is_writeable = axp210x_bat_property_writeable;
	info->bat = power_supply_register(info->dev, desc, &psy_cfg);
	if (IS_ERR(info->bat)) {
		ret = PTR_ERR(info->bat);
		goto err_bat;
	}

	desc = devm_kzalloc(info->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc) {
		ret = -ENOMEM;
		goto err_usb;
	}
	desc->name = "usb";
	desc->type = POWER_SUPPLY_TYPE_USB;
	desc->properties = axp2101_usb_props;
	desc->num_properties = ARRAY_SIZE(axp2101_usb_props);
	desc->get_property = axp210x_usb_get_property;
	desc->set_property = axp210x_usb_set_property;
	desc->property_is_writeable = axp210x_usb_property_writeable;
	info->usb = power_supply_register(info->dev, desc, &psy_cfg);
	if (IS_ERR(info->usb)) {
		ret = PTR_ERR(info->usb);
		goto err_usb;
	}
	info->usb->supplied_to = supply_list;
	info->usb->num_supplicants = ARRAY_SIZE(supply_list);

	desc = devm_kzalloc(info->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc) {
		ret = -ENOMEM;
		goto err_ac;
	}
	desc->name = "ac";
	desc->type = POWER_SUPPLY_TYPE_MAINS;
	desc->properties = axp2101_ac_props;
	desc->num_properties = ARRAY_SIZE(axp2101_ac_props);
	desc->get_property = axp210x_ac_get_property;
	info->ac = power_supply_register(info->dev, desc, &psy_cfg);
	if (IS_ERR(info->ac)) {
		ret = PTR_ERR(info->ac);
		goto err_ac;
	}
	info->ac->supplied_to = supply_list;
	info->ac->num_supplicants = ARRAY_SIZE(supply_list);
	return 0;

err_ac:
	power_supply_unregister(info->usb);
err_usb:
	power_supply_unregister(info->bat);
err_bat:
	info->bat = info->usb = info->ac = NULL;
	return ret;
}

static void axp2101_power_supplies_unregister(struct axp210x_info *info)
{
	if (info->bat)
		power_supply_unregister(info->bat);
	if (info->ac)
		power_supply_unregister(info->ac);
	if (info->usb)
		power_supply_unregister(info->usb);
}

static int axp2101_charger_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	const struct axp_charger_pdata *pd = dev_get_platdata(&pdev->dev);
	struct axp210x_info *info;
	struct regmap *map;
	unsigned int key;
	u8 chip_id;
	int ret;

	if (!axp20x->irq) {
		printk(KERN_ERR "axp2101: can not register axp2101-charger without irq\n\n");
		return -EINVAL;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->regmap = map = axp20x->regmap;
	mutex_init(&info->lock);
	info->model = axp2101_model;
	info->model_len = sizeof(axp2101_model);
	info->present = 1;
	info->check = CHECK_RUN;
	if (!pd)
		pd = &axp_charger_defaults;
	info->charge_current = pd->charge_current;
	info->shutdown_current = pd->shutdown_current;
	info->voltage_limit = pd->voltage_limit;
	info->input_limit = 500;
	axp210x_info = info;

	regmap_update_bits(map, AXP_TS_CTRL, 0x0a, 0);
	regmap_update_bits(map, AXP_CV, 0x07, axp_cv_sel(info->voltage_limit));
	regmap_update_bits(map, AXP_BTN_CHG_CV, 0x07, 3);
	regmap_update_bits(map, AXP_CHGLED, 0x06, 0);
	regmap_write(map, AXP_LOW_WARN, (LOW_WARN1 - 5) << 4 | LOW_WARN2);
	regmap_update_bits(map, AXP_IPRECHG, 0x0f, 4);
	regmap_update_bits(map, AXP_ITERM, 0x0f, (pd->term_current ? pd->term_current : 50) / 25);
	regmap_update_bits(map, AXP_ITERM, BIT(4), BIT(4));
	if (pd->dcdc3_pwm)
		regmap_update_bits(map, AXP_DCDC_PWM, BIT(4), BIT(4));
	dcdc3_pwm_in_run = pd->dcdc3_pwm;
	regmap_update_bits(map, AXP_CHGLED, BIT(0), 0);
	axp_set_input_current_limit(info, info->input_limit);
	axp_set_charge_current(info, info->charge_current);
	regmap_write(map, AXP_BAT_DET, 1);
	if (pd->pwron_time && !regmap_read(map, AXP_IRQ_LEVEL, &key)) {
		key &= 0x3c;
		if (pd->pwron_time >= 2000)
			key |= 3;
		else if (pd->pwron_time >= 1000)
			key |= 2;
		else if (pd->pwron_time >= 512)
			key |= 1;
		regmap_write(map, AXP_IRQ_LEVEL, key);
	}

	if (axp_read(info, AXP_CHIP_ID, &chip_id, 1) < 0) {
		printk(KERN_INFO "axp2101: read AXP210X_CHIP_ID error\n\n");
		ret = -ENODEV;
		goto err;
	}
	chip_id &= 0xcf;
	if (chip_id != AXP2101_CHIP_A && chip_id != AXP2101_CHIP_B) {
		ret = -ENODEV;
		goto err;
	}
	printk(KERN_INFO "axp2101: read chipid = %x\n\n", chip_id);
	info->chip_b = chip_id == AXP2101_CHIP_B;

	if (axp210x_init_chip(info) < 0) {
		ret = -ENODEV;
		goto err;
	}

	ret = axp2101_power_supplies_register(info);
	if (ret) {
		// The model load may have turned charging off.
		mutex_lock(&info->lock);
		axp_charge_enable_write(info);
		mutex_unlock(&info->lock);
		goto err;
	}

	device_create_file(&info->bat->dev, &dev_attr_charger);

	// Without the monitor thread the probe carries on.
	info->thread = kthread_run(axp2101_monitor, info, "axp2101");
	if (IS_ERR(info->thread)) {
		dev_warn(info->dev, "no monitor thread: %ld\n", PTR_ERR(info->thread));
		info->thread = NULL;
	}
	platform_set_drvdata(pdev, info);

	INIT_DELAYED_WORK(&info->work, axp210x_battery_check);
	queue_delayed_work(system_wq, &info->work, 50);
	return 0;

err:
	axp210x_info = NULL;
	return ret;
}

static int axp2101_charger_remove(struct platform_device *pdev)
{
	struct axp210x_info *info = platform_get_drvdata(pdev);

	if (info->thread)
		kthread_stop(info->thread);
	cancel_delayed_work_sync(&info->work);
	axp_charge_release(info);
	axp2101_power_supplies_unregister(info);
	axp210x_info = NULL;
	return 0;
}

// At power off: the ADC in its low-rate mode for the battery alone (REG36h
// bit 7), charging on, and the charge current
// for while the player is off.
static void axp2101_shutdown(struct platform_device *pdev)
{
	struct axp210x_info *info = platform_get_drvdata(pdev);
	u8 val = 0x80;

	regmap_write(info->regmap, AXP_VBAT_H, 0);
	axp_write(info, AXP_TS_H, &val, 1);
	cancel_delayed_work_sync(&info->work);
	if (info->thread) {
		kthread_stop(info->thread);
		info->thread = NULL;
	}
	axp_charge_release(info);
	axp_set_charge_current(info, info->shutdown_current);
}

// DCDC3 leaves forced PWM for the suspend.
static int axp2101_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct axp210x_info *info = platform_get_drvdata(pdev);

	if (dcdc3_pwm_in_run)
		regmap_update_bits(info->regmap, AXP_DCDC_PWM, BIT(4), 0);
	cancel_delayed_work_sync(&info->work);
	return 0;
}

static int axp2101_resume(struct platform_device *pdev)
{
	struct axp210x_info *info = platform_get_drvdata(pdev);

	if (dcdc3_pwm_in_run)
		regmap_update_bits(info->regmap, AXP_DCDC_PWM, BIT(4), BIT(4));
	queue_delayed_work(system_wq, &info->work, 50);
	return 0;
}

static struct platform_driver axp2101_charger_driver = {
	.probe = axp2101_charger_probe,
	.remove = axp2101_charger_remove,
	.shutdown = axp2101_shutdown,
	.suspend = axp2101_suspend,
	.resume = axp2101_resume,
	.driver = {
		.name = AXP2101_SUPPLY_NAME,
	},
};

// --- the I2C device and the parameters -------------------------------------

static struct pmu_param axp_rail_params[AXP2101_NR_RAILS];
static int pmu_int_gpio = -1;
static int i2c_bus_num = -1;
static int charge_voltage_limit = 4200;
static int charge_term_current = 100;
static int dcdc3_always_pwmmode;

static struct axp_charger_pdata axp_sply_init_data = {
	.charge_current = 780,
	.shutdown_current = 780,
	.pwron_time = 1000,
};

static struct axp_dev_info axp_splydev[] = {
	{
		.name = AXP2101_SUPPLY_NAME,
		.platform_data = &axp_sply_init_data,
		.pdata_size = sizeof(axp_sply_init_data),
	},
};

static struct axp_platform_data axp_pdata = {
	.nsupplies = ARRAY_SIZE(axp_splydev),
	.supplies = axp_splydev,
};

static struct i2c_board_info axp2101_i2c_info = {
	I2C_BOARD_INFO("axp2101_mfd", AXP2101_I2C_ADDR),
	.platform_data = &axp_pdata,
};

static struct i2c_client *axp2101_client;

// Whether a selector of the regulator gives exactly UV.
static bool axp2101_voltage_selectable(const struct regulator_desc *desc, unsigned int uv)
{
	unsigned int sel;
	int i;

	if (desc->ops == &axp20x_ops_fixed)
		return uv == desc->min_uV;
	if (!desc->n_linear_ranges) {
		if (uv < desc->min_uV || (uv - desc->min_uV) % desc->uV_step)
			return false;
		return (uv - desc->min_uV) / desc->uV_step < desc->n_voltages;
	}
	for (i = 0; i < desc->n_linear_ranges; i++) {
		const struct regulator_linear_range *r = &desc->linear_ranges[i];

		if (uv < r->min_uV)
			continue;
		if (!r->uV_step) {
			if (uv != r->min_uV)
				continue;
			sel = r->min_sel;
		} else {
			if ((uv - r->min_uV) % r->uV_step)
				continue;
			sel = r->min_sel + (uv - r->min_uV) / r->uV_step;
		}
		if (sel <= r->max_sel && sel < desc->n_voltages)
			return true;
	}
	return false;
}

// Whether a parameter voltage is inside the rail's range and one of its
// selectors gives it.
static bool axp_rail_voltage_ok(int rail, unsigned int uv, const char *what)
{
	const struct regulation_constraints *c = &axp_regl_init_data[rail].constraints;

	if ((int)uv >= c->min_uV && (int)uv <= c->max_uV &&
	    axp2101_voltage_selectable(&axp2101_regulators[rail], uv))
		return true;
	pr_warn("axp2101: %s %s voltage %u uV is not one the rail has between %d and %d uV, not used\n",
		c->name, what, uv, c->min_uV, c->max_uV);
	return false;
}

// Applies the regulator parameters to the regulators' constraints, and
// creates the I2C device.
static int axp2101_i2c_device_init(void)
{
	int irq = gpiod_to_irq(gpio_to_desc(pmu_int_gpio));
	int i;

	axp2101_i2c_info.irq = irq;
	axp_sply_init_data.term_current = charge_term_current;
	axp_sply_init_data.voltage_limit = charge_voltage_limit;
	axp_sply_init_data.dcdc3_pwm = dcdc3_always_pwmmode;
	printk("AXP2101: IRQ GPIO is %d -> irq is %d\n", pmu_int_gpio, irq);
	printk("AXP2101: Charge Voltage Limit %d\n", charge_voltage_limit);
	printk("AXP2101: Charge Current Term %d\n", charge_term_current);

	for (i = 0; i < AXP2101_NR_RAILS; i++) {
		struct regulation_constraints *c = &axp_regl_init_data[i].constraints;
		const struct pmu_param *p = &axp_rail_params[i];

		c->boot_on = p->boot_on & 1;
		if (!p->suspend_voltage || axp_rail_voltage_ok(i, p->suspend_voltage, "suspend"))
			c->state_standby.uV = p->suspend_voltage;
		if (p->work_voltage && axp_rail_voltage_ok(i, p->work_voltage, "work")) {
			c->apply_uV = 1;
			c->min_uV = p->work_voltage;
			c->max_uV = p->work_voltage;
		}
	}
	axp_regulators_init_data = axp_regl_init_data;
	axp_nregulators_init_data = AXP2101_NR_RAILS;

	axp2101_client = i2c_register_device(&axp2101_i2c_info, i2c_bus_num);
	return axp2101_client ? 0 : -EINVAL;
}

static int __init axp2101_init(void)
{
	int ret;

	ret = i2c_add_driver(&axp2101_i2c_driver);
	if (ret) {
		pr_err("axp2101 i2c registration failed %d\n", ret);
		return ret;
	}

	axp2101_i2c_device_init();
	platform_driver_register(&axp2101_charger_driver);
	platform_driver_register(&axp2101_regulator_driver);
	return 0;
}
module_init(axp2101_init);

static void __exit axp2101_exit(void)
{
	platform_driver_unregister(&axp2101_regulator_driver);
	platform_driver_unregister(&axp2101_charger_driver);
	if (axp2101_client)
		i2c_unregister_device(axp2101_client);
	i2c_del_driver(&axp2101_i2c_driver);
}
module_exit(axp2101_exit);

#define AXP_RAIL_PARAM(_name, _rail) \
	module_param_cb(_name, &param_pmu_ops, &axp_rail_params[_rail], 0644)

AXP_RAIL_PARAM(dcdc1, AXP_DCDC1);
AXP_RAIL_PARAM(dcdc2, AXP_DCDC2);
AXP_RAIL_PARAM(dcdc3, AXP_DCDC3);
AXP_RAIL_PARAM(dcdc4, AXP_DCDC4);
AXP_RAIL_PARAM(dcdc5, AXP_DCDC5);
AXP_RAIL_PARAM(rtcldo, AXP_RTCLDO);
AXP_RAIL_PARAM(rtcldo1, AXP_RTCLDO1);
AXP_RAIL_PARAM(aldo1, AXP_ALDO1);
AXP_RAIL_PARAM(aldo2, AXP_ALDO2);
AXP_RAIL_PARAM(aldo3, AXP_ALDO3);
AXP_RAIL_PARAM(aldo4, AXP_ALDO4);
AXP_RAIL_PARAM(bldo1, AXP_BLDO1);
AXP_RAIL_PARAM(bldo2, AXP_BLDO2);
AXP_RAIL_PARAM(dldo1, AXP_DLDO1);
AXP_RAIL_PARAM(dldo2, AXP_DLDO2);
AXP_RAIL_PARAM(cpusldo, AXP_CPUSLDO);
module_param_cb(pmu_int_gpio, &param_gpio_ops, &pmu_int_gpio, 0644);
module_param(i2c_bus_num, int, 0644);
module_param(charge_voltage_limit, int, 0644);
module_param(charge_term_current, int, 0644);
module_param(dcdc3_always_pwmmode, int, 0644);

MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_DESCRIPTION("X-Powers AXP2101 PMIC: regulators, charger, fuel gauge");
MODULE_LICENSE("GPL");
