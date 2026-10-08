// SPDX-License-Identifier: GPL-2.0
//
// i2c_gpio_add -- bit-banged I2C buses on two pins, for the HiBy R3 Pro II
// and R1: bus 3, which carries the DAC.
//
// Based on drivers/i2c/busses/i2c-gpio.c, Copyright (C) 2007 Atmel
// Corporation, and drivers/i2c/algos/i2c-algo-bit.c, Copyright (C)
// 1995-2000 Simon G. Vogl, with changes from Frodo Looijaard, Kyosti Malkki
// and Jean Delvare. The kernel has neither built in, so both are here: the
// platform driver md_i2c_gpio and the algorithm, exported as
// md_i2c_bit_add_numbered_bus().
//
// A drop-in replacement for the vendor module of the same name. A bus is
// added through the parameter i2c_bus, one line per bus (the stock
// i2c_gpio_add.sh writes it):
//
//   bus_num=N scl=PIN sda=PIN [rate=HZ]
//
// rate sets the half-period to 1000000 / rate us; without it, 5 us. Reading
// the parameter lists the buses. Unloading takes the buses away.
//
// Loaded after utils.ko, which provides pin names such as "PB30" and the
// word splitting.

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>

// Exported by utils.ko.
extern char *gpio_to_str(int gpio, char *buf);
extern char **str_to_words(const char *str, int *count);
extern void str_free_words(char **words);
extern int str_to_gpio(const char *str);

// ---- The bit-banging algorithm ----

static int bit_test;	// test the lines before adding a bus
module_param(bit_test, int, S_IRUGO);
MODULE_PARM_DESC(bit_test, "lines testing - 0 off; 1 report; 2 fail if stuck");

#define setsda(adap, val)	adap->setsda(adap->data, val)
#define setscl(adap, val)	adap->setscl(adap->data, val)
#define getsda(adap)		adap->getsda(adap->data)
#define getscl(adap)		adap->getscl(adap->data)

static inline void sdalo(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 0);
	udelay((adap->udelay + 1) / 2);
}

static inline void sdahi(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 1);
	udelay((adap->udelay + 1) / 2);
}

static inline void scllo(struct i2c_algo_bit_data *adap)
{
	setscl(adap, 0);
	udelay(adap->udelay / 2);
}

// Raises SCL and, when SCL can be read, waits for a slave holding it low
// (clock stretching), up to the timeout.
static int sclhi(struct i2c_algo_bit_data *adap)
{
	unsigned long start;

	setscl(adap, 1);

	if (!adap->getscl)
		goto done;

	start = jiffies;
	while (!getscl(adap)) {
		if (time_after(jiffies, start + adap->timeout)) {
			// Once more, in case of preemption since the last read.
			if (getscl(adap))
				break;
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

done:
	udelay(adap->udelay);
	return 0;
}

static void i2c_start(struct i2c_algo_bit_data *adap)
{
	// SCL and SDA are high.
	setsda(adap, 0);
	udelay(adap->udelay);
	scllo(adap);
}

static void i2c_repstart(struct i2c_algo_bit_data *adap)
{
	// SCL is low.
	sdahi(adap);
	sclhi(adap);
	setsda(adap, 0);
	udelay(adap->udelay);
	scllo(adap);
}

static void i2c_stop(struct i2c_algo_bit_data *adap)
{
	// SCL is low.
	sdalo(adap);
	sclhi(adap);
	setsda(adap, 1);
	udelay(adap->udelay);
}

// Sends a byte without start condition: 1 if the slave acknowledged, 0 if
// not, -ETIMEDOUT if SCL could not be raised.
static int i2c_outb(struct i2c_adapter *i2c_adap, unsigned char c)
{
	int i;
	int sb;
	int ack;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	// SCL is low.
	for (i = 7; i >= 0; i--) {
		sb = (c >> i) & 1;
		setsda(adap, sb);
		udelay((adap->udelay + 1) / 2);
		if (sclhi(adap) < 0)
			return -ETIMEDOUT;
		scllo(adap);
	}
	sdahi(adap);
	if (sclhi(adap) < 0)
		return -ETIMEDOUT;

	// The slave pulls SDA low to acknowledge.
	ack = !getsda(adap);

	scllo(adap);
	return ack;
}

// Reads a byte without start or stop; the acknowledge is the caller's.
static int i2c_inb(struct i2c_adapter *i2c_adap)
{
	int i;
	unsigned char indata = 0;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	// SCL is low.
	sdahi(adap);
	for (i = 0; i < 8; i++) {
		if (sclhi(adap) < 0)
			return -ETIMEDOUT;
		indata *= 2;
		if (getsda(adap))
			indata |= 0x01;
		setscl(adap, 0);
		udelay(i == 7 ? adap->udelay / 2 : adap->udelay);
	}
	return indata;
}

// Checks that the lines follow what is driven on them, from an idle bus.
static int test_bus(struct i2c_adapter *i2c_adap)
{
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;
	const char *name = i2c_adap->name;
	int scl, sda, ret;

	if (adap->pre_xfer) {
		ret = adap->pre_xfer(i2c_adap);
		if (ret < 0)
			return -ENODEV;
	}

	if (adap->getscl == NULL)
		pr_info("%s: Testing SDA only, SCL is not readable\n", name);

	sda = getsda(adap);
	scl = (adap->getscl == NULL) ? 1 : getscl(adap);
	if (!scl || !sda) {
		printk(KERN_WARNING "%s: bus seems to be busy (scl=%d, sda=%d)\n", name, scl, sda);
		goto bailout;
	}

	sdalo(adap);
	sda = getsda(adap);
	scl = (adap->getscl == NULL) ? 1 : getscl(adap);
	if (sda) {
		printk(KERN_WARNING "%s: SDA stuck high!\n", name);
		goto bailout;
	}
	if (!scl) {
		printk(KERN_WARNING "%s: SCL unexpected low while pulling SDA low!\n", name);
		goto bailout;
	}

	sdahi(adap);
	sda = getsda(adap);
	scl = (adap->getscl == NULL) ? 1 : getscl(adap);
	if (!sda) {
		printk(KERN_WARNING "%s: SDA stuck low!\n", name);
		goto bailout;
	}
	if (!scl) {
		printk(KERN_WARNING "%s: SCL unexpected low while pulling SDA high!\n", name);
		goto bailout;
	}

	scllo(adap);
	sda = getsda(adap);
	scl = (adap->getscl == NULL) ? 0 : getscl(adap);
	if (scl) {
		printk(KERN_WARNING "%s: SCL stuck high!\n", name);
		goto bailout;
	}
	if (!sda) {
		printk(KERN_WARNING "%s: SDA unexpected low while pulling SCL low!\n", name);
		goto bailout;
	}

	sclhi(adap);
	sda = getsda(adap);
	scl = (adap->getscl == NULL) ? 1 : getscl(adap);
	if (!scl) {
		printk(KERN_WARNING "%s: SCL stuck low!\n", name);
		goto bailout;
	}
	if (!sda) {
		printk(KERN_WARNING "%s: SDA unexpected low while pulling SCL high!\n", name);
		goto bailout;
	}

	if (adap->post_xfer)
		adap->post_xfer(i2c_adap);

	pr_info("%s: Test OK\n", name);
	return 0;
bailout:
	sdahi(adap);
	sclhi(adap);

	if (adap->post_xfer)
		adap->post_xfer(i2c_adap);

	return -ENODEV;
}

// Sends the address up to retries + 1 times: 1 when the chip answered, 0
// when it did not, below 0 on a bus error.
static int try_address(struct i2c_adapter *i2c_adap, unsigned char addr, int retries)
{
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;
	int i, ret = 0;

	for (i = 0; i <= retries; i++) {
		ret = i2c_outb(i2c_adap, addr);
		if (ret == 1 || i == retries)
			break;
		i2c_stop(adap);
		udelay(adap->udelay);
		yield();
		i2c_start(adap);
	}
	return ret;
}

static int sendbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	const unsigned char *temp = msg->buf;
	int count = msg->len;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	int retval;
	int wrcount = 0;

	while (count > 0) {
		retval = i2c_outb(i2c_adap, *temp);

		// Acknowledged, or a NAK that may be ignored.
		if ((retval > 0) || (nak_ok && (retval == 0))) {
			count--;
			temp++;
			wrcount++;
		} else if (retval == 0) {
			dev_err(&i2c_adap->dev, "sendbytes: NAK bailout.\n");
			return -EIO;
		} else {
			dev_err(&i2c_adap->dev, "sendbytes: error %d\n", retval);
			return retval;
		}
	}
	return wrcount;
}

static int acknak(struct i2c_adapter *i2c_adap, int is_ack)
{
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	// SDA is high.
	if (is_ack)
		setsda(adap, 0);
	udelay((adap->udelay + 1) / 2);
	if (sclhi(adap) < 0) {
		dev_err(&i2c_adap->dev, "readbytes: ack/nak timeout\n");
		return -ETIMEDOUT;
	}
	scllo(adap);
	return 0;
}

static int readbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	int inval;
	int rdcount = 0;
	unsigned char *temp = msg->buf;
	int count = msg->len;
	const unsigned flags = msg->flags;

	while (count > 0) {
		inval = i2c_inb(i2c_adap);
		if (inval >= 0) {
			*temp = inval;
			rdcount++;
		} else {
			break;
		}

		temp++;
		count--;

		// SMBus block reads: the first byte is the length to follow.
		if (rdcount == 1 && (flags & I2C_M_RECV_LEN)) {
			if (inval <= 0 || inval > I2C_SMBUS_BLOCK_MAX) {
				if (!(flags & I2C_M_NO_RD_ACK))
					acknak(i2c_adap, 0);
				dev_err(&i2c_adap->dev, "readbytes: invalid block length (%d)\n", inval);
				return -EPROTO;
			}
			count += inval;
			msg->len += inval;
		}

		if (!(flags & I2C_M_NO_RD_ACK)) {
			inval = acknak(i2c_adap, count);
			if (inval < 0)
				return inval;
		}
	}
	return rdcount;
}

// Sends the address of a message, 7 or 10 bits: 0 when the chip answered
// (or I2C_M_IGNORE_NAK is set), below 0 otherwise.
static int bit_doAddress(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	unsigned short flags = msg->flags;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	unsigned char addr;
	int ret, retries;

	retries = nak_ok ? 0 : i2c_adap->retries;

	if (flags & I2C_M_TEN) {
		addr = 0xf0 | ((msg->addr >> 7) & 0x06);
		ret = try_address(i2c_adap, addr, retries);
		if ((ret != 1) && !nak_ok)  {
			dev_err(&i2c_adap->dev, "died at extended address code\n");
			return -ENXIO;
		}
		ret = i2c_outb(i2c_adap, msg->addr & 0xff);
		if ((ret != 1) && !nak_ok) {
			dev_err(&i2c_adap->dev, "died at 2nd address code\n");
			return -ENXIO;
		}
		if (flags & I2C_M_RD) {
			i2c_repstart(adap);
			addr |= 0x01;
			ret = try_address(i2c_adap, addr, retries);
			if ((ret != 1) && !nak_ok) {
				dev_err(&i2c_adap->dev, "died at repeated address code\n");
				return -EIO;
			}
		}
	} else {
		addr = msg->addr << 1;
		if (flags & I2C_M_RD)
			addr |= 1;
		if (flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;
		ret = try_address(i2c_adap, addr, retries);
		if ((ret != 1) && !nak_ok)
			return -ENXIO;
	}

	return 0;
}

static int bit_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
	struct i2c_msg *pmsg;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;
	int i, ret;
	unsigned short nak_ok;

	if (adap->pre_xfer) {
		ret = adap->pre_xfer(i2c_adap);
		if (ret < 0)
			return ret;
	}

	i2c_start(adap);
	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;
		if (!(pmsg->flags & I2C_M_NOSTART)) {
			if (i)
				i2c_repstart(adap);
			ret = bit_doAddress(i2c_adap, pmsg);
			if ((ret != 0) && !nak_ok)
				goto bailout;
		}
		if (pmsg->flags & I2C_M_RD) {
			ret = readbytes(i2c_adap, pmsg);
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EIO;
				goto bailout;
			}
		} else {
			ret = sendbytes(i2c_adap, pmsg);
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EIO;
				goto bailout;
			}
		}
	}
	ret = i;

bailout:
	i2c_stop(adap);

	if (adap->post_xfer)
		adap->post_xfer(i2c_adap);
	return ret;
}

static u32 bit_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_NOSTART | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL |
	       I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING;
}

static const struct i2c_algorithm md_i2c_bit_algo = {
	.master_xfer	= bit_xfer,
	.functionality	= bit_func,
};

// Adds the adapter as bus adap->nr.
int md_i2c_bit_add_numbered_bus(struct i2c_adapter *adap)
{
	struct i2c_algo_bit_data *bit_adap = adap->algo_data;
	int ret;

	if (bit_test) {
		ret = test_bus(adap);
		if (bit_test >= 2 && ret < 0)
			return -ENODEV;
	}

	adap->algo = &md_i2c_bit_algo;
	adap->retries = 3;

	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0)
		return ret;

	if (bit_adap->getscl == NULL) {
		dev_warn(&adap->dev, "Not I2C compliant: can't read SCL\n");
		dev_warn(&adap->dev, "Bus may be unreliable\n");
	}
	return 0;
}
EXPORT_SYMBOL(md_i2c_bit_add_numbered_bus);

// ---- The platform driver ----

struct i2c_gpio_private_data {
	struct i2c_adapter adap;
	struct i2c_algo_bit_data bit_data;
	struct i2c_gpio_platform_data pdata;
};

// SDA by direction: input lets the pull-up raise it.
static void i2c_gpio_setsda_dir(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	if (state)
		gpio_direction_input(pdata->sda_pin);
	else
		gpio_direction_output(pdata->sda_pin, 0);
}

// SDA by value, for an open-drain pin.
static void i2c_gpio_setsda_val(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	gpio_set_value(pdata->sda_pin, state);
}

static void i2c_gpio_setscl_dir(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	if (state)
		gpio_direction_input(pdata->scl_pin);
	else
		gpio_direction_output(pdata->scl_pin, 0);
}

// SCL by value, for an open-drain or output-only pin.
static void i2c_gpio_setscl_val(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	gpio_set_value(pdata->scl_pin, state);
}

static int i2c_gpio_getsda(void *data)
{
	struct i2c_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->sda_pin);
}

static int i2c_gpio_getscl(void *data)
{
	struct i2c_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->scl_pin);
}

static int of_i2c_gpio_get_pins(struct device_node *np, unsigned int *sda_pin,
				unsigned int *scl_pin)
{
	if (of_gpio_count(np) < 2)
		return -ENODEV;

	*sda_pin = of_get_gpio(np, 0);
	*scl_pin = of_get_gpio(np, 1);

	if (!gpio_is_valid(*sda_pin) || !gpio_is_valid(*scl_pin)) {
		pr_err("%s: invalid GPIO pins, sda=%d/scl=%d\n", np->full_name, *sda_pin, *scl_pin);
		return -ENODEV;
	}

	return 0;
}

static void of_i2c_gpio_get_props(struct device_node *np, struct i2c_gpio_platform_data *pdata)
{
	u32 reg;

	of_property_read_u32(np, "i2c-gpio,delay-us", &pdata->udelay);

	if (!of_property_read_u32(np, "i2c-gpio,timeout-ms", &reg))
		pdata->timeout = msecs_to_jiffies(reg);

	pdata->sda_is_open_drain = of_property_read_bool(np, "i2c-gpio,sda-open-drain");
	pdata->scl_is_open_drain = of_property_read_bool(np, "i2c-gpio,scl-open-drain");
	pdata->scl_is_output_only = of_property_read_bool(np, "i2c-gpio,scl-output-only");
}

static int i2c_gpio_probe(struct platform_device *pdev)
{
	struct i2c_gpio_private_data *priv;
	struct i2c_gpio_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	unsigned int sda_pin, scl_pin;
	int ret;

	if (pdev->dev.of_node) {
		ret = of_i2c_gpio_get_pins(pdev->dev.of_node, &sda_pin, &scl_pin);
		if (ret)
			return ret;
	} else {
		if (!dev_get_platdata(&pdev->dev))
			return -ENXIO;
		pdata = dev_get_platdata(&pdev->dev);
		sda_pin = pdata->sda_pin;
		scl_pin = pdata->scl_pin;
	}

	// A pin not there yet: try again later.
	ret = gpio_request(sda_pin, "sda");
	if (ret) {
		if (ret == -EINVAL)
			ret = -EPROBE_DEFER;
		return ret;
	}
	ret = gpio_request(scl_pin, "scl");
	if (ret) {
		if (ret == -EINVAL)
			ret = -EPROBE_DEFER;
		goto err_scl;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err_add_bus;
	}
	adap = &priv->adap;
	bit_data = &priv->bit_data;
	pdata = &priv->pdata;

	if (pdev->dev.of_node) {
		pdata->sda_pin = sda_pin;
		pdata->scl_pin = scl_pin;
		of_i2c_gpio_get_props(pdev->dev.of_node, pdata);
	} else {
		memcpy(pdata, dev_get_platdata(&pdev->dev), sizeof(*pdata));
	}

	if (pdata->sda_is_open_drain) {
		gpio_direction_output(pdata->sda_pin, 1);
		bit_data->setsda = i2c_gpio_setsda_val;
	} else {
		gpio_direction_input(pdata->sda_pin);
		bit_data->setsda = i2c_gpio_setsda_dir;
	}

	if (pdata->scl_is_open_drain || pdata->scl_is_output_only) {
		gpio_direction_output(pdata->scl_pin, 1);
		bit_data->setscl = i2c_gpio_setscl_val;
	} else {
		gpio_direction_input(pdata->scl_pin);
		bit_data->setscl = i2c_gpio_setscl_dir;
	}

	if (!pdata->scl_is_output_only)
		bit_data->getscl = i2c_gpio_getscl;
	bit_data->getsda = i2c_gpio_getsda;

	if (pdata->udelay)
		bit_data->udelay = pdata->udelay;
	else if (pdata->scl_is_output_only)
		bit_data->udelay = 50;			// 10 kHz
	else
		bit_data->udelay = 5;			// 100 kHz

	if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		// 100 ms

	bit_data->data = pdata;

	adap->owner = THIS_MODULE;
	if (pdev->dev.of_node)
		strlcpy(adap->name, dev_name(&pdev->dev), sizeof(adap->name));
	else
		snprintf(adap->name, sizeof(adap->name), "i2c-gpio%d", pdev->id);

	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	adap->nr = pdev->id;
	ret = md_i2c_bit_add_numbered_bus(adap);
	if (ret)
		goto err_add_bus;

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "using pins %u (SDA) and %u (SCL%s)\n",
		 pdata->sda_pin, pdata->scl_pin,
		 pdata->scl_is_output_only ? ", no clock stretching" : "");

	return 0;

err_add_bus:
	gpio_free(scl_pin);
err_scl:
	gpio_free(sda_pin);
	return ret;
}

static int i2c_gpio_remove(struct platform_device *pdev)
{
	struct i2c_gpio_private_data *priv = platform_get_drvdata(pdev);

	i2c_del_adapter(&priv->adap);
	gpio_free(priv->pdata.scl_pin);
	gpio_free(priv->pdata.sda_pin);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id i2c_gpio_dt_ids[] = {
	{ .compatible = "md_i2c_gpio", },
	{ }
};
MODULE_DEVICE_TABLE(of, i2c_gpio_dt_ids);
#endif

static struct platform_driver i2c_gpio_driver = {
	.driver		= {
		.name	= "md_i2c_gpio",
		.of_match_table	= of_match_ptr(i2c_gpio_dt_ids),
	},
	.probe		= i2c_gpio_probe,
	.remove		= i2c_gpio_remove,
};

// ---- The i2c_bus parameter ----

// One bus added through the parameter: its device and the pins.
struct i2c_gpio_add_device {
	struct list_head list;
	struct platform_device pdev;
	struct i2c_gpio_platform_data pdata;
};

static LIST_HEAD(gpio_i2c_list);

// The devices own nothing; i2c_gpio_platform_device_remove() frees them.
static void i2c_gpio_add_device_release(struct device *dev)
{
}

// The value of "key=..." in w, or NULL for another word.
static const char *i2c_bus_arg(const char *w, const char *key)
{
	return strncmp(w, key, strlen(key)) ? NULL : w + strlen(key);
}

static int i2c_bus_pin(const char *w, const char *v, unsigned int *pin)
{
	int gpio = str_to_gpio(v);

	if (gpio < -1) {
		pr_err("%s is not a gpio\n", w);
		return -EINVAL;
	}
	*pin = gpio;
	return 0;
}

static int param_i2c_bus_set(const char *val, const struct kernel_param *kp)
{
	struct i2c_gpio_platform_data pdata;
	struct i2c_gpio_add_device *d;
	unsigned long n;
	int bus_num = -1;
	const char *v, *w;
	char **words;
	int count, i, ret = 0;

	memset(&pdata, 0, sizeof(pdata));
	pdata.sda_pin = -1;
	pdata.scl_pin = -1;

	words = str_to_words(val, &count);
	for (i = 0; i < count && !ret; i++) {
		w = words[i];
		if ((v = i2c_bus_arg(w, "scl="))) {
			ret = i2c_bus_pin(w, v, &pdata.scl_pin);
		} else if ((v = i2c_bus_arg(w, "sda="))) {
			ret = i2c_bus_pin(w, v, &pdata.sda_pin);
		} else if ((v = i2c_bus_arg(w, "rate="))) {
			if (kstrtoul(v, 0, &n)) {
				pr_err("%s is not a valid rate\n", w);
				ret = -EINVAL;
			} else {
				pdata.udelay = n ? 1000000 / n : 0;
			}
		} else if ((v = i2c_bus_arg(w, "bus_num="))) {
			if (kstrtoul(v, 0, &n)) {
				pr_err("%s is not a valid num\n", w);
				ret = -EINVAL;
			} else {
				bus_num = n;
			}
		} else {
			pr_err("%s can not be parse!\n", w);
			ret = -EINVAL;
		}
	}
	if (!ret && pdata.scl_pin == -1) {
		pr_err("scl must define!\n");
		ret = -EINVAL;
	} else if (!ret && pdata.sda_pin == -1) {
		pr_err("sda must define!\n");
		ret = -EINVAL;
	} else if (!ret && bus_num == -1) {
		pr_err("bus_num must define!\n");
		ret = -EINVAL;
	}
	str_free_words(words);
	if (ret)
		return ret;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->pdata = pdata;
	d->pdev.name = "md_i2c_gpio";
	d->pdev.id = bus_num;
	d->pdev.dev.platform_data = &d->pdata;
	d->pdev.dev.release = i2c_gpio_add_device_release;
	ret = platform_device_register(&d->pdev);
	if (ret < 0) {
		pr_err("\ni2c_gpio: failed to register platform device\n");
		platform_device_put(&d->pdev);
		kfree(d);
		return ret;
	}
	list_add_tail(&d->list, &gpio_i2c_list);
	return 0;
}

static int param_i2c_bus_get(char *buffer, const struct kernel_param *kp)
{
	struct i2c_gpio_add_device *d;
	char scl[16], sda[16];
	char *p = buffer;

	list_for_each_entry(d, &gpio_i2c_list, list)
		p += sprintf(p, "bus_num=%d udelay=%dus scl=%s sda=%s\n", d->pdev.id, d->pdata.udelay,
			     gpio_to_str(d->pdata.scl_pin, scl), gpio_to_str(d->pdata.sda_pin, sda));
	return p - buffer;
}

static const struct kernel_param_ops param_i2c_bus_ops = {
	.set = param_i2c_bus_set,
	.get = param_i2c_bus_get,
};
module_param_cb(i2c_bus, &param_i2c_bus_ops, NULL, 0644);

// Takes away every bus added through the parameter.
int i2c_gpio_platform_device_remove(void)
{
	struct i2c_gpio_add_device *d, *next;

	list_for_each_entry_safe(d, next, &gpio_i2c_list, list) {
		list_del(&d->list);
		platform_device_unregister(&d->pdev);
		kfree(d);
	}
	return 0;
}
EXPORT_SYMBOL(i2c_gpio_platform_device_remove);

static int __init i2c_gpio_init(void)
{
	int ret;

	ret = platform_driver_register(&i2c_gpio_driver);
	if (ret)
		printk(KERN_ERR "i2c-gpio: probe failed: %d\n", ret);

	return ret;
}
module_init(i2c_gpio_init);

static void __exit i2c_gpio_exit(void)
{
	platform_driver_unregister(&i2c_gpio_driver);
	i2c_gpio_platform_device_remove();
}
module_exit(i2c_gpio_exit);

MODULE_ALIAS("platform:md_i2c_gpio");
MODULE_DESCRIPTION("Platform-independent bitbanging I2C driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
