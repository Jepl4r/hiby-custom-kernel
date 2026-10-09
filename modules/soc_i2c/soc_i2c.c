// SPDX-License-Identifier: GPL-2.0
//
// soc_i2c -- the two I2C controllers of the X1600 (i2c0, i2c1) as I2C
// adapters 0 and 1, the buses of the on-board chips such as the touch panel.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock soc_i2c.sh passes them):
//
//   i2cN_is_enable   1: register bus N
//   i2cN_rate        bus speed in Hz: up to 100000 runs at 100 kHz, above
//                    at 400 kHz
//   i2cN_scl         pins, by name ("PB30"); each must be one of the pins
//   i2cN_sda         the controller can be routed to
//
// The controller clock runs only during a transfer. Messages are moved
// through the 64-entry FIFOs from the interrupt handler, and a transfer waits
// for it with a timeout scaled to the message length.

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <soc/gpio.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern int gpio_set_func(int gpio, int func);
extern char *gpio_to_str(int gpio, char *buf);

// --- registers --------------------------------------------------------------

#define I2C_CON			0x00
#define I2C_TAR			0x04
#define I2C_DC			0x10	// data and command
#define I2C_SHCNT		0x14
#define I2C_SLCNT		0x18
#define I2C_FHCNT		0x1c
#define I2C_FLCNT		0x20
#define I2C_INTST		0x2c
#define I2C_INTM		0x30
#define I2C_RXTL		0x38
#define I2C_TXTL		0x3c
#define I2C_CTXABRT		0x54	// read: clears TXABRT
#define I2C_CSTP		0x60	// read: clears STOP_DET
#define I2C_ENB			0x6c
#define I2C_STA			0x70
#define I2C_TXFLR		0x74
#define I2C_RXFLR		0x78
#define I2C_SDAHD		0x7c
#define I2C_TXABRT		0x80
#define I2C_FSSPKLEN		0x94
#define I2C_ENSTA		0x9c
#define I2C_SDASU		0xa0

#define I2C_CON_SPEED_MASK	(3 << 1)
#define I2C_CON_SPEED_STD	(1 << 1)
#define I2C_CON_SPEED_FAST	(2 << 1)
#define I2C_CON_RESTART_EN	BIT(5)
#define I2C_TAR_10BIT		BIT(12)
#define I2C_DC_READ		BIT(8)
#define I2C_DC_STOP		BIT(9)
#define I2C_DC_RESTART		BIT(10)
#define I2C_INT_RXFL		BIT(2)
#define I2C_INT_TXEMP		BIT(4)
#define I2C_INT_TXABT		BIT(6)
#define I2C_INT_STP		BIT(9)
#define I2C_STA_MSTACT		BIT(5)

#define I2C_FIFO_DEPTH		64
#define I2C_BUSES		2

// Per-message flags.
#define MSG_FIRST		BIT(0)	// starts the transfer: address loaded
#define MSG_RESTART		BIT(1)	// the first command carries a restart
#define MSG_LAST		BIT(2)	// the last command carries a stop

// A pin the controller can be routed to, and the function that does it.
struct jz_i2c_pin {
	int scl;
	int scl_func;
	int sda;
	int sda_func;
};

struct jz_i2c {
	int id;
	int is_enable;
	int inited;
	int rate;		// Hz, set to 100000 or 400000 by the speed set-up
	const char *scl_label;
	const char *sda_label;
	int scl;
	int scl_func;
	int sda;
	int sda_func;
	int pin_count;
	const struct jz_i2c_pin *pins;
	int irq;
	int reserved;
	u32 abort;		// TXABRT source bits of a failed message
	int remain;		// bytes still to write, or to read
	int flags;		// MSG_*
	int rx_cmds;		// read commands still to queue
	u8 *wbuf;
	u8 *rbuf;
	struct clk *clk;
	const char *clk_name;
	struct completion done;
	const char *irq_name;
	struct i2c_adapter adap;
};

static void __iomem *const iobase[I2C_BUSES] = {
	(void __iomem *)0xb0050000,
	(void __iomem *)0xb0051000,
};

static const struct jz_i2c_pin jz_i2c0_pin[] = {
	{ GPIO_PA(28), GPIO_FUNC_2, GPIO_PA(29), GPIO_FUNC_2 },
	{ GPIO_PB(30), GPIO_FUNC_0, GPIO_PB(31), GPIO_FUNC_0 },
};

static const struct jz_i2c_pin jz_i2c1_pin[] = {
	{ GPIO_PB(15), GPIO_FUNC_2, GPIO_PB(16), GPIO_FUNC_2 },
	{ GPIO_PB(19), GPIO_FUNC_0, GPIO_PB(20), GPIO_FUNC_0 },
};

static struct jz_i2c jz_i2c_dev[I2C_BUSES] = {
	{
		.id = 0,
		.scl_label = "i2c0_scl",
		.sda_label = "i2c0_sda",
		.pin_count = ARRAY_SIZE(jz_i2c0_pin),
		.pins = jz_i2c0_pin,
		.irq = 69,
		.clk_name = "gate_i2c0",
		.irq_name = "I2C0",
	},
	{
		.id = 1,
		.scl_label = "i2c1_scl",
		.sda_label = "i2c1_sda",
		.pin_count = ARRAY_SIZE(jz_i2c1_pin),
		.pins = jz_i2c1_pin,
		.irq = 68,
		.clk_name = "gate_i2c1",
		.irq_name = "I2C1",
	},
};

module_param_named(i2c0_is_enable, jz_i2c_dev[0].is_enable, int, 0644);
module_param_named(i2c0_rate, jz_i2c_dev[0].rate, int, 0644);
module_param_cb(i2c0_scl, &param_gpio_ops, &jz_i2c_dev[0].scl, 0644);
module_param_cb(i2c0_sda, &param_gpio_ops, &jz_i2c_dev[0].sda, 0644);
module_param_named(i2c1_is_enable, jz_i2c_dev[1].is_enable, int, 0644);
module_param_named(i2c1_rate, jz_i2c_dev[1].rate, int, 0644);
module_param_cb(i2c1_scl, &param_gpio_ops, &jz_i2c_dev[1].scl, 0644);
module_param_cb(i2c1_sda, &param_gpio_ops, &jz_i2c_dev[1].sda, 0644);

static const char *const abrt_src[] = {
	"I2C_TXABRT_ABRT_7B_ADDR_NOACK",
	"I2C_TXABRT_ABRT_10ADDR1_NOACK",
	"I2C_TXABRT_ABRT_10ADDR2_NOACK",
	"I2C_TXABRT_ABRT_XDATA_NOACK",
	"I2C_TXABRT_ABRT_GCALL_NOACK",
	"I2C_TXABRT_ABRT_GCALL_READ",
	"I2C_TXABRT_ABRT_HS_ACKD",
	"I2C_TXABRT_SBYTE_ACKDET",
	"I2C_TXABRT_ABRT_HS_NORSTRT",
	"I2C_TXABRT_SBYTE_NORSTRT",
	"I2C_TXABRT_ABRT_10B_RD_NORSTRT",
	"I2C_TXABRT_ABRT_MASTER_DIS",
	"I2C_TXABRT_ARB_LOST",
	"I2C_TXABRT_SLVFLUSH_TXFIFO",
	"I2C_TXABRT_SLV_ARBLOST",
	"I2C_TXABRT_SLVRD_INTX",
};

static inline u32 i2c_read(int id, unsigned int off)
{
	return readl(iobase[id] + off);
}

static inline void i2c_write(int id, unsigned int off, u32 v)
{
	writel(v, iobase[id] + off);
}

// Replaces the bits of mask in a register.
static inline void i2c_update(int id, unsigned int off, u32 mask, u32 v)
{
	u32 old = i2c_read(id, off);

	i2c_write(id, off, ((v ^ old) & mask) ^ old);
}

static void jz_i2c_hal_disable_smb(int id)
{
	i2c_update(id, I2C_ENB, 1, 0);
}

// Queues read commands, at most the ones still to queue; the first one
// carries a restart if restart is set and the message asks for it, the last
// one of the message a stop if it is the last message.
static void jz_i2c_write_rx_fifo(struct jz_i2c *i2c, int n, int restart)
{
	int flags = i2c->flags;
	u32 first = flags & MSG_RESTART ? I2C_DC_READ | I2C_DC_RESTART : I2C_DC_READ;
	u32 cmd;

	if (n >= i2c->rx_cmds)
		n = i2c->rx_cmds;
	i2c->rx_cmds -= n;
	for (; n > 0; n--) {
		cmd = I2C_DC_READ;
		if (restart) {
			cmd = first;
			if (flags & MSG_RESTART)
				restart = 0;
		}
		if (n == 1 && !i2c->rx_cmds && (flags & MSG_LAST))
			cmd |= I2C_DC_STOP;
		i2c_write(i2c->id, I2C_DC, cmd);
	}
}

// Queues bytes to write, as for jz_i2c_write_rx_fifo().
static void jz_i2c_write_tx_fifo(struct jz_i2c *i2c, int n, int restart)
{
	int flags = i2c->flags;
	u32 data;

	if (n >= i2c->remain)
		n = i2c->remain;
	i2c->remain -= n;
	for (; n > 0; n--) {
		data = *i2c->wbuf++;
		if (restart && (flags & MSG_RESTART)) {
			data |= I2C_DC_RESTART;
			restart = 0;
		}
		if (n == 1 && !i2c->remain && (flags & MSG_LAST))
			data |= I2C_DC_STOP;
		i2c_write(i2c->id, I2C_DC, data);
	}
}

static u32 jz_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}

// An abort ends the message; an empty transmit FIFO is refilled, and with
// nothing left to send ends a message that has no stop; a full receive FIFO
// is emptied and the read commands topped up; a stop ends the message.
static irqreturn_t i2c_intr_handler(int irq, void *data)
{
	struct jz_i2c *i2c = data;
	int id = i2c->id;
	u32 st = i2c_read(id, I2C_INTST);
	int n;

	if (st & I2C_INT_TXABT) {
		i2c->abort = i2c_read(id, I2C_TXABRT);
		i2c_read(id, I2C_CTXABRT);
		complete(&i2c->done);
		return IRQ_HANDLED;
	}

	if (st & I2C_INT_TXEMP) {
		if (!i2c->remain) {
			i2c_update(id, I2C_INTM, I2C_INT_TXEMP, 0);
			if (!(i2c->flags & MSG_LAST)) {
				i2c_update(id, I2C_INTM, I2C_INT_STP, 0);
				complete(&i2c->done);
				return IRQ_HANDLED;
			}
		} else {
			n = I2C_FIFO_DEPTH - i2c_read(id, I2C_TXFLR);
			i2c_update(id, I2C_TXTL, 0x3f, i2c->remain > I2C_FIFO_DEPTH ? 32 : 0);
			jz_i2c_write_tx_fifo(i2c, n, 0);
		}
	}

	if (st & I2C_INT_RXFL) {
		int i;

		n = i2c_read(id, I2C_RXFLR);
		i2c->remain -= n;
		for (i = n; i > 0; i--)
			*i2c->rbuf++ = i2c_read(id, I2C_DC);
		if (i2c->remain <= 0) {
			complete(&i2c->done);
			return IRQ_HANDLED;
		}
		i2c_update(id, I2C_RXTL, 0x3f,
			   i2c->remain > I2C_FIFO_DEPTH ? 31 : i2c->remain - 1);
		jz_i2c_write_rx_fifo(i2c, n, 0);
	}

	if (st & I2C_INT_STP) {
		i2c_read(id, I2C_CSTP);
		complete(&i2c->done);
	}
	return IRQ_HANDLED;
}

// SCL high and low counts of half a period each, at least 6 and 8; spike
// filter and SDA hold of a quarter period.
static void jz_i2c_hal_set_speed(int id)
{
	struct jz_i2c *i2c = &jz_i2c_dev[id];
	unsigned long rate = clk_get_rate(i2c->clk);
	u32 speed = i2c->rate <= 100000 ? 100000 : 400000;
	u32 half = rate / (speed * 2);
	u32 quarter = rate / (speed * 4);
	u32 spk;

	i2c->rate = speed;
	if (quarter < 2)
		spk = 1;
	else
		spk = quarter - 1 < 256 ? quarter - 1 : 255;

	if (speed == 100000) {
		i2c_update(id, I2C_CON, I2C_CON_SPEED_MASK, I2C_CON_SPEED_STD);
		i2c_update(id, I2C_SHCNT, 0xffff, half < 6 ? 6 : half);
		i2c_update(id, I2C_SLCNT, 0xffff, half < 8 ? 8 : half);
	} else {
		i2c_update(id, I2C_CON, I2C_CON_SPEED_MASK, I2C_CON_SPEED_FAST);
		i2c_update(id, I2C_FHCNT, 0xffff, half < 6 ? 6 : half);
		i2c_update(id, I2C_FLCNT, 0xffff, half < 8 ? 8 : half);
	}
	i2c_update(id, I2C_FSSPKLEN, 0xff, spk);
	i2c_update(id, I2C_SDAHD, 0xffff, quarter);
}

static void jz_i2c_enable(int id)
{
	clk_enable(jz_i2c_dev[id].clk);
	jz_i2c_hal_disable_smb(id);
	while (i2c_read(id, I2C_ENSTA) & 1)
		;
	jz_i2c_hal_set_speed(id);
	i2c_write(id, I2C_INTM, 0);
	i2c_update(id, I2C_ENB, 1, 1);
}

// Waits up to 10 s for the master to go idle, then switches the controller
// and its clock off.
static void jz_i2c_disable(int id)
{
	int tries = 1000000;

	while (i2c_read(id, I2C_STA) & I2C_STA_MSTACT) {
		if (!--tries) {
			printk(KERN_ERR "i2c%d disable timeout!\n", id);
			break;
		}
		udelay(10);
	}
	i2c_write(id, I2C_INTM, 0);
	jz_i2c_hal_disable_smb(id);
	while (i2c_read(id, I2C_ENSTA) & 1)
		;
	clk_disable(jz_i2c_dev[id].clk);
}

static void jz_i2c_report_abort(struct jz_i2c *i2c, const char *what)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(abrt_src); i++)
		if ((i2c->abort >> i) & 1)
			printk(KERN_ERR "i2c%d TXABRT[%d]=%s\n", i2c->id, i, abrt_src[i]);
	printk(KERN_DEBUG "I2C DEVICE %s MAYBE NO SUPPORT RESTART CMD\n", what);
	i2c->abort = 0;
}

// Waits for the interrupt handler to finish the message; on a timeout the
// controller is reset.
static int jz_i2c_wait(struct jz_i2c *i2c, int timeout, const char *what)
{
	int ms = timeout / 1000 + 1;
	int ret = 0;

	if (!wait_for_completion_timeout(&i2c->done, msecs_to_jiffies(ms))) {
		printk(KERN_ERR "i2c%d %s timeout(%d ms)\n", i2c->id, what, ms);
		jz_i2c_disable(i2c->id);
		udelay(10);
		jz_i2c_enable(i2c->id);
		ret = -ETIMEDOUT;
	}
	return ret;
}

static int jz_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msg, int num)
{
	struct jz_i2c *i2c = adap->algo_data;
	int id, i, ret = 0;

	jz_i2c_enable(i2c->id);
	for (i = 0; i < num; i++, msg++) {
		int flags = MSG_FIRST;
		int timeout = 0;
		u32 intm;

		if (i)
			flags = msg->flags & I2C_M_NOSTART ? 0 : MSG_RESTART;
		if (i == num - 1)
			flags |= MSG_LAST;
		id = i2c->id;
		i2c_update(id, I2C_TAR, I2C_TAR_10BIT, msg->flags & I2C_M_TEN ? I2C_TAR_10BIT : 0);
		if (msg->len)
			timeout = (msg->len + 1) * 18000000U / (jz_i2c_dev[id].rate / 1000);

		if (msg->flags & I2C_M_RD) {
			intm = i2c_read(id, I2C_INTM);
			if (!msg->len)
				continue;
			i2c->rx_cmds = msg->len;
			i2c->remain = msg->len;
			i2c->rbuf = msg->buf;
			i2c->flags = flags;
			reinit_completion(&i2c->done);
			i2c_read(id, I2C_CSTP);
			i2c_read(id, I2C_CTXABRT);
			i2c_update(id, I2C_RXTL, 0x3f, msg->len > I2C_FIFO_DEPTH ? 31 : msg->len - 1);
			if (flags & MSG_FIRST)
				i2c_write(id, I2C_TAR, msg->addr);
			jz_i2c_write_rx_fifo(i2c, I2C_FIFO_DEPTH, 1);
			intm |= I2C_INT_RXFL | I2C_INT_TXABT;
			if (flags & MSG_LAST)
				intm |= I2C_INT_STP;
			i2c_write(id, I2C_INTM, intm);
			ret = jz_i2c_wait(i2c, timeout, "read");
			if (jz_i2c_dev[id].abort) {
				jz_i2c_report_abort(i2c, "Read");
				ret = -EIO;
			}
		} else {
			intm = i2c_read(id, I2C_INTM);
			i2c->flags = flags;
			i2c->wbuf = msg->buf;
			i2c->remain = msg->len;
			reinit_completion(&i2c->done);
			i2c_read(id, I2C_CSTP);
			i2c_read(id, I2C_CTXABRT);
			i2c_update(id, I2C_TXTL, 0x3f, 0);
			if (flags & MSG_FIRST)
				i2c_write(id, I2C_TAR, msg->addr);
			jz_i2c_write_tx_fifo(i2c, I2C_FIFO_DEPTH, 1);
			intm |= I2C_INT_TXEMP | I2C_INT_TXABT;
			if (i2c->flags & MSG_LAST)
				intm |= I2C_INT_STP;
			i2c_write(id, I2C_INTM, intm);
			ret = jz_i2c_wait(i2c, timeout, "write");
			if (jz_i2c_dev[id].abort) {
				jz_i2c_report_abort(i2c, "Write");
				ret = -EIO;
			}
		}
		if (ret)
			break;
	}
	jz_i2c_disable(i2c->id);
	return ret ? ret : i;
}

static const struct i2c_algorithm soc_i2c_algorithm = {
	.master_xfer = jz_i2c_transfer,
	.functionality = jz_i2c_functionality,
};

// The pin entries of the parameters; SCL and SDA are looked up separately.
static int jz_i2c_find_pins(struct jz_i2c *i2c)
{
	char buf[16];
	int i;

	for (i = 0; i < i2c->pin_count; i++)
		if (i2c->pins[i].scl == i2c->scl)
			break;
	if (i == i2c->pin_count) {
		printk(KERN_ERR "I2C%d scl(%s) is invaild\n", i2c->id, gpio_to_str(i2c->scl, buf));
		return -EINVAL;
	}
	i2c->scl_func = i2c->pins[i].scl_func;

	for (i = 0; i < i2c->pin_count; i++)
		if (i2c->pins[i].sda == i2c->sda)
			break;
	if (i == i2c->pin_count) {
		printk(KERN_ERR "I2C%d sda(%s) is invaild\n", i2c->id, gpio_to_str(i2c->sda, buf));
		return -EINVAL;
	}
	i2c->sda_func = i2c->pins[i].sda_func;
	return 0;
}

static int jz_i2c_init(int id)
{
	struct jz_i2c *i2c = &jz_i2c_dev[id];
	struct clk *apb;
	unsigned long apb_rate = 0;
	u32 v;
	int ret;

	ret = jz_i2c_find_pins(i2c);
	if (ret) {
		printk(KERN_ERR "i2c%d modules parameter is invaild\n", id);
		goto out;
	}

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &soc_i2c_algorithm;
	i2c->adap.retries = 5;
	i2c->adap.timeout = 5;
	i2c->adap.nr = i2c->id;
	i2c->adap.algo_data = i2c;
	sprintf(i2c->adap.name, "i2c%u", i2c->id);

	i2c->clk = clk_get(NULL, i2c->clk_name);
	if (IS_ERR(i2c->clk)) {
		printk(KERN_ERR "i2c%d get clock(%s) failed\n", id, i2c->clk_name);
		ret = PTR_ERR(i2c->clk);
		goto out;
	}
	ret = clk_prepare(i2c->clk);
	if (ret < 0) {
		printk(KERN_ERR "i2c%d  prepare clock(%s) failed\n", id, i2c->clk_name);
		goto put_clk;
	}
	ret = gpio_request(i2c->scl, i2c->scl_label);
	if (ret < 0)
		goto gpio_err;
	gpio_set_func(i2c->scl, i2c->scl_func);
	ret = gpio_request(i2c->sda, i2c->sda_label);
	if (ret < 0) {
		gpio_free(i2c->scl);
		goto gpio_err;
	}
	gpio_set_func(i2c->sda, i2c->sda_func);

	// The controller set up once with its clock on: restarts allowed, 7-bit
	// addresses, the speed, the SDA set-up time from the APB clock.
	clk_enable(i2c->clk);
	jz_i2c_hal_disable_smb(i2c->id);
	i2c_update(i2c->id, I2C_CON, I2C_CON_RESTART_EN, I2C_CON_RESTART_EN);
	while (i2c_read(i2c->id, I2C_ENSTA) & 1)
		;
	i2c_update(i2c->id, I2C_TAR, I2C_TAR_10BIT, 0);
	jz_i2c_hal_set_speed(i2c->id);
	apb = clk_get(NULL, "gate_apb0");
	if (!IS_ERR(apb)) {
		apb_rate = clk_get_rate(apb);
		clk_put(apb);
	}
	v = apb_rate / jz_i2c_dev[i2c->id].rate / 6;
	if (v >= 256)
		v = 255;
	i2c_write(i2c->id, I2C_SDASU, v);
	i2c_write(i2c->id, I2C_INTM, 0);
	clk_disable(i2c->clk);

	init_completion(&i2c->done);
	ret = request_threaded_irq(i2c->irq, i2c_intr_handler, NULL, 0, i2c->irq_name, i2c);
	if (ret < 0) {
		printk(KERN_ERR "i2c%d Failed to request irq\n", id);
		goto free_gpio;
	}
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_ERR "i2c%d Failed to add bus\n", id);
		free_irq(i2c->irq, i2c);
		goto free_gpio;
	}
	i2c->inited = 1;
	return 0;

free_gpio:
	gpio_free(i2c->scl);
	gpio_free(i2c->sda);
	clk_unprepare(i2c->clk);
	goto put_clk;
gpio_err:
	printk(KERN_ERR "i2c%d gpio requeset failed\n", id);
	clk_unprepare(i2c->clk);
put_clk:
	clk_put(i2c->clk);
out:
	i2c->inited = 0;
	return ret;
}

static void jz_i2c_deinit(int id)
{
	struct jz_i2c *i2c = &jz_i2c_dev[id];

	i2c_del_adapter(&i2c->adap);
	free_irq(i2c->irq, i2c);
	gpio_free(i2c->scl);
	gpio_free(i2c->sda);
	clk_unprepare(i2c->clk);
	clk_put(i2c->clk);
}

static int __init jz_arch_i2c_init(void)
{
	if (jz_i2c_dev[0].is_enable)
		jz_i2c_init(0);
	if (jz_i2c_dev[1].is_enable)
		jz_i2c_init(1);
	return 0;
}

static void __exit jz_arch_i2c_exit(void)
{
	if (jz_i2c_dev[0].inited)
		jz_i2c_deinit(0);
	if (jz_i2c_dev[1].inited)
		jz_i2c_deinit(1);
}

module_init(jz_arch_i2c_init);
module_exit(jz_arch_i2c_exit);

MODULE_DESCRIPTION("X1600 SoC I2C driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
