// SPDX-License-Identifier: GPL-2.0
//
// soc_gpio -- /dev/gpio: the function and level of X1600 pins from user
// space, by pin name ("PA09", see utils.ko).
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameter (the stock soc_gpio.sh passes it):
//
//   debug   1: log every function set
//
// ioctls, all _IO('G', n), on pins of ports A..D:
//   0x78 GPIO_IOC_SET_FUNC   struct gpio_set_func: OR of the named functions,
//                            at most one of func0..input and one pull, set
//                            with gpio_set_func(); interrupt functions refused
//   0x79 GPIO_IOC_DUMP_FUNC  struct gpio_dump: "FUNC PULL" into buf
//   0x7a GPIO_IOC_GET_VALUE  the pin name itself as argument: its level
//   0x7b GPIO_IOC_HELP       struct gpio_buf: the function names
//   0x7c GPIO_IOC_GET_FUNC   struct gpio_get_func: the function into bufs[0],
//                            the pull into bufs[1] when count > 1; returns
//                            count, at most 3
//   0x7d GPIO_IOC_CLEAR      the pin name: PAT0 cleared (output low)
//   0x7e GPIO_IOC_SET        the pin name: PAT0 set (output high)

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <asm/addrspace.h>
#include <soc/gpio.h>

// Exported by utils.ko.
extern int gpio_set_func(int gpio, int func);
extern int str_to_gpio(const char *str);

#define GPIO_IOC_SET_FUNC	_IO('G', 0x78)
#define GPIO_IOC_DUMP_FUNC	_IO('G', 0x79)
#define GPIO_IOC_GET_VALUE	_IO('G', 0x7a)
#define GPIO_IOC_HELP		_IO('G', 0x7b)
#define GPIO_IOC_GET_FUNC	_IO('G', 0x7c)
#define GPIO_IOC_CLEAR		_IO('G', 0x7d)
#define GPIO_IOC_SET		_IO('G', 0x7e)

#define GPIO_PORT_BASE(port)	((void __iomem *)CKSEG1ADDR(0x10010000 + (port) * 0x100))
#define PXPIN			0x00
#define PXINT			0x10
#define PXMSK			0x20
#define PXPAT1			0x30
#define PXPAT0			0x40
#define PXPAT0S			0x44
#define PXPAT0C			0x48
#define PXEDG			0x70
#define PXPEN			0x80

#define GPIO_MAX		(4 * 32)
#define NAME_MAX_LEN		32

struct gpio_set_func {
	const char __user *gpio;
	int count;
	const char __user *funcs[];
};

struct gpio_dump {
	const char __user *gpio;
	char __user *buf;
	unsigned int len;
};

struct gpio_buf {
	char __user *buf;
	unsigned int len;
};

struct gpio_get_func {
	const char __user *gpio;
	char __user *__user *bufs;
	unsigned int count;
	unsigned int len;
};

struct func_name {
	int value;
	const char *name;
};

static const struct func_name common_func_names[] = {
	{ GPIO_FUNC_0, "func0" },
	{ GPIO_FUNC_1, "func1" },
	{ GPIO_FUNC_2, "func2" },
	{ GPIO_FUNC_3, "func3" },
	{ GPIO_OUTPUT0, "output0" },
	{ GPIO_OUTPUT1, "output1" },
	{ GPIO_INPUT, "input" },
	{ GPIO_INT_LO, "int_lo" },
	{ GPIO_INT_HI, "int_hi" },
	{ GPIO_INT_FE, "int_fe" },
	{ GPIO_INT_RE, "int_re" },
	{ GPIO_INT_RE_FE, "int_re_fe" },
	{ GPIO_INT_MASK_LO, "int_lo_m" },
	{ GPIO_INT_MASK_HI, "int_hi_m" },
	{ GPIO_INT_MASK_FE, "int_fe_m" },
	{ GPIO_INT_MASK_RE, "int_re_m" },
	{ GPIO_INT_MASK_RE_FE, "int_re_fe_m" },
	{ 0, NULL },
};

static const struct func_name pull_func_names[] = {
	{ GPIO_PULL_HIZ, "no pull_hiz" },
	{ GPIO_PULL, "pull" },
	{ 0, NULL },
};

static const char help[] =
	" common_func:\n"
	"    func0: device func0\n"
	"    func1: device func1\n"
	"    func2: device func2\n"
	"    func3: device func3\n"
	"    output0: output 0\n"
	"    output1: output 1\n"
	"    input: input mode\n"
	"    int_lo: interrupt by low level\n"
	"    int_hi: interrupt by high level\n"
	"    int_fe: interrupt by falling edge\n"
	"    int_re: interrupt by rising edge\n"
	"    int_fe_re: interrupt by dual edge (rising & falling edge)\n"
	"    int_lo_m: interrupt by low level, interrupt is masked\n"
	"    int_hi_m: interrupt by high level, interrupt is masked\n"
	"    int_fe_m: interrupt by falling edge, interrupt is masked\n"
	"    int_re_m: interrupt by rising edge, interrupt is masked\n"
	"    int_fe_re_m: interrupt by dual edge(rising & falling edge), interrupt is masked\n"
	" pull_func:\n"
	"    pull_hiz: no pull, keep hiz\n"
	"    pull: pull enable\n";

static int debug;
module_param(debug, int, 0644);

static DEFINE_SPINLOCK(gpio_lock);

static int to_gpio_func(const char *name, const struct func_name *table)
{
	for (; table->name; table++)
		if (!strcmp(name, table->name))
			return table->value;
	return -1;
}

// The pin's function and pull, as enum gpio_function values ORed together.
static int gpio_get_func(int gpio)
{
	void __iomem *base = GPIO_PORT_BASE(gpio / 32);
	u32 bit = BIT(gpio % 32);
	unsigned long flags;
	int func = GPIO_PULL_HIZ | GPIO_FUNC_0;

	spin_lock_irqsave(&gpio_lock, flags);
	if (readl(base + PXINT) & bit)
		func |= 0x08;
	if (readl(base + PXMSK) & bit)
		func |= 0x04;
	if (readl(base + PXPAT1) & bit)
		func |= 0x02;
	if (readl(base + PXPAT0) & bit)
		func |= 0x01;
	if (readl(base + PXEDG) & bit)
		func |= 0x10;
	if (readl(base + PXPEN) & bit)
		func |= GPIO_PULL & ~GPIO_PULL_HIZ;
	spin_unlock_irqrestore(&gpio_lock, flags);
	return func;
}

// prefix and the name of func in table into buf, when it fits in size;
// returns the length written, or 0.
static int dump_func(int func, char *buf, unsigned int size, const char *prefix,
		     const struct func_name *table)
{
	for (; table->name; table++) {
		if (table->value != func)
			continue;
		if (size < strlen(prefix) + strlen(table->name) + 1)
			return 0;
		return sprintf(buf, "%s%s", prefix, table->name);
	}
	return 0;
}

// A pin name from user space: the pin of ports A..D, or a negative error.
static int user_gpio(const char __user *uname)
{
	char name[NAME_MAX_LEN];
	long n = strncpy_from_user(name, uname, sizeof(name));
	int gpio;

	if (n < 0)
		return -EFAULT;
	name[sizeof(name) - 1] = 0;
	gpio = str_to_gpio(name);
	if ((unsigned int)gpio >= GPIO_MAX) {
		pr_err("gpio: %s is not valid\n", name);
		return -EINVAL;
	}
	return gpio;
}

static long gpio_set_funcs(struct gpio_set_func __user *uarg)
{
	char name[NAME_MAX_LEN];
	const char __user *uname;
	int gpio, count, func = 0, v, i;

	if (get_user(uname, &uarg->gpio) || get_user(count, &uarg->count))
		return -EFAULT;
	gpio = user_gpio(uname);
	if (gpio < 0)
		return gpio;
	for (i = 0; i < count; i++) {
		if (get_user(uname, &uarg->funcs[i]) ||
		    strncpy_from_user(name, uname, sizeof(name)) < 0)
			return -EFAULT;
		name[sizeof(name) - 1] = 0;
		v = to_gpio_func(name, common_func_names);
		if (v == -1)
			v = to_gpio_func(name, pull_func_names);
		if (v == -1) {
			pr_err("gpio: error unknown func: %s\n", name);
			return -EINVAL;
		}
		if (v >= GPIO_INT_LO && v < GPIO_INT_LO + 24) {
			pr_err("gpio: can't set as irq func: %s\n", name);
			return -EINVAL;
		}
		if (func & v & GPIO_FUNC_0) {
			pr_err("gpio: %s func double set\n", "common");
			return -EINVAL;
		}
		if (func & v & GPIO_PULL_HIZ) {
			pr_err("gpio: %s func double set\n", "pull");
			return -EINVAL;
		}
		func |= v;
	}
	if (debug)
		pr_err("gpio: set func %d %x\n", gpio, func);
	gpio_set_func(gpio, func);
	return 0;
}

static long gpio_dump_func(struct gpio_dump __user *uarg)
{
	struct gpio_dump arg;
	char buf[NAME_MAX_LEN];
	int gpio, func, n;

	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;
	gpio = user_gpio(arg.gpio);
	if (gpio < 0)
		return gpio;
	if (arg.len < 2)
		return -EINVAL;
	func = gpio_get_func(gpio);
	n = dump_func(func & 0x1ff, buf, arg.len, "", common_func_names);
	if (!n)
		return 0;
	n += dump_func(func & 0xe000, buf + n, arg.len - n, " ", pull_func_names);
	if (copy_to_user(arg.buf, buf, n + 1))
		return -EFAULT;
	return 0;
}

static long gpio_get_funcs(struct gpio_get_func __user *uarg)
{
	struct gpio_get_func arg;
	char __user *ubuf;
	char buf[NAME_MAX_LEN];
	int gpio, func, n;

	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;
	gpio = user_gpio(arg.gpio);
	if (gpio < 0)
		return gpio;
	if (arg.len < 2 || !arg.count)
		return -EINVAL;
	func = gpio_get_func(gpio);

	if (get_user(ubuf, &arg.bufs[0]))
		return -EFAULT;
	n = dump_func(func & 0x1ff, buf, arg.len, "", common_func_names);
	if (n && copy_to_user(ubuf, buf, n + 1))
		return -EFAULT;
	if (arg.count != 1) {
		if (get_user(ubuf, &arg.bufs[1]))
			return -EFAULT;
		n = dump_func(func & 0xe000, buf, arg.len, "", pull_func_names);
		if (n && copy_to_user(ubuf, buf, n + 1))
			return -EFAULT;
	}
	return min(arg.count, 3U);
}

static long gpio_help(struct gpio_buf __user *uarg)
{
	struct gpio_buf arg;
	unsigned int n;

	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;
	if (arg.len < 2)
		return -EINVAL;
	n = min_t(unsigned int, arg.len - 1, sizeof(help) - 1);
	if (copy_to_user(arg.buf, help, n) || put_user(0, arg.buf + n))
		return -EFAULT;
	return 0;
}

static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	int gpio;

	switch (cmd) {
	case GPIO_IOC_SET_FUNC:
		return gpio_set_funcs(uarg);
	case GPIO_IOC_DUMP_FUNC:
		return gpio_dump_func(uarg);
	case GPIO_IOC_GET_FUNC:
		return gpio_get_funcs(uarg);
	case GPIO_IOC_HELP:
		return gpio_help(uarg);
	case GPIO_IOC_GET_VALUE:
	case GPIO_IOC_CLEAR:
	case GPIO_IOC_SET:
		gpio = user_gpio(uarg);
		if (gpio < 0)
			return gpio;
		if (cmd == GPIO_IOC_GET_VALUE)
			return !!(readl(GPIO_PORT_BASE(gpio / 32) + PXPIN) & BIT(gpio % 32));
		writel(BIT(gpio % 32), GPIO_PORT_BASE(gpio / 32) +
		       (cmd == GPIO_IOC_SET ? PXPAT0S : PXPAT0C));
		return 0;
	default:
		pr_err("gpio: do not support this cmd: %x\n", cmd);
		return -ENODEV;
	}
}

static int gpio_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int gpio_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations gpio_misc_fops = {
	.owner = THIS_MODULE,
	.open = gpio_open,
	.release = gpio_release,
	.unlocked_ioctl = gpio_ioctl,
};

static struct miscdevice gpio_mdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpio",
	.fops = &gpio_misc_fops,
};

static int __init gpio_init(void)
{
	return misc_register(&gpio_mdev);
}

static void __exit gpio_exit(void)
{
	misc_deregister(&gpio_mdev);
}

module_init(gpio_init);
module_exit(gpio_exit);

MODULE_DESCRIPTION("JZ x1600 gpio driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
