// SPDX-License-Identifier: GPL-2.0
//
// utils -- helpers the other HiBy modules link against: pin names, module
// parameter types, word splitting, I2C and SPI device registration, clocks
// in microseconds and milliseconds, and the /dev/log_manager ring buffer.
//
// A drop-in replacement for the vendor module of the same name, loaded first
// by driver_default_init_script.sh, with the same parameter:
//
//   buffer_size   bytes of the log_manager ring, allocated at the first open
//
// Pin names are "PA09", "pa9", "gpio_pb(12)" and the like: an optional
// "gpio_" before the P, a port letter A..G, a pin 0..31 with or without
// parentheses. "-1" stands for no pin. Ports A..G are pins 0..223.

#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <soc/gpio.h>

// --- assert -----------------------------------------------------------------

void __assert(const char *expr, const char *file, int line, const char *func)
{
	panic("assert %s failed in %s %d %s\n", expr, file, line, func);
}
EXPORT_SYMBOL(__assert);

#define UTILS_ASSERT(x) \
	do { if (!(x)) __assert(#x, __FILE__, __LINE__, __func__); } while (0)

// --- strings ----------------------------------------------------------------

// Case-insensitive strcmp().
int stricmp(const char *a, const char *b)
{
	int c1, c2;

	do {
		c1 = tolower(*a++);
		c2 = tolower(*b++);
		if (c1 != c2)
			return c1 - c2;
	} while (c1);
	return 0;
}
EXPORT_SYMBOL(stricmp);

// 0 when prefix starts str, case-insensitively; otherwise the difference of
// the first two characters that differ, prefix minus str.
int strimatch(const char *str, const char *prefix)
{
	int c1, c2;

	do {
		c1 = tolower(*str++);
		c2 = tolower(*prefix++);
		if (c1 != c2)
			return c2 ? c2 - c1 : 0;
	} while (c1);
	return 0;
}
EXPORT_SYMBOL(strimatch);

char *str_first_space(const char *s)
{
	while (*s && !isspace(*s))
		s++;
	return (char *)s;
}
EXPORT_SYMBOL(str_first_space);

char *str_first_not_space(const char *s)
{
	while (*s && isspace(*s))
		s++;
	return (char *)s;
}
EXPORT_SYMBOL(str_first_not_space);

// The start of the trailing white space, or the terminating NUL.
char *str_first_tail_space(const char *s)
{
	const char *p = s + strlen(s) - 1;

	while (p >= s && isspace(*p))
		p--;
	return (char *)(p + 1);
}
EXPORT_SYMBOL(str_first_tail_space);

// The first non-space character of s; *end gets the start of the trailing
// white space. s is not modified.
char *strip(char *s, char **end)
{
	char *e;

	while (*s && isspace(*s))
		s++;
	e = s + strlen(s) - 1;
	while (e >= s && isspace(*e))
		e--;
	*end = e + 1;
	return s;
}
EXPORT_SYMBOL(strip);

// Copies the first word of s to out (when not NULL) and returns the end of
// it in s. Words are separated by white space; '...' and "..." quote spaces,
// and \" is a double quote (kept as \" inside '...').
static const char *str_get_first_word(const char *s, char *out)
{
	char quote = 0;

	for (s = str_first_not_space(s); *s; s++) {
		char c = *s;

		if (!quote) {
			if (isspace(c))
				break;
			if (c == '\'' || c == '"') {
				quote = c;
				continue;
			}
		} else if (c == quote) {
			quote = 0;
			continue;
		}
		if (c == '\\' && s[1] == '"') {
			if (out) {
				if (quote == '\'')
					*out++ = '\\';
				*out++ = '"';
			}
			s++;
			continue;
		}
		if (out)
			*out++ = c;
	}
	if (out)
		*out = 0;
	return s;
}

// Splits str into words: a NULL-terminated array, freed with
// str_free_words(), its strings in the same allocation. *count, when not
// NULL, gets the number of words.
char **str_to_words(const char *str, int *count)
{
	const char *p = str, *q;
	char **array, *buf;
	int n = 0, total = 0, i;

	for (;;) {
		p = str_first_not_space(p);
		if (!*p)
			break;
		q = str_get_first_word(p, NULL);
		total += q - p + 1;
		p = q;
		n++;
	}

	array = kmalloc((n + 1) * sizeof(char *) + total, GFP_KERNEL);
	UTILS_ASSERT(array);
	buf = (char *)(array + n + 1);
	for (i = 0; i < n; i++) {
		array[i] = buf;
		str = str_get_first_word(str, buf);
		buf += strlen(buf) + 1;
	}
	array[n] = NULL;
	if (count)
		*count = n;
	return array;
}
EXPORT_SYMBOL(str_to_words);

void str_free_words(char **words)
{
	kfree(words);
}
EXPORT_SYMBOL(str_free_words);

// --- pins -------------------------------------------------------------------

// Set by the platform device "ingenic-utils-gpiodev" when one registers;
// otherwise the X1600 pin controller is used.
struct utils_gpio_ops {
	int (*set_func)(int port, int func, int pin);
	int (*port_set_func)(int port, int func, unsigned int pins);
};

static struct utils_gpio_ops *gpio_ops;

int gpio_set_func(int gpio, int func)
{
	if (gpio_ops && gpio_ops->set_func)
		return gpio_ops->set_func(gpio / 32, func, gpio % 32);
	return jzgpio_set_func(gpio / 32, func, 1 << (gpio % 32));
}
EXPORT_SYMBOL(gpio_set_func);

// Sets func on every pin of port in the mask pins; stops at the first error.
int gpio_port_set_func(int port, unsigned int pins, int func)
{
	int i, ret;

	if (gpio_ops && gpio_ops->port_set_func)
		return gpio_ops->port_set_func(port, func, pins);
	for (i = 0; i < 32 && pins >> i; i++) {
		if (!((pins >> i) & 1))
			continue;
		ret = jzgpio_set_func(port, func, 1 << i);
		if (ret)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL(gpio_port_set_func);

// Parses a pin name at *str and moves *str past it. Returns the pin number,
// -1 for "-1", or -EINVAL.
int str_match_gpio(const char **str)
{
	const char *s = *str;
	int c, port, pin, paren;

	if (!strncmp(s, "-1", 2)) {
		*str = s + 2;
		return -1;
	}
	if (!strimatch(s, "gpio_p"))
		s += 6;
	else if (!strimatch(s, "p"))
		s += 1;
	else
		return -EINVAL;

	c = tolower(*s);
	if (c < 'a' || c > 'g')
		return -EINVAL;
	port = c - 'a';

	paren = s[1] == '(';
	s += paren ? 2 : 1;
	if (!isdigit(*s))
		return -EINVAL;
	pin = *s - '0';
	if (isdigit(s[1])) {
		pin = pin * 10 + s[1] - '0';
		if (pin >= 32)
			return -EINVAL;
		s += 2;
	} else {
		s += 1;
	}
	if (paren) {
		if (*s != ')')
			return -EINVAL;
		s++;
	}
	*str = s;
	return port * 32 + pin;
}
EXPORT_SYMBOL(str_match_gpio);

// A whole string as a pin name: the pin, -1, or -EINVAL.
int str_to_gpio(const char *str)
{
	const char *s = str;
	int gpio = str_match_gpio(&s);

	if (gpio < -1)
		return -EINVAL;
	return *s ? -EINVAL : gpio;
}
EXPORT_SYMBOL(str_to_gpio);

// "PA09", "-1" or "error" into buf, or into a new allocation when buf is
// NULL.
char *gpio_to_str(int gpio, char *buf)
{
	if (!buf) {
		buf = kmalloc(16, GFP_KERNEL);
		if (!buf)
			return NULL;
	}
	if (gpio == -1)
		sprintf(buf, "-1");
	else if (gpio < -1 || gpio >= 7 * 32)
		sprintf(buf, "error");
	else
		sprintf(buf, "P%c%02d", 'A' + (gpio >> 5), gpio & 31);
	return buf;
}
EXPORT_SYMBOL(gpio_to_str);

// A module parameter holding a pin, given and shown by name.
static int param_gpio_set(const char *val, const struct kernel_param *kp)
{
	int gpio = str_to_gpio(val);

	if (gpio < -1)
		return gpio;
	*(int *)kp->arg = gpio;
	return 0;
}

static int param_gpio_get(char *buffer, const struct kernel_param *kp)
{
	gpio_to_str(*(int *)kp->arg, buffer);
	return strlen(buffer);
}

const struct kernel_param_ops param_gpio_ops = {
	.set = param_gpio_set,
	.get = param_gpio_get,
};
EXPORT_SYMBOL(param_gpio_ops);

static int gpio_plat_probe(struct platform_device *pdev)
{
	gpio_ops = dev_get_platdata(&pdev->dev);
	return 0;
}

static int gpio_plat_remove(struct platform_device *pdev)
{
	gpio_ops = NULL;
	return 0;
}

static struct platform_driver gpio_drv = {
	.probe = gpio_plat_probe,
	.remove = gpio_plat_remove,
	.driver = {
		.name = "ingenic-utils-gpiodev",
		.owner = THIS_MODULE,
	},
};

// --- regulator parameters ---------------------------------------------------

// A module parameter of the PMU drivers, given as words:
//   regulator_name=NAME boot_on=enable|disable work_voltage=N suspend_volage=N
// regulator_name points into the parsed words, which are kept.
struct pmu_param {
	const char *regulator_name;
	int boot_on;
	unsigned int work_voltage;
	unsigned int suspend_voltage;
};

static int param_pmu_set(const char *val, const struct kernel_param *kp)
{
	struct pmu_param *pmu = kp->arg;
	char **words, **w;
	int count, i;

	words = str_to_words(val, &count);
	for (i = 0, w = words; i < count; i++, w++) {
		if (!strncmp(*w, "regulator_name=", strlen("regulator_name=")))
			pmu->regulator_name = *w + strlen("regulator_name=");
		if (!strncmp(*w, "boot_on=", strlen("boot_on="))) {
			if (!strcmp(*w + strlen("boot_on="), "enable"))
				pmu->boot_on = 1;
			if (!strcmp(*w + strlen("boot_on="), "disable"))
				pmu->boot_on = 0;
		}
		if (!strncmp(*w, "work_voltage=", strlen("work_voltage=")) &&
		    kstrtouint(*w + strlen("work_voltage="), 0, &pmu->work_voltage)) {
			pr_err("Try to set a invalid work_voltage\n");
			goto err;
		}
		if (!strncmp(*w, "suspend_volage=", strlen("suspend_volage=")) &&
		    kstrtouint(*w + strlen("suspend_volage="), 0, &pmu->suspend_voltage)) {
			pr_err("Try to set a invalid suspend_volage\n");
			goto err;
		}
	}
	return 0;
err:
	str_free_words(words);
	return -EINVAL;
}

static int param_pmu_get(char *buffer, const struct kernel_param *kp)
{
	struct pmu_param *pmu = kp->arg;

	if (!buffer)
		buffer = kmalloc(128, GFP_KERNEL);
	sprintf(buffer, "%s :regulator_name=%s, boot_on=%d, work_voltage:%d, suspend_voltage:%d\n",
		kp->name, pmu->regulator_name, pmu->boot_on, pmu->work_voltage,
		pmu->suspend_voltage);
	return strlen(buffer);
}

const struct kernel_param_ops param_pmu_ops = {
	.set = param_pmu_set,
	.get = param_pmu_get,
};
EXPORT_SYMBOL(param_pmu_ops);

// --- devices ----------------------------------------------------------------

struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum)
{
	struct i2c_adapter *adap = i2c_get_adapter(busnum);
	struct i2c_client *client;

	if (!adap) {
		pr_err("error: failed to get i2c adapter %d\n", busnum);
		return NULL;
	}
	client = i2c_new_device(adap, info);
	i2c_put_adapter(adap);
	return client;
}
EXPORT_SYMBOL(i2c_register_device);

static DEFINE_MUTEX(spi_lock);

// Registers info on SPI bus busnum, on the first chip select without a
// device "spiB.C" yet.
struct spi_device *spi_register_device(struct spi_board_info *info, int busnum)
{
	struct spi_master *master = spi_busnum_to_master(busnum);
	struct spi_device *spi = NULL;
	struct device *dev;
	char name[32];
	int cs;

	mutex_lock(&spi_lock);
	if (!master) {
		pr_err("error: failed to get spi master %d\n", busnum);
		goto out;
	}
	for (cs = 0; cs < master->num_chipselect + 1; cs++) {
		sprintf(name, "spi%d.%d", busnum, cs);
		dev = bus_find_device_by_name(&spi_bus_type, NULL, name);
		if (!dev) {
			info->chip_select = cs;
			break;
		}
		put_device(dev);
	}
	if (cs < master->num_chipselect) {
		info->bus_num = busnum;
		spi = spi_new_device(master, info);
		if (!spi)
			pr_err("can not register spi device to %d busnum!", busnum);
	}
	put_device(&master->dev);
out:
	mutex_unlock(&spi_lock);
	return spi;
}
EXPORT_SYMBOL(spi_register_device);

// --- clocks -----------------------------------------------------------------

u64 local_clock_us(void)
{
	u64 t = local_clock();

	do_div(t, 1000);
	return t;
}
EXPORT_SYMBOL(local_clock_us);

u64 local_clock_ms(void)
{
	u64 t = local_clock();

	do_div(t, 1000000);
	return t;
}
EXPORT_SYMBOL(local_clock_ms);

// --- /dev/log_manager -------------------------------------------------------

// A ring of records, each a struct log_hdr and len bytes. A write appends one
// record (at most LOG_MSG_MAX bytes) at m_end, overwriting the oldest records
// as needed, and leaves a record header for the free space behind it. Every
// open file reads from its own position, starting at the oldest record, one
// record per read, as "SECONDS.MICROSECONDS:TEXT\n"; a read at the end waits
// for a write.
#define LOG_MAGIC	0x2be5
#define LOG_MSG_MAX	1024

struct log_hdr {
	u32 len;
	u16 magic;
	u64 ts;			// local_clock_us() at the write
};

static int buffer_size = 0x20000;
module_param(buffer_size, int, 0644);

static struct {
	char *buf;
	int size;
} ring;

static char *log_buffer;
static int m_start, m_end;
static DECLARE_WAIT_QUEUE_HEAD(log_wait);
static DEFINE_MUTEX(log_lock);
static char log_msg[LOG_MSG_MAX + 1];
static char log_line[LOG_MSG_MAX + 32];

static int ring_write(int pos, const void *src, int n)
{
	int off = pos % ring.size;
	int end = off + n;

	if (end > ring.size) {
		memcpy(ring.buf + off, src, ring.size - off);
		memcpy(ring.buf, src + ring.size - off, n - (ring.size - off));
	} else {
		memcpy(ring.buf + off, src, n);
	}
	return end % ring.size;
}

static int ring_read(int pos, void *dst, int n)
{
	int off = pos % ring.size;
	int end = off + n;

	if (end > ring.size) {
		memcpy(dst, ring.buf + off, ring.size - off);
		memcpy(dst + ring.size - off, ring.buf, n - (ring.size - off));
	} else {
		memcpy(dst, ring.buf + off, n);
	}
	return end % ring.size;
}

static int write_msg(int pos, u32 len)
{
	struct log_hdr hdr = { .len = len, .magic = LOG_MAGIC };

	hdr.ts = local_clock_us();
	return ring_write(pos, &hdr, sizeof(hdr));
}

static int log_open(struct inode *inode, struct file *file)
{
	static const char first[] = "-----------\n";
	int *pos = vmalloc(sizeof(*pos));

	if (!pos)
		return -ENOMEM;
	mutex_lock(&log_lock);
	if (!log_buffer) {
		log_buffer = vmalloc(buffer_size);
		if (!log_buffer) {
			mutex_unlock(&log_lock);
			vfree(pos);
			return -ENOMEM;
		}
		ring.buf = log_buffer;
		ring.size = buffer_size;
		m_start = 0;
		m_end = ring_write(write_msg(m_start, sizeof(first)), first, sizeof(first));
		write_msg(m_end, buffer_size - m_end - sizeof(struct log_hdr));
	}
	*pos = m_start;
	mutex_unlock(&log_lock);
	file->private_data = pos;
	return 0;
}

static int log_release(struct inode *inode, struct file *file)
{
	vfree(file->private_data);
	return 0;
}

static ssize_t log_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	int *pos = file->private_data;
	struct log_hdr hdr;
	int at, n;
	u32 rem;
	u64 ts;

	wait_event_interruptible(log_wait, *pos != m_end);
	if (*pos == m_end)
		return -EINTR;

	mutex_lock(&log_lock);
	for (;;) {
		at = ring_read(*pos, &hdr, sizeof(hdr));
		if (hdr.magic == LOG_MAGIC)
			break;
		(*pos)++;
	}
	n = min_t(u32, hdr.len, LOG_MSG_MAX);
	ring_read(at, log_msg, n);
	log_msg[n] = 0;
	*pos = (at + hdr.len) % ring.size;

	ts = hdr.ts;
	rem = do_div(ts, 1000000);
	n = snprintf(log_line, sizeof(log_line), "%llu.%06u:%s\n", ts, rem, log_msg);
	n = min_t(size_t, n, count);
	if (copy_to_user(ubuf, log_line, n))
		n = -EFAULT;
	mutex_unlock(&log_lock);
	return n;
}

static ssize_t log_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	int n = min_t(size_t, count, LOG_MSG_MAX);
	int pos, freed = 0, last = 0;
	bool overwrote = false;
	struct log_hdr hdr;

	mutex_lock(&log_lock);
	if (copy_from_user(log_msg, ubuf, n)) {
		mutex_unlock(&log_lock);
		return -EFAULT;
	}

	// Takes records from m_end on until they make room for this one and a
	// free-space header.
	pos = m_end;
	for (;;) {
		ring_read(pos, &hdr, sizeof(hdr));
		if (hdr.magic != LOG_MAGIC)
			panic("failed to check msg: %x\n", hdr.magic);
		last = freed + hdr.len;
		freed = last + sizeof(hdr);
		pos += hdr.len + sizeof(hdr);
		if (freed >= n + 2 * sizeof(hdr))
			break;
		overwrote = true;
	}

	m_end = ring_write(write_msg(m_end, n), log_msg, n);
	write_msg(m_end, last - n - sizeof(hdr));
	if (overwrote)
		m_start = pos;
	wake_up_all(&log_wait);
	mutex_unlock(&log_lock);
	return n;
}

static long log_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static const struct file_operations log_misc_fops = {
	.owner = THIS_MODULE,
	.open = log_open,
	.release = log_release,
	.read = log_read,
	.write = log_write,
	.unlocked_ioctl = log_ioctl,
};

static struct miscdevice log_mdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "log_manager",
	.fops = &log_misc_fops,
};

// --- module -----------------------------------------------------------------

static int __init utils_init(void)
{
	int ret;

	ret = misc_register(&log_mdev);
	if (ret)
		return ret;
	ret = platform_driver_register(&gpio_drv);
	if (ret)
		misc_deregister(&log_mdev);
	return ret;
}

static void __exit utils_exit(void)
{
	misc_deregister(&log_mdev);
	platform_driver_unregister(&gpio_drv);
	vfree(log_buffer);
}

module_init(utils_init);
module_exit(utils_exit);

MODULE_DESCRIPTION("HiBy utility functions");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
