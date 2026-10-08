// SPDX-License-Identifier: GPL-2.0
//
// soc_adc -- the SAR ADC of the Ingenic X1600: four auxiliary channels, read
// by the key, jack and headset remote modules through adc_enable(),
// adc_disable() and adc_read_channel_voltage(), and by user space through
// /dev/jz_adc_aux_0..3.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameter (the stock soc_adc.sh passes it):
//
//   adc_vref   reference voltage in mV (3300)
//
// A conversion enables its channel and waits for its interrupt, 20 ms at
// most; the result is 12 bits, so a voltage is raw * adc_vref / 4096.
//
// The misc devices, one per channel:
//   read                  the voltage in mV, 4 bytes
//   ioctl 0x80044121      set adc_vref from *arg
//   ioctl 0xc004412c      returns adc_vref
//   ioctl 0xc004413c      returns the raw value of the channel
//   ioctl 0x2000410b, 0x20004116   accepted, nothing done

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/addrspace.h>

#define ADC_BASE		((void __iomem *)CKSEG1ADDR(0x10070000))
#define ADC_ENA			0x00	// bits 3..0 start a channel, bit 15 powers off
#define ADC_CTRL		0x08	// bits 3..0 mask the channel interrupts
#define ADC_STATUS		0x0c	// bits 3..0 channel done, write 1 to clear
#define ADC_DATA01		0x10	// channel 0 bits 11..0, channel 1 bits 27..16
#define ADC_DATA23		0x14
#define ADC_CLKDIV		0x20
#define ADC_STABLE		0x24

#define ADC_ENA_POWER_OFF	BIT(15)
#define ADC_CHANNELS		4
#define ADC_IRQ			19
#define ADC_TRIES		100000
#define ADC_TIMEOUT		2	// jiffies

#define ADC_IOC_SET_VREF	0x80044121
#define ADC_IOC_GET_VREF	0xc004412c
#define ADC_IOC_READ_RAW	0xc004413c
#define ADC_IOC_NOP_0		0x2000410b
#define ADC_IOC_NOP_1		0x20004116

static int adc_vref = 3300;
module_param_named(adc_vref, adc_vref, int, 0644);

static int adc_busy;	// adc_enable() calls not yet balanced

static struct {
	struct clk *clk;
	struct mutex mutex;
	unsigned int waiting;	// bit of the channel being converted
	wait_queue_head_t wq;
	struct miscdevice *mdev;
} adc_device;

static DEFINE_SPINLOCK(adc_hal_lock);

// --- registers --------------------------------------------------------------

static inline u32 adc_readl(unsigned int reg)
{
	return __raw_readl(ADC_BASE + reg);
}

static inline void adc_writel(u32 val, unsigned int reg)
{
	__raw_writel(val, ADC_BASE + reg);
}

static void adc_set_bits(unsigned int reg, int start, int end, u32 val)
{
	u32 mask = ((1u << (end - start + 1)) - 1) << start;
	u32 old = adc_readl(reg);

	adc_writel((((val << start) ^ old) & mask) ^ old, reg);
}

static u32 adc_get_bits(unsigned int reg, int start, int end)
{
	u32 mask = ((1u << (end - start + 1)) - 1) << start;

	return (adc_readl(reg) & mask) >> start;
}

static void adc_hal_disable_controller(void)
{
	int i = ADC_TRIES;

	do {
		adc_writel(adc_readl(ADC_ENA) | ADC_ENA_POWER_OFF, ADC_ENA);
		if (adc_readl(ADC_ENA) & ADC_ENA_POWER_OFF)
			return;
	} while (--i);
	printk(KERN_ERR "adc hal disable timeout\n");
}

static void adc_hal_enable_controller(void)
{
	int i = ADC_TRIES;

	do {
		adc_writel(adc_readl(ADC_ENA) & ~ADC_ENA_POWER_OFF, ADC_ENA);
		udelay(2000);
		if (!(adc_readl(ADC_ENA) & ADC_ENA_POWER_OFF))
			break;
	} while (--i);
	usleep_range(2000, 2000);
	if (!i)
		printk(KERN_ERR "adc hal enable timeout\n");
}

static void adc_hal_enable_channel(int ch)
{
	unsigned long flags;
	int i = ADC_TRIES;

	spin_lock_irqsave(&adc_hal_lock, flags);
	do {
		adc_set_bits(ADC_ENA, ch, ch, 1);
		if (adc_get_bits(ADC_ENA, ch, ch))
			break;
	} while (--i);
	spin_unlock_irqrestore(&adc_hal_lock, flags);
	if (!i)
		printk(KERN_ERR "adc enable channel %d timeout\n", ch);
}

static void adc_hal_disable_channel(int ch)
{
	do
		adc_set_bits(ADC_ENA, ch, ch, 0);
	while (adc_get_bits(ADC_ENA, ch, ch));
}

static void adc_hal_enable_all_interrupt(void)
{
	adc_writel(adc_readl(ADC_CTRL) & ~0xf, ADC_CTRL);
}

static void adc_hal_mask_all_interrupt(void)
{
	adc_writel(adc_readl(ADC_CTRL) | 0xf, ADC_CTRL);
}

static void adc_hal_clean_all_interrupt_flag(void)
{
	adc_writel(adc_readl(ADC_STATUS) | 0xf, ADC_STATUS);
}

static u32 adc_hal_get_all_interrupt_flag(void)
{
	return adc_readl(ADC_STATUS) & 0xf;
}

static int adc_hal_read_channel_data(int ch)
{
	switch (ch) {
	case 0:
		return adc_readl(ADC_DATA01) & 0xfff;
	case 1:
		return (adc_readl(ADC_DATA01) >> 16) & 0xfff;
	case 2:
		return adc_readl(ADC_DATA23) & 0xfff;
	case 3:
		return (adc_readl(ADC_DATA23) >> 16) & 0xfff;
	default:
		return -1;
	}
}

// Clock dividers: bits 7..0, 15..8 and 31..16 of ADC_CLKDIV.
static void adc_hal_set_clkdiv(u32 div0, u32 div1, u32 div2)
{
	u32 v = adc_readl(ADC_CLKDIV);

	adc_writel(((div0 ^ v) & 0xff) ^ v, ADC_CLKDIV);
	v = adc_readl(ADC_CLKDIV);
	adc_writel((((div1 << 8) ^ v) & 0xff00) ^ v, ADC_CLKDIV);
	adc_writel((adc_readl(ADC_CLKDIV) & 0xffff) | (div2 << 16), ADC_CLKDIV);
}

static void adc_hal_set_wait_sampling_stable_time(u32 t)
{
	u32 v = adc_readl(ADC_STABLE);

	adc_writel(((t ^ v) & 0xffff) ^ v, ADC_STABLE);
}

// --- conversions --------------------------------------------------------------

int adc_enable(void)
{
	mutex_lock(&adc_device.mutex);
	if (adc_busy++ == 0) {
		adc_hal_enable_controller();
		adc_hal_clean_all_interrupt_flag();
		adc_hal_enable_all_interrupt();
	}
	mutex_unlock(&adc_device.mutex);
	return 0;
}
EXPORT_SYMBOL(adc_enable);

int adc_disable(void)
{
	mutex_lock(&adc_device.mutex);
	if (--adc_busy == 0) {
		adc_hal_disable_controller();
		adc_hal_mask_all_interrupt();
		adc_hal_clean_all_interrupt_flag();
	}
	mutex_unlock(&adc_device.mutex);
	return 0;
}
EXPORT_SYMBOL(adc_disable);

// The raw 12-bit value of a channel, or -EBUSY when it does not convert in
// time. Called with adc_device.mutex held.
static int adc_read_channel_value(int ch)
{
	long ret;
	int val;

	adc_device.waiting = 1u << ch;
	adc_hal_enable_channel(ch);
	ret = wait_event_interruptible_timeout(adc_device.wq, !adc_device.waiting, ADC_TIMEOUT);
	if (!ret) {
		printk(KERN_ERR "%s:adc get value timeout!\n", __func__);
		adc_hal_disable_channel(ch);
		return -EBUSY;
	}
	val = adc_hal_read_channel_data(ch);
	adc_hal_disable_channel(ch);
	return val;
}

// The voltage of a channel in mV, or a negative error.
int adc_read_channel_voltage(int ch)
{
	int val;

	mutex_lock(&adc_device.mutex);
	val = adc_read_channel_value(ch);
	if (val >= 0)
		val = (unsigned int)val * adc_vref >> 12;
	mutex_unlock(&adc_device.mutex);
	return val;
}
EXPORT_SYMBOL(adc_read_channel_voltage);

static irqreturn_t adc_irq_handler(int irq, void *dev_id)
{
	u32 flags = adc_hal_get_all_interrupt_flag();

	adc_hal_clean_all_interrupt_flag();
	if (flags & adc_device.waiting) {
		adc_device.waiting = 0;
		wake_up_interruptible(&adc_device.wq);
	}
	return IRQ_HANDLED;
}

// --- the misc devices ---------------------------------------------------------

static const struct file_operations adc_fops;

static struct miscdevice adc_mdev[ADC_CHANNELS] = {
	{ .minor = MISC_DYNAMIC_MINOR, .name = "jz_adc_aux_0", .fops = &adc_fops },
	{ .minor = MISC_DYNAMIC_MINOR, .name = "jz_adc_aux_1", .fops = &adc_fops },
	{ .minor = MISC_DYNAMIC_MINOR, .name = "jz_adc_aux_2", .fops = &adc_fops },
	{ .minor = MISC_DYNAMIC_MINOR, .name = "jz_adc_aux_3", .fops = &adc_fops },
};

// The channel of a device: misc_open() leaves the device in private_data.
static int adc_channel(struct file *file)
{
	return (struct miscdevice *)file->private_data - adc_device.mdev;
}

static int adc_open(struct inode *inode, struct file *file)
{
	adc_enable();
	return 0;
}

static int adc_release(struct inode *inode, struct file *file)
{
	adc_disable();
	return 0;
}

static ssize_t adc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned int ch = adc_channel(file);
	int val;

	BUG_ON(ch >= ADC_CHANNELS);
	mutex_lock(&adc_device.mutex);
	val = adc_read_channel_value(ch);
	if (val < 0) {
		mutex_unlock(&adc_device.mutex);
		return val;
	}
	val = (unsigned int)val * adc_vref >> 12;
	mutex_unlock(&adc_device.mutex);
	if (copy_to_user(buf, &val, sizeof(val)))
		return -EFAULT;
	return sizeof(val);
}

static long adc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;

	switch (cmd) {
	case ADC_IOC_SET_VREF:
		if (get_user(val, (int __user *)arg))
			return -EFAULT;
		adc_vref = val;
		return 0;
	case ADC_IOC_GET_VREF:
		return adc_vref;
	case ADC_IOC_READ_RAW:
		mutex_lock(&adc_device.mutex);
		val = adc_read_channel_value(adc_channel(file));
		mutex_unlock(&adc_device.mutex);
		return val;
	case ADC_IOC_NOP_0:
	case ADC_IOC_NOP_1:
		return 0;
	default:
		printk(KERN_ERR "%s:unsupported ioctl cmd %x\n", __func__, cmd);
		return -EINVAL;
	}
}

static const struct file_operations adc_fops = {
	.owner = THIS_MODULE,
	.read = adc_read,
	.unlocked_ioctl = adc_ioctl,
	.open = adc_open,
	.release = adc_release,
};

// --- module ---------------------------------------------------------------------

static int __init jz_adc_init(void)
{
	int ret, i;

	adc_device.clk = clk_get(NULL, "gate_sadc");
	if (IS_ERR(adc_device.clk))
		return PTR_ERR(adc_device.clk);
	clk_prepare_enable(adc_device.clk);

	adc_hal_disable_controller();
	adc_hal_mask_all_interrupt();
	adc_hal_clean_all_interrupt_flag();
	adc_hal_set_clkdiv(119, 1, 99);
	adc_hal_set_wait_sampling_stable_time(1);

	mutex_init(&adc_device.mutex);
	init_waitqueue_head(&adc_device.wq);

	ret = request_irq(ADC_IRQ, adc_irq_handler, 0, "adc", NULL);
	if (ret)
		goto err_clk;

	adc_device.mdev = adc_mdev;
	for (i = 0; i < ADC_CHANNELS; i++) {
		ret = misc_register(&adc_mdev[i]);
		if (ret < 0)
			goto err_misc;
	}
	return 0;

err_misc:
	while (--i >= 0)
		misc_deregister(&adc_mdev[i]);
	free_irq(ADC_IRQ, NULL);
err_clk:
	clk_disable_unprepare(adc_device.clk);
	clk_put(adc_device.clk);
	return ret;
}
module_init(jz_adc_init);

static void __exit jz_adc_exit(void)
{
	int i;

	for (i = 0; i < ADC_CHANNELS; i++)
		misc_deregister(&adc_mdev[i]);
	free_irq(ADC_IRQ, NULL);
	clk_disable_unprepare(adc_device.clk);
	clk_put(adc_device.clk);
	adc_busy = 0;
}
module_exit(jz_adc_exit);

MODULE_DESCRIPTION("JZ x1600 ADC driver");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
