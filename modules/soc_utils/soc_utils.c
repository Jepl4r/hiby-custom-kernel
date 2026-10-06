// SPDX-License-Identifier: GPL-2.0
//
// soc_utils -- the 32 kHz clock output of the X1600 on PC26 (function 0),
// the low-power clock of the board's Wi-Fi/Bluetooth chip.
//
// A drop-in replacement for the vendor module of the same name, with the same
// exports and the same parameter:
//
//   ingenic_rtc32k_enable()   counted: the first call switches PC26 to the
//   ingenic_rtc32k_disable()  clock, the call that brings the count back to 0
//                             makes it an input again
//   rtc32k_exit()             releases PC26
//   rtc32k_init_on            1: the clock is on from load, as one enable
//
// A disable with the count at 0 is ignored; the vendor module takes the count
// below 0, and the next enable then leaves the clock off.
//
// Loaded after utils.ko, which provides gpio_set_func().

#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>

// Exported by utils.ko: sets a pin's function, with the encoding of
// enum gpio_function in the X1600 SoC headers.
extern int gpio_set_func(int gpio, int func);

#define GPIO_PC(n)		(2 * 32 + (n))
#define RTC32K_GPIO		GPIO_PC(26)
#define GPIO_FUNC_0		0x0100
#define GPIO_INPUT		0x0106

void ingenic_rtc32k_enable(void);
void ingenic_rtc32k_disable(void);
void rtc32k_exit(void);

static int rtc32k_init_on;
module_param(rtc32k_init_on, int, 0644);
MODULE_PARM_DESC(rtc32k_init_on, "1: the 32 kHz clock is on from load");

static int refcount;
static int rtc32k_initialized;
static DEFINE_SPINLOCK(rtc32k_lock);

void ingenic_rtc32k_enable(void)
{
	unsigned long flags;

	if (!rtc32k_initialized) {
		pr_err("rtc32k is not init ok\n");
		return;
	}
	spin_lock_irqsave(&rtc32k_lock, flags);
	if (refcount++ == 0)
		gpio_set_func(RTC32K_GPIO, GPIO_FUNC_0);
	spin_unlock_irqrestore(&rtc32k_lock, flags);
}
EXPORT_SYMBOL(ingenic_rtc32k_enable);

void ingenic_rtc32k_disable(void)
{
	unsigned long flags;

	if (!rtc32k_initialized) {
		pr_err("rtc32k is not init ok\n");
		return;
	}
	spin_lock_irqsave(&rtc32k_lock, flags);
	if (refcount == 0)
		pr_warn_once("rtc32k disabled more often than enabled\n");
	else if (--refcount == 0)
		gpio_set_func(RTC32K_GPIO, GPIO_INPUT);
	spin_unlock_irqrestore(&rtc32k_lock, flags);
}
EXPORT_SYMBOL(ingenic_rtc32k_disable);

void rtc32k_exit(void)
{
	if (rtc32k_initialized)
		gpio_free(RTC32K_GPIO);
	rtc32k_initialized = 0;
	refcount = 0;
}
EXPORT_SYMBOL(rtc32k_exit);

static void rtc32k_init(void)
{
	if (gpio_request(RTC32K_GPIO, "clk32k_out_o")) {
		pr_err("request rtc32k gpio fail!\n");
		return;
	}
	rtc32k_initialized = 1;
	if (rtc32k_init_on) {
		printk("init enable rtc32k out\n");
		ingenic_rtc32k_enable();
	}
}

static int __init utils_init(void)
{
	rtc32k_init();
	return 0;
}

static void __exit utils_exit(void)
{
	rtc32k_exit();
}

module_init(utils_init);
module_exit(utils_exit);

MODULE_DESCRIPTION("X1600 32 kHz clock output on PC26");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
