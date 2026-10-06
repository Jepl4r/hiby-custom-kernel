// SPDX-License-Identifier: GPL-2.0
//
// bcm_wlbt_power -- power and wiring for a BCM43430 Wi-Fi/Bluetooth combo on
// an Ingenic X1600 board (HiBy R3 Pro II, R1), for the mainline brcmfmac driver
// and the stock Bluetooth userspace (brcm_patchram_plus on a UART).
//
// It takes over the board side of the vendor cywdhd.ko and nothing else:
//
//   Bluetooth  an rfkill named "bluetooth". Writing 1 to its state starts the
//              32 kHz LPO, raises BT_REG_ON and waits 300 ms, which is the
//              sequence cywdhd runs; 0 reverses it. The radio is off until the
//              first write. host_wake_bt (BT_WAKE) is driven high for as long
//              as the module is loaded.
//
//   Wi-Fi      /sys/devices/platform/bcm_wlbt_power/wifi_power. Writing 1 starts
//              the LPO and pulses WL_REG_ON low for 100 ms, then has the MMC
//              core bring the SDIO card up; 0 powers the card down through
//              the MMC core and drops WL_REG_ON. brcmfmac is loaded after 1
//              and removed before 0.
//
//   brcmfmac   the "brcmfmac_sdio" platform device, which hands brcmfmac the
//              out-of-band interrupt (WL_HOST_WAKE, active high).
//
//   chipvendor the attribute cywdhd puts on SDIO function 2, read by the
//              player to pick the module's Bluetooth patch: "0x81" on an
//              AzureWave AW-NB372SM, "0x00" on an AP6212A. The same value is
//              in /sys/devices/platform/bcm_wlbt_power/chipvendor.
//
// The SDIO controller belongs to soc_msc.ko and the LPO to soc_utils.ko; both
// are loaded first. The controller scans its card only when told to
// (jzmmc_manual_detect), so the card is enumerated once at load, its vendor
// tuple is read, and it is then kept registered and only powered down and up.
// A card the MMC core has dropped meanwhile is enumerated again.

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_data/brcmfmac-sdio.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/workqueue.h>

// Exported by soc_msc.ko: 1 marks the controller's card present, enables its
// clock and rescans; 0 marks it absent.
extern int jzmmc_manual_detect(int index, int on);

// Exported by soc_utils.ko: the 32 kHz clock on PC26, reference counted.
extern void ingenic_rtc32k_enable(void);
extern void ingenic_rtc32k_disable(void);

#define CARD_WAIT_MS 3000

// A pin is given as the SoC names it ("PF02") or as a GPIO number; -1 is none.
// Port A starts at GPIO 0 and every port is 32 lines wide, the expander's
// port F included (gpio_base 160).
static int param_set_pin(const char *val, const struct kernel_param *kp)
{
	int *pin = kp->arg;
	char port;
	int line;

	if ((val[0] == 'P' || val[0] == 'p') && sscanf(val + 1, "%c%d", &port, &line) == 2) {
		port = toupper(port);
		if (port < 'A' || port > 'F' || line < 0 || line > 31)
			return -EINVAL;
		*pin = (port - 'A') * 32 + line;
		return 0;
	}
	return kstrtoint(val, 0, pin);
}

static const struct kernel_param_ops param_ops_pin = {
	.set = param_set_pin,
	.get = param_get_int,
};

static int wl_reg_on = -1;
static int wl_host_wake = -1;
static int wl_mmc;
static int bt_reg_on = -1;
static int host_wake_bt = -1;

module_param_cb(wl_reg_on, &param_ops_pin, &wl_reg_on, 0444);
MODULE_PARM_DESC(wl_reg_on, "WL_REG_ON, the Wi-Fi power enable");
module_param_cb(wl_host_wake, &param_ops_pin, &wl_host_wake, 0444);
MODULE_PARM_DESC(wl_host_wake, "WL_HOST_WAKE, the out-of-band Wi-Fi interrupt (-1: in-band)");
module_param(wl_mmc, int, 0444);
MODULE_PARM_DESC(wl_mmc, "MSC controller of the SDIO card");
module_param_cb(bt_reg_on, &param_ops_pin, &bt_reg_on, 0444);
MODULE_PARM_DESC(bt_reg_on, "BT_REG_ON, the Bluetooth power enable");
module_param_cb(host_wake_bt, &param_ops_pin, &host_wake_bt, 0444);
MODULE_PARM_DESC(host_wake_bt, "BT_WAKE, held high to keep the controller awake");

static struct platform_device *radio_pdev;
static struct device *msc_dev;		// md_ingenic,mmc.N, owned by soc_msc
static struct mmc_host *host;
static struct rfkill *bt_rfkill;
static struct sdio_func *vendor_func;	// the function carrying "chipvendor"
static struct work_struct probe_work;
static DEFINE_MUTEX(radio_lock);

static bool wifi_on;
static bool bt_on;
static bool bt_synced;
static unsigned int chipvendor;

static struct platform_device *brcmf_pdev;

// ---------------------------------------------------------------------------
// Bluetooth
// ---------------------------------------------------------------------------

static void bt_power(bool on)
{
	if (on == bt_on)
		return;

	if (on) {
		ingenic_rtc32k_enable();
		gpio_set_value_cansleep(bt_reg_on, 1);
		msleep(300);
	} else {
		ingenic_rtc32k_disable();
		gpio_set_value_cansleep(bt_reg_on, 0);
		msleep(100);
	}
	bt_on = on;
}

// The first call is the rfkill core applying the global Bluetooth state at
// registration, unblocked by default: the radio stays off until something
// writes to the state file, as with cywdhd.
static int bt_set_block(void *data, bool blocked)
{
	mutex_lock(&radio_lock);
	if (bt_synced)
		bt_power(!blocked);
	bt_synced = true;
	mutex_unlock(&radio_lock);
	return 0;
}

static const struct rfkill_ops bt_rfkill_ops = {
	.set_block = bt_set_block,
};

// ---------------------------------------------------------------------------
// Wi-Fi
// ---------------------------------------------------------------------------

static int match_mmc_host(struct device *dev, void *data)
{
	return dev->class && !strcmp(dev->class->name, "mmc_host");
}

// The mmc_host of the controller, found through its platform device: soc_msc
// registers it as md_ingenic,mmc.N and the host's class device is its child.
static struct mmc_host *find_host(void)
{
	char name[32];
	struct device *child;

	if (host)
		return host;

	snprintf(name, sizeof(name), "md_ingenic,mmc.%d", wl_mmc);
	if (!msc_dev)
		msc_dev = bus_find_device_by_name(&platform_bus_type, NULL, name);
	if (!msc_dev)
		return NULL;

	child = device_find_child(msc_dev, NULL, match_mmc_host);
	if (!child)
		return NULL;
	host = container_of(child, struct mmc_host, class_dev);
	return host;
}

// SDIO function 2 of the card, once it is registered: the rescan sets
// host->card before it adds the functions, and adds them with the host
// released. Called with the host claimed, which keeps the card from going.
static struct sdio_func *registered_func2(void)
{
	struct mmc_card *card = host->card;
	int i;

	if (!card)
		return NULL;
	for (i = 0; i < card->sdio_funcs; i++) {
		struct sdio_func *func = card->sdio_func[i];

		if (func && func->num == 2 && sdio_func_present(func))
			return func;
	}
	return NULL;
}

static bool wait_for_card(void)
{
	int waited;
	bool found;

	for (waited = 0; waited < CARD_WAIT_MS; waited += 20) {
		mmc_claim_host(host);
		found = registered_func2() != NULL;
		mmc_release_host(host);
		if (found)
			return true;
		msleep(20);
	}
	return false;
}

static ssize_t chipvendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%02x", chipvendor);
}

static DEVICE_ATTR_RO(chipvendor);

// The module maker is in a vendor tuple of the card's common CIS: code 0x81,
// one byte, 0x01 on an AzureWave module. cywdhd reports 0x81 for that and 0
// otherwise, and puts the attribute on function 2.
static void read_vendor(void)
{
	struct sdio_func_tuple *t;
	struct sdio_func *func;

	mmc_claim_host(host);
	func = registered_func2();
	if (func) {
		chipvendor = 0;
		for (t = host->card->tuples; t; t = t->next) {
			if (t->code == 0x81 && t->size >= 1 && t->data[0] == 0x01) {
				chipvendor = 0x81;
				break;
			}
		}
		if (!device_create_file(&func->dev, &dev_attr_chipvendor)) {
			get_device(&func->dev);
			vendor_func = func;
		}
		dev_info(&radio_pdev->dev, "SDIO card %04x:%04x, module vendor 0x%02x (%s)\n", host->card->cis.vendor,
			 host->card->cis.device, chipvendor, chipvendor == 0x81 ? "AzureWave" : "AP6212A");
	}
	mmc_release_host(host);
}

// Lets go of the function carrying "chipvendor". The card it belongs to may
// have been removed meanwhile -- a resume that finds the chip unpowered can
// take it out -- and then its attributes went with it.
static void forget_vendor_func(void)
{
	if (!vendor_func)
		return;
	if (device_is_registered(&vendor_func->dev))
		device_remove_file(&vendor_func->dev, &dev_attr_chipvendor);
	put_device(&vendor_func->dev);
	vendor_func = NULL;
}

static void wl_reg_on_pulse(void)
{
	gpio_set_value_cansleep(wl_reg_on, 0);
	msleep(100);
	gpio_set_value_cansleep(wl_reg_on, 1);
}

static int wifi_power_up(void)
{
	int ret = 0;

	if (wifi_on)
		return 0;
	if (!find_host())
		return -ENODEV;

	if (vendor_func && (!host->card || vendor_func->card != host->card || !device_is_registered(&vendor_func->dev)))
		forget_vendor_func();

	ingenic_rtc32k_enable();
	wl_reg_on_pulse();

	if (host->card) {
		mmc_claim_host(host);
		ret = mmc_power_restore_host(host);
		mmc_release_host(host);
	} else {
		jzmmc_manual_detect(wl_mmc, 1);
		if (!wait_for_card())
			ret = -ETIMEDOUT;
	}

	if (ret) {
		gpio_set_value_cansleep(wl_reg_on, 0);
		ingenic_rtc32k_disable();
		dev_err(&radio_pdev->dev, "Wi-Fi did not come up: %d\n", ret);
		return ret;
	}

	// The card may already be there when the module loads: the controller
	// scans it once at probe, and WL_REG_ON is not driven before this module.
	if (!vendor_func)
		read_vendor();

	wifi_on = true;
	return 0;
}

static void wifi_power_down(void)
{
	if (!wifi_on)
		return;

	if (host && host->card) {
		mmc_claim_host(host);
		mmc_power_save_host(host);
		mmc_release_host(host);
	}
	gpio_set_value_cansleep(wl_reg_on, 0);
	ingenic_rtc32k_disable();
	wifi_on = false;
}

static ssize_t wifi_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", wifi_on);
}

static ssize_t wifi_power_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	bool on;
	int ret = 0;

	if (strtobool(buf, &on))
		return -EINVAL;

	flush_work(&probe_work);
	mutex_lock(&radio_lock);
	if (on)
		ret = wifi_power_up();
	else
		wifi_power_down();
	mutex_unlock(&radio_lock);

	return ret ? ret : count;
}

static DEVICE_ATTR_RW(wifi_power);

static struct attribute *radio_attrs[] = {
	&dev_attr_wifi_power.attr,
	&dev_attr_chipvendor.attr,
	NULL,
};

static const struct attribute_group radio_group = {
	.attrs = radio_attrs,
};

// Enumerates the card once so that its vendor is known before anything asks,
// then powers it down. Run from a work item: it takes a few hundred
// milliseconds and the module scripts that follow do not need it.
static void probe_card(struct work_struct *work)
{
	mutex_lock(&radio_lock);
	if (!wifi_power_up())
		wifi_power_down();
	mutex_unlock(&radio_lock);
}

// ---------------------------------------------------------------------------
// module
// ---------------------------------------------------------------------------

static int request_out(int pin, const char *label, int level)
{
	if (pin < 0)
		return 0;
	return gpio_request_one(pin, level ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW, label);
}

static void release(int pin)
{
	if (pin >= 0)
		gpio_free(pin);
}

static int __init bcm_wlbt_power_init(void)
{
	int ret;

	if (wl_reg_on < 0 || bt_reg_on < 0) {
		pr_err("bcm_wlbt_power: wl_reg_on and bt_reg_on are required\n");
		return -EINVAL;
	}

	radio_pdev = platform_device_register_simple("bcm_wlbt_power", -1, NULL, 0);
	if (IS_ERR(radio_pdev))
		return PTR_ERR(radio_pdev);

	ret = request_out(wl_reg_on, "wl_reg_on", 0);
	if (ret)
		goto err_pdev;
	ret = request_out(bt_reg_on, "bt_reg_on", 0);
	if (ret)
		goto err_wl;
	ret = request_out(host_wake_bt, "host_wake_bt", 1);
	if (ret)
		goto err_bt;
	if (wl_host_wake >= 0) {
		ret = gpio_request_one(wl_host_wake, GPIOF_IN, "wl_host_wake");
		if (ret)
			goto err_wake_bt;
	}

	// First, so that it is rfkill0: the player and the stock bt_init write
	// /sys/class/rfkill/rfkill0/state. Not given an initial state with
	// rfkill_init_sw_state(): that would set the global Bluetooth state, and
	// the rfkill of hci0, registered later by the Bluetooth core, would start
	// blocked with it.
	bt_rfkill = rfkill_alloc("bluetooth", &radio_pdev->dev, RFKILL_TYPE_BLUETOOTH, &bt_rfkill_ops, NULL);
	if (!bt_rfkill) {
		ret = -ENOMEM;
		goto err_wake_wl;
	}
	ret = rfkill_register(bt_rfkill);
	if (ret)
		goto err_rfkill;

	if (wl_host_wake >= 0) {
		struct brcmfmac_sdio_platform_data pdata = {
			.oob_irq_supported = true,
			.oob_irq_nr = gpio_to_irq(wl_host_wake),
			.oob_irq_flags = IRQF_TRIGGER_HIGH,
		};

		brcmf_pdev = platform_device_register_data(NULL, BRCMFMAC_SDIO_PDATA_NAME, PLATFORM_DEVID_NONE,
							   &pdata, sizeof(pdata));
		if (IS_ERR(brcmf_pdev)) {
			ret = PTR_ERR(brcmf_pdev);
			brcmf_pdev = NULL;
			goto err_unregister;
		}
	}

	INIT_WORK(&probe_work, probe_card);
	ret = sysfs_create_group(&radio_pdev->dev.kobj, &radio_group);
	if (ret)
		goto err_brcmf;

	schedule_work(&probe_work);
	return 0;

err_brcmf:
	if (brcmf_pdev)
		platform_device_unregister(brcmf_pdev);
err_unregister:
	rfkill_unregister(bt_rfkill);
err_rfkill:
	rfkill_destroy(bt_rfkill);
err_wake_wl:
	release(wl_host_wake);
err_wake_bt:
	release(host_wake_bt);
err_bt:
	release(bt_reg_on);
err_wl:
	release(wl_reg_on);
err_pdev:
	platform_device_unregister(radio_pdev);
	return ret;
}

static void __exit bcm_wlbt_power_exit(void)
{
	flush_work(&probe_work);
	sysfs_remove_group(&radio_pdev->dev.kobj, &radio_group);

	// brcmfmac lets go of the card before the power goes, and the rfkill
	// before Bluetooth is switched off, so that nothing switches it back on.
	if (brcmf_pdev)
		platform_device_unregister(brcmf_pdev);
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	mutex_lock(&radio_lock);
	wifi_power_down();
	bt_power(false);
	forget_vendor_func();
	mutex_unlock(&radio_lock);

	if (host)
		put_device(&host->class_dev);
	if (msc_dev)
		put_device(msc_dev);

	release(wl_host_wake);
	release(host_wake_bt);
	release(bt_reg_on);
	release(wl_reg_on);
	platform_device_unregister(radio_pdev);
}

module_init(bcm_wlbt_power_init);
module_exit(bcm_wlbt_power_exit);

MODULE_DESCRIPTION("Wi-Fi and Bluetooth power for a BCM43430 combo on Ingenic boards");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
