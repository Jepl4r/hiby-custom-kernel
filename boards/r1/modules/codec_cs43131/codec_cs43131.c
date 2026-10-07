// SPDX-License-Identifier: GPL-2.0
//
// codec_cs43131 -- the Cirrus Logic CS43131 DAC of the HiBy R1 as an ASoC
// codec, "cs43131-hifi" on cs43131.3-0030.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock codec_cs43131.sh passes them):
//
//   cs43131_i2c_bus_num   I2C bus of the DAC (address 0x30)
//   cs43131_rst_gpio      reset, active low (-1: none)
//   cs43131_pwr_gpio      power, active high (-1: none)
//   cs43131_*_en_level, cs43131_mute_gpio, cs43131_po_sel_gpio   not read
//
// Every stream start powers the DAC up and every prepare loads the whole
// register set, then the PCM or DSD settings. The register set keeps the
// last value written to each of its registers. Registers are written only
// while a stream is open.
//
// Mixer controls (reading any of them returns 0):
//   Left/Right Playback Volume   0..255, the attenuation register value
//   DOP_EN           1: the next stream is DSD over PCM
//   Digital Filter   PCM filter 0..3: bit 0 phase compensated, bit 1 slow
//                    roll-off
//   NOS_EN           1: PCM filter off (non-oversampling emulation)
//   Mute Output, Soft Mute   accepted, no effect
//
// Write-only sysfs files on the I2C device:
//   write_reg_val    "REG VAL" in hex: write VAL to REG
//   reg_val          "w REG VAL": the same; "r REG N": read N registers, the
//                    values are not reported
//   write_lr_flag    "right" or anything else; not read
//
// Loaded after utils.ko, which provides i2c_register_device() and the pin
// parameters.

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

// Exported by utils.ko.
extern const struct kernel_param_ops param_gpio_ops;
extern struct i2c_client *i2c_register_device(struct i2c_board_info *info, int busnum);

// Registers, 24-bit addresses.
#define CS43131_SP_SRATE	0x01000b
#define CS43131_SP_BITSIZE	0x01000c
#define CS43131_DSD_VOL_B	0x070000
#define CS43131_DSD_VOL_A	0x070001
#define CS43131_DSD_PATH_CTL_2	0x070004
#define CS43131_PCM_FILTER	0x090000
#define CS43131_PCM_VOL_B	0x090001
#define CS43131_PCM_VOL_A	0x090002

#define CS43131_REG_END		0xffffff

struct cs43131_reg {
	unsigned int reg;
	unsigned int val;
};

struct cs43131_pdata {
	int reset_gpio;
	int power_gpio;
};

struct cs43131_priv {
	struct cs43131_pdata *pdata;
	struct mutex reg_mutex;
	struct snd_pcm_hw_params params;	// of the last hw_params
	int dsd_en;
	int powered;				// a stream is open
};

static int cs43131_i2c_bus_num = -1;
static int cs43131_pwr_gpio = -1;
static int cs43131_pwr_en_level = 1;
static int cs43131_rst_gpio = -1;
static int cs43131_rst_en_level;
static int cs43131_mute_gpio = -1;
static int cs43131_mute_en_level = 1;
static int cs43131_po_sel_gpio = -1;
static int cs43131_po_sel_en_level;

module_param(cs43131_i2c_bus_num, int, 0644);
module_param_cb(cs43131_pwr_gpio, &param_gpio_ops, &cs43131_pwr_gpio, 0644);
module_param(cs43131_pwr_en_level, int, 0644);
module_param_cb(cs43131_rst_gpio, &param_gpio_ops, &cs43131_rst_gpio, 0644);
module_param(cs43131_rst_en_level, int, 0644);
module_param_cb(cs43131_mute_gpio, &param_gpio_ops, &cs43131_mute_gpio, 0644);
module_param(cs43131_mute_en_level, int, 0644);
module_param_cb(cs43131_po_sel_gpio, &param_gpio_ops, &cs43131_po_sel_gpio, 0644);
module_param(cs43131_po_sel_en_level, int, 0644);

static struct cs43131_pdata cs43131_data = {
	.reset_gpio = -1,
	.power_gpio = -1,
};

static struct i2c_board_info cs43131_board_info = {
	I2C_BOARD_INFO("cs43131", 0x30),
	.platform_data = &cs43131_data,
};

static struct i2c_client *cs43131_i2c_dev;
static struct i2c_client *this_i2c;
static struct cs43131_priv *g_cs43131;

static int lr_flag;
static long volume_left = -1;
static long volume_right = -1;

// PCM filter register: bit 7 slow roll-off, bit 6 phase compensated, bit 5
// non-oversampling emulation, bit 1 the high-pass filter, as in the
// register set.
#define CS43131_FILTER_HPF	0x02
#define CS43131_FILTER_NOS	0x20
static int nos_enable;
static u8 filter_value = CS43131_FILTER_HPF;

// Loaded on every prepare.
static struct cs43131_reg cs43131_reg[] = {
	{ 0x010006, 0x04 }, { 0x020052, 0x06 }, { 0x01000b, 0x01 },
	{ 0x01000c, 0x00 }, { 0x05000a, 0x07 }, { 0x05000b, 0x0f },
	{ 0x06000a, 0x07 }, { 0x06000b, 0x0f }, { 0x070000, 0x00 },
	{ 0x070001, 0x00 }, { 0x070004, 0x02 }, { 0x070006, 0x40 },
	{ 0x090000, 0x02 }, { 0x090001, 0x00 }, { 0x090002, 0x00 },
	{ 0x090003, 0x90 }, { 0x090004, 0x00 }, { 0x0b0000, 0x06 },
	{ 0x080000, 0x31 }, { 0x070002, 0xa0 }, { 0x020000, 0xbe },
	{ 0x020000, 0xae }, { 0x09000a, 0xf0 }, { 0x09000b, 0x0c },
	{ CS43131_REG_END, 0x00 },
};

// One register write: the address MSB first, a zero byte, the value.
// -ENODEV while no stream is open.
static int cs43131_i2c_write_reg(unsigned int reg, u8 val)
{
	struct i2c_client *client = this_i2c;
	u8 buf[5];
	struct i2c_msg msg = {
		.flags = 0,
		.len = sizeof(buf),
		.buf = buf,
	};
	int ret;

	if (!client || !g_cs43131 || !g_cs43131->powered)
		return -ENODEV;

	msg.addr = client->addr;
	buf[0] = reg >> 16;
	buf[1] = reg >> 8;
	buf[2] = reg;
	buf[3] = 0;
	buf[4] = val;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1)
		return 0;
	return ret < 0 ? ret : -EIO;
}

// Sets every entry of reg in the register set to val. Called with reg_mutex
// held.
static void cs43131_save_reg(unsigned int reg, u8 val)
{
	struct cs43131_reg *r;

	for (r = cs43131_reg; r->reg != CS43131_REG_END; r++)
		if (r->reg == reg)
			r->val = val;
}

// Writes the register and keeps the value in the register set.
static int cs43131_write_reg_save(unsigned int reg, u8 val)
{
	int ret;

	mutex_lock(&g_cs43131->reg_mutex);
	cs43131_save_reg(reg, val);
	ret = cs43131_i2c_write_reg(reg, val);
	mutex_unlock(&g_cs43131->reg_mutex);
	return ret;
}

// Drives the power pin: called by x1600_hiby_r1_sound_card.ko on output
// changes.
void cs43131_set_power(int on)
{
	struct cs43131_pdata *pdata;

	if (!g_cs43131)
		return;
	pdata = g_cs43131->pdata;
	if (gpio_is_valid(pdata->power_gpio))
		gpio_set_value(pdata->power_gpio, on ? 1 : 0);
}
EXPORT_SYMBOL(cs43131_set_power);

// --- mixer controls ---------------------------------------------------------

static int cs43131_dummy_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int cs43131_vol_left_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	long v = ucontrol->value.integer.value[0];

	if (v != volume_left) {
		mutex_lock(&g_cs43131->reg_mutex);
		volume_left = v;
		cs43131_save_reg(CS43131_PCM_VOL_A, v);
		mutex_unlock(&g_cs43131->reg_mutex);
		cs43131_write_reg_save(CS43131_PCM_VOL_A, volume_left);
		cs43131_write_reg_save(CS43131_DSD_VOL_A, volume_left);
	}
	return 0;
}

static int cs43131_vol_right_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	long v = ucontrol->value.integer.value[0];

	if (v != volume_right) {
		volume_right = v;
		cs43131_write_reg_save(CS43131_PCM_VOL_B, v);
		cs43131_write_reg_save(CS43131_DSD_VOL_B, volume_right);
	}
	return 0;
}

static int cs43131_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int cs43131_digital_filter_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	unsigned long v = ucontrol->value.integer.value[0];

	if (v >= 4)
		v = 0;
	filter_value = (v << 6) | CS43131_FILTER_HPF;
	if (!nos_enable)
		cs43131_write_reg_save(CS43131_PCM_FILTER, filter_value);
	return 0;
}

static int cs43131_nos_en_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	unsigned long v = ucontrol->value.integer.value[0];

	if (v >= 2)
		v = 0;
	nos_enable = v;
	cs43131_write_reg_save(CS43131_PCM_FILTER,
			       v ? CS43131_FILTER_NOS | CS43131_FILTER_HPF : filter_value);
	return 0;
}

static int cs43131_soft_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int cs43131_dop_en_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	g_cs43131->dsd_en = ucontrol->value.integer.value[0];
	return 0;
}

static const struct snd_kcontrol_new cs43131_snd_controls[] = {
	SOC_SINGLE_EXT("Left Playback Volume", 0, 0, 255, 0, cs43131_dummy_get, cs43131_vol_left_put),
	SOC_SINGLE_EXT("Right Playback Volume", 0, 0, 255, 0, cs43131_dummy_get, cs43131_vol_right_put),
	SOC_SINGLE_EXT("Mute Output", 0, 0, 1, 0, cs43131_dummy_get, cs43131_mute_put),
	SOC_SINGLE_EXT("Digital Filter", 0, 0, 4, 0, cs43131_dummy_get, cs43131_digital_filter_put),
	SOC_SINGLE_EXT("Soft Mute", 0, 0, 1, 0, cs43131_dummy_get, cs43131_soft_mute_put),
	SOC_SINGLE_EXT("DOP_EN", 0, 0, 1, 0, cs43131_dummy_get, cs43131_dop_en_put),
	SOC_SINGLE_EXT("NOS_EN", 0, 0, 1, 0, cs43131_dummy_get, cs43131_nos_en_put),
};

// --- DAI --------------------------------------------------------------------

static int cs43131_dai_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct cs43131_priv *cs43131 = snd_soc_codec_get_drvdata(dai->codec);
	struct cs43131_pdata *pdata = cs43131->pdata;

	if (gpio_is_valid(pdata->power_gpio)) {
		gpio_set_value(pdata->power_gpio, 1);
		mdelay(10);
	}
	if (gpio_is_valid(pdata->reset_gpio)) {
		gpio_set_value(pdata->reset_gpio, 1);
		mdelay(10);
	}
	cs43131->powered = 1;
	return 0;
}

static void cs43131_dai_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct cs43131_priv *cs43131 = snd_soc_codec_get_drvdata(dai->codec);
	struct cs43131_pdata *pdata = cs43131->pdata;

	cs43131->powered = 0;
	if (gpio_is_valid(pdata->power_gpio))
		gpio_set_value(pdata->power_gpio, 0);
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_set_value(pdata->reset_gpio, 0);
}

static int cs43131_dai_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	memcpy(&g_cs43131->params, params, sizeof(*params));
	return 0;
}

// The serial port rate setting for a PCM rate.
static unsigned int cs43131_pcm_speed(unsigned int rate)
{
	switch (rate) {
	case 32000:
		return 0;
	case 48000:
		return 2;
	case 88200:
		return 3;
	case 96000:
		return 4;
	case 176400:
		return 5;
	case 192000:
		return 6;
	case 352800:
		return 7;
	case 384000:
		return 8;
	default:
		return 1;
	}
}

static int cs43131_dai_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct cs43131_priv *cs43131 = g_cs43131;
	unsigned int rate = params_rate(&cs43131->params);
	const struct cs43131_reg *r;

	for (r = cs43131_reg; r->reg != CS43131_REG_END; r++) {
		mutex_lock(&cs43131->reg_mutex);
		cs43131_i2c_write_reg(r->reg, r->val);
		mutex_unlock(&cs43131->reg_mutex);
	}

	if (cs43131->dsd_en) {
		unsigned int speed, srate;

		printk("cs43131_dai_prepare dsd\n");
		// The DSD speed and the serial port rate for the DoP frame rate.
		if (rate * 32 == 5644800) {
			speed = 1;
			srate = 7;
		} else if (rate * 32 == 11289600) {
			speed = 2;
			srate = 0;
		} else {
			speed = 0;
			srate = 5;
		}
		cs43131_write_reg_save(CS43131_DSD_PATH_CTL_2, speed << 2 | 0x50);
		cs43131_write_reg_save(CS43131_SP_SRATE, srate);
		cs43131_write_reg_save(CS43131_SP_BITSIZE, 5);
	} else {
		printk("cs43131_dai_prepare pcm\n");
		cs43131_write_reg_save(CS43131_DSD_PATH_CTL_2, 0x02);
		cs43131_write_reg_save(CS43131_SP_SRATE, cs43131_pcm_speed(rate));
	}
	return 0;
}

// Format, clock, mute and trigger are fixed by the sound card; these accept
// any setting.
static int cs43131_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int cs43131_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int cs43131_dai_digital_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int cs43131_dai_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops cs43131_dai_drv_ops = {
	.set_sysclk = cs43131_dai_set_sysclk,
	.set_fmt = cs43131_dai_set_fmt,
	.digital_mute = cs43131_dai_digital_mute,
	.startup = cs43131_dai_startup,
	.shutdown = cs43131_dai_shutdown,
	.hw_params = cs43131_dai_hw_params,
	.prepare = cs43131_dai_prepare,
	.trigger = cs43131_dai_trigger,
};

static struct snd_soc_dai_driver cs43131_dai_drv = {
	.name = "cs43131-hifi",
	.ops = &cs43131_dai_drv_ops,
	.playback = {
		.stream_name = "Playback",
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000 |
			 SNDRV_PCM_RATE_352800 | SNDRV_PCM_RATE_384000,
		.channels_min = 1,
		.channels_max = 2,
	},
};

// --- codec ------------------------------------------------------------------

// The registers are loaded per stream; the codec callbacks have nothing to do.
static int cs43131_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43131_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43131_codec_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43131_codec_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43131_codec_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
	return 0;
}

static struct snd_soc_codec_driver cs43131_codec_drv = {
	.probe = cs43131_codec_probe,
	.remove = cs43131_codec_remove,
	.suspend = cs43131_codec_suspend,
	.resume = cs43131_codec_resume,
	.set_bias_level = cs43131_codec_set_bias_level,
	.component_driver = {
		.controls = cs43131_snd_controls,
		.num_controls = ARRAY_SIZE(cs43131_snd_controls),
	},
};

// --- sysfs ------------------------------------------------------------------

static ssize_t cs43131_write_lr_flag(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	lr_flag = !strcmp(buf, "right");
	return count;
}

static ssize_t cs43131_write_reg(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int reg;
	u8 val;

	if (sscanf(buf, "%x %hhx", &reg, &val) != 2)
		return -EINVAL;
	cs43131_write_reg_save(reg, val);
	return count;
}

static ssize_t sys_set_reg_val(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	char cmd[12];
	unsigned int reg, n, i;
	u8 addr[4];
	u8 data = 0;
	struct i2c_msg msgs[2];

	if (sscanf(buf, "%11s %x %x\n", cmd, &reg, &n) != 3)
		return -EINVAL;

	if (!strncmp(cmd, "r", 1)) {
		for (i = 0; i < n; i++, reg++) {
			if (!this_i2c)
				continue;
			addr[0] = reg >> 16;
			addr[1] = reg >> 8;
			addr[2] = reg;
			addr[3] = 0;
			msgs[0].addr = this_i2c->addr;
			msgs[0].flags = 0;
			msgs[0].len = sizeof(addr);
			msgs[0].buf = addr;
			msgs[1].addr = this_i2c->addr;
			msgs[1].flags = I2C_M_RD;
			msgs[1].len = 1;
			msgs[1].buf = &data;
			i2c_transfer(this_i2c->adapter, msgs, 2);
		}
	} else if (!strncmp(cmd, "w", 1)) {
		cs43131_write_reg_save(reg, n);
	}
	return count;
}

static DEVICE_ATTR(write_lr_flag, 0200, NULL, cs43131_write_lr_flag);
static DEVICE_ATTR(write_reg_val, 0200, NULL, cs43131_write_reg);
static DEVICE_ATTR(reg_val, 0200, NULL, sys_set_reg_val);

static struct attribute *sys_codec_attributes[] = {
	&dev_attr_reg_val.attr,
	&dev_attr_write_lr_flag.attr,
	&dev_attr_write_reg_val.attr,
	NULL,
};

static const struct attribute_group sys_codec_group = {
	.attrs = sys_codec_attributes,
};

// --- I2C device -------------------------------------------------------------

static void cs43131_free_gpios(struct cs43131_pdata *pdata)
{
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->power_gpio))
		gpio_free(pdata->power_gpio);
}

// Takes the power and reset pins, both low: the DAC stays off until a stream
// starts.
static int cs43131_platform_init(struct cs43131_pdata *pdata)
{
	int ret;

	if (gpio_is_valid(pdata->power_gpio)) {
		ret = gpio_request(pdata->power_gpio, "cs43131_power");
		if (ret < 0) {
			printk("[CS43131_MX30] Gpio request failed. cs43131_power pin:%d\n", pdata->power_gpio);
			goto err;
		}
		gpio_direction_output(pdata->power_gpio, 0);
		printk("[CS43131_MX30] cs43131_power pin:%d output\n", pdata->power_gpio);
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request(pdata->reset_gpio, "cs43131_reset");
		if (ret < 0) {
			printk("[CS43131_MX30] Gpio request failed. cs43131_reset pin:%d\n", pdata->reset_gpio);
			if (gpio_is_valid(pdata->power_gpio))
				gpio_free(pdata->power_gpio);
			goto err;
		}
		gpio_direction_output(pdata->reset_gpio, 0);
		printk("[CS43131_MX30] cs43131_reset pin:%d output\n", pdata->reset_gpio);
	}
	return 0;

err:
	printk("[CS43131_MX30] %s Occur error. error=%d\n", __func__, ret);
	return ret;
}

// The pins come from the device tree node when there is one, else from the
// module parameters.
static struct cs43131_pdata *cs43131_dt_pdata(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct cs43131_pdata *pdata;
	enum of_gpio_flags flags;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		printk("[CS43131_MX30] Failed to allocate memory\n");
		return NULL;
	}

	pdata->power_gpio = of_get_named_gpio_flags(np, "cs43131,power-gpio", 0, &flags);
	if (!gpio_is_valid(pdata->power_gpio))
		printk("[CS43131_MX30] Unable to get power-gpio\n");
	else
		printk("[CS43131_MX30] Get pdata->power_gpio gpio %d\n", pdata->power_gpio);

	pdata->reset_gpio = of_get_named_gpio_flags(np, "cs43131,reset-gpio", 0, &flags);
	if (!gpio_is_valid(pdata->reset_gpio))
		printk("[CS43131_MX30] Unable to get reset-gpio\n");
	else
		printk("[CS43131_MX30] Get pdata->reset gpio %d\n", pdata->reset_gpio);

	return pdata;
}

static int cs43131_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct cs43131_priv *cs43131;
	struct cs43131_pdata *pdata;
	int ret;

	cs43131 = devm_kzalloc(dev, sizeof(*cs43131), GFP_KERNEL);
	if (!cs43131) {
		ret = -ENOMEM;
		goto err;
	}

	if (dev->of_node) {
		pdata = cs43131_dt_pdata(dev);
		if (!pdata) {
			ret = -ENOMEM;
			goto err;
		}
	} else {
		pdata = dev->platform_data;
	}
	cs43131->pdata = pdata;
	i2c_set_clientdata(client, cs43131);
	this_i2c = client;

	ret = cs43131_platform_init(pdata);
	if (ret < 0) {
		printk("[CS43131_MX30] Platform initial failed.\n");
		goto err_free;
	}

	sysfs_create_group(&dev->kobj, &sys_codec_group);

	ret = snd_soc_register_codec(dev, &cs43131_codec_drv, &cs43131_dai_drv, 1);
	if (ret < 0) {
		printk("[CS43131_MX30] Register codec failed.\n");
		sysfs_remove_group(&dev->kobj, &sys_codec_group);
		cs43131_free_gpios(pdata);
		goto err_free;
	}

	g_cs43131 = cs43131;
	mutex_init(&cs43131->reg_mutex);
	cs43131->powered = 0;
	return 0;

err_free:
	this_i2c = NULL;
	i2c_set_clientdata(client, NULL);
	devm_kfree(dev, cs43131);
err:
	printk("[CS43131_MX30] %s Occur error. error=%d\n", __func__, ret);
	return ret;
}

static int cs43131_i2c_remove(struct i2c_client *client)
{
	struct cs43131_priv *cs43131 = i2c_get_clientdata(client);

	snd_soc_unregister_codec(&client->dev);
	sysfs_remove_group(&client->dev.kobj, &sys_codec_group);
	cs43131_free_gpios(cs43131->pdata);
	g_cs43131 = NULL;
	devm_kfree(&client->dev, cs43131);
	i2c_set_clientdata(client, NULL);
	this_i2c = NULL;
	return 0;
}

static const struct i2c_device_id cs43131_i2c_id[] = {
	{ "cs43131", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cs43131_i2c_id);

static struct i2c_driver cs43131_i2c_driver = {
	.driver = {
		.name = "cs43131",
		.owner = THIS_MODULE,
	},
	.probe = cs43131_i2c_probe,
	.remove = cs43131_i2c_remove,
	.id_table = cs43131_i2c_id,
};

static int __init cs43131_init(void)
{
	int ret;

	cs43131_data.reset_gpio = cs43131_rst_gpio;
	cs43131_data.power_gpio = cs43131_pwr_gpio;

	ret = i2c_add_driver(&cs43131_i2c_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register CS43131 I2C driver\n");
		return ret;
	}

	cs43131_i2c_dev = i2c_register_device(&cs43131_board_info, cs43131_i2c_bus_num);
	if (!cs43131_i2c_dev) {
		printk(KERN_ERR "Failed to register CS43131 i2c device\n");
		i2c_del_driver(&cs43131_i2c_driver);
		return -EINVAL;
	}
	return 0;
}
module_init(cs43131_init);

static void __exit cs43131_exit(void)
{
	i2c_unregister_device(cs43131_i2c_dev);
	i2c_del_driver(&cs43131_i2c_driver);
}
module_exit(cs43131_exit);

MODULE_DESCRIPTION("CS43131 DAC of the HiBy R1");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
