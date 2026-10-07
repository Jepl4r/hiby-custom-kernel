// SPDX-License-Identifier: GPL-2.0
//
// codec_cs43198_dual -- the two Cirrus Logic CS43198 DACs of the HiBy R3 Pro
// II as one ASoC codec, "cs43198-hifi" on cs43198.3-0030.
//
// A drop-in replacement for the vendor module of the same name, with the same
// parameters (the stock codec_cs43198_dual.sh passes them):
//
//   cs43198_i2c_bus_num   I2C bus of the two DACs
//   cs43198_rst_gpio      reset of both DACs, active low (-1: none)
//   cs43198_pwr_gpio      power of both DACs, active high (-1: none)
//   cs43198_*_en_level, cs43198_mute_gpio, cs43198_po_sel_gpio   not read
//
// The DACs answer at 0x30 (right channel) and 0x33 (left channel) on the
// same bus; the I2C device is the one at 0x30. Every stream start powers
// them up and every prepare loads the whole register set again, for PCM or
// for DSD as x1600_hiby_r3proii_sound_card.ko says through
// cs43198_set_dsd_en(). Registers are written only while a stream is open.
//
// Mixer controls (reading any of them returns 0):
//   Left/Right Playback Volume   0..255, the attenuation register value
//   Digital Filter   PCM filter 0..3 (fast/slow roll-off, low latency or
//                    phase compensated)
//   NOS_EN           1: PCM filter off (non-oversampling)
//   DRE_EN           dynamic range enhancement
//   Mute Output, Soft Mute   accepted, no effect
//
// Write-only sysfs files on the I2C device:
//   write_reg_val    "REG VAL" in hex: write VAL to REG on both DACs
//   reg_val          "w REG VAL": the same; "r REG N": read N registers of
//                    the right DAC, the values are not reported
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

#define CS43198_ADDR_RIGHT	0x30
#define CS43198_ADDR_LEFT	0x33

// Registers, 24-bit addresses.
#define CS43198_DSD_VOL_B	0x070000
#define CS43198_DSD_VOL_A	0x070001
#define CS43198_PCM_FILTER	0x090000
#define CS43198_PCM_VOL_B	0x090001
#define CS43198_PCM_VOL_A	0x090002

#define CS43198_REG_END		0xffffff

struct cs43198_reg {
	unsigned int reg;
	unsigned int val;
};

struct cs43198_pdata {
	int reset_gpio;
	int power_gpio;
};

struct cs43198_priv {
	struct cs43198_pdata *pdata;
	struct mutex reg_mutex;
	struct snd_pcm_hw_params params;	// of the last hw_params
	int dsd_en;
	int powered;				// a stream is open
};

static int cs43198_i2c_bus_num = -1;
static int cs43198_pwr_gpio = -1;
static int cs43198_pwr_en_level = 1;
static int cs43198_rst_gpio = -1;
static int cs43198_rst_en_level;
static int cs43198_mute_gpio = -1;
static int cs43198_mute_en_level = 1;
static int cs43198_po_sel_gpio = -1;
static int cs43198_po_sel_en_level;

module_param(cs43198_i2c_bus_num, int, 0644);
module_param_cb(cs43198_pwr_gpio, &param_gpio_ops, &cs43198_pwr_gpio, 0644);
module_param(cs43198_pwr_en_level, int, 0644);
module_param_cb(cs43198_rst_gpio, &param_gpio_ops, &cs43198_rst_gpio, 0644);
module_param(cs43198_rst_en_level, int, 0644);
module_param_cb(cs43198_mute_gpio, &param_gpio_ops, &cs43198_mute_gpio, 0644);
module_param(cs43198_mute_en_level, int, 0644);
module_param_cb(cs43198_po_sel_gpio, &param_gpio_ops, &cs43198_po_sel_gpio, 0644);
module_param(cs43198_po_sel_en_level, int, 0644);

static struct cs43198_pdata cs43198_data = {
	.reset_gpio = -1,
	.power_gpio = -1,
};

static struct i2c_board_info cs43198_board_info = {
	I2C_BOARD_INFO("cs43198", CS43198_ADDR_RIGHT),
	.platform_data = &cs43198_data,
};

static struct i2c_client *cs43198_i2c_dev;
static struct i2c_client *this_i2c;
static struct cs43198_priv *g_cs43198;

static int lr_flag;
static int nos_enable;
static int dre_enable = 1;
static u8 filter_value = 0x02;
static u8 user_volume_left = 0xff;
static u8 user_volume_right = 0xff;

// Loaded on every prepare, then patched by the PCM or DSD set below.
// CS43198_PCM_FILTER here follows the Digital Filter and NOS_EN controls.
static struct cs43198_reg cs43198_default_regs[] = {
	{ 0x010006, 0x04 }, { 0x020052, 0x06 }, { 0x01000b, 0x01 },
	{ 0x01000c, 0x00 }, { 0x05000a, 0x07 }, { 0x05000b, 0x0f },
	{ 0x06000a, 0x07 }, { 0x06000b, 0x0f }, { 0x070000, 0xff },
	{ 0x070001, 0xff }, { 0x070004, 0x02 }, { 0x070006, 0x40 },
	{ 0x090000, 0x02 }, { 0x090001, 0xff }, { 0x090002, 0xff },
	{ 0x090003, 0x90 }, { 0x090004, 0x00 }, { 0x0b0000, 0x06 },
	{ 0x080000, 0x31 }, { 0x070002, 0xa0 }, { 0x020000, 0xae },
	{ 0x09000a, 0xf0 }, { 0x09000b, 0x0c },
	{ CS43198_REG_END, 0x00 },
};

static const struct cs43198_reg cs43198_dsd_regs[] = {
	{ 0x180009, 0xd1 }, { 0x18000a, 0x4b }, { 0x18000b, 0xda },
	{ 0x18000c, 0x7f }, { 0x18000f, 0xf7 }, { 0x180010, 0x04 },
	{ 0x180019, 0xd1 }, { 0x18001a, 0x4b }, { 0x18001b, 0xda },
	{ 0x18001c, 0x7f }, { 0x18001f, 0x6d }, { 0x180020, 0x02 },
	{ 0x180029, 0xd1 }, { 0x18002a, 0x4b }, { 0x18002b, 0xda },
	{ 0x18002c, 0x7f }, { 0x18002f, 0x84 }, { 0x180030, 0x02 },
	{ 0x180039, 0xd1 }, { 0x18003a, 0x4b }, { 0x18003b, 0xda },
	{ 0x18003c, 0x7f }, { 0x18003f, 0xbf }, { 0x180040, 0x01 },
	{ CS43198_REG_END, 0xff },
};

static const struct cs43198_reg cs43198_pcm_regs[] = {
	{ 0x180009, 0x2f }, { 0x18000a, 0xb4 }, { 0x18000b, 0x25 },
	{ 0x18000c, 0x80 }, { 0x18000f, 0x09 }, { 0x180010, 0xfb },
	{ 0x180019, 0x2f }, { 0x18001a, 0xb4 }, { 0x18001b, 0x25 },
	{ 0x18001c, 0x80 }, { 0x18001f, 0x93 }, { 0x180020, 0xfd },
	{ 0x180029, 0x2f }, { 0x18002a, 0xb4 }, { 0x18002b, 0x25 },
	{ 0x18002c, 0x80 }, { 0x18002f, 0x7c }, { 0x180030, 0xfd },
	{ 0x180039, 0x2f }, { 0x18003a, 0xb4 }, { 0x18003b, 0x25 },
	{ 0x18003c, 0x80 }, { 0x18003f, 0x41 }, { 0x180040, 0xfe },
	{ CS43198_REG_END, 0xff },
};

// One register write: the address MSB first, a zero byte, the value.
// -ENODEV while no stream is open.
static int cs43198_i2c_write_reg(u16 addr, unsigned int reg, u8 val)
{
	struct i2c_client *client = this_i2c;
	u8 buf[5];
	struct i2c_msg msg = {
		.addr = addr,
		.flags = 0,
		.len = sizeof(buf),
		.buf = buf,
	};
	int ret;

	if (!client || !g_cs43198 || !g_cs43198->powered)
		return -ENODEV;

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

static int cs43198_write_reg_right(unsigned int reg, u8 val)
{
	int ret;

	mutex_lock(&g_cs43198->reg_mutex);
	ret = cs43198_i2c_write_reg(CS43198_ADDR_RIGHT, reg, val);
	mutex_unlock(&g_cs43198->reg_mutex);
	return ret;
}

static int cs43198_write_reg_left(unsigned int reg, u8 val)
{
	int ret;

	mutex_lock(&g_cs43198->reg_mutex);
	ret = cs43198_i2c_write_reg(CS43198_ADDR_LEFT, reg, val);
	mutex_unlock(&g_cs43198->reg_mutex);
	return ret;
}

static int cs43198_write_reg_both(unsigned int reg, u8 val)
{
	cs43198_write_reg_left(reg, val);
	return cs43198_write_reg_right(reg, val);
}

static void cs43198_write_reg_array(const struct cs43198_reg *r)
{
	for (; r->reg != CS43198_REG_END; r++)
		cs43198_write_reg_both(r->reg, r->val);
}

// Sets the PCM filter now and in the default set.
static void cs43198_set_pcm_filter(u8 val)
{
	struct cs43198_reg *r;

	for (r = cs43198_default_regs; r->reg != CS43198_REG_END; r++)
		if (r->reg == CS43198_PCM_FILTER)
			r->val = val;
	cs43198_write_reg_both(CS43198_PCM_FILTER, val);
}

static int cs43198_write_left_vol(u8 vol)
{
	cs43198_write_reg_left(CS43198_PCM_VOL_A, vol);
	cs43198_write_reg_left(CS43198_PCM_VOL_B, vol);
	cs43198_write_reg_left(CS43198_DSD_VOL_A, vol);
	cs43198_write_reg_left(CS43198_DSD_VOL_B, vol);
	return 0;
}

static int cs43198_write_right_vol(u8 vol)
{
	cs43198_write_reg_right(CS43198_PCM_VOL_A, vol);
	cs43198_write_reg_right(CS43198_PCM_VOL_B, vol);
	cs43198_write_reg_right(CS43198_DSD_VOL_A, vol);
	cs43198_write_reg_right(CS43198_DSD_VOL_B, vol);
	return 0;
}

static int cs43198_dre_enable(int en)
{
	cs43198_write_reg_both(0x010010, 0x99);
	if (en) {
		cs43198_write_reg_both(0x0c0001, 1);
		cs43198_write_reg_both(0x010010, 0);
		cs43198_write_reg_both(0x010010, 0x99);
		cs43198_write_reg_both(0x0c0056, 0x2f);
	} else {
		cs43198_write_reg_both(0x0c0001, 0);
	}
	return cs43198_write_reg_both(0x010010, 0);
}

// --- mixer controls ---------------------------------------------------------

static int cs43198_dummy_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int cs43198_vol_left_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	long v = ucontrol->value.integer.value[0];

	if (v != user_volume_left) {
		user_volume_left = v;
		cs43198_write_left_vol(v);
	}
	return 0;
}

static int cs43198_vol_right_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	long v = ucontrol->value.integer.value[0];

	if (v != user_volume_right) {
		user_volume_right = v;
		cs43198_write_right_vol(v);
	}
	return 0;
}

static int cs43198_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

// Bits 7..6 of the PCM filter register; bit 1 stays set.
static int cs43198_digital_filter_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	unsigned long v = ucontrol->value.integer.value[0];

	if (v >= 4)
		v = 0;
	filter_value = (v << 6) | 0x02;
	if (!nos_enable)
		cs43198_set_pcm_filter(filter_value);
	return 0;
}

static int cs43198_soft_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int cs43198_nos_en_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	unsigned long v = ucontrol->value.integer.value[0];

	if (v >= 2)
		v = 0;
	mdelay(150);
	cs43198_set_pcm_filter(v ? 0x22 : filter_value);
	nos_enable = v;
	return 0;
}

static int cs43198_dre_en_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	unsigned long v = ucontrol->value.integer.value[0];

	if (v >= 2)
		v = 0;
	cs43198_dre_enable(v);
	dre_enable = v;
	return 0;
}

static const struct snd_kcontrol_new cs43198_snd_controls[] = {
	SOC_SINGLE_EXT("Left Playback Volume", 0, 0, 255, 0, cs43198_dummy_get, cs43198_vol_left_put),
	SOC_SINGLE_EXT("Right Playback Volume", 0, 0, 255, 0, cs43198_dummy_get, cs43198_vol_right_put),
	SOC_SINGLE_EXT("Mute Output", 0, 0, 1, 0, cs43198_dummy_get, cs43198_mute_put),
	SOC_SINGLE_EXT("Digital Filter", 0, 0, 4, 0, cs43198_dummy_get, cs43198_digital_filter_put),
	SOC_SINGLE_EXT("Soft Mute", 0, 0, 1, 0, cs43198_dummy_get, cs43198_soft_mute_put),
	SOC_SINGLE_EXT("NOS_EN", 0, 0, 1, 0, cs43198_dummy_get, cs43198_nos_en_put),
	SOC_SINGLE_EXT("DRE_EN", 0, 0, 1, 0, cs43198_dummy_get, cs43198_dre_en_put),
};

// --- DAI --------------------------------------------------------------------

// Called by x1600_hiby_r3proii_sound_card.ko before a stream: 1 for DSD.
void cs43198_set_dsd_en(int en)
{
	if (g_cs43198)
		g_cs43198->dsd_en = en;
}
EXPORT_SYMBOL(cs43198_set_dsd_en);

static int cs43198_dai_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct cs43198_priv *cs43198 = snd_soc_codec_get_drvdata(dai->codec);
	struct cs43198_pdata *pdata = cs43198->pdata;

	if (gpio_is_valid(pdata->reset_gpio))
		gpio_set_value(pdata->reset_gpio, 0);
	if (gpio_is_valid(pdata->power_gpio)) {
		gpio_set_value(pdata->power_gpio, 1);
		mdelay(100);
	}
	if (gpio_is_valid(pdata->reset_gpio)) {
		gpio_set_value(pdata->reset_gpio, 1);
		mdelay(20);
	}
	cs43198->powered = 1;
	return 0;
}

static void cs43198_dai_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct cs43198_priv *cs43198 = snd_soc_codec_get_drvdata(dai->codec);
	struct cs43198_pdata *pdata = cs43198->pdata;

	cs43198->powered = 0;
	if (gpio_is_valid(pdata->power_gpio))
		gpio_set_value(pdata->power_gpio, 0);
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_set_value(pdata->reset_gpio, 0);
}

static int cs43198_dai_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	memcpy(&g_cs43198->params, params, sizeof(*params));
	return 0;
}

// The speed setting for a PCM rate.
static unsigned int cs43198_pcm_speed(unsigned int rate)
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

// Resets the DACs and loads the register set for the stream.
static int cs43198_dai_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct cs43198_priv *cs43198 = g_cs43198;
	unsigned int rate = params_rate(&cs43198->params);
	unsigned int speed;

	if (gpio_is_valid(cs43198->pdata->reset_gpio)) {
		gpio_set_value(cs43198->pdata->reset_gpio, 0);
		udelay(5000);
		gpio_set_value(cs43198->pdata->reset_gpio, 1);
		mdelay(20);
	}

	cs43198_write_reg_array(cs43198_default_regs);

	if (cs43198->dsd_en) {
		printk("cs43198_dai_prepare dsd\n");
		cs43198_write_reg_array(cs43198_dsd_regs);
		cs43198_dre_enable(dre_enable);
		cs43198_write_reg_left(0x070006, 0x89);
		cs43198_write_reg_right(0x070006, 0x89);
		// The speed setting for the frame rate of the DSD stream.
		if (rate == 352800)
			speed = 1;
		else if (rate == 705600)
			speed = 2;
		else
			speed = 0;
		cs43198_write_reg_both(0x010006, 0x04);
		cs43198_write_reg_both(0x070004, (speed << 2) | 0x12);
		cs43198_write_reg_both(0x020000, 0xce);
	} else {
		printk("cs43198_dai_prepare pcm\n");
		cs43198_write_reg_array(cs43198_pcm_regs);
		cs43198_dre_enable(dre_enable);
		cs43198_write_reg_left(0x090004, 0);
		cs43198_write_reg_right(0x090004, 0);
		speed = cs43198_pcm_speed(rate);
		cs43198_write_reg_both(0x070004, 0x02);
		cs43198_write_reg_both(0x01000b, speed);
	}

	cs43198_write_left_vol(user_volume_left);
	cs43198_write_right_vol(user_volume_right);
	return 0;
}

// Format, clock, mute and trigger are fixed by the sound card; these accept
// any setting.
static int cs43198_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int cs43198_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int cs43198_dai_digital_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int cs43198_dai_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops cs43198_dai_drv_ops = {
	.set_sysclk = cs43198_dai_set_sysclk,
	.set_fmt = cs43198_dai_set_fmt,
	.digital_mute = cs43198_dai_digital_mute,
	.startup = cs43198_dai_startup,
	.shutdown = cs43198_dai_shutdown,
	.hw_params = cs43198_dai_hw_params,
	.prepare = cs43198_dai_prepare,
	.trigger = cs43198_dai_trigger,
};

// Bit 16 is set as in the vendor driver; it names no rate in this kernel.
#define CS43198_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 | \
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000 | \
			 SNDRV_PCM_RATE_352800 | SNDRV_PCM_RATE_384000 | SNDRV_PCM_RATE_768000 | \
			 (1 << 16))

static struct snd_soc_dai_driver cs43198_dai_drv = {
	.name = "cs43198-hifi",
	.ops = &cs43198_dai_drv_ops,
	.playback = {
		.stream_name = "Playback",
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
		.rates = CS43198_RATES,
		.channels_min = 1,
		.channels_max = 2,
	},
};

// --- codec ------------------------------------------------------------------

// The registers are loaded per stream; the codec callbacks have nothing to do.
static int cs43198_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43198_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43198_codec_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43198_codec_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int cs43198_codec_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
	return 0;
}

static struct snd_soc_codec_driver cs43198_codec_drv = {
	.probe = cs43198_codec_probe,
	.remove = cs43198_codec_remove,
	.suspend = cs43198_codec_suspend,
	.resume = cs43198_codec_resume,
	.set_bias_level = cs43198_codec_set_bias_level,
	.component_driver = {
		.controls = cs43198_snd_controls,
		.num_controls = ARRAY_SIZE(cs43198_snd_controls),
	},
};

// --- sysfs ------------------------------------------------------------------

static ssize_t cs43198_write_lr_flag(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	lr_flag = !strcmp(buf, "right");
	return count;
}

static ssize_t cs43198_write_reg(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int reg;
	u8 val;

	if (sscanf(buf, "%x %hhx", &reg, &val) != 2)
		return -EINVAL;
	cs43198_write_reg_both(reg, val);
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
		cs43198_write_reg_both(reg, n);
	}
	return count;
}

static DEVICE_ATTR(write_lr_flag, 0200, NULL, cs43198_write_lr_flag);
static DEVICE_ATTR(write_reg_val, 0200, NULL, cs43198_write_reg);
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

static void cs43198_free_gpios(struct cs43198_pdata *pdata)
{
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->power_gpio))
		gpio_free(pdata->power_gpio);
}

// Takes the power and reset pins, both low: the DACs stay off until a stream
// starts.
static int cs43198_platform_init(struct cs43198_pdata *pdata)
{
	int ret;

	if (gpio_is_valid(pdata->power_gpio)) {
		ret = gpio_request(pdata->power_gpio, "cs43198_power");
		if (ret < 0) {
			printk("[CS43198_DUAL] Gpio request failed. cs43198_power pin:%d\n", pdata->power_gpio);
			goto err;
		}
		gpio_direction_output(pdata->power_gpio, 0);
		printk("[CS43198_DUAL] cs43198_power pin:%d output\n", pdata->power_gpio);
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request(pdata->reset_gpio, "cs43198_reset");
		if (ret < 0) {
			printk("[CS43198_DUAL] Gpio request failed. cs43198_reset pin:%d\n", pdata->reset_gpio);
			if (gpio_is_valid(pdata->power_gpio))
				gpio_free(pdata->power_gpio);
			goto err;
		}
		gpio_direction_output(pdata->reset_gpio, 0);
		printk("[CS43198_DUAL] cs43198_reset pin:%d output\n", pdata->reset_gpio);
	}
	return 0;

err:
	printk("[CS43198_DUAL] %s Occur error. error=%d\n", __func__, ret);
	return ret;
}

// The pins come from the device tree node when there is one, else from the
// module parameters.
static struct cs43198_pdata *cs43198_dt_pdata(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct cs43198_pdata *pdata;
	enum of_gpio_flags flags;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		printk("[CS43198_DUAL] Failed to allocate memory\n");
		return NULL;
	}

	pdata->power_gpio = of_get_named_gpio_flags(np, "cs43198,power-gpio", 0, &flags);
	if (!gpio_is_valid(pdata->power_gpio))
		printk("[CS43198_DUAL] Unable to get power-gpio\n");
	else
		printk("[CS43198_DUAL] Get pdata->power_gpio gpio %d\n", pdata->power_gpio);

	pdata->reset_gpio = of_get_named_gpio_flags(np, "cs43198,reset-gpio", 0, &flags);
	if (!gpio_is_valid(pdata->reset_gpio))
		printk("[CS43198_DUAL] Unable to get reset-gpio\n");
	else
		printk("[CS43198_DUAL] Get pdata->reset gpio %d\n", pdata->reset_gpio);

	return pdata;
}

static int cs43198_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct cs43198_priv *cs43198;
	struct cs43198_pdata *pdata;
	int ret;

	cs43198 = devm_kzalloc(dev, sizeof(*cs43198), GFP_KERNEL);
	if (!cs43198) {
		ret = -ENOMEM;
		goto err;
	}

	if (dev->of_node) {
		pdata = cs43198_dt_pdata(dev);
		if (!pdata) {
			ret = -ENOMEM;
			goto err;
		}
	} else {
		pdata = dev->platform_data;
	}
	cs43198->pdata = pdata;
	i2c_set_clientdata(client, cs43198);
	this_i2c = client;

	ret = cs43198_platform_init(pdata);
	if (ret < 0) {
		printk("[CS43198_DUAL] Platform initial failed.\n");
		goto err_free;
	}

	sysfs_create_group(&dev->kobj, &sys_codec_group);

	ret = snd_soc_register_codec(dev, &cs43198_codec_drv, &cs43198_dai_drv, 1);
	if (ret < 0) {
		printk("[CS43198_DUAL] Register codec failed.\n");
		sysfs_remove_group(&dev->kobj, &sys_codec_group);
		cs43198_free_gpios(pdata);
		goto err_free;
	}

	g_cs43198 = cs43198;
	mutex_init(&cs43198->reg_mutex);
	cs43198->powered = 0;
	return 0;

err_free:
	this_i2c = NULL;
	i2c_set_clientdata(client, NULL);
	devm_kfree(dev, cs43198);
err:
	printk("[CS43198_DUAL] %s Occur error. error=%d\n", __func__, ret);
	return ret;
}

static int cs43198_i2c_remove(struct i2c_client *client)
{
	struct cs43198_priv *cs43198 = i2c_get_clientdata(client);

	snd_soc_unregister_codec(&client->dev);
	sysfs_remove_group(&client->dev.kobj, &sys_codec_group);
	cs43198_free_gpios(cs43198->pdata);
	g_cs43198 = NULL;
	devm_kfree(&client->dev, cs43198);
	i2c_set_clientdata(client, NULL);
	this_i2c = NULL;
	return 0;
}

static const struct i2c_device_id cs43198_i2c_id[] = {
	{ "cs43198", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cs43198_i2c_id);

static struct i2c_driver cs43198_i2c_driver = {
	.driver = {
		.name = "cs43198",
		.owner = THIS_MODULE,
	},
	.probe = cs43198_i2c_probe,
	.remove = cs43198_i2c_remove,
	.id_table = cs43198_i2c_id,
};

static int __init cs43198_init(void)
{
	int ret;

	cs43198_data.reset_gpio = cs43198_rst_gpio;
	cs43198_data.power_gpio = cs43198_pwr_gpio;

	ret = i2c_add_driver(&cs43198_i2c_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register CS43198 I2C driver\n");
		return ret;
	}

	cs43198_i2c_dev = i2c_register_device(&cs43198_board_info, cs43198_i2c_bus_num);
	if (!cs43198_i2c_dev) {
		printk(KERN_ERR "Failed to register CS43198 i2c device\n");
		i2c_del_driver(&cs43198_i2c_driver);
		return -EINVAL;
	}
	return 0;
}
module_init(cs43198_init);

static void __exit cs43198_exit(void)
{
	i2c_unregister_device(cs43198_i2c_dev);
	i2c_del_driver(&cs43198_i2c_driver);
}
module_exit(cs43198_exit);

MODULE_DESCRIPTION("Two CS43198 DACs of the HiBy R3 Pro II");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
