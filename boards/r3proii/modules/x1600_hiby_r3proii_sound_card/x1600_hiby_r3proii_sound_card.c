// SPDX-License-Identifier: GPL-2.0
//
// x1600_hiby_r3proii_sound_card -- the ASoC machine driver of the HiBy
// R3 Pro II: the X1600 I2S controller (soc_aic.ko), the HBC3000 clock and
// routing chip (sa_sound_hbc3000.ko) and a CS43198 DAC on I2C bus 3
// (codec_cs43198_dual.ko).
//
// A drop-in replacement for the vendor module of the same name. The platform
// device, the card, the DAI links and the controls keep the vendor names,
// which the player and the stock scripts look for:
//
//   card      "hiby-sound-card" on the platform device "hiby-hifi-board.0"
//   links     "x1600-i2s": ingenic-aic and cs43198-hifi on cs43198.3-0030;
//             "x1600-spdif": ingenic-aic and the dummy codec. Both I2S with
//             the codec side as clock and frame master (the HBC3000).
//   controls  "Output Port Switch" 0..7, the route (see codec_board_ot_put);
//             "Balance Lineout En", line level on the 4.4 mm jack;
//             "DOP_EN", DoP passthrough for rates from 176.4 kHz.
//             The last two read back as 0.
//
// Loaded after soc_aic.ko, sa_sound_hbc3000.ko and codec_cs43198_dual.ko.

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

// Exported by sa_sound_hbc3000.ko.
extern int hbc3000_enable(void);
extern int hbc3000_disable(void);
extern int hbc3000_start(void);
extern int hbc3000_stop(void);
extern int hbc3000_set_codec_type(int type);
extern int hbc3000_set_gpo(int gpo, int value);
extern int hbc3000_set_power_down(int xtal, int down);
extern int hbc3000_set_samplerate(int rate, int mode);
extern int hbc3000_set_spdif_mode(int spdif);

// Exported by codec_cs43198_dual.ko.
extern void cs43198_set_dsd_en(int en);

// Exported by soc_aic.ko: callbacks run from the controller's start and stop
// work, skipped while NULL.
extern void aic_slave_trigger(void (*start)(void *data),
			      void (*stop)(void *data));

#define GPIO_PF(n)		(5 * 32 + (n))
#define GPIO_DAC_PWR_EN		GPIO_PF(13)
#define GPIO_LO_PO_MUTE		GPIO_PF(12)
#define GPIO_PO_HP_MUTE		GPIO_PF(10)
#define GPIO_PO_BAL_MUTE	GPIO_PF(9)

#define HBC3000_CODEC_CS43198	4
#define HBC3000_RATE_PCM	0
#define HBC3000_RATE_DOP	2
#define DOP_MIN_RATE		176400

// How long the outputs stay muted after the CS43198 is powered up.
#define DAC_SETTLE_MS		50

// Clock id of the I2S controller's set_sysclk (soc_aic.ko): MCLK output off.
#define AIC_SYSCLK_MCLK_OFF	3

// Output Port Switch values.
#define ROUTE_LINEOUT		1
#define ROUTE_HP		2
#define ROUTE_BALANCED		3
#define ROUTE_SPDIF		4

static int balance_lineout_en;
static int codec_board_ot;
static int current_samplerate;
static int dop_en;
static struct delayed_work snd_card_delay_work;
static struct workqueue_struct *snd_card_wq;

struct gpo_setting {
	u8 gpo;
	u8 value;
};

// HBC3000 output lines for each route, written in this order.
static const struct gpo_setting gpo_balanced_lineout[] = {
	{ 0x0d, 0 }, { 0x10, 0 }, { 0x0e, 0 }, { 0x0f, 0 }, { 0x1d, 1 },
	{ 0x1e, 0 }, { 0x20, 0 }, { 0x23, 1 }, { 0x21, 1 }, { 0x22, 1 },
};

static const struct gpo_setting gpo_balanced_hp[] = {
	{ 0x0d, 1 }, { 0x10, 0 }, { 0x0e, 1 }, { 0x0f, 1 }, { 0x1d, 0 },
	{ 0x1e, 1 }, { 0x20, 1 }, { 0x23, 0 }, { 0x21, 1 }, { 0x22, 0 },
};

static const struct gpo_setting gpo_single_ended[] = {
	{ 0x0d, 1 }, { 0x10, 1 }, { 0x0e, 1 }, { 0x0f, 0 }, { 0x1d, 0 },
	{ 0x1e, 1 }, { 0x20, 0 }, { 0x1b, 1 }, { 0x1c, 1 }, { 0x23, 0 },
	{ 0x21, 0 }, { 0x22, 0 },
};

static const struct gpo_setting gpo_balanced_dac[] = {
	{ 0x1b, 1 }, { 0x1c, 1 },
};

static const struct gpo_setting gpo_analog_off[] = {
	{ 0x0d, 0 }, { 0x10, 0 }, { 0x0e, 0 }, { 0x0f, 0 }, { 0x1d, 0 },
	{ 0x1e, 0 }, { 0x20, 0 }, { 0x1b, 0 }, { 0x1c, 0 },
};

static void set_gpos(const struct gpo_setting *s, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		hbc3000_set_gpo(s[i].gpo, s[i].value);
}

#define SET_GPOS(table) set_gpos(table, ARRAY_SIZE(table))

// Mute (1) or unmute (0) the three analog output stages.
static void analog_mute(int mute)
{
	gpio_set_value(GPIO_PO_HP_MUTE, mute);
	gpio_set_value(GPIO_PO_BAL_MUTE, mute);
	gpio_set_value(GPIO_LO_PO_MUTE, mute);
}

// Powers the CS43198 up from off: DAC_PWR_EN low for 2 ms, then high.
static void dac_power_up(void)
{
	gpio_set_value(GPIO_DAC_PWR_EN, 0);
	mdelay(2);
	gpio_set_value(GPIO_DAC_PWR_EN, 1);
}

static int codec_board_ot_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	if (mc->reg)
		return -EINVAL;
	ucontrol->value.integer.value[0] = codec_board_ot;
	return 0;
}

// Routes:
//   1, 2  the 3.5 mm jack (line out, headphones): the two share one path,
//         so moving between them does nothing
//   3     the 4.4 mm jack; its mute, routing and unmute run on every write
//         of 3, so a change of "Balance Lineout En" applies on the next one
//   4     S/PDIF: the HBC3000 on, the DAC off, the analog stages muted
//   other everything off
// A write of the current route returns at once, apart from route 3 above.
//
// The analog stages stay muted while the HBC3000 is powered and loaded, the
// DAC powered up and the output switches set, and DAC_SETTLE_MS after.
static int codec_board_ot_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	int val = ucontrol->value.integer.value[0];
	bool change = val != codec_board_ot;

	if ((val == ROUTE_LINEOUT && codec_board_ot == ROUTE_HP) ||
	    (val == ROUTE_HP && codec_board_ot == ROUTE_LINEOUT))
		return 0;
	if (!change && val != ROUTE_BALANCED)
		return 0;

	switch (val) {
	case ROUTE_LINEOUT:
	case ROUTE_HP:
		analog_mute(1);
		mdelay(5);
		hbc3000_set_spdif_mode(0);
		hbc3000_enable();
		dac_power_up();
		SET_GPOS(gpo_single_ended);
		msleep(DAC_SETTLE_MS);
		analog_mute(0);
		break;
	case ROUTE_BALANCED:
		analog_mute(1);
		mdelay(5);
		if (change) {
			hbc3000_set_spdif_mode(0);
			hbc3000_enable();
			dac_power_up();
		}
		if (balance_lineout_en)
			SET_GPOS(gpo_balanced_lineout);
		else
			SET_GPOS(gpo_balanced_hp);
		if (change) {
			SET_GPOS(gpo_balanced_dac);
			msleep(DAC_SETTLE_MS);
		}
		analog_mute(0);
		if (!change)
			return 0;
		break;
	case ROUTE_SPDIF:
		hbc3000_set_spdif_mode(1);
		hbc3000_enable();
		gpio_set_value(GPIO_DAC_PWR_EN, 0);
		SET_GPOS(gpo_analog_off);
		analog_mute(1);
		break;
	default:
		hbc3000_set_spdif_mode(1);
		hbc3000_disable();
		gpio_set_value(GPIO_DAC_PWR_EN, 0);
		SET_GPOS(gpo_analog_off);
		analog_mute(1);
		break;
	}
	codec_board_ot = val;
	hbc3000_start();
	return 0;
}

static int balance_lineout_en_dummy_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int balance_lineout_en_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	balance_lineout_en = ucontrol->value.integer.value[0];
	return 0;
}

static int dop_en_dummy_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int dop_en_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	dop_en = ucontrol->value.integer.value[0];
	return 0;
}

static const struct snd_kcontrol_new codec_board_controls[] = {
	SOC_SINGLE_EXT("Balance Lineout En", 0, 0, 1, 0,
		       balance_lineout_en_dummy_get, balance_lineout_en_put),
	SOC_SINGLE_EXT("Output Port Switch", 0, 0, 7, 0,
		       codec_board_ot_get, codec_board_ot_put),
	SOC_SINGLE_EXT("DOP_EN", 0, 0, 1, 0,
		       dop_en_dummy_get, dop_en_put),
};

static int codec_board_i2s_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int rate = params_rate(params);
	int ret;

	current_samplerate = rate;
	if (rate >= DOP_MIN_RATE && dop_en == 1) {
		hbc3000_set_samplerate(rate, HBC3000_RATE_DOP);
		cs43198_set_dsd_en(1);
	} else {
		hbc3000_set_samplerate(rate, HBC3000_RATE_PCM);
		cs43198_set_dsd_en(0);
	}

	// The HBC3000 is the I2S master; the X1600 follows its clocks.
	ret = snd_soc_dai_set_sysclk(cpu_dai, AIC_SYSCLK_MCLK_OFF, rate,
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;
	hbc3000_start();
	return 0;
}

static int codec_board_i2s_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	return snd_soc_dai_set_sysclk(rtd->cpu_dai, AIC_SYSCLK_MCLK_OFF, 0,
				      SND_SOC_CLOCK_IN);
}

static int codec_board_i2s_trigger(struct snd_pcm_substream *substream,
				   int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return 0;
	default:
		return -EINVAL;
	}
}

static struct snd_soc_ops i2s_ops = {
	.hw_params = codec_board_i2s_hw_params,
	.hw_free = codec_board_i2s_hw_free,
	.trigger = codec_board_i2s_trigger,
};

static int codec_board_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static struct snd_soc_dai_link codec_board_dais[] = {
	{
		.name = "x1600-i2s",
		.stream_name = "x1600-i2s-pcm",
		.cpu_dai_name = "ingenic-aic",
		.platform_name = "ingenic-aic",
		.codec_name = "cs43198.3-0030",
		.codec_dai_name = "cs43198-hifi",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM,
		.init = codec_board_dai_link_init,
		.ops = &i2s_ops,
	},
	{
		.name = "x1600-spdif",
		.stream_name = "x1600-i2s-pcm",
		.cpu_dai_name = "ingenic-aic",
		.platform_name = "ingenic-aic",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM,
		.init = codec_board_dai_link_init,
		.ops = &i2s_ops,
	},
};

// In a suspend the HBC3000 loses its power after this card, the output
// switches with it. The analog stages are muted and the DAC powered down
// first; the route is forgotten, so the next write of one powers everything
// back and unmutes at the end.
static int codec_board_suspend_pre(struct snd_soc_card *card)
{
	cancel_delayed_work_sync(&snd_card_delay_work);
	analog_mute(1);
	mdelay(5);
	gpio_set_value(GPIO_DAC_PWR_EN, 0);
	codec_board_ot = 0;
	return 0;
}

static struct snd_soc_card codec_snd_card = {
	.name = "hiby-sound-card",
	.owner = THIS_MODULE,
	.dai_link = codec_board_dais,
	.num_links = ARRAY_SIZE(codec_board_dais),
	.controls = codec_board_controls,
	.num_controls = ARRAY_SIZE(codec_board_controls),
	.suspend_pre = codec_board_suspend_pre,
};

// Powers down an HBC3000 crystal 1 s after playback starts; aic_trigger_stop
// powers it back up. Crystal 2 for the 44.1 kHz family, 1 for the others.
static int playback_xtal(void)
{
	return current_samplerate % 44100 ? 1 : 2;
}

static void snd_card_work_schedule(struct work_struct *work)
{
	hbc3000_set_power_down(playback_xtal(), 1);
}

// Called by soc_aic.ko when the I2S controller starts and stops. With DoP the
// HBC3000 is started and stopped 10 ms later.
static void aic_trigger_start(void *data)
{
	if (dop_en)
		mdelay(10);
	hbc3000_start();
	cancel_delayed_work_sync(&snd_card_delay_work);
	queue_delayed_work(snd_card_wq, &snd_card_delay_work, HZ);
}

static void aic_trigger_stop(void *data)
{
	cancel_delayed_work_sync(&snd_card_delay_work);
	hbc3000_set_power_down(playback_xtal(), 0);
	if (dop_en)
		mdelay(10);
	hbc3000_stop();
}

static void gpio_request_output(int gpio, const char *name, int value)
{
	int ret;

	ret = gpio_request(gpio, name);
	if (ret) {
		pr_err("%s GPIO[%c]_%d error:%d\n", name, 'A' + gpio / 32,
		       gpio % 32, ret);
		return;
	}
	gpio_direction_output(gpio, value);
	pr_err("%s GPIO[%c]_%d success\n", name, 'A' + gpio / 32, gpio % 32);
}

static int codec_board_probe(struct platform_device *pdev)
{
	codec_snd_card.dev = &pdev->dev;
	// Deferred until the I2S controller and the codec have registered.
	if (snd_soc_register_card(&codec_snd_card))
		return -EPROBE_DEFER;

	snd_card_wq = create_singlethread_workqueue("snd_card");
	if (!snd_card_wq) {
		snd_soc_unregister_card(&codec_snd_card);
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&snd_card_delay_work, snd_card_work_schedule);

	gpio_request_output(GPIO_DAC_PWR_EN, "DAC_PWR_EN", 0);
	gpio_request_output(GPIO_LO_PO_MUTE, "LO_PO_MUTE", 1);
	gpio_request_output(GPIO_PO_HP_MUTE, "PO_HP_MUTE", 1);
	gpio_request_output(GPIO_PO_BAL_MUTE, "PO_BAL_MUTE", 1);
	hbc3000_set_codec_type(HBC3000_CODEC_CS43198);
	aic_slave_trigger(aic_trigger_start, aic_trigger_stop);
	return 0;
}

static int codec_board_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&codec_snd_card);
	aic_slave_trigger(NULL, NULL);
	cancel_delayed_work_sync(&snd_card_delay_work);
	destroy_workqueue(snd_card_wq);
	snd_card_wq = NULL;
	gpio_free(GPIO_PO_BAL_MUTE);
	gpio_free(GPIO_PO_HP_MUTE);
	gpio_free(GPIO_LO_PO_MUTE);
	gpio_free(GPIO_DAC_PWR_EN);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void codec_board_dev_release(struct device *dev)
{
}

static struct platform_device codec_board_device = {
	.name = "hiby-hifi-board",
	.id = 0,
	.dev = {
		.release = codec_board_dev_release,
	},
};

static struct platform_driver codec_board_driver = {
	.probe = codec_board_probe,
	.remove = codec_board_remove,
	.driver = {
		.name = "hiby-hifi-board",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
};

static int __init codec_board_init(void)
{
	int ret;

	ret = platform_device_register(&codec_board_device);
	if (ret)
		return ret;
	return platform_driver_register(&codec_board_driver);
}

static void __exit codec_board_exit(void)
{
	platform_device_unregister(&codec_board_device);
	platform_driver_unregister(&codec_board_driver);
}

module_init(codec_board_init);
module_exit(codec_board_exit);

MODULE_DESCRIPTION("HiBy R3 Pro II sound card (X1600 I2S + HBC3000 + CS43198)");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
