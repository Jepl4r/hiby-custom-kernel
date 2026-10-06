// SPDX-License-Identifier: GPL-2.0
//
// x1600_hiby_r1_sound_card -- the ASoC machine driver of the HiBy R1: the
// X1600 I2S controller (soc_aic.ko) and a CS43131 DAC on I2C bus 3
// (codec_cs43131.ko).
//
// A drop-in replacement for the vendor module of the same name. The platform
// device, the card, the DAI link and the control keep the vendor names, which
// the player and the stock scripts look for:
//
//   card     "hiby-sound-card" on the platform device "hiby-hifi-board.0"
//   link     "x1600-i2s": ingenic-aic and cs43131-hifi on cs43131.3-0030,
//            I2S with the codec as clock and frame slave
//   control  "Output Port Switch", 0..5: 2 and 3 power the CS43131 up, any
//            other value powers it down
//
// Loaded after soc_aic.ko and codec_cs43131.ko.

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

// Exported by codec_cs43131.ko: drives the DAC's power GPIO.
extern void cs43131_set_power(int on);

// Clock ids of the I2S controller's set_sysclk (soc_aic.ko): MCLK output on
// and off.
#define AIC_SYSCLK_MCLK_ON	2
#define AIC_SYSCLK_MCLK_OFF	3

static int codec_board_ot;

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

static int codec_board_ot_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	int val = ucontrol->value.integer.value[0];

	if (val == codec_board_ot)
		return 0;
	cs43131_set_power(val == 2 || val == 3);
	codec_board_ot = val;
	return 0;
}

static const struct snd_kcontrol_new codec_board_controls[] = {
	SOC_SINGLE_EXT("Output Port Switch", 0, 0, 5, 0,
		       codec_board_ot_get, codec_board_ot_put),
};

// MCLK for the codec: 22.5792 MHz for the 44.1 kHz family, 24.576 MHz for
// the 48 kHz family, 0 for any other rate.
static unsigned int codec_board_mclk(unsigned int rate)
{
	switch (rate) {
	case 44100:
	case 88200:
	case 176400:
	case 352800:
		return 22579200;
	case 48000:
	case 96000:
	case 192000:
	case 384000:
		return 24576000;
	default:
		return 0;
	}
}

static int codec_board_i2s_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	// The X1600 is the I2S master and drives MCLK.
	ret = snd_soc_dai_set_sysclk(cpu_dai, AIC_SYSCLK_MCLK_ON,
				     codec_board_mclk(params_rate(params)),
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;
	return snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				   SND_SOC_DAIFMT_CBS_CFS);
}

static int codec_board_i2s_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	return snd_soc_dai_set_sysclk(rtd->cpu_dai, AIC_SYSCLK_MCLK_OFF, 0,
				      SND_SOC_CLOCK_IN);
}

static struct snd_soc_ops i2s_ops = {
	.hw_params = codec_board_i2s_hw_params,
	.hw_free = codec_board_i2s_hw_free,
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
		.codec_name = "cs43131.3-0030",
		.codec_dai_name = "cs43131-hifi",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS,
		.init = codec_board_dai_link_init,
		.ops = &i2s_ops,
	},
};

static struct snd_soc_card codec_snd_card = {
	.name = "hiby-sound-card",
	.owner = THIS_MODULE,
	.dai_link = codec_board_dais,
	.num_links = ARRAY_SIZE(codec_board_dais),
	.controls = codec_board_controls,
	.num_controls = ARRAY_SIZE(codec_board_controls),
};

static int codec_board_probe(struct platform_device *pdev)
{
	codec_snd_card.dev = &pdev->dev;
	// Deferred until the I2S controller and the codec have registered.
	if (snd_soc_register_card(&codec_snd_card))
		return -EPROBE_DEFER;
	return 0;
}

static int codec_board_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&codec_snd_card);
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

MODULE_DESCRIPTION("HiBy R1 sound card (X1600 I2S + CS43131)");
MODULE_AUTHOR("Mattia D'Oronzo <doronzomattia26@icloud.com>");
MODULE_LICENSE("GPL");
