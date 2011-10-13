/*
 * saarc.c -- SoC audio for saarc
 *
 * Copyright (C) 2010 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <mach/audio.h>
#include <plat/ssp.h>

#include "../codecs/88pm860x-codec.h"
#include "pxa-ssp.h"

/*
 * SSP audio private data
 */
struct ssp_priv {
	struct ssp_device *ssp;
	unsigned int sysclk;
	int dai_fmt;
#ifdef CONFIG_PM
	uint32_t	cr0;
	uint32_t	cr1;
	uint32_t	to;
	uint32_t	psp;
#endif
};

static int saarc_pm860x_init(struct snd_soc_pcm_runtime *rtd);

#ifdef CONFIG_SND_PXA95X_DAPM_ENABLE
static struct snd_soc_jack hs_jack, mic_jack;

static struct snd_soc_jack_pin hs_jack_pins[] = {
	{ .pin = "Headset Stereophone",	.mask = SND_JACK_HEADPHONE, },
};

static struct snd_soc_jack_pin mic_jack_pins[] = {
	{ .pin = "Headset Mic 2",	.mask = SND_JACK_MICROPHONE, },
};

/* saarc machine dapm widgets */
static const struct snd_soc_dapm_widget saarc_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Stereophone", NULL),
	SND_SOC_DAPM_LINE("Lineout Out 1", NULL),
	SND_SOC_DAPM_LINE("Lineout Out 2", NULL),
	SND_SOC_DAPM_SPK("Ext Speaker", NULL),
	SND_SOC_DAPM_MIC("Ext Mic 1", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Ext Mic 3", NULL),
};

/* saarc machine audio map */
static const struct snd_soc_dapm_route audio_map[] = {
	{"Headset Stereophone", NULL, "HS1"},
	{"Headset Stereophone", NULL, "HS2"},

	{"Ext Speaker", NULL, "LSP"},
	{"Ext Speaker", NULL, "LSN"},

	{"Lineout Out 1", NULL, "LINEOUT1"},
	{"Lineout Out 2", NULL, "LINEOUT2"},

	{"MIC1P", NULL, "Mic1 Bias"},
	{"MIC1N", NULL, "Mic1 Bias"},
	{"Mic1 Bias", NULL, "Ext Mic 1"},

	{"MIC2P", NULL, "Mic1 Bias"},
	{"MIC2N", NULL, "Mic1 Bias"},
	{"Mic1 Bias", NULL, "Headset Mic 2"},

	{"MIC3P", NULL, "Mic3 Bias"},
	{"MIC3N", NULL, "Mic3 Bias"},
	{"Mic3 Bias", NULL, "Ext Mic 3"},
};
#endif

static int saarc_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int width = snd_pcm_format_physical_width(params_format(params));
	int ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA_SSP_CLK_NET_PLL, 0,
				     PM860X_CLK_DIR_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 0, PM860X_CLK_DIR_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 3, 3, 2, width);

	return ret;
}

static int saarc_i2s_startup(struct snd_pcm_substream * substream)
{
	pxa95x_abu_mfp_init(true);
	pr_info("[saarc_i2s_startup]: switch to ABU\n");
	return 0;
}

static int saarc_pcm_startup(struct snd_pcm_substream * substream)
{
	pxa95x_ssp_mfp_init(true);
	pr_info("[saarc_pcm_startup]: switch to BSSP3\n");
	return 0;
}

static int saarc_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0, sscr1;

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr1 = __raw_readl(ssp->mmio_base + SSCR1);

	__raw_writel(sscr0 | 0x41D0003F, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1 | 0x03B01DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x02100004, ssp->mmio_base + SSPSP);

	return 0;
}

static int saarc_pcm_shutdown(struct snd_pcm_substream * substream)
{
	pxa95x_ssp_mfp_init(false);
	pr_info("[saarc_pcm_startup]: switch to GSSP1\n");
	return 0;
}

static struct snd_soc_ops saarc_i2s_ops = {
	.hw_params	= saarc_i2s_hw_params,
	.startup	= saarc_i2s_startup,
};

static struct snd_soc_ops saarc_pcm_ops = {
	.startup	= saarc_pcm_startup,
	.shutdown	= saarc_pcm_shutdown,
	.prepare	= saarc_pcm_prepare,
};

static struct snd_soc_dai_link saarc_dai[] = {
	{
		.name		= "88PM860x I2S",
		.stream_name	= "I2S Audio",
		.cpu_dai_name	= "pxa95x-abu-dai",
		.codec_dai_name	= "88pm860x-i2s",
		.platform_name	= "pxa95x-pcm-abu",
		.codec_name	= "88pm860x-codec",
		.init		= saarc_pm860x_init,
		.ops		= &saarc_i2s_ops,
	},
	{
		.name		= "88PM860x PCM",
		.stream_name	= "PCM Audio",
		.cpu_dai_name	= "pxa-ssp-dai.2",
		.codec_dai_name	= "88pm860x-pcm",
		.platform_name	= "pxa-pcm-audio",
		.codec_name	= "88pm860x-codec",
		.ops		= &saarc_pcm_ops,
	}
};

static struct snd_soc_card snd_soc_card_saarc[] = {
	{
		.name = "88pm860x",
		.dai_link = saarc_dai,
		.num_links = ARRAY_SIZE(saarc_dai),
	},
};

static int saarc_pm860x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

#ifdef CONFIG_SND_PXA95X_DAPM_ENABLE
	snd_soc_dapm_new_controls(dapm, saarc_dapm_widgets,
				  ARRAY_SIZE(saarc_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	/* connected pins */
	snd_soc_dapm_enable_pin(dapm, "Ext Speaker");
	snd_soc_dapm_enable_pin(dapm, "Ext Mic 1");
	snd_soc_dapm_enable_pin(dapm, "Ext Mic 3");
	snd_soc_dapm_disable_pin(dapm, "Headset Mic 2");
	snd_soc_dapm_disable_pin(dapm, "Headset Stereophone");

	ret = snd_soc_dapm_sync(dapm);
	if (ret)
		return ret;

	/* Headset jack detection */
	snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE
			| SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2,
			&hs_jack);
	snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
			      hs_jack_pins);
	snd_soc_jack_new(codec, "Microphone Jack", SND_JACK_MICROPHONE,
			 &mic_jack);
	snd_soc_jack_add_pins(&mic_jack, ARRAY_SIZE(mic_jack_pins),
			      mic_jack_pins);

	/* headphone, microphone detection & headset short detection */
	pm860x_hs_jack_detect(codec, &hs_jack, SND_JACK_HEADPHONE,
			      SND_JACK_BTN_0, SND_JACK_BTN_1, SND_JACK_BTN_2);
	pm860x_mic_jack_detect(codec, &hs_jack, SND_JACK_MICROPHONE);

	return 0;
#else
	/* do not use DAPM, set all pin to NC */
	/* input widget */
	snd_soc_dapm_nc_pin(dapm, "AUX1");
	snd_soc_dapm_nc_pin(dapm, "AUX2");
	snd_soc_dapm_nc_pin(dapm, "MIC1P");
	snd_soc_dapm_nc_pin(dapm, "MIC1N");
	snd_soc_dapm_nc_pin(dapm, "MIC2P");
	snd_soc_dapm_nc_pin(dapm, "MIC2N");
	snd_soc_dapm_nc_pin(dapm, "MIC3P");
	snd_soc_dapm_nc_pin(dapm, "MIC3N");

	/* output widget */
	snd_soc_dapm_nc_pin(dapm, "HS1");
	snd_soc_dapm_nc_pin(dapm, "HS2");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2");
	snd_soc_dapm_nc_pin(dapm, "EARP");
	snd_soc_dapm_nc_pin(dapm, "EARN");
	snd_soc_dapm_nc_pin(dapm, "LSP");
	snd_soc_dapm_nc_pin(dapm, "LSN");

	ret = snd_soc_dapm_sync(dapm);

	return ret;
#endif
}

static struct platform_device *saarc_snd_device[ARRAY_SIZE(snd_soc_card_saarc)];

static int __init saarc_init(void)
{
	int i, ret = 0;

	if (!machine_is_saarc())
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(snd_soc_card_saarc); i++) {
		saarc_snd_device[i] = platform_device_alloc("soc-audio", i);
		if (!saarc_snd_device[i])
			break;

		platform_set_drvdata(saarc_snd_device[i], &snd_soc_card_saarc[i]);

		ret = platform_device_add(saarc_snd_device[i]);
		if (ret)
			platform_device_put(saarc_snd_device[i]);
	}

	return ret;
}

static void __exit saarc_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(snd_soc_card_saarc); i++)
		platform_device_unregister(saarc_snd_device[i]);
}

module_init(saarc_init);
module_exit(saarc_exit);

MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_DESCRIPTION("ALSA SoC 88PM860x Saarc");
MODULE_LICENSE("GPL");
