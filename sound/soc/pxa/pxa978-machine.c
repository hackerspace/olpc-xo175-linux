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
#include <linux/mfd/88pm8xxx.h>
#include <mach/../../generic.h>

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
	__raw_writel(0x02100000, ssp->mmio_base + SSPSP);

	return 0;
}

static void saarc_pcm_shutdown(struct snd_pcm_substream * substream)
{
	pxa95x_ssp_mfp_init(false);
	pr_info("[saarc_pcm_startup]: switch to GSSP1\n");
}

static int saarc_ihdmi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0, sscr1, val;

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
	val = (sscr1 | 0x00B01DC0) & (~0x03000000);/* config ssp as master*/
	__raw_writel(sscr0 | 0x41D0003F, ssp->mmio_base + SSCR0);
	__raw_writel(val, ssp->mmio_base + SSCR1);
	__raw_writel(0x02100000, ssp->mmio_base + SSPSP);

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

static struct snd_soc_ops saarc_ihdmi_ops = {
	.prepare	= saarc_ihdmi_prepare,
};

static struct snd_soc_dai_link saarc_pm860x_dai[] = {
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

static struct snd_soc_dai_link saarc_pm805_dai[] = {
	{
		.name		= "88PM860x I2S",
		.stream_name	= "I2S Audio",
		.cpu_dai_name	= "pxa95x-abu-dai",
		.codec_dai_name	= "88pm805-i2s",
		.platform_name	= "pxa95x-pcm-abu",
		.codec_name	= "88pm80x-codec",
		.init		= saarc_pm860x_init,
		.ops		= &saarc_i2s_ops,
	},
	{
		.name		= "88PM860x PCM",
		.stream_name	= "PCM Audio",
		.cpu_dai_name	= "pxa-ssp-dai.2",
		.codec_dai_name	= "88pm805-pcm",
		.platform_name	= "pxa-pcm-audio",
		.codec_name	= "88pm80x-codec",
		.ops		= &saarc_pcm_ops,
	}
};

static struct snd_soc_dai_link hdmi_dai[] = {
	{
		.name		= "iHDMI I2S",
		.stream_name	= "iHDMI Audio",
		.cpu_dai_name	= "pxa-ssp-dai.2",
		.codec_dai_name = "dummy-dai",
		.platform_name	= "pxa-pcm-audio",
		.codec_name	= "dummy-codec",
		.ops		= &saarc_ihdmi_ops,
	},
};

static struct snd_soc_card snd_soc_card_saarc_pm860x = {
	.name = "88pm860x",
	.dai_link = saarc_pm860x_dai,
	.num_links = ARRAY_SIZE(saarc_pm860x_dai),
};

static struct snd_soc_card snd_soc_card_saarc_pm805 = {
	.name = "88pm805",
	.dai_link = saarc_pm805_dai,
	.num_links = ARRAY_SIZE(saarc_pm805_dai),
};

static struct snd_soc_card snd_soc_card_hdmi = {
	.name  = "hdmi",
	.dai_link  = hdmi_dai,
	.num_links  = ARRAY_SIZE(hdmi_dai),
};

static int saarc_pm860x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

}

static struct platform_device *saarc_snd_device, *hdmi_snd_device;

static int __init saarc_init(void)
{
	int ret = 0;

	if (!machine_is_saarc())
		return -ENODEV;

	saarc_snd_device = platform_device_alloc("soc-audio", 0);
	if (!saarc_snd_device)
		return -ENOMEM;

	if (get_pmic_id() >= PM800_CHIP_A0)
		platform_set_drvdata(saarc_snd_device, &snd_soc_card_saarc_pm805);
	else
		platform_set_drvdata(saarc_snd_device, &snd_soc_card_saarc_pm860x);

	ret = platform_device_add(saarc_snd_device);
	if (ret)
		platform_device_put(saarc_snd_device);

	/* hdmi */
	hdmi_snd_device = platform_device_alloc("soc-audio", 1);
	if (hdmi_snd_device) {
		platform_set_drvdata(hdmi_snd_device, &snd_soc_card_hdmi);

		ret = platform_device_add(hdmi_snd_device);
		if (ret)
			platform_device_put(hdmi_snd_device);
	}

	return ret;
}

static void __exit saarc_exit(void)
{
	platform_device_unregister(saarc_snd_device);
	if (hdmi_snd_device)
		platform_device_unregister(hdmi_snd_device);
}

module_init(saarc_init);
module_exit(saarc_exit);

MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_DESCRIPTION("ALSA SoC 88PM860x Saarc");
MODULE_LICENSE("GPL");
