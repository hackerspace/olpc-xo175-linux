/*
 * linux/sound/soc/pxa/emei-dkb.c
 *
 * Copyright (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <plat/ssp.h>
#include <mach/regs-mpmu.h>
#include <sound/pcm_params.h>
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

static int emei_dkb_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	/*
	 * init ssp clock: MPMU ssp clock, need to enable SYSCLK_EN
	 * and BITCLK_EN to make sure input/output clock work well
	 * for SSP unit
	 */
	__raw_writel(0xf820130b, MPMU_ISCCRX0);
	__raw_writel(0xf820130b, MPMU_ISCCRX1);

	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

	return 0;
}

static int emei_dkb_hifi_prepare(struct snd_pcm_substream *substream)
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

static struct snd_soc_ops emei_dkb_machine_ops = {
	.hw_params = emei_dkb_hw_params,
	.prepare = emei_dkb_hifi_prepare,
};

static struct snd_soc_dai_link emei_dkb_hifi_dai[] = {
{
	 .name = "88pm805 i2s",
	 .stream_name = "audio playback",
	 .codec_name = "88pm80x-codec",
	 .platform_name = "pxa910-squ-audio",
	 .cpu_dai_name = "pxa-ssp-dai.1",
	 .codec_dai_name = "88pm805-i2s",
	 .ops = &emei_dkb_machine_ops,
},
};

static struct snd_soc_card emei_dkb_snd_soc_machine = {
	.name = "emei_dkb-hifi",
	.dai_link = emei_dkb_hifi_dai,
	.num_links = ARRAY_SIZE(emei_dkb_hifi_dai),
};

static struct platform_device *emei_dkb_snd_device;

static int __init emei_dkb_init(void)
{
	int ret;

	if (!(machine_is_emeidkb()))
		return -ENODEV;

	emei_dkb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!emei_dkb_snd_device)
		return -ENOMEM;

	platform_set_drvdata(emei_dkb_snd_device, &emei_dkb_snd_soc_machine);
	ret = platform_device_add(emei_dkb_snd_device);

	if (ret)
		platform_device_put(emei_dkb_snd_device);

	return ret;
}

static void __exit emei_dkb_exit(void)
{
	platform_device_unregister(emei_dkb_snd_device);
}

module_init(emei_dkb_init);
module_exit(emei_dkb_exit);

/* Module information */
MODULE_AUTHOR("zhouqiao@marvell.com");
MODULE_DESCRIPTION("ALSA SoC TTC DKB");
MODULE_LICENSE("GPL");
