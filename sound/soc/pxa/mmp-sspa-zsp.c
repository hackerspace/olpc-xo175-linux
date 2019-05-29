/*
 * linux/sound/soc/pxa/mmp-sspa-zsp.c
 * Base on pxa2xx-ssp.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/io.h>
#include <mach/regs-sspa.h>
#include <mach/pxa910-squ.h>
#include <mach/mmp-zmq.h>
#include <plat/dma.h>
#include <plat/ssp.h>

#include "mmp-zsp-audio.h"
#include "mmp2-sspa.h"

static int mmp_sspa_zsp_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_device *sspa;
	struct platform_device *pdev;

	if (!cpu_dai->active) {
		sspa = sspa_request(cpu_dai->id + 1, "SSPA");
		snd_soc_dai_set_drvdata(dai, sspa);
		pdev = sspa->pdev;

		if (pdev->id == 0) {
			sspa->render_type[0] = ZMQ_MSG_ID_RENDER_PLAYBACK_0;
			sspa->render_type[1] = ZMQ_MSG_ID_RENDER_CAPTURE_0;
		} else if (pdev->id == 1) {
			sspa->render_type[0] = ZMQ_MSG_ID_RENDER_PLAYBACK_1;
			sspa->render_type[1] = ZMQ_MSG_ID_RENDER_CAPTURE_1;
		}
	}

	return 0;
}

static void mmp_sspa_zsp_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_device *sspa = snd_soc_dai_get_drvdata(cpu_dai);

	if (!cpu_dai->active)
		sspa_free(sspa);

	return;
}

#ifdef CONFIG_PM
static int mmp_sspa_zsp_suspend(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int mmp_sspa_zsp_resume(struct snd_soc_dai *cpu_dai)
{
	return 0;
}
#else
#define mmp_sspa_zsp_suspend	NULL
#define mmp_sspa_zsp_resume	NULL
#endif

/*
 * Set the SSP ports SYSCLK.
 */
static int mmp_sspa_zsp_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	/*FIX ME: SSPA sys clock*/
	return 0;
}

/*
 * Set the SSPA audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int mmp_sspa_zsp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_device *sspa = snd_soc_dai_get_drvdata(cpu_dai);
	u32 sspactl;

	/* bit size */
	sspactl = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		sspactl &= ~0x7;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		sspactl &= ~0x7;
		sspactl |= 0x2;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		sspactl &= ~0x7;
		sspactl |= 0x3;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		sspactl &= ~0x7;
		sspactl |= 0x4;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sspactl &= ~0x7;
		sspactl |= 0x5;
		break;
	default:
		return -EINVAL;
	}

	/* setup sspa, using I2S mode */
	sspa->zsp_sspa_conf.fwid = 32;
	sspa->zsp_sspa_conf.fsync_active = 64;
	sspa->zsp_sspa_conf.msl_select = SSPA_CONFIG_MS_SEL_MASTER;
	sspa->zsp_sspa_conf.fsp = SSPA_CONFIG_FRAME_SYNC_POLA_LOW;
	sspa->zsp_sspa_conf.ch_num = params_channels(params);
	sspa->zsp_sspa_conf.word_length = \
		SSPA_CONFIG_WORD_LENGTH_32_BITS;
	sspa->zsp_sspa_conf.sample_size = sspactl;
	sspa->zsp_sspa_conf.jst = SSPA_CONFIG_SAMPLE_JST_LEFT;
	sspa->zsp_sspa_conf.data_delay = SSPA_CONFIG_DATA_DELAY_1_BIT;
	sspa->zsp_sspa_conf.sample_rate = params_rate(params);

	return 0;
}

static int mmp_sspa_zsp_trigger(struct snd_pcm_substream *substream,
			      int cmd, struct snd_soc_dai *dai)
{
	/* nothing to do. everything is done in platform driver */
	return 0;
}

#define PXA688_SSPA_RATES 0xffffffff
#define PXA688_SSPA_FORMATS 0xffffffff

static struct snd_soc_dai_ops mmp_sspa_zsp_dai_ops = {
	.startup	= mmp_sspa_zsp_startup,
	.shutdown	= mmp_sspa_zsp_shutdown,
	.trigger	= mmp_sspa_zsp_trigger,
	.hw_params	= mmp_sspa_zsp_hw_params,
	.set_sysclk	= mmp_sspa_zsp_set_dai_sysclk,
};

struct snd_soc_dai_driver mmp_sspa_zsp_dai = {
	.suspend = mmp_sspa_zsp_suspend,
	.resume = mmp_sspa_zsp_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 128,
		.rates = PXA688_SSPA_RATES,
		.formats = PXA688_SSPA_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = PXA688_SSPA_RATES,
		.formats = PXA688_SSPA_FORMATS,
	},
	.ops = &mmp_sspa_zsp_dai_ops,
};
EXPORT_SYMBOL_GPL(mmp_sspa_zsp_dai);

static __devinit int asoc_mmp_sspa_zsp_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &mmp_sspa_zsp_dai);
}

static __devexit int asoc_mmp_sspa_zsp_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver asoc_mmp_sspa_zsp_driver = {
	.driver = {
		.name = "mmp-sspa-dai",
		.owner = THIS_MODULE,
	},
	.probe = asoc_mmp_sspa_zsp_probe,
	.remove = __devexit_p(asoc_mmp_sspa_zsp_remove),
};

static int __init mmp_sspa_zsp_modinit(void)
{
	return platform_driver_register(&asoc_mmp_sspa_zsp_driver);
}

module_init(mmp_sspa_zsp_modinit);

static void __exit mmp_sspa_zsp_exit(void)
{
	platform_driver_unregister(&asoc_mmp_sspa_zsp_driver);
}
module_exit(mmp_sspa_zsp_exit);

/* Module information */
MODULE_AUTHOR("lbyang@marvell.com");
MODULE_DESCRIPTION("MMP ZSP SSPA SoC Interface");
MODULE_LICENSE("GPL");
