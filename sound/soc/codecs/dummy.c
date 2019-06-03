/*
 * linux/sound/soc/codecs/dummy.c
 * Base on linux/sound/soc/codecs/wm8753.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * 			 Yael Sheli Chemla<yael.s.shemla@marvell.com>
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
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#define HDMI_AUDIO_HIFI_RATES SNDRV_PCM_RATE_8000_48000

#define HDMI_AUDIO_HIFI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_codec_driver soc_codec_dev_dummy;

struct snd_soc_dai_driver dummy_audio_dai = {
	.name = "dummy-dai",
	.id = 1,
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = HDMI_AUDIO_HIFI_RATES,
		.formats = HDMI_AUDIO_HIFI_FORMATS,
	},
};

static int __devinit dummy_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_dummy,
			&dummy_audio_dai, 1);
}

static int __devexit dummy_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver dummy_codec_driver = {
	.driver = {
		   .name = "dummy-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = dummy_probe,
	.remove = __devexit_p(dummy_remove),
};

static int __init dummy_audio_modinit(void)
{
	return platform_driver_register(&dummy_codec_driver);
}
module_init(dummy_audio_modinit);

static void __exit dummy_audio_exit(void)
{
	platform_driver_unregister(&dummy_codec_driver);
}
module_exit(dummy_audio_exit);

MODULE_DESCRIPTION("ASoC dummy audio driver");
MODULE_LICENSE("GPL");


