/*
 * linux/sound/soc/pxa/brownstone.c
 *
 * Copyright (C) 2011 Marvell International Ltd.
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
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/switch_headset.h>
#include <linux/mfd/wm8994/registers.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/regs-sspa.h>

#include "../codecs/wm8994.h"
#include "mmp-pcm.h"
#include "mmp-sspa.h"

#define BROWNSTONE_CTRL_ON	0
#define BROWNSTONE_CTRL_OFF	1

static struct snd_soc_card brownstone[];
static struct platform_device *brownstone_snd_device[2];

static struct clk *audio_clk;

static int brownstone_headphone_func;
static int brownstone_external_mic_func;
static int brownstone_spk_func;
static int brownstone_internal_mic_func;

static void brownstone_ext_control(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (brownstone_headphone_func == BROWNSTONE_CTRL_ON)
		snd_soc_dapm_enable_pin(dapm, "Headset Stereophone");
	else
		snd_soc_dapm_disable_pin(dapm, "Headset Stereophone");

	if (brownstone_external_mic_func == BROWNSTONE_CTRL_ON)
		snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	else
		snd_soc_dapm_disable_pin(dapm, "Headset Mic");

	if (brownstone_spk_func == BROWNSTONE_CTRL_ON) {
		snd_soc_dapm_enable_pin(dapm, "Ext Left Spk");
		snd_soc_dapm_enable_pin(dapm, "Ext Right Spk");
	} else {
		snd_soc_dapm_disable_pin(dapm, "Ext Left Spk");
		snd_soc_dapm_disable_pin(dapm, "Ext Right Spk");
	}

	if (brownstone_internal_mic_func == BROWNSTONE_CTRL_ON)
		snd_soc_dapm_enable_pin(dapm, "Main Mic");
	else
		snd_soc_dapm_disable_pin(dapm, "Main Mic");

	snd_soc_dapm_sync(dapm);
	return;
}

static int brownstone_get_headphone(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = brownstone_headphone_func;
	return 0;
}

static int brownstone_set_headphone(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (brownstone_headphone_func == ucontrol->value.integer.value[0])
		return 0;

	brownstone_headphone_func = ucontrol->value.integer.value[0];
	brownstone_ext_control(codec);
	return 1;
}

static int brownstone_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = brownstone_spk_func;
	return 0;
}

static int brownstone_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (brownstone_spk_func == ucontrol->value.integer.value[0])
		return 0;

	brownstone_spk_func = ucontrol->value.integer.value[0];
	brownstone_ext_control(codec);
	return 1;
}

static int brownstone_get_external_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = brownstone_external_mic_func;
	return 0;
}

static int brownstone_set_external_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (brownstone_external_mic_func == ucontrol->value.integer.value[0])
		return 0;

	brownstone_external_mic_func = ucontrol->value.integer.value[0];
	brownstone_ext_control(codec);
	return 1;
}

static int brownstone_get_internal_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = brownstone_internal_mic_func;
	return 0;
}

static int brownstone_set_internal_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (brownstone_internal_mic_func == ucontrol->value.integer.value[0])
		return 0;

	brownstone_internal_mic_func = ucontrol->value.integer.value[0];
	brownstone_ext_control(codec);
	return 1;
}

static const struct snd_soc_dapm_widget brownstone_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Left Spk", NULL),
	SND_SOC_DAPM_SPK("Ext Right Spk", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
};

static const struct snd_soc_dapm_route brownstone_dapm_routes[] = {
	{"Ext Left Spk", NULL, "SPKOUTLP"},
	{"Ext Left Spk", NULL, "SPKOUTLN"},

	{"Ext Right Spk", NULL, "SPKOUTRP"},
	{"Ext Right Spk", NULL, "SPKOUTRN"},

	{"Headset Stereophone", NULL, "HPOUT1L"},
	{"Headset Stereophone", NULL, "HPOUT1R"},

	{"IN1RN", NULL, "Headset Mic"},

	{"DMIC1DAT", NULL, "MICBIAS1"},
	{"MICBIAS1", NULL, "Main Mic"},
};

static const char *headphone_function[] = {"On", "Off"};
static const char *external_mic_function[] = {"On", "Off"};
static const char *spk_function[]  = {"On", "Off"};
static const char *internal_mic_function[]  = {"On", "Off"};
static const struct soc_enum brownstone_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, headphone_function),
	SOC_ENUM_SINGLE_EXT(2, external_mic_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, internal_mic_function),
};

static const struct snd_kcontrol_new brownstone_wm8994_controls[] = {
	SOC_ENUM_EXT("Headphone Function", brownstone_enum[0],
		     brownstone_get_headphone, brownstone_set_headphone),
	SOC_ENUM_EXT("External Mic Function", brownstone_enum[1],
		     brownstone_get_external_mic, brownstone_set_external_mic),
	SOC_ENUM_EXT("Speaker Function", brownstone_enum[2],
		     brownstone_get_spk, brownstone_set_spk),
	SOC_ENUM_EXT("Internal Mic Function", brownstone_enum[3],
		     brownstone_get_internal_mic, brownstone_set_internal_mic),
};

static int brownstone_wm8994_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int err;

	/* Add brownstone specific controls */
	err = snd_soc_add_controls(codec, brownstone_wm8994_controls,
				   ARRAY_SIZE(brownstone_wm8994_controls));
	if (err < 0)
		return err;

	/* add brownstone specific widgets */
	snd_soc_dapm_new_controls(dapm, brownstone_dapm_widgets,
				  ARRAY_SIZE(brownstone_dapm_widgets));

	/* set up brownstone specific audio routes */
	snd_soc_dapm_add_routes(dapm, brownstone_dapm_routes,
				ARRAY_SIZE(brownstone_dapm_routes));

	snd_soc_dapm_disable_pin(dapm, "Ext Left Spk");
	snd_soc_dapm_disable_pin(dapm, "Ext Right Spk");
	snd_soc_dapm_disable_pin(dapm, "Headset Stereophone");
	snd_soc_dapm_disable_pin(dapm, "Headset Mic");
	snd_soc_dapm_disable_pin(dapm, "Main Mic");

	brownstone_headphone_func    = BROWNSTONE_CTRL_OFF;
	brownstone_external_mic_func = BROWNSTONE_CTRL_OFF;
	brownstone_spk_func	     = BROWNSTONE_CTRL_OFF;
	brownstone_internal_mic_func = BROWNSTONE_CTRL_OFF;

	/* set endpoints to not connected */
	snd_soc_dapm_nc_pin(dapm, "HPOUT2P");
	snd_soc_dapm_nc_pin(dapm, "HPOUT2N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1P");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2P");
	snd_soc_dapm_nc_pin(dapm, "IN1LN");
	snd_soc_dapm_nc_pin(dapm, "IN1LP");
	snd_soc_dapm_nc_pin(dapm, "IN1RP");
	snd_soc_dapm_nc_pin(dapm, "IN2LP:VXRN");
	snd_soc_dapm_nc_pin(dapm, "IN2RN");
	snd_soc_dapm_nc_pin(dapm, "IN2RP:VXRP");
	snd_soc_dapm_nc_pin(dapm, "IN2LN");

	snd_soc_dapm_sync(dapm);


	return 0;
}

static int codec_hdmi_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int brownstone_probe(struct snd_soc_card *card)
{
	pr_debug("%s: enter\n", __func__);

	if (audio_clk)
		return 0;

	audio_clk = clk_get(NULL, "mmp-audio");
	if (IS_ERR(audio_clk))
		return PTR_ERR(audio_clk);

	clk_enable(audio_clk);
	return 0;
}

static int brownstone_hdmi_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk;
	int sspa_div;

	pr_debug("%s: enter\n", __func__);

	freq_in = 26000000;
	if (params_rate(params) > 11025) {
		freq_out  = params_rate(params) * 512;
		sysclk    = params_rate(params) * 256;
		sspa_mclk = params_rate(params) * 64;
	} else {
		freq_out  = params_rate(params) * 1024;
		sysclk    = params_rate(params) * 512;
		sspa_mclk = params_rate(params) * 64;
	}
	sspa_div = freq_out;
	do_div(sspa_div, sspa_mclk);

	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_pll(cpu_dai, SSPA_AUDIO_PLL, 0, freq_in, freq_out);
	snd_soc_dai_set_clkdiv(cpu_dai, 0, sspa_div);
	snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, 0);

	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

	return 0;
}

static int brownstone_wm8994_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	/* turn on micbias 1/2 always */
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,
			    WM8994_MICB1_ENA_MASK |
			    WM8994_MICB2_ENA_MASK,
			    WM8994_MICB1_ENA |
			    WM8994_MICB2_ENA);

	return 0;
}

static int brownstone_wm8994_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk;
	int sspa_div;

	printk("%s: enter, rate %d\n", __func__, params_rate(params));

	freq_in = 26000000;
	if (params_rate(params) > 11025) {
		freq_out  = params_rate(params) * 512;
		sysclk    = params_rate(params) * 256;
		sspa_mclk = params_rate(params) * 64;
	} else {
		freq_out  = params_rate(params) * 1024;
		sysclk    = params_rate(params) * 512;
		sspa_mclk = params_rate(params) * 64;
	}
	sspa_div = freq_out;
	do_div(sspa_div, sspa_mclk);

	snd_soc_dai_set_sysclk(cpu_dai, MMP_SSPA_CLK_AUDIO, freq_out, 0);
	snd_soc_dai_set_pll(cpu_dai, MMP_SYSCLK, 0, freq_out, sysclk);
	snd_soc_dai_set_pll(cpu_dai, MMP_SSPA_CLK, 0, freq_out, sspa_mclk);

	/* set wm8994 sysclk */
	snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1, sysclk, 0);

	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops brownstone_ops[] = {
{
	.hw_params = brownstone_hdmi_hw_params,
},
{
	.startup = brownstone_wm8994_startup,
	.hw_params = brownstone_wm8994_hw_params,
},
};

static struct snd_soc_dai_link brownstone_hdmi_dai = {
	.name        = "hdmi",
	.stream_name = "hdmi",
	.cpu_dai_name	= "mmp-sspa-dai.0",
	.codec_dai_name	= "dummy-dai",
	.platform_name	= "mmp-pcm-audio",
	.codec_name	= "dummy-codec",
	.ops		= &brownstone_ops[0],
	.init		= codec_hdmi_init,
};


static struct snd_soc_dai_link brownstone_wm8994_dai[] = {
{
	.name		= "WM8994",
	.stream_name	= "WM8994 HiFi",
	.cpu_dai_name	= "mmp-sspa-dai.0",
	.codec_dai_name	= "wm8994-aif1",
	.platform_name	= "mmp-pcm-audio",
	.codec_name	= "wm8994-codec",
	.ops		= &brownstone_ops[1],
	.init		= brownstone_wm8994_init,
},
};

/* audio machine driver */
static struct snd_soc_card brownstone[] = {
{
	.name         = "brownstone",
	.dai_link     = brownstone_wm8994_dai,
	.num_links    = ARRAY_SIZE(brownstone_wm8994_dai),
	.probe        = brownstone_probe,
},
{	.name         = "hdmi",
	.dai_link     = &brownstone_hdmi_dai,
	.num_links    = 1,
	.probe        = brownstone_probe,
},
};

static int __init brownstone_init(void)
{
	int ret;

	if (!machine_is_brownstone())
		return -ENODEV;

	brownstone_snd_device[0] = platform_device_alloc("soc-audio", 0);
	if (!brownstone_snd_device[0]) {
		ret = -ENOMEM;
		goto err_dev1;
	}

	platform_set_drvdata(brownstone_snd_device[0], &brownstone[0]);
	ret = platform_device_add(brownstone_snd_device[0]);

	if (ret) {
		platform_device_put(brownstone_snd_device[0]);
		goto err_dev1;
	}

	brownstone_snd_device[1] = platform_device_alloc("soc-audio", 1);
	if (!brownstone_snd_device[1]) {
		ret = -ENOMEM;
		goto err_dev1;
	}

	platform_set_drvdata(brownstone_snd_device[1], &brownstone[1]);
	ret = platform_device_add(brownstone_snd_device[1]);

	if (ret) {
		platform_device_put(brownstone_snd_device[1]);
		goto err_dev1;
	}

err_dev1:
	return ret;
}

static void __exit brownstone_exit(void)
{
	platform_device_unregister(brownstone_snd_device[0]);
	platform_device_unregister(brownstone_snd_device[1]);
}

module_init(brownstone_init);
module_exit(brownstone_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC Brownstone");
MODULE_LICENSE("GPL");
