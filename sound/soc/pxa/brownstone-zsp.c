/*
 * linux/sound/soc/pxa/brownstone-zsp.c
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * Author: Libin Yang <lbyang@marvell.com>
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
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/switch_headset.h>
#include <linux/mfd/wm8994/registers.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <plat/ssp.h>
#include <mach/addr-map.h>
#include <mach/regs-sspa.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>
#include <mach/mmp-zsp.h>
#include <linux/wakelock.h>

#include "../codecs/wm8994.h"
#include "mmp-squ-zsp.h"
#include "mmp2-sspa.h"

#define BROWNSTONE_HP_ON		0
#define BROWNSTONE_HP_OFF		1
#define BROWNSTONE_EMIC_ON		0
#define BROWNSTONE_EMIC_OFF		1
#define BROWNSTONE_SPK_ON		0
#define BROWNSTONE_SPK_OFF		1
#define BROWNSTONE_IMIC_ON		0
#define BROWNSTONE_IMIC_OFF		1
#define BROWNSTONE_SAMPLE_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
				SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 | \
				SNDRV_PCM_RATE_32000)

static int brownstone_headphone_func;
static int brownstone_external_mic_func;
static int brownstone_spk_func;
static int brownstone_internal_mic_func;

static void brownstone_ext_control(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (brownstone_headphone_func == BROWNSTONE_HP_ON)
		snd_soc_dapm_enable_pin(dapm, "Headset Stereophone");
	else
		snd_soc_dapm_disable_pin(dapm, "Headset Stereophone");

	if (brownstone_external_mic_func == BROWNSTONE_EMIC_ON)
		snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	else
		snd_soc_dapm_disable_pin(dapm, "Headset Mic");

	if (brownstone_spk_func == BROWNSTONE_SPK_ON) {
		snd_soc_dapm_enable_pin(dapm, "Ext Left Spk");
		snd_soc_dapm_enable_pin(dapm, "Ext Right Spk");
	} else {
		snd_soc_dapm_disable_pin(dapm, "Ext Left Spk");
		snd_soc_dapm_disable_pin(dapm, "Ext Right Spk");
	}

	if (brownstone_internal_mic_func == BROWNSTONE_IMIC_ON)
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

	/*
	 * FIXME: walk around MICBIAS2 in dapm to make sure
	 * MICBIAS2 is enabled and headset interrupt can be
	 * triggered all the time.
	 */
	{"IN1RN", NULL, "Headset Mic"},

	{"DMIC1DAT", NULL, "Main Mic"},
};

static const char * const headphone_function[] = {"On", "Off"};
static const char * const external_mic_function[] = {"On", "Off"};
static const char * const spk_function[]  = {"On", "Off"};
static const char * const internal_mic_function[]  = {"On", "Off"};
static const struct soc_enum brownstone_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, headphone_function),
	SOC_ENUM_SINGLE_EXT(2, external_mic_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, internal_mic_function),
};

static const struct snd_kcontrol_new brownstone_wm8994_controls[] = {
	SOC_DOUBLE("ADC Mux", WM8994_POWER_MANAGEMENT_4, 1, 0, 1, 0),
	SOC_DOUBLE("AIF2ADC", WM8994_POWER_MANAGEMENT_4, 13, 12, 1, 0),
	SOC_SINGLE("AIF2CLK Switch", WM8994_AIF2_CLOCKING_1, 0, 1, 0),
	SOC_SINGLE("WM8994 Clocking 1", WM8994_CLOCKING_1, 2, 1, 0),
	SOC_ENUM_EXT("Headphone Function", brownstone_enum[0],
		     brownstone_get_headphone, brownstone_set_headphone),
	SOC_ENUM_EXT("External Mic Function", brownstone_enum[1],
		     brownstone_get_external_mic, brownstone_set_external_mic),
	SOC_ENUM_EXT("Speaker Function", brownstone_enum[2],
		     brownstone_get_spk, brownstone_set_spk),
	SOC_ENUM_EXT("Internal Mic Function", brownstone_enum[3],
		     brownstone_get_internal_mic, brownstone_set_internal_mic),
};

#ifdef CONFIG_SWITCH_WM8994_HEADSET
static struct snd_soc_codec *brownstone_zsp_wm8994_codec;

int wm8994_headset_detect(void)
{
	struct snd_soc_codec *codec;
	int status1, status2, reg;
	int ret = 0;

	codec = brownstone_zsp_wm8994_codec;
	if (codec == NULL)
		return 0;

	status1 = snd_soc_read(codec, WM8994_INTERRUPT_STATUS_1);
	status2 = snd_soc_read(codec, WM8994_INTERRUPT_STATUS_2);

	reg = snd_soc_read(codec, WM8994_INTERRUPT_RAW_STATUS_2);
	switch (reg & (WM8994_MIC2_SHRT_STS | WM8994_MIC2_DET_STS)) {
	case WM8994_MIC2_DET_STS:
		ret = 1;
		break;
	case (WM8994_MIC2_SHRT_STS | WM8994_MIC2_DET_STS):
		ret = 2;
		break;
	default:
		break;
	}

	/* clear all irqs */
	snd_soc_write(codec, WM8994_INTERRUPT_STATUS_1, status1);
	snd_soc_write(codec, WM8994_INTERRUPT_STATUS_2, status2);

	/* reset clocking to make sure trigger irq */
	snd_soc_update_bits(codec, WM8994_CLOCKING_2,
			WM8994_DBCLK_DIV_MASK |
			WM8994_TOCLK_DIV_MASK,
			(2 << WM8994_DBCLK_DIV_SHIFT) |
			(4 << WM8994_TOCLK_DIV_SHIFT));

	snd_soc_update_bits(codec, WM8994_CLOCKING_1,
			WM8994_TOCLK_ENA_MASK,
			WM8994_TOCLK_ENA);
	return ret;
}
#endif

static int codec_hdmi_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int codec_wm8994_init(struct snd_soc_pcm_runtime *rtd)
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

	brownstone_headphone_func = BROWNSTONE_HP_OFF;
	brownstone_external_mic_func = BROWNSTONE_EMIC_OFF;
	brownstone_spk_func  = BROWNSTONE_SPK_OFF;
	brownstone_internal_mic_func  = BROWNSTONE_IMIC_OFF;

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
	snd_soc_dapm_nc_pin(dapm, "IN2LN");
	snd_soc_dapm_nc_pin(dapm, "DMIC2DAT");
	snd_soc_dapm_nc_pin(dapm, "IN2RP:VXRP");

	snd_soc_dapm_sync(dapm);


#ifdef CONFIG_SWITCH_WM8994_HEADSET

	brownstone_zsp_wm8994_codec = codec;
	headset_detect_func = wm8994_headset_detect;

	snd_soc_update_bits(codec, WM8994_CLOCKING_2,
			    WM8994_DBCLK_DIV_MASK |
			    WM8994_TOCLK_DIV_MASK,
			    (2 << WM8994_DBCLK_DIV_SHIFT) |
			    (4 << WM8994_TOCLK_DIV_SHIFT));

	snd_soc_update_bits(codec, WM8994_CLOCKING_1,
			    WM8994_TOCLK_ENA_MASK,
			    WM8994_TOCLK_ENA);

	snd_soc_update_bits(codec, WM8994_IRQ_DEBOUNCE,
			    WM8994_MIC2_DET_DB_MASK |
			    WM8994_MIC2_SHRT_DB_MASK,
			    WM8994_MIC2_DET_DB |
			    WM8994_MIC2_SHRT_DB);

	snd_soc_update_bits(codec, WM8994_MICBIAS,
			    WM8994_MICD_ENA_MASK |
			    WM8994_MICD_SCTHR_MASK,
			    WM8994_MICD_ENA |
			    (1 << WM8994_MICD_SCTHR_SHIFT));

	/* FIXME: turn on micbias 1/2 all the time */
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,
			WM8994_MICB1_ENA_MASK |
			WM8994_MICB2_ENA_MASK,
			WM8994_MICB1_ENA |
			WM8994_MICB2_ENA);

	/* unmask mic2 shrt and det int */
	snd_soc_update_bits(codec, WM8994_INTERRUPT_STATUS_2_MASK,
			    WM8994_IM_MIC2_SHRT_EINT_MASK |
			    WM8994_IM_MIC2_DET_EINT_MASK, 0);

	/* unmask int */
	snd_soc_update_bits(codec, WM8994_INTERRUPT_CONTROL,
			    WM8994_IM_IRQ_MASK, 0);

#endif
	return 0;
}

static struct platform_device *brownstone_zsp_snd_device[3];

static int brownstone_zsp_probe(struct snd_soc_card *card)
{
	/* CPU PMU audio clock, enable SRAM bank 3 */
	__raw_writel(0x0600, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0610, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0710, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);
	__raw_writel(0x0712, APMU_AUDIO_CLK_RES_CTRL);
	udelay(10);

	/* temporarily setting the SSPA registers */
	/* select audio pll : 34M*/
	__raw_writel(0x10800, SSPA_AUD_PLL_CTRL1);
	__raw_writel(0x10800, SSPA_AUD_PLL_CTRL1);
	/* div : 12, 44.1K*/
	__raw_writel(0x00001105, SSPA_AUD_CTRL);
	__raw_writel(0x00001105, SSPA_AUD_CTRL);

	return 0;
}

static int brownstone_zsp_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ssp_device *sspa = snd_soc_dai_get_drvdata(cpu_dai);
	struct pxa910_runtime_data *prtd = runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->render_id = sspa->render_type[0];
		prtd->stream_id = (sspa->render_type[0] == 0x01 ? \
				AUDIO_STREAM_ID_SSPA1_PLAYBACK : \
				AUDIO_STREAM_ID_SSPA2_PLAYBACK);
	} else {
		prtd->render_id = sspa->render_type[1];
		prtd->stream_id = (sspa->render_type[1] == 0x09 ? \
				AUDIO_STREAM_ID_SSPA1_CAPTURE : \
				AUDIO_STREAM_ID_SSPA2_CAPTURE);
	}

	cpu_dai->driver->playback.channels_min = 2;
	cpu_dai->driver->playback.channels_max = 2;

	if (cpu_dai->active == 0) {
		cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
		cpu_dai->driver->playback.rates = BROWNSTONE_SAMPLE_RATES;
	}

	return 0;
}
static int brownstone_zsp_wm8994_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ssp_device *sspa = snd_soc_dai_get_drvdata(cpu_dai);
	struct pxa910_runtime_data *prtd = runtime->private_data;
	int format;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->render_id = sspa->render_type[0];
		prtd->stream_id = (sspa->render_type[0] == 0x01 ? \
				AUDIO_STREAM_ID_SSPA1_PLAYBACK : \
				AUDIO_STREAM_ID_SSPA2_PLAYBACK);
	} else {
		prtd->render_id = sspa->render_type[1];
		prtd->stream_id = (sspa->render_type[1] == 0x09 ? \
				AUDIO_STREAM_ID_SSPA1_CAPTURE : \
				AUDIO_STREAM_ID_SSPA2_CAPTURE);
	}

	cpu_dai->driver->playback.channels_min = 2;
	cpu_dai->driver->playback.channels_max = 2;
	cpu_dai->driver->capture.channels_min = 2;
	cpu_dai->driver->capture.channels_max = 2;


	if (cpu_dai->active == 0) {
		cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
		cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
		cpu_dai->driver->playback.rates = BROWNSTONE_SAMPLE_RATES;
		cpu_dai->driver->capture.rates = BROWNSTONE_SAMPLE_RATES;
	}

	format = SND_SOC_DAIFMT_I2S | \
		SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	codec_dai->driver->ops->set_fmt(codec_dai, format);
	codec_dai->driver->ops->set_sysclk(codec_dai,
				WM8994_SYSCLK_MCLK1, 11289600 , 0);

	/* turn on micbias 1/2 always */
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,
			    WM8994_MICB1_ENA_MASK |
			    WM8994_MICB2_ENA_MASK,
			    WM8994_MICB1_ENA |
			    WM8994_MICB2_ENA);

	return 0;
}

static int brownstone_zsp_hifi_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int brownstone_zsp_wm8994_prepare(struct snd_pcm_substream *substream)
{

	return 0;
}

static int brownstone_zsp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_device *sspa = snd_soc_dai_get_drvdata(cpu_dai);

	/* limit the SSPA sample rate based on the HW requirement */
	/* maybe it is better to put the code in the file mmp2-sspa.c */
	cpu_dai->driver->playback.rates = \
		snd_pcm_rate_to_rate_bit(params_rate(params));
	cpu_dai->driver->capture.rates = \
		snd_pcm_rate_to_rate_bit(params_rate(params));

	if (strcmp(codec_dai->name, "wm8994-aif1") == 0 ||
		strcmp(codec_dai->name, "wm8994-aif2") == 0) {
		int retclk;
		struct mmp_zsp_clkcfg zspclkcfg;

		if ((params_rate(params) % 8000) == 0)
			zspclkcfg.asclk = MMP_ZSP_ASCLK_24576000;
		else
			zspclkcfg.asclk = MMP_ZSP_ASCLK_22579200;

		/* get current clock settings */
		retclk = zsp_set_clock_preference(MMP_ZSP_ASCLK_FLAGS,
				&zspclkcfg);
		if (retclk != 0) {
			/* failed, cannot change now */
			return -EINVAL;
		}

		/* set ok or compatible */
		if (zspclkcfg.asclk == MMP_ZSP_ASCLK_22579200)
			sspa->zsp_sspa_conf.mclk = 11289600;
		else
			sspa->zsp_sspa_conf.mclk = 12288000;
		codec_dai->driver->ops->set_sysclk(codec_dai,
			WM8994_SYSCLK_MCLK1,
			sspa->zsp_sspa_conf.mclk, 0);
	} else {
		/* HDMI do not need to set the mclk */
		sspa->zsp_sspa_conf.mclk = 0;
	}

	return 0;
}

static void brownstone_zsp_hifi_shutdown(struct snd_pcm_substream *substream)
{
	return;
}

static void brownstone_zsp_wm8994_shutdown(struct snd_pcm_substream *substream)
{
	return;
}

/* machine stream operations */
static struct snd_soc_ops brownstone_zsp_machine_ops[] = {
{
	.startup = brownstone_zsp_hifi_startup,
	.prepare = brownstone_zsp_hifi_prepare,
	.hw_params = brownstone_zsp_hw_params,
	.shutdown = brownstone_zsp_hifi_shutdown,
},
{
	.startup = brownstone_zsp_wm8994_startup,
	.prepare = brownstone_zsp_wm8994_prepare,
	.hw_params = brownstone_zsp_hw_params,
	.shutdown = brownstone_zsp_wm8994_shutdown,
},
};

static struct snd_soc_dai_link brownstone_zsp_hdmi_dai[] = {
{
	.name = "HDMI HiFi",
	.stream_name = "HDMI HiFi",
	.cpu_dai_name = "mmp-zsp-sspa-dai.0",
	.codec_dai_name = "wm8994-aif1",
	.platform_name = "mmp-zsp-pcm-audio",
	.codec_name = "wm8994-codec",
	.ops = &brownstone_zsp_machine_ops[0],
	.init = codec_hdmi_init,
}
};

static struct snd_soc_dai_link brownstone_zsp_wm8994_dai[] = {
{
	.name = "WM8994 Voice",
	.stream_name = "Wm8994 Voice",
	.cpu_dai_name = "mmp-sspa-dai.0",
	.codec_dai_name = "wm8994-aif1",
	.platform_name = "mmp-zsp-pcm-audio",
	.codec_name = "wm8994-codec",
	.ops = &brownstone_zsp_machine_ops[1],
	.init = codec_wm8994_init,
},
{
	.name = "WM8994 Voice 2",
	.stream_name = "WM8994 Voice 2",
	.cpu_dai_name = "mmp-sspa-dai.1",
	.codec_dai_name = "wm8994-aif2",
	.platform_name = "mmp-zsp-pcm-audio",
	.codec_name = "wm8994-codec",
	.ops = &brownstone_zsp_machine_ops[1],
}
};

/* audio machine driver */
static struct snd_soc_card snd_soc_machine_brownstone_zsp[] = {
{
	.name = "brownstone hdmi",
	.dai_link = brownstone_zsp_hdmi_dai,
	.num_links = ARRAY_SIZE(brownstone_zsp_hdmi_dai),
	.probe = brownstone_zsp_probe,
},
{
	.name = "brownstone wm8994",
	.dai_link = brownstone_zsp_wm8994_dai,
	.num_links = ARRAY_SIZE(brownstone_zsp_wm8994_dai),
	.probe = brownstone_zsp_probe,
}
};

static int __init brownstone_zsp_init(void)
{
	int ret[3];

	if (!machine_is_brownstone())
		return -ENODEV;

	brownstone_zsp_snd_device[1] = platform_device_alloc("soc-audio", 1);

	if (!brownstone_zsp_snd_device[1])
		return -ENOMEM;
	platform_set_drvdata(brownstone_zsp_snd_device[1],
			     &snd_soc_machine_brownstone_zsp[1]);
	ret[1] = platform_device_add(brownstone_zsp_snd_device[1]);

	if (ret[1])
		platform_device_put(brownstone_zsp_snd_device[1]);

	brownstone_zsp_snd_device[0] = platform_device_alloc("soc-audio", 0);
	if (!brownstone_zsp_snd_device[0])
		return -ENOMEM;
	platform_set_drvdata(brownstone_zsp_snd_device[0],
			     &snd_soc_machine_brownstone_zsp[0]);
	ret[0] = platform_device_add(brownstone_zsp_snd_device[0]);

	if (ret[0])
		platform_device_put(brownstone_zsp_snd_device[0]);

	return ret[0] | ret[1];
}

static void __exit brownstone_zsp_exit(void)
{
	platform_device_unregister(brownstone_zsp_snd_device[0]);
	platform_device_unregister(brownstone_zsp_snd_device[1]);
}

module_init(brownstone_zsp_init);
module_exit(brownstone_zsp_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC Brownstone ZSP");
MODULE_LICENSE("GPL");
