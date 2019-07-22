/*
 * linux/sound/soc/pxa/qseven.c
 *
 * Copyright (C) 2009 Marvell International Ltd.
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
#include <linux/i2c.h>

#include <asm/mach-types.h>
#include <linux/io.h>
#include <linux/switch.h>

#include <linux/uaccess.h>
#include <plat/ssp.h>
#include <mach/addr-map.h>
#include <mach/regs-sspa.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>

#include <linux/mfd/wm8994/registers.h>
#include "../codecs/wm8994.h"
#include "../codecs/wm8731.h"
#include "mmp2-squ.h"
#include "mmp2-sspa.h"
#include <linux/delay.h>

#define MMP3ASOC_SAMPLE_RATES SNDRV_PCM_RATE_44100

#define MMP3ASOC_HEADPHONE_FUNC		0
#define MMP3ASOC_HS_MIC_FUNC		1
#define MMP3ASOC_SPK_FUNC		2
#define MMP3ASOC_MAIN_MIC_FUNC		3
#define MMP3ASOC_JACK_FUNC		4

#define MMP3ASOC_CTRL_ON	0
#define MMP3ASOC_CTRL_OFF	1

static int mmp3asoc_headphone_func;
static int mmp3asoc_hs_mic_func;
static int mmp3asoc_spk_func;
static int mmp3asoc_main_mic_func;
static int mmp3_qseven_jack;

static void mmp3asoc_ext_control(struct snd_soc_dapm_context *dapm, int func)
{
	switch (func) {
	case MMP3ASOC_HEADPHONE_FUNC:
		if (mmp3asoc_headphone_func == MMP3ASOC_CTRL_ON)
			snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
		else
			snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
		break;
	case MMP3ASOC_HS_MIC_FUNC:
		if (mmp3asoc_hs_mic_func == MMP3ASOC_CTRL_ON)
			snd_soc_dapm_enable_pin(dapm, "Headset Jack");
		else
			snd_soc_dapm_disable_pin(dapm, "Headset Jack");
		break;
	default:
		pr_err("wrong func type\n");
		return;
	}

	snd_soc_dapm_sync(dapm);
	return;
}

static int qseven_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mmp3_qseven_jack;
	return 0;
}

static int qseven_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (mmp3_qseven_jack == ucontrol->value.integer.value[0])
		return 0;

	mutex_lock(&codec->mutex);
	mmp3_qseven_jack = ucontrol->value.integer.value[0];

	if (mmp3_qseven_jack == 0) {
		mmp3asoc_headphone_func =  MMP3ASOC_CTRL_OFF;
		mmp3asoc_hs_mic_func = MMP3ASOC_CTRL_OFF;
		mmp3asoc_ext_control(dapm, MMP3ASOC_HEADPHONE_FUNC);
		mmp3asoc_ext_control(dapm, MMP3ASOC_HS_MIC_FUNC);
	} else {
		mmp3asoc_headphone_func =  MMP3ASOC_CTRL_ON;
		mmp3asoc_hs_mic_func = MMP3ASOC_CTRL_ON;
		mmp3asoc_ext_control(dapm, MMP3ASOC_HEADPHONE_FUNC);
		mmp3asoc_ext_control(dapm, MMP3ASOC_HS_MIC_FUNC);
	}
	mutex_unlock(&codec->mutex);
	return 1;
}

static int mmp3asoc_get_headphone(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mmp3asoc_headphone_func;
	return 0;
}

static int mmp3asoc_set_headphone(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (mmp3asoc_headphone_func == ucontrol->value.integer.value[0])
		return 0;

	mutex_lock(&codec->mutex);
	mmp3asoc_headphone_func = ucontrol->value.integer.value[0];
	mmp3asoc_ext_control(dapm, MMP3ASOC_HEADPHONE_FUNC);
	mutex_unlock(&codec->mutex);
	return 1;
}

static int mmp3asoc_get_hs_mic(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mmp3asoc_hs_mic_func;
	return 0;
}

static int mmp3asoc_set_hs_mic(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (mmp3asoc_hs_mic_func == ucontrol->value.integer.value[0])
		return 0;

	mutex_lock(&codec->mutex);
	mmp3asoc_hs_mic_func = ucontrol->value.integer.value[0];
	mmp3asoc_ext_control(dapm, MMP3ASOC_HS_MIC_FUNC);
	mutex_unlock(&codec->mutex);
	return 1;
}

static int mmp3asoc_get_spk(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mmp3asoc_spk_func;
	return 0;
}

static int mmp3asoc_set_spk(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (mmp3asoc_spk_func == ucontrol->value.integer.value[0])
		return 0;

	mutex_lock(&codec->mutex);
	mmp3asoc_spk_func = ucontrol->value.integer.value[0];
	mmp3asoc_ext_control(dapm, MMP3ASOC_SPK_FUNC);
	mutex_unlock(&codec->mutex);
	return 1;
}

static int mmp3asoc_get_main_mic(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mmp3asoc_main_mic_func;
	return 0;
}

static int mmp3asoc_set_main_mic(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (mmp3asoc_main_mic_func == ucontrol->value.integer.value[0])
		return 0;

	mutex_lock(&codec->mutex);
	mmp3asoc_main_mic_func = ucontrol->value.integer.value[0];
	mmp3asoc_ext_control(dapm, MMP3ASOC_MAIN_MIC_FUNC);
	mutex_unlock(&codec->mutex);
	return 1;
}

static const struct snd_soc_dapm_widget mmp3asoc_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Left Spk", NULL),
	SND_SOC_DAPM_SPK("Ext Right Spk", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
};

static const struct snd_soc_dapm_route mmp3asoc_dapm_routes[] = {
	{"Ext Left Spk", NULL, "SPKOUTLP"},
	{"Ext Left Spk", NULL, "SPKOUTLN"},

	{"Ext Right Spk", NULL, "SPKOUTRP"},
	{"Ext Right Spk", NULL, "SPKOUTRN"},

	{"Headset Stereophone", NULL, "HPOUT1L"},
	{"Headset Stereophone", NULL, "HPOUT1R"},

	{"IN1RN", NULL, "MICBIAS2"},
	{"MICBIAS2", NULL, "Headset Mic"},

	{"IN1LP", NULL, "MICBIAS1"},
	{"IN1LN", NULL, "MICBIAS1"},
	{"MICBIAS1", NULL, "Main Mic"},
};

static const char *headphone_function[] = {"On", "Off"};
static const char *hs_mic_function[] = {"On", "Off"};
static const char *spk_function[] = {"On", "Off"};
static const char *main_mic_function[] = {"On", "Off"};

static const struct soc_enum mmp3asoc_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, headphone_function),
	SOC_ENUM_SINGLE_EXT(2, hs_mic_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, main_mic_function),
};


static const struct snd_kcontrol_new mmp3asoc_wm8994_controls[] = {
	SOC_ENUM_EXT("Headphone Function", mmp3asoc_enum[0],
		     mmp3asoc_get_headphone, mmp3asoc_set_headphone),
	SOC_ENUM_EXT("Headset Mic Function", mmp3asoc_enum[1],
		     mmp3asoc_get_hs_mic, mmp3asoc_set_hs_mic),
	SOC_ENUM_EXT("Speaker Function", mmp3asoc_enum[2],
		     mmp3asoc_get_spk, mmp3asoc_set_spk),
	SOC_ENUM_EXT("Main Mic Function", mmp3asoc_enum[3],
		     mmp3asoc_get_main_mic, mmp3asoc_set_main_mic),
};

/* qseven machine dapm widgets */
static const struct snd_soc_dapm_widget wm8731_qseven_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_HP("Headset Jack", NULL),
};

/* qseven machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route qseven_audio_map[] = {

	/* headset Jack  - in = micin, out = LHPOUT*/
	{"Headset Jack", NULL, "LHPOUT"},
	{"Headset Jack", NULL, "RHPOUT"},

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},
	{"MICIN", NULL, "Headphone Jack"},

	/* speaker connected to LOUT, ROUT
	{"Ext Spk", NULL, "ROUT"},
	{"Ext Spk", NULL, "LOUT"},

	mic is connected to MICIN (via right channel of headphone jack)
	{"MICIN", NULL, "Mic Jack"},

	Same as the above but no mic bias for line signals
	{"MICIN", NULL, "Line Jack"},*/
};

static const char *jack_function[] = {"Headphone", "Mic", "Line", "Headset",
	"Off"};
static const struct soc_enum qseven_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, jack_function),
};

static const struct snd_kcontrol_new wm8731_qseven_controls[] = {
	SOC_ENUM_EXT("Jack Function", qseven_enum[0], qseven_get_jack,
		qseven_set_jack),
};

static int codec_hdmi_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static struct platform_device *mmp3asoc_snd_device[2];

#define __raw_modify(addr, toclear, toset)		\
	do {						\
		volatile unsigned int tval;		\
		tval = __raw_readl(addr);		\
		tval &= ~toclear;			\
		tval |= toset;				\
		__raw_writel(tval, (addr));		\
		tval = __raw_readl(addr);		\
		udelay(100);				\
	} while (0)

static void audio_subsystem_poweron(void)
{
	printk(KERN_INFO " audio subsystem power on (A stepping) 0x%08x\n",
	       __raw_readl(APMU_REG(0x220)));

	/* enable power switch 01 */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0, (1u << 9));
	/* enable power switch 11 */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0, (2u << 9));
	/* enable SRAM power */
	__raw_modify(APMU_AUDIO_SRAM_PWR, 0, 0x5);
	__raw_modify(APMU_AUDIO_SRAM_PWR, 0, 0xa);
	__raw_modify(APMU_AUDIO_SRAM_PWR, 0, 0xc0);
	/* audio island */
	__raw_modify(APMU_ISLD_DSPA_CTRL, 0x7, 0);
	__raw_modify(APMU_ISLD_DSPA_CTRL, 0, (1u << 4));
	__raw_modify(APMU_ISLD_DSPA_CTRL, (1u << 4), 0);
	/* audio DSA */
	__raw_modify(APMU_AUDIO_DSA, 0xf, 0xa);
	__raw_modify(APMU_AUDIO_DSA, 0xf, 0xf);
	/* SSPA1 BIT/SYSCLK */
	__raw_writel(0xd3ee2276, MPMU_ISCCRX0);
	__raw_writel(0xd0040040, MPMU_ISCCRX1);
	/* disable isolation */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0, (1u << 8));
	/* enable peripheral */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0, (1u << 4));
	/* pull peripheral out of reset */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0, (1u << 1));

	/* Audio CFG: DSP core will stall after release from reset */
	__raw_modify(DSP_AUDIO_CONFIG_REG, (1u << 1), 0);
	/* DSP core clock : enable clock divier  */
	__raw_modify(DSA_CORE_CLK_RES_CTRL, 0, (1u << 3));
	/* Release the core reset */
	__raw_modify(DSA_CORE_CLK_RES_CTRL, 0, (1u << 0));
	/* Release the AXI reset */
	__raw_modify(DSA_CORE_CLK_RES_CTRL, 0, (1u << 1));

	/* devices */
	udelay(100);
	__raw_writel(0x8, DSA_SSP_CLK_RES_CTRL);
	__raw_writel(0x9, DSA_SSP_CLK_RES_CTRL);
	udelay(100);
	__raw_writel(0x8, DSA_ABU_CLK_RES_CTRL);
	__raw_writel(0x9, DSA_ABU_CLK_RES_CTRL);
	udelay(100);

	__raw_modify(MPMU_CCGR, 0, 0x20);

	udelay(1000);
}

static void audio_subsystem_poweroff(void)
{
	/* enable isolation */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0x100, 0);
	/* assert AXI and peripheral reset */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0x3, 0);
	/* gate axi and peripheral clock */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0x18, 0);
	/* power off */
	__raw_modify(APMU_AUDIO_CLK_RES_CTRL, 0x600, 0);
}

static void audio_subsystem_pll_config(void)
{
	/* select audio pll: Fvco = 135.4752MHz; OCLK = 11.2896MHz;
	 * DIV_MCLK[1:0] = 0b10;
	 * DIV_MODULO[2:0] = 0b010;
	 * FRACT[27:8] = 0x8a18;
	 * DIV_FBCCLK[1:0] = 0b00; */
	__raw_writel(0x908a1899, SSPA_AUD_PLL_CTRL0);
	msleep(100);

	/* sspa_aud_pll_ctrl1[11] = 1 to choose audio PLL
	 * div : 12, 44.1K
	 * DIV_OCLK_PATTERN[1:0] = 0b01; */
	__raw_writel(0x2e010801, SSPA_AUD_PLL_CTRL1);
	msleep(100);

	/* audio clock select: choose separate clock source for sspa1 and sspa2.
	 * sspa_aud_ctrl[7] = 0, sspa1 chooses pm_vctcxo;
	 * sspa_aud_ctrl[23] = 0, sspa2 chooses i2s_sc_apb; */
	__raw_writel(0x111109, SSPA_AUD_CTRL);
	msleep(100);

	/* Switch the source clock for core and AXI clock to Audio PLL*/
	__raw_modify(DSA_CORE_CLK_RES_CTRL, 0x0, (1u << 2));
	__raw_modify(DSA_CORE_CLK_RES_CTRL, 0x0, (3u << 4));
	__raw_modify(DSA_CORE_CLK_RES_CTRL, (1u << 2), 0x0);

}

static int codec_wm8731_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int err;

	audio_subsystem_poweron();
	/* currently the audio pll of mmp3 a0 stepping is not working */
	audio_subsystem_pll_config();

	snd_soc_dapm_nc_pin(dapm, "LLINEIN");
	snd_soc_dapm_nc_pin(dapm, "RLINEIN");

	/* Add mmp3asoc specific controls */
	err = snd_soc_add_controls(codec, wm8731_qseven_controls,
				   ARRAY_SIZE(wm8731_qseven_controls));
	if (err < 0)
		return err;

	/* add mmp3asoc specific widgets */
	snd_soc_dapm_new_controls(dapm, wm8731_qseven_dapm_widgets,
				  ARRAY_SIZE(wm8731_qseven_dapm_widgets));

	/* set up mmp3asoc specific audio routes */
	snd_soc_dapm_add_routes(dapm, qseven_audio_map,
				ARRAY_SIZE(qseven_audio_map));

	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Headset Jack");

	/* init: disable HEADSET, enable SPK */
	mutex_lock(&codec->mutex);
	mmp3asoc_headphone_func = MMP3ASOC_CTRL_ON;
	mmp3asoc_hs_mic_func = MMP3ASOC_CTRL_ON;
	mmp3_qseven_jack = MMP3ASOC_CTRL_ON;
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);

	return 0;
}

static int mmp3asoc_probe(struct snd_soc_card *card)
{
	return 0;
}

static int mmp3asoc_hdmi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	cpu_dai->driver->playback.channels_min = 2;
	cpu_dai->driver->playback.channels_max = 2;

	cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->playback.rates = MMP3ASOC_SAMPLE_RATES;

	return 0;
}

static int mmp3asoc_hdmi_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;

	pr_debug("%s: enter, rate %d\n", __func__, params_rate(params));

	freq_in = 26000000;
	if (params_rate(params) > 11025) {
		freq_out = params_rate(params) * 512;
		sysclk = params_rate(params) * 256;
		sspa_mclk = params_rate(params) * 64;
	} else {
		freq_out = params_rate(params) * 1024;
		sysclk = params_rate(params) * 512;
		sspa_mclk = params_rate(params) * 64;
	}
	sspa_div = freq_out;
	do_div(sspa_div, sspa_mclk);

#ifdef CONFIG_SND_WM8994_MASTER_MODE
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#else
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif

	/* workaround for audio PLL, and should be removed after A1 */
	switch (params_rate(params)) {
	case 48000:
		__raw_writel(0xd0040040, MPMU_ISCCRX1);
		break;
	case 44100:
		__raw_writel(0xd0040044, MPMU_ISCCRX1);
		break;
	case 32000:
		__raw_writel(0xd00800c0, MPMU_ISCCRX1);
		break;
	case 24000:
		__raw_writel(0xd0020040, MPMU_ISCCRX1);
		break;
	case 22050:
		__raw_writel(0xd0020044, MPMU_ISCCRX1);
		break;
	case 16000:
		__raw_writel(0xd00400c0, MPMU_ISCCRX1);
		break;
	case 8000:
		__raw_writel(0xd00400c0, MPMU_ISCCRX1);
		break;
	default:
		break;
	}

	snd_soc_dai_set_pll(cpu_dai, SSPA_AUDIO_PLL, 0, freq_in, freq_out);
	snd_soc_dai_set_clkdiv(cpu_dai, 0, sspa_div);
	snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, 0);

	return 0;
}
static int mmp3asoc_wm8731_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->playback.rates = MMP3ASOC_SAMPLE_RATES;
	cpu_dai->driver->capture.rates = MMP3ASOC_SAMPLE_RATES;

	mmp3asoc_headphone_func =  MMP3ASOC_CTRL_ON;
	mmp3asoc_hs_mic_func = MMP3ASOC_CTRL_ON;
	mmp3asoc_ext_control(dapm, MMP3ASOC_HEADPHONE_FUNC);
	mmp3asoc_ext_control(dapm, MMP3ASOC_HS_MIC_FUNC);
	/* turn on micbias 1/2 always */
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,
			    WM8994_MICB1_ENA_MASK |
			    WM8994_MICB2_ENA_MASK,
			    WM8994_MICB1_ENA |
			    WM8994_MICB2_ENA);

	return 0;
}

static int mmp3asoc_wm8731_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;

	pr_info("%s: enter, rate %d\n", __func__, params_rate(params));
	freq_in = 26000000;
	if (params_rate(params) > 11025) {
		freq_out = params_rate(params) * 512;
		sysclk = params_rate(params) * 256;
		sspa_mclk = params_rate(params) * 64;
	} else {
		freq_out = params_rate(params) * 1024;
		sysclk = params_rate(params) * 512;
		sspa_mclk = params_rate(params) * 64;
	}
	sspa_div = freq_out;
	do_div(sspa_div, sspa_mclk);

#ifdef CONFIG_SND_WM8731_MASTER_MODE
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#else
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif

	/* workaround for audio PLL, and should be removed after A1 */
	/* SSPA2 clock formula: sysclk = (PLL1/4) * ISCCR1 Nom/Denom4
	 * for 48k, the sysclk should be 12.2880MHz, but here we only get
	 * approximate 12.458MHz, only for test purpose */
	switch (params_rate(params)) {
	case 48000:
		__raw_writel(0xd0040040, MPMU_ISCCRX1);
		break;
	case 44100:
		__raw_writel(0xd0040044, MPMU_ISCCRX1);
		break;
	case 32000:
		__raw_writel(0xd00800c0, MPMU_ISCCRX1);
		break;
	case 24000:
		__raw_writel(0xd0020040, MPMU_ISCCRX1);
		break;
	case 22050:
		__raw_writel(0xd0020044, MPMU_ISCCRX1);
		break;
	case 16000:
		__raw_writel(0xd00400c0, MPMU_ISCCRX1);
		break;
	case 8000:
		__raw_writel(0xd00400c0, MPMU_ISCCRX1);
		break;
	default:
		break;
	}

	/* set the codec system clock for DAC and ADC */
	snd_soc_dai_set_pll(cpu_dai, SSPA_AUDIO_PLL, 0, freq_in, freq_out);
	snd_soc_dai_set_clkdiv(cpu_dai, 0, sspa_div);
	snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, 0);

	/* set wm8731 sysclk */
	snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK_XTAL, sysclk,
				SND_SOC_CLOCK_IN);

	return 0;
}

#ifdef CONFIG_PM
static int mmp3asoc_suspend_post(struct snd_soc_card *card)
{

	return 0;
}

static int mmp3asoc_resume_pre(struct snd_soc_card *card)
{

	return 0;
}

#endif

/* machine stream operations */
static struct snd_soc_ops mmp3asoc_machine_ops[] = {
	{
	 .startup = mmp3asoc_hdmi_startup,
	 .hw_params = mmp3asoc_hdmi_hw_params,
	 },
	{
	 .startup = mmp3asoc_wm8731_startup,
	 .hw_params = mmp3asoc_wm8731_hw_params,
	 },
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mmp3_asoc_wm8731_dai[] = {
	{
	 .name = "WM8731",
	 .stream_name = "WM8731",
	 .codec_name = "wm8731.2-001a",
	 .platform_name = "mmp3-pcm-audio",
	 .cpu_dai_name = "mmp3-sspa-dai.0",
	 .codec_dai_name = "wm8731-hifi",
	 .ops = &mmp3asoc_machine_ops[1],
	 .init = codec_wm8731_init,
	 },
};

static struct snd_soc_dai_link mmp3_asoc_hdmi_dai[] = {
	{
	 .name = "HDMI",
	 .stream_name = "hdmi Audio",
	 .codec_name = "dummy-codec",
	 .platform_name = "mmp3-pcm-audio",
	 .cpu_dai_name = "mmp3-sspa-dai.0",
	 .codec_dai_name = "dummy-dai",
	 .ops = &mmp3asoc_machine_ops[0],
	 .init = codec_hdmi_init,
	 },
};

/* audio machine driver */
static struct snd_soc_card snd_soc_mmp3asoc[] = {
	{
	 .name = "mmp3 asoc",
	 .dai_link = &mmp3_asoc_wm8731_dai[0],
	 .num_links = 1,
	 .probe = mmp3asoc_probe,
#ifdef CONFIG_PM
	 .suspend_post = mmp3asoc_suspend_post,
	 .resume_pre = mmp3asoc_resume_pre,
#endif
	 },
	{
	 .name = "mmp3 hdmi",
	 .dai_link = &mmp3_asoc_hdmi_dai[0],
	 .num_links = 1,
	 .probe = mmp3asoc_probe,
	 },
};

static int __init mmp3asoc_init(void)
{
	int i, ret[2];

	if (!machine_is_mmp3_qseven())
		return -ENODEV;

	for (i = 0; i < 2; i++) {
		mmp3asoc_snd_device[i] = platform_device_alloc("soc-audio", i);
		if (!mmp3asoc_snd_device[i])
			return -ENOMEM;
		platform_set_drvdata(mmp3asoc_snd_device[i], &snd_soc_mmp3asoc[i]);
		ret[i] = platform_device_add(mmp3asoc_snd_device[i]);

		if (ret[i])
			platform_device_put(mmp3asoc_snd_device[i]);
	}

	return ret[1];
}

static void __exit mmp3asoc_exit(void)
{
	int i;
	for (i = 0; i < 2; i++)
		platform_device_unregister(mmp3asoc_snd_device[i]);
}

module_init(mmp3asoc_init);
module_exit(mmp3asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC WM8994 MMP3");
MODULE_LICENSE("GPL");
