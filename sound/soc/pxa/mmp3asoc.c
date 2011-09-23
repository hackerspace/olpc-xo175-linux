/*
 * linux/sound/soc/pxa/mmp3asoc.c
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
#include "mmp2-squ.h"
#include "mmp2-sspa.h"
#include <linux/delay.h>

#define MMP3ASOC_SAMPLE_RATES SNDRV_PCM_RATE_8000_48000

#define MMP3ASOC_HP        0
#define MMP3ASOC_MIC       1
#define MMP3ASOC_HEADSET   2
#define MMP3ASOC_HP_OFF    3
#define MMP3ASOC_SPK_ON    0
#define MMP3ASOC_SPK_OFF   1

static int mmp3asoc_jack_func;
static int mmp3asoc_spk_func;

static void mmp3asoc_ext_control(struct snd_soc_dapm_context *dapm)
{
	if (mmp3asoc_spk_func == MMP3ASOC_SPK_ON) {
		snd_soc_dapm_enable_pin(dapm, "Ext Left Spk");
		snd_soc_dapm_enable_pin(dapm, "Ext Right Spk");
	} else {
		snd_soc_dapm_disable_pin(dapm, "Ext Left Spk");
		snd_soc_dapm_disable_pin(dapm, "Ext Right Spk");
	}

	/* set up jack connection */
	switch (mmp3asoc_jack_func) {
	case MMP3ASOC_HP:
		snd_soc_dapm_disable_pin(dapm, "Headset Mic");
		snd_soc_dapm_enable_pin(dapm, "Main Mic");
		snd_soc_dapm_enable_pin(dapm, "Headset Stereophone");
		break;
	case MMP3ASOC_MIC:
		snd_soc_dapm_disable_pin(dapm, "Headset Mic");
		snd_soc_dapm_enable_pin(dapm, "Main Mic");
		snd_soc_dapm_disable_pin(dapm, "Headset Stereophone");
		break;
	case MMP3ASOC_HEADSET:
		snd_soc_dapm_enable_pin(dapm, "Headset Mic");
		snd_soc_dapm_disable_pin(dapm, "Main Mic");
		snd_soc_dapm_enable_pin(dapm, "Headset Stereophone");
		break;
	case MMP3ASOC_HP_OFF:
		snd_soc_dapm_disable_pin(dapm, "Headset Mic");
		snd_soc_dapm_disable_pin(dapm, "Main Mic");
		snd_soc_dapm_disable_pin(dapm, "Headset Stereophone");
		break;
	}
	snd_soc_dapm_sync(dapm);
	return;
}

static int mmp3asoc_get_jack(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mmp3asoc_jack_func;
	return 0;
}

static int mmp3asoc_set_jack(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (mmp3asoc_jack_func == ucontrol->value.integer.value[0])
		return 0;

	mmp3asoc_jack_func = ucontrol->value.integer.value[0];
	mmp3asoc_ext_control(dapm);
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

	mmp3asoc_spk_func = ucontrol->value.integer.value[0];
	mmp3asoc_ext_control(dapm);
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

	{"IN1RN", NULL, "Headset Mic"},

	{"DMIC1DAT", NULL, "MICBIAS1"},
	{"MICBIAS1", NULL, "Main Mic"},
};

static const char *const jack_function[] = {
	"Headphone", "Mic", "Headset", "Off" };
static const char *const spk_function[] = { "On", "Off" };

static const struct soc_enum mmp3asoc_enum[] = {
	SOC_ENUM_SINGLE_EXT(4, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new mmp3asoc_wm8994_controls[] = {
	SOC_ENUM_EXT("Jack Function", mmp3asoc_enum[0],
		     mmp3asoc_get_jack, mmp3asoc_set_jack),
	SOC_ENUM_EXT("Speaker Function", mmp3asoc_enum[1],
		     mmp3asoc_get_spk, mmp3asoc_set_spk),
};

#ifdef CONFIG_SWITCH_WM8994_HEADSET
static struct snd_soc_codec *mmp3asoc_wm8994_codec;

int wm8994_headset_detect(void)
{
	struct snd_soc_codec *codec;
	int status1, status2, reg;
	int ret = 0;

	codec = mmp3asoc_wm8994_codec;
	if (codec == NULL)
		return 0;

	/* disable interrupt, mask interrupt mask */
	snd_soc_write(codec, WM8994_INTERRUPT_CONTROL, 0x1);
	/* mask MICBIAS interrupt */
	snd_soc_write(codec, WM8994_INTERRUPT_STATUS_2_MASK,
			 0xffff);

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
	snd_soc_write(codec, WM8994_INTERRUPT_RAW_STATUS_2, 0);
	snd_soc_write(codec, WM8994_INTERRUPT_STATUS_1, status1);
	snd_soc_write(codec, WM8994_INTERRUPT_STATUS_2, status2);

	/* enable interrupt, unmask interrupt mask */
	snd_soc_write(codec, WM8994_INTERRUPT_CONTROL, 0x0);
	/* unmask MICBIAS interrupt */
	snd_soc_write(codec, WM8994_INTERRUPT_STATUS_2_MASK,
			 0xffe7);

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
	/* select audio pll */
	__raw_writel(0x908a1881, SSPA_AUD_PLL_CTRL0);
	msleep(100);

	/* select audio pll */
	__raw_writel(0x10801, SSPA_AUD_PLL_CTRL1);
	msleep(100);

	/* div : 12, 44.1K */
	__raw_writel(0x911185, SSPA_AUD_CTRL);
	msleep(100);

}

static int codec_wm8994_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int err;

	audio_subsystem_poweron();
	/* currently the audio pll of mmp3 a0 stepping is not working */
	audio_subsystem_pll_config();

	/* Add mmp3asoc specific controls */
	err = snd_soc_add_controls(codec, mmp3asoc_wm8994_controls,
				   ARRAY_SIZE(mmp3asoc_wm8994_controls));
	if (err < 0)
		return err;

	/* add mmp3asoc specific widgets */
	snd_soc_dapm_new_controls(dapm, mmp3asoc_dapm_widgets,
				  ARRAY_SIZE(mmp3asoc_dapm_widgets));

	/* set up mmp3asoc specific audio routes */
	snd_soc_dapm_add_routes(dapm, mmp3asoc_dapm_routes,
				ARRAY_SIZE(mmp3asoc_dapm_routes));

	snd_soc_dapm_enable_pin(dapm, "Ext Left Spk");
	snd_soc_dapm_enable_pin(dapm, "Ext Right Spk");
	snd_soc_dapm_enable_pin(dapm, "Headset Stereophone");
	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Main Mic");

	mmp3asoc_jack_func = MMP3ASOC_SPK_ON;
	mmp3asoc_spk_func = MMP3ASOC_HEADSET;

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

	snd_soc_dapm_sync(dapm);
#ifdef CONFIG_SWITCH_WM8994_HEADSET
	mmp3asoc_wm8994_codec = codec;
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

	/* 3. GPIO setup */
	/* 3.1 setup the GPIOn as IRQ output */
	snd_soc_write(codec, WM8994_GPIO_1, 0x0003);

	snd_soc_update_bits(codec, WM8994_MICBIAS,
			    WM8994_MICD_ENA_MASK |
			    WM8994_MICD_SCTHR_MASK,
			    WM8994_MICD_ENA |
			    (1 << WM8994_MICD_SCTHR_SHIFT));

	/* turn on micbias 1/2 always */
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

#if 0
	/* need to enable Reg(0x302) bit 13, bit 12 for wm8994 master */
	snd_soc_update_bits(codec, WM8994_AIF1_MASTER_SLAVE,
			    WM8994_AIF1_CLK_FRC_MASK |
			    WM8994_AIF1_LRCLK_FRC_MASK,
			    WM8994_AIF1_CLK_FRC |
			    WM8994_AIF1_LRCLK_FRC);
#endif
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
static int mmp3asoc_wm8994_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;

	cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->playback.rates = MMP3ASOC_SAMPLE_RATES;
	cpu_dai->driver->capture.rates = MMP3ASOC_SAMPLE_RATES;

	return 0;
}

static int mmp3asoc_wm8994_hw_params(struct snd_pcm_substream *substream,
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

	/* set wm8994 sysclk */
	snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1, sysclk, 0);

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
	 .startup = mmp3asoc_wm8994_startup,
	 .hw_params = mmp3asoc_wm8994_hw_params,
	 },
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mmp3_asoc_wm8994_dai[] = {
	{
	 .name = "WM8994 I2S",
	 .stream_name = "I2S Audio",
	 .codec_name = "wm8994-codec",
	 .platform_name = "mmp3-pcm-audio",
	 .cpu_dai_name = "mmp3-sspa-dai.0",
	 .codec_dai_name = "wm8994-aif1",
	 .ops = &mmp3asoc_machine_ops[1],
	 .init = codec_wm8994_init,
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
	 .dai_link = &mmp3_asoc_wm8994_dai[0],
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

	if (!machine_is_abilene())
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
