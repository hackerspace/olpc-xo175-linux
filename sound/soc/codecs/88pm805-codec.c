/*
 * 88pm805-codec.c -- 88PM805 ALSA SoC Audio Driver
 *
 * Copyright 2011 Marvell International Ltd.
 * Author: Xiaofan Tian <tianxf@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mfd/88pm80x.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <trace/events/asoc.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include "88pm805-codec.h"

#define IRQ_NUM			1

#define MAX_NAME_LEN		20

struct pm805_priv {
	unsigned int		sysclk;
	unsigned int		pcmclk;
	unsigned int		dir;
	unsigned int		filter;
	unsigned int		playback_samplerate;
	unsigned int		capture_samplerate;
	struct snd_soc_codec	*codec;
	struct i2c_client	*i2c;
	struct pm80x_chip	*chip;

	int			irq[IRQ_NUM];
	unsigned char		name[IRQ_NUM][MAX_NAME_LEN];
};

static int pm805_volatile(unsigned int reg)
{
	switch (reg) {
	default:
		return 0;
	}
}

static unsigned int pm805_read_reg_cache(struct snd_soc_codec *codec,
					  unsigned int reg)
{
	unsigned char *cache = codec->reg_cache;
	struct pm80x_chip *chip = (struct pm80x_chip *)codec->control_data;
	struct i2c_client *i2c;

	BUG_ON(reg >= codec->reg_size);

	if (!pm805_volatile(reg))
		return cache[reg];

	i2c = chip->pm805_chip->client;
#ifdef CONFIG_CPU_PXA978
	if (chip->base_page && reg >= PM800_CLASS_D_INDEX) {
		reg = reg - PM800_CLASS_D_INDEX + PM800_CLASS_D_REG_BASE;
		i2c = chip->base_page;
	}
#endif
	return pm80x_reg_read(i2c, reg);
}

static int pm805_write_reg_cache(struct snd_soc_codec *codec,
				  unsigned int reg, unsigned int value)
{
	unsigned char *cache = codec->reg_cache;
	struct pm80x_chip *chip = (struct pm80x_chip *)codec->control_data;
	struct i2c_client *i2c;

	BUG_ON(reg >= codec->reg_size);

	value &= 0xff;

	pr_debug("write reg 0x%x, value 0x%x\n", reg, value);

	if (!pm805_volatile(reg))
		cache[reg] = (unsigned char)value;

	i2c = chip->pm805_chip->client;
#ifdef CONFIG_CPU_PXA978
	if (chip->base_page && reg >= PM800_CLASS_D_INDEX) {
		reg = reg - PM800_CLASS_D_INDEX + PM800_CLASS_D_REG_BASE;
		i2c = chip->base_page;
	}

	/* Enable pm800 audio mode */
	if (chip->base_page && reg == PM805_CODEC_MAIN_POWERUP) {
		if (value & PM805_STBY_B)
			pm80x_set_bits(chip->base_page, PM800_LOW_POWER_CONFIG_2,
					PM800_AUDIO_MODE_EN, PM800_AUDIO_MODE_EN);
		else
			pm80x_set_bits(chip->base_page, PM800_LOW_POWER_CONFIG_2,
					PM800_AUDIO_MODE_EN, 0);
	}
#endif

	return pm80x_reg_write(i2c, reg, value);
}

static int pm805_bulk_read_reg(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
static int pm805_bulk_write_reg(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned char buf[PM805_MIXER_COEFFICIENT_MAX_NUM];
	int i, count = 0;
	struct pm80x_chip *chip = (struct pm80x_chip *)codec->control_data;
	struct i2c_client *i2c = chip->pm805_chip->client;

	count = (ucontrol->value.integer.value[0] & 0xff);

	if (count < 1 || count > PM805_MIXER_COEFFICIENT_MAX_NUM) {
		printk("pm805_write_burst_reg: error count %d, must between 1~32\n", count);
		return -EINVAL;
	}

	pr_debug("pm805_burst_write 0x%x, count %d\n", reg, count);

	for (i = 0; i < count; i++) {
		buf[i] = (ucontrol->value.integer.value[i + 1]);
		pr_debug("    value 0x%x\n", buf[i]);
	}

	return pm80x_bulk_write(i2c, reg, count, buf);
}

static int pm805_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int platform_max;

	if (!mc->platform_max)
		mc->platform_max = mc->max;
	platform_max = mc->platform_max;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = PM805_MIXER_COEFFICIENT_MAX_NUM + 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = platform_max;
	return 0;
}

#define SOC_SINGLE_INFO(xname, xreg, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put, xhandler_info) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = xhandler_info, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmax, xinvert) }

static const struct snd_kcontrol_new pm805_audio_controls[] = {
	/* Main Section */
	SOC_SINGLE("PM805_CODEC_ID", PM805_CODEC_ID, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MAIN_POWERUP", PM805_CODEC_MAIN_POWERUP, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_INT_MANAGEMENT", PM805_CODEC_INT_MANAGEMENT, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_INT_1", PM805_CODEC_INT_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_INT_2", PM805_CODEC_INT_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_INT_MASK_1", PM805_CODEC_INT_MASK_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_INT_MASK_2", PM805_CODEC_INT_MASK_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MIC_DETECT_1", PM805_CODEC_MIC_DETECT_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MIC_DETECT_2", PM805_CODEC_MIC_DETECT_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MIC_DETECT_STS", PM805_CODEC_MIC_DETECT_STS, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MIC_DETECT_3", PM805_CODEC_MIC_DETECT_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_AUTO_SEQUENCE_STS_1", PM805_CODEC_AUTO_SEQUENCE_STS_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_AUTO_SEQUENCE_STS_2", PM805_CODEC_AUTO_SEQUENCE_STS_2, 0, 0xff, 0),

	/* ADC/DMIC Section */
	SOC_SINGLE("PM805_CODEC_ADCS_SETTING_1", PM805_CODEC_ADCS_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADCS_SETTING_2", PM805_CODEC_ADCS_SETTING_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADCS_SETTING_3", PM805_CODEC_ADCS_SETTING_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADC_GAIN_1", PM805_CODEC_ADC_GAIN_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADC_GAIN_2", PM805_CODEC_ADC_GAIN_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_DMIC_SETTING", PM805_CODEC_DMIC_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_DWS_SETTING", PM805_CODEC_DWS_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MIC_CONFLICT_STS", PM805_CODEC_MIC_CONFLICT_STS, 0, 0xff, 0),

	/* DAC/PDM Section */
	SOC_SINGLE("PM805_CODEC_PDM_SETTING_1", PM805_CODEC_PDM_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_PDM_SETTING_2", PM805_CODEC_PDM_SETTING_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_PDM_SETTING_3", PM805_CODEC_PDM_SETTING_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_PDM_CONTROL_1", PM805_CODEC_PDM_CONTROL_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_PDM_CONTROL_2", PM805_CODEC_PDM_CONTROL_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_PDM_CONTROL_3", PM805_CODEC_PDM_CONTROL_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_HEADPHONE_SETTING", PM805_CODEC_HEADPHONE_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_HEADPHONE_GAIN_A2A", PM805_CODEC_HEADPHONE_GAIN_A2A, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_HEADPHONE_SHORT_STS", PM805_CODEC_HEADPHONE_SHORT_STS, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_EARPHONE_SETTING", PM805_CODEC_EARPHONE_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_AUTO_SEQUENCE_SETTING", PM805_CODEC_AUTO_SEQUENCE_SETTING, 0, 0xff, 0),

	/* SAI/SRC Section */
	SOC_SINGLE("PM805_CODEC_SAI1_SETTING_1", PM805_CODEC_SAI1_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI1_SETTING_2", PM805_CODEC_SAI1_SETTING_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI1_SETTING_3", PM805_CODEC_SAI1_SETTING_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI1_SETTING_4", PM805_CODEC_SAI1_SETTING_4, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI1_SETTING_5", PM805_CODEC_SAI1_SETTING_5, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI2_SETTING_1", PM805_CODEC_SAI2_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI2_SETTING_2", PM805_CODEC_SAI2_SETTING_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI2_SETTING_3", PM805_CODEC_SAI2_SETTING_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI2_SETTING_4", PM805_CODEC_SAI2_SETTING_4, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SAI2_SETTING_5", PM805_CODEC_SAI2_SETTING_5, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SRC_DPLL_LOCK", PM805_CODEC_SRC_DPLL_LOCK, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SRC_SETTING_1", PM805_CODEC_SRC_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SRC_SETTING_2", PM805_CODEC_SRC_SETTING_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SRC_SETTING_3", PM805_CODEC_SRC_SETTING_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SIDETONE_SETTING", PM805_CODEC_SIDETONE_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SIDETONE_COEFFICIENT_1", PM805_CODEC_SIDETONE_COEFFICIENT_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SIDETONE_COEFFICIENT_2", PM805_CODEC_SIDETONE_COEFFICIENT_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SIDETONE_COEFFICIENT_3", PM805_CODEC_SIDETONE_COEFFICIENT_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SIDETONE_COEFFICIENT_4", PM805_CODEC_SIDETONE_COEFFICIENT_4, 0, 0xff, 0),

	/* DIG/PROC Section */
	SOC_SINGLE("PM805_CODEC_DIGITAL_BLOCK_EN_1", PM805_CODEC_DIGITAL_BLOCK_EN_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_DIGITAL_BLOCK_EN_2", PM805_CODEC_DIGITAL_BLOCK_EN_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_VOL_CHANNEL_1_2_SEL", PM805_CODEC_VOL_CHANNEL_1_2_SEL, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_VOL_CHANNLE_3_4_SEL", PM805_CODEC_VOL_CHANNLE_3_4_SEL, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ZERO_CROSS_AUTOMUTE", PM805_CODEC_ZERO_CROSS_AUTOMUTE, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_VOL_CTRL_PARAM_SEL", PM805_CODEC_VOL_CTRL_PARAM_SEL, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_VOL_SEL_CHANNEL_1", PM805_CODEC_VOL_SEL_CHANNEL_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_VOL_SEL_CHANNEL_2", PM805_CODEC_VOL_SEL_CHANNEL_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_VOL_SEL_CHANNEL_3", PM805_CODEC_VOL_SEL_CHANNEL_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_VOL_SEL_CHANNEL_4", PM805_CODEC_VOL_SEL_CHANNEL_4, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MIX_EQ_COEFFICIENT_1", PM805_CODEC_MIX_EQ_COEFFICIENT_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_MIX_EQ_COEFFICIENT_2", PM805_CODEC_MIX_EQ_COEFFICIENT_2, 0, 0xff, 0),
	SOC_SINGLE_INFO("PM805_CODEC_MIX_EQ_COEFFICIENT_3", PM805_CODEC_MIX_EQ_COEFFICIENT_3, 0, 0xff, 0,
			pm805_bulk_read_reg, pm805_bulk_write_reg, pm805_info_volsw),
	SOC_SINGLE("PM805_CODEC_MIX_EQ_COEFFICIENT_4", PM805_CODEC_MIX_EQ_COEFFICIENT_4, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_CLIP_BITS_1", PM805_CODEC_CLIP_BITS_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_CLIP_BITS_2", PM805_CODEC_CLIP_BITS_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_CLIP_BITS_3", PM805_CODEC_CLIP_BITS_3, 0, 0xff, 0),

	/* Advanced Settings Section */
	SOC_SINGLE("PM805_CODEC_ANALOG_BLOCK_EN", PM805_CODEC_ANALOG_BLOCK_EN, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ANALOG_BLOCK_STS_1", PM805_CODEC_ANALOG_BLOCK_STS_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_PAD_ANALOG_SETTING", PM805_CODEC_PAD_ANALOG_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ANALOG_BLOCK_STS_2", PM805_CODEC_ANALOG_BLOCK_STS_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_CHARGE_PUMP_SETTING_1", PM805_CODEC_CHARGE_PUMP_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_CHARGE_PUMP_SETTING_2", PM805_CODEC_CHARGE_PUMP_SETTING_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_CHARGE_PUMP_SETTING_3", PM805_CODEC_CHARGE_PUMP_SETTING_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_CLOCK_SETTING", PM805_CODEC_CLOCK_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_HEADPHONE_AMP_SETTING", PM805_CODEC_HEADPHONE_AMP_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_POWER_AMP_ENABLE", PM805_CODEC_POWER_AMP_ENABLE, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_HEAD_EAR_PHONE_SETTING", PM805_CODEC_HEAD_EAR_PHONE_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_RECONSTRUCTION_FILTER_1", PM805_CODEC_RECONSTRUCTION_FILTER_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_RECONSTRUCTION_FILTER_2", PM805_CODEC_RECONSTRUCTION_FILTER_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_RECONSTRUCTION_FILTER_3", PM805_CODEC_RECONSTRUCTION_FILTER_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_DWA_SETTING", PM805_CODEC_DWA_SETTING, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_SDM_VOL_DELAY", PM805_CODEC_SDM_VOL_DELAY, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_REF_GROUP_SETTING_1", PM805_CODEC_REF_GROUP_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADC_SETTING_1", PM805_CODEC_ADC_SETTING_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADC_SETTING_2", PM805_CODEC_ADC_SETTING_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADC_SETTING_3", PM805_CODEC_ADC_SETTING_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_ADC_SETTING_4", PM805_CODEC_ADC_SETTING_4, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_FLL_SPREAD_SPECTRUM_1", PM805_CODEC_FLL_SPREAD_SPECTRUM_1, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_FLL_SPREAD_SPECTRUM_2", PM805_CODEC_FLL_SPREAD_SPECTRUM_2, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_FLL_SPREAD_SPECTRUM_3", PM805_CODEC_FLL_SPREAD_SPECTRUM_3, 0, 0xff, 0),
	SOC_SINGLE("PM805_CODEC_FLL_STS", PM805_CODEC_FLL_STS, 0, 0xff, 0),
#ifdef CONFIG_CPU_PXA978
	SOC_SINGLE("PM800_CLASS_D_1", PM800_CLASS_D_1, 0, 0xff, 0),
	SOC_SINGLE("PM800_CLASS_D_2", PM800_CLASS_D_2, 0, 0xff, 0),
	SOC_SINGLE("PM800_CLASS_D_3", PM800_CLASS_D_3, 0, 0xff, 0),
	SOC_SINGLE("PM800_CLASS_D_4", PM800_CLASS_D_4, 0, 0xff, 0),
	SOC_SINGLE("PM800_CLASS_D_5", PM800_CLASS_D_5, 0, 0xff, 0),
#endif
};

/*
 * Use MUTE_LEFT & MUTE_RIGHT to implement digital mute.
 * These bits can also be used to mute.
 */
static int pm805_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	return 0;
}

static int pm805_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned char inf, addr;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		inf = PM805_WLEN_8_BIT;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		inf = PM805_WLEN_16_BIT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		inf = PM805_WLEN_20_BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		inf = PM805_WLEN_24_BIT;
		break;
	default:
		return -EINVAL;
	}

	addr = PM805_CODEC_SAI1_SETTING_1 + (dai->id - 1) * 0x5;
	snd_soc_update_bits(codec, addr, PM805_WLEN_24_BIT, inf);

	/* sample rate */
	switch (params_rate(params)) {
	case 8000:
		inf = PM805_FSYN_RATE_8000;
		break;
	case 11025:
		inf = PM805_FSYN_RATE_11025;
		break;
	case 16000:
		inf = PM805_FSYN_RATE_16000;
		break;
	case 22050:
		inf = PM805_FSYN_RATE_22050;
		break;
	case 32000:
		inf = PM805_FSYN_RATE_32000;
		break;
	case 44100:
		inf = PM805_FSYN_RATE_44100;
		break;
	case 48000:
		inf = PM805_FSYN_RATE_48000;
		break;
	default:
		return -EINVAL;
	}
	addr = PM805_CODEC_SAI1_SETTING_2 + (dai->id - 1) * 0x5;
	snd_soc_update_bits(codec, addr, PM805_FSYN_RATE_128000, inf);

	return 0;
}

static int pm805_set_dai_fmt(struct snd_soc_dai *codec_dai,
				  unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned char inf = 0, mask = 0, addr;

	addr = PM805_CODEC_SAI1_SETTING_1 + (codec_dai->id - 1) * 0x5;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		inf |= PM805_SAI_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		inf &= ~PM805_SAI_MASTER;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		inf |= PM805_SAI_I2S_MODE;
		break;
	default:
		inf &= ~PM805_SAI_I2S_MODE;
		break;
	}
	mask |= PM805_SAI_MASTER | PM805_SAI_I2S_MODE;
	snd_soc_update_bits(codec, addr, mask, inf);
	return 0;
}

static int pm805_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF)
			/* Enable Audio PLL & Audio section */
			snd_soc_update_bits(codec, PM805_CODEC_MAIN_POWERUP,
					PM805_STBY_B, PM805_STBY_B);
		break;

	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
			snd_soc_update_bits(codec, PM805_CODEC_MAIN_POWERUP,
					PM805_STBY_B, 0);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static int pm805_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}
static struct snd_soc_dai_ops pm805_dai_ops = {
	.digital_mute	= pm805_digital_mute,
	.hw_params	= pm805_hw_params,
	.set_fmt	= pm805_set_dai_fmt,
	.set_sysclk	= pm805_set_dai_sysclk,
};

static struct snd_soc_dai_driver pm805_dai[] = {
	{
		/* DAI I2S(SAI1) */
		.name	= "88pm805-i2s",
		.id	= 1,
		.playback = {
			.stream_name	= "I2S Playback",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FORMAT_S16_LE | \
					  SNDRV_PCM_FORMAT_S18_3LE,
		},
		.capture = {
			.stream_name	= "I2S Capture",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FORMAT_S16_LE | \
					  SNDRV_PCM_FORMAT_S18_3LE,
		},
		.ops	= &pm805_dai_ops,
	}, {
		/* DAI PCM(SAI2) */
		.name	= "88pm805-pcm",
		.id	= 2,
		.playback = {
			.stream_name	= "PCM Playback",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FORMAT_S8|		\
					  SNDRV_PCM_FORMAT_S16_LE |	\
					  SNDRV_PCM_FORMAT_S20_3LE |	\
					  SNDRV_PCM_FORMAT_S24,
		},
		.capture = {
			.stream_name	= "PCM Capture",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SNDRV_PCM_FORMAT_S8|		\
					  SNDRV_PCM_FORMAT_S16_LE |	\
					  SNDRV_PCM_FORMAT_S20_3LE |	\
					  SNDRV_PCM_FORMAT_S24,
		},
		.ops	= &pm805_dai_ops,
	},
};

static int pm805_audio_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	return 0;
}


static int pm805_audio_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int pm805_probe(struct snd_soc_codec *codec)
{
	struct pm805_priv *pm805 = snd_soc_codec_get_drvdata(codec);
	struct pm80x_chip *chip = pm805->chip;
	int ret;

	pm805->codec = codec;
	codec->control_data = chip;

	ret = pm80x_bulk_read(pm805->i2c, PM805_CODEC_BASE,
			       PM805_CODEC_REG_SIZE, codec->reg_cache);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to fill register cache: %d\n",
			ret);
		goto out;
	}
#ifdef CONFIG_CPU_PXA978
	if (chip->base_page) {
		ret = pm80x_bulk_read(chip->base_page, PM800_CLASS_D_REG_BASE,
				CODEC_TOTAL_REG_SIZE - PM805_CODEC_REG_SIZE,
				codec->reg_cache + PM805_CODEC_REG_SIZE);
		if (ret < 0) {
			dev_err(codec->dev, "Failed to fill register cache: %d\n",
					ret);
			goto out;
		}
	}
#endif
	/* add below snd ctls to keep align with audio server */
	snd_soc_add_controls(codec, pm805_audio_controls,
			     ARRAY_SIZE(pm805_audio_controls));
	return 0;

out:
	return ret;
}

static int pm805_remove(struct snd_soc_codec *codec)
{
	pm805_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_pm805 = {
	.probe		= pm805_probe,
	.remove		= pm805_remove,
	.read		= pm805_read_reg_cache,
	.write		= pm805_write_reg_cache,
	.suspend	= pm805_audio_suspend,
	.resume		= pm805_audio_resume,
	.reg_cache_size	= CODEC_TOTAL_REG_SIZE,
	.reg_word_size	= sizeof(u8),
	.set_bias_level	= pm805_set_bias_level,
};

static int __devinit pm805_codec_probe(struct platform_device *pdev)
{
	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm805_priv *pm805;
	struct resource *res;
	int i, ret;

	pm805 = kzalloc(sizeof(struct pm805_priv), GFP_KERNEL);
	if (pm805 == NULL)
		return -ENOMEM;

	pm805->chip = chip;
	pm805->i2c = chip->pm805_chip->client;

	platform_set_drvdata(pdev, pm805);

	for (i = 0; i < IRQ_NUM; i++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!res) {
			dev_err(&pdev->dev, "Failed to get IRQ resources\n");
			goto out;
		}
		pm805->irq[i] = res->start + chip->pm805_chip->irq_base;
		strncpy(pm805->name[i], res->name, MAX_NAME_LEN);
	}

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_pm805,
				     pm805_dai, ARRAY_SIZE(pm805_dai));
	if (ret) {
		dev_err(&pdev->dev, "Failed to register codec\n");
		goto out;
	}

	/*
	   Write pm805 debug registers to fix the bug in B0 chip,
	   will remove after pm80x C0
	 */
	if (pm80x_reg_read(pm805->i2c, 0x0) <= 0x3) {
		pm805_debug_reg_write(pm805->i2c, 0xbf, 0x08);
		pm805_debug_reg_write(pm805->i2c, 0xcf, 0x01);
		pm805_debug_reg_write(pm805->i2c, 0xe3, 0x58);
		pm805_debug_reg_write(pm805->i2c, 0x26, 0x09);
	}

	return ret;

out:
	platform_set_drvdata(pdev, NULL);
	kfree(pm805);
	return -EINVAL;
}

static int __devexit pm805_codec_remove(struct platform_device *pdev)
{
	struct pm805_priv *pm805 = platform_get_drvdata(pdev);

	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	kfree(pm805);
	return 0;
}

static struct platform_driver pm805_codec_driver = {
	.driver	= {
		.name	= "88pm80x-codec",
		.owner	= THIS_MODULE,
	},
	.probe	= pm805_codec_probe,
	.remove	= __devexit_p(pm805_codec_remove),
};

static __init int pm805_init(void)
{
	return platform_driver_register(&pm805_codec_driver);
}
module_init(pm805_init);

static __exit void pm805_exit(void)
{
	platform_driver_unregister(&pm805_codec_driver);
}
module_exit(pm805_exit);

MODULE_DESCRIPTION("ASoC 88PM805 driver");
MODULE_AUTHOR("Xiaofan Tian <tianxf@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm805-codec");
