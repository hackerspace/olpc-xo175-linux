/*
 * ce156.c  --  88CE156 ALSA Soc Audio driver
 *
 * Copyright 2011  Marvell International Ltd.
 *
 * All rights reserved
 *
 * Version :
 * 	2013/02/26 Init. For Ariel EVT 2 Build.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include "88ce156-codec.h"

#define AUDIO_NAME "ce156"
#define CE156_VERSION "a0"
#define USE_DAPM_CTRL 0  // remove for audio recoder

#define CE156_DAPM_OUTPUT(wname, wevent)	\
{	.id = snd_soc_dapm_pga, .name = wname, .reg = SND_SOC_NOPM, \
	.shift = 0, .invert = 0, .kcontrols = NULL, \
	.num_kcontrols = 0, .event = wevent, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD, }

struct ce156_private {
	void *control_data;
	struct regmap *regmap;
};

struct ce156_init_reg {
	char name[30];
	u8	reg_value;
	u8	reg_index;
};

static struct ce156_init_reg ce156_init_list[] = {
	{"SPK Input Sel",	0x06,	CE156_SPK_INPUT_SEL},
	{"HS1 Volume",		0x04,	CE156_DAC_HS1_CTRL},
	{"HS2 Volume",		0x04,	CE156_DAC_HS2_CTRL},
	{"SPK Volume",		0x24,	CE156_DAC_SPKR_CTRL},
	{"MIC1 Gain",		0x20,	CE156_MIC1_PGA_GAIN},
	{"MIC2 Gain",		0x20,	CE156_MIC2_PGA_GAIN},
	{"Line1 In Gain",	0x08,	CE156_ADC1_PGA_GAIN},
	{"Line2 In Gain",	0x08,	CE156_ADC2_PGA_GAIN},
};

#define CE156_INIT_REG_NUM ARRAY_SIZE(ce156_init_list)

static const struct reg_default ce156_reg_defaults[] = {
	{ CE156_ADC_PATH,		0x00 },
	{ CE156_ADC_RATE,		0x00 },
	{ CE156_ADCL_ATTN,		0x00 },
	{ CE156_ADCR_ATTN,		0x00 },
	{ CE156_CHARGEPUMP_REG,		0x00 },
	{ CE156_I2S1,			0x00 },
	{ CE156_I2S2,			0x00 },
	{ CE156_DAC_DWA,		0x40 },
	{ CE156_DACEQUL_N0LO,		0x00 },
	{ CE156_DACEQUL_N0HI,		0x00 },
	{ CE156_DACEQUL_N1LO,		0x00 },
	{ CE156_DACEQUL_N1HI,		0x00 },
	{ CE156_DACEQUL_D1LO,		0x00 },
	{ CE156_DACEQUL_D1HI,		0x00 },
	{ CE156_DAC_GAINLL,		0x3f },
	{ 0x10,				0x3f },
	{ 0x11,				0x3f },
	{ CE156_DAC_GAINRR,		0x3f },
	{ CE156_DAC_DWA_OFST,		0x44 },
	{ CE156_PLL1,			0xa1 },
	{ CE156_PLL2,			0x00 },
	{ CE156_PLL_FRACT1,		0x08 },
	{ CE156_PLL_FRACT2,		0x82 },
	{ CE156_PLL_FRACT3,		0x00 },
	{ CE156_CLKGEN1,		0x00 },
	{ CE156_CLKGEN2,		0x06 },
	{ CE156_MIC_CTRL,		0x00 },
	{ CE156_MIC1_PGA_GAIN,		0x00 },
	{ CE156_MIC2_PGA_GAIN,		0x00 },
	{ CE156_ADC1_PGA_GAIN,		0x00 },
	{ CE156_ADC2_PGA_GAIN,		0x00 },
	{ CE156_ADC_RSVD,		0x00 },
	{ CE156_ANALOG_PATH_SEL,	0x00 },
	{ CE156_DAC_HS1_CTRL,		0x00 },
	{ CE156_DAC_HS2_CTRL,		0x00 },
	{ CE156_DAC_SPKR_CTRL,		0x00 },
	{ CE156_DAC_ANA_MISC,		0xa0 },
	{ CE156_AUD_PWR_ENABLE,		0x00 },
	{ CE156_ADC_ANA_ENABLE,		0x00 },
	{ CE156_ADC_DIG_ENABLE,		0x00 },
	{ CE156_DAC_ANA_ENABLE,		0x00 },
	{ CE156_DAC_DIG_ENABLE,		0x00 },
	{ CE156_HS_INPUT_SEL,		0x00 },
	{ CE156_SPK_INPUT_SEL,		0x00 },
	{ CE156_HS_MIC_DET,		0x20 },
	{ CE156_STATUS1,		0x00 },
	{ CE156_STATUS2,		0x00 },
	{ CE156_REVISION,		0x03 },
	{ CE156_CLKEN,			0xff },
	{ 0x4a,				0x12 },
};

static const struct regmap_config ce156_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x9f, // XXX
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = ce156_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ce156_reg_defaults),
};

#ifdef CONFIG_MACH_QSEVEN
#define HP_L_EN		1
#define HP_R_EN		2
#define SPK_L_EN	32
#define SPK_R_EN	64
#endif

static int ce156_sync(struct snd_soc_component *component)
{
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa0);
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa4);
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa0);
	return 0;
}

static int ce156_reg_init(struct snd_soc_component *component)
{
	int i;

	for (i = 0; i < CE156_INIT_REG_NUM; i++)
		snd_soc_component_write(component,
			     ce156_init_list[i].reg_index,
			     ce156_init_list[i].reg_value);

	return 0;
}

static int caps_charge = 2000;
module_param(caps_charge, int, 0);
MODULE_PARM_DESC(caps_charge, "CE156 cap charge time (msecs)");


/* FIXME:
 * how to define AUD155_RESET?
 */
static int ce156_reset(struct snd_soc_component *component)
{
	snd_soc_component_write(component, CE156_CLKGEN2, 0x4);
	snd_soc_component_write(component, CE156_AUD_PWR_ENABLE, 0);

	snd_soc_component_write(component, CE156_CHARGEPUMP_REG, 0x7);
	snd_soc_component_write(component, CE156_CLKGEN1, 0x2);
	snd_soc_component_write(component, CE156_CLKGEN2, 0x7);
	snd_soc_component_write(component, CE156_AUD_PWR_ENABLE, 0x43);
	return 0;
}

static int ce156_dapm_event(struct snd_soc_component *component, enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		printk(KERN_INFO "SND_SOC_BIAS_ON\n");
		break;
	case SND_SOC_BIAS_PREPARE:
		printk(KERN_INFO "SND_SOC_BIAS_PREPARE\n");
// XXX //	ce156_hw_init(component);
		break;
	case SND_SOC_BIAS_STANDBY:
		printk(KERN_INFO "SND_SOC_BIAS_STANDBY\n");
//		ce156_hw_init(component);
		break;
	case SND_SOC_BIAS_OFF:
		printk(KERN_INFO "SND_SOC_BIAS_OFF\n");
		/* ce156 power off sequence */
		break;
	default:
		break;
	}
	
	return 0;
}


int ce156_set_sample_rate(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_component *component = codec_dai->component;
	// XXX update bits
	u8 rate = snd_soc_component_read32(component, CE156_ADC_RATE) & 0xf0;
	switch(div_id){
	case SAMPLE_RATE_8000:
		rate |= 0x0;
		break;
	case SAMPLE_RATE_12000:
		rate |= 0x1;
		break;
	case SAMPLE_RATE_16000:
		rate |= 0x2;
		break;
	case SAMPLE_RATE_24000:
		rate |= 0x3;
		break;
	case SAMPLE_RATE_32000:
		rate |= 0x4;
		break;
	case SAMPLE_RATE_48000:
		rate |= 0x5;
		break;
	case SAMPLE_RATE_96000:
		rate |= 0x6;
		break;
	case SAMPLE_RATE_11025:
		rate |= 0x7;
		break;
	case SAMPLE_RATE_22050:
		rate |= 0x8;
		break;
	case SAMPLE_RATE_44100:
		rate |= 0x9;
		break;
	case SAMPLE_RATE_88200:
		rate |= 0xa;
		break;
	default:
		printk(KERN_INFO "sample rate set failed!\n");
		return -EINVAL;
	}
	snd_soc_component_write(component, CE156_ADC_RATE, rate);
	return 0;
}
EXPORT_SYMBOL_GPL(ce156_set_sample_rate);

/* DAPM Widget Events */

static int ce156_mixer_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	u8 val;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		pr_debug("after power up!\n");
		val = snd_soc_component_read32(component,CE156_ADC_ANA_ENABLE);
		if((val & 0xc0) == 0x40)
			snd_soc_component_write(component, CE156_ADC_ANA_ENABLE, (val & 0xf7));
		else if((val & 0xc0) == 0x80)
			snd_soc_component_write(component, CE156_ADC_ANA_ENABLE, (val & 0xfb));
		else if((val & 0xc0) == 0xc0)
			snd_soc_component_write(component, CE156_ADC_ANA_ENABLE, (val & 0xcf));
		break;

	default:
		break;
	}

	return 0;
}


/*
 * A lot registers are belong to RSYNC domain. It requires enabling RSYNC bit
 * after updating these registers. Otherwise, these updated registers won't
 * be effective.
 */
static int ce156_rsync_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	/*
	 * In order to avoid current on the load, mute power-on and power-off
	 * should be transients.
	 * Unmute by DAC_MUTE. It should be unmuted when DAPM sequence is
	 * finished.
	 */
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa4);
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa0);

	return 0;
}


static const char * ce156_mic1mux_source_sel[] = {
	"MIC1 SE ON", "MIC1 DIFF ON", "MIC1 OFF", "MIC1 DIFF ON"
};

static const char * ce156_mic2mux_source_sel[] = {
	"MIC2 SE ON", "MIC2 OFF", "MIC2 DIFF ON", "MIC2 DIFF ON"
};

static const char * ce156_linelmux_source_sel[] = {
	"LINEIN L ON", "LINEIN L ON", "LINEIN L ON", "LINEIN L OFF"
};

static const char * ce156_linermux_source_sel[] = {
	"LINEIN R ON", "LINEIN R ON", "LINEIN R ON", "LINEIN R OFF"
};

static const char * ce156_adclmux_source_sel[] = {
	"left modulator data", "right modulator data"
};

static const char * ce156_adcrmux_source_sel[] = {
	"right modulator data", "left modulator data"
};

static const char * ce156_adcmixer_source_sel[] = {
	"(L+R)/2", "(L-R)/2"
};

static const char * ce156_adc1_output_mux_source_sel[] = {
	"left adc channel", "adc mixer output"
};

static const char * ce156_adc2_output_mux_source_sel[] = {
	"right adc channel", "adc mixer output"
};

static const char * ce156_hplmux_source_sel[] = {
	"no input", "right channel", "left channel", "no input"
};

static const char * ce156_hprmux_source_sel[] = {
	"no input", "right channel", "left channel", "no input"
};

static const char * ce156_spk1mux_source_sel[] = {
	"no input", "right channel", "left channel", "left + right"
};

static const char * ce156_spk2mux_source_sel[] = {
	"no input", "right channel", "left channel", "reserved"
};

static const char * ce156_hs1mux_source_sel[] = {
	"DAC stream input", "analog left mixer input"
};

static const char * ce156_hs2mux_source_sel[] = {
	"DAC stream input", "analog right mixer input"
};


static const struct soc_enum mic1mux_source =
	SOC_ENUM_SINGLE(CE156_ADC_ANA_ENABLE, 6, 4,	ce156_mic1mux_source_sel);

static const struct soc_enum mic2mux_source =
	SOC_ENUM_SINGLE(CE156_ADC_ANA_ENABLE, 6, 4,	ce156_mic2mux_source_sel);

static const struct soc_enum linelmux_source =
	SOC_ENUM_SINGLE(CE156_ADC_ANA_ENABLE, 6, 4,	ce156_linelmux_source_sel);

static const struct soc_enum linermux_source =
	SOC_ENUM_SINGLE(CE156_ADC_ANA_ENABLE, 6, 4,	ce156_linermux_source_sel);

static const struct soc_enum adc_lsel_mux_source =
	SOC_ENUM_SINGLE(CE156_ADC_PATH, 3, 2,	ce156_adclmux_source_sel);

static const struct soc_enum adc_rsel_mux_source =
	SOC_ENUM_SINGLE(CE156_ADC_PATH, 2, 2,	ce156_adcrmux_source_sel);

static const struct soc_enum adc_mixer_mux_source =
	SOC_ENUM_SINGLE(CE156_ADC_PATH, 6, 2,	ce156_adcmixer_source_sel);

static const struct soc_enum adc1_output_mux_source =
	SOC_ENUM_SINGLE(CE156_ADC_PATH, 7, 2,	ce156_adc1_output_mux_source_sel);

static const struct soc_enum adc2_output_mux_source =
	SOC_ENUM_SINGLE(CE156_ADC_PATH, 7, 2,	ce156_adc2_output_mux_source_sel);

static const struct soc_enum hp_lsel_mux_source =
	SOC_ENUM_SINGLE(CE156_HS_INPUT_SEL, 0, 4, ce156_hplmux_source_sel);

static const struct soc_enum hp_rsel_mux_source =
	SOC_ENUM_SINGLE(CE156_HS_INPUT_SEL, 2, 4, ce156_hprmux_source_sel);

static const struct soc_enum spk_1sel_mux_source =
	SOC_ENUM_SINGLE(CE156_SPK_INPUT_SEL, 0, 4, ce156_spk1mux_source_sel);

static const struct soc_enum spk_2sel_mux_source =
	SOC_ENUM_SINGLE(CE156_SPK_INPUT_SEL, 2, 4, ce156_spk2mux_source_sel);

static const struct soc_enum hs_1sel_mux_source =
	SOC_ENUM_SINGLE(CE156_ANALOG_PATH_SEL, 0, 2, ce156_hs1mux_source_sel);

static const struct soc_enum hs_2sel_mux_source =
	SOC_ENUM_SINGLE(CE156_ANALOG_PATH_SEL, 1, 2, ce156_hs2mux_source_sel);


static const struct snd_kcontrol_new ce156_mic1_mux_controls =
	SOC_DAPM_ENUM("Route", mic1mux_source);

static const struct snd_kcontrol_new ce156_mic2_mux_controls =
	SOC_DAPM_ENUM("Route", mic2mux_source);

static const struct snd_kcontrol_new ce156_line_l_mux_controls =
	SOC_DAPM_ENUM("Route", linelmux_source);

static const struct snd_kcontrol_new ce156_line_r_mux_controls =
	SOC_DAPM_ENUM("Route", linermux_source);

static const struct snd_kcontrol_new ce156_adc_lsel_mux_controls =
	SOC_DAPM_ENUM("Route", adc_lsel_mux_source);

static const struct snd_kcontrol_new ce156_adc_rsel_mux_controls =
	SOC_DAPM_ENUM("Route", adc_rsel_mux_source);

static const struct snd_kcontrol_new ce156_adc_mixer_mux_controls =
	SOC_DAPM_ENUM("Route", adc_mixer_mux_source);

static const struct snd_kcontrol_new ce156_adc1_output_mux_controls =
	SOC_DAPM_ENUM("Route", adc1_output_mux_source);

static const struct snd_kcontrol_new ce156_adc2_output_mux_controls =
	SOC_DAPM_ENUM("Route", adc2_output_mux_source);

static const struct snd_kcontrol_new ce156_hp_lsel_mux_controls =
	SOC_DAPM_ENUM("Route", hp_lsel_mux_source);

static const struct snd_kcontrol_new ce156_hp_rsel_mux_controls =
	SOC_DAPM_ENUM("Route", hp_rsel_mux_source);

static const struct snd_kcontrol_new ce156_spk_1sel_mux_controls =
	SOC_DAPM_ENUM("Route", spk_1sel_mux_source);

static const struct snd_kcontrol_new ce156_spk_2sel_mux_controls =
	SOC_DAPM_ENUM("Route", spk_2sel_mux_source);

static const struct snd_kcontrol_new ce156_hs_1sel_mux_controls =
	SOC_DAPM_ENUM("Route", hs_1sel_mux_source);

static const struct snd_kcontrol_new ce156_hs_2sel_mux_controls =
	SOC_DAPM_ENUM("Route", hs_2sel_mux_source);

static const unsigned int mic_pga_tlv[] = {
	TLV_DB_RANGE_HEAD(8),
	0, 5, TLV_DB_SCALE_ITEM(0, 100, 0),
	6, 7, TLV_DB_SCALE_ITEM(500, 0, 0),
	8, 13, TLV_DB_SCALE_ITEM(600, 100, 0),
	14, 15, TLV_DB_SCALE_ITEM(1100, 0, 0),
	16, 21, TLV_DB_SCALE_ITEM(-600, 100, 0),
	22, 23, TLV_DB_SCALE_ITEM(-100, 0, 0),
	24, 29, TLV_DB_SCALE_ITEM(1200, 100, 0),
	30, 31, TLV_DB_SCALE_ITEM(1700, 0, 0),
};

static const unsigned int adc_pga_tlv[] = {
	TLV_DB_RANGE_HEAD(8),
	0, 5, TLV_DB_SCALE_ITEM(0, 100, 0),
	6, 7, TLV_DB_SCALE_ITEM(500, 0, 0),
	8, 13, TLV_DB_SCALE_ITEM(600, 100, 0),
	14, 15, TLV_DB_SCALE_ITEM(1100, 0, 0),
	16, 21, TLV_DB_SCALE_ITEM(-600, 100, 0),
	22, 23, TLV_DB_SCALE_ITEM(-100, 0, 0),
	24, 29, TLV_DB_SCALE_ITEM(1200, 100, 0),
	30, 31, TLV_DB_SCALE_ITEM(1700, 0, 0),
};

/* mute(-1), 0, 3, 6, 7.66, 9.54, 10.66, 12.89 db*/
static const unsigned int hs_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(8),
	0, 0, TLV_DB_SCALE_ITEM(-100, 0, 1),
	1, 1, TLV_DB_SCALE_ITEM(600, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(300, 0, 0),
	3, 3, TLV_DB_SCALE_ITEM(1066, 0, 0),
	4, 4, TLV_DB_SCALE_ITEM(0, 0, 0),
	5, 5, TLV_DB_SCALE_ITEM(954, 0, 0),
	6, 6, TLV_DB_SCALE_ITEM(766, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(1289, 0, 0),
};

/* mute(-1), 0, 3, 6, 7.66, 9.54, 10.66, 12.89 db*/
static const unsigned int spk_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(8),
	0, 0, TLV_DB_SCALE_ITEM(-100, 0, 1),
	1, 1, TLV_DB_SCALE_ITEM(600, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(300, 0, 0),
	3, 3, TLV_DB_SCALE_ITEM(1066, 0, 0),
	4, 4, TLV_DB_SCALE_ITEM(0, 0, 0),
	5, 5, TLV_DB_SCALE_ITEM(954, 0, 0),
	6, 6, TLV_DB_SCALE_ITEM(766, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(1289, 0, 0),
};

static const unsigned int dac_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 63, TLV_DB_SCALE_ITEM(-9450, 150, 1),
};

#ifdef CONFIG_MACH_QSEVEN
static int ce156_mute_mode_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	u8 mute_reg = snd_soc_component_read32(component, CE156_DAC_ANA_ENABLE) & 0xff;

	if( mute_reg == 0x00 ) {
		ucontrol->value.integer.value[0] = 1;
	}
	else if( mute_reg == 0x63 ) {
		ucontrol->value.integer.value[0] = 0;
	}
	
	return 0;
}

static int ce156_mute_mode_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	u8 mute_reg = snd_soc_component_read32(component, CE156_DAC_ANA_ENABLE) & 0xff;
	u8 tmp = 0;

	if( ucontrol->value.integer.value[0] == 1 ) {
		tmp = 0x63;
		mute_reg &= ~tmp;
		snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, mute_reg);
	}
	else if( ucontrol->value.integer.value[0] == 0 ) {
		mute_reg |= 0x63;
		snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, mute_reg);
	}

	return 0;
}

#define LEFT_HEADPHONE_PA_ENABLE	0x01
#define RIGHT_HEADPHONE_PA_ENABLE	1 << LEFT_HEADPHONE_PA_ENABLE
static int ce156_right_mute_mode_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	u8 mute_reg = snd_soc_component_read32(component, CE156_DAC_ANA_ENABLE) & 0xff;
	
	if( mute_reg == 0x63 ) {
		ucontrol->value.integer.value[0] = 0;
	}
	else if( mute_reg == 0x61 ) {
		ucontrol->value.integer.value[0] = 1;
	}
	else if( mute_reg == 0x60 ) {		
		ucontrol->value.integer.value[0] = 1;
	}	
	
	return 0;
}

static int ce156_right_mute_mode_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	u8 mute_reg = snd_soc_component_read32(component, CE156_DAC_ANA_ENABLE) & 0xff;

	if( ucontrol->value.integer.value[0] == 1 ) {
		mute_reg &= ~(RIGHT_HEADPHONE_PA_ENABLE);
		snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, mute_reg);
	}
	else if( ucontrol->value.integer.value[0] == 0 ) {
		mute_reg |= RIGHT_HEADPHONE_PA_ENABLE;
		snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, mute_reg);
	}

	return 0;
}

static int ce156_left_mute_mode_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	u8 mute_reg = snd_soc_component_read32(component, CE156_DAC_ANA_ENABLE) & 0xff;

        if( mute_reg == 0x63 ) {
                ucontrol->value.integer.value[0] = 0;
	}
        else if( mute_reg == 0x62 ) {
                ucontrol->value.integer.value[0] = 1;
	}
        else if( mute_reg == 0x60 ) {
		ucontrol->value.integer.value[0] = 1;
	}
	else {
	}
		
	return 0;
}

static int ce156_left_mute_mode_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	u8 mute_reg = snd_soc_component_read32(component, CE156_DAC_ANA_ENABLE) & 0xff;
	u8 tmp = 0;
	
        if( ucontrol->value.integer.value[0] == 1 ) {
		tmp = LEFT_HEADPHONE_PA_ENABLE;
                mute_reg &= ~(LEFT_HEADPHONE_PA_ENABLE);
		snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, mute_reg);
        }
        else if( ucontrol->value.integer.value[0] == 0 ) {
		mute_reg |= LEFT_HEADPHONE_PA_ENABLE;
		snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, mute_reg);
        }

	return 0;
}
#endif

static const struct snd_kcontrol_new ce156_snd_controls[] = {
#ifdef CONFIG_MACH_QSEVEN
	SOC_SINGLE_TLV("HP/L Volume", CE156_DAC_HS1_CTRL, 0, 7, 0, hs_gain_tlv),
        SOC_SINGLE_TLV("HP/R Volume", CE156_DAC_HS2_CTRL, 0, 7, 0, hs_gain_tlv),
	SOC_SINGLE_TLV("SPK Volume", CE156_DAC_SPKR_CTRL, 3, 7, 0, spk_gain_tlv),
	SOC_SINGLE_TLV("MIC1 PGA Volume", CE156_MIC1_PGA_GAIN, 0, 31, 0, mic_pga_tlv),
	SOC_SINGLE_EXT("HP LEFT Mute", CE156_DAC_ANA_ENABLE, 1, 1, 0, ce156_left_mute_mode_get, ce156_left_mute_mode_set),
	SOC_SINGLE_EXT("HP RIGHT Mute", CE156_DAC_ANA_ENABLE, 0, 1, 0, ce156_right_mute_mode_get, ce156_right_mute_mode_set),
	SOC_SINGLE_EXT("Mute", CE156_DAC_DIG_ENABLE, 4, 1, 0, ce156_mute_mode_get, ce156_mute_mode_set),
#else
	SOC_SINGLE_TLV("MIC1 PGA Volume", CE156_MIC1_PGA_GAIN, 0, 31, 0, mic_pga_tlv),
	SOC_SINGLE_TLV("MIC2 PGA Volume", CE156_MIC2_PGA_GAIN, 0, 31, 0, mic_pga_tlv),
	SOC_SINGLE_TLV("Linein1 PGA Volume", CE156_ADC1_PGA_GAIN, 0, 31, 0, adc_pga_tlv),
	SOC_SINGLE_TLV("Linein2 PGA Volume", CE156_ADC2_PGA_GAIN, 0, 31, 0, adc_pga_tlv),
	SOC_SINGLE_TLV("HS1 Gain Volume", CE156_DAC_HS1_CTRL, 0, 7, 0, hs_gain_tlv),
	SOC_SINGLE_TLV("HS2 Gain Volume", CE156_DAC_HS2_CTRL, 0, 7, 0, hs_gain_tlv),
	SOC_SINGLE_TLV("SPK1 Gain Volume", CE156_DAC_SPKR_CTRL, 0, 7, 0, spk_gain_tlv),
	SOC_SINGLE_TLV("SPK2 Gain Volume", CE156_DAC_SPKR_CTRL, 3, 7, 0, spk_gain_tlv),
	SOC_SINGLE_TLV("DAC LL GAIN Volume", CE156_DAC_GAINLL, 0, 63, 1, dac_gain_tlv),
	SOC_SINGLE_TLV("DAC RR GAIN Volume", CE156_DAC_GAINRR, 0, 63, 1, dac_gain_tlv),
#endif
};

static const struct snd_soc_dapm_widget ce156_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC1P"),
	SND_SOC_DAPM_INPUT("MIC2P"),
	SND_SOC_DAPM_INPUT("MIC1N"),
	SND_SOC_DAPM_INPUT("MIC2N"),
	SND_SOC_DAPM_INPUT("LINEL"),
	SND_SOC_DAPM_INPUT("LINER"),

	SND_SOC_DAPM_MIXER("MIC1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("MIC2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("MIC1 Mux", SND_SOC_NOPM, 0, 0, &ce156_mic1_mux_controls),
	SND_SOC_DAPM_MUX("MIC2 Mux", SND_SOC_NOPM, 0, 0, &ce156_mic2_mux_controls),
	SND_SOC_DAPM_MUX("Line L Mux", SND_SOC_NOPM, 0, 0, &ce156_line_l_mux_controls),
	SND_SOC_DAPM_MUX("Line R Mux", SND_SOC_NOPM, 0, 0, &ce156_line_r_mux_controls),


	SND_SOC_DAPM_PGA("MIC1 GAIN", CE156_MIC1_PGA_GAIN, 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC2 GAIN", CE156_MIC2_PGA_GAIN, 5, 0, NULL, 0),

	SND_SOC_DAPM_PGA("MIC PGA1", CE156_ADC_ANA_ENABLE, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC PGA2", CE156_ADC_ANA_ENABLE, 3, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Line PGA1", CE156_ADC_ANA_ENABLE, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Line PGA2", CE156_ADC_ANA_ENABLE, 5, 0, NULL, 0),

	/* need to be fixed here */
	SND_SOC_DAPM_MIXER_E("Left in Mixer", SND_SOC_NOPM, 0, 0, NULL, 0,
		ce156_mixer_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("Right in Mixer", SND_SOC_NOPM, 0, 0, NULL, 0,
		ce156_mixer_event, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_PGA("ADC Modulator1", CE156_ADC_ANA_ENABLE, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC Modulator2", CE156_ADC_ANA_ENABLE, 1, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Left ADC channel", SND_SOC_NOPM, 0, 0, &ce156_adc_lsel_mux_controls),
	SND_SOC_DAPM_MUX("Right ADC channel", SND_SOC_NOPM, 0, 0, &ce156_adc_rsel_mux_controls),

	SND_SOC_DAPM_MUX("ADC Mixer", SND_SOC_NOPM, 0, 0, &ce156_adc_mixer_mux_controls),

	SND_SOC_DAPM_MUX("ADC1 output", SND_SOC_NOPM, 0, 0, &ce156_adc1_output_mux_controls),
	SND_SOC_DAPM_MUX("ADC2 output", SND_SOC_NOPM, 0, 0, &ce156_adc2_output_mux_controls),

	SND_SOC_DAPM_DAC("Left DAC", "Playback DAC", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Playback DAC", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_PGA("DAC Modulator", CE156_DAC_DIG_ENABLE, 3, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Left DAC DIG", CE156_DAC_DIG_ENABLE, 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right DAC DIG", CE156_DAC_DIG_ENABLE, 4, 0, NULL, 0),


	SND_SOC_DAPM_MUX("HP L Mux", SND_SOC_NOPM, 0, 0, &ce156_hp_lsel_mux_controls),
	SND_SOC_DAPM_MUX("HP R Mux", SND_SOC_NOPM, 0, 0, &ce156_hp_rsel_mux_controls),
	SND_SOC_DAPM_MUX("SPK1 Mux", SND_SOC_NOPM, 0, 0, &ce156_spk_1sel_mux_controls),
	SND_SOC_DAPM_MUX("SPK2 Mux", SND_SOC_NOPM, 0, 0, &ce156_spk_2sel_mux_controls),

	SND_SOC_DAPM_MUX("HS1 Mux", SND_SOC_NOPM, 0, 0, &ce156_hs_1sel_mux_controls),
	SND_SOC_DAPM_MUX("HS2 Mux", SND_SOC_NOPM, 0, 0, &ce156_hs_2sel_mux_controls),

	SND_SOC_DAPM_PGA("HS1 PGA", CE156_DAC_ANA_ENABLE, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HS2 PGA", CE156_DAC_ANA_ENABLE, 1, 0, NULL, 0),

	SND_SOC_DAPM_PGA("SPK1 PGA", CE156_DAC_ANA_ENABLE, 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("SPK2 PGA", CE156_DAC_ANA_ENABLE, 6, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("SPKLP"),
	SND_SOC_DAPM_OUTPUT("SPKLN"),
	SND_SOC_DAPM_OUTPUT("SPKRP"),
	SND_SOC_DAPM_OUTPUT("SPKRN"),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),

	CE156_DAPM_OUTPUT("RSYNC", ce156_rsync_event),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* mic1  mic2 linel liner mux */
	{"MIC1 Mixer", NULL, "MIC1P"},
	{"MIC1 Mixer", NULL, "MIC1N"},
	{"MIC2 Mixer", NULL, "MIC2P"},
	{"MIC2 Mixer", NULL, "MIC2N"},

	{"MIC1 Mux", "MIC1 SE ON", "MIC1P"},
	{"MIC1 Mux", "MIC1 DIFF ON", "MIC1 Mixer"},
	{"MIC2 Mux", "MIC2 SE ON", "MIC2P"},
	{"MIC2 Mux", "MIC2 DIFF ON", "MIC2 Mixer"},

	{"Line L Mux", "LINEIN L ON", "MIC1N"},
	{"Line R Mux", "LINEIN R ON", "MIC2N"},

	/* mic1 mic2 gain */
	{"MIC1 GAIN", NULL, "MIC1 Mux"},
	{"MIC2 GAIN", NULL, "MIC2 Mux"},

	/* mic1 mic2 PGA */
	{"MIC PGA1", NULL, "MIC1 GAIN"},
	{"MIC PGA2", NULL, "MIC2 GAIN"},

	/* linel liner PGA */
	{"Line PGA1", NULL, "Line L Mux"},
	{"Line PGA2", NULL, "Line R Mux"},

	/* Left_Mixer */
	{"Left in Mixer", NULL, "MIC PGA1"},
	{"Left in Mixer", NULL, "Line PGA1"},
	/* Right_Mixer */
	{"Right in Mixer", NULL, "MIC PGA2"},
	{"Right in Mixer", NULL, "Line PGA2"},

	{"ADC Modulator1", NULL, "Left in Mixer"},
	{"ADC Modulator2", NULL, "Right in Mixer"},

	/* MUX4 */
	{"Left ADC channel", "left modulator data", "ADC Modulator1"},
	{"Left ADC channel", "right modulator data", "ADC Modulator2"},

	/* MUX5 */
	{"Right ADC channel", "left modulator data", "ADC Modulator1"},
	{"Right ADC channel", "right modulator data", "ADC Modulator2"},

	/* ADC MIXER */
	{"ADC Mixer", "(L+R)/2", "Left ADC channel"},
	{"ADC Mixer", "(L+R)/2", "Right ADC channel"},
	{"ADC Mixer", "(L-R)/2", "Left ADC channel"},
	{"ADC Mixer", "(L-R)/2", "Right ADC channel"},

	/* MUX22 & MUX23 */
	{"ADC1 output", "left adc channel", "Left ADC channel"},
	{"ADC1 output", "adc mixer output", "ADC Mixer"},
	{"ADC2 output", "right adc channel", "Right ADC channel"},
	{"ADC2 output", "adc mixer output", "ADC Mixer"},

	/* DAC in */
	{"DAC Modulator", NULL, "Left DAC"},
	{"DAC Modulator", NULL, "Right DAC"},

	/* Left Right DAC Dig */
	{"Left DAC DIG", NULL, "DAC Modulator"},
	{"Right DAC DIG", NULL, "DAC Modulator"},

	/* HP SPK Mux */
	{"HP L Mux", "left channel", "Left DAC DIG"},
	{"HP L Mux", "right channel", "Right DAC DIG"},
	{"HP R Mux", "left channel", "Left DAC DIG"},
	{"HP R Mux", "right channel", "Right DAC DIG"},
	{"SPK1 Mux", "left channel", "Left DAC DIG"},
	{"SPK1 Mux", "right channel", "Right DAC DIG"},
	{"SPK1 Mux", "left + right", "Left DAC DIG"},
	{"SPK1 Mux", "left + right", "Right DAC DIG"},
	{"SPK2 Mux", "left channel", "Left DAC DIG"},
	{"SPK2 Mux", "right channel", "Right DAC DIG"},

	/* HS Mux  */
	{"HS1 Mux", "analog left mixer input", "Left in Mixer"},
	{"HS1 Mux", "DAC stream input", "HP L Mux"},
	{"HS2 Mux", "analog right mixer input", "Right in Mixer"},
	{"HS2 Mux", "DAC stream input", "HP R Mux"},

	/* HS PGA */
	{"HS1 PGA", NULL, "HS1 Mux"},
	{"HS2 PGA", NULL, "HS2 Mux"},

	/* SPK PGA */
	{"SPK1 PGA", NULL, "SPK1 Mux"},
	{"SPK2 PGA", NULL, "SPK2 Mux"},

	/* SYNC */
	{"RSYNC", NULL, "HS1 PGA"},
	{"RSYNC", NULL, "HS2 PGA"},
	{"RSYNC", NULL, "SPK1 PGA"},
	{"RSYNC", NULL, "SPK2 PGA"},

	/* Output */
	{"HPL", NULL, "RSYNC"},
	{"HPR", NULL, "RSYNC"},
	{"SPKLP", NULL, "RSYNC"},
	{"SPKLN", NULL, "RSYNC"},
	{"SPKRP", NULL, "RSYNC"},
	{"SPKRN", NULL, "RSYNC"},
};


static int ce156_add_widgets(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_new_controls(dapm, ce156_dapm_widgets,
				  ARRAY_SIZE(ce156_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(dapm->card);

	return 0;
}


struct ce156_sample_rate_list {
	u8 rate_i2s_pcm;
	unsigned int rate;
	unsigned int bclk;
};

static const struct ce156_sample_rate_list sample_rate_list[] = {
	{ 0x0,  8000,  672000},
	{ 0x1, 12000,  672000},
	{ 0x2, 16000, 1344000},
	{ 0x3, 24000, 1344000},
	{ 0x4, 32000, 2688000},
	{ 0x5, 48000, 2688000},
	{ 0x6, 96000, 8064000},
	{ 0x7, 11025,  705600},
	{ 0x8, 22050, 1411200},
	{ 0x9, 44100, 2822400},
	{ 0xa, 88200, 5644800},
};

static int ce156_get_coeff(unsigned int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sample_rate_list); i++) {
		if (sample_rate_list[i].rate == rate)
			return i;
	}

	return -EINVAL;
}

static int ce156_set_dai_pll(struct snd_soc_dai *codec_dai,
			      int pll_id, int source,
			      unsigned int freq_in,
			      unsigned int freq_out)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 pll1 = 0, pll2 = 0;
	u8 pll_fract1 = 0, pll_fract2 = 0, pll_fract3 = 0;
	u8 pll_div_mclk = 0, pll_div_ref = 0, pll_div_fbc = 0;
	u32 pll_fract = 0;

	printk(KERN_INFO "enter %s\n", __func__);
	printk(KERN_DEBUG "%s: freq_in = %d, freq_out = %d\n",
	       __func__, freq_in, freq_out);

	switch(freq_in) {
		case 13000000:
			pll_div_mclk = 0x1;
			pll_div_ref = 0x5;
			pll_div_fbc = 0x0;
			pll_fract = 0x08208;
			break;
		case 16934400:
			pll_div_mclk = 0x2;
			pll_div_ref = 0x0;
			pll_div_fbc = 0x3;
			pll_fract = 0x06666;
			break;
		case 18432000:
			pll_div_mclk = 0x2;
			pll_div_ref = 0x1;
			pll_div_fbc = 0x2;
			pll_fract = 0x02527;
			break;
		case 22579200:
			pll_div_mclk = 0x2;
			pll_div_ref = 0x3;
			pll_div_fbc = 0x2;
			pll_fract = 0x0;
			break;
		case 24576000:
			pll_div_mclk = 0x2;
			pll_div_ref = 0x4;
			pll_div_fbc = 0x2;
			pll_fract = 0x02b23;
			break;
		case 26000000:
			pll_div_mclk = 0x2;
			pll_div_ref = 0x5;
			pll_div_fbc = 0x0;
			pll_fract = 0x08208;
			break;
		case 38400000:
			pll_div_mclk = 0x3;
			pll_div_ref = 0x5;
			pll_div_fbc = 0x0;
			pll_fract = 0x17c18;
			break;
		default:
			printk(KERN_WARNING "the input MCLK not supported\n");
			return -EINVAL;
	}
	pll1 = (pll_div_mclk << 1) | (pll_div_ref << 5);
	pll2 = pll_div_fbc;
	pll_fract1 = pll_fract & 0xff;
	pll_fract2 = (pll_fract >> 8) & 0xff;
	pll_fract3 = (pll_fract >> 16) & 0xff;

	snd_soc_component_write(component, CE156_PLL1, pll1);
	snd_soc_component_write(component, CE156_PLL2, pll2);
	snd_soc_component_write(component, CE156_PLL_FRACT1, pll_fract1);
	snd_soc_component_write(component, CE156_PLL_FRACT2, pll_fract2);
	snd_soc_component_write(component, CE156_PLL_FRACT3, pll_fract3);

	printk(KERN_INFO "exit %s, PLL1 = %x, PLL2 = %x, "
		 "FRAC1 = %x, FRAC2 = %x, FRAC3 = %x\n",
		 __func__, pll1, pll2, pll_fract1, pll_fract2, pll_fract3);
	return 0;
}

static int ce156_set_dai_sysclk(struct snd_soc_dai * codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	//struct snd_soc_component *component = codec_dai->component;
	//struct ce156_priv *ce156 = codec->private_data;
	printk(KERN_INFO "enter %s\nexit %s\n", __func__, __func__);
	pr_debug("enter %s\n", __func__);
	return 0;
}

static int ce156_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	//struct ce156_priv *ce156 = codec->private_data;

	// XXX update bits
	u8 iface = snd_soc_component_read32(component, CE156_I2S1) & 0xcf;
	int rate  = params_rate(params);
	int coeff = ce156_get_coeff(rate);

	printk(KERN_INFO "enter %s, rate is %d\n", __func__, rate);
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x10;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x20;
		break;
	}
	
	pr_debug("rate = %d, iface = %d\n", rate, iface);
	/* set iface */
	snd_soc_component_write(component, CE156_I2S1, iface);
	snd_soc_component_write(component, CE156_ADC_RATE, sample_rate_list[coeff].rate_i2s_pcm);

	printk(KERN_INFO "exit %s\n", __func__);

	return 0;
}

static int ce156_dev_init(struct snd_soc_component *component);

static int ce156_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 iface = 0;
	int ret = 0;	
	snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, 0x63);
/*	
	ret = ce156_dev_init(component);
	if (ret < 0) {
		dev_err(component->dev, "Failed to init codec\n");
		printk("Failed to init codec\n");
		return ret;
	}
*/
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x01;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface &= 0xfe;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface &= 0x3f;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface |= 0x40;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x80;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0xc0;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0xc2;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface &= 0xfb;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x04;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	pr_debug("enter %s, iface = %d\n", __func__, iface);
	snd_soc_component_write(component, CE156_I2S1, iface);

	printk(KERN_INFO "exit %s\n", __func__);
	return ret;
}

// XXX what
static int ce156_mute_state = 1;

static int ce156_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_component *component = dai->component;
	
	u8 mute_reg = snd_soc_component_read32(component, CE156_DAC_DWA_OFST) & 0xff;
	
	ce156_mute_state = mute;

	if (mute) {
		snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, 0x00);
		snd_soc_component_write(component, CE156_DAC_DWA_OFST, mute_reg | 0x44);
		
		printk(KERN_INFO "start %s\n", __func__);
	} else {
		snd_soc_component_write(component, CE156_DAC_DWA_OFST, mute_reg & 0xbb);	
		printk(KERN_INFO "stop %s\n", __func__);
	}

	ce156_sync(component);
	
	return 0;
}

#define CE156_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define CE156_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops ce156_dai_ops= {
	.hw_params	= ce156_hw_params,
	.digital_mute	= ce156_mute,
	.set_fmt	= ce156_set_dai_fmt,
	.set_sysclk	= ce156_set_dai_sysclk,
	.set_pll	= ce156_set_dai_pll,
};

struct snd_soc_dai_driver ce156_dai = {
	/* hifi codec dai */
	.name = "CE156",
	.id = 1,
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates	      = CE156_RATES,
		.formats      = CE156_FORMATS,
	},
	.capture = {
		.stream_name  = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = CE156_RATES,
		.formats      = CE156_FORMATS,
	},
	.ops = &ce156_dai_ops,
	.symmetric_rates = 1,
};

static int ce156_dev_init(struct snd_soc_component *component)
{
	int ret = 0;

	ce156_reset(component);
	mdelay(10);

        snd_soc_component_write(component, CE156_PLL1, 0xA5);//a4
        snd_soc_component_write(component, CE156_PLL2, 0x00);//00
        snd_soc_component_write(component, CE156_PLL_FRACT1, 0x08);
        snd_soc_component_write(component, CE156_PLL_FRACT2, 0x82);
	
	snd_soc_component_write(component, CE156_ADC_RATE, 0x9);
	snd_soc_component_write(component, CE156_I2S1, 0x0);
	snd_soc_component_write(component, CE156_I2S2, 0x0);
	snd_soc_component_write(component, CE156_ADC_RSVD, 0x2);
	snd_soc_component_write(component, CE156_ADC_ANA_ENABLE, 0x3f);
	snd_soc_component_write(component, CE156_ADC_DIG_ENABLE, 0x30);
	snd_soc_component_write(component, CE156_DAC_DIG_ENABLE, 0x39);
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa4);
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa0);
	snd_soc_component_write(component, CE156_HS_MIC_DET, 0x61);  //paul add for HP detect
	snd_soc_component_write(component, CE156_MIC_CTRL, 0x2);
	snd_soc_component_write(component, CE156_DAC_ANA_ENABLE, 0x63);
	snd_soc_component_write(component, CE156_HS_INPUT_SEL, 0x6);
	snd_soc_component_write(component, CE156_DAC_DWA_OFST, 0x0);
	snd_soc_component_write(component, CE156_DAC_GAINLL, 0x0);
	snd_soc_component_write(component, CE156_DAC_GAINRR, 0x0);
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa4);
	snd_soc_component_write(component, CE156_DAC_ANA_MISC, 0xa0);
	snd_soc_component_write(component, CE156_DAC_DWA, 0x40);/*by pass equalizer*/
	ce156_reg_init(component);
	snd_soc_component_write(component, CE156_STATUS2, 0x40);

	printk(KERN_INFO "ce156 reg write finished\n");

	ce156_dapm_event(component, SND_SOC_BIAS_PREPARE);

#ifndef CONFIG_MACH_QSEVEN
	snd_soc_add_component_controls(component, ce156_snd_controls,
			     ARRAY_SIZE(ce156_snd_controls));
	printk(KERN_INFO "controls added finished\n");
#endif

#if USE_DAPM_CTRL
	ce156_add_widgets(component);
#endif	

	printk(KERN_INFO "ce156: initial ok\n");
	
	return ret;

}


static int ce156_probe(struct snd_soc_component *component)
{
	struct ce156_private *ce156_priv = snd_soc_component_get_drvdata(component);
	int ret = 0;	

	/*ret = snd_soc_codec_set_cache_io(component, ce156_priv->control_data);
	if (ret < 0) {
		dev_err(component->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}*/

	ret = ce156_dev_init(component);
	if (ret < 0) {
		dev_err(component->dev, "Failed to Init\n");
		return ret;
	}

	ret = snd_soc_add_component_controls(component, ce156_snd_controls,
                                   ARRAY_SIZE(ce156_snd_controls));
	if( ret != 0 ) {
		printk("%s snd_soc_add_component_controls failed. ret = %d\n", __func__, ret);
	}
	
	return ret;
}

static void ce156_remove(struct snd_soc_component *component)
{
	ce156_dapm_event(component, SND_SOC_BIAS_OFF);
}

static int ce156_suspend(struct snd_soc_component *component)
{
	ce156_dapm_event(component, SND_SOC_BIAS_OFF);

	return 0;
}

static int ce156_resume(struct snd_soc_component *component)
{
	
	ce156_dapm_event(component, SND_SOC_BIAS_STANDBY);

	return 0;
}


struct snd_soc_component_driver soc_component_dev_ce156 = {
	.probe   = ce156_probe,
	.remove  = ce156_remove,
	.suspend = ce156_suspend,
	.resume  = ce156_resume,
	.set_bias_level = ce156_dapm_event,
};

static int ce156_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct ce156_private *ce156_priv;
	int ret;
	
	printk(KERN_INFO "CE156: register i2c driver successfully\n");
	
	ce156_priv = kzalloc(sizeof(struct ce156_private), GFP_KERNEL);
	if (ce156_priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ce156_priv);
	ce156_priv->control_data = i2c;

	ce156_priv->regmap = devm_regmap_init_i2c(i2c, &ce156_regmap);
	if (IS_ERR(ce156_priv->regmap)) {
		ret = PTR_ERR(ce156_priv->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap (%d)\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_component(&i2c->dev, &soc_component_dev_ce156, &ce156_dai, 1);
		
	if (ret) {
		printk(KERN_INFO "CE156: Failed to register codec\n");
		goto out;
	}
		
	return ret;
	
out:
	kfree(ce156_priv);
	return -EINVAL;
}


static int ce156_i2c_remove(struct i2c_client *client)
{
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id ce156_i2c_id[] = {
	{"ce156", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ce156_i2c_id);

static struct i2c_driver ce156_i2c_driver = {
	.driver = {
		.name  = "ce156",
		.owner = THIS_MODULE,
	},
	.probe    = ce156_i2c_probe,
	.remove   = ce156_i2c_remove,
	.id_table = ce156_i2c_id,
};

static int __init ce156_init(void)
{
	int ret = 0;
	printk("paul init and register ce156 i2c driver:\n");

// XXX driver
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&ce156_i2c_driver);
	if(ret != 0){
		printk( "Failed to register ce156 i2c driver: %d\n", 
			ret);
	}
#endif
	return ret;
}

static void __exit ce156_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&ce156_i2c_driver);
#endif
}

module_init(ce156_init);
module_exit(ce156_exit);

MODULE_DESCRIPTION("ASoc Marvell 88CE156 driver");
MODULE_AUTHOR("Inventec BU3A <webmaster@inventec.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ce156.2-0030");
