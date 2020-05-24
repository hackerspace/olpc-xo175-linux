/*
 *  88ce156.h  --  audio driver for 88ce156
 *
 *  Copyright 2011  Marvell International Ltd.
 *
 *  All right reserved
 * 
 */

#ifndef _CE156_H
#define _CE156_H

#define ce156_CACHEREGNUM	72 
#define CE156_RESET	0x0	// ?

#define	CE156_ADC_PATH		0x00	/* Configuration parameters for ADC audio path */
#define	CE156_ADC_RATE		0x01	/* Configuration parameters for ADC and DAC audio path */
#define	CE156_ADCL_ATTN	0x02	/* Left channel ADC attenuation control */
#define	CE156_ADCR_ATTN	0x03	/* Right channel ADC attenuation control */
#define CE156_CHARGEPUMP_REG	0x05	/* undocumented reg */	
#define	CE156_I2S1		0x06	/* Configuration of i2S/PCM interface */
#define	CE156_I2S2		0x07	/* Configuration parameters for the I2S interface and DAC audio path */
#define	CE156_DAC_DWA		0x08
#define	CE156_DACEQUL_N0LO	0x09
#define	CE156_DACEQUL_N0HI	0x0A
#define	CE156_DACEQUL_N1LO	0x0B
#define	CE156_DACEQUL_N1HI	0x0C
#define	CE156_DACEQUL_D1LO	0x0D
#define	CE156_DACEQUL_D1HI	0x0E
#define	CE156_DAC_GAINLL	0x0F
#define	CE156_DAC_GAINRR	0x12
#define	CE156_DAC_DWA_OFST	0x13
#define	CE156_PLL1		0x18
#define	CE156_PLL2		0x19
#define	CE156_PLL_FRACT1	0x1A
#define	CE156_PLL_FRACT2	0x1B
#define	CE156_PLL_FRACT3	0x1C
#define	CE156_CLKGEN1		0x1D	/* undocumented reg */
#define	CE156_CLKGEN2		0x1E
#define	CE156_MIC_CTRL		0x20
#define	CE156_MIC1_PGA_GAIN	0x21
#define	CE156_MIC2_PGA_GAIN	0x22
#define	CE156_ADC1_PGA_GAIN	0x23
#define	CE156_ADC2_PGA_GAIN	0x24
#define	CE156_ADC_RSVD		0x25	/* undocumented reg */
#define	CE156_ANALOG_PATH_SEL	0x26	/* Analog settings: analog to analog path */
#define	CE156_DAC_HS1_CTRL	0x27
#define	CE156_DAC_HS2_CTRL	0x28
#define	CE156_DAC_SPKR_CTRL	0x29
#define	CE156_DAC_ANA_MISC	0x2A
#define	CE156_AUD_PWR_ENABLE	0x2C
#define	CE156_ADC_ANA_ENABLE	0x2D
#define	CE156_ADC_DIG_ENABLE	0x2E
#define	CE156_DAC_ANA_ENABLE	0x2F
#define	CE156_DAC_DIG_ENABLE	0x30
#define	CE156_HS_INPUT_SEL	0x36
#define	CE156_SPK_INPUT_SEL	0x37
#define	CE156_HS_MIC_DET	0x38
#define	CE156_STATUS1		0x39
#define	CE156_STATUS2		0x3A
#define CE156_REVISION		0x48
#define CE156_CLKEN		0x49

#define SAMPLE_RATE_8000 0
#define SAMPLE_RATE_12000 1
#define SAMPLE_RATE_16000 2
#define SAMPLE_RATE_24000 3
#define SAMPLE_RATE_32000 4
#define SAMPLE_RATE_48000 5
#define SAMPLE_RATE_96000 6
#define SAMPLE_RATE_11025 7
#define SAMPLE_RATE_22050 8
#define SAMPLE_RATE_44100 9
#define SAMPLE_RATE_88200 10

/*struct ce156_setup_data {
	int i2c_address;
	int i2c_bus;
	unsigned detect_hp_gpio;
};*/

#endif
