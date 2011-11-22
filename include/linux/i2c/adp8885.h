/*
 * Definitions and platform data for Analog Devices
 * Backlight drivers ADP8885
 *
 * Copyright 2011 Marvell
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __LINUX_I2C_ADP8885_H
#define __LINUX_I2C_ADP8885_H

#include <linux/types.h>

#define ADP8885_FSW_600KHZ      0
#define ADP8885_FSW_1200KHZ     1

#define ADP8885_PWS_ENABLE      1
#define ADP8885_PWS_DISABLE     0

#define ADP8885_IPEAK_500mA     0
#define ADP8885_IPEAK_600mA     1
#define ADP8885_IPEAK_700mA     2
#define ADP8885_IPEAK_850mA     3

#define ADP8885_SOVP_ENABLE     1
#define ADP8885_SOVP_DISABLE    0

#define ADP8885_FB_ENABLE       0
#define ADP8885_FB_DISABLE      1

#define ADP8885_DIM_ENABLE      1
#define ADP8885_DIM_DISABLE     0

/* Output current in percents */
#define ADP8885_ISET_0          0
#define ADP8885_ISET_10         81
#define ADP8885_ISET_20         114
#define ADP8885_ISET_30         140
#define ADP8885_ISET_40         161
#define ADP8885_ISET_50         180
#define ADP8885_ISET_60         198
#define ADP8885_ISET_70         214
#define ADP8885_ISET_80         228
#define ADP8885_ISET_90         242
#define ADP8885_ISET_100        255
#define ADP8885_MAX_BRIGHTNESS  ADP8885_ISET_100

#define ADP8885_FADE_2ms        0
#define ADP8885_FADE_260ms      1
#define ADP8885_FADE_510ms      2
#define ADP8885_FADE_760ms      3
#define ADP8885_FADE_1020ms     4
#define ADP8885_FADE_1260ms     5
#define ADP8885_FADE_1530ms     6
#define ADP8885_FADE_1780ms     7
#define ADP8885_FADE_2040ms     8
#define ADP8885_FADE_2300ms     9
#define ADP8885_FADE_2550ms     10
#define ADP8885_FADE_2800ms     11
#define ADP8885_FADE_3060ms     12
#define ADP8885_FADE_3320ms     13
#define ADP8885_FADE_3570ms     14
#define ADP8885_FADE_3820ms     15

#define ADP8885_OVP_LVL_25V     0
#define ADP8885_OVP_LVL_28V     1

#define ADP8885_OVR_ENABLE      1
#define ADP8885_OVR_DISABLE     0

/* Backlight maximum current 0..25600uA */
#define ADP8885_BL_MAX_CUR_uA(I)	((I * 255) / 25600)

#define MAX_CHAN_NUM		2

struct adp8885_bl_channel_data {
	u8 sovp;		/* Soft OVP mode enable */
	u8 fb_dis;		/* Disable Feedback */
	u8 dim_en;		/* enable DIM */
	u8 imax;		/* Maximum output current */
	u8 iset;		/* Output current */
	u8 fade_out;		/* Fade out rate */
	u8 fade_in;		/* Fade in rate */
	u8 iovr;		/* Output current override */
};

struct adp8885_bl_platform_data {
	u8 psm_en;		/* power save mode enable */
	u8 fsw;			/* bost power stage frequency */
	u8 ipeak;		/* Minimum inductor peak current */
	u8 ovp_lvl;		/* Over-voltage protection */
	u8 ledout_h;		/* Led outovr current MSB */
	u8 ledout_l;		/* Led outovr current LSB */
	u8 num_chs;
	struct adp8885_bl_channel_data *ch;
	int(*chip_enable)(bool);
};

#endif /* __LINUX_I2C_ADP8885_H */
