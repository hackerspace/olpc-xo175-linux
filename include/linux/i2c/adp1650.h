/*
 * Definitions and platform data for Analog Devices
 * LED/flash drivers ADP1650
 *
 * Copyright (C) 2012 Marvell Internation Ltd.
 *
 * Bin Zhou <zhoub@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef ____LINUX_I2C_ADP1650_H
#define ____LINUX_I2C_ADP1650_H

#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>

/*
0x00 design information
0x01 unknown
0x02 verf & timer
0x03 current set
0x04 output mode
0x05 fault information
0x06 input control
0x07 additional mode: AD_MOD
0x08 additional mode: ADC
0x09 battery low mode
*/
enum adp1650_reg_def {
	REG_DESIGN_INFO	= 0x00,
		DI_DEFAULT_VAL		= 0x22,
	REG_UNKNOWN	= 0x01,
	REG_VERF_TIMER	= 0x02,
		GPIO1_CONFIG_TORCH	= (1<<4),
		GPIO1_CONFIG_MASK	= (3<<4),
	REG_CURRENT_SET	= 0x03,
	REG_OUTPUT_MODE	= 0x04,
		OUTPUT_ENABLE		= (1<<3),
		OUTPUT_MODE_STANDBY	= (0<<0),
		OUTPUT_MODE_FLASH	= (3<<0),
		OUTPUT_MODE_MASK	= (OUTPUT_ENABLE | OUTPUT_MODE_FLASH),
	REG_FAULT_INFO	= 0x05,
	REG_INPUT_CTL	= 0x06,
	REG_AD_MOD	= 0x07,
		DYNAMIC_OVP_ON		= (1<<7),
	REG_ADC		= 0x08,
	REG_BATTERY_LOW	= 0x09,
};

enum adp1650_mode {
	MODE_STANDBY = '0',
	MODE_TORCH,
	MODE_FLASH,
};

struct adp1650_platform_data {
	u8 torch_is_on;
	u8 strobe_enable;
	int(*torch_enable)(bool);
};

struct adp1650 {
	struct i2c_client *client;
	struct adp1650_platform_data *pdata;
	struct mutex lock;
};

#endif /* ____LINUX_I2C_ADP1650_H */

