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

enum adp1650_mode {
	MODE_TORCH_OFF = '0',
	MODE_TORCH_ON,
	MODE_SET_FLASH_MODE,
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

