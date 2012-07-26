/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __LINUX_88PM867_H__
#define __LINUX_88PM867_H__

/* Regulator */
enum {
	/* BUCK */
	MAR88PM867_ID_SD0 = 0,
	MAR88PM867_ID_SD1,
	MAR88PM867_VREG_MAX,
};

/* Regulator registers */
#define MAR88PM867_VREG_SD0		0x24
#define MAR88PM867_VREG_SD0_SLEEP	0x10
#define MAR88PM867_VREG_SD0_CFG		0x11

#define MAR88PM867_VREG_SD1		0x13
#define MAR88PM867_VREG_SD1_SLEEP	0x12
#define MAR88PM867_VREG_SD1_CFG		0x14

#define MAR88PM867_MODE_NORMAL		0x1
#define MAR88PM867_MODE_SLEEP		0x2
#define MAR88PM867_MODE_DISABLE		0x0

#define MAR88PM867_ENABLE_REG		0x03
#define MAR88PM867_SYS_CTRL_REG		0x02
#define MAR88PM867_SYS_STATUS_REG	0x01

#define MAR88PM867_VREG_TYPE_SD		0x00
#define MAR88PM867_VREG_TYPE_LDO	0x01

struct mar88pm867_chip {
	struct device *dev;
	struct i2c_client *i2c;
	struct mutex io_lock;
};

struct mar88pm867_platform_data {
	struct regulator_init_data *regulator;
	int buck_id;
};

#endif				/* __LINUX_MAX77601_H__ */
