/*
 * Definitions and platform data for Rohm Devices
 * LED/flash drivers bd7704mux
 *
 * Copyright (C) 2012 Marvell Internation Ltd.
 *
 * Bin Zhou <zhoub@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef ____LINUX_I2C_BD7704_H
#define ____LINUX_I2C_BD7704_H

#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>

/*
0x1 Flash/Torch/Indicate control & TEN function set
0x2 Flash current LED1 setup
0x3 Flash current LED2 setup
0x4 Torch current ratio setup
0x5 Flash timer setup
0x6 Torch/Flash current ratio: [max. torch current] = [max flash current] * ratio
0x7 Coil current limit
0x8 Low battery detect setup
*/
enum bd7704_reg_def {
	REG_MODE_SET	= 0x1,
		DATA_POWER_OFF		= 1,
		DATA_TORCH_ON_LED1,
		DATA_TORCH_ON_LED2,
		DATA_TORCH_ON_BOTH,
		DATA_FLASH_ON,
		DATA_TSEL0,	/* TEN = Torch enable */
		DATA_TSEL1,	/* TEN = Flash mask signal */
	REG_FLASH_CURRENT_LED1	= 0x2,
	REG_FLASH_CURRENT_LED2	= 0x3,
		/* LED1 and LED2 share the same current setup data */
		DATA_FLASH_CURRENT_FULL	= 1,
		DATA_FLASH_CURRENT_90P	= 4,
		DATA_FLASH_CURRENT_80P	= 7,
		DATA_FLASH_CURRENT_70P	= 10,
		DATA_FLASH_CURRENT_60P	= 13,
		DATA_FLASH_CURRENT_50P	= 16,
	REG_TORCH_RATIO	= 0x4,
		DATA_TORCH_CURRENT_FULL	= 1,
		DATA_TORCH_CURRENT_80P	= 4,
		DATA_TORCH_CURRENT_60P	= 7,
		DATA_TORCH_CURRENT_40P	= 10,
		DATA_TORCH_CURRENT_20P	= 13,
		DATA_TORCH_CURRENT_0P	= 16,
	REG_FLASH_TIMER	= 0x5,
		DATA_FLASH_TIMER_10MS	= 1,
		DATA_FLASH_TIMER_20MS,
		DATA_FLASH_TIMER_40MS,
		DATA_FLASH_TIMER_60MS,
		DATA_FLASH_TIMER_80MS,
		DATA_FLASH_TIMER_100MS,
		DATA_FLASH_TIMER_200MS,
		DATA_FLASH_TIMER_300MS,
		DATA_FLASH_TIMER_400MS,
		DATA_FLASH_TIMER_500MS,
		DATA_FLASH_TIMER_600MS,
		DATA_FLASH_TIMER_700MS,
		DATA_FLASH_TIMER_800MS,
		DATA_FLASH_TIMER_900MS,
		DATA_FLASH_TIMER_1000MS,
		DATA_FLASH_TIMER_OFF,
	REG_TORCH_FLASH_CURRENT_RATIO	= 0x6,
	REG_COIL_CURRENT_LIMIT		= 0x7,
	REG_LOW_BATT_DETECT		= 0x8,
};

enum bd7704_cmd {
	BD7704_CMD_MODE_STANDBY = '0',
	BD7704_CMD_MODE_TORCH,
	BD7704_CMD_MODE_FLASH,
	BD7704_CMD_SENSOR_CONTROL,
	BD7704_CMD_BUTTON_CONTROL,
	BD7704_CMD_QUERY,
};

/* unit: us */
#define TMAX_EN_TIME		(100)
#define TMAX_ACCESS_READY	(500)
#define TMAX_LATCH_TIME		(1000)

/* default_control decide which way of control has high priority.
sensor control:
means the operations performed by v4l2 interface from a camera sensor request.
LED flash chip can work under both torch and flash modes.
from button:
means the operations performed by "echo x > bd7704", triggered by sw/hw button.
only torch mode is meaningful if LED flash chip drives the LED as a torch.

by default the sensor control has high priority.
user can echo 6 > bd7704 to reverse the priority.
*/
struct bd7704_platform_data {
	u8 default_control;
	u8 current_control;
	int(*upic_control)(bool);
};

struct bd7704 {
	struct bd7704_platform_data *pdata;
	struct mutex lock;
};

#endif /* ____LINUX_I2C_BD7704_H */

