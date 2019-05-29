/*
 * include/linux/power/isl9519.h
 * Battery Charger ISL9519
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * Yunfan Zhang <yfzhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LINUX_POWER_ISL9519_H
#define _LINUX_POWER_ISL9519_H

struct isl9519_charger_pdata {
	u16 max_sys_vol;
	u16 min_sys_vol;
	u16 chg_cur;
	u16 input_cur;
	int stay_awake_en;
	int update_interval;

	char **supplied_to;
	size_t num_supplicants;
};

#endif
