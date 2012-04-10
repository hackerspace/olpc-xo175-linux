/*
 * include/linux/power/fan4010_battery.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __FAN4010_BATTERY_H__
#define __FAN4010_BATTERY_H__

struct fan4010_battery_pdata {
	unsigned int bat_design_cap;
	unsigned int interval;
	unsigned int r_bat;
	unsigned int r_sns;
	unsigned int r_vout;
	int (*get_bat_vol)(void);
	int (*get_sys_cur_vol)(void);
	int (*get_bat_state)(void);
};

#endif
