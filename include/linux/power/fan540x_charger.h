/*
 * include/linux/power/fan540x_charger.h
 * Battery Fuel Gauge MAX17042
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __FAN540x_CHARGER_H__
#define __FAN540x_CHARGER_H__

struct fan540x_charger_pdata {
	int monitor_interval;

	char **supplied_to;
	size_t num_supplicants;

	int voreg;

	int gpio_dis;
	bool gpio_dis_active_low;
};

#ifdef CONFIG_CHARGER_FAN540X
int fan540x_set_vbus(int on);
#else
static inline int fan540x_set_vbus(int on)
{
	pr_err("NOP: %s is not supported!\n", __func__);
	return 0;
}
#endif	/* CONFIG_CHARGER_FAN540X */

#endif
