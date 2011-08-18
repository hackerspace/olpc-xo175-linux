/*
 * drivers/switch/switch_headset.h
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#ifndef _SWITCH_HEADSET_H
#define _SWITCH_HEADSET_H

extern int wm8994_headset_detect(void);

struct headset_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	int invert;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
};

#endif
