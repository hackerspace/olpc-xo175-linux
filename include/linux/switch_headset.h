/*
 * include/linux/switch_headset.h
 * Headset Detection Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SWITCH_HEADSET_H__
#define __SWITCH_HEADSET_H__

struct switch_headset_pdata {
	char *name;
	int gpio;
	bool invert;
	/* if NULL, "0" or "1" will be printed */
	const char *state_on;
	const char *state_off;
};

extern int (*headset_detect_func)(void);
extern int (*get_mic_state)(void);

#endif	/* __SWITCH_HEADSET_H__ */
