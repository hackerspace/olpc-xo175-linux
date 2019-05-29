/*
 * Maxim 8900A, 8900B, 8900C charger driver.
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX8900_CHARGER_H__
#define __MAX8900_CHARGER_H__

/* Max8900 type */
enum max8900_type {
	MAX8900A_TYPE = 0,
	MAX8900B_TYPE,
	MAX8900C_TYPE,
};

/* Max8900 charger type */
enum max8900_charger_type {
	MAX8900_UNDEFINED_CHARGER = 0,
	MAX8900_USB_CHARGER,
	MAX8900_AC_CHARGER,
};

/* Max8900 charge mode */
enum max8900_charge_mode {
	MAX8900_DISABLED_CHARGE_MODE = 0,
	MAX8900_NORMAL_CHARGE_MODE,
	MAX8900_FAST_CHARGE_MODE,
};

/* Max8900 state */
enum max8900_state {
	MAX8900_UNDEFINED_STATE = 0,
	MAX8900_CHARGER_DISABLED_STATE,
	MAX8900_CHARGING_STATE,
	MAX8900_DONE_STATE,
	MAX8900_BATTERY_COLD_STATE,
	MAX8900_BATTERY_HOT_STATE,
	MAX8900_TIMER_FAULT_STATE,
	MAX8900_VIN_HIGH_STATE,
	MAX8900_UNSPEC_FAULT_STATE,
};

struct max8900_pdata {
	enum max8900_type type;	/* Max8900 HW type - A/B/C	*/

	/* GPIO configuration */
	unsigned char gpio_cen;	/* Charger enable input	*/
	unsigned char gpio_seti;	/* Fast charge enable input	*/
	unsigned char gpio_stat1;	/* Charger status output 1	*/
	unsigned char gpio_stat2;	/* Charger status output 2	*/
	unsigned char gpio_stat3;	/* Charger status output 3	*/

	/* Charger state changed notification call back */
	void (*notify_state_changed)(enum max8900_state state);
};

extern int max8900_set_charger_online(bool online);
extern int max8900_set_charger_type(enum max8900_charger_type type);
extern int max8900_set_charge_mode(enum max8900_charge_mode mode);

#endif		/* __MAX8900_CHARGER_H__ */
