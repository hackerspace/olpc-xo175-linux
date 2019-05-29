/*
 * max8903_charger.h - Maxim 8903 USB/Adapter Charger Driver
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 * Copyright (C) 2011 Marvell Technology Ltd.
 * Yunfan Zhang <yfzhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __MAX8903_CHARGER_H__
#define __MAX8903_CHARGER_H__

/* Max Charging Current Limitation Map:
 * -------------------------------------------------
 * CHARGING |   DCM   |   IUSB  |  USUS   |  CEN_N
 * ---------|---------|---------|-------------------
 *   2A     |    1    |    x    |    x    |    0
 * ---------|---------|---------|-------------------
 *  500MA   |    0    |    1    |    0    |    0
 * ---------|---------|---------|-------------------
 *  100MA   |    0    |    0    |    0    |    0
 * ---------|---------|---------|-------------------
 * DISABLED |    0    |    x    |    1    |    0
 * ---------|---------|---------|-------------------
 * DISABLED |    x    |    x    |    x    |    1
 * -------------------------------------------------
 */
enum {
	MAX8903_PIN_UNKNOWN = -1,
	/* Input Pins(to charger) */
	MAX8903_PIN_DCM = 0,	/* DC Current-Limit Mode Setting */
	MAX8903_PIN_IUSB,	/* USB Current-Limit Setting */
	MAX8903_PIN_USUS,	/* USB Suspend */
	MAX8903_PIN_CEN_N,	/* Charger Enable */
	/* Output Pins(from charger) */
	MAX8903_PIN_DOK_N,	/* DC Power-OK */
	MAX8903_PIN_UOK_N,	/* USB Power-OK */
	MAX8903_PIN_CHG_N,	/* Charing */
	MAX8903_PIN_FLT_N,	/* Fault */
	MAX8903_PIN_END,	/* End of PIN definition */
};

struct max8903_gpio {
	int id;
	int gpio;
	bool active_low;
	const char *desc;
};

struct max8903_pdata {
	struct max8903_gpio *gpios;
	unsigned int gpio_nums;
	bool dc_valid;	/* DC is wired */
	bool usb_valid;	/* USB is wired */

	char **supplied_to;
	size_t num_supplicants;
};

#endif /* __MAX8903_CHARGER_H__ */
