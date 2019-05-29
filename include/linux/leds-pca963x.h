/*
 * pca963x.h - platform data structure for pca963x LED controllers
 *
 * Copyright (C) 2011 David Sayada <dsayada@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheets:
 *
 * PCA9632: http://www.nxp.com/documents/data_sheet/PCA9632.pdf
 * PCA9633: http://www.nxp.com/documents/data_sheet/PCA9633.pdf
 * PCA9634: http://www.nxp.com/documents/data_sheet/PCA9634.pdf
 * PCA9635: http://www.nxp.com/documents/data_sheet/PCA9635.pdf
 *
 */

#ifndef __LINUX_PCA963X_H
#define __LINUX_PCA963X_H

#include <linux/leds.h>
#include <linux/workqueue.h>

/* pca963x registers definitions */

#define PCA963X_MODE1			(0x00)
#define PCA963X_MODE2			(0x01)
#define PCA963X_PWM(led_num)		(0x02 + led_num)
#define PCA963X_GRPPWM			(PCA963X_PWM(chip->num_leds) + 0)
#define PCA963X_GRPFREQ			(PCA963X_PWM(chip->num_leds) + 1)
#define PCA963X_LEDOUT(led_num)		(PCA963X_PWM(chip->num_leds) + 2 + \
					(led_num/4))
#define PCA963X_SUBADR1			(PCA963X_LEDOUT(chip->num_leds) + 0)
#define PCA963X_SUBADR2			(PCA963X_LEDOUT(chip->num_leds) + 1)
#define PCA963X_SUBADR3			(PCA963X_LEDOUT(chip->num_leds) + 2)
#define PCA963X_ALLCALLADR		(PCA963X_LEDOUT(chip->num_leds) + 3)

/* pca963x MODE1 register shifts and masks */

#define PCA963X_SLEEP_SHIFT		(4)
#define PCA963X_SLEEP_MASK		(1 << PCA963X_SLEEP_SHIFT)
#define PCA963X_SUB1			(1 << 3)
#define PCA963X_SUB2			(1 << 2)
#define PCA963X_SUB3			(1 << 1)
#define PCA963X_ALLCALL			(1 << 0)

#define PCA963X_ADR_MASK		(0xfe)

/* pca963x MODE2 register shifts and masks */

#define PCA963X_DMBLNK_SHIFT		(5)
#define PCA963X_DMBLNK_MASK		(1 << PCA963X_DMBLNK_SHIFT)
#define PCA963X_INVRT			(1 << 4)
#define PCA963X_OCH			(1 << 3)
#define PCA963X_OUTDRV			(1 << 2)

#define PCA963X_IS_BLINKING()		((mode2 & PCA963X_DMBLNK_MASK) >> \
					PCA963X_DMBLNK_SHIFT)

/* pca963x LEDOUT register shifts and masks */

#define PCA963X_LDR3_SHIFT		(6)
#define PCA963X_LDR3_MASK		(3 << PCA963X_LDR3_SHIFT)
#define PCA963X_LDR2_SHIFT		(4)
#define PCA963X_LDR2_MASK		(3 << PCA963X_LDR2_SHIFT)
#define PCA963X_LDR1_SHIFT		(2)
#define PCA963X_LDR1_MASK		(3 << PCA963X_LDR1_SHIFT)
#define PCA963X_LDR0_SHIFT		(0)
#define PCA963X_LDR0_MASK		(3 << PCA963X_LDR0_SHIFT)

#define PCA93X_LED_NUM			(led_num & 3)

/* pca963x enable/disable */

#define PCA963X_DISABLE			(0)
#define PCA963X_ENABLE			(1)

/* pca963x power mode */

#define PCA963X_NORMAL			(0)
#define PCA963X_LOW_POWER		(1)

/* pca963x duty cycle */

#define PCA963X_93_75_PERCENT_DC	(15)
#define PCA963X_98_4_PERCENT_DC		(63)
#define PCA963X_99_6_PERCENT_DC		(255)
#define PCA963X_16_STEPS_DC_MASK	(0xfe)
#define PCA963X_64_STEPS_DC_MASK	(0xfc)
#define PCA963X_256_STEPS_DC_MASK	(0xff)

/* pca963x frequency */

#define PCA963X_6_HZ_FREQ		(4)
#define PCA963X_256_STEPS_FREQ_MASK	(0xff)

/* Maximum number of LEDS supported by pca963x LED controllers */

#define PCA963X_MAX_LEDS_NUMBER		(16)

/* Enumeration of all possible LED states */

enum pca963x_led_state {
	PCA963X_LED_OFF = 0x0,	/* OFF */
	PCA963X_LED_ON = 0x1,	/* Fully ON */
	PCA963X_LED_PWM = 0x2,	/* Individual brightness */
	PCA963X_LED_PWM_GRPPWM = 0x3	/* Individual brightness and group */
};

/* LED platform data structure definition */

struct pca963x_led_platform_data {
	char *name;
	enum pca963x_led_state state;
};

/* pca963x controller platform data structure definition */

struct pca963x_platform_data {
	u8 num_leds;		/* number of LEDS for current pca963x LED controller */
	struct pca963x_led_platform_data leds[PCA963X_MAX_LEDS_NUMBER];

	u8 sub1;		/* sub1 I2C subaddress [7:1] and enable/disable [0] */
	u8 sub2;		/* sub2 I2C subaddress [7:1] and enable/disable [0] */
	u8 sub3;		/* sub3 I2C subaddress [7:1] and enable/disable [0] */
	u8 allcall;		/* allcall I2C address [7:1] and enable/disable [0] */
	u8 props;		/* not_invrt/invrt [0], on_stop/on_ack [1] */
					/* open_drain/totem_pole [2] */
};

#endif /* __LINUX_PCA963X_H */
