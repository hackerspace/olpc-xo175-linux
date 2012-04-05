/*
 * Marvell 88PM8xxx general
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * Joseph(Yossi) Hanin <yhanin@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_88PMXXX_H
#define __LINUX_MFD_88PMXXX_H

#define MFD_NAME_SIZE		(40)
#define PM8XXX_VERSION_MASK		(0xFF)	/* 8XXX chip ID mask */

enum {
	CHIP_INVALID = 0,
	CHIP_PM8607,
	CHIP_PM8606,
	CHIP_PM800,
	CHIP_PM805,
	CHIP_MAX,
};

enum {
	/* Levante */
	PM8607_CHIP_A0 = 0x40,
	PM8607_CHIP_A1 = 0x41,
	PM8607_CHIP_B0 = 0x48,
	PM8607_CHIP_C0 = 0x50,
	PM8607_CHIP_C1 = 0x51,
	PM8607_CHIP_D0 = 0x58,
	PM8607_CHIP_D1 = 0x59,
	PM8607_CHIP_END = PM8607_CHIP_D1,

	/* Procida */
	PM800_CHIP_A0  = 0x60,
	PM800_CHIP_A1  = 0x61,
	PM800_CHIP_B0  = 0x62,
	PM800_CHIP_C0  = 0x63,
	PM800_CHIP_END = PM800_CHIP_C0,

	/* Make sure to update this to the last stepping */
	PM8XXX_CHIP_END = PM800_CHIP_END
};

enum enum_result {
	ENUMERATION_START	= 0,
	ENUMERATION_500MA,
};

enum {
	GI2C_PORT = 0,
	PI2C_PORT,
};

enum {
	PM8XXX_GPIO1_SUPPLY_VBUS = 1,
	PM8XXX_GPIO2_SUPPLY_VBUS = 2,
	PM8XXX_GPIO3_SUPPLY_VBUS = 3,
	PM8XXX_GPIO4_SUPPLY_VBUS = 4,
};

enum {
	PM8XXX_IDPIN_NO_USE = 0,
	PM8XXX_IDPIN_USE_GPADC0,
	PM8XXX_IDPIN_USE_GPADC1,
	PM8XXX_IDPIN_USE_GPADC2,
	PM8XXX_IDPIN_USE_GPADC3,
};

#define get_pmic_version(chip) (*(unsigned char *) chip)
extern void pm8xxx_system_poweroff(void);
extern void pm8xxx_system_restart(void);

#endif /* __LINUX_MFD_88PMXXX_H */
