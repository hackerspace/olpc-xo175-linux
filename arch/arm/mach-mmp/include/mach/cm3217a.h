/*
 * linux/arch/arm/mach-mmp/include/mach/axis_sensor.h
 *
 *  Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_MACH_CM3217A_H
#define __ASM_MACH_CM3217A_H

struct cm3217a_platform_data {
	int     (*set_power)(int);
};

#endif
