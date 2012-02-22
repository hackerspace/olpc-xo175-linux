/*
 * linux/mfd/max77601-core.h
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __LINUX_MAX77601_H__
#define __LINUX_MAX77601_H__

#include <linux/irq.h>
#include <linux/mfd/core.h>

/* Top level Interrupt */
enum {
	MAX77601_IRQTOP_START,
	MAX77601_IRQTOP_NVER = MAX77601_IRQTOP_START,
	MAX77601_IRQTOP_ONOFF,
	MAX77601_IRQTOP_32K,
	MAX77601_IRQTOP_RTC,
	MAX77601_IRQTOP_GPIO,
	MAX77601_IRQTOP_LDO,
	MAX77601_IRQTOP_SD,
	MAX77601_IRQTOP_GLBL,
	MAX77601_IRQTOP_END = MAX77601_IRQTOP_GLBL,

	MAX77601_ONOFFIRQ_START,
	MAX77601_ONOFFIRQ_ACOK_R = MAX77601_ONOFFIRQ_START,
	MAX77601_ONOFFIRQ_ACOK_F,
	MAX77601_ONOFFIRQ_LID_R,
	MAX77601_ONOFFIRQ_LID_F,
	MAX77601_ONOFFIRQ_EN0_R,
	MAX77601_ONOFFIRQ_EN0_F,
	MAX77601_ONOFFIRQ_EN0_1SEC,
	MAX77601_ONOFFIRQ_HRDPOWRN,
	MAX77601_ONOFFIRQ_END = MAX77601_ONOFFIRQ_HRDPOWRN,

	MAX77601_GPIOIRQ_START,
	MAX77601_GPIOIRQ_EDGE0 = MAX77601_GPIOIRQ_START,
	MAX77601_GPIOIRQ_EDGE1,
	MAX77601_GPIOIRQ_EDGE2,
	MAX77601_GPIOIRQ_EDGE3,
	MAX77601_GPIOIRQ_EDGE4,
	MAX77601_GPIOIRQ_EDGE5,
	MAX77601_GPIOIRQ_EDGE6,
	MAX77601_GPIOIRQ_EDGE7,
	MAX77601_GPIOIRQ_END = MAX77601_GPIOIRQ_EDGE7,

	MAX77601_INTLBT_START,
	MAX77601_INTLBT_TJALRM2_R = MAX77601_INTLBT_START,
	MAX77601_INTLBT_TJALRM1_R,
	MAX77601_INTLBT_MBATTLOW_R,
	MAX77601_INTLBT_END = MAX77601_INTLBT_MBATTLOW_R,
	MAX77601_MAX_IRQ,
};

#define MAX77601_IRQTOP_NR_INTS (MAX77601_IRQTOP_END - MAX77601_IRQTOP_START + 1)
#define MAX77601_ONOFFIRQ_NR_INTS (MAX77601_ONOFFIRQ_END - MAX77601_ONOFFIRQ_START + 1)
#define MAX77601_GPIOIRQ_NR_INTS (MAX77601_GPIOIRQ_END - MAX77601_GPIOIRQ_START + 1)
#define MAX77601_INTLBT_NR_INTS (MAX77601_INTLBT_END - MAX77601_INTLBT_START + 1)

/* IRQ */
#define MAX77601_IRQTOP_REG				0x05
#define MAX77601_IRQTOPM_REG			0x0D
#define MAX77601_IRQ_GLBL_MASK			0x80
#define MAX77601_IRQ_SD_MASK			0x40
#define MAX77601_IRQ_LDO_MASK			0x20
#define MAX77601_IRQ_GPIO_MASK			0x10
#define MAX77601_IRQ_RTC_MASK			0x08
#define MAX77601_IRQ_32K_MASK			0x04
#define MAX77601_IRQ_ONOFF_MASK			0x02
#define MAX77601_IRQ_NVER_MASK			0x01

#define MAX77601_INTLBT_REG				0x06
#define MAX77601_INTLBTM_REG			0x0E
#define MAX77601_INTLBT_LB_MASK			0x08
#define MAX77601_INTLBT_TJALRM1_MASK	0x04
#define MAX77601_INTLBT_TJALRM2_MASK	0x02
#define MAX77601_INTLBT_GLB_MASK		0x01

#define MAX77601_ONOFFIRQ_REG			0x0B
#define MAX77601_ONOFFIRQM_REG			0x12
#define MAX77601_ONOFFIRQ_EN0_RM		0x08
#define MAX77601_ONOFFIRQ_EN0_FM		0x04

/* Regulator */
enum {
	/* BUCK */
	MAX77601_ID_SD0 = 0,
	MAX77601_ID_DVSSD0,
	MAX77601_ID_SD1,
	MAX77601_ID_DVSSD1,
	MAX77601_ID_SD2,
	MAX77601_ID_SD3,
	MAX77601_ID_SD4,
	/* LDO */
	MAX77601_ID_L0 = 7,
	MAX77601_ID_L1,
	MAX77601_ID_L2,
	MAX77601_ID_L3,
	MAX77601_ID_L4,
	MAX77601_ID_L5,
	MAX77601_ID_L6,
	MAX77601_ID_L7,
	MAX77601_ID_L8,

	MAX77601_VREG_MAX,
};

/* FPS register */
#define MAX77601_CNFGFPS0		0x43
#define MAX77601_CNFGFPS1		0x44
#define MAX77601_CNFGFPS2		0x45
#define MAX77601_FPS_L0			0x46
#define MAX77601_FPS_L1			0x47
#define MAX77601_FPS_L2			0x48
#define MAX77601_FPS_L3			0x49
#define MAX77601_FPS_L4			0x4A
#define MAX77601_FPS_L5			0x4B
#define MAX77601_FPS_L6			0x4C
#define MAX77601_FPS_L7			0x4D
#define MAX77601_FPS_L8			0x4E
#define MAX77601_FPS_SD0		0x4F
#define MAX77601_FPS_SD1		0x50
#define MAX77601_FPS_SD2		0x51
#define MAX77601_FPS_SD3		0x52
#define MAX77601_FPS_SD4		0x53

#define MAX77601_FPSSRC_MASK	0xC0
#define MAX77601_FPSSRC_NOTFPS	(0x3 << 6)

/* DVS register */
#define MAX77601_AME_GPIO		0x40
#define MAX77601_AME5_MASK		(0x1 << 5)
#define MAX77601_AME4_MASK		(0x1 << 4)

/* ROV register */
#define MAX77601_CNFG2SD		0x22
#define MAX77601_ROV_EN_SD0		(0x1 << 2)


struct max77601_chip {
	struct device *dev;
	struct i2c_client *i2c;
	struct i2c_client *rtc;
	struct mutex io_lock;
	struct mutex irq_lock;

	int irq_base;
	int core_irq;
};

struct max77601_platform_data {
	struct regulator_init_data *regulator;
	int irq_base;
	int (*setup) (struct max77601_chip *);
};



int max77601_read(struct max77601_chip *chip, u8 addr, u8 * values,
		  unsigned int len);
int max77601_write(struct max77601_chip *chip, u8 addr, u8 * values,
		   unsigned int len);
int max77601_set_bits(struct max77601_chip *chip, u8 addr, u8 mask,
		      u8 value);

int max77601_pmic_reg_write(int reg, unsigned char data);
int max77601_pmic_reg_read(int reg);
int max77601_pmic_set_bits(int reg, unsigned char mask,
			   unsigned char data);

extern int max77601_device_init(struct max77601_chip *chip,
				struct max77601_platform_data *pdata);
extern void max77601_device_exit(struct max77601_chip *chip);

#endif				/* __LINUX_MAX77601_H__ */
