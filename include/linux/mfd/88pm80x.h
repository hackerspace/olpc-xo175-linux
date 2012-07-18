/*
 * Marvell 88PM80x Interface
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * Joseph (Yossi) Hanin <yhanin@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_88PM80X_H
#define __LINUX_MFD_88PM80X_H

#include <linux/interrupt.h>
#include <linux/mfd/88pm8xxx.h>

enum {
	PM800_ID_INVALID,
	PM800_ID_VIBRATOR,
	PM800_ID_SOUND,
	PM800_ID_MAX,
};

enum {
	PM800_ID_BUCK1 = 0,
	PM800_ID_BUCK2,
	PM800_ID_BUCK3,
	PM800_ID_BUCK4,
	PM800_ID_BUCK5,

	PM800_ID_LDO1,
	PM800_ID_LDO2,
	PM800_ID_LDO3,
	PM800_ID_LDO4,
	PM800_ID_LDO5,
	PM800_ID_LDO6,
	PM800_ID_LDO7,
	PM800_ID_LDO8,
	PM800_ID_LDO9,
	PM800_ID_LDO10,
	PM800_ID_LDO11,
	PM800_ID_LDO12,
	PM800_ID_LDO13,
	PM800_ID_LDO14,
	PM800_ID_LDO15,
	PM800_ID_LDO16,
	PM800_ID_LDO17,
	PM800_ID_LDO18,
	PM800_ID_LDO19,

	PM800_ID_RG_MAX,
};
#define PM800_MAX_REGULATOR	PM800_ID_RG_MAX	/* 5 Bucks, 19 LDOs */
#define PM800_NUM_BUCK (5)	/*5 Bucks */
#define PM800_NUM_LDO (19)	/*19 Bucks */

/* 88PM805 Registers */
#define PM805_CHIP_ID			(0x00)

/*Audio*/

/*88PM800 registers*/
enum {
	PM80X_INVALID_PAGE = 0,
	PM80X_BASE_PAGE,
	PM80X_POWER_PAGE,
	PM80X_GPADC_PAGE,
	PM80X_TEST_PAGE,
};
/*********************************/
/*page 0 basic: slave adder 0x60*/
/*********************************/
/* Interrupt Registers */
#define PM800_CHIP_ID			(0x00)

#define PM800_STATUS_1			(0x01)
#define PM800_ONKEY_STS1		(1 << 0)
#define PM800_EXTON_STS1		(1 << 1)
#define PM800_CHG_STS1			(1 << 2)
#define PM800_BAT_STS1			(1 << 3)
#define PM800_VBUS_STS1			(1 << 4)
#define PM800_LDO_PGOOD_STS1	(1 << 5)
#define PM800_BUCK_PGOOD_STS1	(1 << 6)

#define PM800_STATUS_2			(0x02)
#define PM800_RTC_ALARM_STS2	(1 << 0)
#define PM800_GPADC1_MEAS_EN	(1 << 3)

#define PM800_INT_STATUS1		(0x05)
#define PM800_ONKEY_INT_STS1		(1 << 0)
#define PM800_EXTON_INT_STS1		(1 << 1)
#define PM800_CHG_INT_STS1			(1 << 2)
#define PM800_BAT_INT_STS1			(1 << 3)
#define PM800_RTC_INT_STS1			(1 << 4)
#define PM800_CLASSD_OC_INT_STS1	(1 << 5)

#define PM800_INT_STATUS2		(0x06)
#define PM800_VBAT_INT_STS2		(1 << 0)
#define PM800_VSYS_INT_STS2		(1 << 1)
#define PM800_VCHG_INT_STS2		(1 << 2)
#define PM800_TINT_INT_STS2		(1 << 3)
#define PM800_GPADC0_INT_STS2	(1 << 4)
#define PM800_TBAT_INT_STS2		(1 << 5)
#define PM800_GPADC2_INT_STS2	(1 << 6)
#define PM800_GPADC3_INT_STS2	(1 << 7)

#define PM800_INT_STATUS3		(0x07)

#define PM800_INT_STATUS4		(0x08)
#define PM800_GPIO0_INT_STS4		(1 << 0)
#define PM800_GPIO1_INT_STS4		(1 << 1)
#define PM800_GPIO2_INT_STS4		(1 << 2)
#define PM800_GPIO3_INT_STS4		(1 << 3)
#define PM800_GPIO4_INT_STS4		(1 << 4)
#define PM800_GPADC_BD_EN		(1 << 5)
#define PM800_GPADC_BD_GP1_EN		(1 << 6)

#define PM800_INT_ENA_1		(0x09)
#define PM800_ONKEY_INT_ENA1		(1 << 0)
#define PM800_EXTON_INT_ENA1		(1 << 1)
#define PM800_CHG_INT_ENA1			(1 << 2)
#define PM800_BAT_INT_ENA1			(1 << 3)
#define PM800_RTC_INT_ENA1			(1 << 4)
#define PM800_CLASSD_OC_INT_ENA1	(1 << 5)

#define PM800_INT_ENA_2		(0x0A)
#define PM800_VBAT_INT_ENA2		(1 << 0)
#define PM800_VSYS_INT_ENA2		(1 << 1)
#define PM800_VCHG_INT_ENA2		(1 << 2)
#define PM800_TINT_INT_ENA2		(1 << 3)

#define PM800_INT_ENA_3		(0x0B)
#define PM800_GPADC0_INT_ENA3		(1 << 0)
#define PM800_GPADC1_INT_ENA3		(1 << 1)
#define PM800_GPADC2_INT_ENA3		(1 << 2)
#define PM800_GPADC3_INT_ENA3		(1 << 3)
#define PM800_GPADC4_INT_ENA3		(1 << 4)

#define PM800_INT_ENA_4		(0x0C)
#define PM800_GPIO0_INT_ENA4		(1 << 0)
#define PM800_GPIO1_INT_ENA4		(1 << 1)
#define PM800_GPIO2_INT_ENA4		(1 << 2)
#define PM800_GPIO3_INT_ENA4		(1 << 3)
#define PM800_GPIO4_INT_ENA4		(1 << 4)

#define PM800_GPADC_ODD_6UA			(1 << 4)

/*number of INT_ENA & INT_STATUS regs*/
#define PM800_INT_REG_NUM			(4)

/* Wakeup Registers */
#define PM800_WAKEUP1		(0x0D)

#define PM800_WAKEUP2		(0x0E)
#define PM800_WAKEUP2_INV_INT		(1 << 0)
#define PM800_WAKEUP2_INT_CLEAR		(1 << 1)
#define PM800_WAKEUP2_INT_MASK		(1 << 2)

#define PM800_POWER_UP_LOG	(0x10)

/*test page*/
#define PM800_TEST_PAGE_ENTRY	(0x1F)

/*Referance and low power registers*/
#define PM800_LOW_POWER1		(0x20)
#define PM800_LOW_POWER2		(0x21)
#define PM800_LOW_POWER_CONFIG3	(0x22)
#define PM800_LOW_POWER_CONFIG4	(0x23)

/*GPIO register*/
#define PM800_GPIO_0_1_CNTRL		(0x30)
#define PM800_GPIO0_VAL				(1 << 0)
#define PM800_GPIO0_GPIO_MODE(x)	(x << 1)
#define PM800_GPIO1_VAL				(1 << 4)
#define PM800_GPIO1_GPIO_MODE(x)	(x << 5)

#define PM800_GPIO_2_3_CNTRL		(0x31)
#define PM800_GPIO2_VAL				(1 << 0)
#define PM800_GPIO2_GPIO_MODE(x)	(x << 1)
#define PM800_GPIO3_VAL				(1 << 4)
#define PM800_GPIO3_GPIO_MODE(x)	(x << 5)
#define PM800_GPIO3_MODE_MASK		0x1F
#define PM800_GPIO3_HEADSET_MODE	PM800_GPIO3_GPIO_MODE(6)

#define PM800_GPIO_4_CNTRL			(0x32)
#define PM800_GPIO4_VAL				(1 << 0)
#define PM800_GPIO4_GPIO_MODE(x)	(x << 1)

#define PM800_HEADSET_CNTRL		(0x38)
#define PM800_HEADSET_DET_EN		(1 << 7)
#define PM800_HSDET_SLP			(1 << 1)
#define PM800_MIC_CNTRL			(0x39)
#define PM800_MICDET_EN			(1 << 0)

/*PWM register*/
#define PM800_PWM1		(0x40)
#define PM800_PWM2		(0x41)
#define PM800_PWM3		(0x42)
#define PM800_PWM4		(0x43)

/*RTC Registers*/
#define PM800_RTC_CONTROL		(0xD0)
#define PM800_RTC_COUNTER1		(0xD1)
#define PM800_RTC_COUNTER2		(0xD2)
#define PM800_RTC_COUNTER3		(0xD3)
#define PM800_RTC_COUNTER4		(0xD4)
#define PM800_RTC_EXPIRE1_1		(0xD5)
#define PM800_RTC_EXPIRE1_2		(0xD6)
#define PM800_RTC_EXPIRE1_3		(0xD7)
#define PM800_RTC_EXPIRE1_4		(0xD8)
#define PM800_RTC_TRIM1			(0xD9)
#define PM800_RTC_TRIM2			(0xDA)
#define PM800_RTC_TRIM3			(0xDB)
#define PM800_RTC_TRIM4			(0xDC)
#define PM800_RTC_EXPIRE2_1		(0xDD)
#define PM800_RTC_EXPIRE2_2		(0xDE)
#define PM800_RTC_EXPIRE2_3		(0xDF)
#define PM800_RTC_EXPIRE2_4		(0xE0)
#define PM800_RTC_MISC1			(0xE1)
#define PM800_RTC_MISC2			(0xE2)
#define PM800_RTC_MISC3			(0xE3)
#define PM800_RTC_MISC4			(0xE4)

/* bit definitions of RTC Register 1 (0xD0) */
#define PM800_ALARM1_EN			(1 << 0)
#define PM800_ALARM_WAKEUP		(1 << 4)
#define PM800_ALARM			(1 << 5)
#define PM800_RTC1_USE_XO		(1 << 7)

/*for save RTC offset*/
#define PM800_USER_DATA1		(0xE8)
#define PM800_USER_DATA2		(0xE9)
#define PM800_USER_DATA3		(0xEA)
#define PM800_USER_DATA4		(0xEB)
#define PM800_USER_DATA5		(0xEC)
#define PM800_USER_DATA6		(0xED)
#define PM800_USER_DATA7		(0xEE)
#define PM800_USER_DATA8		(0xEF)

#define PM800_POWER_DOWN_LOG1	(0xE5)
#define PM800_POWER_DOWN_LOG2	(0xE6)

#define PM800_RTC_MISC5			(0xE7)

/*********************************/
/*page 1 Power: slave adder 0x01*/
/********************************/
#define PM800_BUCK_POWER_GOOD_STS	(0x01)
#define PM800_LDO_POWER_GOOD_STS1	(0x02)
#define PM800_LDO_POWER_GOOD_STS2	(0x03)
#define PM800_LDO_POWER_GOOD_STS3	(0x04)
#define PM800_LDO_LAST_GROUP		(0x05)	/*reg#? not sure */

/* Regulator Control Registers: BUCK1,BUCK5,LDO1 have DVC */
/* LDO1 with DVC[0..3] */
#define PM800_LDO1_VOUT		(0x08) /* VOUT1 */
#define PM800_LDO1_VOUT_2	(0x09)
#define PM800_LDO1_VOUT_3	(0x0A)
#define PM800_LDO2_VOUT		(0x0B)
#define PM800_LDO3_VOUT		(0x0C)
#define PM800_LDO4_VOUT		(0x0D)
#define PM800_LDO5_VOUT		(0x0E)
#define PM800_LDO6_VOUT		(0x0F)
#define PM800_LDO7_VOUT		(0x10)
#define PM800_LDO8_VOUT		(0x11)
#define PM800_LDO9_VOUT		(0x12)
#define PM800_LDO10_VOUT	(0x13)
#define PM800_LDO11_VOUT	(0x14)
#define PM800_LDO12_VOUT	(0x15)
#define PM800_LDO13_VOUT	(0x16)
#define PM800_LDO14_VOUT	(0x17)
#define PM800_LDO15_VOUT	(0x18)
#define PM800_LDO16_VOUT	(0x19)
#define PM800_LDO17_VOUT	(0x1A)
#define PM800_LDO18_VOUT	(0x1B)
#define PM800_LDO19_VOUT	(0x1C)

/*buck registers*/
#define PM800_SLEEP_BUCK1	(0x30)
#define PM800_SLEEP_BUCK2	(0x31)
#define PM800_SLEEP_BUCK3	(0x32)
#define PM800_SLEEP_BUCK4	(0x33)
#define PM800_SLEEP_BUCK5	(0x34)
#define PM800_BUCK4_AUDIO	(0x39)
/* BUCK1 with DVC[0..3] */
#define PM800_BUCK1		(0x3C)
#define PM800_BUCK1_1		(0x3D)
#define PM800_BUCK1_2		(0x3E)
#define PM800_BUCK1_3		(0x3F)
#define PM800_BUCK2		(0x40)
#define PM800_BUCK3		(0x41)
#define PM800_BUCK3_DOUBLE	(1 << 6)

/* BUCK4 with DVC[0..3] */
#define PM800_BUCK4		(0x42)
#define PM800_BUCK4_1		(0x43)
#define PM800_BUCK4_2		(0x44)
#define PM800_BUCK4_3		(0x45)
#define PM800_BUCK5		(0x46)

#define PM800_BUCK_ENA		(0x50)
#define PM800_LDO_ENA1_1	(0x51)
#define PM800_LDO_ENA1_2	(0x52)
#define PM800_LDO_ENA1_3	(0x53)

#define PM800_LDO_ENA2_1	(0x56)
#define PM800_LDO_ENA2_2	(0x57)
#define PM800_LDO_ENA2_3	(0x58)

/* BUCK Sleep Mode Register 1: BUCK[1..4] */
#define PM800_BUCK_SLP1		(0x5A)
#define PM800_BUCK1_SLP1_SHIFT	0
#define PM800_BUCK1_SLP1_MASK	(0x3 << PM800_BUCK1_SLP1_SHIFT)
#define PM800_BUCK2_SLP1_SHIFT	2
#define PM800_BUCK2_SLP1_MASK	(0x3 << PM800_BUCK2_SLP1_SHIFT)
#define PM800_BUCK3_SLP1_SHIFT	4
#define PM800_BUCK3_SLP1_MASK	(0x3 << PM800_BUCK3_SLP1_SHIFT)
#define PM800_BUCK4_SLP1_SHIFT	6
#define PM800_BUCK4_SLP1_MASK	(0x3 << PM800_BUCK4_SLP1_SHIFT)
/* BUCK Sleep Mode Register 2: BUCK5 */
#define PM800_BUCK_SLP2		(0x5B)
#define PM800_BUCK5_SLP2_SHIFT	0
#define PM800_BUCK5_SLP2_MASK	(0x3 << PM800_BUCK5_SLP2_SHIFT)

/* LDO Sleep Mode Register 1: LDO[1..4] */
#define PM800_LDO_SLP1		(0x5C)
#define PM800_LDO1_SLP1_SHIFT	0
#define PM800_LDO1_SLP1_MASK	(0x3 << PM800_LDO1_SLP1_SHIFT)
#define PM800_LDO2_SLP1_SHIFT	2
#define PM800_LDO2_SLP1_MASK	(0x3 << PM800_LDO2_SLP1_SHIFT)
#define PM800_LDO3_SLP1_SHIFT	4
#define PM800_LDO3_SLP1_MASK	(0x3 << PM800_LDO3_SLP1_SHIFT)
#define PM800_LDO4_SLP1_SHIFT	6
#define PM800_LDO4_SLP1_MASK	(0x3 << PM800_LDO4_SLP1_SHIFT)

/* LDO Sleep Mode Register 2: LDO[5..8] */
#define PM800_LDO_SLP2		(0x5D)
#define PM800_LDO5_SLP2_SHIFT	0
#define PM800_LDO5_SLP2_MASK	(0x3 << PM800_LDO5_SLP2_SHIFT)
#define PM800_LDO6_SLP2_SHIFT	2
#define PM800_LDO6_SLP2_MASK	(0x3 << PM800_LDO6_SLP2_SHIFT)
#define PM800_LDO7_SLP2_SHIFT	4
#define PM800_LDO7_SLP2_MASK	(0x3 << PM800_LDO7_SLP2_SHIFT)
#define PM800_LDO8_SLP2_SHIFT	6
#define PM800_LDO8_SLP2_MASK	(0x3 << PM800_LDO8_SLP2_SHIFT)

/* LDO Sleep Mode Register 3: LDO[9..12] */
#define PM800_LDO_SLP3		(0x5E)
#define PM800_LDO9_SLP3_SHIFT	0
#define PM800_LDO9_SLP3_MASK	(0x3 << PM800_LDO9_SLP3_SHIFT)
#define PM800_LDO10_SLP3_SHIFT	2
#define PM800_LDO10_SLP3_MASK	(0x3 << PM800_LDO10_SLP3_SHIFT)
#define PM800_LDO11_SLP3_SHIFT	4
#define PM800_LDO11_SLP3_MASK	(0x3 << PM800_LDO11_SLP3_SHIFT)
#define PM800_LDO12_SLP3_SHIFT	6
#define PM800_LDO12_SLP3_MASK	(0x3 << PM800_LDO12_SLP3_SHIFT)

/* LDO Sleep Mode Register 4: LDO[13..16] */
#define PM800_LDO_SLP4		(0x5F)
#define PM800_LDO13_SLP4_SHIFT	0
#define PM800_LDO13_SLP4_MASK	(0x3 << PM800_LDO13_SLP4_SHIFT)
#define PM800_LDO14_SLP4_SHIFT	2
#define PM800_LDO14_SLP4_MASK	(0x3 << PM800_LDO14_SLP4_SHIFT)
#define PM800_LDO15_SLP4_SHIFT	4
#define PM800_LDO15_SLP4_MASK	(0x3 << PM800_LDO15_SLP4_SHIFT)
#define PM800_LDO16_SLP4_SHIFT	6
#define PM800_LDO16_SLP4_MASK	(0x3 << PM800_LDO16_SLP4_SHIFT)

/* LDO Sleep Mode Register 5: LDO[17..19] */
#define PM800_LDO_SLP5		(0x60)
#define PM800_LDO17_SLP5_SHIFT	0
#define PM800_LDO17_SLP5_MASK	(0x3 << PM800_LDO17_SLP5_SHIFT)
#define PM800_LDO18_SLP5_SHIFT	2
#define PM800_LDO18_SLP5_MASK	(0x3 << PM800_LDO18_SLP5_SHIFT)
#define PM800_LDO19_SLP5_SHIFT	4
#define PM800_LDO19_SLP5_MASK	(0x3 << PM800_LDO19_SLP5_SHIFT)

#define PM800_LDO_GROUP1	(0x68)
#define PM800_LDO_GROUP2	(0x69)
#define PM800_LDO_GROUP3	(0x6A)
#define PM800_LDO_GROUP4	(0x6B)
#define PM800_LDO_GROUP5	(0x6C)
#define PM800_LDO_GROUP6	(0x6D)
#define PM800_LDO_GROUP7	(0x6E)
#define PM800_LDO_GROUP8	(0x6F)
#define PM800_LDO_GROUP9	(0x70)
#define PM800_LDO_GROUP10	(0x71)

#define PM800_BUCK1_MISC1	(0x78)
#define PM800_BUCK3_MISC1	(0x7E)
#define PM800_BUCK4_MISC1	(0x81)
#define PM800_BUCK5_MISC1	(0x84)

#define PM800_LDO_MISC1		(0x90)
#define PM800_LDO_MISC2		(0x91)
#define PM800_LDO_MISC3		(0x92)
#define PM800_LDO_MISC4		(0x93)
#define PM800_LDO_MISC5		(0x94)
#define PM800_LDO_MISC6		(0x95)
#define PM800_LDO_MISC7		(0x96)
#define PM800_LDO_MISC8		(0x97)
#define PM800_LDO_MISC9		(0x98)
#define PM800_LDO_MISC10	(0x99)
#define PM800_LDO_MISC11	(0x9A)

/*********************************/
/*page 2 GPADC: slave adder 0x02*/
/********************************/
#define PM800_GPADC_MEAS_EN1		(0x01)
#define PM800_MEAS_EN1_VBAT         (1 << 2)
#define PM800_GPADC_MEAS_EN2		(0x02)
#define PM800_MEAS_EN2_RFTMP        (1 << 0)
#define PM800_MEAS_GP0_EN			(1 << 2)
#define PM800_MEAS_GP1_EN			(1 << 3)
#define PM800_MEAS_GP2_EN			(1 << 4)
#define PM800_MEAS_GP3_EN			(1 << 5)
#define PM800_MEAS_GP4_EN			(1 << 6)

#define PM800_GPADC_MISC_CONFIG1	(0x05)
#define PM800_GPADC_MISC_CONFIG2	(0x06)
#define PM800_GPADC_MISC_GPFSM_EN	(1 << 0)
#define PM800_GPADC_SLOW_MODE(x)	(x << 3)
#define PM800_GPADC4_DIR			(1 << 6)

#define PM800_GPADC_MEAS_OFF_TIME1	(0x07)
#define PM800_GPADC_MEAS_OFF_TIME2	(0x08)

#define PM800_GPADC_MISC_CONFIG3		(0x09)
#define PM800_GPADC_MISC_CONFIG4		(0x0A)
#define PM800_GPADC_MEAS_OFF_TIME2_1	(0x07)
#define PM800_GPADC_MEAS_OFF_TIME2_2	(0x08)

#define PM800_GP_BIAS_ENA1				(0x14)
#define PM800_GPADC_GP_BIAS_EN0			(1 << 0)
#define PM800_GPADC_GP_BIAS_EN1			(1 << 1)
#define PM800_GPADC_GP_BIAS_EN2			(1 << 2)
#define PM800_GPADC_GP_BIAS_EN3			(1 << 3)

#define PM800_GP_BIAS_OUT1		(0x15)
#define PM800_BIAS_OUT_GP0		(1 << 0)
#define PM800_BIAS_OUT_GP1		(1 << 1)
#define PM800_BIAS_OUT_GP2		(1 << 2)
#define PM800_BIAS_OUT_GP3		(1 << 3)

#define PM800_GPADC0_LOW_TH		0x20
#define PM800_GPADC1_LOW_TH		0x21
#define PM800_GPADC2_LOW_TH		0x22
#define PM800_GPADC3_LOW_TH		0x23
#define PM800_GPADC4_LOW_TH		0x24

#define PM800_GPADC0_UPP_TH		0x30
#define PM800_GPADC1_UPP_TH		0x31
#define PM800_GPADC2_UPP_TH		0x32
#define PM800_GPADC3_UPP_TH		0x33
#define PM800_GPADC4_UPP_TH		0x34

#define PM800_VBBAT_MEAS1		0x40
#define PM800_VBBAT_MEAS2		0x41
#define PM800_VBAT_MEAS1		0x42
#define PM800_VBAT_MEAS2		0x43
#define PM800_VSYS_MEAS1		0x44
#define PM800_VSYS_MEAS2		0x45
#define PM800_VCHG_MEAS1		0x46
#define PM800_VCHG_MEAS2		0x47
#define PM800_TINT_MEAS1		0x50
#define PM800_TINT_MEAS2		0x51
#define PM800_PMOD_MEAS1		0x52
#define PM800_PMOD_MEAS2		0x53

#define PM800_GPADC0_MEAS1		0x54
#define PM800_GPADC0_MEAS2		0x55
#define PM800_GPADC1_MEAS1		0x56
#define PM800_GPADC1_MEAS2		0x57
#define PM800_GPADC2_MEAS1		0x58
#define PM800_GPADC2_MEAS2		0x59
#define PM800_GPADC3_MEAS1		0x5A
#define PM800_GPADC3_MEAS2		0x5B
#define PM800_GPADC4_MEAS1		0x5C
#define PM800_GPADC4_MEAS2		0x5D

#define PM800_GPADC4_AVG1		0xA8
#define PM800_GPADC4_AVG2		0xA9
#define PM800_GPADC4_MIN1		0x88
#define PM800_GPADC4_MIN2		0x89
#define PM800_GPADC4_MAX1		0x98
#define PM800_GPADC4_MAX2		0x99
/*********************************/
/*page 7 TEST PAGE: slave adder 0x07*/
/********************************/

/*******************************
 * customer configuration start*
********************************/

/*****************************
 * customer configuration end*
******************************/

/* 88PM805 Registers */
#define PM805_MAIN_POWERUP		(0x01)
#define PM805_INT_STATUS0		(0x02)	/*for ena/dis all interrupts */

#define PM805_STATUS0_INT_CLEAR		(1 << 0)
#define PM805_STATUS0_INV_INT		(1 << 1)
#define PM800_STATUS0_INT_MASK		(1 << 2)

#define PM805_INT_STATUS1		(0x03)
#define PM805_INT_STATUS2		(0x04)
#define PM805_INT_MASK1			(0x05)
#define PM805_INT_MASK2			(0x06)
#define PM805_SHRT_BTN_DET		(1 << 1)

/*number of status and int reg in a row*/
#define PM805_INT_REG_NUM		(2)

#define PM805_MIC_DET1			(0x07)
#define PM805_MIC_DET_EN_MIC_DET (1 << 0)
#define PM805_MIC_DET2			(0x08)
#define PM805_MIC_DET_STATUS1	(0x09)
/*where is 2?*/
#define PM805_MIC_DET_STATUS3	(0x0A)
#define PM805_AUTO_SEQ_STATUS1	(0x0B)
#define PM805_AUTO_SEQ_STATUS2	(0x0C)

#define PM805_ADC_SETTING1		(0x10)
#define PM805_ADC_SETTING2		(0x11)
#define PM805_ADC_SETTING3		(0x11)
#define PM805_ADC_GAIN1			(0x12)
#define PM805_ADC_GAIN2			(0x13)
#define PM805_DMIC_SETTING		(0x15)
#define PM805_DWS_SETTING		(0x16)
#define PM805_MIC_CONFLICT_STS	(0x17)

#define PM805_PDM_SETTING1		(0x20)
#define PM805_PDM_SETTING2		(0x21)
#define PM805_PDM_SETTING3		(0x22)
#define PM805_PDM_CONTROL1		(0x23)
#define PM805_PDM_CONTROL2		(0x24)
#define PM805_PDM_CONTROL3		(0x25)

#define PM805_HEADPHONE_SETTING			(0x26)
#define PM805_HEADPHONE_GAIN_A2A		(0x27)
#define PM805_HEADPHONE_SHORT_STATE		(0x28)
#define PM805_EARPHONE_SETTING			(0x29)
#define PM805_AUTO_SEQ_SETTING			(0x2A)

/*******************************
 * customer configuration start*
********************************/
/* for disabling the use of PM805*/
/*#define NO_PM805_CHIP*/
/*****************************
 * customer configuration end*
******************************/

/* Interrupt Number in 88PM800 */
enum {
	PM800_IRQ_ONKEY,	/*EN1b0 *//*0 */
	PM800_IRQ_EXTON,	/*EN1b1 */
	PM800_IRQ_CHG,		/*EN1b2 */
	PM800_IRQ_BAT,		/*EN1b3 */
	PM800_IRQ_RTC,		/*EN1b4 */
	PM800_IRQ_CLASSD,	/*EN1b5 *//*5 */
	PM800_IRQ_VBAT,		/*EN2b0 */
	PM800_IRQ_VSYS,		/*EN2b1 */
	PM800_IRQ_VCHG,		/*EN2b2 */
	PM800_IRQ_TINT,		/*EN2b3 */
	PM800_IRQ_GPADC0,	/*EN3b0 *//*10 */
	PM800_IRQ_GPADC1,	/*EN3b1 */
	PM800_IRQ_GPADC2,	/*EN3b2 */
	PM800_IRQ_GPADC3,	/*EN3b3 */
	PM800_IRQ_GPADC4,	/*EN3b4 */
	PM800_IRQ_GPIO0,	/*EN4b0 *//*15 */
	PM800_IRQ_GPIO1,	/*EN4b1 */
	PM800_IRQ_GPIO2,	/*EN4b2 */
	PM800_IRQ_GPIO3,	/*EN4b3 */
	PM800_IRQ_GPIO4,	/*EN4b4 *//*19 */
	PM800_MAX_IRQ,
};

/* Interrupt Number in 88PM805 */
enum {
	PM805_IRQ_LDO_OFF,	/*0 */
	PM805_IRQ_SRC_DPLL_LOCK,	/*1 */
	PM805_IRQ_CLIP_FAULT,
	PM805_IRQ_MIC_CONFLICT,
	PM805_IRQ_HP2_SHRT,
	PM805_IRQ_HP1_SHRT,	/*5 */
	PM805_IRQ_FINE_PLL_FAULT,
	PM805_IRQ_RAW_PLL_FAULT,
	PM805_IRQ_VOLP_BTN_DET,
	PM805_IRQ_VOLM_BTN_DET,
	PM805_IRQ_SHRT_BTN_DET,	/*10 */
	PM805_IRQ_MIC_DET,	/*11 */

	PM805_MAX_IRQ,
};

struct pm80x_subchip {
	struct device *dev;
	struct pm80x_chip *chip;
	struct pm80x_platform_data *pdata;
	struct i2c_client *client;
	int irq;
	int irq_base;
	int irq_mode;
};

struct pm80x_chip {
	/*chip_version can only on the top of the struct*/
	unsigned char chip800_version;
	unsigned char chip805_version;
	struct device *dev;
	struct pm80x_subchip *pm800_chip;
	struct pm80x_subchip *pm805_chip;
	struct mutex io_lock;
	struct mutex pm800_irq_lock;
	struct mutex pm805_irq_lock;
	struct i2c_client *client;
	struct i2c_client *companion;	/* companion chip client */
	struct i2c_client *base_page;	/* chip client for base page */
	struct i2c_client *power_page;	/* chip client for power page */
	struct i2c_client *gpadc_page;	/* chip client for gpadc page */
	struct i2c_client *test_page;	/* chip client for test page */
	struct pm80x_dvc_pdata *dvc;

	struct workqueue_struct *pm800_wqueue;
	struct workqueue_struct *pm805_wqueue;

	int buck3_double;	/* DVC ramp slope double */
	unsigned short companion_addr;
	unsigned short base_page_addr;	/* base page I2C address */
	unsigned short power_page_addr;	/* power page I2C address */
	unsigned short gpadc_page_addr;	/* gpadc page I2C address */
	unsigned short test_page_addr;	/* test page I2C address */
	int id;
	int irq_base;
	int irq_companion;
};

enum {
	PM80X_GPIO1_SUPPLY_VBUS = 1,
	PM80X_GPIO2_SUPPLY_VBUS = 2,
};

enum {
	PM80X_IDPIN_NO_USE = 0,
	PM80X_IDPIN_USE_GPADC0,
	PM80X_IDPIN_USE_GPADC1,
	PM80X_IDPIN_USE_GPADC2,
	PM80X_IDPIN_USE_GPADC3,
};

struct pm80x_vbus_pdata {
	int             supply;
	int             idpin;
	unsigned int    reg_base;       /* Physical address */
	unsigned int    reg_end;        /* Physical address end */
};

struct pm80x_rtc_pdata {
	int		(*sync)(unsigned int ticks);
	int		vrtc;
	int             rtc_wakeup;
};

struct pm80x_headset_pdata {
	int		gpio;
	int		gpio_ctl;
	int		gpio_enable_irq;
	int		gpio_set_mask;
	int		gpio_set_val;
	int		gpio_val_bit;
	void		(*mic_set_power)(int on);
};

struct pm80x_vibrator_pdata {
	int		min_timeout;
	void		(*vibrator_power)(int on);
};

struct pm80x_dvc_pdata {
	int dvc1;
	int dvc2;
	unsigned int *vol_val;
	int size;
	int gpio_dvc;	/* 0 when gpios are not used for dvc */
};

struct pm80x_platform_data {
	struct pm80x_rtc_pdata *rtc;
	struct pm80x_headset_pdata *headset;
	struct pm80x_vbus_pdata *vbus;
	struct pm80x_vibrator_pdata *vibrator;
	struct regulator_init_data *regulator;
	struct pm80x_dvc_pdata *dvc;

	unsigned short companion_addr;	/* companion chip I2C address */
	unsigned short base_page_addr;	/* base page I2C address */
	unsigned short power_page_addr;	/* power page I2C address */
	unsigned short gpadc_page_addr;	/* gpadc page I2C address */
	unsigned short test_page_addr;	/* test page regs I2C address */
	int i2c_port;		/* Controlled by GI2C or PI2C */
	int irq_mode;		/* Clear interrupt by read/write(0/1) */
	int irq_base;		/* IRQ base number of 88pm80x */
	int irq_companion;	/* IRQ number of companion chip */
	int batt_det;		/* enable/disable */
	int num_regulators;
	int headset_flag;
	int (*pm800_plat_config)(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata);
	int (*pm805_plat_config)(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata);
};
#ifdef CONFIG_MFD_88PM80X
extern int pm80x_reg_read(struct i2c_client *, int);
extern int pm80x_reg_write(struct i2c_client *, int, unsigned char);
extern int pm80x_codec_reg_read(int reg);
extern int pm80x_codec_reg_write(int reg, unsigned char data);
extern int pm80x_codec_reg_set_bits(int reg,
				    unsigned char mask, unsigned char data);
extern int pm80x_bulk_read(struct i2c_client *, int, int, unsigned char *);
extern int pm80x_bulk_write(struct i2c_client *, int, int, unsigned char *);
extern int pm80x_set_bits(struct i2c_client *, int, unsigned char,
			  unsigned char);
extern int pm805_debug_reg_read(struct i2c_client *i2c, int reg);
extern int pm805_debug_reg_write(struct i2c_client *i2c, int reg,
				unsigned char data);
extern int pm80x_device_init(struct pm80x_chip *chip,
			     struct pm80x_platform_data *pdata) __devinit;
extern void pm80x_device_exit(struct pm80x_chip *chip) __devexit;

extern int pm805_device_init(struct pm80x_chip *chip,
			     struct pm80x_platform_data *pdata) __devinit;
extern void pm805_device_exit(struct pm80x_chip *chip) __devexit;

#ifdef CONFIG_USB_VBUS_88PM80X
extern int pm80x_read_id_val(void);
extern void pm80x_init_id(void);
extern int pm80x_set_vbus(unsigned int vbus);
extern int pm80x_read_vbus_val(void);
#else
static inline int pm80x_read_id_val(void) { return 0; }
static inline void pm80x_init_id(void) { return; }
static inline int pm80x_set_vbus(unsigned int vbus) { return 0; }
static inline int pm80x_read_vbus_val(void) { return 0; }
#endif	/* CONFIG_USB_VBUS_88PM80X */

#else

static inline int pm80x_reg_read(struct i2c_client *i2c, int reg)
{
	return 0;
}
static inline int pm80x_reg_write(struct i2c_client *i2c, int reg, unsigned char data)
{
	return 0;
}
static inline int pm80x_codec_reg_read(int reg)
{
	return 0;
}
static inline int pm80x_codec_reg_write(int reg, unsigned char data)
{
	return 0;
}
static inline int pm80x_codec_reg_set_bits(int reg,
				    unsigned char mask, unsigned char data)
{
	return 0;
}
static inline int pm80x_bulk_read(struct i2c_client *i2c, int reg,
		    int count, unsigned char *buf)
{
	return 0;
}
static inline int pm80x_bulk_write(struct i2c_client *i2c, int reg,
		     int count, unsigned char *buf)
{
	return 0;
}
static inline int pm80x_set_bits(struct i2c_client *i2c, int reg,
		   unsigned char mask, unsigned char data)
{
	return 0;
}
static inline int pm805_debug_reg_read(struct i2c_client *i2c, int reg)
{
	return 0;
}
static inline int pm805_debug_reg_write(struct i2c_client *i2c, int reg,
				unsigned char data)
{
	return 0;
}
static inline int pm80x_device_init(struct pm80x_chip *chip,
			     struct pm80x_platform_data *pdata)
{
	return 0;
}
static inline void pm80x_device_exit(struct pm80x_chip *chip)
{
}
static inline int pm80x_read_id_val(void)
{
	return 0;
}
static inline void pm80x_init_id(void)
{
}
static inline int pm80x_set_vbus(unsigned int vbus)
{
	return 0;
}
static inline int pm80x_read_vbus_val(void)
{
	return 0;
}
#endif
#endif /* __LINUX_MFD_88PM80X_H */
