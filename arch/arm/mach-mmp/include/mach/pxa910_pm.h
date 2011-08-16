/*
 * PXA910 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __PXA910_PM_H__
#define __PXA910_PM_H__

#define PMUA_MOH_DIS_MC_SW_REQ	(1 << 21)
#define PMUA_MOH_MC_WAKE_EN	(1 << 20)
#define PMUA_MOH_SRAM_PWRDWN	(1 << 6)
#define PMUA_MOH_PWRDWN		(1 << 5)
#define PMUA_MOH_IDLE		(1 << 1)

#define PMUM_AXISD		(1 << 31)
#define PMUM_DSPSD		(1 << 30)
#define PMUM_SLPEN		(1 << 29)
#define PMUM_DTCMSD		(1 << 28)
#define PMUM_DDRCORSD		(1 << 27)
#define PMUM_APBSD		(1 << 26)
#define PMUM_BBSD		(1 << 25)
#define PMUM_INTCLR		(1 << 24)
#define PMUM_SLPWP0		(1 << 23)
#define PMUM_SLPWP1		(1 << 22)
#define PMUM_SLPWP2		(1 << 21)
#define PMUM_SLPWP3		(1 << 20)
#define PMUM_VCTCXOSD		(1 << 19)
#define PMUM_SLPWP4		(1 << 18)
#define PMUM_SLPWP5		(1 << 17)
#define PMUM_SLPWP6		(1 << 16)
#define PMUM_SLPWP7		(1 << 15)
#define PMUM_MSASLPEN		(1 << 14)
#define PMUM_STBYEN		(1 << 13)

#define PMUM_GSM_WAKEUPWMX	(1 << 29)
#define PMUM_WCDMA_WAKEUPX	(1 << 28)
#define PMUM_GSM_WAKEUPWM	(1 << 27)
#define PMUM_WCDMA_WAKEUPWM	(1 << 26)
#define PMUM_AP_ASYNC_INT	(1 << 25)
#define PMUM_AP_FULL_IDLE	(1 << 24)
#define PMUM_SDH1		(1 << 23)
#define PMUM_SDH2		(1 << 22)
#define PMUM_KEYPRESS		(1 << 21)
#define PMUM_TRACKBALL		(1 << 20)
#define PMUM_NEWROTARY		(1 << 19)
#define PMUM_WDT		(1 << 18)
#define PMUM_RTC_ALARM		(1 << 17)
#define PMUM_CP_TIMER_3		(1 << 16)
#define PMUM_CP_TIMER_2		(1 << 15)
#define PMUM_CP_TIMER_1		(1 << 14)
#define PMUM_AP2_TIMER_3	(1 << 13)
#define PMUM_AP2_TIMER_2	(1 << 12)
#define PMUM_AP2_TIMER_1	(1 << 11)
#define PMUM_AP1_TIMER_3	(1 << 10)
#define PMUM_AP1_TIMER_2	(1 << 9)
#define PMUM_AP1_TIMER_1	(1 << 8)
#define PMUM_WAKEUP7		(1 << 7)
#define PMUM_WAKEUP6		(1 << 6)
#define PMUM_WAKEUP5		(1 << 5)
#define PMUM_WAKEUP4		(1 << 4)
#define PMUM_WAKEUP3		(1 << 3)
#define PMUM_WAKEUP2		(1 << 2)
#define PMUM_WAKEUP1		(1 << 1)
#define PMUM_WAKEUP0		(1 << 0)

enum {
	POWER_MODE_ACTIVE = 0,
	POWER_MODE_CORE_INTIDLE,
	POWER_MODE_CORE_EXTIDLE,
	POWER_MODE_APPS_IDLE,
	POWER_MODE_APPS_SLEEP,
	POWER_MODE_SYS_SLEEP,
	POWER_MODE_HIBERNATE,
	POWER_MODE_UDR,
};

enum {
	IDLE_ACTIVE = 0,
	IDLE_CORE_INTIDLE = 1,
	IDLE_CORE_EXTIDLE = 2,
	IDLE_APPS_IDLE = 4,
	IDLE_APPS_SLEEP = 8,
	IDLE_SYS_SLEEP = 16,
};

struct pxa910_peripheral_config_ops{
	int	(*pin_lpm_config)(void);
	int	(*pin_restore)(void);
};

extern int pxa910_power_config_register(struct pxa910_peripheral_config_ops *ops);
extern void gc_pwr(int power_on);

#endif
