/*
 *  linux/arch/arm/mach-mmp/reset.c
 *
 *  Copyright (C) 2009-2011 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-timers.h>
#include <mach/cputype.h>
#include <linux/delay.h>
#include <linux/mfd/88pm860x.h>

#define REG_RTC_BR0	(APB_VIRT_BASE + 0x010014)

#define MPMU_APRR_WDTR	(1<<4)
#define MPMU_APRR_CPR	(1<<0)
#define MPMU_CPRR_DSPR	(1<<2)
#define MPMU_CPRR_BBR	(1<<3)

/* Using watchdog reset */
static void do_wdt_reset(const char *cmd)
{
	u32 reg, backup;
	u32 watchdog_virt_base;
	int i;
	int match = 0, count = 0;

	if (cpu_is_pxa910_family() || cpu_is_pxa920_family())
		watchdog_virt_base = CP_TIMERS2_VIRT_BASE;
	else if (cpu_is_pxa168())
		watchdog_virt_base = TIMERS1_VIRT_BASE;
	else
		return;

	/* reset/enable WDT clock */
	writel(0x7, MPMU_WDTPCR);
	readl(MPMU_WDTPCR);
	writel(0x3, MPMU_WDTPCR);
	readl(MPMU_WDTPCR);

	if (cpu_is_pxa910_family() || cpu_is_pxa920_family()) {
		if (cmd && !strcmp(cmd, "recovery")) {
			for (i = 0, backup = 0; i < 4; i++) {
				backup <<= 8;
				backup |= *(cmd + i);
			}
			do {
				writel(backup, REG_RTC_BR0);
			} while (readl(REG_RTC_BR0) != backup);
		}
	}

	/* enable WDT reset */
	writel(0xbaba, watchdog_virt_base + TMR_WFAR);
	writel(0xeb10, watchdog_virt_base + TMR_WSAR);
	writel(0x3, watchdog_virt_base + TMR_WMER);

	if (cpu_is_pxa910_family() || cpu_is_pxa920_family()) {
		/*hold CP first */
		reg = readl(MPMU_APRR) | MPMU_APRR_CPR;
		writel(reg, MPMU_APRR);
		udelay(10);
		/*CP reset MSA */
		reg = readl(MPMU_CPRR) | MPMU_CPRR_DSPR | MPMU_CPRR_BBR;
		writel(reg, MPMU_CPRR);
		udelay(10);
	}
	/* negate hardware reset to the WDT after system reset */
	reg = readl(MPMU_APRR) | MPMU_APRR_WDTR;
	writel(reg, MPMU_APRR);

	/* clear previous WDT status */
	writel(0xbaba, watchdog_virt_base + TMR_WFAR);
	writel(0xeb10, watchdog_virt_base + TMR_WSAR);
	writel(0, watchdog_virt_base + TMR_WSR);

	match = readl(watchdog_virt_base + TMR_WMR);
	count = readl(watchdog_virt_base + TMR_WVR);

	if (match - count > 0x20) {
		/* set match counter */
		writel(0xbaba, watchdog_virt_base + TMR_WFAR);
		writel(0xeb10, watchdog_virt_base + TMR_WSAR);
		writel(0x20 + count, watchdog_virt_base + TMR_WMR);
	}
}

int pxa_board_reset(char mode, const char *cmd)
{
	return 0;
}

int (*board_reset)(char mode, const char *cmd) = pxa_board_reset;
EXPORT_SYMBOL(board_reset);

static void mmp_arch_reset(char mode, const char *cmd)
{
	if (board_reset(mode, cmd))
		return;

	if ((!cpu_is_pxa910_family()) && (!cpu_is_pxa920_family()) && (!cpu_is_pxa168()))
		return;

	switch (mode) {
	case 's':
		/* Jump into ROM at address 0 */
		cpu_reset(0);
		break;
	case 'w':
	default:
		do_wdt_reset(cmd);
		break;
	}
}

/*
 * Reset the system. It is called by machine_restart().
 */
void (*arch_reset)(char, const char *) = mmp_arch_reset;
