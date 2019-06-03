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

/* Raw i2c operations __ONLY__ can be used for reboot routine */
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)	\
		|| defined(CONFIG_CPU_PXA910)

#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
#define MAX_BUS_NUM	6
static const u32 i2c_reg_idbr[MAX_BUS_NUM] = {
	APB_VIRT_BASE + 0x11000 + 0x08,
	APB_VIRT_BASE + 0x31000 + 0x08,
	APB_VIRT_BASE + 0x32000 + 0x08,
	APB_VIRT_BASE + 0x33000 + 0x08,
	APB_VIRT_BASE + 0x33800 + 0x08,
	APB_VIRT_BASE + 0x34000 + 0x08,
};
static const u32 i2c_reg_icr[MAX_BUS_NUM] = {
	APB_VIRT_BASE + 0x11000 + 0x10,
	APB_VIRT_BASE + 0x31000 + 0x10,
	APB_VIRT_BASE + 0x32000 + 0x10,
	APB_VIRT_BASE + 0x33000 + 0x10,
	APB_VIRT_BASE + 0x33800 + 0x10,
	APB_VIRT_BASE + 0x34000 + 0x10,
};
static const u32 i2c_reg_isr[MAX_BUS_NUM] = {
	APB_VIRT_BASE + 0x11000 + 0x18,
	APB_VIRT_BASE + 0x31000 + 0x18,
	APB_VIRT_BASE + 0x32000 + 0x18,
	APB_VIRT_BASE + 0x33000 + 0x18,
	APB_VIRT_BASE + 0x33800 + 0x18,
	APB_VIRT_BASE + 0x34000 + 0x18,
};
#endif

#if defined(CONFIG_CPU_PXA910)
#define MAX_BUS_NUM	2
static const u32 i2c_reg_idbr[MAX_BUS_NUM] = {
	APB_VIRT_BASE + 0x11000 + 0x08,
	APB_VIRT_BASE + 0x37000 + 0x08,
};
static const u32 i2c_reg_icr[MAX_BUS_NUM] = {
	APB_VIRT_BASE + 0x11000 + 0x10,
	APB_VIRT_BASE + 0x37000 + 0x10,
};
static const u32 i2c_reg_isr[MAX_BUS_NUM] = {
	APB_VIRT_BASE + 0x11000 + 0x18,
	APB_VIRT_BASE + 0x37000 + 0x18,
};
#endif

/* Control register bits */
#define ICR_START	(1 << 0)
#define ICR_STOP	(1 << 1)
#define ICR_ACKNAK	(1 << 2)
#define ICR_TB		(1 << 3)
#define ICR_ALDIE	(1 << 12)
/* Status register bits */
#define ISR_ACKNAK	(1 << 1)
#define ISR_IBB		(1 << 3)
#define ISR_ITE		(1 << 6)
#define ISR_IRF		(1 << 7)

static int i2c_isr_set_cleared(u32 reg_isr, u32 set, u32 cleared)
{
	int isr, timeout = 1000;
	do {
		udelay(10);
		isr = readl(reg_isr);
		if (timeout-- < 0)
			return 0;
	} while (((isr & set) != set) ||
			((isr & cleared) != 0));
	return 1;
}

int __raw_i2c_bus_reset(u8 bus_num)
{
	u32 reg_icr;
	if (bus_num == 0 || bus_num > MAX_BUS_NUM) {
		pr_err("%s: bus_num if out of range!\n", __func__);
		return -1;
	}
	reg_icr = i2c_reg_icr[bus_num - 1];
	writel(0x4060, reg_icr);
	udelay(500);
	writel(0x60, reg_icr);
	udelay(500);
	return 0;
}

int __raw_i2c_write_reg(u8 bus_num, u8 addr, u8 reg, u8 val)
{
	u32 reg_idbr, reg_icr, reg_isr;
	if (bus_num == 0 || bus_num > MAX_BUS_NUM) {
		pr_err("%s: bus_num if out of range!\n", __func__);
		return -1;
	}
	reg_idbr = i2c_reg_idbr[bus_num - 1];
	reg_icr = i2c_reg_icr[bus_num - 1];
	reg_isr = i2c_reg_isr[bus_num - 1];
	/* Is bus busy? */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_IBB)) {
		pr_err("%s: bus is busy!\n", __func__);
		return -1;
	}
	/* -----1. Send chip addr -------  */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write chip addr */
	writel((addr << 1), reg_idbr);
	/* Send start bit */
	writel(readl(reg_icr) | ICR_START, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(reg_isr, ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_ACKNAK))
		return -1;
	/* -----2. Send reg addr ------ */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write reg addr */
	writel(reg, reg_idbr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(reg_isr, ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_ACKNAK))
		return -1;
	/* -----3. Send val ------ */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write val */
	writel(val, reg_idbr);
	/* Send Stop bit */
	writel(readl(reg_icr) | ICR_STOP, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(reg_isr, ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_ACKNAK))
		return -1;

	return 0;
}

int __raw_i2c_read_reg(u8 bus_num, u8 addr, u8 reg, u8 *buf, int len)
{
	u32 reg_idbr, reg_icr, reg_isr;
	if (bus_num == 0 || bus_num > MAX_BUS_NUM) {
		pr_err("%s: bus_num if out of range!\n", __func__);
		return -1;
	}
	reg_idbr = i2c_reg_idbr[bus_num - 1];
	reg_icr = i2c_reg_icr[bus_num - 1];
	reg_isr = i2c_reg_isr[bus_num - 1];
	/* Is bus busy? */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_IBB)) {
		pr_err("%s: bus is busy!\n", __func__);
		return -1;
	}
	/* -----1. Send chip addr -------  */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write chip addr */
	writel((addr << 1), reg_idbr);
	/* Send start bit */
	writel(readl(reg_icr) | ICR_START, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(reg_isr, ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_ACKNAK))
		return -1;
	/* -----2. Send reg addr ------ */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write reg addr */
	writel(reg, reg_idbr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(reg_isr, ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_ACKNAK))
		return -1;
	/* -----3. Start read sequence -------  */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write chip addr: R/nW=1 */
	writel((addr << 1) | 0x1, reg_idbr);
	/* Send start bit */
	writel(readl(reg_icr) | ICR_START, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(reg_isr, ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(reg_isr, 0, ISR_ACKNAK))
		return -1;
	while (len--) {
		/* Clear START and STOP bits */
		writel(readl(reg_icr) & ~(ICR_START), reg_icr);
		writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
		if (len == 0)
			/* Send NACK */
			writel(readl(reg_icr) | ICR_ACKNAK, reg_icr);
		else
			/* Send ACK */
			writel(readl(reg_icr) & ~ICR_ACKNAK, reg_icr);
		/* Clear ALDIE */
		writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
		/* Transfer, receive a byte */
		writel(readl(reg_icr) | ICR_TB, reg_icr);
		/* Receive register full? */
		if (!i2c_isr_set_cleared(reg_isr, ISR_IRF, 0))
			return -1;
		*buf = readl(reg_idbr);
		buf++;
		/* Clear ITE: W1C */
		writel(readl(reg_isr) | ISR_IRF, reg_isr);
	}
	return 0;
}
#endif

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
	printk(KERN_ERR "MK2 reset\n");
	mdelay(2000);
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
