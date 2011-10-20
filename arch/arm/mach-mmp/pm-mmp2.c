/*
 * MMP2 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/addr-map.h>
#include <mach/dma.h>
#include <mach/gpio.h>
#include <mach/regs-apbc.h>
#include <mach/regs-icu.h>
#include <mach/regs-smc.h>
#include <mach/regs-timers.h>
#include <mach/mmp2_pm.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/wakelock.h>

static int pmic_wakeup_detect;
static int sdh_wakeup_detect;
struct wake_lock wakelock_pmic_wakeup;
struct wake_lock wakelock_sdh_wakeup;

static void apbc_set_rst(unsigned int clk_addr, int flag)
{
	unsigned int clkreg;
	unsigned int rst_msk;

	rst_msk = 1 << 2;
	clkreg = __raw_readl(clk_addr);
	if (flag)
		clkreg |= rst_msk;
	else
		clkreg &= ~rst_msk;
	__raw_writel(clkreg, clk_addr);

	return ;
}

static void pm_apbc_clk_disable(void)
{
	unsigned int clkreg;
	unsigned int rst_msk;

	rst_msk = 0x03;
	clkreg = __raw_readl(APBC_MMP2_IPC);
	clkreg &= ~rst_msk;
	__raw_writel(clkreg, APBC_MMP2_IPC);
	clkreg = __raw_readl(APBC_MMP2_THSENS1);
	clkreg &= ~rst_msk;
	__raw_writel(clkreg, APBC_MMP2_THSENS1);

	apbc_set_rst(APBC_MMP2_IPC, 1);

	return ;
}

static void pm_apbc_clk_enable(void)
{
	unsigned int clkreg;
	unsigned int rst_msk;

	rst_msk = 0x03;
	clkreg = __raw_readl(APBC_MMP2_IPC);
	clkreg |= rst_msk;
	__raw_writel(clkreg, APBC_MMP2_IPC);
	clkreg = __raw_readl(APBC_MMP2_THSENS1);
	clkreg |= rst_msk;
	__raw_writel(clkreg, APBC_MMP2_THSENS1);

	apbc_set_rst(APBC_MMP2_IPC, 0);

	return ;
}

static void pm_scu_clk_disable(void)
{
	unsigned int val;

	/* close AXI fabric clock gate */
	__raw_writel(0x0, 0xfe282c64);
	__raw_writel(0x0, 0xfe282c68);

	val = __raw_readl(0xfe282c1c);	/* MCB_CONF */
	val |= 0xf0;
	__raw_writel(val, 0xfe282c1c);

	return ;
}

static void pm_scu_clk_enable(void)
{
	unsigned int val;

	/* open AXI fabric clock gate */
	__raw_writel(0x03003003, 0xfe282c64);
	__raw_writel(0x00303030, 0xfe282c68);

	val = __raw_readl(0xfe282c1c);	/* MCB_CONF */
	val &= ~(0xf0);
	__raw_writel(val, 0xfe282c1c);

	return ;
}

static void pm_apmu_clk_disable(void)
{
	unsigned int val;

	val = __raw_readl(0xfe2828d8);	/* PMUA_MSPRO_CLK_RES_CTRL */
	val &= ~(0x3f);
	__raw_writel(val, 0xfe2828d8);

	val = __raw_readl(0xfe2828dc);	/* PMUA_GLB_CLK_RES_CTRL */
	val &= ~(0x1fffd);
	__raw_writel(val, 0xfe2828dc);

	val = __raw_readl(0xfe28285c);		/* PMUA_USB_CLK_RES_CTRL */
	val &= ~(0x9);
	val = __raw_writel(val, 0xfe28285c);

	return ;
}

static void pm_apmu_clk_enable(void)
{
	unsigned int val;

	val = __raw_readl(0xfe2828d8);
	val |= 0x3f;
	__raw_writel(val, 0xfe2828d8);

	val = __raw_readl(0xfe2828dc);
	val |= 0x1fffd;
	__raw_writel(val, 0xfe2828dc);

	val = __raw_readl(0xfe28285c);
	val |= 0x9;
	__raw_writel(val, 0xfe28285c);

	return ;
}

static void pm_mpmu_clk_disable(void)
{
	unsigned int val;

	/* disable clocks in MPMU_CGR_PJ register
	 * except clock for APMU_PLL1, APMU_PLL1_2 and AP_26M*/
	__raw_writel(0x0000a010, MPMU_ACGR);

	/* disable I2S clock input/output to SYSCLKx */
	val = __raw_readl(MPMU_ISCCRX0);
	val &= ~(1 << 31);
	__raw_writel(val, MPMU_ISCCRX0);
	val = __raw_readl(MPMU_ISCCRX1);
	val &= ~(1 << 31);
	__raw_writel(val, MPMU_ISCCRX1);

	return ;
}

static void pm_mpmu_clk_enable(void)
{
	unsigned int val;

	__raw_writel(0xdffefffe, MPMU_ACGR);
	val = __raw_readl(MPMU_PLL2_CTRL1);
	val |= (1 << 29);
	__raw_writel(val, MPMU_PLL2_CTRL1);
	val = __raw_readl(MPMU_ISCCRX0);
	val |= (1 << 31);
	__raw_writel(val, MPMU_ISCCRX0);
	val = __raw_readl(MPMU_ISCCRX1);
	val |= (1 << 31);
	__raw_writel(val, MPMU_ISCCRX1);

	return ;
}

void mmp2_pm_enter_lowpower_mode(int state)
{
	uint32_t idle_cfg, apcr;

	idle_cfg = __raw_readl(APMU_IDLE_CFG);
	apcr = __raw_readl(MPMU_APCR);

	apcr &= ~(PMUM_SLPEN | PMUM_DDRCORSD | PMUM_APBSD | PMUM_AXISD);
	apcr &= ~PMUM_VCTCXOSD;
	idle_cfg &= ~PMUA_MOH_IDLE;

	switch (state) {
	case POWER_MODE_SYS_SLEEP:
		apcr |= PMUM_SLPEN;			/* set the SLPEN bit */
		apcr |= PMUM_VCTCXOSD;			/* set VCTCXOSD */
		/* fall through */
	case POWER_MODE_CHIP_SLEEP:
		apcr |= PMUM_SLPEN;
		/* fall through */
	case POWER_MODE_APPS_SLEEP:
		apcr |= PMUM_APBSD;			/* set APBSD */
		/* fall through */
	case POWER_MODE_APPS_IDLE:
		apcr |= PMUM_AXISD;			/* set AXISDD bit */
		apcr |= PMUM_DDRCORSD;			/* set DDRCORSD bit */
		/* fall through */
	case POWER_MODE_CORE_EXTIDLE:
		idle_cfg |= PMUA_MOH_IDLE;		/* set the IDLE bit */
		idle_cfg &= ~(0x3<<28);
		idle_cfg |= 0x000a0000;
		/* fall through */
	case POWER_MODE_CORE_INTIDLE:
		break;
	}

	/* program the memory controller hardware sleep type and auto wakeup */
	idle_cfg |= PMUA_MOH_DIS_MC_SW_REQ;
	idle_cfg |= PMUA_MOH_MC_WAKE_EN;
	__raw_writel(0x0, APMU_MC_HW_SLP_TYPE);		/* auto refresh */

	/* set DSPSD, DTCMSD, BBSD, MSASLPEN */
	apcr |= PMUM_DSPSD | PMUM_DTCMSD | PMUM_BBSD;

	/* finally write the registers back */
	__raw_writel(idle_cfg, APMU_IDLE_CFG);
	__raw_writel(apcr, MPMU_APCR);	/* 0xfe086000 */
}


static int mmp2_pm_enter(suspend_state_t state)
{
	int temp;

	temp = __raw_readl(ICU_INT_CONF(IRQ_MMP2_PMIC_MUX));
	if (!(temp & ICU_INT_ROUTE_PJ4_IRQ)) {
		printk(KERN_ERR "%s: PMIC interrupt is handling\n", __func__);
		return -EAGAIN;
	}

	temp = __raw_readl(APMU_SRAM_PWR_DWN);
	temp |= ((1 << 19) | (1 << 18));
	__raw_writel(temp, APMU_SRAM_PWR_DWN);

	/* set RST in APBC_TWSIx_CLK_RST registers
	 * just disable TWSI1 clk rather than reset
	 * since it's needed to access PMIC onkey
	 * when system is waken up from low power mode */
	__raw_writel(0x0, APBC_MMP2_TWSI1);
	apbc_set_rst(APBC_MMP2_TWSI2, 1);
	apbc_set_rst(APBC_MMP2_TWSI3, 1);
	apbc_set_rst(APBC_MMP2_TWSI4, 1);
	apbc_set_rst(APBC_MMP2_TWSI5, 1);
	apbc_set_rst(APBC_MMP2_TWSI6, 1);

	pm_apbc_clk_disable();		/* disable clocks of GPIO,
					 * MPMU, IPC, THSES1 in APBC */
	pm_scu_clk_disable();		/* disable clocks in SCU */
	pm_apmu_clk_disable();		/* disable clocks in APMU */
	pm_mpmu_clk_disable();		/* disable clocks in MPMU */

	printk(KERN_INFO "%s: before suspend\n", __func__);
	cpu_do_idle();
	printk(KERN_INFO "%s: after suspend\n", __func__);

	pm_mpmu_clk_enable();		/* enable clocks in MPMU */
	pm_apmu_clk_enable();		/* enable clocks in APMU */
	pm_scu_clk_enable();		/* enable clocks in SCU */
	pm_apbc_clk_enable();		/* enable clocks of GPIO,
					 * MPMU, IPC, THSES1 in APBC */

	/* clear RST in APBC_TWSIx_CLK_RST registers */
	__raw_writel(0x3, APBC_MMP2_TWSI1);	/* enable TWSI1 clk */
	apbc_set_rst(APBC_MMP2_TWSI2, 0);
	apbc_set_rst(APBC_MMP2_TWSI3, 0);
	apbc_set_rst(APBC_MMP2_TWSI4, 0);
	apbc_set_rst(APBC_MMP2_TWSI5, 0);
	apbc_set_rst(APBC_MMP2_TWSI6, 0);

	if (__raw_readl(MPMU_AWUCRS) & PMUM_WAKEUP7)
		pmic_wakeup_detect = 1;
	if (__raw_readl(MPMU_AWUCRS) & PMUM_WAKEUP2)
		sdh_wakeup_detect = 1;

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mmp2_pm_prepare(void)
{
	mmp2_pm_enter_lowpower_mode(POWER_MODE_SYS_SLEEP);
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mmp2_pm_finish(void)
{
	mmp2_pm_enter_lowpower_mode(POWER_MODE_CORE_INTIDLE);
}

static void mmp2_pm_wake(void)
{
	if (pmic_wakeup_detect)
		wake_lock_timeout(&wakelock_pmic_wakeup, HZ * 2);
	pmic_wakeup_detect = 0;
	if (sdh_wakeup_detect)
		wake_lock_timeout(&wakelock_sdh_wakeup, HZ * 3);
	sdh_wakeup_detect = 0;
}

static int mmp2_pm_valid(suspend_state_t state)
{
	return ((state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM));
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static const struct platform_suspend_ops mmp2_pm_ops = {
	.valid		= mmp2_pm_valid,
	.prepare	= mmp2_pm_prepare,
	.enter		= mmp2_pm_enter,
	.wake		= mmp2_pm_wake,
	.finish		= mmp2_pm_finish,
};

static int __init mmp2_pm_init(void)
{
	uint32_t apcr;

	if (!cpu_is_mmp2())
		return -EIO;

	suspend_set_ops(&mmp2_pm_ops);

	/* Clear default low power control bit */
	apcr = __raw_readl(MPMU_APCR);
	apcr &= ~PMUM_SLPEN & ~PMUM_DDRCORSD & ~PMUM_APBSD & ~PMUM_AXISD;
	__raw_writel(apcr, MPMU_APCR);

	wake_lock_init(&wakelock_pmic_wakeup,
		WAKE_LOCK_SUSPEND, "wakelock_pmic_wakeup");
	wake_lock_init(&wakelock_sdh_wakeup,
		WAKE_LOCK_SUSPEND, "wakelock_sdh_wakeup");
	return 0;
}

late_initcall(mmp2_pm_init);
