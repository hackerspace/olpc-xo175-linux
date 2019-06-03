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
#include <mach/regs-ciu.h>
#include <mach/regs-smc.h>
#include <mach/regs-timers.h>
#include <mach/mmp2_pm.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/io.h>

static int pmic_wakeup_detect;
static int sdh_wakeup_detect;

struct mmp2_pm_info *mmp2_pm_info_p;
#define MMP2_PMUM_BASE	0xd4050000
#define MMP2_PMUM_END	0xd4051050
#define MMP2_PMUA_BASE	0xd4282800
#define MMP2_PMUA_END	0xd4282920
#define	MMP2_DMCU_BASE	0xd0000000
#define MMP2_DMCU_END	0xd0001000

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
	apcr &= ~(PMUM_SLPEN | PMUM_DDRCORSD | PMUM_APBSD | PMUM_AXISD | 1<<13);
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
		idle_cfg |= PMUA_MOH_PWRDWN;		/* PJ power down */
		apcr |= PMUM_DTCMSD;
		/* fall through */
	case POWER_MODE_CORE_EXTIDLE:
		idle_cfg |= PMUA_MOH_IDLE;		/* set the IDLE bit */
		idle_cfg &= ~(0x3<<28);
		idle_cfg |= 0x000a0000;
		break;
	case POWER_MODE_CORE_INTIDLE:
		apcr &= ~PMUM_DTCMSD;
		break;
	}

	/* set DSPSD, DTCMSD, BBSD, MSASLPEN */
	apcr |= PMUM_DSPSD | PMUM_BBSD;

	/* finally write the registers back */
	__raw_writel(idle_cfg, APMU_IDLE_CFG);
	__raw_writel(apcr, MPMU_APCR);	/* 0xfe086000 */
}

/************************************************************
 *	WTM related code start
 ***********************************************************/
struct wtm_cmd {
	volatile unsigned int prim_cmd_parm0;          /* 0x0  */
	volatile unsigned int prim_cmd_parm1;          /* 0x4  */
	volatile unsigned int prim_cmd_parm2;          /* 0x8  */
	volatile unsigned int prim_cmd_parm3;          /* 0xc  */
	volatile unsigned int prim_cmd_parm4;          /* 0x10 */
	volatile unsigned int prim_cmd_parm5;          /* 0x14 */
	volatile unsigned int prim_cmd_parm6;          /* 0x18 */
	volatile unsigned int prim_cmd_parm7;          /* 0x1c */
	volatile unsigned int prim_cmd_parm8;          /* 0x20 */
	volatile unsigned int prim_cmd_parm9;          /* 0x24 */
	volatile unsigned int prim_cmd_parm10;         /* 0x28 */
	volatile unsigned int prim_cmd_parm11;         /* 0x2c */
	volatile unsigned int prim_cmd_parm12;         /* 0x30 */
	volatile unsigned int prim_cmd_parm13;         /* 0x34 */
	volatile unsigned int prim_cmd_parm14;         /* 0x38 */
	volatile unsigned int prim_cmd_parm15;         /* 0x3c */
	volatile unsigned int secure_processor_cmd;    /* 0x40 */
};

/*
 * WTM register file for host communication
 */
struct wtm_mail_box {
	volatile unsigned int prim_cmd_parm0;          /* 0x0  */
	volatile unsigned int prim_cmd_parm1;          /* 0x4  */
	volatile unsigned int prim_cmd_parm2;          /* 0x8  */
	volatile unsigned int prim_cmd_parm3;          /* 0xc  */
	volatile unsigned int prim_cmd_parm4;          /* 0x10 */
	volatile unsigned int prim_cmd_parm5;          /* 0x14 */
	volatile unsigned int prim_cmd_parm6;          /* 0x18 */
	volatile unsigned int prim_cmd_parm7;          /* 0x1c */
	volatile unsigned int prim_cmd_parm8;          /* 0x20 */
	volatile unsigned int prim_cmd_parm9;          /* 0x24 */
	volatile unsigned int prim_cmd_parm10;         /* 0x28 */
	volatile unsigned int prim_cmd_parm11;         /* 0x2c */
	volatile unsigned int prim_cmd_parm12;         /* 0x30 */
	volatile unsigned int prim_cmd_parm13;         /* 0x34 */
	volatile unsigned int prim_cmd_parm14;         /* 0x38 */
	volatile unsigned int prim_cmd_parm15;         /* 0x3c */
	volatile unsigned int secure_processor_cmd;    /* 0x40 */
	volatile unsigned char reserved_0x44[60];
	volatile unsigned int cmd_return_status;       /* 0x80 */
	volatile unsigned int cmd_status_0;            /* 0x84 */
	volatile unsigned int cmd_status_1;            /* 0x88 */
	volatile unsigned int cmd_status_2;            /* 0x8c */
	volatile unsigned int cmd_status_3;            /* 0x90 */
	volatile unsigned int cmd_status_4;            /* 0x94 */
	volatile unsigned int cmd_status_5;            /* 0x98 */
	volatile unsigned int cmd_status_6;            /* 0x9c */
	volatile unsigned int cmd_status_7;            /* 0xa0 */
	volatile unsigned int cmd_status_8;            /* 0xa4 */
	volatile unsigned int cmd_status_9;            /* 0xa8 */
	volatile unsigned int cmd_status_10;           /* 0xac */
	volatile unsigned int cmd_status_11;           /* 0xb0 */
	volatile unsigned int cmd_status_12;           /* 0xb4 */
	volatile unsigned int cmd_status_13;           /* 0xb8 */
	volatile unsigned int cmd_status_14;           /* 0xbc */
	volatile unsigned int cmd_status_15;           /* 0xc0 */
	volatile unsigned int cmd_fifo_status;         /* 0xc4 */
	volatile unsigned int host_interrupt_register; /* 0xc8 */
	volatile unsigned int host_interrupt_mask;     /* 0xcc */
	volatile unsigned int host_exception_address;  /* 0xd0 */
	volatile unsigned int sp_trust_register;       /* 0xd4 */
	volatile unsigned int wtm_identification;      /* 0xd8 */
	volatile unsigned int wtm_revision;            /* 0xdc */
};

#define WTM_BASE                        0xD4290000
#define WTM_PRIM_CMD_COMPLETE_MASK      (1 << 0)

unsigned int mmp2_ack_from_wtm   = 0x0;
static volatile struct wtm_mail_box *wtm_mb;

static int wtm_exe_cmd(struct wtm_cmd *cmd)
{
	int i;

	volatile unsigned int *pcmd = &cmd->prim_cmd_parm0;
	volatile unsigned int *phi = &wtm_mb->prim_cmd_parm0;

	for (i = 0; i <= 16; i++)
		*phi++ = *pcmd++;

	/* try 10000 times */
	for (i = 0; i < 10000; i++) {
		if (wtm_mb->host_interrupt_register &
				WTM_PRIM_CMD_COMPLETE_MASK) {
			/* clean interrupt */
			wtm_mb->host_interrupt_register = 0xFFFFFFFF;
			return wtm_mb->cmd_return_status;
		}
		udelay(1);
	}

	/* read fail */
	return -1;
}

#define WTM_GET_TRUST_STATUS_REGISTER   0x1000
static int wtm_trig_wtmirq(void)
{
	struct wtm_cmd cmd;
	int status;

	memset(&cmd, 0, sizeof(cmd));
	cmd.secure_processor_cmd = WTM_GET_TRUST_STATUS_REGISTER;
	status = wtm_exe_cmd(&cmd);
	if (status < 0) {
		mmp2_ack_from_wtm = 0;
		goto out;
	}

	mmp2_ack_from_wtm   = 1;

out:
	if (!mmp2_ack_from_wtm)
		printk(KERN_ERR "!!! no ack received from WTM\n");
	return status;
}
/************************************************************
 *	WTM related code end
 ***********************************************************/

void mmp2_cpu_do_idle(void)
{
	if ((mmp2_pm_info_p->dmcu_base) && (mmp2_pm_info_p->pm_vaddr))
		jump_to_lp_sram((mmp2_pm_info_p->pm_vaddr),
					mmp2_pm_info_p->dmcu_base);
	else
		cpu_do_idle();
}

static int mmp2_pm_enter(suspend_state_t state)
{
	int temp;

	temp = __raw_readl(MMP2_ICU_INT4_MASK);
	if (temp & (1<<1)) {
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
	mmp2_cpu_do_idle();
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
	{
		u32 temp = 0;

		/*
		 * 1. trigger irq to wake up SP
		 * 2. disable SPSD of PMUM_PCR_PJ
		 * 3. check whether SP is in ext idle, if so, repeat step 1 and 2
		 */

		do {
			if (temp & (0x1 << 29))
				printk(KERN_ERR "SP in ext idle, repeat step 1 and step 2\n");
			wtm_trig_wtmirq();
			temp = __raw_readl(MPMU_APSR);
			while (!(temp & (0x1 << 26)))
				temp = __raw_readl(MPMU_APSR);
		} while (temp & (0x1 << 29));
	}

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

	/*
	 * Set bit 0, Slow clock Select 32K clock input instead of VCXO
	 * VCXO is chosen by default, which would be disabled in suspend
	 */
	__raw_writel(0x5, MPMU_SCCR);

	/*
	 * Clear bit 23 of CIU_CPU_CONF
	 * direct PJ4 to DDR access through Memory Controller slow queue
	 * fast queue has issue and cause lcd will flick
	 */
	__raw_writel(__raw_readl(CIU_CPU_CONF) & ~(0x1<<23), CIU_CPU_CONF);

	/* Clear default low power control bit */
	apcr = __raw_readl(MPMU_APCR);
	apcr &= ~(PMUM_SLPEN | PMUM_DDRCORSD | PMUM_APBSD | PMUM_AXISD | 1<<13);
	__raw_writel(apcr, MPMU_APCR);

	wake_lock_init(&wakelock_pmic_wakeup,
		WAKE_LOCK_SUSPEND, "wakelock_pmic_wakeup");
	wake_lock_init(&wakelock_sdh_wakeup,
		WAKE_LOCK_SUSPEND, "wakelock_sdh_wakeup");

	/* do initialization about FC seq */
	if (!(mmp2_pm_info_p = kzalloc(sizeof(struct mmp2_pm_info), GFP_KERNEL))) {
		pr_err("failed to request mem for mmp2_pm_info\n");
		goto err;
	}
	mmp2_pm_info_p->pmum_base = ioremap(MMP2_PMUM_BASE, MMP2_PMUM_END - MMP2_PMUM_BASE + 1);
	mmp2_pm_info_p->pmua_base = ioremap(MMP2_PMUA_BASE, MMP2_PMUA_END - MMP2_PMUA_BASE + 1);
	mmp2_pm_info_p->dmcu_base = ioremap(MMP2_DMCU_BASE, MMP2_DMCU_END - MMP2_DMCU_BASE + 1);
	mmp2_pm_info_p->fc_vaddr = (void *)FC_VIRT_BASE;
	mmp2_pm_info_p->fc_vstack = (void *)FC_VIRT_BASE + 0x800;
	mmp2_pm_info_p->pm_vaddr = (void *)FC_VIRT_BASE + 0xc00;

	copy_lp_to_sram(mmp2_pm_info_p->pm_vaddr);

	/* init mail box address for wtm */
	wtm_mb = (volatile struct wtm_mail_box *)ioremap(WTM_BASE, sizeof(struct wtm_mail_box));

	return 0;
err:
	return -ENOMEM;
}

early_initcall(mmp2_pm_init);
