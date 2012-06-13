/*
 * arch/arm/mach-pxa/include/mach/pxa95x_pm.c
 *
 * PXA95x Power Management Routines
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef DEBUG
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
#include <mach/pxa3xx-regs.h>
#include <mach/regs-rtc.h>
#include <mach/regs-intc.h>
#include <mach/regs-ost.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa3xx.h>
#include <mach/gpio.h>
#include <mach/debug_pm.h>
#ifdef CONFIG_ISPT
#include <mach/pxa_ispt.h>
#endif
#include <mach/pxa95x_pm.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/soc_vmeta.h>
#ifdef CONFIG_ARMV7_OS_SAVE_AND_RESTORE
#include <asm/hardware/armv7_jtag.h>
#endif
#ifdef CONFIG_PXA_MIPSRAM
#include <mach/pxa_mips_ram.h>
#endif
#include <mach/pxa9xx_pm_logger.h>	/* for pm debug tracing */
#include <mach/dma.h>
#include <asm/mach/map.h>
#include <mach/ca9_asm.h>
#include <linux/wakelock.h>

/* mtd.h declares another DEBUG macro definition */
#undef DEBUG
#include <linux/mtd/mtd.h>
/* The first 32KB is reserved and can't be accessed by kernel.
 * This restrict is only valid on BootROM V2.
 */
#define ISRAM_START	0x5c000000

/* for OVERHEATING */
#define FRQ_TEMP	(HZ / 100)

enum {
	OVH_TEMP_40C = 0,
	OVH_TEMP_80C,
	OVH_TEMP_85C,
	OVH_TEMP_90C,
	OVH_TEMP_95C,
	OVH_TEMP_100C,
	OVH_TEMP_105C,
	OVH_TEMP_110C,
};

#define OVH_OTIS_DEFAULT	OVH_TEMP_100C
#define OVH_OTIS_LOW_THRES	OVH_TEMP_90C
#define OVH_OVWF_DEFAULT	OVH_TEMP_105C
#define TSS_THRESHOLD		OVH_OTIS_DEFAULT
#define OVH_TO_TEMP_CONVERT(x) (((x - 1) * 5) + 80)
#define OVH_OTIS_LOW_THRES_PXA978	0xBE /*~90C*/
#define OVH_OTIS_DEFAULT_PXA978		0xB1 /*~100C*/
#define OVH_OVWF_DEFAULT_PXA978		0xAA /*~105C*/
#define OVH_TO_TEMP_CONVERT_PXA978(x) ((3153000 - (x * 10000)) / 13825)
static struct timer_list temp_detecting_timer;
static struct work_struct overheating_work;
static int overheating_status;
static int temp_of_core;

static int isram_size;
unsigned int is_wkr_mg1_1274_value;
EXPORT_SYMBOL(is_wkr_mg1_1274_value);	/*this is used in LPM entry and exit */

static struct wake_lock system_wakeup;

/* Counter Structure for Debugging ENTER/EXIT D2/CGM */
extern pxa95x_DVFM_LPM_Global_Count DVFMLPMGlobalCount;
extern int d2_led_toggle_flag;

extern int ForceVCTCXO_EN;
extern int EnableD2VoltageChange;
extern unsigned int D2voltageLevelValue;
extern int cur_op;

#define VLSCR_D2_VALUE (VLSCR_LVL2_SINGLE_RAIL | VLSCR_LVL3_SINGLE_RAIL \
		| VLSCR_LPM_SINGLE_RAIL)
#define VLSCR_D1_VALUE (VLSCR_LVL2_SINGLE_RAIL | VLSCR_LVL3_SINGLE_RAIL)
#define VLSCR_SINGLE_RAIL_MASK (VLSCR_LPM_SINGLE_RAIL \
		| VLSCR_LVL1_SINGLE_RAIL | VLSCR_LVL2_SINGLE_RAIL \
		| VLSCR_LVL3_SINGLE_RAIL | VLSCR_LVL0_SINGLE_RAIL \
		| VLSCR_VCT0_LVL0_REMAP_MASK | VLSCR_VCT0_LVL1_REMAP_MASK \
		| VLSCR_VCT0_LVL2_REMAP_MASK | VLSCR_VCT0_LVL3_REMAP_MASK)

void mmc_jira_2339_wr_before_lpm(void);
void mmc_jira_2339_wr_after_lpm(void);

/* masks of the reserved bits */
unsigned int ckena_rsvd_bit_mask;
unsigned int ckenb_rsvd_bit_mask;
unsigned int ckenc_rsvd_bit_mask;
static void cken_rsvd_bit_mask_setup(void)
{
	if (cpu_is_pxa978()) {
		ckena_rsvd_bit_mask = 0x00000001;
		ckenb_rsvd_bit_mask = 0x97FCF040;
		ckenc_rsvd_bit_mask = 0x00C00000;
	} else {
		ckena_rsvd_bit_mask = 0x00080001;
		ckenb_rsvd_bit_mask = 0x97FCF040;
		ckenc_rsvd_bit_mask = 0x00C00000;
	}
}

#ifdef CONFIG_INPUT_88PM8XXX_ONKEY
extern void pm8xxx_system_poweroff(void);
#endif

int PowerDisabled;		/* enables/disables pm */

#ifdef CONFIG_PXA9XX_ACIPC
extern u32 set_DDR_avail_flag(void);
extern u32 clear_DDR_avail_flag(void);
extern u32 get_acipc_pending_events(void);
extern void acipc_start_cp_constraints(void);
extern int acipc_handle_DDR_req_relq(void);
#endif

#define CHECK_APPS_COMM_SYNC						\
{									\
	u32 iir_tmp;							\
	iir_tmp = get_acipc_pending_events();				\
	if (iir_tmp != 0)						\
		printk(KERN_ERR "CHECK_APPS_COMM_SYNC %x\n", iir_tmp);	\
}

enum pxa95x_pm_mode {
	PXA95x_PM_RUN = 0,
	PXA95x_PM_IDLE = 1,
	PXA95x_PM_LCDREFRESH = 2,
	PXA95x_PM_STANDBY = 3,
	PXA95x_PM_SLEEP = 6,
	PXA95x_PM_DEEPSLEEP = 7,
	PXA95x_PM_CG = 8,
};

extern struct kobject *power_kobj;

pm_wakeup_src_t wakeup_src;	/* interface to set up wakeup_src */
EXPORT_SYMBOL(wakeup_src);

static pm_wakeup_src_t waked;	/* It records the latest wakeup source */

static struct pxa95x_peripheral_wakeup_ops *wakeup_ops;

/* Flag of reseting CP */
unsigned int pm_cp;

#ifdef CONFIG_ISPT
#define ispt_power_state_d2() ispt_power_msg(CT_P_PWR_STATE_ENTRY_D2)
#define ispt_power_state_d1() ispt_power_msg(CT_P_PWR_STATE_ENTRY_D1)
#define ispt_power_state_cgm() ispt_power_msg(CT_P_PWR_MODE_ENTRY_CGM)
#define ispt_power_state_exit_lpm() ispt_power_msg(CT_P_PWR_STATE_ENTRY_C0)
#else
static int __attribute__ ((unused)) ispt_power_state_d2(void)
{
	return 0;
}

static int __attribute__ ((unused)) ispt_power_state_d1(void)
{
	return 0;
}

static int __attribute__ ((unused)) ispt_power_state_cgm(void)
{
	return 0;
}

static int __attribute__ ((unused)) ispt_power_state_exit_lpm(void)
{
	return 0;
}
#endif

#ifdef CONFIG_IPM
int enable_deepidle;		/* IDLE_D0 -- 0 */
int save_deepidle;

void (*event_notify) (int, int, void *, unsigned int);
EXPORT_SYMBOL(event_notify);

static void (*orig_poweroff) (void);
#endif

/* low level stanby and lcdrefresh routine need to access DMC regs */
unsigned char __iomem *dmc_membase;
EXPORT_SYMBOL(dmc_membase);
unsigned char __iomem *ost_membase;
EXPORT_SYMBOL(ost_membase);
unsigned char __iomem *pm_membase;
unsigned int __iomem *pl310_membase;
unsigned int __iomem *remap_c2_reg;

extern void pxa95x_cpu_sleep(unsigned int, unsigned int);
extern void pxa95x_cpu_resume(void);
extern void pxa95x_init_standby(unsigned int);
extern void pxa95x_pm_clear_Dcache_L2Cache(void);
extern void pxa95x_pm_invalidate_Dcache_L2Cache(void);

#ifdef CONFIG_IPM
static struct clk *clk_tout_s0, *clk_pout, *clk_gc;
#endif

static struct pxa95x_pm_regs pxa95x_pm_regs;

static unsigned long pm_state;

/* for pm logger print */
static unsigned int lpm_exit_timestamp;
static void pm_log_wakeup_reason(unsigned int wakeup_data);
static unsigned int pxa95x_get_gwsr(int reg_num);

static u32 disable_sram_use;
int get_pm_state(void)
{
	return pm_state;
}

/*************************************************************************/
/* workaround for bug JIRA MG1-1358 */
int is_wkr_mg1_1358(void)
{
	unsigned int cpuid;
	cpuid = read_cpuid(CPUID_ID);
	/*
	 * This WA is relevant to PXA955 as it is changing the GC bits
	 * and can go to D2 from High PP. PXA950 does not change these bits
	 * because they are reserved but this JIRA is relevant for PXA950 as
	 * well since bit 21 is controlling 13M D2 enable functionality
	 */
	if (cpu_is_pxa978())
		return 0;
	/* It's PXA955 */
	if ((cpuid & 0x0000FFF0) == 0x5810)
		return 1;
	/* It's PXA950 */
	if ((cpuid & 0x0000FFF0) == 0x6970)
		return 1;
	return 0;
}

/* workaround for bug JIRA MG2-388 */
int is_wkr_mg2_388(void)
{
	int wkr = ((cpu_is_pxa968() && !(cpu_is_pxa968_Ax())));

	return wkr;
}

/* workaround for bug JIRA MG1-1677 */
int is_wkr_mg1_1677(void)
{
	return cpu_is_pxa955_E0();
}

/* workaround for bug JIRA NEVO-2368 */
int is_wkr_nevo_2368(void)
{
	return cpu_is_pxa978();
}

/* workaround for bug JIRA NEVO-2339 */
int is_wkr_nevo_2339(void)
{
	return cpu_is_pxa978();
}

static u32 ddadr[32], dtadr[32], dsadr[32], dcmd[32];
void save_dma_registers(void)
{
	int i;
	for (i = 0; i < 32; i++) {
		ddadr[i] = DDADR(i);
		dtadr[i] = DTADR(i);
		dsadr[i] = DSADR(i);
		dcmd[i] = DCMD(i);
	}
}

void restore_dma_registers(void)
{
	int i;
	for (i = 0; i < 32; i++) {
		if (DCSR(i) & DCSR_NODESC) {
			DTADR(i) = dtadr[i];
			DSADR(i) = dsadr[i];
			DCMD(i) = dcmd[i];
		}
	}
}

/*************************************************************************/
/* workaround for bug JIRA MG1-1274 */
int is_wkr_mg1_1274(void)
{
	return 0;
}

static unsigned int lpm_with_axi_on;
void enable_axi_lpm_entry(void)
{
	if ((CKENC & ((1 << (CKEN_AXI - 64)) |
		      (1 << (CKEN_AXI_2X - 64)))) == 0x0) {
		/* notify WA is used */
		lpm_with_axi_on = 1;
		/* turning on the AXI clocks */
		CKENC |= ((1 << (CKEN_AXI - 64)) | (1 << (CKEN_AXI_2X - 64)));
	}
}

void enable_axi_lpm_exit(void)
{
	if (lpm_with_axi_on) {
		lpm_with_axi_on = 0;
		/* turning off the AXI clocks */
		CKENC &= ~((1 << (CKEN_AXI - 64)) | (1 << (CKEN_AXI_2X - 64)));
	}
}

int is_wkr_mg1_1468(void)
{
	return cpu_is_pxa95x();
}

static void pxa95x_sysbus_init(struct pxa95x_pm_regs *context)
{
	context->smc.membase = ioremap(SMC_START, SMC_END - SMC_START + 1);
	context->arb.membase = ioremap(ARB_START, ARB_END - ARB_START + 1);

	dmc_membase = ioremap(DMC_START, DMC_END - DMC_START + 1);
	ost_membase = ioremap(OST_START, OST_END - OST_START + 1);
	pm_membase = ioremap(PM_START, PM_END - PM_START + 1);

	isram_size = (128 * 1024);

	context->sram_map = __arm_ioremap(ISRAM_START, isram_size, MT_MEMORY_NONCACHED);
	context->sram = vmalloc(isram_size);
	/* Two words begun from 0xC0000000 are used to store key information.
	 */
	context->data_pool = (unsigned char *)0xC0000000;
	if (cpu_is_pxa978()) {
		pl310_membase = ioremap(0x58120000, 0x1000);
	}
}


static void pxa95x_pm_set_clk(char *id, int enable)
{
	struct clk *clk;

	clk = clk_get(NULL, id);
	if (IS_ERR(clk)) {
		printk(KERN_INFO "clk_get failed when getting \"%s\"\n", id);
		return;
	}

	if (enable)
		clk_enable(clk);
	else
		clk_disable(clk);
}

/* This function is used to clear power manager status.
 */
static void pxa95x_clear_pm_status(int sys_level)
{
	unsigned int tmp;

	if (sys_level && !cpu_is_pxa978()) {
		/* clear power manager status */
		tmp = PSR;
		tmp &= PSR_MASK;
		PSR = tmp;
	}
	/* clear application system status */
	tmp = ASCR;
	tmp |= ASCR_MASK;
	ASCR = tmp;
	/* clear all application subsystem reset status */
	tmp = ARSR;
	ARSR = tmp;
}

/*
 * Clear the wakeup source event.
 */
static void pm_clear_wakeup_src(pm_wakeup_src_t src)
{
	if (wakeup_ops->ext)
		wakeup_ops->ext(src, 0);
	if (wakeup_ops->key)
		wakeup_ops->key(src, 0);
	if (wakeup_ops->mmc1)
		wakeup_ops->mmc1(src, 0);
	if (wakeup_ops->mmc3)
		wakeup_ops->mmc3(src, 0);
	if (wakeup_ops->uart)
		wakeup_ops->uart(src, 0);
	if (wakeup_ops->eth)
		wakeup_ops->eth(src, 0);
	if (wakeup_ops->tsi)
		wakeup_ops->tsi(src, 0);
}

static void pm_select_wakeup_src(enum pxa95x_pm_mode lp_mode,
				 pm_wakeup_src_t src)
{
	unsigned int tmp, reg_src = 0;

	if (!wakeup_ops)
		printk(KERN_ERR "ERROR: wakeup_ops is still NULL!\n");
	if (wakeup_ops->ext)
		reg_src |= wakeup_ops->ext(src, 1);
	if (wakeup_ops->key)
		reg_src |= wakeup_ops->key(src, 1);
	if (wakeup_ops->mmc1)
		reg_src |= wakeup_ops->mmc1(src, 1);
	if (wakeup_ops->mmc3)
		reg_src |= wakeup_ops->mmc3(src, 1);
	if (wakeup_ops->uart)
		reg_src |= wakeup_ops->uart(src, 1);
	if (wakeup_ops->eth)
		reg_src |= wakeup_ops->eth(src, 1);
	if (wakeup_ops->tsi)
		reg_src |= wakeup_ops->tsi(src, 1);
	if (src.bits.rtc) {
		reg_src |= PXA95x_PM_WE_RTC;
	}
	if (src.bits.ost) {
		reg_src |= PXA95x_PM_WE_OST;
	}
	if (src.bits.msl)
		reg_src |= PXA95x_PM_WE_MSL0;

	/* set wakeup register */
	if (lp_mode == PXA95x_PM_SLEEP) {
		if (!cpu_is_pxa978()) {
			PWSR = 0xFFFFFFFF;
			PWER = 0;

			tmp = PWER;
			if (src.bits.rtc)
				tmp |= PWER_WERTC;
			if (src.bits.ext0)
				tmp |= (PWER_WER0 | PWER_WEF0);
			if (src.bits.ext1)
				tmp |= (PWER_WER1 | PWER_WEF1);
			PWER = tmp;
		}
	}
	if (lp_mode == PXA95x_PM_DEEPSLEEP) {
		if (!cpu_is_pxa978()) {
			PWSR = 0xFFFFFFFF;
			PWER = 0;

			tmp = PWER;
			/* RTC cause exit from S3 mode too shortly - TO FIX */
			/*
			   if (src.bits.rtc)
			   tmp |= PWER_WERTC;
			 */
			if (src.bits.ext0)
				tmp |= (PWER_WER0 | PWER_WEF0);
			/* on MG1 there is only one external wake up */
			/*
			   if (src.bits.ext1)
			   tmp |= (PWER_WER1 | PWER_WEF1);
			 */
			PWER = tmp;
		}
	}
	if (lp_mode == PXA95x_PM_STANDBY) {
		AD2D0SR = 0xFFFFFFFF;
		AD2D0ER = 0;
		AD2D0ER = reg_src;
	}
	if (lp_mode == PXA95x_PM_LCDREFRESH) {
		AD1D0SR = 0xFFFFFFFF;
		AD1D0ER = 0;
		/* add the minilcd wakeup event */
		AD1D0ER = reg_src | PXA95x_PM_WE_MLCD;
	}
	if (lp_mode == PXA95x_PM_CG) {
		ACGD0SR = 0xFFFFFFFF;
		/* add the interrupt and dmemc wakeup event */
		ACGD0ER = reg_src | PXA95x_PM_WE_INTC | PXA95x_PM_WE_DMC;
	}
}

static unsigned int pm_query_wakeup_src(void)
{
	unsigned int data;

	memset(&waked, 0, sizeof(pm_wakeup_src_t));

	if (ASCR & 0x07) {
		data = ASCR & 0x07;
		ASCR = data;
		switch (data) {
		case 4:
			/* check D1 wakeup source */
			data = AD1D0SR;
			AD1D0SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			break;
		case 2:
			/* check D2 wakeup source */
			data = AD2D0SR;
			AD2D0SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			break;
		case 1:
			/* check D3 wakeup source */
			data = AD3SR;
			AD3SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			if (!cpu_is_pxa978()) {
				data = PWSR;
				PWSR = data;
				if (data & PWSR_EERTC)
					waked.bits.rtc = 1;
				if (data & PWSR_EDR0)
					waked.bits.ext0 = 1;
				if (data & PWSR_EDR1)
					waked.bits.ext1 = 1;
			}
			break;
		}
	} else if (ARSR & 0x04) {
		/* check S3 wakeup source */
		data = ARSR & 0x04;
		ARSR = data;
		if (!cpu_is_pxa978()) {
			data = PWSR;
			PWSR = data;
			if (data & PWSR_EERTC)
				waked.bits.rtc = 1;
			if (data & PWSR_EDR0)
				waked.bits.ext0 = 1;
			if (data & PWSR_EDR1)
				waked.bits.ext1 = 1;
		}
	} else {
		/* check clock gate mode wakeup source */
		data = ACGD0SR;
		ACGD0SR = data;
		if (wakeup_ops->query)
			wakeup_ops->query(data, &waked);
	}

	return data;
}

static void __attribute__ ((unused)) dump_wakeup_src(pm_wakeup_src_t *src)
{
	printk(KERN_DEBUG "wakeup source: ");
	if (src->bits.rtc)
		printk(KERN_DEBUG "rtc, ");
	if (src->bits.ost)
		printk(KERN_DEBUG "ost, ");
	if (src->bits.msl)
		printk(KERN_DEBUG "msl, ");
	if (src->bits.wifi)
		printk(KERN_DEBUG "wifi, ");
	if (src->bits.uart1)
		printk(KERN_DEBUG "uart1, ");
	if (src->bits.uart2)
		printk(KERN_DEBUG "uart2, ");
	if (src->bits.uart3)
		printk(KERN_DEBUG "uart3, ");
	if (src->bits.mkey)
		printk(KERN_DEBUG "mkey, ");
	if (src->bits.dkey)
		printk(KERN_DEBUG "dkey, ");
	if (src->bits.mlcd)
		printk(KERN_DEBUG "mlcd, ");
	if (src->bits.tsi)
		printk(KERN_DEBUG "tsi, ");
	if (src->bits.ext0)
		printk(KERN_DEBUG "ext0, ");
	if (src->bits.ext1)
		printk(KERN_DEBUG "ext1, ");
	if (src->bits.mmc1_cd)
		printk(KERN_DEBUG "mmc1 card detect, ");
	if (src->bits.mmc2_cd)
		printk(KERN_DEBUG "mmc2 card detect, ");
	if (src->bits.mmc3_cd)
		printk(KERN_DEBUG "mmc3 card detect, ");
	if (src->bits.mmc1_dat1)
		printk(KERN_DEBUG "mmc1 dat1, ");
	if (src->bits.mmc2_dat1)
		printk(KERN_DEBUG "mmc2 dat1, ");
	if (src->bits.mmc3_dat1)
		printk(KERN_DEBUG "mmc3 dat1, ");
	if (src->bits.eth)
		printk(KERN_DEBUG "eth, ");
	if (src->bits.cmwdt)
		printk(KERN_DEBUG "comm watchdog, ");
}

void get_wakeup_source(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	memcpy(src, &waked, sizeof(pm_wakeup_src_t));
}
EXPORT_SYMBOL(get_wakeup_source);

int pxa95x_wakeup_register(struct pxa95x_peripheral_wakeup_ops *ops)
{
	wakeup_ops = ops;

	/* set default wakeup source */
	if (wakeup_ops->init)
		wakeup_ops->init(&wakeup_src);

	/* clear the related wakeup source */
	pm_clear_wakeup_src(wakeup_src);

	return 0;
}
EXPORT_SYMBOL(pxa95x_wakeup_register);

void pxa95x_wakeup_unregister(void)
{
	wakeup_ops = NULL;
}
EXPORT_SYMBOL(pxa95x_wakeup_unregister);

/*************************************************************************/
struct os_header {
	int version;
	int identifier;
	int address;
	int size;
	int reserved;
};

static void pm_preset_standby(void)
{
	pxa95x_clear_pm_status(0);
}

static void enter_d2(void)
{
	unsigned int pollreg, vlscr = VLSCR;
	pr_debug("enter D2.\n");
	AD2D0SR = 0xFFFFFFFF;
	AD2D0ER = 0;
	AD2D0ER = PXA95x_PM_WE_MMC3	/* MMC3 Data1 */
		|PXA95x_PM_WE_RTC		/* RTC */
		| PXA95x_PM_WE_MSL0		/* ACS-IPC */
		| PXA95x_PM_WE_GENERIC(3);	/* On-key */
	pm_preset_standby();

	clk_disable(clk_pout);

	if (is_wkr_mg1_1468())
		enable_axi_lpm_entry();

	/* PWRMODE = D2 */
	PWRMODE = (PXA95x_PM_S0D2C2 | PXA95x_PM_I_Q_BIT);
	do {
		pollreg = PWRMODE;
	} while (pollreg != (PXA95x_PM_S0D2C2 | PXA95x_PM_I_Q_BIT));

	/* D2 is using single rail mode */
	vlscr &= ~(VLSCR_SINGLE_RAIL_MASK);
	vlscr |= (VLSCR_D2_VALUE);
	VLSCR = vlscr;

	pxa978_pm_enter(pollreg);

	if (is_wkr_mg1_1468())
		enable_axi_lpm_exit();

	clk_enable(clk_pout);

	pr_debug("exit D2.\n");
}

static void enter_cg(void)
{
	unsigned int pollreg, reg;
	unsigned int cken[3] = {0, 0, 0 };

	pr_debug("enter CG.\n");
	ACGD0SR = 0xFFFFFFFF;
	ACGD0ER = 0;
	ACGD0ER = PXA95x_PM_WE_MMC3	/* MMC3 Data1 */
		|PXA95x_PM_WE_RTC		/* RTC */
		| PXA95x_PM_WE_MSL0		/* ACS-IPC */
		| PXA95x_PM_WE_INTC		/* IRQ or FIQ */
#if 0
		| PXA95x_PM_WE_DMC		/* DMC interrupt */
#endif
		| PXA95x_PM_WE_GENERIC(3);	/* On-key */

	PWRMODE = (PXA978_PM_S0D0CG | PXA95x_PM_I_Q_BIT);
	do {
		pollreg = PWRMODE;
	} while (pollreg != (PXA978_PM_S0D0CG | PXA95x_PM_I_Q_BIT));

	cken[0] = CKENA;
	cken[1] = CKENB;
	cken[2] = CKENC;

	/* writing 1 to AGENP[2] to enable the wakeup detection
	 * window */
	AGENP |= AGENP_SAVE_WK;

	/*
	 * Turn off all clocks except PX1 bus clock,
	 * DMEMC clock and reserved bits
	 */
	CKENA = ckena_rsvd_bit_mask | (1 << 8);
	reg = CKENA;
	CKENB = ckenb_rsvd_bit_mask | (1 << 29);
	reg = CKENB;
	CKENC = ckenc_rsvd_bit_mask;
	reg = CKENC;

	/*
	 * making sure that registers are written with the correct value
	 * before moving on
	 */
	while ((CKENA != (ckena_rsvd_bit_mask | (1 << 8))) ||
			(CKENB != (ckenb_rsvd_bit_mask | (1 << 29))) ||
			(CKENC != ckenc_rsvd_bit_mask)) {
	};
	/* Turn off PX1 bus clock */
	CKENB &= ~(1 << 29);

	pxa978_pm_enter(pollreg);

	/* restore clocks after exiting from clock gated mode */
	CKENA = cken[0];
	reg = CKENA;
	CKENB = cken[1];
	reg = CKENB;
	CKENC = cken[2];
	reg = CKENC;
	pr_debug("exit CG.\n");
}

#define ACS_DDR_REQ (1 << 19)
#define ACS_DDR_260_REQ (1 << 8)
#define ACS_RELQ_OTHERS (1 << 5)
int pxa95x_pm_enter_sleep(struct pxa95x_pm_regs *pm_regs)
{
	unsigned int wakeup_data, icip, icip2, icip3;
	unsigned long flags;
	int delta;

	local_fiq_disable();
	local_irq_save(flags);

	while (1) {
		wakeup_data = 0;
		/* Configure edge detection */
		pm_select_wakeup_src(PXA95x_PM_SLEEP, wakeup_src);
		pr_debug("start loop.\n");
		icip = ICIP, icip2 = ICIP2, icip3 = ICIP3;
		/* check if any IRQ/FIQ pending */
		if (!(icip || icip2 || icip3 || ICFP || ICFP2 || ICFP3)) {
#if defined(CONFIG_PXA9XX_ACIPC)
			delta = dvfm_is_comm_wakep_near();
			/*
			 * checking if comm wakeup is around corner and
			 * enter cg if so.
			 */
			if ((!delta) && (0 == clear_DDR_avail_flag())) {
				CHECK_APPS_COMM_SYNC
#endif
				enter_d2();
#if defined(CONFIG_PXA9XX_ACIPC)
				set_DDR_avail_flag();
			} else {
				enter_cg();
			}
#endif
			wakeup_data = pm_query_wakeup_src();

			pr_debug("Wakeup by 0x%08x.\n", wakeup_data);
			pxa95x_clear_pm_status(0);
			icip = ICIP, icip2 = ICIP2, icip3 = ICIP3;
		} else {
			if (icip & (ACS_RELQ_OTHERS | ACS_DDR_REQ)) {
				pr_debug("New ACIPC event pending: ");
				if (icip & ACS_DDR_REQ)
					pr_debug("DDR REQ.\n");
				else
					pr_debug("DDR RELQ.\n");
			}
			else {
				pr_info("Unexpected interrupt pending. Exit!\n"
						"Unmasked interrupt: 0x%08x, 0x"
						"%08x, 0x%08x.\n", icip & ICMR,
						icip2 & ICMR2, icip3 & ICMR3);
				break;
			}
		}
		/* Clear edge detection */
		pm_clear_wakeup_src(wakeup_src);

		/* Exit suspend when wakeup by other source than ACIPC */
		if (wakeup_data & ~(PXA95x_PM_WE_MSL0)) {
			pr_info("Exit suspend waked by: 0x%08x.\nUnmasked "
					"interrupt: 0x%08x, 0x%08x, 0x%08x.\n",
					wakeup_data, icip & ICMR,
					icip2 & ICMR2, icip3 & ICMR3);
			break;
		}
		/* Exit suspend when ACS_DDR_260_REQ pended */
		if (icip2 & ACS_DDR_260_REQ) {
			pr_info("Exit suspend since hi-freq DDR request.\n");
			break;
		}

#if defined(CONFIG_PXA9XX_ACIPC)
		pr_debug("Handle ACIPC events, interrupts status: ");
		pr_debug("ICIP: 0x%08x, ICIP2: 0x%08x, ICIP3: 0x%08x.\n",
				icip, icip2, icip3);
		/* wakeup by DDR request or relinquish, clear interrupt */
		if (acipc_handle_DDR_req_relq()) {
			break;
		}
#endif
	}

	local_irq_restore(flags);
	local_fiq_enable();
	return 0;
}

#ifdef CONFIG_IPM
static void pxa95x_pm_poweroff(void)
{
	unsigned int mode = PXA95x_PM_DEEPSLEEP;
	unsigned long cser;
	printk(KERN_INFO "Enter pxa95x_pm_poweroff\n");

	pm_select_wakeup_src(mode, wakeup_src);
	/* No need to set CKEN bits in PowerOff path */
	/* pxa95x_pm_set_cken(); */
	pxa95x_clear_pm_status(1);
	__raw_writel(0x0, pm_membase + CSER_OFF);
	while ((cser = __raw_readl(pm_membase + CSER_OFF)) != 0)
		;

	PWRMODE = mode | PXA95x_PM_I_Q_BIT;
	while (PWRMODE != (mode | PXA95x_PM_I_Q_BIT))
		;
	__asm__("dsb");
	__asm__("wfi");
}
#endif

static unsigned int pm_postset_standby(void)
{
	unsigned int wakeup_data;

	wakeup_data = pm_query_wakeup_src();
	pm_log_wakeup_reason(wakeup_data);

	/* clear RDH */
	ASCR &= ~ASCR_RDH;

	pxa95x_clear_pm_status(0);
	pm_clear_wakeup_src(wakeup_src);

	return wakeup_data;
}

int pxa95x_pm_enter_standby(struct pxa95x_pm_regs *pm_regs)
{
	unsigned long ticks = 1;
	unsigned int power_state = PXA95x_PM_S0D2C2;

	pm_select_wakeup_src(PXA95x_PM_STANDBY, wakeup_src);
	pm_preset_standby();
#if defined(CONFIG_PXA9XX_ACIPC)
	if (0 == clear_DDR_avail_flag()) {
		CHECK_APPS_COMM_SYNC
#endif
		    pxa95x_cpu_standby((unsigned int)pm_regs->sram_map + 0x8000,
				       (unsigned int)pm_regs->sram_map +
				       0xa000 - 4, (unsigned int)&ticks,
				       power_state);
#if defined(CONFIG_PXA9XX_ACIPC)
		set_DDR_avail_flag();
	} else
		printk(KERN_ERR "******EDDR WARNING: %s DDR_req=1 shared "
		       "flag=1. should not happen. check the apps-comm"
		       " sync\n", __func__);
#endif
	pm_postset_standby();

	pr_debug("*** made it back from standby\n");

	return 0;
}

unsigned int pm_core_pwdn(unsigned int powerState)
{
	unsigned int cpupwr = 0;

	if (cpu_is_pxa95x() && !(cpu_is_pxa978())) {
		/*This function is called before & after LPM
		   Normal functionallity is that outside of these functions
		   the register will be set to 0x0001 -
		   this will allow C2 with 0x0 counter (see JIRA 1495).
		   when entring D2 this will be set to C2 with 0x3F counter. */
		switch (powerState) {
		case CPU_PDWN_LPM_ENTRY:
			cpupwr = CPU_PDWN_SETALLWAYS | CPU_PDWN_ENABLE;
			break;
		case CPU_PDWN_LPM_EXIT:
			cpupwr = CPU_PDWN_SETALLWAYS | CPU_PDWN_ENABLE;
			break;
		}
		/*configuring direction */
		__raw_writel(cpupwr, (void *)&(CPUPWR));
	}
	return cpupwr;
}

unsigned int user_index;
void vmeta_pwr(unsigned int enableDisable)
{
	unsigned int vmpwr = 0;
	static unsigned int onetime;
	unsigned int vmeta_clk_on = 1 << (CKEN_VMETA - 32);
	vmpwr = VMPWR;

	if (onetime == 0) {
		onetime = 1;
		dvfm_enable_op_name("208M_HF", user_index);
		dvfm_enable_op_name("416M_VGA", user_index);
	}
	if (VMETA_PWR_ENABLE == enableDisable) {
		if (vmpwr & VMPWR_PWR_ST)
			return;	/*Pwr is already on */
		CKENB |= vmeta_clk_on;
		if (cpu_is_pxa978()) {
			VMPWR = 0xc0070000 | VMPWR_SETALLWAYS;
			VMPWR = 0xc0070000 | VMPWR_SETALLWAYS | VMPWR_PWON;
			usleep_range(100, 100);
			VMPWR = 0x70000 | VMPWR_SETALLWAYS | VMPWR_PWON;
			usleep_range(100, 100);
			do {
				vmpwr = VMPWR;
			} while ((vmpwr & (VMPWR_PWR_ST | 1 << 3)) != (VMPWR_PWR_ST | 1 << 3));
		} else {
			VMPWR = VMPWR_SETALLWAYS | VMPWR_PWON;
			do {
				vmpwr = VMPWR;
			} while ((vmpwr & VMPWR_PWR_ST) != VMPWR_PWR_ST);
		}
		CKENB &= ~vmeta_clk_on;
	} else if (VMETA_PWR_DISABLE == enableDisable) {
		if ((vmpwr & VMPWR_PWR_ST) != VMPWR_PWR_ST)
			return;	/*Pwr is already off */
		if (CKENB & vmeta_clk_on) {
			printk(KERN_ERR "VMeta clock is still on, can't power off VMeta !\n");
			BUG_ON(1);
		}
		if (cpu_is_pxa978()) {
			VMPWR = 0x70000 | VMPWR_SETALLWAYS;
			usleep_range(100, 100);
			do {
				vmpwr = VMPWR;
			} while (vmpwr & (VMPWR_PWR_ST | 1 << 3));
		} else
			VMPWR = VMPWR_SETALLWAYS;
	}
}

static struct dvfm_lock dvfm_lock = {
	.lock = __SPIN_LOCK_UNLOCKED(dvfm_lock.lock),
	.dev_idx = -1,
	.count = 0,
};

void gc_pwr(int enableDisable)
{
	unsigned int gcpwr = 0;
	unsigned int gc_clk_on = 1 << (CKEN_GC_1X - 64) | 1 << (CKEN_GC_2X - 64);
	gcpwr = GCPWR;
	if (GC_PWR_ENABLE == enableDisable) {
		dvfm_disable_lowpower(dvfm_lock.dev_idx);
		if (gcpwr & GCPWR_PWR_ST)
			return;	/*Pwr is already on */
		/*gc clock on*/
		CKENC |= gc_clk_on;
		/*gc power on*/
		GCPWR = GCPWR_SETALLWAYS | GCPWR_PWON;
		/* poll for gc power mode status*/
		do {
			gcpwr = GCPWR;
		} while ((gcpwr & GCPWR_PWR_ST) != GCPWR_PWR_ST);
		gcpwr = GCPWR;
		/*De-assert reset*/
		gcpwr |= GCPWR_RST_N;
		GCPWR = gcpwr;
		/*gc clock off*/
		CKENC &= ~(gc_clk_on);
	} else if (GC_PWR_DISABLE == enableDisable) {
		if ((gcpwr & GCPWR_PWR_ST) != GCPWR_PWR_ST)
			return;	/*Pwr is already off */
		if (CKENC & gc_clk_on) {
			printk(KERN_ERR "GC clock is still on, can't power off GC !\n");
			BUG_ON(1);
		}
		/* GCPWR_RST_N,GCPWR_PWON = 0 */
		GCPWR = GCPWR_SETALLWAYS;
		do {
			gcpwr = GCPWR;
		} while ((gcpwr & GCPWR_PWR_ST) == GCPWR_PWR_ST);
		dvfm_enable_lowpower(dvfm_lock.dev_idx);
	}
}
EXPORT_SYMBOL(gc_pwr);

#ifdef CONFIG_PXA95x_DVFM

extern int calc_switchtime(unsigned int, unsigned int);

static unsigned int pm_postset_clockgate(void)
{
	unsigned int wakeup_data;

	wakeup_data = pm_query_wakeup_src();
	pm_log_wakeup_reason(wakeup_data);

	pxa95x_clear_pm_status(0);
	pm_clear_wakeup_src(wakeup_src);

	return wakeup_data;
}

static void pm_log_wakeup_reason(unsigned int wakeup_data)
{
	int i;
	unsigned int regs[6];

	/* print if this is gpio wakeup */
	if (wakeup_data & PXA95x_PM_WE_GENERIC(13)) {
		for (i = 1; i <= 6; i++)
			regs[i-1] = pxa95x_get_gwsr(i);

		pm_logger_app_add_trace(7, PM_WAKEUP_GPIO, lpm_exit_timestamp,
		regs[0], regs[1], regs[2], regs[3],
		regs[4], regs[5]);
	}
}

unsigned int get_sram_base(void)
{
	return (unsigned int) pxa95x_pm_regs.sram_map;
}

unsigned int get_c2_sram_base(void)
{
	return (unsigned int) (pxa95x_pm_regs.sram_map + 0x8000);
}
#if (!defined(CONFIG_MMC_BLOCK_CMD13_AFTER_CMD18))
	#define CONFIG_MMC_BLOCK_CMD13_AFTER_CMD18
#endif
#ifdef CONFIG_MMC_BLOCK_CMD13_AFTER_CMD18

/*
 * For eMMC, use CMD5 to put it into LPM. However, we can do it in
 * MMC driver's suspend callback. Otherwise, the eMMC chip will
 * be deselect later by callback.
 * */
#define PREFER_CMD5 0

u32 mmc_base[3] = {0,0,0};
u32 mmc_last_cmd[3] = {0,0,0};
static void mmc_wait_4_complete(u32 base)
{
	volatile u32 status;

	while(1) {
		status = readl(base+0x30);
		if (status) {
			if (status & 0x1) {
				//printk("Got cmd13 interrupt \n");
			} else {
				printk(KERN_WARNING "Got unexpected error 0x%x \n", status);
			}
			writel(status, base+0x30);
			break;
		}
		udelay(1);
	}
}

#if PREFER_CMD5
static void mmc_sleep(u32 base, int sleep)
{
	u32 arg = 1<<16;//rca = 1,

	if (sleep == 1) {
		/* de-select the card before sleep */
		writel(0, base + 0x8);//set argument
		writew(0x0, base + 0xc);//set transfer mode
		writew((7<<8 | 0x10), base + 0xe);//cmd7, no-response
		mmc_wait_4_complete(base);

		/* sleep */
		writel( (arg | (1<<15)), base + 0x8);//set argument
		writew(0x0, base + 0xc);//set transfer mode
		writew((5<<8 | 0x1b), base + 0xe);//cmd5, r1b
		mmc_wait_4_complete(base);
	//	printk("eMMC sleep\n");
	} else {
		/* awake */
		writel( arg , base + 0x8);//set argument
		writew(0x0, base + 0xc);//set transfer mode
		writew((5<<8 | 0x1b), base + 0xe);//cmd8, r1b
		mmc_wait_4_complete(base);

		/* select the card*/
		writel(arg, base + 0x8);//set argument
		writew(0x0, base + 0xc);//set transfer mode
		writew((7<<8 | 0x12), base + 0xe);//cmd7,r1, no-crc
		mmc_wait_4_complete(base);
	}

	return;
}
#endif

static void mmc_send_status_poll(u32 base)
{
	u8 flags = 0x1a;
	u32 arg = 1<<16;
	u8 op_code = 13;

	writel(arg, base + 0x8);//set argument
	writew(0x0, base + 0xc);//set transfer mode, no special
	writew((op_code<<8 | flags), base + 0xe);//set cmd index and option

	mmc_wait_4_complete(base);

	return;
}

void mmc_clear_int_status(void)
{
	u32 status;
	int i;

	for (i=0; i<3; i++) {
		status = readl(mmc_base[i]+0x30);
		if (status) {
			printk(KERN_WARNING "mmc[%d] clear status error %d \n", i, status);
			writel(status, mmc_base[i]+0x30);
		}
	}
	return;
}

/*
 * return the restore CKEN_C value
 * */
static u32 mmc_clk_enable(void)
{
	volatile u32 mmc_cken;

	mmc_cken = CKENC;
	CKENC = 0xFFFFFFFF;
	udelay(1);
	return mmc_cken;
}

static void mmc_clk_disable(u32 c)
{
	CKENC = c;
	udelay(1);
}

#include <linux/platform_data/pxa_sdhci.h>

#ifdef CONFIG_CPU_PXA978
extern struct sdhci_pxa_platdata mci0_platform_data;
extern struct sdhci_pxa_platdata mci1_platform_data;
extern struct sdhci_pxa_platdata mci2_platform_data;
#endif

static int mmc_wr_check(u32 base)
{
	u32 ier;

	ier = readl(base + 0x38);//read interrupt signal/enable bits;
	if ( !(ier & 0x1)) {
		printk(KERN_WARNING "CMD END not enabled(0x%x) \n", ier);
		printk("CKENC = 0x%x \n", CKENC);
		return -1;
	}

	return 0;
}

void mmc_jira_2339_wr_before_lpm(void)
{
	int i;
	u16 t;
	volatile u32 cken_c;

#ifdef CONFIG_CPU_PXA978
	if (mci0_platform_data.suspended)
		return ;
#endif

	cken_c = mmc_clk_enable();

	for (i=0; i<3; i++) {
		if (mmc_wr_check(mmc_base[i])) {
			mmc_clk_disable(cken_c);
			return ;
		}
		t = readw(mmc_base[i]+0xe);
#if PREFER_CMD5
		if (i==0)  {
			mmc_sleep(mmc_base[i], 1);
			continue;
		}
		if (t & (1<<5)) {
			if (i==1) {
				mmc_send_status_poll(mmc_base[i]);
			} else if (i==2) {
				//send cmd52 here
			}
#else
		if (t & (1<<5)) {
			if (i!=2) {
				mmc_send_status_poll(mmc_base[i]);
			} else {
				//send cmd52 here
			}

#endif
		}
	}
	mmc_clear_int_status();

	mmc_clk_disable(cken_c);
}

void mmc_jira_2339_wr_after_lpm(void)
{
#if PREFER_CMD5
	volatile u32 cken_c;
#endif

#ifdef CONFIG_CPU_PXA978
	if (mci2_platform_data.suspended)
		return ;
#endif
#if PREFER_CMD5
	cken_c = mmc_clk_enable();

	if (mmc_wr_check(mmc_base[0])) {
		mmc_clk_disable(cken_c);
		return;
	}
	mmc_sleep(mmc_base[0], 0);

	mmc_clk_disable(cken_c);
#endif
}
#else
void mmc_jira_2339_wr_before_lpm(void) {};
void mmc_jira_2339_wr_after_lpm(void) {};
#endif

void enter_lowpower_mode(int state)
{
	unsigned int start_tick = 0, end_tick = 0;
	unsigned int cken[3] = {0, 0, 0 }, icmr[3] = {0, 0, 0};
	unsigned int reg, sram, pollreg, aicsr;
	unsigned int accr_gcfs = 0;
	unsigned int accr, acsr;
	unsigned int power_state;
	unsigned int wakeup_data, cpupwr;
	static unsigned int last_d2exit_time;
	unsigned long flags;

	local_fiq_disable();
	local_irq_save(flags);

	if (is_wkr_mg1_1274()) {
		/*
		 * Mask all interrupts except R2B20 BCCU
		 */
		icmr[0] = ICMR;
		icmr[1] = ICMR2;
		icmr[2] = ICMR3;
		ICMR = 0x00000000;
		ICMR2 = 0x00100000;
		ICMR3 = 0x00100000;
		/* go to LPM only if there are no pending interrupts */
		if (((ICPR & icmr[0]) != 0x0) ||
		    ((ICPR2 & icmr[1]) != 0x0) || ((ICPR3 & icmr[2]) != 0x0)) {
			ICMR = icmr[0];
			ICMR2 = icmr[1];
			ICMR3 = icmr[2];
			return;
		}
	}
	/*C2 is entered in power-down states */
	cpupwr = pm_core_pwdn(CPU_PDWN_LPM_ENTRY);
	/* the counter must be disabeld and it value saved before enterinf
	 * lpm */
	mipsram_disable_counter();

#ifdef CONFIG_ARMV7_OS_SAVE_AND_RESTORE
	/* Save JTAG registers before going to low power mode */
	armv7_jtag_registers_save();
#endif

	if (state == POWER_MODE_D1) {
		ispt_power_state_d1();
		power_state = PXA95x_PM_S0D1C2;
		/* D1 is represented by LCDREFRESH */
		pm_select_wakeup_src(PXA95x_PM_LCDREFRESH, wakeup_src);
		/* check if any IRQ/FIQ pending by disable */
		if (!(ICIP || ICIP2 || ICIP3 || ICFP || ICFP2 || ICFP3)) {
			/* need the same action as for D2 */
			if (PXA9xx_Force_D1 == ForceLPM) {
				LastForceLPM = PXA9xx_Force_D1;
				AD1D0ER = ForceLPMWakeup;
			}
			pm_preset_standby();
			end_tick = OSCR4;

			if (is_wkr_mg1_1468())
				enable_axi_lpm_entry();

			if (is_wkr_mg1_1677())
				save_dma_registers();

			if (likely(!ForceVCTCXO_EN)) {
				clk_disable(clk_pout);
				if (is_wkr_nevo_2368())
					OSCC &= ~OSCC_VCTVCEN;
			}
#if defined(CONFIG_PXA9XX_ACIPC)
			if (0 == clear_DDR_avail_flag()) {
				CHECK_APPS_COMM_SYNC
#endif
#ifdef CONFIG_PXA_MIPSRAM
				MIPS_RAM_ADD_32K_TIME_STAMP(end_tick);
				MIPS_RAM_ADD_PM_TRACE(ENTER_D1_MIPS_RAM);
#endif
				/* PWRMODE = D1 */
				PWRMODE = (PXA95x_PM_S0D1C2 | PXA95x_PM_I_Q_BIT);
				do {
					pollreg = PWRMODE;
				} while (pollreg != (PXA95x_PM_S0D1C2 | PXA95x_PM_I_Q_BIT));


				pm_logger_app_add_trace(6, PM_D1_ENTRY, end_tick,
						pollreg, CKENA, CKENB, CKENC,
						get_mipi_reference_control(),
						cpupwr);

				sram = (unsigned int)pxa95x_pm_regs.sram_map;
				if (cpu_is_pxa978()) {
					/* D1 is using dual rail mode */
					unsigned int vlscr = VLSCR;
					vlscr &= ~(VLSCR_SINGLE_RAIL_MASK);
					vlscr |= (VLSCR_D1_VALUE);
					VLSCR = vlscr;
					if (is_wkr_nevo_2339())
						mmc_jira_2339_wr_before_lpm();
					pxa978_pm_enter(pollreg);
					start_tick = OSCR4;
				} else {
					pxa95x_cpu_standby(sram + 0x8000,
							sram + 0xa000 - 4,
							(unsigned int) &start_tick,
							power_state);
				}
#if defined(CONFIG_PXA9XX_ACIPC)
				set_DDR_avail_flag();
			} else
				printk(KERN_WARNING "******EDDR WARNING: %s DDR_req=1 \
						shared flag=1. should not happen. \
						check the apps-comm sync \n", __func__);
#endif
			if (likely(!ForceVCTCXO_EN)) {
				if (is_wkr_nevo_2368())
					OSCC |= OSCC_VCTVCEN;
				clk_enable(clk_pout);
			}

			if (is_wkr_mg1_1677())
				restore_dma_registers();

			if (is_wkr_mg1_1468())
				enable_axi_lpm_exit();
#ifdef CONFIG_PXA_MIPSRAM
			MIPS_RAM_ADD_PM_TRACE(EXIT_D1_MIPS_RAM);
			MIPS_RAM_ADD_32K_TIME_STAMP(start_tick);
#endif
#ifdef CONFIG_PXA95x_DVFM_STATS
			calc_switchtime(end_tick, start_tick);
#endif

			lpm_exit_timestamp = start_tick;
			wakeup_data = pm_postset_standby();
			if (cpu_is_pxa978() && is_wkr_nevo_2339())
				mmc_jira_2339_wr_after_lpm();

			pm_logger_app_add_trace(1, PM_D1_EXIT, start_tick, wakeup_data);
		} else {
			pm_clear_wakeup_src(wakeup_src);
		}
	} else if (state == POWER_MODE_D2) {
		ispt_power_state_d2();
		power_state = PXA95x_PM_S0D2C2;
		pm_select_wakeup_src(PXA95x_PM_STANDBY, wakeup_src);
		/* check if any IRQ/FIQ pending by disable */
		if (!(ICIP || ICIP2 || ICIP3 || ICFP || ICFP2 || ICFP3)) {
			pm_preset_standby();
			end_tick = OSCR4;

			/* pm logger debug - print when long wakeup */
			debug_check_active_time(end_tick, last_d2exit_time);

			/* if VCTCXO_EN is forced we aill not enable shutdown when
			 * entering D2 */
			if (likely(!ForceVCTCXO_EN))
				clk_disable(clk_pout);
			if (PXA9xx_Force_D2 == ForceLPM) {
				/*Setting to forcedD2wakeups */
				LastForceLPM = PXA9xx_Force_D2;
				AD2D0ER = ForceLPMWakeup;
			}
#if defined(CONFIG_PXA9XX_ACIPC)
			if (0 == clear_DDR_avail_flag()) {
				CHECK_APPS_COMM_SYNC
#endif
#ifdef CONFIG_PXA_MIPSRAM
				MIPS_RAM_ADD_32K_TIME_STAMP(end_tick);
				MIPS_RAM_ADD_PM_TRACE(ENTER_D2_MIPS_RAM);
#endif
				if (is_wkr_mg1_1358()) {
					accr_gcfs = ((ACCR & ACCR_GCFS_MASK) >>
							ACCR_GCFS_OFFSET);
					accr = ACCR;
					accr &= ~ACCR_GCFS_MASK;
					ACCR = accr;
					do {
						acsr = ACSR;
					} while (((accr & ACCR_GCFS_MASK) >>
							ACCR_GCFS_OFFSET !=
							((acsr & ACSR_GCFS_MASK)
							 >> ACSR_GCFS_OFFSET)));
				}
				if (is_wkr_mg1_1468())
					enable_axi_lpm_entry();

				if (is_wkr_mg1_1677())
					save_dma_registers();

				/*if (d2_led_toggle_flag) {
				  configuring direction
				  __raw_writel(0x00100000, (void *)&(__REG(0x40e00404)));
				  clearing gpio 52  (turning on D10 led)
				  __raw_writel(0x00100000, (void *)&(__REG(0x40e00028)));
				  } */

				/* PWRMODE = D2 */
				PWRMODE = (PXA95x_PM_S0D2C2 | PXA95x_PM_I_Q_BIT);
				do {
					pollreg = PWRMODE;
				} while (pollreg != (PXA95x_PM_S0D2C2 |
							PXA95x_PM_I_Q_BIT));

				pm_logger_app_add_trace(7, PM_D2_ENTRY, end_tick,
						pollreg, CKENA, CKENB, CKENC,
						OSCC,
						get_mipi_reference_control(),
						cpupwr);

				sram = (unsigned int)pxa95x_pm_regs.sram_map;
				if (cpu_is_pxa978()) { /*Nevo C0*/
					/* D2 is using single rail mode */
					unsigned int vlscr = VLSCR;
					vlscr &= ~(VLSCR_SINGLE_RAIL_MASK);
					vlscr |= (VLSCR_D2_VALUE);
					VLSCR = vlscr;
					if ( is_wkr_nevo_2339())
						mmc_jira_2339_wr_before_lpm();
					pxa978_pm_enter(pollreg);
					start_tick = OSCR4;
				} else {
					pxa95x_cpu_standby(sram + 0x8000,
							sram + 0xa000 - 4,
							(unsigned int) &start_tick,
							power_state);
				}

				if (is_wkr_mg1_1468())
					enable_axi_lpm_exit();

				if (is_wkr_mg1_1677())
					restore_dma_registers();

				if (is_wkr_mg1_1358()) {
					accr = ACCR;
					accr &= ~ACCR_GCFS_MASK;
					accr |= ((accr_gcfs << ACCR_GCFS_OFFSET) &
							ACCR_GCFS_MASK);
					ACCR = accr;
					do {
						acsr = ACSR;
					} while (((accr & ACCR_GCFS_MASK) >>
							ACCR_GCFS_OFFSET !=
							((acsr & ACSR_GCFS_MASK)
							 >> ACSR_GCFS_OFFSET)));
				}

				/* for pm logger debug */
				turn_on_pm_logger_print();

				/* if (d2_led_toggle_flag) {
				   setting gpio 52 to high (turning on D10 led)
				   __raw_writel(0x00100000, (void *)&(__REG(0x40e0001C)));
				   } */
#if defined(CONFIG_PXA9XX_ACIPC)
				set_DDR_avail_flag();
#endif
				DVFMLPMGlobalCount.D2_Enter_Exit_count++;
#ifdef CONFIG_PXA_MIPSRAM
				MIPS_RAM_ADD_PM_TRACE(EXIT_D2_MIPS_RAM);
				MIPS_RAM_ADD_32K_TIME_STAMP(start_tick);
#endif
#if defined(CONFIG_PXA9XX_ACIPC)
			} else
				printk(KERN_ERR "******EDDR WARNING: %s DDR_req=1 "
						"shared flag=1. should not happen. "
						"check the apps-comm sync\n", __func__);
#endif

			/* if forced we will not enabled/disable VCTCXO shutdown */
			if (likely(!ForceVCTCXO_EN))
				clk_enable(clk_pout);
#ifdef CONFIG_PXA95x_DVFM_STATS
			calc_switchtime(end_tick, start_tick);
#endif
			last_d2exit_time = start_tick;
			lpm_exit_timestamp = start_tick;
			wakeup_data = pm_postset_standby();
			if (cpu_is_pxa978() && is_wkr_nevo_2339())
				mmc_jira_2339_wr_after_lpm();

			pm_logger_app_add_trace(1, PM_D2_EXIT, start_tick, wakeup_data);
		} else
			pm_clear_wakeup_src(wakeup_src);
	} else if (state == POWER_MODE_CG) {
		ispt_power_state_cgm();
		pm_select_wakeup_src(PXA95x_PM_CG, wakeup_src);

		/* check if any IRQ/FIQ pending by disable */
		if (!(ICIP || ICIP2 || ICIP3 || ICFP || ICFP2 || ICFP3)) {
			if (cpu_is_pxa955() || cpu_is_pxa968()) {
				/* Enable ACCU internal interrupt */
				ICMR2 |= 0x00100000;
				/* Turn on wakeup to core during idle mode */
				aicsr = AICSR;
				/* enable bit 10 - wakeup during core idle */
				aicsr |= AICSR_WEIDLE;
				/* do not write to status bits (write to clear) */
				aicsr &= ~AICSR_STATUS_BITS;
				AICSR = aicsr;
			}
			if (PXA9xx_Force_CGM == ForceLPM) {
				LastForceLPM = PXA9xx_Force_CGM;
				ACGD0ER = ForceLPMWakeup;
			}
			end_tick = OSCR4;

#ifdef CONFIG_PXA_MIPSRAM
			MIPS_RAM_ADD_32K_TIME_STAMP(end_tick);
			MIPS_RAM_ADD_PM_TRACE(ENTER_CGM_MIPS_RAM);
#endif

			/* PWRMODE = C1 */
			if (cpu_is_pxa978()) {
				PWRMODE = (PXA978_PM_S0D0CG | PXA95x_PM_I_Q_BIT);
				do {
					pollreg = PWRMODE;
				} while (pollreg != (PXA978_PM_S0D0CG | PXA95x_PM_I_Q_BIT));
			} else {
				PWRMODE = (PXA95x_PM_S0D0C1 | PXA95x_PM_I_Q_BIT);
				do {
					pollreg = PWRMODE;
				} while (pollreg != (PXA95x_PM_S0D0C1 | PXA95x_PM_I_Q_BIT));
			}

			cken[0] = CKENA;
			cken[1] = CKENB;
			cken[2] = CKENC;

			pm_logger_app_add_trace(6, PM_CGM_ENTRY, end_tick, pollreg,
					cken[0], cken[1], cken[2],
					get_mipi_reference_control(), cpupwr);

			/* writing 1 to AGENP[2] to enable the wakeup detection
			 * window */
			AGENP |= AGENP_SAVE_WK;

			/*
			 * Turn off all clocks except PX1 bus clock,
			 * DMEMC clock and reserved bits
			 */
			CKENA = ckena_rsvd_bit_mask | (1 << 8);
			reg = CKENA;
			CKENB = ckenb_rsvd_bit_mask | (1 << 29);
			reg = CKENB;
			CKENC = ckenc_rsvd_bit_mask;
			reg = CKENC;

			/*
			 * making sure that registers are written with the correct value
			 * before moving on
			 */
			while ((CKENA != (ckena_rsvd_bit_mask | (1 << 8))) ||
					(CKENB != (ckenb_rsvd_bit_mask | (1 << 29))) ||
					(CKENC != ckenc_rsvd_bit_mask)) {
			};
			/* Turn off PX1 bus clock */
			CKENB &= ~(1 << 29);

			if (is_wkr_mg1_1274())
				pxa95x_pm_clear_Dcache_L2Cache();
			/* enter clock gated mode by entering core idle */
			/*
			   Since timer functional clock is disable on CGM exit, we can't use the original workaround (JIRA )
			   Instead we will use constant delay at CGM exist to insure L2 is ready
			   loop was calibrated to create ~25uSec delay.
			   therefor for PP 2-7 set 6000 loop count and for PP1 set 1500
			   */
			sram = (unsigned int) pxa95x_pm_regs.sram_map;
			if (cpu_is_pxa978()) {
				pxa978_pm_enter(pollreg);
			} else {
				if (cur_op < 2)
					pm_enter_cgm_deepidle
						(CPU_LOOP_COUNT_ON_EXIT_CGM_LOW_PP);
				else
					pm_enter_cgm_deepidle
						(CPU_LOOP_COUNT_ON_EXIT_CGM_HIGH_PP);
			}
			if (is_wkr_mg1_1274())
				pxa95x_pm_invalidate_Dcache_L2Cache();
			/* restore clocks after exiting from clock gated mode */
			CKENA = cken[0];
			reg = CKENA;
			CKENB = cken[1];
			reg = CKENB;
			CKENC = cken[2];
			reg = CKENC;
			if (cpu_is_pxa955() || cpu_is_pxa968()) {
				/* clear AICSR wakeup status */
				aicsr = AICSR;
				/*do not write to status bits (write to clear) */
				aicsr &= ~(AICSR_STATUS_BITS | AICSR_WEIDLE);
				/*enable bit 10 - wakeup during core idle */
				aicsr |= AICSR_WSIDLE;
				AICSR = aicsr;
			}

			start_tick = OSCR4;
			lpm_exit_timestamp = start_tick;
#ifdef CONFIG_PXA95x_DVFM_STATS
			calc_switchtime(end_tick, start_tick);
#endif
			wakeup_data = pm_postset_clockgate();
			DVFMLPMGlobalCount.CGM_Enter_Exit_count++;
#ifdef CONFIG_PXA_MIPSRAM
			MIPS_RAM_ADD_PM_TRACE(EXIT_CGM_MIPS_RAM);
			MIPS_RAM_ADD_32K_TIME_STAMP(start_tick);
			pm_logger_app_add_trace(1, PM_CGM_EXIT, start_tick,
					wakeup_data);
#endif
		} else
			pm_clear_wakeup_src(wakeup_src);
	}
	/* this indicates exit d2 or cgm */
	ispt_power_state_exit_lpm();

	if (is_wkr_mg1_1274()) {
		/* Unmask all masked interrupts within LPM entry */
		ICMR = icmr[0];
		ICMR2 = icmr[1];
		ICMR3 = icmr[2];
	}
#ifdef CONFIG_ARMV7_OS_SAVE_AND_RESTORE
	/* Restore JTAG registers */
	armv7_jtag_registers_restore();
#endif

	/* restoring the performance counter and compensating its value */
	mipsram_reinit_counter();
	/* C2 is entered in power-down states - returning to normal C1 */
	pm_core_pwdn(CPU_PDWN_LPM_EXIT);
	if (!RepeatMode) {
		if (ForceLPM && LastForceLPM == ForceLPM)
			LastForceLPM = ForceLPM = PXA9xx_Force_None;
	}
	local_irq_restore(flags);
	local_fiq_enable();
}
#endif

static int pxa95x_pm_enter(suspend_state_t state)
{
	if (state == PM_SUSPEND_MEM)
		return pxa95x_pm_enter_sleep(&pxa95x_pm_regs);
	else if (state == PM_SUSPEND_STANDBY)
		return pxa95x_pm_enter_standby(&pxa95x_pm_regs);
	else
		return -EINVAL;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
extern int pxa95x_check_constraint(void);
static int pxa95x_pm_prepare(void)
{
	/* Check constriants */
	pxa95x_check_constraint();
	/* Request OP1 before entering suspend.
	 * This is for Nevo C0 silicon issue which can't wakeup core
	 * if core freq > 915MHz. Nevo-2067.
	 * And it can also save power when AP is in D0CG. */
	dvfm_request_op(1);
	pr_debug("Prepare done.\n");

	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void pxa95x_pm_finish(void)
{
	pr_debug("Finish done.\n");
}

static int pxa95x_pm_valid(suspend_state_t state)
{
	int ret = 1;

	if (state == PM_SUSPEND_MEM)
		pm_state = PM_SUSPEND_MEM;
	else if (state == PM_SUSPEND_STANDBY)
		pm_state = PM_SUSPEND_STANDBY;
	else
		ret = 0;
	return ret;
}

static void pxa95x_pm_wake(void)
{
	/* Add 5s wakelock here to make sure the event wakeing
	 * up system can be handled by userspace application.
	 * This is not a good solution and tuning is pended.
	 * TODO
	 */
	wake_lock_timeout(&system_wakeup, HZ * 5);

	pr_debug("PM wake done.\n");
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static const struct platform_suspend_ops pxa95x_pm_ops = {
	.valid = pxa95x_pm_valid,
	.prepare = pxa95x_pm_prepare,
	.enter = pxa95x_pm_enter,
	.finish = pxa95x_pm_finish,
	.wake = pxa95x_pm_wake
};

#define pm_attr(_name, object)						\
static ssize_t _name##_store(struct kobject *kobj,			\
		struct kobj_attribute *attr,				\
		const char *buf, size_t n)				\
{									\
	sscanf(buf, "%u", &object);					\
	return n;							\
}									\
static ssize_t _name##_show(struct kobject *kobj,			\
		struct kobj_attribute *attr,				\
		char *buf)						\
{									\
	return sprintf(buf, "%u\n", object);				\
}									\
static struct kobj_attribute _name##_attr = {				\
	.attr	= {							\
		.name = __stringify(_name),				\
		.mode = 0644,						\
	},								\
	.show	= _name##_show,						\
	.store	= _name##_store,					\
}

#ifdef CONFIG_IPM

static int tokenizer(char **tbuf, const char *userbuf, ssize_t n,
		     char **tokptrs, int maxtoks)
{
	char *cp, *tok;
	char *whitespace = " \t\r\n";
	int ntoks = 0;

	cp = kmalloc(n + 1, GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	*tbuf = cp;
	memcpy(cp, userbuf, n);
	cp[n] = '\0';

	do {
		cp = cp + strspn(cp, whitespace);
		tok = strsep(&cp, whitespace);
		if (tok == NULL)
			break;
		if ((*tok == '\0') || (ntoks == maxtoks))
			break;
		tokptrs[ntoks++] = tok;
	} while (cp);

	return ntoks;
}

static ssize_t deepidle_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	int len = 0;

	if (enable_deepidle & IDLE_D1)
		len += sprintf(buf + len, "D1IDLE, ");
	if (enable_deepidle & IDLE_D2)
		len += sprintf(buf + len, "D2IDLE, ");
	if (enable_deepidle & IDLE_CG)
		len += sprintf(buf + len, "CGIDLE, ");
	len += sprintf(buf + len, "D0IDLE\n");
	len += sprintf(buf + len, "Command: echo [set|unset] [d1|d2|cg] "
		       "> deepidle\n");
	return len;
}

#define MAXTOKENS	80

static ssize_t deepidle_store(struct kobject *kobj,
			      struct kobj_attribute *attr, const char *buf,
			      size_t len)
{
	int error = 0;
	char *tbuf = NULL;
	char *token[MAXTOKENS];
	int ntoks = tokenizer(&tbuf, buf, len, (char **)&token, MAXTOKENS);

	if (ntoks <= 0) {
		error = ntoks;
		goto out;
	}

	if (strcmp(token[0], "set") == 0) {
		if (strcmp(token[1], "d1") == 0)
			enable_deepidle |= IDLE_D1;
		else if (strcmp(token[1], "d2") == 0)
			enable_deepidle |= IDLE_D2;
		else if (strcmp(token[1], "cg") == 0)
			enable_deepidle |= IDLE_CG;
		else
			error = -EINVAL;
	} else if (strcmp(token[0], "unset") == 0) {
		if (strcmp(token[1], "d1") == 0)
			enable_deepidle &= ~IDLE_D1;
		else if (strcmp(token[1], "d2") == 0)
			enable_deepidle &= ~IDLE_D2;
		else if (strcmp(token[1], "cg") == 0)
			enable_deepidle &= ~IDLE_CG;
		else
			error = -EINVAL;
	} else {
		if (strcmp(token[0], "0") == 0)
			enable_deepidle = IDLE_D0;
		else
			error = -EINVAL;
	}
out:
	kfree(tbuf);
	return error ? error : len;
}

static struct kobj_attribute deepidle_attr = {
	.attr = {
		 .name = __stringify(deepidle),
		 .mode = 0644,
		 },
	.show = deepidle_show,
	.store = deepidle_store,
};
#endif

static ssize_t cp_show(struct kobject *kobj, struct kobj_attribute *attr,
		       char *buf)
{
	return sprintf(buf, "%u\n", pm_cp);
}

static int pm_cp_disabled;
static int __init pm_nocp_setup(char *this_opt)
{
	/* Once "nocp" token is present disable CP regardless of the value */
	pm_cp_disabled = 1;
	return 1;
}

__setup("tavorcfg_nocp", pm_nocp_setup);

static ssize_t cp_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	if (!pm_cp_disabled)
		sscanf(buf, "%u", &pm_cp);
#if defined(CONFIG_PXA9XX_ACIPC)
	/* Must be called prior to acipc_start_cp_constraints
	 * This will update acipc driver on cp status */
	set_acipc_cp_enable(pm_cp);
#endif
	if (pm_cp) {
		/* release CP */
		__raw_writel(0x11, pm_membase + CSER_OFF);
#if defined(CONFIG_PXA9XX_ACIPC)
		acipc_start_cp_constraints();
#endif
	} else {
		/* reset CP */
		__raw_writel(0x0, pm_membase + CSER_OFF);
	}
	return len;
}

static struct kobj_attribute cp_attr = {
	.attr = {
		 .name = __stringify(cp),
		 .mode = 0644,
		 },
	.show = cp_show,
	.store = cp_store,
};

static ssize_t temp_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	int len = 0;
	len += sprintf(buf + len, "OVH %x ", OVH);
	len += sprintf(buf + len, "PSR %x ", PSR);
	len += sprintf(buf + len, "temp_of_core %x\n", temp_of_core);
	return len;
}

static struct kobj_attribute temp_attr = {
	.attr = {
		 .name = __stringify(temp),
		 .mode = 0644,
		 },
	.show = temp_show,
};

static ssize_t reg_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int len = 0;

	len += sprintf(buf + len, "ARSR:0x%08x\n", ARSR);
	len += sprintf(buf + len, "PSR:0x%08x\n", PSR);
	len += sprintf(buf + len, "AD1D0SR:0x%08x\n", AD1D0SR);
	len += sprintf(buf + len, "AD2D0SR:0x%08x\n", AD2D0SR);
	len += sprintf(buf + len, "AD3SR:0x%08x\n", AD3SR);
	if (!cpu_is_pxa978())
		len += sprintf(buf + len, "PWSR:0x%08x\n", PWSR);
	len += sprintf(buf + len, "OVH:0x%08x\n", OVH);
	len += sprintf(buf + len, "PMCR:0x%08x\n", PMCR);

	return len;
}

static struct kobj_attribute reg_attr = {
	.attr = {
		 .name = __stringify(reg),
		 .mode = 0644,
		 },
	.show = reg_show,
};

static struct attribute *g[] = {
#ifdef CONFIG_IPM
	&deepidle_attr.attr,
#endif
	&cp_attr.attr,
	&temp_attr.attr,
	&reg_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static void log_overheating_event(unsigned char *str_log)
{
	struct timeval now;

	do_gettimeofday(&now);
	pr_warning("[%10lu]%s PSR.TSS 0x%x\n",
		   now.tv_sec, str_log, temp_of_core);
	/* TODO: record the warning into NVM and nodify the user */
}

static unsigned int overheating_core_cooled_down(unsigned int pmcr,	unsigned int average_meas)
{
	unsigned int ret = 0;
	if (cpu_is_pxa978()) {
		if (!(pmcr & PMCR_TIE) &&
			(average_meas < OVH_TO_TEMP_CONVERT_PXA978( \
			OVH_OTIS_LOW_THRES_PXA978)))
			ret = 1;
	} else /* Not PXA978*/
		if (average_meas < OVH_TO_TEMP_CONVERT(OVH_OTIS_LOW_THRES))
			ret = 1;
	return ret;
}
static void detect_core_temp(unsigned long data)
{
	struct timer_list *timer = &temp_detecting_timer;
	static int conscutive_counter, average_meas;
	unsigned int pmcr = PMCR;
	/* detect the temp of core */
	if (cpu_is_pxa978()) {
		temp_of_core = (PSR >> PSR_TSS_OFF) & 0x1ff;
		temp_of_core = OVH_TO_TEMP_CONVERT_PXA978(temp_of_core);
	} else {
		temp_of_core = (PSR >> PSR_TSS_OFF) & 0x7;
		if (temp_of_core == 0)
			temp_of_core = 80;
		else
			temp_of_core = OVH_TO_TEMP_CONVERT(temp_of_core);
	}

	average_meas += (temp_of_core / 3);
	conscutive_counter++;
	if (conscutive_counter == 3) {
		if (overheating_core_cooled_down(pmcr, average_meas)) {
			log_overheating_event("INFO: AP core cooled down!");
			/* do not clean other status bits
			 * and do not write to sw gpio reset*/
			pmcr &= ~(PMCR_STATUS_BITS|PMCR_RSVD_CLR_ALWAYS_MASK);
			pmcr |= PMCR_TIE;
			PMCR = pmcr;
#ifdef CONFIG_PXA95x_DVFM
			overheating_status = CORE_COLLING_DETECTED;
			schedule_work(&overheating_work);
#endif
		} else {
			/* reset the timer */
			mod_timer(timer, jiffies + FRQ_TEMP);
		}

		pr_debug("%s:PSR 0x%x PMCR 0x%x OVH 0x%x\n", __func__, PSR, pmcr, OVH);
		average_meas = conscutive_counter = 0;
	} else {		/*waiting for 3 consecutive reads */
		/* reset the timer */
		mod_timer(timer, jiffies + FRQ_TEMP);
	}
}

static irqreturn_t core_overhearting_irq(int irq, void *data)
{
	struct timer_list *timer = &temp_detecting_timer;
	unsigned int pmcr = PMCR;
#ifdef CONFIG_PXA95x_DVFM
	overheating_status = CORE_OVERHEATING_DETECTED;
	schedule_work(&overheating_work);
#endif
	pr_debug("%s:PSR 0x%x PMCR 0x%x OVH 0x%x\n", __func__, PSR, pmcr, OVH);
	if (pmcr & PMCR_TIS) {
		/* disable the interrupt & clear the status bit */
		/* do not clean other status bits
		 * and do not write to sw gpio reset*/
		pmcr &= ~(PMCR_STATUS_BITS|PMCR_RSVD_CLR_ALWAYS_MASK|PMCR_TIE);
		/* clean overtemp bit*/
		pmcr |= PMCR_TIS;
		PMCR = pmcr;

		log_overheating_event("WARNING: AP core is OVERHEATING!");

		/* start the timer for measuring the temp of core */
		mod_timer(timer, jiffies + FRQ_TEMP);
	} else
		WARN_ON(1);	/*This should not happen since currently
		only the overtemperature is used as int source of
		MPMU*/
	return IRQ_HANDLED;
}

static void overheating_work_handler(struct work_struct *work)
{
	temperture_sensor_int_high_freq_pp_callback(overheating_status);
}

static void overheating_init(void)
{
	struct timer_list *timer = &temp_detecting_timer;
	int retval;
	unsigned int pmcr;
	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_SGP, core_overhearting_irq,
			     IRQF_DISABLED, "Overheating", NULL);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       "Overheating", IRQ_SGP, retval);
		return;
	}

	if (!cpu_is_pxa978()) {
		OVH = (OVH_OTIS_DEFAULT) << OVH_OTIF_OFF;
		OVH |= (OVH_OVWF_DEFAULT) << OVH_OVWF_OFF;
	} else {
		OVH = (OVH_OTIS_DEFAULT_PXA978) << OVH_OTIF_OFF;
		OVH |= (OVH_OVWF_DEFAULT_PXA978) << OVH_OVWF_OFF;
	}

	pmcr = PMCR;
	/* do not clean other status bits
	 * and do not write to sw gpio reset*/
	pmcr &= ~(PMCR_STATUS_BITS|PMCR_RSVD_CLR_ALWAYS_MASK);
	pmcr |= (PMCR_TIE | PMCR_TIS);
	PMCR = pmcr;
	OVH |= OVH_TEMP_EN;
	OVH |= OVH_OWM;

	/* initialize the timer for measuring the temp of core */
	init_timer(timer);
	timer->function = detect_core_temp;
	INIT_WORK(&overheating_work, overheating_work_handler);
}

static unsigned char __iomem *bpb_membase;

/*
 * Query whether the specified pin waked up system.
 *
 * return 1 -- yes, 0 -- no, negative -- failure
 */
int pxa95x_query_gwsr(int pin)
{
	int off, ret;

	if (pin < 0)
		return -EINVAL;
	off = pin / 32 + 1;
	/* read data from GWSR(x), x is [1..4] */
	ret = __raw_readl(bpb_membase + GWSR(off));
	ret = (ret >> (pin - (off - 1) * 32)) & 1;
	return ret;
}

/* reg_num is in 1..6 */
static unsigned int pxa95x_get_gwsr(int reg_num)
{
	return __raw_readl(bpb_membase + GWSR(reg_num));
}

static int __init pxa95x_pm_init(void)
{
	int ret = 0;
#ifdef CONFIG_IPM
	unsigned int oscc, dmemvlr;

	wake_lock_init(&system_wakeup, WAKE_LOCK_SUSPEND, "system_wakeup_detect");

#ifdef CONFIG_MMC_BLOCK_CMD13_AFTER_CMD18
	mmc_base[0] = (u32) ioremap(0x55000000, 4096);
	mmc_base[1] = (u32) ioremap(0x55100000, 4096);
	mmc_base[2] = (u32) ioremap(0x55200000, 4096);
#endif

	suspend_set_ops(&pxa95x_pm_ops);

#ifdef CONFIG_IPM_D2IDLE
	enable_deepidle |= IDLE_D2;
#endif
#ifdef CONFIG_IPM_D1IDLE
	enable_deepidle |= IDLE_D1;
#endif
#ifdef CONFIG_IPM_CGIDLE
	enable_deepidle |= IDLE_CG;
#endif
	orig_poweroff = pm_power_off;

#ifdef CONFIG_INPUT_88PM8XXX_ONKEY
	if (cpu_is_pxa978())
		pm_power_off = pm8xxx_system_poweroff;
	else
#endif
		pm_power_off = pxa95x_pm_poweroff;

	clk_tout_s0 = clk_get(NULL, "CLK_TOUT_S0");
	if (IS_ERR(clk_tout_s0)) {
		pr_err("unable to get tout_s0 clock");
		return PTR_ERR(clk_tout_s0);
	}
	clk_pout = clk_get(NULL, "CLK_POUT");
	if (IS_ERR(clk_pout)) {
		pr_err("unable to get pout clock");
		return PTR_ERR(clk_pout);
	}
	clk_gc = clk_get(NULL, "GCCLK");
	if (IS_ERR(clk_gc)) {
		pr_err("unable to get gc clock");
		return PTR_ERR(clk_gc);
	}
	ret = dvfm_register("Galcore", &dvfm_lock.dev_idx);
	if (ret)
		printk(KERN_ERR "GC dvfm register fail(%d)\n", ret);

	/* if nowhere use tout_s0, it would be disable */
	clk_enable(clk_tout_s0);
	clk_disable(clk_tout_s0);
	oscc = OSCC;
	if (!cpu_is_pxa978())
		/* Disable CLK_TOUT in S0/S2/S3 state */
		oscc &= ~0x600;
	else
		/* Clear all clearalways bits */
		oscc &= ~0x6600;
	/* Disable TD bit */
	oscc &= ~0x10000;
	/* configuring VCTCXO_EN to be used by PMIC
	 * as a trigger for volateg change */
	oscc |= OSCC_VCTVCEN;
	/* wait time for voltage change completion
	 * is set to 1.5 32Khz cycles */
	oscc &= ~(0x7 << OSCC_VCTVSTB_OFFSET);
	oscc |= (0x1 << OSCC_VCTVSTB_OFFSET);

	if (!cpu_is_pxa978()) {
		dmemvlr = DMEMVLR;
		dmemvlr &= ~(0x3 << DMEMVLR_DMCHV_OFFSET);
		dmemvlr |= (0x1 << DMEMVLR_DMCHV_OFFSET);
		DMEMVLR = dmemvlr;
	}

	{
		static u32 *base;
#define GEN_REG3	0x42404008	/* general Register 3 */
#define GEN_REG3_SPLGEN	   (1 << 19)
		base = ioremap_nocache(GEN_REG3, 4);
		/* system pll auto gating set */
		*base |= GEN_REG3_SPLGEN;
		iounmap(base);
	}
	/* set VCTSTB as 0x11 ~ 0.5ms */
	OSCC = (oscc & ~(0xFF << 24)) | (0x11 << 24);
	/* Enable CLK_POUT */
	clk_enable(clk_pout);
#endif
	overheating_init();

	pxa95x_pm_set_clk("GPIOCLK", 1);

	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;

	bpb_membase = ioremap(BPB_START, BPB_END - BPB_START + 1);

#ifdef CONFIG_PXA95x_DVFM
	/* Init debugfs folder and files. */
	pxa_9xx_power_init_debugfs();
#endif

	pxa95x_sysbus_init(&pxa95x_pm_regs);

	/* make sure that sram bank 0 is not off in D1
	 * base on JIRA MG1-1021, it should be set to 0x01
	 * For NEVO: set this bit to 0 to retain the state such
	 * as L2$, or L2$ corruption when exit from D1
	 */
	 if (cpu_is_pxa978())
		AD1R = 0;
	else
		AD1R = 1;
	/* setting bit 2 (SV_WK) to enable wakeup event detection in clock
	 * gated mode entery sequecne */
	AGENP |= 0x4;

	/* setting bit 4 to avoid hangs in C2 entry (MG2-388) */
	if (is_wkr_mg2_388())
		AGENP |= 0x10;

	/* Setting C2 as default */
#ifdef CONFIG_PXA95x_DVFM
	pm_core_pwdn(CPU_PDWN_LPM_EXIT);
	if (cpu_is_pxa978()) {
		remap_c2_reg = ioremap(REMAP_C2_REG , 0x4);
		pxa978_save_reset_handler(get_c2_sram_base());
		c2_address_remap();
	}
#endif

	/* Enabling ACCU and BPMU interrupts */
	ICMR2 |= 0x100000;
	ICMR3 |= 0x100000;

	/* Clearing RDH bit in Startup */
	ASCR &= ~ASCR_RDH;

	/* if SRAM is allocate to GB by uboot param, the lpm code will not be
	 * copied to SRAM. note, power cannot be enabled if sram allocate to
	 * GB */
	if (!disable_sram_use && !(cpu_is_pxa978()))
		pxa95x_init_standby((unsigned int)pxa95x_pm_regs.sram_map +
				    0x8000);

#ifdef CONFIG_PXA95x_DVFM
	cken_rsvd_bit_mask_setup();
#endif

	/* Reading ProductID Full Layer info from FuseReg */
	is_wkr_mg1_1274_value = is_wkr_mg1_1274();

#ifdef CONFIG_PXA95x_DVFM
	if (dvfm_find_index("User", &user_index))
		pr_err("Can't get \"User\" index for DVFM in %s.\n", __func__);
#endif

	return 0;
}

module_init(pxa95x_pm_init);

/* uboot parameters handling */

static int uboot_sram_allocation(char *s)
{
	disable_sram_use = 1;
	/* no SRAM so pm must be disabled */
	PowerDisabled = 1;
	return 1;
}

__setup("tavorcfg_bsram2gb=", uboot_sram_allocation);

/* uboot parameters handling code end */
