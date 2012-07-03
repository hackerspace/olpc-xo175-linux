/*
 * arch/arm/mach-pxa/include/mach/pxa95x_dvfm.h
 *
 * PXA95x DVFM Driver Head File
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef PXA95X_DVFM_H
#define PXA95X_DVFM_H

#include <mach/dvfm.h>
#include <mach/pxa95x_pm.h>

#define DMEMC_FREQ_HIGH		0
#define DMEMC_FREQ_LOW		1

#define GBPLL_FREQ_HIGH		0
#define GBPLL_FREQ_LOW		1
#define OP_NAME_LEN		16

/* the following defines are for debug */
#define DVFM_FORCE_PP 1

#define DVFM_FORCE_D2_WAKEUP_SELECT 4
#define DVFM_FORCE_D2 5
#define UNDO_FORCE_PP  6
#define FORCE_C0  7
#define VCTCXO_FORCE_ON 8

#define GET_CURRENT_PP 11

#define POWER_DISABLE 13
#define GET_SUSPEND_STATE 14
#define FORCE_SUSPEND_STATE 15

#define RESET_COMM 20

#define DEVICE_DRIVER_DISABLE_PP 80
#define DEVICE_DRIVER_ENABLE_PP 81

#define RELEASE_CG_CONSTRAINT false
#define GET_CG_CONSTRAINT true
extern void GC_switch_cg_constraint(int enable);

/* Used for testing LPM - "Start" will zero the counters. */
#define DEBUG_MSPM_START_LPM_DEBUG_COUNT 100
#define DEBUG_MSPM_GET_LPM_DEBUG_COUNT 101
/* the following are not commands!!!! */
#define DEBUG_DRIVER_REMOVE_REQ_OFFSET 20

#define FORCE_LPM 110
/* This structure is used to count the number of times we enter/exit
 * D2/CGM/DOCS
 */
typedef struct {
	unsigned long D2_Enter_Exit_count;
	unsigned long CGM_Enter_Exit_count;
	unsigned long D0C1_Enter_count;
} pxa95x_DVFM_LPM_Global_Count;
/* end of defines for debug */

struct proc_op_array {
	unsigned int chip_id;
	char *cpu_name;
	struct dvfm_md_opt *op_array;
	unsigned int nr_op;
};

enum {
	POWER_MODE_D0 = 0,
	POWER_MODE_D1,
	POWER_MODE_D2,
	POWER_MODE_CG,
};

enum {
	OP_FLAG_FACTORY = 0,
	OP_FLAG_USER_DEFINED,
	OP_FLAG_BOOT,
	OP_FLAG_ALL,
};

enum {
	IDLE_D0 = 0,
	IDLE_D1 = 1,
	IDLE_D2 = 2,
	IDLE_CG = 4,
};

struct dvfm_md_opt {
	unsigned int vcc_core;
	unsigned int vcc_sram;
	unsigned int xl;
	unsigned int xn;
	unsigned int core;
	unsigned int aclk;
	unsigned int smcfs;
	unsigned int sflfs;
	unsigned int hss;
	unsigned int axifs;	/* AXI Bus Frequency */
	unsigned int display;
	unsigned int dsi;
	unsigned int dmcfs;
	unsigned int df_clk;
	unsigned int empi_clk;
	unsigned int gcfs;	/*GC frequency */
	unsigned int gcaxifs;
	unsigned int vmfc;	/*Vmeta frequency */
	unsigned int power_mode;
	unsigned int flag;
	unsigned int lpj;
	char name[OP_NAME_LEN];
};

/* This structure is similar to dvfm_md_opt.
 * Reserve this structure in order to keep compatible
 */
struct pxa95x_fv_info {
	unsigned long xl;
	unsigned long xn;
	unsigned int vcc_core;
	unsigned int vcc_sram;
	unsigned long smcfs;
	unsigned long sflfs;
	unsigned long hss;
	unsigned long dmcfs;
	unsigned long df_clk;
	unsigned long empi_clk;
	/* WARNING: above fields must be consistent with PM_FV_INFO!!! */
	int axifs;		/* AXI Bus Frequency */
	int dsi;
	unsigned int gcfs;
	unsigned int gcaxifs;
	unsigned int vmfc;
	unsigned int display;
	unsigned int lpj;	/* New value for loops_per_jiffy */
};

struct pxa95x_freq_mach_info {
	int flags;
};

#define PXA95x_USE_POWER_I2C  (1UL << 0)

#ifdef __KERNEL__
extern void set_pxa95x_freq_info(struct pxa95x_freq_mach_info *info);
/* extern void set_pxa95x_freq_parent(struct device *parent_dev); */

extern int md2fvinfo(struct pxa95x_fv_info *fv_info, struct dvfm_md_opt *orig);
extern int ForceC0;
extern pxa95x_DVFM_LPM_Global_Count DVFMLPMGlobalCount;
extern void write_accr_in_sram(u32, u32, u32, u32, u32, u32, u32);
extern void write_accr0(unsigned int value, unsigned int mask);
extern unsigned long max_pp, max_gc, min_gc, max_vmeta, max_core;
#endif /*__KERNEL__*/

#endif
