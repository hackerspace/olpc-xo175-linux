/*
 * linux/arch/arm/mach-mmp/include/mach/regs-ciu.h
 *
 *  CPU Interface Unit Registers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_CIU_H
#define __ASM_MACH_REGS_CIU_H

#include <mach/addr-map.h>

#define CIU_VIRT_BASE		(AXI_VIRT_BASE + 0x82c00)
#define CIU_REG(x)		(CIU_VIRT_BASE + (x))

#define CIU_CHIP_ID		CIU_REG(0x0000)
#define CIU_CPU_CONF		CIU_REG(0x0008)
#define CIU_CPU_SRAM_SPD	CIU_REG(0x0010)
#define CIU_CPU_L2C_SRAM_SPD	CIU_REG(0x0018)
#define CIU_MCB_CONF		CIU_REG(0x001c)
#define CIU_SYS_BOOT_CNTRL	CIU_REG(0x0020)
#define CIU_SW_BRANCH_ADDR	CIU_REG(0x0024)
#define CIU_PERF_COUNT0_CNTRL	CIU_REG(0x0028)
#define CIU_PERF_COUNT1_CNTRL	CIU_REG(0x002c)
#define CIU_PERF_COUNT2_CNTRL	CIU_REG(0x0030)
#define CIU_PERF_COUNT0		CIU_REG(0x0034)
#define CIU_PERF_COUNT1		CIU_REG(0x0038)
#define CIU_PERF_COUNT2		CIU_REG(0x003c)
#define CIU_MC_CONF		CIU_REG(0x0040)
#define CIU_MCB_SRAM_SPD	CIU_REG(0x0044)
#define CIU_AXI_SRAM_SPD	CIU_REG(0x0048)
#define CIU_DDR_ILV_CTRL	CIU_REG(0x00a0)
#define CIU_FABRIC_CKGT_CTRL0	CIU_REG(0x0064)
#define CIU_FABRIC_CKGT_CTRL1	CIU_REG(0x0068)
#define CIU_FABRIC_CKGT_CTRL2	CIU_REG(0x00dc)

/* This is used by pxa988 for PMU interrupt */
#define CIU_CA9_CPU_CORE0_CONF CIU_REG(0x00d0)
#define CIU_CA9_CPU_CORE1_CONF CIU_REG(0x00e0)

/* This is used by pxa988 for warm reset */
#define CIU_WARM_RESET_VECTOR   CIU_REG(0x00d8)

/* used for set xtc */
#define CIU_GPU_XTC_REG		CIU_REG(0x00a4)
#define CIU_VPU_XTC_REG		CIU_REG(0x00a8)
#define CIU_CA9_CPU_CONF_SRAM0	CIU_REG(0Xc8)
#define CIU_CA9_CPU_CONF_SRAM1	CIU_REG(0Xcc)

static __maybe_unused int ciu_ddr_ilv_on(void)
{
	return ((__raw_readl(CIU_DDR_ILV_CTRL) & 0x7f) != 0);
}

static __maybe_unused u32 ciu_ddr_ilv_size(void)
{
	u32 regval;
	regval = __raw_readl(CIU_DDR_ILV_CTRL) & 0x7f;
	/* there should be only 1 bit set */
	BUG_ON(((regval - 1) & regval) != 0);
	if ((regval & (~0x1f)) != 0)
		return regval << 24;
	else
		return (regval * regval) << 12;
}


#endif /* __ASM_MACH_REGS_CIU_H */
