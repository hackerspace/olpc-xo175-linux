/*
 * linux/arch/arm/mach-mmp/include/mach/regs-zsp.h
 *
 *  Audio Accelerator Processor Registers
 *  jgjing@marvell.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_ZSP_H
#define __ASM_MACH_REGS_ZSP_H

#include <mach/addr-map.h>

#define ZSP_DTCM_START		0xc0000000
#define ZSP_ITCM_START		0xc0040000
#define ZSP_DATA_OFFSET		0x00020000
#define ZSP_BADDR_TO_PA(x)	((x)+ZSP_DTCM_START-ZSP_DATA_OFFSET)

#define ZSP_VIRT_BASE		(AXI_VIRT_BASE + 0xA1000)
#define ZSP_REG(x)		(ZSP_VIRT_BASE + (x))

#define	ZSP_CONFIG_SVTADDR	ZSP_REG(0x0000)
#define	ZSP_CONFIG_DBG		ZSP_REG(0x0004)
#define	ZSP_CONFIG_MEMPARAM	ZSP_REG(0x0008)
#define	ZSP_CONFIG_REG0		ZSP_REG(0x000c)
#define	ZSP_CONFIG_REG1		ZSP_REG(0x0010)
#define	ZSP_CONFIG_REG2		ZSP_REG(0x0014)

#define	ZSP_CONFIG_REG2_REG6	ZSP_REG(0x0018)

#define	ZSP_INT_MASK0		ZSP_REG(0x0028)
#define	ZSP_INT_MASK1		ZSP_REG(0x002c)
#define	ZSP_STATUS_REG0		ZSP_REG(0x0200)
#define	ZSP_STATUS_REG1		ZSP_REG(0x0204)
#define	ZSP_STATUS_REG2		ZSP_REG(0x0208)
#define	ZSP_INT_STATUS0		ZSP_REG(0x020c)
#define	ZSP_INT_STATUS1		ZSP_REG(0x0210)

/*
 *	THE BIT DEFINES
 */
/* zsp boot vector */
#define	ZSP_CONFIG_SVTADDR_ZSP_SVTADDR_MSK		(0xffffff << 8)
#define	ZSP_CONFIG_SVTADDR_ZSP_SVTADDR_BASE		8
/* ZSP halt */
#define	ZSP_CONFIG_DBG_Z_HALT				(1 << 4)
/* ZSP external breakpoint */
#define	ZSP_CONFIG_DBG_ZEXT_BP_MSK			0xf
#define	ZSP_CONFIG_DBG_ZEXT_BP_BASE			0

/* RTC for imem/dmem */
#define	ZSP_CONFIG_MEMPARAM_RTC_MSK		(0x3 << 2)
#define	ZSP_CONFIG_MEMPARAM_RTC_BASE		2
/* WTC for imem/dmem */
#define	ZSP_CONFIG_MEMPARAM_WTC_MSK		0x3
#define	ZSP_CONFIG_MEMPARAM_WTC_BASE		0

/* ZSP DDR OK to SLEEP */
#define	ZSP_CONFIG_REG0_ZSP_DDR_OK_TO_SLEEP 	(1 << 12)
/* ZSP SRAM powerdown enable */
#define	ZSP_CONFIG_REG0_ZSP_SRAM_PWRDWN_EN_I 	(1 << 11)
/* ZSP powerdown enable */
#define	ZSP_CONFIG_REG0_ZSP_PWRDWN_EN_I 	(1 << 10)
/* divider reset enable */
#define	ZSP_CONFIG_REG0_DIVIDER_RESET_EN	(1 << 9)
/* Clock gate control */
#define	ZSP_CONFIG_REG0_CLK_GATE_CTRL		(0x3 << 7)
#define	ZSP_CONFIG_REG0_CLK_GATE_CTRL_BASE		7
/* Dynamic frequency change request */
#define	ZSP_CONFIG_REG0_DYN_FC_REQ		(1 << 6)
/* freqeuency change request */
#define	ZSP_CONFIG_REG0_FC_REQ			(1 << 5)
/* Clock devider select for core clk */
#define	ZSP_CONFIG_REG0_CORE_PCLK_DIV_N		(0x7 << 2)
#define	ZSP_CONFIG_REG0_CORE_PCLK_DIV_N_BASE	2
/*Bit(s) ZSPCONFIG_REG0_RSRV_1 reserved */
/* core pll clock select 0 */
#define	ZSP_CONFIG_REG0_CORE_PLL_SEL0         	1

/* Cam enable for Audio RAM */
#define	ZSP_CONFIG_REG1_CAMENABLE_ARAM		(1 << 16)
/* RTC for Audio RAM */
#define	ZSP_CONFIG_REG1_RTC_ARAM_MSK		(0x3 << 10)
#define	ZSP_CONFIG_REG1_RTC_ARAM_BASE		10
/* WTC for Audio RAM */
#define	ZSP_CONFIG_REG1_WTC_ARAM_MSK		(0x3 << 8)
#define	ZSP_CONFIG_REG1_WTC_ARAM_BASE		8
/* powerdown for Audio RAM */
#define	ZSP_CONFIG_REG1_PDWN1_ARAM		(1 << 7)
/* ROM RTC REF */
#define	ZSP_CONFIG_REG1_ROM_RTC_REF_MSK		(0x3 << 5)
#define	ZSP_CONFIG_REG1_ROM_RTC_REF_BASE	5
/* ROM RTC */
#define	ZSP_CONFIG_REG1_ROM_RTC_MSK		(0x3 << 3)
#define	ZSP_CONFIG_REG1_ROM_RTC_BASE		3
/* Power Down BROM */
#define	ZSP_CONFIG_REG1_PDWN_BROM		(1 << 2)
/* ROM Wait Cycle */
#define	ZSP_CONFIG_REG1_ROM_WAIT_CYCLE_MSK	0x3
#define	ZSP_CONFIG_REG1_ROM_WAIT_CYCLE_BASE	0
/*ZSP_STATUS_REG0	0x0200	ZSP STATUS REG0 */
/* PLL_SEL */
#define	ZSP_STATUS_REG0_PLL_SEL			(1 << 3)
/* PCLK divider */
#define	ZSP_STATUS_REG0_PCLK_DIV_MSK		0x7
#define	ZSP_STATUS_REG0_PCLK_DIV_BASE		0
/*ZSP_STATUS_REG1	0x0204	ZSP STATUS REG1 */
/*Bit(s) ZSP_STATUS_REG1_RSRV_31_0 reserved */

/*ZSP_STATUS_REG2 0x0208 ZSP STATUS REG2 */
/*Bit(s) ZSP_STATUS_REG2_RSRV_31_1 reserved */
/* freq change done */
#define	ZSP_STATUS_REG2_FC_DONE			1
/*ZSP INT_STATUS0	0x020c	ZSP INT_STATUS0 */
/*Bit(s) ZSPINT_STATUS0_RSRV_31_13 reserved */

/*ZSP INT_STATUS1 0x0210 ZSP INT_STATUS1 */
/*Bit(s) ZSPINT_STATUS1_RSRV_31_13 reserved */

#endif /* __ASM_MACH_REGS_ZSP_H */
