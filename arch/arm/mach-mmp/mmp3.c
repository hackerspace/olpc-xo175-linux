/*
 *  Marvell MMP3 aka PXA2128 aka 88AP2128 support
 *
 *  Copyright (C) 2019 Lubomir Rintel <lkundrak@v3.sk>
 *
 *  Based on linux/arch/arm/mach-mmp/mmp2-dt.c
 *
 *  Copyright (C) 2012 Marvell Technology Group Ltd.
 *  Author: Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/clk-provider.h>
#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>

#include <linux/delay.h>
#include "addr-map.h"

#include "common.h"

static const char *const mmp3_dt_board_compat[] __initconst = {
	"marvell,mmp3",
	NULL,
};

//static inline void set_reg(u32 val, volatile void __iomem *addr, const char *name)
noinline void set_reg(u32 val, volatile void __iomem *addr, const char *name)
{
	printk ("XXX WRITE 0x%08x: 0x%08x -> 0x%08x  %s\n", addr, __raw_readl(addr), val, name);
//	mdelay(400);
	__raw_writel(val, addr);
//	mdelay(400);
}


static void __init meh(void)
{
#if 0
MPMU_SPSR       0xd4050004 0xA5000000    | MPMU_SPSR       0xd4050004 0x85000000
MPMU_FCCR       0xd4050008 0x61800000    | MPMU_FCCR       0xd4050008 0x41800000
MPMU_POSR       0xd4050010 0x11C50000    | MPMU_POSR       0xd4050010 0x31C50000
MPMU_PLL2CR     0xd4050034 0x001C5200    | MPMU_PLL2CR     0xd4050034 0x001C5300
MPMU_ISCCR1     0xd4050040 0xD3EE2276    | MPMU_ISCCR1     0xd4050040 0x5820130B
MPMU_ISCCR2     0xd4050044 0xD0040040    | MPMU_ISCCR2     0xd4050044 0x5820130B
MPMU_PLL1_REG1  0xd4050050 0x001B0F00    | MPMU_PLL1_REG1  0xd4050050 0x001A5B00
MPMU_PLL1_REG2  0xd4050054 0x000046C3    | MPMU_PLL1_REG2  0xd4050054 0x00004696
MPMU_PLL1_SSC   0xd4050058 0x25210399    | MPMU_PLL1_SSC   0xd4050058 0x25290499
MPMU_PLL2_CTRL1 0xd4050414 0xDFFFFFFF    | MPMU_PLL2_CTRL1 0xd4050414 0x25390699
MPMU_APCR       0xd4051000 0x31006000    | MPMU_APCR       0xd4051000 0x21006000
MPMU_APSR       0xd4051004 0xA5000000    | MPMU_APSR       0xd4051004 0x85000000

APMU_CC_PJ                          0xd4282804 0x00BC8410     | APMU_CC_PJ                          0xd4282804 0x00BC0510
APMU_DM_CC_SP                       0xd4282808 0x02FC80D9     | APMU_DM_CC_SP                       0xd4282808 0x00FC00D9
APMU_DM_CC_PJ                       0xd428280c 0x02FC8010     | APMU_DM_CC_PJ                       0xd428280c 0x00FC0110
APMU_PJ_IDLE_CFG                    0xd4282818 0x00000400     | APMU_PJ_IDLE_CFG                    0xd4282818 0xE0000700
APMU_DISPLAY1_CLK_RES_CTRL          0xd428284c 0x0000211B     | APMU_DISPLAY1_CLK_RES_CTRL          0xd428284c 0x0000011B
APMU_CCIC_CLK_RES_CTRL              0xd4282850 0x00020000     | APMU_CCIC_CLK_RES_CTRL              0xd4282850 0x00026800
APMU_SDH1_CLK_RES_CTRL              0xd4282854 0x0000081B     | APMU_SDH1_CLK_RES_CTRL              0xd4282854 0x0000041B
APMU_BUS_CLK_RES_CTRL               0xd428286c 0x00000643     | APMU_BUS_CLK_RES_CTRL               0xd428286c 0x00000603
APMU_CORE_STATUS                    0xd4282890 0x000001F3     | APMU_CORE_STATUS                    0xd4282890 0x000003C1
APMU_AP_ISR                         0xd42828a0 0x00000020     | APMU_AP_ISR                         0xd42828a0 0x00000000
APMU_PLL_SEL_STATUS                 0xd42828c4 0x000032D8     | APMU_PLL_SEL_STATUS                 0xd42828c4 0x000030D0
APMU_SDH3_CLK_RES_CTRL              0xd42828e8 0x0000001B     | APMU_SDH3_CLK_RES_CTRL              0xd42828e8 0x00000000
APMU_CCIC2_CLK_RES_CTRL             0xd42828f4 0x00020000     | APMU_CCIC2_CLK_RES_CTRL             0xd42828f4 0x00016800
APMU_AUDIO_CLK_RES_CTRL             0xd428290c 0x00000712     | APMU_AUDIO_CLK_RES_CTRL             0xd428290c 0x00000700
APMU_DISPLAY2_CLK_RES_CTRL          0xd4282910 0x0000035B     | APMU_DISPLAY2_CLK_RES_CTRL          0xd4282910 0x00000240
APMU_CC2_PJ                         0xd4282950 0x8E220003     | APMU_CC2_PJ                         0xd4282950 0x8E220001
APMU_DM2_CC_SP                      0xd4282954 0x000000DB     | APMU_DM2_CC_SP                      0xd4282954 0x000000D9
APMU_DM2_CC_PJ                      0xd4282958 0x042200DB     | APMU_DM2_CC_PJ                      0xd4282958 0x042200D9
APMU_PJ_IDLE_CFG2                   0xd4282a00 0x00000062     | APMU_PJ_IDLE_CFG2                   0xd4282a00 0xE0000300
APMU_AUDIO_ISLAND_SRAM_PWR_DWN_CTRL 0xd4282a40 0x000000CF     | APMU_AUDIO_ISLAND_SRAM_PWR_DWN_CTRL 0xd4282a40 0x0000000F
APMU_PJ_C1_CC4                      0xd4282a4c 0x00000C13     | APMU_PJ_C1_CC4                      0xd4282a4c 0x00000C12
APMU_SDHM_DLL_STAT0                 0xd4282a74 0x19E0F078     | APMU_SDHM_DLL_STAT0                 0xd4282a74 0x18F0783C
APMU_SDHM_DLL_STAT1                 0xd4282a78 0x000000F1     | APMU_SDHM_DLL_STAT1                 0xd4282a78 0x00000078

APBC_MMP2_RTC           0xd4015000 0x00000083                 | APBC_MMP2_RTC           0xd4015000 0x00000004
APBC_MMP2_SSP2          0xd4015054 0x00000003                 | APBC_MMP2_SSP2          0xd4015054 0x00000004
APBC_MMP2_SSP3          0xd4015058 0x00000004                 | APBC_MMP2_SSP3          0xd4015058 0x00000003
APBC_MMP2_SSP4          0xd401505c 0x00000004                 | APBC_MMP2_SSP4          0xd401505c 0x00000003
APBC_MMP2_TWSI2         0xd4015008 0x00000003                 | APBC_MMP2_TWSI2         0xd4015008 0x00000004
APBC_MMP2_TIMERS        0xd4015024 0x00000013                 | APBC_MMP2_TIMERS        0xd4015024 0x00000033
APBC_MMP2_THSENS1       0xd4015090 0x00000003                 | APBC_MMP2_THSENS1       0xd4015090 0x00000004


MMP2_ICU_PJ4_IRQ_GLOBAL_MASK  0xd4282110 0x00000001           | MMP2_ICU_PJ4_IRQ_GLOBAL_MASK  0xd4282110 0x00000000
MMP3_ICU_GBL_IRQ1_MSK         0xd4282110 0x00000001           | MMP3_ICU_GBL_IRQ1_MSK         0xd4282110 0x00000000
MMP3_ICU_GBL_IRQ3_MSK         0xd428410C 0x00000001           | MMP3_ICU_GBL_IRQ3_MSK         0xd428410C 0x00000000
MMP3_ICU_GBL_IRQ4_MSK         0xd4284110 0x00000001           | MMP3_ICU_GBL_IRQ4_MSK         0xd4284110 0x00000000
MMP3_ICU_GBL_IRQ5_MSK         0xd4284114 0x00000001           | MMP3_ICU_GBL_IRQ5_MSK         0xd4284114 0x00000000
MMP3_ICU_GBL_IRQ6_MSK         0xd4284190 0x00000001           | MMP3_ICU_GBL_IRQ6_MSK         0xd4284190 0x00000000
MMP2_ICU_INT5_MASK            0xd428216c 0x000000FE           | MMP2_ICU_INT5_MASK            0xd428216c 0x000000FF
MMP2_ICU_INT35_MASK           0xd4282174 0x7E3FFFFF           | MMP2_ICU_INT35_MASK           0xd4282174 0x7FFFFFFF
#endif



#if 0
set_reg(0xA5000000, APB_VIRT_BASE + 0x50004, "MPMU_SPSR");
set_reg(0x61800000, APB_VIRT_BASE + 0x50008, "MPMU_FCCR");
set_reg(0x11C50000, APB_VIRT_BASE + 0x50010, "MPMU_POSR");
//set_reg(0x001C5200, APB_VIRT_BASE + 0x50034, "MPMU_PLL2CR");
// crash
set_reg(0xD3EE2276, APB_VIRT_BASE + 0x50040, "MPMU_ISCCR1");
set_reg(0xD0040040, APB_VIRT_BASE + 0x50044, "MPMU_ISCCR2");
set_reg(0x001B0F00, APB_VIRT_BASE + 0x50050, "MPMU_PLL1_REG1");
set_reg(0x000046C3, APB_VIRT_BASE + 0x50054, "MPMU_PLL1_REG2");
set_reg(0x25210399, APB_VIRT_BASE + 0x50058, "MPMU_PLL1_SSC");
set_reg(0xDFFFFFFF, APB_VIRT_BASE + 0x50414, "MPMU_PLL2_CTRL1");
set_reg(0x31006000, APB_VIRT_BASE + 0x51000, "MPMU_APCR");
set_reg(0xA5000000, APB_VIRT_BASE + 0x51004, "MPMU_APSR");
#endif

#if 0
set_reg(0x00BC8410, AXI_VIRT_BASE + 0x82804, "APMU_CC_PJ");
set_reg(0x02FC80D9, AXI_VIRT_BASE + 0x82808, "APMU_DM_CC_SP");
set_reg(0x02FC8010, AXI_VIRT_BASE + 0x8280c, "APMU_DM_CC_PJ");
set_reg(0x00000400, AXI_VIRT_BASE + 0x82818, "APMU_PJ_IDLE_CFG");
set_reg(0x0000211B, AXI_VIRT_BASE + 0x8284c, "APMU_DISPLAY1_CLK_RES_CTRL");
set_reg(0x00020000, AXI_VIRT_BASE + 0x82850, "APMU_CCIC_CLK_RES_CTRL");
set_reg(0x0000081B, AXI_VIRT_BASE + 0x82854, "APMU_SDH1_CLK_RES_CTRL");
set_reg(0x00000643, AXI_VIRT_BASE + 0x8286c, "APMU_BUS_CLK_RES_CTRL");
set_reg(0x000001F3, AXI_VIRT_BASE + 0x82890, "APMU_CORE_STATUS");
set_reg(0x00000020, AXI_VIRT_BASE + 0x828a0, "APMU_AP_ISR");
set_reg(0x000032D8, AXI_VIRT_BASE + 0x828c4, "APMU_PLL_SEL_STATUS");
set_reg(0x0000001B, AXI_VIRT_BASE + 0x828e8, "APMU_SDH3_CLK_RES_CTRL");
set_reg(0x00020000, AXI_VIRT_BASE + 0x828f4, "APMU_CCIC2_CLK_RES_CTRL");
set_reg(0x00000712, AXI_VIRT_BASE + 0x8290c, "APMU_AUDIO_CLK_RES_CTRL");
set_reg(0x0000035B, AXI_VIRT_BASE + 0x82910, "APMU_DISPLAY2_CLK_RES_CTRL");
set_reg(0x8E220003, AXI_VIRT_BASE + 0x82950, "APMU_CC2_PJ");
set_reg(0x000000DB, AXI_VIRT_BASE + 0x82954, "APMU_DM2_CC_SP");
set_reg(0x042200DB, AXI_VIRT_BASE + 0x82958, "APMU_DM2_CC_PJ");
set_reg(0x00000062, AXI_VIRT_BASE + 0x82a00, "APMU_PJ_IDLE_CFG2");
set_reg(0x000000CF, AXI_VIRT_BASE + 0x82a40, "APMU_AUDIO_ISLAND_SRAM_PWR_DWN_CTRL");
set_reg(0x00000C13, AXI_VIRT_BASE + 0x82a4c, "APMU_PJ_C1_CC4");
set_reg(0x19E0F078, AXI_VIRT_BASE + 0x82a74, "APMU_SDHM_DLL_STAT0");
set_reg(0x000000F1, AXI_VIRT_BASE + 0x82a78, "APMU_SDHM_DLL_STAT1");
#endif

// OK

set_reg(0x00000083, APB_VIRT_BASE + 0x15000, "APBC_MMP2_RTC");
set_reg(0x00000003, APB_VIRT_BASE + 0x15054, "APBC_MMP2_SSP2");
set_reg(0x00000004, APB_VIRT_BASE + 0x15058, "APBC_MMP2_SSP3");
set_reg(0x00000004, APB_VIRT_BASE + 0x1505c, "APBC_MMP2_SSP4");
set_reg(0x00000003, APB_VIRT_BASE + 0x15008, "APBC_MMP2_TWSI2");
set_reg(0x00000013, APB_VIRT_BASE + 0x15024, "APBC_MMP2_TIMERS");
set_reg(0x00000003, APB_VIRT_BASE + 0x15090, "APBC_MMP2_THSENS1");


//set_reg(0x00000001, AXI_VIRT_BASE + 0x82110, "MMP2_ICU_PJ4_IRQ_GLOBAL_MASK");
set_reg(0x00000001, AXI_VIRT_BASE + 0x82110, "MMP3_ICU_GBL_IRQ1_MSK");
set_reg(0x00000001, AXI_VIRT_BASE + 0x82114, "MMP3_ICU_GBL_IRQ2_MSK"); /// not in diff
set_reg(0x00000001, AXI_VIRT_BASE + 0x8410C, "MMP3_ICU_GBL_IRQ3_MSK");
set_reg(0x00000001, AXI_VIRT_BASE + 0x84110, "MMP3_ICU_GBL_IRQ4_MSK");
set_reg(0x00000001, AXI_VIRT_BASE + 0x84114, "MMP3_ICU_GBL_IRQ5_MSK");
set_reg(0x00000001, AXI_VIRT_BASE + 0x84190, "MMP3_ICU_GBL_IRQ6_MSK");
//set_reg(0x000000FE, (volatile void __iomem *addr)AXI_VIRT_BASE + 0x8216c, "MMP2_ICU_INT5_MASK");
//set_reg(0x7E3FFFFF, (volatile void __iomem *addr)AXI_VIRT_BASE + 0x82174, "MMP2_ICU_INT35_MASK");

printk ("=======================================================================\n");

}

DT_MACHINE_START(MMP2_DT, "Marvell MMP3")
	.map_io		= mmp2_map_io,
	.init_machine	= meh,
	.dt_compat	= mmp3_dt_board_compat,

	/* Auxiliary Control:
	   TODO: According to the manual, this register should be
		written in secure access, we may need to move the
		configuration in early stage of boot if TZ enabled

	   [ 0.. 2]     cycles of latency of data RAM read
	   [ 3.. 5]     cycles of latency of data RAM write
	   [ 6.. 8]     cycles of latency of tag RAM
	   [ 9..11]     cycles of latency of dirty RAM
	   [12]         exclusive cache op, 0:disable,1:enable
	   [13..16]     associativity
	   [17..19]     way-size
	   [20]         event monitor bus enable
	   [21]         parity enable
	   [22]         shared attribute override enable
	   [23..24]     force write allocate
			0: use AWCACHE
			1: force no WA
			2: force WA on
			3: internal mapped
	   [25]         reserved, SBO/RAO
	   [26]         Non-secure lockdown enable
	   [27]         Non-secure interrupt access enable
	   [28..31]     reserved, SBZ
	*/
	/*
	  force NO WA, for A0 memory performance, bug in WA
	  64KB way-size
	  clear bit[16] to make sure l2x0_init call take it as 8-way
	*/
	.l2c_aux_val	= 1 << L220_AUX_CTRL_FWA_SHIFT |
			  L310_AUX_CTRL_DATA_PREFETCH |
			  L310_AUX_CTRL_INSTR_PREFETCH |
			  L2C_AUX_CTRL_WAY_SIZE(3),
	.l2c_aux_mask	= 0xc200ffff,
MACHINE_END
