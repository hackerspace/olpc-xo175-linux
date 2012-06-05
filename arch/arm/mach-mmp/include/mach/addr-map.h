/*
 * linux/arch/arm/mach-mmp/include/mach/addr-map.h
 *
 *   Common address map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_ADDR_MAP_H
#define __ASM_MACH_ADDR_MAP_H

/* APB - Application Subsystem Peripheral Bus
 *
 * NOTE: the DMA controller registers are actually on the AXI fabric #1
 * slave port to AHB/APB bridge, due to its close relationship to those
 * peripherals on APB, let's count it into the ABP mapping area.
 */
#define APB_PHYS_BASE		0xd4000000
#define APB_VIRT_BASE		0xfe000000
#define APB_PHYS_SIZE		0x00200000

#define AXI_PHYS_BASE		0xd4200000
#define AXI_VIRT_BASE		0xfe200000
#define AXI_PHYS_SIZE		0x00200000

#define DMCU_PHYS_BASE		0xd0000000
#define DMCU_VIRT_BASE		0xfe500000
#define DMCU_PHYS_SIZE		0x00020000

#ifdef CONFIG_CPU_MMP3
#define PGU_PHYS_BASE		0xe0000000
#define PGU_VIRT_BASE		0xfe400000
#define PGU_PHYS_SIZE		0x00100000

#define SCU_VIRT_BASE		(PGU_VIRT_BASE)
#define GIC_DIST_VIRT_BASE	(PGU_VIRT_BASE + 0x1000)
#define GIC_CPU_VIRT_BASE	(PGU_VIRT_BASE + 0x0100)
#define TWD_VIRT_BASE		(PGU_VIRT_BASE + 0x600)

#define SL2C_PHYS_BASE		0xd0020000
#define SL2C_VIRT_BASE		0xfe800000
#define SL2C_PHYS_SIZE		SZ_8K

#define TZ_HV_PHYS_BASE		0x00000000
#define TZ_HV_VIRT_BASE		0xfe900000
#define TZ_HV_PHYS_SIZE		SZ_1M
#endif

#ifdef CONFIG_CPU_PXA988
#define SCU_PHYS_BASE		0xd1dfe000
#define SCU_VIRT_BASE		0xfe400000
#define SCU_PHYS_SIZE		0x00100000
#define GIC_DIST_VIRT_BASE	(SCU_VIRT_BASE + 0x1000)
#define GIC_CPU_VIRT_BASE	(SCU_VIRT_BASE + 0x0100)
#define TWD_VIRT_BASE		(SCU_VIRT_BASE + 0x0600)

#define SL2C_PHYS_BASE		0xd1dfb000
#define SL2C_VIRT_BASE		0xfe800000
#define SL2C_PHYS_SIZE		SZ_8K
#endif /*CONFIG_CPU_PXA988*/

/* Static Memory Controller - Chip Select 0 and 1 */
#define SMC_CS0_PHYS_BASE	0x80000000
#define SMC_CS0_PHYS_SIZE	0x10000000
#define SMC_CS1_PHYS_BASE	0x90000000
#define SMC_CS1_PHYS_SIZE	0x10000000

#define SRAM_PHYS_BASE		0xd1000000

#ifdef CONFIG_CPU_PXA988
#define SRAM_CP_BASE		SRAM_PHYS_BASE
#define SRAM_CP_SIZE		0x4000

#define SRAM_AUDIO_BASE		(SRAM_CP_BASE + SRAM_CP_SIZE)
#define SRAM_AUDIO_SIZE		0xd000

#define SRAM_VIDEO_BASE		(SRAM_AUDIO_BASE + SRAM_AUDIO_SIZE)
#define SRAM_VIDEO_SIZE		0xe000

#define SRAM_POWER_BASE		(SRAM_VIDEO_BASE + SRAM_VIDEO_SIZE)
#define SRAM_POWER_SIZE		0x1000
#endif

#endif /* __ASM_MACH_ADDR_MAP_H */
