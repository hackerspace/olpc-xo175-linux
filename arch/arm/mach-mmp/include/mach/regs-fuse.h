/*
 * linux/arch/arm/mach-mmp/include/mach/regs-fuse.h
 *
 *   Fuse Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_FUSE_H
#define __ASM_MACH_FUSE_H

#include <mach/addr-map.h>

#define WTM_PHYS_BASE	(AXI_PHYS_BASE + 0x90000)
#define WTM_VIRT_BASE	(AXI_VIRT_BASE + 0x90000)
#define WTM_PHYS_SIZE	(0x3004)

#define MMP2_FUSE_BLK0_CFG1		(0x2904)
#define MMP2_FUSE_BLK0_CFG2		(0x2908)
#define MMP2_FUSE_BLK0_CFG3		(0x290C)
#define MMP2_FUSE_BLK0_CFG4		(0x2910)
#define MMP2_FUSE_BLK0_CFG5		(0x2914)
#define MMP2_FUSE_BLK0_CFG6		(0x2918)
#define MMP2_FUSE_BLK0_CFG7		(0x291C)
#define MMP2_FUSE_BLK0_CFG8		(0x2920)

#define MMP2_FUSE_BLK3_CFG1		(0x2888)
#define MMP2_FUSE_BLK3_CFG2		(0x288C)
#define MMP2_FUSE_BLK3_CFG3		(0x2890)
#define MMP2_FUSE_BLK3_CFG4		(0x2894)
#define MMP2_FUSE_BLK3_CFG5		(0x2898)
#define MMP2_FUSE_BLK3_CFG6		(0x289C)
#define MMP2_FUSE_BLK3_CFG7		(0x28A0)
#define MMP2_FUSE_BLK3_CFG8		(0x29A4)

#define MMP2_FUSE_BLK8_CFG1		(0x28A8)
#define MMP2_FUSE_BLK8_CFG2		(0x28AC)
#define MMP2_FUSE_BLK8_CFG3		(0x28B0)
#define MMP2_FUSE_BLK8_CFG4		(0x28B4)
#define MMP2_FUSE_BLK8_CFG5		(0x28B8)
#define MMP2_FUSE_BLK8_CFG6		(0x28BC)
#define MMP2_FUSE_BLK8_CFG7		(0x28C0)
#define MMP2_FUSE_BLK8_CFG8		(0x28C4)

#define MMP2_FUSE_BLK9_CFG1		(0x28C8)
#define MMP2_FUSE_BLK9_CFG2		(0x28CC)
#define MMP2_FUSE_BLK9_CFG3		(0x28D0)
#define MMP2_FUSE_BLK9_CFG4		(0x28D4)

#endif /* __ASM_MACH_FUSE_H */
