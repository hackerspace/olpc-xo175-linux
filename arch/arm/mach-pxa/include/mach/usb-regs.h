/*
 * linux/include/asm-arm/arch-pxa/pxa_u2o.h
 *
 * This supports machine-specific differences in how the PXA
 * USB 2.0 Device Controller (U2O) is wired.
 *
 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __ASM_ARCH_PXA_U2O_H
#define __ASM_ARCH_PXA_U2O_H

/* PHY registers */
#define U2PPLL		(0x000)       /* U2O PHY PLL Control */
#define U2PTX		(0x004)       /* U2O PHY TX Control */
#define U2PRX		(0x008)       /* U2O PHY RX Control */
#define U2IVREF		(0x00C)       /* U2O PHY IVREF Control */
#define U2PT0		(0x010)       /* U2O PHY Test 0 Control */
#define U2PT1		(0x014)       /* U2O PHY Test 1 Control */
#define U2PT2		(0x018)       /* U2O PHY Test 2 Control */
#define U2PT3		(0x01C)       /* U2O PHY Test 3 Control */
#define U2PT4		(0x020)       /* U2O PHY Test 4 Control */
#define U2PT5		(0x024)       /* U2O PHY Test 5 Control */
#define U2PID		(0x028)       /* U2O PHY ID Register */
#define U2PRS		(0x02C)       /* U2O PHY Reserve Register */
#define U2PMN		(0x030)       /* U2O PHY Monitor Register */
#define U2OCG		(0x108)       /* U2O Clock Gate Register */

#define PXA935_U2O_REGBASE  (0x55502000)
#define PXA935_U2O_PHYBASE  (0x5550a000)

#define USB_REG_RANGE		(0x1ff)
#define USB_PHY_RANGE		(0xff)

#endif /* __ASM_ARCH_PXA_U2O_H */
