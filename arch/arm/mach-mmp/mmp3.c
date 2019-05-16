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

#include "common.h"

static const char *const mmp3_dt_board_compat[] __initconst = {
	"marvell,mmp3",
	NULL,
};

DT_MACHINE_START(MMP2_DT, "Marvell MMP3")
	.map_io		= mmp2_map_io,
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
