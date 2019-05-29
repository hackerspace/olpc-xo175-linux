/*
 * arch/arm/mach-mmp/delay-mmp3.S
 *
 * Copyright (C) 2011 Marvell, Inc.
 *
 * Author:
 *  Raul Xiong <xjian@marvell.com>
 *
 * based on arch/arm/mach-tegra/delay.S
 *	 Copyright (C) 2010 Google, Inc. by Colin Cross <ccross@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>

	.text

ENTRY(__udelay)
ENTRY(__const_udelay)

/*
 * r4 - cycles we have to wait
 * r5 - cycles before udelay
 * r0 - input useconds/cycles after udelay
 */
	stmdb	sp!, {r4, r5, r6, lr}
	mov	r4, r0
	mrrc	p15, 0, r0, r1, c14
	mov	r5, r0
	mov	r0, r4
	bl	us2cyc
	mov	r4, r0
loop:
	mrrc	p15, 0, r0, r1, c14
	rsb	r0, r5, r0
	cmp	r0, r4
	bcc	loop
end:
	ldmia	sp!, {r4, r5, r6, pc}
ENDPROC(__const_udelay)
ENDPROC(__udelay)

@ Delay routine
ENTRY(__delay)
	subs  r0, r0, #1
	bhi __delay
	mov pc, lr
ENDPROC(__delay)

