/*
 * linux/arch/arm/mach-mmp/pxa988_headsmp.S
 *
 * Copyright (C) 2012 Marvell, Inc.
 *
 * Author: Neil Zhang <zhangwm@marvell.com>
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
#include <linux/init.h>

	__CPUINIT

/*
 * PXA specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 */
ENTRY(pxa988_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #15
	adr	r4, 1f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4
pen:	ldr	r7, [r6]
	cmp	r7, r0
	bne	pen

	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b	secondary_startup
ENDPROC(pxa988_secondary_startup)

	.align	2
1:	.long	.
	.long	pen_release


        .text
/*
 * set the software reset flag
 * r0 = cpu id
 */
ENTRY(pxa988_set_sreset_flag)
	ldr     r1, =sw_reset_flag
	mov     r2, #0x1
	str     r2, [r1, r0, lsl #2]    @ set the sw reset flag

	mov     pc, lr                  @ return
ENDPROC(pxa988_set_sreset_flag)

/*
 * Note: The following code is located into the .data section. This is to
 *       allow sw_reset_flag and cpu_plugin_handler to be accessed with a
 *       relative load while we can't rely on any MMU translation.
 *       Reference from: arch/arm/kernel/sleep.S
 */

	.data
	.align

/*
 * This function will bring up the secondary cpu
 * r0 = CPUID
 * r1 = sw_reset_flag
 */
pxa988_sreset_handler:
	mov     r2, #0
	str     r2, [r1, r0, lsl #2]    @ clear sw reset flag

	adr     r2, secondary_cpu_handler
	ldr     r3, [r2]
	mov     pc, r3			@ jump to pxa_secondary_startup

/*
 * ROM code jumps to this function while waking up from CPU
 * OFF or software reset state. Physical address of the function is
 * stored at CA9_WARM_RESET_VECTOR while system is bring up.
 */

ENTRY(pxa988_cpu_reset_handler)
	adr     r1, sw_reset_flag
	mrc 	p15, 0, r0, c0, c0, 5
	and     r0, r0, #15		@ fetch CPUID
	ldr     r2, [r1, r0, lsl #2]    @ fetch sw reset flag
	cmp     r2, #0x1                @ sw reset ?
	beq     pxa988_sreset_handler	@ Yes, jump to sw_reset_handler
					@ No, jump to pxa_cpu_resume
					@ by another patch
ENDPROC(pxa988_cpu_reset_handler)

sw_reset_flag:
	.rept   CONFIG_NR_CPUS
	.long   0			@ preserve sw reset flag
	.endr

	.globl secondary_cpu_handler
secondary_cpu_handler:
	.long   0
