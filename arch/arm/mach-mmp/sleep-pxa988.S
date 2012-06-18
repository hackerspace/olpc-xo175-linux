/*
 * linux/arch/arm/mach-mmp/sleep-pxa988.S
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
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/addr-map.h>
#include <mach/pxa988_lowpower.h>

/*
 * The following Macros should be defined in mach/pxa988_lowpower.h
 *
 * #define LPM_NUM
 * #define L2_MASK
 * #define OFFSET_SPINLOCK
 *
 * The topology of the reserved data is as following.
 * Each core will use 4 bytes to save the flags.
 * The base address is pointed by pm_reserve_pa
 *
 * Note: We can support more than 2 cores here.
 * 	 current we define MAX_CPU_NUM as 2.
 *
 * +---------------------------------------------------------------------+
 * | ... | hotplug | l2_shutdow | LPM[MAX_NUM_LPM - 1] | LPM[1] | LPM[0] |
 * +---------------------------------------------------------------------+
 * | ... | hotplug | l2_shutdow | LPM[MAX_NUM_LPM - 1] | LPM[1] | LPM[0] |
 * +---------------------------------------------------------------------+
 * |     spin_lock                                                       |
 * +---------------------------------------------------------------------+
 */

#define SCU_CPU_POWER_STATUS    (SCU_PHYS_BASE + 0x08)
/* We won't clear C1 and C2 flag in reset code */
#define LPM_CLEAN_MASK		(1 << LPM_C2 | 1 << LPM_C1)


/*
 * Note: The following code is located into the .data section. This is to
 *	 allow l2x0_regs_phys to be accessed with a relative load while we
 *	 can't rely on any MMU translation.
 *	 Reference from: arch/arm/kernel/sleep.S
 */
        .data
        .align

/*
 * r0, CPUID
 * r1, the base physical address of pm reserved space
 */
ENTRY(pxa988_cpu_resume_handler)
#ifdef CONFIG_SMP
	/* fetch the CPU ID */
	mrc 	p15, 0, r0, c0, c0, 5
	and     r0, r0, #15		@ fetch CPUID

	ldr	r2, =pm_reserve_pa
	ldr	r1, [r2]

	/* spin lock */
	mov     r2, #OFFSET_SPINLOCK
	add     r2, r2, r1
	mov     r3, #1
1:	ldrex	r4, [r2]
	teq     r4, #0
	wfene
	strexeq r4, r3, [r2]
	teqeq   r4, #0
	bne     1b
	dmb

	/* Clear cpu flags */
	ldr     r2, [r1, r0, lsl #2]
	mov	r3, #1
	mov     r3, r3, lsl #LPM_NUM	@ 1 << LPM_NUM
	sub	r3, r3, #1
	bic	r3, #LPM_CLEAN_MASK	@ don't clear C1 and C2 flag
	orr	r3, r3, #L2_MASK	@ clear L2_MASK too
	bic	r2, r2, r3
	str     r2, [r1, r0, lsl #2]
#endif

	/* check L2, if disabled, then enable it */
#ifdef CONFIG_CACHE_L2X0
	adr	r2, l2x0_regs_phys
	ldr	r2, [r2]
	ldr	r3, [r2, #L2X0_R_PHY_BASE]	@ phys addr
	ldr	r4, [r3, #L2X0_CTRL]
	tst	r4, #0x1
	bne	l2on
	ldmia   r2!, {r4-r7}
	str     r5, [r3, #L2X0_AUX_CTRL]
	str     r6, [r3, #L2X0_TAG_LATENCY_CTRL]
	str     r7, [r3, #L2X0_DATA_LATENCY_CTRL]
	ldmia   r2!, {r4-r7}
	str     r4, [r3, #L2X0_ADDR_FILTER_START]
	str     r5, [r3, #L2X0_ADDR_FILTER_END]
	str     r6, [r3, #L2X0_PREFETCH_CTRL]
	str     r7, [r3, #L2X0_POWER_CTRL]
	mov	r4, #1
	str	r4, [r3, #L2X0_CTRL]
l2on:
#endif

#ifdef CONFIG_SMP
	/* spin unlock */
	dmb
	mov     r2, #0
	str     r2, [r1, #OFFSET_SPINLOCK]
	dsb
	sev

	/* scu_power_mode(scu_base_addr, SCU_PM_NORMAL) */
	ldr     r1, =SCU_CPU_POWER_STATUS
	ldrb    r2, [r1, r0]
	bic     r2, r2, #0x3
	strb    r2, [r1, r0]
#endif

	b cpu_resume

	.globl l2x0_regs_phys
l2x0_regs_phys:
	.long   0
ENDPROC(pxa988_cpu_resume_handler)
