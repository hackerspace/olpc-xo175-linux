/*
 * Low-level frequency change code
 *
 * Copyright (C) 2010, Marvell Semicondutor.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */

#include <mach/mmp2_pm.h>

	.global bs_do_fcs, bs_do_fcs_end

	.macro	status_clear
	ldr	r7, [r6, #0x18]
	and	r7, r7, #0x6f0
	str	r7, [r6, #0x18]
	.endm

	.macro	wait_delay
2:	ldr	r7, [r6, #0x18]
	and	r7, r7, #0x6f0
	str	r7, [r6, #0x18]
	tst	r7, #0x40
	beq	2b
	.endm

@******************************************************************************
@
@ bs_do_fcs
@
@ frequency change sequence
@
@ Inputs:
@	r0 = va of PMUA
@       r1 = va of DMCU
@       r2 = cc_ap value to be written
@       r3 = fccr value
@       r4 = va of PMUM
@       r5 = decrease or raise voltage
@	r6 = memory config
@	r7 = priv -- value to be written to pmic
@
@ Outputs:
@	None
@

bs_do_fcs:
	@ save registers on stack
	stmfd	sp!, {r1 - r12, lr}
	@ mov	r0, r2
	@ ldmfd	sp!, {r1 - r12, pc}

	sub	sp, sp, #8

	ldr	r9, [r7]	@ r9 = voltage value written to pmic
	mov	r7, r6		@ r7 = mem config
	mov	r10, r5		@ r10 = decrease or raise voltage
	mov     r11, r4		@ r11 = va of pmum
	mov     r8, r3		@ r8 = fccr
	mov     r4, r0		@ r4 = va of PMUA

	mov	r6, #0x1000
	orr	r6, r6, #0x010000
	orr	r6, r6, #0xfe000000

	mov     r0, #0
	mcr     p15, 0, r0, c7, c10, 4          @ drain write buffer
	mcr	p15, 0, r0, c7, c10, 5		@ flush D&I TLB
	mcr	p15, 0, r0, c8, c7, 0
	mcr	p15, 0, r0, c8, c5, 0
	mcr	p15, 0, r0, c8, c6, 0

	@ touch the registers so as to fill the cache
	ldr	r0, [r11]	@ pmum
	ldr	r0, [r4]	@ pmua
	ldr	r0, [r1]	@ dmcu
	ldr	r0, [r6]

	mov     r0, #0
	str     r0, [sp]
	str     r1, [sp, #4]

	ldr     r5, [r11, #8]	@ value of fccr
	bic     r5, r5, #0x03800000
	and     r0, r8, #0x03800000
	orr     r5, r5, r0
	str     r5, [r11, #8]

	mov	r0, r7		@ keep the mem_config in r0

1:	ldr	r7, [r6, #0x18]
	and	r7, r7, #0xc
	cmp	r7, #0x0
	bne	1b

	@ disable DDR APS
	ldr	r7, =0x0
	str	r7, [r1, #0x080]

	@ ddr blocking
	ldr	r7, =0x1
	str	r7, [r1, #0x7e0]
	ldr	r7, =0x40
	str	r7, [r1, #0x120]

	tst	r10, #(1 << 0)		@ 1 means we need to raise voltage now
	beq	setup_ddr_table

	mov	r6, #0X1000
	orr	r6, r6, #0x010000
	orr	r6, r6, #0xfe000000

	status_clear

	ldr	r7, =0xc0
	str	r7, [r6, #0x08]		@ 0xc0 -> 0xfe011008
	ldr	r7, =0x8700
	orr	r7, r7, #0xe9
	str	r7, [r6, #0x10]		@ start to send a byte -> 0xfe011010
	wait_delay

	ldr	r7, =0x02
	str	r7, [r6, #0x08]
	ldr	r7, =0x9700
	orr	r7, r7, #0xe8
	str	r7, [r6, #0x10]
	wait_delay

	str	r9, [r6, #0x08]
	ldr	r7, =0x9700
	orr	r7, r7, #0xea
	str	r7, [r6, #0x10]
	wait_delay

	ldr	r7, =0x97e0
	str	r7, [r6, #0x10]

	ldr	r7, =0x0
	str	r7, [r4 ,#0x11c]

setup_ddr_table:
	mov	r6, r0		@ keep mem_config in r6

	cmp	r6, #0
	bne	micron_ddr_200
micron_ddr_100:

	@ Test for HW DDR calibration
	ldr	r7, =0x4
	str	r7, [r4, #0x11c]
	b	bus_fcs

micron_ddr_200:
	cmp	r6, #1
	bne	micron_ddr_400

	@ Test for HW DDR calibration
	ldr	r7, =0x5
	str	r7, [r4, #0x11c]
	b	bus_fcs

micron_ddr_400:
	cmp	r6, #MICRON_400
	bne	elpida_ddr_100

	@ Test for HW DDR calibration
	ldr	r7, =0x6
	str	r7, [r4, #0x11c]
	b	bus_fcs

elpida_ddr_100:
	cmp	r6, #ELPIDA_100
	bne	elpida_ddr_200

	@ Test for HW DDR calibration
	ldr	r7, =0x4
	str	r7, [r4, #0x11c]
	b	bus_fcs

elpida_ddr_200:
	cmp	r6, #ELPIDA_200
	bne	elpida_ddr_400

	@ Test for HW DDR calibration
	ldr	r7, =0x5
	str	r7, [r4, #0x11c]
	b	bus_fcs

elpida_ddr_400:
	cmp	r6, #ELPIDA_400
	bne	elpida_1g_ddr_100

	@ Test for HW DDR calibration
	ldr	r7, =0x6
	str	r7, [r4, #0x11c]
	b	bus_fcs

elpida_1g_ddr_100:
	cmp	r6, #ELPIDA_1G_100
	bne	elpida_1g_ddr_200

	@ Test for HW DDR calibration
	ldr	r7, =0x4
	str	r7, [r4, #0x11c]
	b	bus_fcs

elpida_1g_ddr_200:
	cmp	r6, #ELPIDA_1G_200
	bne	elpida_1g_ddr_400

	@ Test for HW DDR calibration
	ldr	r7, =0x5
	str	r7, [r4, #0x11c]
	b	bus_fcs

elpida_1g_ddr_400:
	@ If all mem cfg does not match r6, skip ddr recal
	cmp	r6, #ELPIDA_1G_400
	bne	bus_fcs

	@ Test for HW DDR calibration
	ldr	r7, =0x6
	str	r7, [r4, #0x11c]

bus_fcs:

	tst	r2, #(1<<26)
	beq	ddr_fcs

	ldr	r6, =0xf000000
	bic	r3, r2, r6

	orr	r3, r3, #0x0c000000
	@ orr     r3, r3, #0x80000000
	str     r3, [r4, #4]

wait_for_intr1:
	ldr     r3, [r4, #0xA0]
	tst     r3, #2
	beq     wait_for_intr1

	mov     r0, #0
	str     r0, [r4, #0xA0]

ddr_fcs:
	tst	r2, #(1<<25)
	beq	core_fcs

	ldr	r6, =0xf000000
	bic	r3, r2, r6

	orr	r3, r3, #0x0A000000
	@ orr     r3, r3, #0x80000000
	str     r3, [r4, #4]

wait_for_intr2:
	ldr	r3, [r4, #0xA0]
	tst	r3, #2
	beq	wait_for_intr2
	mov	r0, #0
	str	r0, [r4, #0xA0]

	@ freq change
core_fcs:
	tst	r2, #(1<<24)
	beq	ddr_recal_start

	ldr     r5, [r11, #8]
	bic     r5, r5, #0xe0000000
	and     r0, r8, #0xe0000000
	orr     r5, r5, r0
	str     r5, [r11, #8]

	ldr	r6, =0xf000000
	bic	r3, r2, r6

	orr	r3, r3, #0x09000000
	@ orr     r3, r3, #0x80000000
	str     r3, [r4, #4]

wait_for_intr3:
	ldr     r3, [r4, #0xA0]
	tst     r3, #2
	beq     wait_for_intr3
	mov     r0, #0
	str     r0, [r4, #0xA0]

	nop
	nop

	tst	r10, #(1 << 1)		@ if 2, we need to decrease vol now
	beq	ddr_recal_start

	mov	r6, #0X1000
	orr	r6, r6, #0x010000
	orr	r6, r6, #0xfe000000

	status_clear

	ldr	r7, =0xc0
	str	r7, [r6, #0x08]		@ 0xc0 -> 0xfe011008
	ldr	r7, =0x8700
	orr	r7, r7, #0xe9
	str	r7, [r6, #0x10]		@ start to send a byte -> 0xfe011010
	wait_delay

	ldr	r7, =0x02
	str	r7, [r6, #0x08]
	ldr	r7, =0x9700
	orr	r7, r7, #0xe8
	str	r7, [r6, #0x10]
	wait_delay

	str	r9, [r6, #0x08]
	ldr	r7, =0x9700
	orr	r7, r7, #0xea
	str	r7, [r6, #0x10]
	wait_delay

	ldr	r7, =0x97e0
	str	r7, [r6, #0x10]
	@
	@ ddr re-calibration after frequency change
	@

ddr_recal_start:
	tst     r2, #(1<<25)
	beq     dll_update

	mov	r4, r1		@ r4, va of dmcu

	@ cmp	r7, #255
	@ beq	dummy_read

	ldr	r7, =0x80000000
	str	r7, [r1, #0x240]
	ldr	r7, [r1, #0x230]
	ldr	r8, =0xf0000000
	orr	r7, r7, r8
	str	r7, [r1, #0x230]
	ldr	r7, =0x20000000
	str	r7, [r1, #0x240]

dll_update:
	ldr	r7, =0x40000000
	str	r7, [r1, #0x240]

	ldr	r7, =0x80
	str	r7, [r1, #0x120]

	ldr	r7, [r1, #0x80]
	orr	r7, r7, #0x40
	str	r7, [r1, #0x80]

	ldr	r7, =0x03000100
	str	r7, [r1, #0x120]

	ldr	r7, =0x0
	str	r7, [r1, #0x7e0]

	@ enable DDR APS
	ldr	r7, =0x90045000
	str	r7, [r1, #0x080]

	b	out
dummy_read:
	@ [TODO] need this?
	@ dummy reads for PHY DQ byte read DLLs to update
	mov     r8, #131
	mov     r9, lr
loop:
	ldr     r7, [r9, #0x0]
	subs    r8, r8, #1
	bne     loop

	@ ldr     r7, [r4, #0x240]

out:
	mov	r0, #0xa
	add     sp, sp, #8
	ldmia	sp!, {r1 - r12, pc}
	nop

bs_do_fcs_end:
	nop










