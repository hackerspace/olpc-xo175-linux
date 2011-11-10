/*
 * Low-level frequency change code
 *
 * Copyright (C) 2010, Marvell Semicondutor.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */

	.global copy_fcs_to_sram, jump_to_fcs_sram

@******************************************************************************
@
@ copy_fcs_to_sram
@
@ copy frequency change sequence into SRAM.
@
@ input:
@       r0 - virtual address in sram
@       r1 - virtual address of bs_do_fcs
@	r2 - virutal address of bs_do_fcs_end
@
@   return:
@       r0 - virtual address of do_fcs_asm in sram
@

copy_fcs_to_sram:
	stmfd	sp!, {r1 - r12, lr}

	add	r2, r2, #0xF0

do_copy_fcs:
	ldmia	r1!, {r5 - r12}
	stmia	r0!, {r5 - r12}
	cmp	r1, r2
	ble	do_copy_fcs

	ldmfd	sp!, {r1 - r12, pc}

@******************************************************************************
@
@ jump_to_fcs_sram
@
@ entry to DVFM seq located in SRAM
@
@ Inputs:
@	r0 = Start address of relocated program
@	r1 = Start address of relocated stack
@	r2 = va of op settings structure
@
@ Outputs:
@	None
@

jump_to_fcs_sram:
	stmfd	sp!, {r3-r12, lr}

	mov	r12, r0		@ r12 = va of do_fcs
	mov	r11, r2		@ r11 = va of op setting structure
	mov	r10, sp		@ r10 = sp
	mov	sp, r1		@ r1 = stack

	ldr	r0, [r11]	@ r0 = va of pmua
	ldr	r1, [r11, #4]	@ r1 = va of dmcu
	ldr	r2, [r11, #8]	@ r2 = cc_ap
	ldr	r3, [r11, #12]	@ r3 = fccr
	ldr	r4, [r11, #16]	@ r4 = va of pmum
	ldr	r5, [r11, #20]	@ r5 = decrease or raise voltage
	ldr	r6, [r11, #24]	@ r6 = memory config
	ldr	r7, [r11, #28]	@ r7 = priv

	@ jump to fcs in sram
	mov	lr, pc
	add	lr, lr, #4

	mov	pc, r12

	@ return to C code
	mov	sp, r10		@ restore the stack
	ldmfd	sp!, {r3-r12, pc}

