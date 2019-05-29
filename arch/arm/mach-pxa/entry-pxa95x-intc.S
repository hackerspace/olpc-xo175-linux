/*
 *  arch/arm/mach-pxa/entry-pxa95x-intc.S
 *
 *  Low-level IRQ helper macros for PXA95x-based platforms
 *
 *  Copyright (C) 2011 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or  modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */
#include <asm/entry-macro-multi.S>
#include <mach/hardware.h>
#include <mach/irqs.h>

	.macro  get_pxa95x_irqnr_and_base, irqnr, irqstat, base, tmp
	mov	\base, #io_p2v(0x40000000)	@ IIR Ctl = 0x40d00000
	add	\base, \base, #0x00d00000
	ldr	\irqstat, [\base, #0x18]	@ ICHP offset
	tst	\irqstat, #0x80000000
	beq	2001f
	bic	\irqstat, \irqstat, #0x80000000
	mov	\irqnr, \irqstat, lsr #16
	add	\irqnr, \irqnr, #(PXA_IRQ(0))
2001:
	.endm

	.align  5
	.global pxa95x_handle_irq_intc
pxa95x_handle_irq_intc:
	mov	r4, lr
1:	get_pxa95x_irqnr_and_base r0, r6, r5, lr
	movne	r1, sp

	@ routine called with r0 = irq number, r1 = struct pt_regs *
	adrne	lr, BSYM(1b)
	bne	asm_do_IRQ
	mov	pc, r4

