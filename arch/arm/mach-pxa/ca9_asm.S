#include <asm/hardware/cache-l2x0.h>
#define PXA978_L2CACHE_BASE		0x58120000
#define OSCR_PHY_ADDR			0x40A00010
#define OSCR_VIRTUAL_ADDR		0xF2A00010
#define POWER_MODE_BASE		0x40F40080
	.global dump_cop_regs
dump_cop_regs:
	stmfd	sp!, {r4-r12,lr}
	mrc	p15,0,r1,c0,c0,0	@ MIDR
	mrc	p15,2,r2,c0,c0,0	@ CSSELR
	mrc	p15,0,r3,c1,c0,0	@ SCTLR
	mrc	p15,0,r4,c1,c0,1	@ ACTLR
	mrc	p15,0,r5,c1,c0,2	@ CPACR
	mrc	p15,0,r6,c1,c1,0	@ SCRd
	mrc	p15,0,r7,c1,c1,1	@ SDERc
	mrc	p15,0,r8,c1,c1,2	@ NSACR
	mrc	p15,0,r9,c1,c1,3	@ VCRc
	mrc	p15,0,r10,c2,c0,0	@ TTBR0
	mrc	p15,0,r11,c2,c0,1	@ TTBR1
	mrc	p15,0,r12,c2,c0,2	@ TTBCR
	stm	r0!, {r1-r12}

	mrc	p15,0,r1,c3,c0,0	@ DACR
	mrc	p15,0,r2,c5,c0,0	@ DFSR
	mrc	p15,0,r3,c5,c0,1	@ IFSR
	mrc	p15,0,r4,c6,c0,0	@ DFAR
	mrc	p15,0,r5,c6,c0,2	@ IFAR
	mrc	p15,0,r6,c7,c4,0	@ PAR
	mrc	p15,0,r7,c9,c12,0	@ PMCR
	mrc	p15,0,r8,c9,c12,1	@ PMCNTENSET
	mrc	p15,0,r9,c9,c12,2	@ PMCNTENCLR
	mrc	p15,0,r10,c9,c12,3	@ PMOVSR
	mrc	p15,0,r11,c9,c12,5	@ PMSELR
	stm	r0!, {r1-r11}

	mrc	p15,0,r1,c9,c13,0	@ PMCCNTR
	mrc	p15,0,r2,c9,c13,1	@ PMXEVTYPER
	mrc	p15,0,r3,c9,c13,2	@ PMXEVCNTR
	mrc	p15,0,r4,c9,c14,0	@ PMUSERENR
	mrc	p15,0,r5,c9,c14,1	@ PMINTENSET
	mrc	p15,0,r6,c9,c14,2	@ PMINTENCLR
	mrc	p15,0,r7,c10,c0,0	@ TLB Lockdown Register
	mrc	p15,0,r8,c10,c2,0	@ PRRR
	mrc	p15,0,r9,c10,c2,1	@ NRRR
	mrc	p15,0,r10,c11,c1,0	@ PLEUAR
	mrc	p15,0,r11,c11,c1,1	@ PLEPCR
	mrc	p15,0,r12,c12,c0,0	@ VBAR
	stm	r0!, {r1-r12}

	mrc	p15,0,r1,c12,c0,1	@ MVBAR
	mrc	p15,0,r2,c12,c1,1	@ Virtualization Interrupt Register
	mrc	p15,0,r3,c13,c0,0	@ FCSEIDR
	mrc	p15,0,r4,c13,c0,1	@ CONTEXTIDR
	mrc	p15,0,r5,c13,c0,2	@ TPIDRURW
	mrc	p15,0,r6,c13,c0,4	@ TPIDRPRW
	mrc	p15,0,r7,c15,c0,0	@ Power Control Register
	mrc	p15,5,r8,c15,c5,2	@ Main TLB VA register
	mrc	p15,5,r9,c15,c6,2	@ Main TLB PA register
	mrc	p15,5,r10,c15,c7,2	@ Main TLB Attribute
	stm	r0!, {r1-r10}

	ldmfd	sp!,{r4-r12,pc}

	.global pxa978_enter_c1
pxa978_enter_c1:
	dsb
	wfi
	bx	lr

	.global pxa978_save_reset_handler
pxa978_save_reset_handler:
	stmfd	sp!, {r4-r12,lr}
	mov	r2, r0, lsr #14
	mov	r2, r2, lsl #14

	@ copy reset handler to SRAM
	adr	r3, reset_chunk
	ldrd	r0, [r3]
	strd	r0, [r2]

	adr	r0, pxa978_c2_restore
	bl	VirtualToPhysical 	@ modified r0,r1
	str	r0, [r2, #0x08]

	add	r0, r2, #0x10
	bl	VirtualToPhysical 	@ modified r0,r1
	str	r0, [r2, #0x0c]
	ldmfd	sp!,{r4-r12,pc}

	.global pxa978_cpu_suspend
pxa978_cpu_suspend:
	.extern pl310_suspend
	bl pl310_suspend

	@disable L1$, for NEVO C0
	mrc p15,0,r1,c1,c0,0
	bic r1, r1, #0x1000
	bic r1, r1, #0x0004
	mcr p15,0,r1,c1,c0,0

/*C2 entry*/
	dsb
	wfi
/*in case a wakeup is pending, CPU will fall through without being reset!*/
 .align 5
	mov	r0, r5 	@ setup the saved register base and restore as usual
	ldr	r1, =OSCR_VIRTUAL_ADDR
	b set_l2_redundency
/*return here from reset chunk*/
pxa978_c2_restore:
	ldr	r6, =PXA978_L2CACHE_BASE
	ldr 	r1, =OSCR_PHY_ADDR
	ldr	r2, =POWER_MODE_BASE
	ldr	r7, [r2]
	and	r7, r7, #0x40
set_l2_redundency:
	mov	r2, #0xC3		@60usec
@	mov r2, #0x140
@	orr r2, #0x5		@100usec
@	mov r2, #0x790
@	orr r2, #0xE 		@600usec
	ldr r3, [r1]
loop:
	ldr r4, [r1]
	sub r4, r4, r3
	cmp r4, r2
	ble loop
	cmp 	r7, #0
	beq	restore_pl310
pl310_inv_all:
	mov	r1, #0xFF
	str 	r1, [r6, #L2X0_INV_WAY]
inv_wait:
	ldr	r2, [r6,#L2X0_INV_WAY]
	and 	r2, r2, r1
	cmp 	r2, #0
	bne	inv_wait
	str	r2, [r6, #L2X0_CACHE_SYNC]
restore_pl310:
	ldr	r0, pl310_saved_regs_phys_addr	@ phys address of l2x0_saved_regs
	ldm 	r0!, {r2-r5}
	str	r3, [r6, #L2X0_AUX_CTRL]
	str	r4, [r6, #L2X0_TAG_LATENCY_CTRL]
	str	r5, [r6, #L2X0_DATA_LATENCY_CTRL]
	ldm 	r0!, {r2-r5}
	str	r2, [r6, #L2X0_ADDR_FILTER_START]
	str	r3, [r6, #L2X0_ADDR_FILTER_END]
	str	r4, [r6, #L2X0_PREFETCH_CTRL]
	str	r5, [r6, #L2X0_POWER_CTRL]
	ldr 	r2, [r0]
	str	r2, [r6, #L2X0_CTRL]

	@invalidate L1 I cache, for nevo c0
	@TODO: to distingush c0 and d0
	mov	r2, #0
	mcr	p15, 0, r2, c7, c1, 0

	b cpu_resume

/*reset handler for C2 exit */
reset_chunk:
	ldr	r0, c2_data
	ldr	pc, c2_restore
c2_restore:
	.long	0
c2_data:
	.long	0

	.global pl310_saved_regs_phys_addr
pl310_saved_regs_phys_addr:
	.long	0	@ preserve to store l2x0_saved_regs_phys_addr

	.global	VirtualToPhysical
VirtualToPhysical:
	mcr 	p15, 0x0, r0, c7, c8, 0 @ va reg, priviledged read access
	mrc	p15, 0x0, r1, c7, c4, 0 @ pa reg
	tst	r1,#1
	bxne	lr	@ transation failed - return the original address
	mov 	r0, r0, lsl #20
	mov	r0, r0, lsr #20
	mov	r1, r1, lsr #12
	mov	r1, r1, lsl #12
	orr	r0, r0, r1
	bx	lr

	.global save_performance_monitors
save_performance_monitors:
	PUSH	{r4, r8, r9, r10}

	@ Ignore:
	@        Count Enable Clear Register
	@        Software Increment Register
	@        Interrupt Enable Clear Register

	MRC	p15,0,r8,c9,c12,0	@ PMon: Control Register
	BIC	r1,r8,#1
	MCR	p15,0,r1,c9,c12,0	@ disable counter updates from here
	ISB				@ 0b0 => PMCR<0>
	MRC	p15,0,r9,c9,c12,3	@ PMon: Overflow Flag Status Reg
	MRC	p15,0,r10,c9,c12,5	@ PMon: Event Counter Selection Reg
	STM	r0!, {r8-r10}
	UBFX	r9,r8,#11,#5		@ extract # of event counters, N
	TST	r9, r9
	BEQ	L1

L0:	SUBS	r9,r9,#1		@ decrement N
	MCR	p15,0,r9,c9,c12,5	@ PMon: select CounterN
	ISB
	MRC	p15,0,r3,c9,c13,1	@ PMon: save Event Type register
	MRC	p15,0,r4,c9,c13,2	@ PMon: save Event Counter register
	STM	r0!, {r3,r4}
	BNE	L0

L1:	MRC	p15,0,r1,c9,c13,0	@ PMon: Cycle Count Register
	MRC	p15,0,r2,c9,c14,0	@ PMon: User Enable Register
	MRC	p15,0,r3,c9,c14,1	@ PMon: Interrupt Enable Set Reg
	MRC	p15,0,r4,c9,c12,1	@ PMon: Count Enable Set Register
	STM	r0!, {r1-r4}

	POP	{r4, r8, r9, r10}
	bx	lr

	.global restore_performance_monitors
restore_performance_monitors:
	PUSH	{r4-r5, r8-r10, lr}
	@ NOTE: all counters disabled by PMCR<0> == 0 on reset

	@ Restore performance counters
	LDM	r0!,{r8-r10}	@ recover first block of PMon context
				@ (PMCR, PMOVSR, PMSELR)
	MOV	r1, #0		@ generate register of all 0's
	MVN	r2, #0		@ generate register of all 1's
	MCR	p15,0,r2,c9,c14,2	@ disable all counter related interrupts
	MCR	p15,0,r2,c9,c12,3	@ clear all overflow flags
	ISB

	UBFX	r12,r8,#11,#5	@ extract # of event counters, N (0-31)
	TST	r12, r12
	BEQ	L20
	MOV	r3, r12		@ for N >0, generate a 2nd copy of N
	MOV	r4, #1
	LSL	r4, r4, r3
	SUB	r4, r4, #1	@ set bits<N-1:0> to all 1's

L00:	SUBS    r3,r3,#1            @ decrement N
	MCR	p15,0,r3,c9,c12,5   @ select Event CounterN
	ISB
	MRC	p15,0,r5,c9,c13,1   @ read Event Type register
	BFC	r5,#0,#8
	MCR	p15,0,r5,c9,c13,1   @ set Event Type to 0x0
	MCR	p15,0,r2,c9,c13,2   @ set Event Counter to all 1's
	ISB
	BNE	L00

	MOV	r3, #1
	BIC	r5, r9, #1<<31
	MCR	p15,0,r5,c9,c12,1	@ enable Event Counters
					@ (PMOVSR bits set)
	MCR	p15,0,r3,c9,c12,0	@ set the PMCR global enable bit
	ISB
	MCR	p15,0,r9,c9,c12,4   @ set event count overflow bits
	ISB
	MCR	p15,0,r4,c9,c12,2   @ disable Event Counters

	@ restore the event counters
L10:	SUBS	r12,r12,#1          @ decrement N
	MCR	p15,0,r12,c9,c12,5  @ select Event CounterN
	ISB
	LDM	r0!,{r3-r4}
	MCR	p15,0,r3,c9,c13,1   @ restore Event Type
	MCR	p15,0,r4,c9,c13,2   @ restore Event Counter
	ISB
	BNE	L10

L20:	TST	r9, #0x80000000		@ check for cycle count overflow flag
	BEQ	L40
	MCR	p15,0,r2,c9,c13,0	@ set Cycle Counter to all 1's
	ISB
	MOV	r3, #0x80000000
	MCR	p15,0,r3,c9,c12,1	@ enable the Cycle Counter
	ISB

L30:	MRC	p15,0,r4,c9,c12,3	@ check cycle count overflow now set
	MOVS	r4,r4			@ test bit<31>
	BPL	L30
	MCR	p15,0,r3,c9,c12,2	@ disable the Cycle Counter

L40:	MCR	p15,0,r1,c9,c12,0	@ clear the PMCR global enable bit
	ISB

	@ restore the remaining PMon registers
	LDM	r0!,{r1-r4}
	MCR	p15,0,r1,c9,c13,0	@ restore Cycle Count Register
	MCR	p15,0,r2,c9,c14,0	@ restore User Enable Register
	MCR	p15,0,r3,c9,c14,1	@ restore Interrupt Enable Set Reg
	MCR	p15,0,r4,c9,c12,1	@ restore Count Enable Set Register
	MCR	p15,0,r10,c9,c12,5	@ restore Event Counter Selection
	ISB
	MCR	p15,0,r8,c9,c12,0	@ restore the PM Control Register
	ISB

	POP	{r4-r5, r8-r10, pc}

	@ Debug: see DDI0388F, 10.2.1
	.global save_pxa978_debug
save_pxa978_debug:
@	push	{r4}
	push	{r1-r4}
	mrc	p14, 0, r1, c0, c6, 0
	mrc	p14, 0, r2, c0, c7, 0
@	mrc	p14, 0, r3, c0, c0, 2
	mrc	p14, 0, r4, c0, c2, 2
	stm	r0!,{r1-r4}
@	mrc	p14, 0, r1, c0, c3, 2
	mrc	p14, 0, r2, c0, c0, 4
	mrc	p14, 0, r3, c0, c1, 4
	mrc	p14, 0, r4, c0, c2, 4
	stm	r0!,{r1-r4}
	mrc	p14, 0, r1, c0, c3, 4
	mrc	p14, 0, r2, c0, c4, 4
	mrc	p14, 0, r3, c0, c5, 4
	mrc	p14, 0, r4, c0, c0, 5
	stm	r0!,{r1-r4}
	mrc	p14, 0, r1, c0, c1, 5
	mrc	p14, 0, r2, c0, c2, 5
	mrc	p14, 0, r3, c0, c3, 5
	mrc	p14, 0, r4, c0, c4, 5
	stm	r0!,{r1-r4}
	mrc	p14, 0, r1, c0, c5, 5
	mrc	p14, 0, r2, c0, c0, 6
	mrc	p14, 0, r3, c0, c1, 6
	mrc	p14, 0, r4, c0, c2, 6
	stm	r0!,{r1-r4}
	mrc	p14, 0, r1, c0, c3, 6
	mrc	p14, 0, r2, c0, c0, 7
	mrc	p14, 0, r3, c0, c1, 7
	mrc	p14, 0, r4, c0, c2, 7
	stm	r0!,{r1-r4}
	mrc	p14, 0, r1, c0, c3, 7
	mrc	p14, 0, r2, c7, c8, 6
	mrc	p14, 0, r3, c7, c9, 6
	stm	r0!,{r1-r3}
@	pop	{r4}
	pop	{r1-r4}
	bx	lr
	.global restore_pxa978_debug
restore_pxa978_debug:
@	push	{r4}
	push	{r1-r4}
	ldm	r0!,{r1-r4}
	mcr	p14, 0, r1, c0, c6, 0
	mcr	p14, 0, r2, c0, c7, 0
@	mcr	p14, 0, r3, c0, c0, 2
	mcr	p14, 0, r4, c0, c2, 2
	ldm	r0!,{r1-r4}
@	mcr	p14, 0, r1, c0, c3, 2
	mcr	p14, 0, r2, c0, c0, 4
	mcr	p14, 0, r3, c0, c1, 4
	mcr	p14, 0, r4, c0, c2, 4
	ldm	r0!,{r1-r4}
	mcr	p14, 0, r1, c0, c3, 4
	mcr	p14, 0, r2, c0, c4, 4
	mcr	p14, 0, r3, c0, c5, 4
	mcr	p14, 0, r4, c0, c0, 5
	ldm	r0!,{r1-r4}
	mcr	p14, 0, r1, c0, c1, 5
	mcr	p14, 0, r2, c0, c2, 5
	mcr	p14, 0, r3, c0, c3, 5
	mcr	p14, 0, r4, c0, c4, 5
	ldm	r0!,{r1-r4}
	mcr	p14, 0, r1, c0, c5, 5
	mcr	p14, 0, r2, c0, c0, 6
	mcr	p14, 0, r3, c0, c1, 6
	mcr	p14, 0, r4, c0, c2, 6
	ldm	r0!,{r1-r4}
	mcr	p14, 0, r1, c0, c3, 6
	mcr	p14, 0, r2, c0, c0, 7
	mcr	p14, 0, r3, c0, c1, 7
	mcr	p14, 0, r4, c0, c2, 7
	ldm	r0!,{r1-r3}
	mcr	p14, 0, r1, c0, c3, 7
	mcr	p14, 0, r2, c7, c8, 6
	mcr	p14, 0, r3, c7, c9, 6
@	pop	{r4}
	pop	{r1-r4}
	bx	lr
