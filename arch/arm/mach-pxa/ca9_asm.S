#define PWRMODE_REG	0x40f40080

#define mode_usr	0x10
#define mode_fiq	0x11
#define mode_irq	0x12
#define mode_svc	0x13
#define mode_abt	0x17
#define mode_und	0x1b
#define mode_sys	0x1f



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

	.global save_pl310
save_pl310:
	stmfd	sp!, {r4-r12,lr}   @r0 context, r1 pl310 virtual address
	ldr   r2,   [r1, #0x100]			@ control
	str   r2,  [r0]
	ldr   r2,  [r1, #0x104]			@aux_control
	str   r2,  [r0, #4]
	ldr   r2,  [r1, #0x108]			@tag_ram_control
	str   r2,  [r0, #8]
	ldr   r2,  [r1, #0x10C]			@data_ram_control
	str   r2,  [r0, #12]
	ldr   r2,  [r1, #0x200]			@ev_counter_ctrl
	str   r2,  [r0, #16]
	ldr   r2,  [r1, #0x204]			@ev_counter1_cfg
	str   r2,  [r0, #20]
	ldr   r2,  [r1, #0x208]			@ev_counter0_cfg
	str   r2,  [r0, #24]
	ldr   r2,  [r1, #0x20C]			@ev_counter1
	str   r2,  [r0, #28]
	ldr   r2,  [r1, #0x210]			@ev_counter0
	str   r2,  [r0, #32]
	ldr   r2,  [r1, #0x214]			@int_mask
	str   r2,  [r0, #36]
	ldr   r2,  [r1, #0x950]			@lock_line_en
	str   r2,  [r0, #40]

	mov r2, r0
	mov r3, #0
LOOP1:
	add r4, r3, #288			@lockdown_reg
	add r3, r3, #1
	cmp r3, #8
	add r5, r1,r4, lsl #3
	ldr  r6, [r5]
	ldr  r4, [r5, #4]
	str  r6,  [r2, #44]
	str  r4,  [r2, #48]
	add r2, r2,#8
	bne LOOP1
	ldr   r2,  [r1, #0xF40]		@debug_ctrl
	str   r2,  [r0, #120]
	ldr   r2,  [r1, #0xF60]		@prefetch_ctrl
	str   r2,  [r0, #124]
	ldr   r2,  [r1, #0xF80]		@power_ctrl
	str   r2,  [r0, #128]
	add r1, r0, #132  		@ the size of pl310_context
	@.extern flushL2VaRange	@ we call clean all before WFI, so this one is uncessary
	@bl	flushL2VaRange
	ldmfd	 sp!, {r4-r12,pc}

	.global restore_pl310
restore_pl310:
	@r0 context, r1 pl310 virtual address
	ldr r3, [r1, #0x100]		@control
	cmp r3, #0
	beq restore_reg
	mov r3, #0
	str  r3, [r1, #0x730]            @cache sync
	dsb
	str  r3, [r1, #0x100] 		@control
restore_reg:
	ldr r3, [r0, #4]			@aux control
	str r3, [r1, #0x104]
	ldr r3, [r0, #8]			@tag_ram_control
	str r3, [r1, #0x108]
	ldr r3, [r0, #12]			@data_ram_control
	str r3, [r1, #0x10C]
	ldr r3, [r0, #16]			@ev_counter_ctrl
	str r3, [r1, #0x200]
	ldr r3, [r0, #20]			@ev_counter1_cfg
	str r3, [r1, #0x204]
	ldr r3, [r0, #24]			@ev_counter0_cfg
	str r3, [r1, #0x208]
	ldr r3, [r0, #28]			@ev_counter1
	str r3, [r1, #0x20C]
	ldr r3, [r0, #32]			@ev_counter0
	str r3, [r1, #0x210]
	ldr r3, [r0, #36]			@int_mask
	str r3, [r1, #0x214]
	ldr r3, [r0, #40]			@lock_line_en
	str r3, [r1, #0x950]
	mov r3, #0
	mov r5, r0
LOOP2:
	add r4, r3, #288		@lockdown_reg
	ldr r6, [r5, #44]
	ldr r7, [r5, #48]
	add r3, r3, #1
	add r8, r1, r4, lsl #3
	cmp r3, #8
	str r6, [r8]
	str r7, [r8, #4]
	add r5, r5, #8
	bne LOOP2
	ldr r3, [r0, #120]		@debug_ctrl
	str r3, [r1, #0xF40]
	ldr r3, [r0, #124]		@prefetch_ctrl
	str r3, [r1, #0xF60]
	ldr r3, [r0, #128]		@power_ctrl
	str r3, [r1, #0xF80]
	dsb
	mov r4, r0
	mov r5, r1
	cmp r2, #0
	.extern invalidL2All
	blne	invalidL2All
	ldr r3, [r4]
	str r3, [r5, #0x100]
	dsb
	pop  {r4-r12,pc}

	.global ca9_enter_c2_wrapper
ca9_enter_c2_wrapper:
	push {r4-r12,lr}
	mov  r4,r0
	mov  r5,r1
	mov  r6,r2
	mov  r7,r3
	bl	 save_pl310
	mov  r0, r7
	bl	 ca9_enter_c2
	mov  r0, r4
	mov  r1, r5
	mov  r2, r6
	b restore_pl310

	.global ca9_enter_c2
ca9_enter_c2:
	stmfd	sp!, {r4-r12,lr}
	@ align: [13..0]=0

	mov	r2, r0, lsr #14
	mov	r2, r2, lsl #14
@ copy reset handler to SRAM
	adr	r3, reset_chunk
	ldrd	r0, [r3]
	strd	r0, [r2]

	adr	r0, ca9_c2_restore
	bl	VirtualToPhysical @ modified r0,r1
	str	r0, [r2, #0x08]

	add	r0, r2, #0x10
	mov	r5, r0
	bl	VirtualToPhysical @ modified r0,r1
	str	r0, [r2, #0x0c]
	mov	r0, r5

@ Save CPU registers
	cps	#mode_abt
	mrs	r1, spsr
	stm	r0!,{r1,sp,lr}
	cps	#mode_und
	mrs	r1, spsr
	stm	r0!,{r1,sp,lr}
	cps	#mode_sys
	stm	r0!,{sp,lr}
	cps	#mode_irq
	mrs	r1, spsr
	stm	r0!,{r1,sp,lr}
	cps	#mode_fiq
	mrs	r1, spsr
	stm	r0!,{r1,r8-r12,sp,lr}
	cps	#mode_svc
	mrs	r1,spsr
	stm	r0!,{r1,sp} @ others will be restored from stack when exiting
	bic	r1, r1, #0x1f @ clear mode
	orr	r1, r1, #mode_usr
	msr	spsr, r1
	stm	r0,{sp,lr}^	@ fetch user mode regs as spsr is set to user mode
	add	r0,r0,#8	@ Cannot use base register update in stm above
	@ don't restore svc mode spsr as we won't need it until restored later
@ Save cp15
	mrc	p15,2,r1,c0,c0,0	@ CSSELR
	mrc	p15,0,r2,c1,c0,1	@ ACTLR
	mrc	p15,0,r3,c1,c0,0	@ SCTLR
	mrc	p15,0,r4,c1,c0,2	@ CPACR
	stm	r0!, {r1-r4}

	mrc p15,0,r1,c1,c0,0   @ SCTLR	 disable L1 cache
	bic r1, r1, #0x1000
	bic r1, r1, #0x0004
	mcr p15,0,r1,c1,c0,0

	mrc	p15,0,r1,c12,c0,0	@ VBAR
	mrc	p15,0,r2,c2,c0,0	@ TTBR0
	mrc	p15,0,r3,c2,c0,1	@ TTBR1
	mrc	p15,0,r4,c2,c0,2	@ TTBCR
	stm	r0!, {r1-r4}

	mrc	p15,0,r1,c3,c0,0	@ DACR
	mrc	p15,0,r2,c7,c4,0	@ PAR
	mrc	p15,0,r3,c10,c2,0	@ PRRR
	mrc	p15,0,r4,c10,c2,1	@ NMRR
	stm	r0!, {r1-r4}

	@ TBD:    CP15 registers 9 and 11
	mrc	p15,0,r1,c13,c0,1	@ CONTEXTIDR
	mrc	p15,0,r2,c13,c0,2	@ TPIDRURW
	mrc	p15,0,r3,c13,c0,3	@ TPIDRURO
	mrc	p15,0,r4,c13,c0,4	@ TPIDRPRW
	stm	r0!, {r1-r4}

	@ Adding new coop regs ASTODO
	mrc	p15,0,r1,c12,c0,1	@ MVBAR
	mrc	p15,5,r2,c15,c5,2	@ Main TLB VA register
	mrc	p15,5,r3,c15,c6,2	@ Main TLB PA register
	mrc	p15,5,r4,c15,c7,2	@ Main TLB Attribute
	stm	r0!, {r1-r4}

	@ Adding new coop regs ASTODO
	mrc	p15,0,r1,c1,c1,0	@ SCRd
	mrc	p15,0,r2,c1,c1,1	@ SDERc
	mrc	p15,0,r3,c1,c1,2	@ NSACR
	mrc	p15,0,r4,c1,c1,3	@ VCR
	stm	r0!, {r1-r4}

	@ Adding new coop regs ASTODO
	mrc	p15,0,r1,c10,c0,0	@ TLB Lockdown Register
	mrc	p15,0,r2,c12,c1,1	@ Virtualization Interrupt Register
	mrc	p15,0,r3,c13,c0,0	@ FCSEIDR
	mrc	p15,0,r4,c13,c0,1	@ CONTEXTIDR
	stm	r0!, {r1-r4}

	mrc	p15,0,r1,c15,c0,0	@ power control reg
	str	r1, [r0], #4
	mrc	p15,0,r1,c0,c0,0	@ Read Main ID Register
	ubfx	r1, r1, #20, #4		@ Extract major version number
	cmp	r3, #2
	blt	savenople		@ PLE only possible in r2p0 onwards
	mrc	p15,0,r1,c11,c0,0	@ Read PLE IDR
	cmp	r3, #0
	beq	savenople		@ No PLE present

	mrc	p15,0,r1,c11,c1,0	@ Read PLE UAR
	mrc	p15,0,r2,c11,c1,1	@ Read PLE PCR
	stm	r0!, {r1, r2}
savenople:
@ Clean and invalidate L1 D-cache: this does not survive C2
@	.extern cleanAndInvalidateCache
@	bl	cleanAndInvalidateCache

@	v7_flush_dcache_all corrupts r0-r7, r9-r11
	.extern v7_flush_dcache_all
	push	{r0-r7,r9-r11}
	bl	v7_flush_dcache_all
	pop		{r0-r7,r9-r11}

	.extern v7_flush_icache_all
	bl v7_flush_icache_all
	@ Clean L2 areas that will be used on restore before L2 is reenabled
	@mov	r0, sp
	@add r1, r0, #0x40 @ actually 9 regs saved on stack
	@.extern flushL2VaRange
	@bl	flushL2VaRange
	@mmu page table may in L2 cache,  it is diffcult to get page table addr
	@so we clean all L2 cache as a temp WR
	.extern cleanL2VaRange
	bl	cleanL2VaRange
@ C2 entry
	dsb
	wfi
@ in case a wakeup is pending, CPU will fall through without being reset!
 .align 5
	mov	r0, r5 @ setup the saved register base and restore as usual
	ldr r8, =0xf2e0011c
	mov r9, #0x2000
	str r9, [r8]
	mov r5, #0x1
	b set_l2_redundency
@ return here from reset chunk
ca9_c2_restore:
set_l2_redundency:
@	mov r2, #0xC3	@60usec
@	mov r2, #0x140
@	orr r2, #0x5	@100usec
	mov r2, #0x790
	orr r2, #0xE 	@600usec

	cmp r5, #0x1
	bne no_mmu
	ldr r1,=0xf2a00010
	ldr r3, [r1]
	b loop

no_mmu:
	ldr r1,=0x40a00010
	ldr r3, [r1]
	b loop
loop:
	ldr r4, [r1]
	sub r4, r4, r3
	cmp r4, r2
	ble loop
	cps	#mode_abt
	ldm	r0!,{r1,sp,lr}
	msr	spsr, r1
	cps	#mode_und
	ldm	r0!,{r1,sp,lr}
	msr	spsr, r1
	cps	#mode_sys
	ldm	r0!,{sp,lr}
	msr	spsr, r1
	cps	#mode_irq
	ldm	r0!,{r1,sp,lr}
	msr	spsr, r1
	cps	#mode_fiq
	ldm	r0!,{r1,r8-r12,sp,lr}
	msr	spsr, r1
	cps	#mode_svc
	ldm	r0!,{r1,sp} @ others will be restored from stack when exiting
	mov	r2, r1
	bic	r1, r1, #0x1f @ clear mode
	orr	r1, r1, #mode_usr
	msr	spsr, r1
	ldm	r0,{sp,lr}^	@ set user mode regs as spsr is set to user mode
	add	r0,r0,#8	@ cannot use base register update in ldm above
	msr	spsr, r2	@ now restore the svc mode spsr
@ Restore cp15
	ldm	r0!, {r1-r4}
	mcr	p15,2,r1,c0,c0,0	@ CSSELR
	mcr	p15,0,r2,c1,c0,1	@ ACTLR
	@ mcr	p15,0,r3,c1,c0,0	@ SCTLR
	mov	r5, r3			@ keep until ready to enable MMU
	mcr	p15,0,r4,c1,c0,2	@ CPACR

	ldm	r0!, {r1-r4}
	mcr	p15,0,r1,c12,c0,0	@ VBAR
	mcr	p15,0,r2,c2,c0,0	@ TTBR0
	mcr	p15,0,r3,c2,c0,1	@ TTBR1
	mcr	p15,0,r4,c2,c0,2	@ TTBCR

	ldm	r0!, {r1-r4}
	mcr	p15,0,r1,c3,c0,0	@ DACR
	mcr	p15,0,r2,c7,c4,0	@ PAR
	mcr	p15,0,r3,c10,c2,0	@ PRRR
	mcr	p15,0,r4,c10,c2,1	@ NMRR

	@ TBD:    CP15 registers 9 and 11
	ldm	r0!, {r1-r4}
	mcr	p15,0,r1,c13,c0,1	@ CONTEXTIDR
	mcr	p15,0,r2,c13,c0,2	@ TPIDRURW
	mcr	p15,0,r3,c13,c0,3	@ TPIDRURO
	mcr	p15,0,r4,c13,c0,4	@ TPIDRPRW

	@ Adding new coop regs ASTODO
	ldm	r0!, {r1-r4}
	mcr	p15,0,r1,c12,c0,1	@ MVBAR
	mcr	p15,5,r2,c15,c5,2	@ Main TLB VA register
	mcr	p15,5,r3,c15,c6,2	@ Main TLB PA register
	mcr	p15,5,r4,c15,c7,2	@ Main TLB Attribute

	@ Adding new coop regs ASTODO
	ldm	r0!, {r1-r4}
	mcr	p15,0,r1,c1,c1,0	@ SCRd
	mcr	p15,0,r2,c1,c1,1	@ SDERc
	mcr	p15,0,r3,c1,c1,2	@ NSACR
	mcr	p15,0,r4,c1,c1,3	@ VCR

	@ Adding new coop regs ASTODO
	ldm	r0!, {r1-r4}
	mrc	p15,0,r1,c10,c0,0	@ TLB Lockdown Register
	mrc	p15,0,r2,c12,c1,1	@ Virtualization Interrupt Register
	mrc	p15,0,r3,c13,c0,0	@ FCSEIDR
	mrc	p15,0,r4,c13,c0,1	@ CONTEXTIDR

	@ others
	ldr	r1, [r0], #4
	mcr	p15,0,r1,c15,c0,0	@ power control reg
	mrc	p15,0,r1,c0,c0,0	@ Read Main ID Register
	ubfx	r1, r1, #20, #4		@ Extract major version number
	cmp	r3, #2
	blt	restorenople		@ PLE only possible in r2p0 onwards
	mrc	p15,0,r1,c11,c0,0	@ Read PLE IDR
	cmp	r3, #0
	beq	restorenople		@ No PLE present

	ldm	r0!, {r1, r2}
	mcr	p15,0,r1,c11,c1,0	@ set PLE UAR
	mcr	p15,0,r2,c11,c1,1	@ set PLE PCR
restorenople:
	@ Enable MMU now
	ldr	r0, return_addr		@ this addr is with MMU on
	b	cont
.align 5
cont:
	mcr	p15,0,r5,c1,c0,0	@ SCTLR
	bx	r0
return_on_exit:
	ldmfd	sp!,{r4-r12,pc}
return_addr:
	.long	return_on_exit

@ reset handler for C2 exit
reset_chunk:
	ldr	r0, c2_data
	ldr	pc, c2_restore
c2_restore:
	.long	0
c2_data:
	.long	0

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

	.fpu    neon
/* See arch/arm/include/asm/vfp.h */
#define FPEXC		cr8
#define FPSCR		cr1
#define MVFR0		cr7
	.macro  VMRS, rd, sysreg, cond
	MRC\cond        p10, 7, \rd, \sysreg, cr0, 0    @ FMRX  \rd, \sysreg
	.endm

	.macro  VMSR, sysreg, rd, cond
	MCR\cond        p10, 7, \rd, \sysreg, cr0, 0    @ FMXR  \sysreg, \rd
	.endm

	.global save_vfp
save_vfp:
        @ FPU state save/restore.
        @ FPSID,MVFR0 and MVFR1 don't get serialized/saved (Read Only).
	MRC	p15,0,r3,c1,c0,2	@ CPACR allows CP10 and CP11 access
	ORR	r2,r3,#0xF00000
	MCR	p15,0,r2,c1,c0,2
	ISB
	MRC	p15,0,r2,c1,c0,2
	AND	r2,r2,#0xF00000
	CMP	r2,#0xF00000
	BEQ	L000
	MOVS	r2, #0
	@ Override to 0 to indicate that no FPU is present
	@	STR     r2,[r11,#DM_VFP]		; TODO: autodetect VFP in C!!
	B	L001

L000:	@	Save configuration registers and enable.
	VMRS	r12,FPEXC
	STR	r12,[r0],#4		@ Save the FPEXC
        @ Enable FPU access to save/restore the other registers.
	LDR	r2,=0x40000000
	VMSR	FPEXC,r2
	VMRS	r2,FPSCR
	STR	r2,[r0],#4		@ Save the FPSCR
        @ Store the VFP-D16 registers.
	VSTM	r0!, {D0-D15}
        @ Check for Advanced SIMD/VFP-D32 support
	VMRS	r2,MVFR0
	AND	r2,r2,#0xF		@ extract the A_SIMD bitfield
	CMP	r2, #0x2
	BLT	L001
        @ Store the Advanced SIMD/VFP-D32 additional registers.
	VSTM	r0!, {D16-D31}

		@ IMPLEMENTATION DEFINED: save any subarchitecture defined state
		@ NOTE: Don't change the order of the FPEXC and CPACR restores
	VMSR	FPEXC,r10         @ Restore the original En bit of FPU.
L001:
	MCR	p15,0,r3,c1,c0,2 @ Restore the original CPACR value.
	BX	lr

	.global restore_vfp
restore_vfp:
	@ FPU state save/restore. Obviously FPSID,MVFR0 and MVFR1 don't get
	@ serialized (RO).
	@ Modify CPACR to allow CP10 and CP11 access
	MRC	p15,0,r2,c1,c0,2
	ORR	r2,r2,#0x00F00000
	MCR	p15,0,r2,c1,c0,2
	@ Enable FPU access to save/restore the rest of registers.
	LDR	r2,=0x40000000
	VMSR	FPEXC, r2
	@ Recover FPEXC and FPSCR. These will be restored later.
	LDM	r0!,{r3,r12}
	@ Restore the VFP-D16 registers.
	VLDM	r0!, {D0-D15}
	@ Check for Advanced SIMD/VFP-D32 support
	VMRS	r2, MVFR0
	AND	r2,r2,#0xF		@ extract the A_SIMD bitfield
	CMP	r2, #0x2
	BLT	L0000

	@ Store the Advanced SIMD/VFP-D32 additional registers.
	VLDM	r0!, {D16-D31}

	@ IMPLEMENTATION DEFINED: restore any subarchitecture defined state

L0000:	@ Restore configuration registers and enable.
	@ Restore FPSCR _before_ FPEXC since FPEXC could disable FPU
	@ and make setting FPSCR unpredictable.
	VMSR	FPSCR,r12
	VMSR	FPEXC,r3		@ Restore FPEXC after FPSCR
	BX	lr

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
	.global save_ca9_debug
save_ca9_debug:
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
	.global restore_ca9_debug
restore_ca9_debug:
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
