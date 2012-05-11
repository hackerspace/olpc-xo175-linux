/*
 *  linux/arch/arm/mach-mmp/mmp3_coherent_fix.S
 *
 *  Copyright (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/unwind.h>

/*
 * sync region definition:
 *
 * +---------------------+
 * | COHT_STATE_0        | -> offset 0x0
 * +---------------------+
 * | COHT_STATE_1        | -> offset 0x4
 * +---------------------+
 * | HS_REQ_0            | -> offset 0x8
 * +---------------------+
 * | HS_REQ_1            | -> offset 0xc
 * +---------------------+
 * | ...                 |
 * +---------------------+
 * | SEM4                | -> offset 0x20
 * +---------------------+
 * | ...                 |
 * +---------------------+
 * | MP1_C1_CFG          | -> offset 0x100
 * +---------------------+
 * | MP1_C2_CFG          | -> offset 0x104
 * +---------------------+
 * | MP2_C1_CFG          | -> offset 0x108
 * +---------------------+
 * | MP2_C2_CFG          | -> offset 0x10c
 * +---------------------+
 * | MM_C1_CFG           | -> offset 0x110
 * +---------------------+
 * | MM_C2_CFG           | -> offset 0x114
 * +---------------------+
 * | CM_CID_OFFSET       | -> offset 0x118
 * +---------------------+
 * | ...                 |
 * +---------------------+
 * | INT_STATUS_ADDR     | -> offset 0x200
 * +---------------------+
 * | WAKE_STATUS_ADDR    | -> offset 0x204
 * +---------------------+
 * | SGIR_ADDR           | -> offset 0x208
 * +---------------------+
 * | APMU_BASE_ADDR      | -> offset 0x20c
 * +---------------------+
 *
 */

#define MP1_COHT_STATE_OFFSET	0x0
#define MP2_COHT_STATE_OFFSET	0x4
#define MP1_HANDSHAKE_OFFSET	0x8
#define MP2_HANDSHAKE_OFFSET	0xc

#define SEM4_OFFSET		0x20

#define MP1_C1_CFG_OFFSET	0x100
#define MP1_C2_CFG_OFFSET	0x104
#define MP2_C1_CFG_OFFSET	0x108
#define MP2_C2_CFG_OFFSET	0x10c
#define MM_C1_CFG_OFFSET	0x110
#define MM_C2_CFG_OFFSET	0x114
#define CM_CID_OFFSET		0x118

#define INT_STATUS_ADDR_OFFSET	0x200
#define WAKE_STATUS_ADDR_OFFSET	0x204
#define SGIR_ADDR_OFFSET	0x208
#define APMU_BASE_ADDR_OFFSET	0x20c

#define APMU_PJ_C0_CC4		0x0248
#define APMU_PJ_C1_CC4		0x024C
#define APMU_PJ_C2_CC4		0x0250
#define APMU_PJ_IDLE_CFG0	0x0018
#define APMU_PJ_IDLE_CFG1	0x0200
#define APMU_PJ_IDLE_CFG2	0x0204

#define DDR_FREQ_CHG_REQ2	(1 << 22)
#define DDR_FREQ_CHG_REQ1	(1 << 25)
#define AXI_FREQ_CHG_REQ	(1 << 26)
#define PJ_FREQ_CHG_REQ		(1 << 24)

#define DDR_FREQ_CHG_DONE2	(1 << 12)
#define DDR_FREQ_CHG_DONE1	(1 << 4)
#define AXI_FREQ_CHG_DONE	(1 << 5)
#define PJ_FREQ_CHG_DONE	(1 << 3)

/*
 * registers used in the workaround flow.
 */
try	.req	r5		@ lock try value
dfc	.req	r7		@ dfc trigger value
ncb	.req	r9		@ non-cache buffer address
cid	.req	r10		@ core id number
pmu	.req	r11		@ APMU register address

	.macro invalid_and_flush_l1_cache, rtemp
	mov	\rtemp, #0x0
	mcr	p15, 0, \rtemp, c8, c5, 0	@ instruction tlb
	mcr	p15, 0, \rtemp, c8, c6, 0	@ data tlb
	mcr	p15, 0, \rtemp, c8, c7, 0	@ unified tlb

	mcr	p15, 0, \rtemp, c7, c5, 0	@ invalid I cache
	mcr	p15, 0, \rtemp, c7, c5, 6	@ invalid branch predictor array
	mcr	p15, 0, \rtemp, c7, c5, 4	@ instruction barrier
	mcr	p15, 0, \rtemp, c7, c14, 0	@ flush entire d cache
	mcr	p15, 0, \rtemp, c7, c5, 4	@ flush prefetch buffer
	isb
	dsb
	.endm

	.macro get_coht_val, rstate
	cmp	cid, #0x0
	moveq	\rstate, #MP1_COHT_STATE_OFFSET
	movne	\rstate, #MP2_COHT_STATE_OFFSET
	ldr	\rstate, [ncb, \rstate]
	.endm

	.macro set_coht_val, rstate, temp
	cmp	cid, #0x0
	moveq	\temp, #MP1_COHT_STATE_OFFSET
	movne	\temp, #MP2_COHT_STATE_OFFSET
	str	\rstate, [ncb, \temp]
	.endm

	.macro get_hs_val, rstate
	cmp	cid, #0x0
	moveq	\rstate, #MP1_HANDSHAKE_OFFSET
	movne	\rstate, #MP2_HANDSHAKE_OFFSET
	ldr	\rstate, [ncb, \rstate]
	.endm

	.macro set_hs_val, rstate, temp
	cmp	cid, #0x0
	moveq	\temp, #MP1_HANDSHAKE_OFFSET
	movne	\temp, #MP2_HANDSHAKE_OFFSET
	str	\rstate, [ncb, \temp]
	.endm

	.macro get_peer_core_coht_val, rstate
	cmp	cid, #0x0
	movne	\rstate, #MP1_COHT_STATE_OFFSET
	moveq	\rstate, #MP2_COHT_STATE_OFFSET
	ldr	\rstate, [ncb, \rstate]
	.endm

	.macro get_peer_core_hs_val, rstate
	cmp	cid, #0x0
	movne	\rstate, #MP1_HANDSHAKE_OFFSET
	moveq	\rstate, #MP2_HANDSHAKE_OFFSET
	ldr	\rstate, [ncb, \rstate]
	.endm

	.macro set_amp, temp
	mrc	p15, 0, \temp, c1, c0, 1
	bic	\temp, \temp, #(1 << 6) | (1 << 0)	@ Disable SMP/nAMP mode and
	mcr	p15, 0, \temp, c1, c0, 1	@ TLB ops broadcasting
	isb
	dsb
	.endm

	.macro set_smp, temp
	mrc	p15, 0, \temp, c1, c0, 1
	orr	\temp, \temp, #(1 << 6) | (1 << 0)	@ Enable SMP/nAMP mode and
	mcr	p15, 0, \temp, c1, c0, 1	@ TLB ops broadcasting
	isb
	dsb
	.endm

	.macro disable_fw, temp
	mrc	p15, 0, \temp, c1, c0, 1
	bic	\temp, \temp, #(1 << 0)		@ Disable TLB ops broadcasting
	mcr	p15, 0, \temp, c1, c0, 1
	isb
	dsb
	.endm

	.macro enable_fw, temp
	mrc	p15, 0, \temp, c1, c0, 1
	orr	\temp, \temp, #(1 << 0)		@ Enable TLB ops broadcasting
	mcr	p15, 0, \temp, c1, c0, 1
	.endm

	@ exclusive try lock
	.macro  coht_trylock, temp1, temp2, rtry
	mov	\temp1, #SEM4_OFFSET
	add	\temp1, \temp1, ncb
	mov	\temp2, #1

	ldrex	\rtry, [\temp1]
	cmp	\rtry, #0			@ unlocked
	strexeq	\rtry, \temp2, [\temp1]
	.endm

	@ exclusive unlock
	.macro  coht_unlock, temp1, temp2, temp3
	mov	\temp1, #SEM4_OFFSET
	add	\temp1, \temp1, ncb
	mov	\temp3, #0
2:
	ldrex	\temp2, [\temp1]
	strex	\temp2, \temp3, [\temp1]
	cmp	\temp2, #0
	bne	2b
	.endm

	.macro	get_cid, rcid
	mrc	p15, 0, \rcid, c0, c0, 5	@ read core id
	and	\rcid, \rcid, #0xf
	.endm

ENTRY(req_peer_core_dis_fw)
	get_peer_core_coht_val r4
	cmp	r4, #0x0
	beq	dis_fw_done

	ldr	r1, [ncb, #SGIR_ADDR_OFFSET]
	cmp	cid, #0x0
	moveq	r0, #0x2		@ sgi for mp2
	movne	r0, #0x1		@ sgi for mp1
	lsl	r0, r0, #0x10		@ left shift 16 bits
	orr	r0, r0, #0x8		@ irq number is 8
	str	r0, [r1]		@ send ipi
wait_hs:
	get_peer_core_coht_val r0
	cmp	r0, #0x0
	bne	wait_hs

dis_fw_done:
	mov	pc, lr
ENDPROC(req_peer_core_dis_fw)

ENTRY(req_peer_core_en_fw)
	cmp	r4, #0x0
	beq	en_fw_done
wait_en_hs:
	get_peer_core_coht_val r0
	cmp	r0, #0x1
	bne	wait_en_hs

en_fw_done:
	mov	pc, lr
ENDPROC(req_peer_core_en_fw)

/*
 * mmp3_trigger_dfc_ll()
 *
 * low level function for triggerring DFC; should forbid any snooping
 * operation during DFC, so need set the core to AMP mode and do not
 * access any data which stored in cache.
 *
 * r0 = dfc value
 * r1 = noncache buffer base address
 *
 */
ENTRY(mmp3_trigger_dfc_ll)
	stmfd	sp!, {r4-r5, r7, r9-r11, lr}

	mov	dfc, r0			@ save dfc value
	mov	ncb, r1			@ save base addr
	get_cid cid			@ read core id
	ldr	pmu, [ncb, #APMU_BASE_ADDR_OFFSET] @ ampu addr

	mov	try, #0x0
	coht_trylock r0, r1, try
	cmp	try, #0x0
	bne	dfc_finish

	mov	r0, #0x1		@ set handshake flag
	set_hs_val r0, r1

	disable_fw r0
	invalid_and_flush_l1_cache r0	@ flush L1 cache

	mov	r0, #0x0		@ clear coherent state
	set_coht_val r0, r1

	bl	req_peer_core_dis_fw

	set_amp	r0
	str	dfc, [pmu, #0x4]	@ set meltres cc register

	mov	r1, #0x0
	tst	dfc, #PJ_FREQ_CHG_REQ
	orrne	r1, r1, #PJ_FREQ_CHG_DONE
	tst	dfc, #AXI_FREQ_CHG_REQ
	orrne	r1, r1, #AXI_FREQ_CHG_DONE
	tst	dfc, #DDR_FREQ_CHG_REQ1
	orrne	r1, r1, #DDR_FREQ_CHG_DONE1
	tst	dfc, #DDR_FREQ_CHG_REQ2
	orrne	r1, r1, #DDR_FREQ_CHG_DONE2

check_dfc_finish:
	ldr	r0, [pmu, #0xa0]
	and	r0, r0, r1
	cmp	r0, r1
	bne	check_dfc_finish

	set_smp	r0
	invalid_and_flush_l1_cache r0

	mov	r0, #0x1		@ set coherent state
	set_coht_val r0, r1

	mov	r0, #0x0		@ clear handshake flag
	set_hs_val r0, r1

	bl	req_peer_core_en_fw

	coht_unlock r0, r1, r2

dfc_finish:
	mov	r0, try			@ set return value

	ldmfd	sp!, {r4-r5, r7, r9-r11, lr}
	mov	pc, lr
ENDPROC(mmp3_trigger_dfc_ll)

/*
 * mmp3_coherent_handler()
 *
 * r0 = noncache buffer base address
 *
 */
ENTRY(mmp3_coherent_handler)
	stmfd	sp!, {r4-r5, r7, r9-r11, lr}

	mov	ncb, r0			@ save base addr
	get_cid cid			@ read core id

	disable_fw r0
	mov	r0, #0x0		@ clear coherent state
	set_coht_val r0, r1

wait_req_done:
	get_peer_core_hs_val r0
	cmp	r0, #0x0
	bne	wait_req_done

	enable_fw r0
	mov	r0, #0x1		@ set coherent state
	set_coht_val r0, r1

	mov	r0, #0x0
	ldmfd	sp!, {r4-r5, r7, r9-r11, lr}
	mov	pc, lr
ENDPROC(mmp3_coherent_handler)

/*
 * mmp3_enter_c2()
 *
 * r0 = noncache buffer base address
 *
 */
ENTRY(mmp3_enter_c2)
	stmfd	sp!, {r4-r11, lr}

	mov	ncb, r0			@ save base addr
	get_cid cid			@ read core id
	ldr	pmu, [ncb, #APMU_BASE_ADDR_OFFSET] @ ampu addr

	mov	try, #0x0
	coht_trylock r0, r1, try
	cmp	try, #0x0
	bne	exit_c2

	mov	r0, #0x1		@ set handshake flag
	set_hs_val r0, r1

	disable_fw r0
	invalid_and_flush_l1_cache r0	@ flush L1 cache

	mov	r0, #0x0		@ clear coherent state
	set_coht_val r0, r1

	bl	req_peer_core_dis_fw

	set_amp	r0
	mov	r0, #0x0		@ clear handshake flag
	set_hs_val r0, r1

	bl	req_peer_core_en_fw

	coht_unlock r0, r1, r2

run_into_c2:
	bl	prepare_into_c2
	wfi
	bl	clear_exit_c2

	ldr	r0, [ncb, #INT_STATUS_ADDR_OFFSET]
	ldr	r1, [r0]
	mov	r2, #0x400
	sub	r2, r2, #1		@ test for irq 1023
	and	r1, r1, r2
	cmp	r1, r2			@ 1023 means suprious irq
	bne	acquire_lock

	ldr	r0, [ncb, #WAKE_STATUS_ADDR_OFFSET]
	ldr	r1, [r0]
	mov	r2, #0x8
	lsl	r2, r2, #28
	bic	r1, r1, r2		@ clear bit 31
	mov	r2, #0x38
	lsl	r2, r2, #8
	bic	r1, r1, r2		@ clear bit 0x3800
	cmp	r1, #0x0
	beq	run_into_c2		@ no irq, so run into c2 again

acquire_lock:
	coht_trylock r0, r1, try
	cmp	try, #0x0
	bne	run_into_c2

	mov	r0, #0x1		@ set handshake flag
	set_hs_val r0, r1

	bl	req_peer_core_dis_fw

	set_smp	r0
	invalid_and_flush_l1_cache r0

	mov	r0, #0x1		@ set coherent state
	set_coht_val r0, r1

	mov	r0, #0x0		@ clear handshake flag
	set_hs_val r0, r1

	bl	req_peer_core_en_fw

	coht_unlock r0, r1, r2

exit_c2:
	mov	r0, try

	ldmfd	sp!, {r4-r11, lr}
	mov	pc, lr
ENDPROC(mmp3_enter_c2)

prepare_into_c2:
	cmp	cid, #0x1
	beq	load_mp2_set		@ check is mp2

	/*
	 * note: here cannot directly use CM_CID to check
	 * the core is mp1/mp2/mm core; should be aware there
	 * may have two cores run into C2 at the same time,
	 * so these two cores will set the CM_CID value and let
	 * this value cannot be trusted. So CM_CID is only used
	 * b/t mp1 and mm core, this is based on the assumption
	 * of core morphing, for mp1 and mm core cannot run
	 * at the same time.
	 */
	ldr	r0, [ncb, #CM_CID_OFFSET]
	cmp	r0, #0x2
	beq	load_mm_set		@ check is mm

	mov	r1, #APMU_PJ_IDLE_CFG0	@ mp1 mk apmu
	mov	r2, #APMU_PJ_C0_CC4	@ mp1 cc4
	ldr	r3, [ncb, #MP1_C1_CFG_OFFSET]
	ldr	r4, [ncb, #MP1_C2_CFG_OFFSET]
	b	set_idle_cfg

load_mp2_set:
	mov	r1, #APMU_PJ_IDLE_CFG1	@ mp2 mk apmu
	mov	r2, #APMU_PJ_C1_CC4	@ mp2 cc4
	ldr	r3, [ncb, #MP2_C1_CFG_OFFSET]
	ldr	r4, [ncb, #MP2_C2_CFG_OFFSET]
	b	set_idle_cfg

load_mm_set:
	mov	r1, #APMU_PJ_IDLE_CFG2	@ mm mk apmu
	mov	r2, #APMU_PJ_C2_CC4	@ mm cc4
	ldr	r3, [ncb, #MM_C1_CFG_OFFSET]
	ldr	r4, [ncb, #MM_C2_CFG_OFFSET]

set_idle_cfg:
	str	r4, [pmu, r1]		@ set idle config

	ldr	r0, [pmu, r2]		@ mask gic
	orr	r0, r0, #0x1
	str	r0, [pmu, r2]

	mov	pc, lr
ENDPROC(prepare_into_c2)

clear_exit_c2:				@ r0 = apmu base address
	str	r3, [pmu, r1]		@ set idle config

	ldr	r0, [pmu, r2]		@ unmask gic
	bic	r0, r0, #0x1
	str	r0, [pmu, r2]

	mov	pc, lr
ENDPROC(clear_exit_c2)
