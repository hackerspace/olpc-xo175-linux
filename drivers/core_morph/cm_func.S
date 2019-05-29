/* code that will call core morphing code */

#include <linux/linkage.h>

#define CS_BASE				0xFE906000
#define CS_ACTIVE_CORE_ID_OFFSET	0x8
#define CS_SWAPCODE_OFFSET		0xC

	.text

/*
 * call the coremorphing code
 */
ENTRY(cm_do_swap)
	stmfd	sp!, {r0 - r12, lr}
	ldr	r0, =CS_BASE
	ldr	r0, [r0, #CS_SWAPCODE_OFFSET]
	ldr	lr, =return
	bx	r0

return:
	ldmfd	sp!, {r0 - r12, lr}
	mov	pc, lr
ENDPROC(cm_do_swap)

/* utilities for driver */
ENTRY(get_reg)
	mov	r1, r0
	cmp	r1, #0
	moveq	r0, r0
	moveq	pc, lr
	cmp	r1, #1
	moveq	r0, r1
	moveq	pc, lr
	cmp	r1, #2
	moveq	r0, r2
	moveq	pc, lr
	cmp	r1, #3
	moveq	r0, r3
	moveq	pc, lr
	cmp	r1, #4
	moveq	r0, r4
	moveq	pc, lr
	cmp	r1, #5
	moveq	r0, r5
	moveq	pc, lr
	cmp	r1, #6
	moveq	r0, r6
	moveq	pc, lr
	cmp	r1, #7
	moveq	r0, r7
	moveq	pc, lr
	cmp	r1, #8
	moveq	r0, r8
	moveq	pc, lr
	cmp	r1, #9
	moveq	r0, r9
	moveq	pc, lr
	cmp	r1, #10
	moveq	r0, r10
	moveq	pc, lr
	cmp	r1, #11
	moveq	r0, r11
	moveq	pc, lr
	cmp	r1, #12
	moveq	r0, r12
	moveq	pc, lr
	cmp	r1, #13
	moveq	r0, r13
	moveq	pc, lr
	cmp	r1, #14
	moveq	r0, r14
	moveq   pc, lr
ENDPROC(get_reg)

ENTRY(cm_get_active_core_id)
	ldr	r0, =CS_BASE
	ldrb	r0, [r0, #CS_ACTIVE_CORE_ID_OFFSET]
	mov	pc, lr
ENDPROC(cm_get_active_core_id)
