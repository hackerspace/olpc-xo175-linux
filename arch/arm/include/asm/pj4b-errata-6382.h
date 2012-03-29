#ifndef __ASMARM_PJ4B_ERRATA_6382_H
#define __ASMARM_PJ4B_ERRATA_6382_H

#ifdef __ASSEMBLY__
	.macro  pj4b_errata_6382, tmp
	dsb
	mov \tmp, #0000024 @ delay 24 cycles
5:	subs \tmp, \tmp, #0000001
	bne 5b
	.endm

#endif

#define pj4b_errata_6382() \
	do { \
		int tmp; \
		__asm__ __volatile__( \
		"	dsb\n" \
		"	mov %0, #0000024\n" \
		"1:	subs %0, %0, #0000001\n" \
		"	bne 1b\n" \
		: "=&r" (tmp): : "cc", "memory" ); \
	} while(0)

#endif

