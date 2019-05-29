#include <linux/linkage.h>
#include <asm/assembler.h>

#define SDRAM_CTRL1			0x80
#define		DLL_RESET		(1<<6)
#define		AUTO_POWER_SAVING_EN	(1<<31)

#define USER_INITIATED_COMMAND0		0x120
#define		USER_SR_REQ_ENTER	(1<<6)
#define		USER_SR_REQ_EXIT	(1<<7)
#define		USER_LMR0_REQ		(1<<8)
#define		CHIP_SELECT_0		(1<<24)
#define		CHIP_SELECT_1		(1<<25)

#define PHY_CTRL7			0x1d0
#define		PHY_DQ_ZPDRV		(0xf<<24)
#define		PHY_DQ_ZNDRV		(0xf<<20)

#define PHY_CTRL8			0x1e0
#define		PHY_ADCM_ZNDRV_0_7	(0xf<<20)
#define		PHY_ADCM_ZPDRV_0_7	(0xf<<24)

#define PHY_CTRL9			0x1f0
#define		PHY_CK_ZNR		(0xf<<4)
#define		PHY_CK_ZPR		(0xf<<0)

#define PHY_CTRL13			0x230
#define		DLL_UPDATE_STALL_MC_DIS	(1<<27)

#define PHY_CTRL14			0x240
#define		PHY_PLL_RST		(1<<28)
#define		PHY_DLL_RST		(1<<29)
#define		DLL_UPDATE_EN		(1<<30)
#define		PHY_SYNC_EN		(1<<31)

#define SDRAM_CTRL14			0x7e0
#define		BLOCK_ALL_DATA_REQUESTS	(1<<0)

	.global copy_lp_to_sram, do_lp, do_lp_end, jump_to_lp_sram

copy_lp_to_sram:
	stmfd	sp!, {r1 - r12, lr}

	ldr	r1, =do_lp
	ldr	r2, =do_lp_end

do_copy_lp:
	ldmia	r1!, {r5 - r12}
	stmia	r0!, {r5 - r12}
	cmp	r1, r2
	ble	do_copy_lp

	ldmfd	sp!, {r1 - r12, pc}

@******************************************************************
@
@ jump_to_lp_sram
@
@ Inputs:
@	r0 = start address of relocated program
@	r1 = va of dmcu
@
@ Outputs:
@	None
@

jump_to_lp_sram:
	stmfd	sp!, {r1 - r12, lr}

	mov	r12, r0		@ r12 = va of do_lp
	mov	r11, r1		@ r11 = va of dmcu

	mov	lr, pc
	mov	pc, r12

	@ return to C code
	ldmfd	sp!, {r1 - r12, pc}

ENTRY(do_lp)
	@ Flush the TLB, ensure that there are no outstanding requests
	mov     r0, #0
	mcr     p15, 0, r0, c7, c10, 4          @ drain write buffer
	mcr     p15, 0, r0, c7, c10, 5          @ flush D&I TLB
	mcr     p15, 0, r0, c8, c7, 0
	mcr     p15, 0, r0, c8, c5, 0
	mcr     p15, 0, r0, c8, c6, 0

	@ guarantee all the pending transactions have been drained to MC
	dsb

        @ clear APS - Auto Power Save.  chip issue for MMP2
        @ Enable original value on restore
	ldr	r2, [r11, #SDRAM_CTRL1]
	bic	r1, r2, #AUTO_POWER_SAVING_EN
	str	r1, [r11, #SDRAM_CTRL1]

	@ stop cpu / dma access to dram
	mov     r1, #BLOCK_ALL_DATA_REQUESTS
	str     r1, [r11, #SDRAM_CTRL14]

	@software self-refresh mode enter
	mov	r1, #USER_SR_REQ_ENTER
	str	r1, [r11, #USER_INITIATED_COMMAND0]

	@ read register space to allow time for command to complete
	ldr	r3, [r11, #PHY_CTRL8]
	ldr	r3, [r11, #PHY_CTRL8]

	@ reduce drive strength to memory - save power
	ldr	r3, [r11, #PHY_CTRL8]
	bic	r1, r3, #(PHY_ADCM_ZNDRV_0_7 | PHY_ADCM_ZPDRV_0_7)
	str	r1, [r11, #PHY_CTRL8]

	@ save more power by disabling dq drivers
	ldr	r4, [r11, #PHY_CTRL7]
	bic	r1, r4, #(PHY_DQ_ZPDRV | PHY_DQ_ZNDRV)
	str	r1, [r11, #PHY_CTRL7]

	@ save more power by disabling ck drivers
	ldr	r5, [r11, #PHY_CTRL9]
	bic	r1, r5, #(PHY_CK_ZNR | PHY_CK_ZPR)
	str	r1, [r11, #PHY_CTRL9]

	wfi

	@ enable ADCM/DQ/CK drivers from original settings
	str	r5, [r11, #PHY_CTRL9]
	str	r4, [r11, #PHY_CTRL7]

	@ restore original drive strength
	str     r3, [r11, #PHY_CTRL8]

	@ postpone requests until dll update completes
	ldr     r1, [r11, #PHY_CTRL13]
	bic	r1, r1, #DLL_UPDATE_STALL_MC_DIS
	str	r1, [r11, #PHY_CTRL13]

	@ reset PHY DLL
	mov	r1, #PHY_SYNC_EN
	str	r1, [r11, #PHY_CTRL14]
	mov	r1, #PHY_DLL_RST
	str	r1, [r11, #PHY_CTRL14]
	@ DLL update
	mov	r1, #DLL_UPDATE_EN
	str	r1, [r11, #PHY_CTRL14]

	@software self-refresh mode exit
	mov	r1, #USER_SR_REQ_EXIT
	str	r1, [r11, #USER_INITIATED_COMMAND0]

	@ restore orignal sdram ctrl values
	str     r2, [r11, #SDRAM_CTRL1]

	@ unblock data requests
	mov     r1, #0x0
	str     r1, [r11, #SDRAM_CTRL14]

	mov	pc, lr

ENTRY(do_lp_end)
	.word	. - do_lp
