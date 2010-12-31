/*
 *  linux/arch/arm/mach-pxa/generic.h
 *
 * Author:	Nicolas Pitre
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct irq_data;
struct sys_timer;

extern struct sys_timer pxa_timer;
extern void __init pxa_init_irq(int irq_nr,
				int (*set_wake)(struct irq_data *,
						unsigned int));
extern void __init pxa25x_init_irq(void);
#ifdef CONFIG_CPU_PXA26x
extern void __init pxa26x_init_irq(void);
#endif
extern void __init pxa27x_init_irq(void);
extern void __init pxa3xx_init_irq(void);
extern void __init pxa95x_init_irq(void);

extern void __init pxa_map_io(void);
extern void __init pxa25x_map_io(void);
extern void __init pxa27x_map_io(void);
extern void __init pxa3xx_map_io(void);

extern unsigned int get_clk_frequency_khz(int info);
extern void pxa95x_handle_irq_intc(struct pt_regs *);

typedef enum {
	OBM_EVB_P_BOARD,/*0*/
	OBM_EVB_PV_BOARD,
	OBM_EVB_PV2_BOARD,
	OBM_SAAR_2_5_BOARD,
	OBM_SAAR_2_6_BOARD,
	OBM_SAAR_PV_3_1_BOARD,/*5*/
	OBM_SAAR_PV_3_2_BOARD,
	OBM_SAAR_PV_3_4_BOARD,
	OBM_SAAR_PV_3_5_BOARD,
	OBM_YARDEN_BOARD,
	OBM_EVB_PV2_3_0_BOARD,/*10*/
	OBM_EVB_PV2_3_1_BOARD,
	OBM_EVB_PV2_JIL_BOARD,
	OBM_EVB_PV2_JIL_15_BOARD,
	OBM_SAAR_B_PV2_BOARD,
	OBM_EVB_PV2_3_2_BOARD,/*15*/
	OBM_EVB_MG1_3_3_BOARD,
	OBM_SAAR_B_PV2_B0_BOARD,
	OBM_SAAR_B_PV2_B0_V1_BOARD,
	OBM_SAAR_B_MG1_C0_V12_BOARD,
	OBM_EVB_MG2_4_0_BOARD,/* 20*/
	OBM_SAAR_B_MG2_A0_V13_BOARD,
	OBM_SAAR_C_MG2_B0_V10_BOARD,
	OBM_SAAR_B_MG2_A0_V14_BOARD,
	OBM_SAAR_B_MG2_B0_V15_BOARD,
	OBM_EVB_ESHEL_1_0_BOARD,
	OBM_EVB_ESHEL_1_0_MCP_BOARD,
	OBM_EVB_ESHEL_SAGIE_BOARD,/*25*/
	OBM_EVB_NEVO_1_0_BOARD,
	OBM_EVB_NEVO_1_1_BOARD,
	OBM_EVB_ESHEL_LTE_1_0_BOARD,
	OBM_SAAR_C2_NEVO_A0_V10_BOARD,
	MAX_BOARD_TYPE
} BOARD_ID_TYPE;
long get_board_id(void);

#define SET_BANK(__nr,__start,__size) \
	mi->bank[__nr].start = (__start), \
	mi->bank[__nr].size = (__size)

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

#ifdef CONFIG_PXA25x
extern unsigned pxa25x_get_clk_frequency_khz(int);
#else
#define pxa25x_get_clk_frequency_khz(x)		(0)
#endif

#ifdef CONFIG_PXA27x
extern unsigned pxa27x_get_clk_frequency_khz(int);
#else
#define pxa27x_get_clk_frequency_khz(x)		(0)
#endif

#if defined(CONFIG_PXA25x) || defined(CONFIG_PXA27x)
extern void pxa2xx_clear_reset_status(unsigned int);
#else
static inline void pxa2xx_clear_reset_status(unsigned int mask) {}
#endif

#ifdef CONFIG_PXA3xx
extern unsigned pxa3xx_get_clk_frequency_khz(int);
#else
#define pxa3xx_get_clk_frequency_khz(x)		(0)
#endif

extern void pxa95x_mem_reserve(void);

extern struct syscore_ops pxa_irq_syscore_ops;
extern struct syscore_ops pxa_gpio_syscore_ops;
extern struct syscore_ops pxa2xx_mfp_syscore_ops;
extern struct syscore_ops pxa3xx_mfp_syscore_ops;

void __init pxa_set_ffuart_info(void *info);
void __init pxa_set_btuart_info(void *info);
void __init pxa_set_stuart_info(void *info);
void __init pxa_set_hwuart_info(void *info);
