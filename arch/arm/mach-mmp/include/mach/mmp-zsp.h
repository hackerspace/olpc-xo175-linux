/*
 *
 * Marvell MMP2 ZSP driver.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _MMP2_ZSP_H_
#define _MMP2_ZSP_H_

#include <linux/types.h>
#include <linux/ioctl.h>
#include<linux/interrupt.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	MMP_ZSP_SPD_PERFORMACE		= 0,
	MMP_ZSP_SPD_BALANCED		= 1,
	MMP_ZSP_SPD_POWERSAVING		= 2,
	MMP_ZSP_SPD_NUMBER		= 3,
	MMP_ZSP_SPD_FLAGS		= (1u << 0),

	MMP_ZSP_CORECLKSRC_PMU		= 0,
	MMP_ZSP_CORECLKSRC_AUDIOPLL	= 1,
	MMP_ZSP_CORECLKSRC_NUMBER	= 2,
	MMP_ZSP_CORECLKSRC_FLAGS	= (1u << 1),

	MMP_ZSP_ASCLK_22579200		= 0,	/* for 44.1k group */
	MMP_ZSP_ASCLK_24576000		= 1,	/* for 48k group */
	MMP_ZSP_ASCLK_NUMBER		= 2,
	MMP_ZSP_ASCLK_FLAGS		= (1u << 2),

	MMP_ZSP_FLAGS_MSK		= 0x7,
};
struct mmp_zsp_clkcfg {
	int spd;
	int src;
	int asclk;
};
struct mmp_zsp_platform_device {
	struct mmp_zsp_clkcfg clkcfg;
	void (*domain_halt)(void);
	void (*domain_on)(int spd, int src, int asclk);
	void (*start_core)(void);
};

#define APMU_AUDIO_CLK_RES_CTRL		APMU_REG(0x010c)
#define ZSP_IPC_BASE			(0x1400)
#define ZSP_TMR_BASE			(0x1500)

#define mmp_zsp_set_bit_range(ref, val, msk, shft) \
	(((ref) & (~((msk) << (shft)))) | (((val) & (msk)) << (shft)))


/* interrupt bit setting for IPC */
#define IPC_INTERRUPT_BIT_0_0	(0x00000001)
#define IPC_INTERRUPT_BIT_0_1	(0x00000002)
#define IPC_INTERRUPT_BIT_0_2	(0x00000004)
#define IPC_INTERRUPT_BIT_0_3	(0x00000008)
#define IPC_INTERRUPT_BIT_0_4	(0x00000010)
#define IPC_INTERRUPT_BIT_0_5	(0x00000020)
#define IPC_INTERRUPT_BIT_0_6	(0x00000040)
#define IPC_INTERRUPT_BIT_0_7	(0x00000080)
#define IPC_INTERRUPT_BIT_1_8	(0x00000100)
#define IPC_INTERRUPT_BIT_2_9	(0x00000200)
#define IPC_INTERRUPT_BIT_3_10	(0x00000400)

/* 16 bits define in ZSP for IPC event id */
enum ipc_client_id{
	IPC_HANDSHAKE_ID		= 0x0,
	IPC_MSG_TRANSFER_ID		= 0x1,
	IPC_DMEM_REQ_ID			= 0x2,
	IPC_DMEM_RLS_ID			= 0x3,
	IPC_PORT_FLOWCONTROL_ID		= 0x4,
	IPC_TEST_ID			= 0x5,
	IPC_IPM_ID			= 0x6,
	IPC_SOFT_RESET_ID		= 0x7,
	IPC_MAX_NUM,
};

enum pzipc_return_code{
	PZIPC_RC_OK = 0,
	PZIPC_RC_FAILURE,
	PZIPC_RC_API_FAILURE,
	PZIPC_RC_WRONG_PARAM,
};

enum pzipc_return_code pzipc_add_isr(enum ipc_client_id client_id,
	irq_handler_t isr_handler);
enum pzipc_return_code pzipc_remove_isr(enum ipc_client_id client_id);
enum pzipc_return_code pzipc_set_interrupt(enum ipc_client_id client_id);
void zsp_local_start(atomic_t *pcounter, int timeout_ms, int retry);
void zsp_local_stop(atomic_t *pcounter);
void* zsp_get_datawnd(void);
int zsp_mmap_datawnd(struct vm_area_struct *vma);
int zsp_set_clock_preference(u32 opmask, struct mmp_zsp_clkcfg * pcfg);

#ifdef __cplusplus
}
#endif

#endif
