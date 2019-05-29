/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */
//---------------------------------------------------------------------------
//  Description:    VDEC OS API
//---------------------------------------------------------------------------

#ifndef VDEC_OS_API_H
#define VDEC_OS_API_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct _vmeta_user_info {
    /*in parameters*/
    int usertype;            /*0:dec, 1:enc*/
    int strm_fmt;            /*0:mpeg1, 1:mpeg2, 2:mpeg4, 3:h261, 4:h263, 5:h264, 6:vc1 ap, 7:jpeg, 8:mjpeg, 10:vc1 sp&mp*/
    int width;
    int height;
    int perf_req;            /*-99: expect lowest perf, -1: expect lower perf, 0: default perf, 1: expect higher perf, 99: expect highest perf*/
    /*out parameters*/
    int curr_op;             /*filled by driver, inform high-level user current operation point after user info update*/
}vmeta_user_info;

#ifndef VMETA_OP_MAX
#define VMETA_OP_MAX		15
#define VMETA_OP_MIN		0
#define VMETA_OP_VGA		1
#define VMETA_OP_720P		8
#define VMETA_OP_1080P		14
#define VMETA_OP_VGA_MAX	(VMETA_OP_720P-1)
#define VMETA_OP_720P_MAX	(VMETA_OP_1080P-1)
#define VMETA_OP_1080P_MAX	VMETA_OP_1080P
#define VMETA_OP_VGA_ENC	VMETA_OP_720P
#define VMETA_OP_VGA_ENC_MAX	VMETA_OP_720P_MAX
#define VMETA_OP_INVALID -1
#endif

#ifndef __KERNEL__
//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------

#define VMETA_MAX_OP	VMETA_OP_MAX
#define VMETA_MIN_OP	VMETA_OP_MIN

#ifndef UNSG32
#define UNSG32 unsigned int
#endif

#ifndef SIGN32
#define SIGN32 int
#endif

#ifndef UNSG16
#define UNSG16 unsigned short
#endif

#ifndef SIGN16
#define SIGN16 short
#endif


#ifndef UNSG8
#define UNSG8 unsigned char
#endif

#ifndef SIGN8
#define SIGN8 char
#endif

#ifndef _ASM_LINUX_DMA_MAPPING_H
#define _ASM_LINUX_DMA_MAPPING_H
enum dma_data_direction {
    DMA_BIDIRECTIONAL   = 0,
    DMA_TO_DEVICE       = 1,
    DMA_FROM_DEVICE     = 2,
    DMA_NONE            = 3,
};
#endif

typedef enum _LOCK_RET_CODE {
	LOCK_RET_ERROR_TIMEOUT = -9999,
	LOCK_RET_ERROR_UNKNOWN,
	LOCK_RET_OHTERS_NORM = 0,
	LOCK_RET_NULL,
	LOCK_RET_ME,
	LOCK_RET_FORCE_INIT,
	LOCK_RET_FORCE_TO_OTHERS,
}LOCK_RET_CODE;

//---------------------------------------------------------------------------
// Driver initialization API
//---------------------------------------------------------------------------
SIGN32 vdec_os_driver_init(void);
SIGN32 vdec_os_driver_clean(void);

//---------------------------------------------------------------------------
// Memory operation API
//---------------------------------------------------------------------------
void * vdec_os_api_dma_alloc(UNSG32 size, UNSG32 align, UNSG32 * pPhysical);
void * vdec_os_api_dma_alloc_writecombine(UNSG32 size, UNSG32 align, UNSG32 * pPhysical);
void * vdec_os_api_dma_alloc_cached(UNSG32 size, UNSG32 align, UNSG32 * pPhysical);
void vdec_os_api_dma_free(void *ptr);
void *vdec_os_api_vmalloc(UNSG32 size, UNSG32 align);		// always return VA and can't be translated to PA
void vdec_os_api_vfree(void *ptr);
UNSG32 vdec_os_api_get_va(UNSG32 paddr);
UNSG32 vdec_os_api_get_pa(UNSG32 vaddr);
UNSG32 vdec_os_api_flush_cache(UNSG32 vaddr, UNSG32 size, enum dma_data_direction direction);

//---------------------------------------------------------------------------
// Mem/IO R/W API
//---------------------------------------------------------------------------
UNSG8 vdec_os_api_rd8(UNSG32 addr);
UNSG16 vdec_os_api_rd16(UNSG32 addr);
UNSG32 vdec_os_api_rd32(UNSG32 addr);
void vdec_os_api_wr8(UNSG32 addr, UNSG8 data);
void vdec_os_api_wr16(UNSG32 addr, UNSG16 data);
void vdec_os_api_wr32(UNSG32 addr, UNSG32 data);
UNSG32 vdec_os_api_get_regbase_addr(void);			// return VA

//---------------------------------------------------------------------------
// Interrupt register API
//---------------------------------------------------------------------------
SIGN32 vdec_os_api_set_sync_timeout_isr(UNSG32 timeout);
SIGN32 vdec_os_api_sync_event(void);

//---------------------------------------------------------------------------
// Power Management API
//---------------------------------------------------------------------------
SIGN32 vdec_os_api_power_on(void);
SIGN32 vdec_os_api_power_off(void);
SIGN32 vdec_os_api_suspend_check(void);
void vdec_os_api_suspend_ready(void);
SIGN32 vdec_os_api_clock_on(void);
SIGN32 vdec_os_api_clock_off(void);
SIGN32 vdec_os_api_update_user_info(SIGN32 user_id, vmeta_user_info *info);

//---------------------------------------------------------------------------
// Multi-instance API
//---------------------------------------------------------------------------
SIGN32 vdec_os_api_get_user_id(void);
SIGN32 vdec_os_api_free_user_id(SIGN32 user_id);
SIGN32 vdec_os_api_register_user_id(SIGN32 user_id);
SIGN32 vdec_os_api_unregister_user_id(SIGN32 user_id);
SIGN32 vdec_os_api_get_hw_obj_addr(UNSG32 *vaddr, UNSG32 size);
SIGN32 vdec_os_api_get_hw_context_addr(UNSG32 *paddr, UNSG32 *vaddr, UNSG32 size, SIGN32 flag);
SIGN32 vdec_os_api_lock(SIGN32 user_id, UNSG32 to_ms);
SIGN32 vdec_os_api_unlock(SIGN32 user_id);
SIGN32 vdec_os_api_get_user_count(void);
SIGN32 vdec_os_api_force_ini(void);

#endif // end of #ifndef __KERNEL__

#ifdef __cplusplus
}
#endif

#endif
