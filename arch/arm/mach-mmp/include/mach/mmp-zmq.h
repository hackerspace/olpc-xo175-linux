/*
 *
 * Marvell MMP2 ZSP message queue driver.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _MMP2_MSG_QUEUE_H_
#define _MMP2_MSG_QUEUE_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZMQ_DEVN_DIAGNOSE	"zmq_diagnose"
#define ZMQ_DEVN_CODEC		"zmq_codec"
#define ZMQ_DEVN_PLAYBACK_0	"zmq_playback0"
#define ZMQ_DEVN_CAPTURE_0	"zmq_capture0"
#define ZMQ_DEVN_PLAYBACK_1	"zmq_playback1"
#define ZMQ_DEVN_CAPTURE_1	"zmq_capture1"

typedef struct {
	u32	_input;
	u32	_output;
	u32	_length;
} zmq_ioc_pmemcpy_param_t;
typedef struct {
	u32	_type_id;
} zmq_ioc_bind_param_t;
enum {
	ZMQ_IOC_BR_OP_CREATE		= 0,
	ZMQ_IOC_BR_OP_DESTORY		= 1,
	ZMQ_IOC_BR_OP_GET_BUFFER	= 2,
	ZMQ_IOC_BR_OP_DELIVER_BUFFER	= 3,
};
typedef struct {
	union{
		struct {
			u32	_op	:8;	/* operations */
			u32	_dir	:1;	/* direction */
			u32		:22;	/* reserved */
			u32	_stat	:1;	/* return val */
		};
		u32	_val;
	};
	union {
		struct {
			u32	_physaddr;	/* in: DMA buffer */
			u32	_size;		/* in: range size, on creation */
			u32	_low;		/* in: water marker, on creation */
		}; /* creation */
		struct {
			u32	_usize;		/* out: user usable size, in: user deliver size */
			u32	_upos;		/* out: user usable position */
		}; /* get buffer */
		struct {
			u32	_vsize;		/* in: delivered buffer */
		}; /* deliver buffer */
	};
} zmq_ioc_br_param_t;
typedef struct {
	u32	_handle;
	void*	_handlex;
} zmq_ioc_session_handle_t;

#define ZMQ_MSG_MK(cid, mid) ((((mid)&0x1f) << 3) | (((cid)&0x7) << 0))
enum
{
	ZMQ_DEV_PORT_OFFSET		= 0x100,
	ZMQ_DEV_MAJOR			= 300,
	ZMQ_DEV_IOC_MAGIC		= ZMQ_DEV_MAJOR,
	ZMQ_DEV_IOC_MAX			= 5,

	ZMQ_IOC_BIND			= _IOW(ZMQ_DEV_IOC_MAGIC, 1, int ),
	ZMQ_IOC_MDMA_PMEMCPY		= _IOW(ZMQ_DEV_IOC_MAGIC, 2, int ),
	ZMQ_IOC_ZSP_SOFT_RESET		= _IOW(ZMQ_DEV_IOC_MAGIC, 3, int ),
	ZMQ_IOC_BR_REQ			= _IOW(ZMQ_DEV_IOC_MAGIC, 4, int ),
	ZMQ_IOC_GET_SESSION_HANDLE	= _IOW(ZMQ_DEV_IOC_MAGIC, 5, int ),
	ZMQ_IOC_SET_SESSION_MASTER	= _IOW(ZMQ_DEV_IOC_MAGIC, 6, int ),
	ZMQ_IOC_CLR_SESSION_MASTER	= _IOW(ZMQ_DEV_IOC_MAGIC, 7, int ),

	/* limit the numbers just for a simple mapping deliver */
	ZMQ_MSG_CLASS_NUMBER		= 6,
	ZMQ_MSG_MODULE_NUMBER		= 8,
	ZMQ_MSG_SESSION_NUMBER		= 8,
	ZMQ_MSG_SEND_DEFAULT_RETRY	= 100,

	ZMQ_MSG_CLASS_CODEC		= 0,
	ZMQ_MSG_ID_CODEC		= ZMQ_MSG_MK(ZMQ_MSG_CLASS_CODEC, 0),

	ZMQ_MSG_CLASS_RENDER		= 1,
	ZMQ_MSG_ID_RENDER_PLAYBACK_0	= ZMQ_MSG_MK(ZMQ_MSG_CLASS_RENDER, 0),
	ZMQ_MSG_ID_RENDER_CAPTURE_0	= ZMQ_MSG_MK(ZMQ_MSG_CLASS_RENDER, 1),
	ZMQ_MSG_ID_RENDER_PLAYBACK_1	= ZMQ_MSG_MK(ZMQ_MSG_CLASS_RENDER, 2),
	ZMQ_MSG_ID_RENDER_CAPTURE_1	= ZMQ_MSG_MK(ZMQ_MSG_CLASS_RENDER, 3),
	ZMQ_MSG_ID_RENDER_PLAYBACK_6CH	= ZMQ_MSG_MK(ZMQ_MSG_CLASS_RENDER, 4),
	ZMQ_MSG_ID_RENDER_PLAYBACK_8CH	= ZMQ_MSG_MK(ZMQ_MSG_CLASS_RENDER, 5),

	ZMQ_MSG_CLASS_SYS		= 3,
	ZMQ_MSG_ID_STREAM		= ZMQ_MSG_MK(ZMQ_MSG_CLASS_SYS, 0),
	ZMQ_MSG_ID_TEST			= ZMQ_MSG_MK(ZMQ_MSG_CLASS_SYS, 3),
	ZMQ_MSG_ID_DAI_0		= ZMQ_MSG_MK(ZMQ_MSG_CLASS_SYS, 4),
	ZMQ_MSG_ID_DAI_1		= ZMQ_MSG_MK(ZMQ_MSG_CLASS_SYS, 5),

};

enum {
	ZMQ_DATA_FLOW_EVTS_LOW		= (1u << 0),
	ZMQ_DATA_FLOW_EVTS_AUTO_STOP	= (1u << 1),

	ZMQ_DATA_FLOW_DIR_P2Z		= (0u << 0),
	ZMQ_DATA_FLOW_DIR_Z2P		= (1u << 0),
};
typedef struct {
	u32	_addr;		/* buffer address (physical) */
	u32	_len;		/* buffer length */
	u32	_low;		/* low point of this descriptor */
	u32	_evts	:15;	/* event map */
	u32	_dir	:1;	/* direction, 0 in(pj->zsp), 1 out(zsp->pj) */
	u32	_qclr	:16;	/* response only, cleared descriptor number from last notification */
} zmq_data_flow_t;
typedef void (* zmq_pfn_data_flow_cb)(zmq_data_flow_t *, void *);
typedef struct {
	u32			_upos;		/* offset */
	u32			_size;
	u32			_low;
	u32			_start;
} kzmq_data_flow_br_t;
typedef void (* kzmq_pfn_data_flow_br_cb)(zmq_data_flow_t *, kzmq_data_flow_br_t *);

typedef struct {
	volatile u32	_class_id	:3;
	volatile u32	_module_id	:5;
	volatile u32	_session	:8;
	volatile u32	_reserved	:16;
} kzmq_info_t;

void *	kzmq_open(int type_id);
void	kzmq_close(void* hkzmq);
void	kzmq_info(void* hkzmq, kzmq_info_t * pinfo);
int	kzmq_read(void* hkzmq, void *pmsg, int len);
int	kzmq_write(void* hkzmq, void *pmsg, int len);
bool	kzmq_delegate(void* hkzmq, void* hmaster);
bool	kzmq_register_data_flow(void* hkzmq, zmq_pfn_data_flow_cb callback, void * pctx);
bool	kzmq_unregister_data_flow(void* hkzmq);
bool	kzmq_data_flow_enqueue(void* hkzmq, u32 addr, u32 len, u32 low, u32 evts, u32 dir);
u32	kzmq_data_flow_pos(void* hkzmq);
bool	kzmq_data_flow_wait(void* hkzmq, u32* pos);

/* buffer ring model */
void *  kzmq_br_create(void * hzmq, u32 start, u32 size, u32 low, u32 dir, u8* vptr, kzmq_pfn_data_flow_br_cb callback);
void	kzmq_br_destory(void * hbr);
u32	kzmq_br_zpos(void* hbr);
bool	kzmq_br_get_buffer(void* hbr, u32 * pupos, u32 * pvalidsize);
bool	kzmq_br_deliver_buffer(void* hbr, u32 size);

#ifdef __cplusplus
}
#endif

#endif
