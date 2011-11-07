/*
 *
 * Marvell MMP ZSP message queue driver.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>
#include <linux/ipc.h>
#include <linux/ipc.h>
#include <linux/ioctl.h>
#include <linux/mmp2_mdma.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <mach/dma.h>
#include <mach/irqs.h>
#include <mach/regs-zsp.h>
#include <mach/mmp-zsp.h>
#include <mach/mmp-zmq.h>
#include <linux/genalloc.h>

/*
#define ZMQ_USING_WORKQUEUE
#define ZMQ_USING_DIRECT_DISPATCH_MAPPING
*/
#define ZMQ_DIAGNOSE_ENA
#define ZMQ_DT_ENA

#define _countof(array) (sizeof(array)/sizeof((array)[0]))
typedef struct {
	struct gen_pool *zmq_pool;
	u32	zmq_pool_total_size;
	u32	zmq_pool_size;
	void	*ptr;
} zmq_array_allocator;

zmq_array_allocator g_zmq_packet_alc;
#define MSG_POOL_GRANULARITY	4
int zmq_pool_init(int size)
{
	int ret = 0;
	void *ptr;
	ptr =  kzalloc(size , GFP_KERNEL | GFP_ATOMIC);
	g_zmq_packet_alc.zmq_pool = gen_pool_create(ilog2(MSG_POOL_GRANULARITY), -1);
	if (!g_zmq_packet_alc.zmq_pool) {
		ret = -ENOMEM;
		return ret;
	}

	ret = gen_pool_add(g_zmq_packet_alc.zmq_pool, (unsigned long)ptr, size, -1);
	if (ret < 0) {
		ret = -ENOMEM;
		gen_pool_destroy(g_zmq_packet_alc.zmq_pool);
		return ret;
	}
	g_zmq_packet_alc.zmq_pool_total_size = size;
	g_zmq_packet_alc.zmq_pool_size = size;
	return 0;
}

void zmq_pool_uninit(void)
{
	gen_pool_destroy(g_zmq_packet_alc.zmq_pool);
	kfree(g_zmq_packet_alc.ptr);
	return;
}

void *zmq_pool_alloc(size_t len)
{
	unsigned long vaddr;

	if (!g_zmq_packet_alc.zmq_pool)
		return NULL;

	vaddr = gen_pool_alloc(g_zmq_packet_alc.zmq_pool, len);
	if (!vaddr)
		return NULL;
	g_zmq_packet_alc.zmq_pool_size -= len;
	return (void *)vaddr;
}

void zmq_pool_free(void *addr, size_t len)
{
	gen_pool_free(g_zmq_packet_alc.zmq_pool, (unsigned long) addr, len);
	g_zmq_packet_alc.zmq_pool_size += len;
}

#define ZMQ_MSG_SLOT_DELIVER_NUM	(100)

enum {
	ZMQ_DT_SESSION	= 0x00000001,
	ZMQ_DT_SEND	= 0x00000002,
	ZMQ_DT_RECV	= 0x00000004,
	ZMQ_DT_MEM	= 0x00000008,
	ZMQ_DT_INTR	= 0x00000010,
	ZMQ_DT_DF	= 0x00000020,
	ZMQ_DT_DF_BR	= 0x00000040,
	ZMQ_DT_MSGDUMP	= 0x00000080,
	ZMQ_DT_MDMA	= 0x00000100,
	ZMQ_DT_ALL	= 0xffffffff,
};
static volatile u32 zmq_dt_masks = 0;

#ifdef ZMQ_DT_ENA
#define zmq_filter(filter) (((filter) & zmq_dt_masks)==(filter))
#define zmq_objflag(pobj) ((pobj)->_dtena)
#define zmq_objcond(pobj) (((pobj)==NULL)?true:((pobj)->_dtena))
#define zmq_dt(filter, expr) if (zmq_filter(filter)) printk expr
#define zmq_dtobj(pobj, filter, expr) \
	if (zmq_objcond(pobj) && zmq_filter(filter)) printk expr
#define zmq_dtcond(cond, filter, expr) \
	if ((cond) && zmq_filter(filter)) printk expr
#define zmq_dtena(pobj) do { (pobj)->_dtena = true;  } while(0)
#define zmq_dtdsa(pobj) do { (pobj)->_dtena = false; } while(0)
#define ZMQ_DEFOBJDT bool _dtena
#else
#define zmq_filter(filter) (false)
#define zmq_objflag(pobj) (false)
#define zmq_objcond(pobj)
#define zmq_dt(filter, expr)
#define zmq_dtobj(pobj, filter, expr)
#define zmq_dtcond(cond, filter, expr)
#define zmq_dtena(pobj)
#define zmq_dtdsa(pobj)
#define ZMQ_DEFOBJDT
#endif

typedef union {
	struct {
		volatile u8	_class_id	:3;
		volatile u8	_module_id	:5;
		volatile u8	_session	:8;
		volatile u8	_packet_length	:6; /* payload length, no hdr */
		volatile u8	_reserved	:2;
		volatile u8	_function	:7;
		volatile u8	_response	:1;
	};
	volatile u32		_val;
} zmq_packet_header_t;

typedef struct {
	/* PJ write */
	volatile u32	_p_tx_ptr;
	volatile u32	_p_rx_ptr;
	volatile u32	_p_port_map;
	/* ZSP write */
	volatile u32	_z_tx_ptr;
	volatile u32	_z_rx_ptr;
	volatile u32	_z_port_map;
} zmq_shm_key_section_t;

typedef struct {
	u16	_sn;
	u16	_max_queue_items;
	u32	_peek_window;
	u32	_peek_mask;
} zmq_session_request_t;
typedef struct {
	u32	_peek_window;
} zmq_session_end_t;
enum {
	ZMQ_LOGCTL_FLAGS_ENA	= 0x1,
};
typedef struct {
	u32	_flags_clear;
	u32	_flags_set;
} zmq_logctl_t;
enum {
	ZMQ_MDMA_EVT_COMPLETE	= (0x1 << 0),
};
typedef struct { /* addr and size should be 16B aligned */
	u32	_byteaddr_src;
	u32	_byteaddr_dst;
	u32	_byte_cnt	:16;
	u32	_complete	: 1;
	u32	_dir		: 1;
	u32			: 4; /* reserved */
	u32	_sequence	:10;
} zmq_mdma_t;

#define ZMQ_CMD(cmdname)		ZMQ_CMD_##cmdname
#define ZMQ_CMD_LEN(cmdname)		ZMQ_CMD_##cmdname##_LEN
#define ZMQ_DEFCMD(cmdname, id, sz)		\
	ZMQ_CMD(cmdname)		= id,	\
	ZMQ_CMD_LEN(cmdname)		= sz

#define ZMQ_REP(cmdname)		ZMQ_REP_##cmdname
#define ZMQ_REP_LEN(cmdname)		ZMQ_REP_##cmdname##_LEN
#define ZMQ_DEFREP(cmdname, id, sz)		\
	ZMQ_REP(cmdname)		= id,	\
	ZMQ_REP_LEN(cmdname)		= sz

enum
{

	ZMQ_SHM_SIZE			= 0xC00UL,

	ZMQ_MSG_SLOT_SIZE		= 64,
	ZMQ_MSG_PACKET_HEADER_SIEZ	= sizeof(zmq_packet_header_t),
	ZMQ_MSG_LEN_MAX			= ZMQ_MSG_SLOT_SIZE
						- ZMQ_MSG_PACKET_HEADER_SIEZ,

	ZMQ_SHM_HEADER_SIZE		= ZMQ_MSG_SLOT_SIZE,
	ZMQ_SHM_HEADER_PADDING_SIZE	= ZMQ_SHM_HEADER_SIZE
						- sizeof(zmq_shm_key_section_t),

	ZMQ_SHM_PEEK_SLOT_TOTAL_SIZE	= ZMQ_MSG_SLOT_SIZE * 7, /* 112 slot */
	ZMQ_SHM_PEEK_SLOT_NUMBER	= ZMQ_SHM_PEEK_SLOT_TOTAL_SIZE
						/ sizeof(u32),

	ZMQ_SHM_MSG_SLOT_TOTAL_SIZE	= ZMQ_SHM_SIZE - ZMQ_SHM_HEADER_SIZE
						- ZMQ_SHM_PEEK_SLOT_TOTAL_SIZE,

	ZMQ_SHM_SIZE_Z2P		= ZMQ_SHM_MSG_SLOT_TOTAL_SIZE/2,
	ZMQ_SHM_SIZE_P2Z		= ZMQ_SHM_MSG_SLOT_TOTAL_SIZE/2,
	ZMQ_SHM_SLOT_NUMBER_Z2P		= ZMQ_SHM_SIZE_Z2P/ZMQ_MSG_SLOT_SIZE,
	ZMQ_SHM_SLOT_NUMBER_P2Z		= ZMQ_SHM_SIZE_P2Z/ZMQ_MSG_SLOT_SIZE,


	ZMQ_CMD_RESPONSE_ERROR		= 0,
	ZMQ_CMD_RESPONSE_OK		= 1,

	/* command to ZSP */
	/*		name		id	byte */
	ZMQ_DEFCMD(SESSION_REQUEST,	1,	sizeof(zmq_session_request_t)),
	ZMQ_DEFCMD(SESSION_END,		2,	sizeof(zmq_session_end_t)),
	ZMQ_DEFCMD(DATA_FLOW,		3,	sizeof(zmq_data_flow_t)),
	ZMQ_DEFCMD(LOGCTL,		4,	sizeof(zmq_logctl_t)),
	ZMQ_DEFCMD(LOOPBACK,		5,	0),
	ZMQ_DEFCMD(MDMA_NOTIFY,		6,	sizeof(zmq_mdma_t)),

	/* command from ZSP */
	ZMQ_DEFREP(SESSION_REQUEST,	1,	sizeof(zmq_session_request_t)),
	ZMQ_DEFREP(DATA_FLOW,		3,	sizeof(zmq_data_flow_t)),
	ZMQ_DEFREP(LOG,			4,	0),
	ZMQ_DEFREP(MDMA_REQ,		6,	sizeof(zmq_mdma_t)),

	ZMQ_DATA_FLOW_PEEK_POS_MASK	= 0xfffffffc,
	ZMQ_DATA_FLOW_PEEK_FLAGS_MASK	= 0x00000003,
};
#define ZMQ_NEXT_TX_PTR(curr) \
	((curr >= (ZMQ_SHM_SLOT_NUMBER_P2Z - 1)) ? 0 : (curr + 1))
#define ZMQ_NEXT_RX_PTR(curr) \
	((curr >= (ZMQ_SHM_SLOT_NUMBER_Z2P - 1)) ? 0 : (curr + 1))

typedef struct {
	zmq_packet_header_t	_header;
	u8 			_message[ZMQ_MSG_LEN_MAX];
} zmq_packet_t;

typedef struct {
	zmq_shm_key_section_t	_key_section;
	u8			_padding[ZMQ_SHM_HEADER_PADDING_SIZE];
	u32			_peek_slot[ZMQ_SHM_PEEK_SLOT_NUMBER];
	zmq_packet_t		_z_tx_p_rx_queue[ZMQ_SHM_SLOT_NUMBER_Z2P];
	zmq_packet_t		_p_tx_z_rx_queue[ZMQ_SHM_SLOT_NUMBER_P2Z];
} zmq_shm_t;

static void zmq_dump_packet(zmq_packet_t * ppkt, bool insend)
{
	u16 * p;
	int loop;

	if (ppkt->_header._function == ZMQ_REP_LOG) return;/* omit log packets*/

	printk(KERN_INFO "%s[%d-%02d-%02d](%02d:%02d:%1d)-",
		insend?"S":"R",
		ppkt->_header._class_id,
		ppkt->_header._module_id,
		ppkt->_header._session,
		ppkt->_header._packet_length,
		ppkt->_header._function,
		ppkt->_header._response
		);
	loop	= ppkt->_header._packet_length/sizeof(*p);
	p	= (u16*)(&ppkt->_message[0]);
	if (loop == 0) {
		printk(KERN_INFO "%04x ",*p);
	}
	else while (loop-- > 0) {
		printk(KERN_INFO "%04x ",*p++);
	}
	printk(KERN_INFO "\n");
}

#define ZMQ_ISSUE_CMD(pinstance, keyval, cmdstruct, sendok, cmd, resp, dolog) \
do {									\
	zmq_packet_t		packet;					\
	int			stlen;					\
	packet._header._val		= keyval;			\
	packet._header._function	= cmd;				\
	packet._header._response	= resp;				\
	packet._header._packet_length	= sizeof(cmdstruct);		\
	packet._header._reserved	= 0;				\
	memcpy(&packet._message[0], &(cmdstruct), sizeof(cmdstruct));	\
	stlen = zmq_message_send(pinstance, &packet._header,		\
		&packet._message[0], packet._header._packet_length,	\
		ZMQ_MSG_SEND_DEFAULT_RETRY, dolog);			\
	sendok = (stlen == sizeof(cmdstruct));				\
} while (0)

static int gs_zmq_memaf_cnt = 0;
static DEFINE_SPINLOCK(gs_zmq_memaf_cnt_lock);
static inline void * zmq_alloc(int sz)
{
	unsigned long flags;
	spin_lock_irqsave(&gs_zmq_memaf_cnt_lock, flags);
	gs_zmq_memaf_cnt++;
	spin_unlock_irqrestore(&gs_zmq_memaf_cnt_lock, flags);
	return kzalloc(sz, GFP_KERNEL | GFP_ATOMIC);
}
static inline void   zmq_free(void* p)
{
	unsigned long flags;
	spin_lock_irqsave(&gs_zmq_memaf_cnt_lock, flags);
	gs_zmq_memaf_cnt--;
	spin_unlock_irqrestore(&gs_zmq_memaf_cnt_lock, flags);
	kfree(p);
}

/******************************************************************
 * qcache
 * 	a list serve as queue
 *	alias to msg_cache to buffer input message for each session
 *	alias to bind_list to pair session request
 *
 *	allocator now use slab allocator, can be substitute by a local
 *	fast allocator
 *****************************************************************/
typedef struct {
	struct list_head	_head;
	spinlock_t		_lock;
} zmq_qcache_t;
static inline struct list_head * zmq_qcache_allocate_msg_node(void)
{
	struct list_head * ps;
	ps = (struct list_head *)zmq_pool_alloc(ZMQ_MSG_SLOT_SIZE +
			sizeof(struct list_head));
	if (ps != NULL) INIT_LIST_HEAD(ps);
	return ps;
}

static inline struct list_head * zmq_qcache_allocate_node(int len)
{
	struct list_head * ps;
	ps = (struct list_head *)zmq_alloc(len + sizeof(struct list_head));
	if (ps != NULL) INIT_LIST_HEAD(ps);

	zmq_dt(ZMQ_DT_MEM, ("%d: node allocate @%p, size %d\n", __LINE__
		, ps, len));
	return ps;
}
typedef void (*zmq_pfn_free_node)(void*p);
static inline void zmq_qcache_free_node(void * p)
{
	zmq_dt(ZMQ_DT_MEM, ("%d: node free @%p\n", __LINE__, p));
	zmq_free(p);
}
static inline void zmq_qcache_free_msg_node(void *p)
{
	zmq_pool_free(p, ZMQ_MSG_SLOT_SIZE + sizeof(struct list_head));
}

static inline void * zmq_qcache_node_payload(struct list_head * p)
{
	return p+1;
}
static inline void zmq_qcache_init(zmq_qcache_t * pqc)
{
	INIT_LIST_HEAD(&pqc->_head);
	spin_lock_init(&pqc->_lock);
}
static inline void zmq_qcache_deinit(zmq_qcache_t * pqc,
	zmq_pfn_free_node freepfn)
{
	unsigned long flags;
	struct list_head * pc;

	spin_lock_irqsave(&pqc->_lock, flags);

	while ( !list_empty(&pqc->_head) ) {
		pc = pqc->_head.next;
		__list_del(&pqc->_head, pc->next);
		freepfn(pc);
	}
	spin_unlock_irqrestore(&pqc->_lock, flags);
}
static inline void zmq_qcache_enqueue(zmq_qcache_t * pqc,
	struct list_head * pitem)
{
	unsigned long flags;
	spin_lock_irqsave(&pqc->_lock, flags);
	list_add_tail(pitem, &pqc->_head);
	spin_unlock_irqrestore(&pqc->_lock, flags);
}
static inline unsigned long zmq_qcache_dequeue_start(zmq_qcache_t * pqc,
	struct list_head **pp)
{
	unsigned long token;
	spin_lock_irqsave(&pqc->_lock, token);
	*pp = pqc->_head.next;
	return token;
}
static inline struct list_head * zmq_qcache_dequeue_end(zmq_qcache_t * pqc,
	bool remove, unsigned long token)
{
	struct list_head * pc;

	pc = NULL;
	if (remove && (!list_empty(&pqc->_head))) {
		pc = pqc->_head.next;
		__list_del(&pqc->_head, pc->next);
	}
	spin_unlock_irqrestore(&pqc->_lock, token);
	return pc;
}
static inline struct list_head * zmq_qcache_dequeue(zmq_qcache_t * pqc)
{
	unsigned int token;
	struct list_head * ps;

	token = zmq_qcache_dequeue_start(pqc, &ps);
	return zmq_qcache_dequeue_end(pqc, true, token);
}

typedef bool (*zmq_pfn_same)(void*, void*);
static bool zmq_payload_matched(void* p1, void* p2)
{
	return p1 == p2;
}
static inline struct list_head * zmq_qcache_find(zmq_qcache_t * pqc,
	zmq_pfn_same is_same, void * key, bool remove)
{
	unsigned long flags;
	struct list_head * ps;
	struct list_head * pc;
	ps = pc = NULL;
	spin_lock_irqsave(&pqc->_lock, flags);
	if (!list_empty(&pqc->_head)) {
		ps = pqc->_head.next;
		while (ps != &pqc->_head) {
			if (is_same(key, zmq_qcache_node_payload(ps))) {
				pc = ps;
				if (remove) {
					__list_del(pc->prev, pc->next);
				}
				break;
			}
			ps = ps->next;
		}
	}
	spin_unlock_irqrestore(&pqc->_lock, flags);

	return pc;
}

typedef struct list_head zmq_msg_cache_node_t;
typedef zmq_qcache_t zmq_msg_cache_t;
#define zmq_msg_cache_allocate_node	zmq_qcache_allocate_msg_node
#define zmq_msg_cache_free_node		zmq_qcache_free_msg_node
#define zmq_msg_cache_node_payload	zmq_qcache_node_payload
#define zmq_msg_cache_enqueue		zmq_qcache_enqueue
#define zmq_msg_cache_dequeue_start	zmq_qcache_dequeue_start
#define zmq_msg_cache_dequeue_end	zmq_qcache_dequeue_end
#define zmq_msg_cache_dequeue		zmq_qcache_dequeue
#define zmq_msg_cache_init		zmq_qcache_init
#define zmq_msg_cache_deinit(hdl)	zmq_qcache_deinit(hdl, zmq_msg_cache_free_node)
#define zmq_msg_cache_find		zmq_qcache_find

typedef struct list_head zmq_bind_node_t;
typedef zmq_qcache_t zmq_bind_list_t;
#define zmq_bl_allocate_node		zmq_qcache_allocate_node
#define zmq_bl_free_node		zmq_qcache_free_node
#define zmq_bl_node_payload		zmq_qcache_node_payload
#define zmq_bl_enqueue			zmq_qcache_enqueue
#define zmq_bl_dequeue_start		zmq_qcache_dequeue_start
#define zmq_bl_dequeue_end		zmq_qcache_dequeue_end
#define zmq_bl_dequeue			zmq_qcache_dequeue
#define zmq_bl_init			zmq_qcache_init
#define zmq_bl_deinit(hdl)		zmq_qcache_deinit(hdl, zmq_bl_free_node)
#define zmq_bl_find			zmq_qcache_find


/******************************************************************
 * deliver_queue is the type of session object itself
 * instance is type of the main function
 *****************************************************************/
#define ZMQ_SET_BITS(x, bits) ((x) |= (bits))
#define ZMQ_CLR_BITS(x, bits) ((x) &= ~(bits))
#define ZMQ_IS_BIT_SET(x, bits) (((x)&(bits)) == bits)
enum {
	ZMQ_TEST_MODE_LOOPBACK_BIT	= 0x00000001,
	ZMQ_TEST_MODE_LOGGER_BIT	= 0x00000002,
};
struct zmq_instance_t_;
typedef struct zmq_deliver_queue_t_ {
	zmq_packet_header_t	_key;
	struct mutex		_readlock; 	/* to prevent multiple read */
	struct zmq_instance_t_ *_pinstance;

	u32			_peek_mask;
	u32			_peek_window;	/* slot index of the peek area
						usually when data flow protocol
						is on for the session, this slot
						presents current data position
						*/

	struct completion	_msg_arrival;	/* sync generic messages */
	zmq_msg_cache_t		_queue;		/* queue to hold received messages */

	bool			_data_flow_active;
	struct completion	_data_flow;	/* sync data flow to user space */
	void *			_data_flow_ctx;
	zmq_pfn_data_flow_cb	_data_flow_cb;	/* call back for kernel space */
	spinlock_t		_data_flow_lock;/* to prevent multiple read */
	u32			_data_flow_mqin;/* max number of data flow
						request ZSP can queue */
	u32			_data_flow_qic;	/* number of data flow
						request ZSP queued */

	struct zmq_deliver_queue_t_ * _pmaster;

	ZMQ_DEFOBJDT;
} zmq_deliver_queue_t;
typedef struct zmq_instance_t_ {
	spinlock_t		_genlock;
	spinlock_t		_sendlock; /* this lock is global */
	u32			_test_mode;
	volatile bool		_logger_exit;
	struct task_struct *	_logger;
	struct completion	_logger_wake;
	zmq_msg_cache_t		_logger_cache;

	volatile bool		_mdma_exit;
	struct task_struct *	_mdma;
	struct completion	_mdma_wake;
	zmq_msg_cache_t		_mdma_cache;

	zmq_bind_list_t		_bind_list;
	zmq_shm_t *		_pshm_map;
	atomic_t		_dobjcnt;
	/* simple for direct map */
#ifdef ZMQ_USING_DIRECT_DISPATCH_MAPPING
	zmq_deliver_queue_t *	_zmq_deliver_map[ZMQ_MSG_CLASS_NUMBER]
						[ZMQ_MSG_MODULE_NUMBER]
						[ZMQ_MSG_SESSION_NUMBER];
#else
	zmq_deliver_queue_t **	_zmq_deliver_map[ZMQ_MSG_CLASS_NUMBER]
						[ZMQ_MSG_MODULE_NUMBER];
#endif
} zmq_instance_t;

#define ZMQ_PEEK_VALUE(pinstance, slot) \
	((pinstance)->_pshm_map->_peek_slot[slot])
#define ZMQ_SET_PKWV(pinstance, slot, val) \
	((pinstance)->_pshm_map->_peek_slot[slot] = val)

static struct workqueue_struct *	sg_recieve_working_queue = NULL;
static struct work_struct 		ag_recieve_wq_request;

static void zmq_firmware_acquire(zmq_instance_t * pinstance) {
	zsp_local_start(&pinstance->_dobjcnt, 50, 10);
}
static void zmq_firmware_release(zmq_instance_t * pinstance) {
	zsp_local_stop(&pinstance->_dobjcnt);
}
/*
 * when a general message arrived, triggered by intr, this will be called to
 * queue the message to the session, and notify the reader to fetch
 */
static int zmq_deliver_queue_put_message(zmq_deliver_queue_t * pdeliver,
	zmq_packet_t * pp, u32 slotidx)
{
	int len;
	zmq_msg_cache_node_t * ps;

	len = sizeof(pp->_header) + pp->_header._packet_length;
	ps = zmq_msg_cache_allocate_node();

	if (ps == NULL) {
		printk(KERN_ERR "%s: cannot allocate deliver queue\n", __func__);
		return 0;
	}

	memcpy(zmq_msg_cache_node_payload(ps), pp, len);
	zmq_msg_cache_enqueue(&pdeliver->_queue, ps);

	zmq_dtobj(pdeliver, ZMQ_DT_RECV,
		("%d: message queued on node@%p, length %d, on slot[%d]\n"
			, __LINE__, ps, len, slotidx));

	return len;
}

/*
 * client will call to this function to dequeue a message, if empty it wait here
 */
static int zmq_deliver_queue_get_message(zmq_deliver_queue_t * pdeliver,
	void* pmsg, u32 len)
{
	zmq_msg_cache_node_t * ps;
	zmq_packet_t * pp;
	int result;
	bool remove;
	unsigned long token;
	bool do_complete;

	do_complete = false;
	zmq_dtobj(pdeliver, ZMQ_DT_RECV,
			("%d: wait message arrival\n", __LINE__));

	while (1) {
		result = wait_for_completion_interruptible_timeout(
				&(pdeliver->_msg_arrival),
				msecs_to_jiffies(60*1000));
		if(result > 0) {
			/* real signal */
			break;
		}
		else if (result == 0) {
			/* timeout restart waiting */
			continue;
		}
		else {
			/* error */
			printk(KERN_ERR "%s: %d: unexpected termination\n",
				__func__,__LINE__);
			return -EINTR;
		}
	}

	result	= 0;
	remove	= false;
	token	= zmq_msg_cache_dequeue_start(&pdeliver->_queue, &ps);

	if (ps != NULL) {
		pp = (zmq_packet_t *) zmq_msg_cache_node_payload(ps);

/*
		printk("%s: %d: node@%p, len:%d/%d\n",
				__func__,__LINE__, ps,
				len, pp->_header._packet_length);
*/
		if ((pmsg != NULL) && (len >= pp->_header._packet_length)) {
			result	= pp->_header._packet_length;
			remove	= true;
			memcpy(pmsg, &(pp->_message[0]) ,result);
		}
		else {
			printk(KERN_WARNING "%s: %d: invalid user buffer\n",
				__func__,__LINE__);
			do_complete = true;
		}
	}

	ps = zmq_msg_cache_dequeue_end(&pdeliver->_queue, remove, token);

	if (do_complete) complete(&pdeliver->_msg_arrival); /* notify again */

	zmq_dtobj(pdeliver, ZMQ_DT_RECV,
		("%d: received message copied from node@%p, length %d\n",
			__LINE__, ps, len));

	if (remove && (ps != NULL)) {
		zmq_msg_cache_free_node(ps);
	}

	return result;
}

/*
 * query functions for session objects via message packet header or file
 */
static inline zmq_deliver_queue_t * zmq_find_deliver_queue_by_packet(
		zmq_instance_t * pinstance,
		zmq_packet_header_t * ph)
{
#ifdef ZMQ_USING_DIRECT_DISPATCH_MAPPING
	return pinstance->_zmq_deliver_map[ph->_class_id]
					[ph->_module_id]
					[ph->_session];
#else
	zmq_deliver_queue_t ** ppm;
	ppm = pinstance->_zmq_deliver_map[ph->_class_id][ph->_module_id];
	return (ppm == NULL)? NULL: ppm[ph->_session];
#endif
}
static inline bool zmq_put_deliver_queue_by_packet(
		zmq_instance_t * pinstance,
		zmq_packet_header_t * ph,
		zmq_deliver_queue_t * pq)
{
/*
	printk("%s: %d-%d-%d\n", __func__, ph->_class_id,
			ph->_module_id, ph->_session);
*/
#ifdef ZMQ_USING_DIRECT_DISPATCH_MAPPING
	pinstance->_zmq_deliver_map[ph->_class_id]
				[ph->_module_id]
				[ph->_session] = pq;
#else
	zmq_deliver_queue_t ** ppm;
	ppm = pinstance->_zmq_deliver_map[ph->_class_id][ph->_module_id];

	if (ppm == NULL) {
		if (pq != NULL) {
			ppm = (zmq_deliver_queue_t **)zmq_alloc(
					sizeof(*ppm)*ZMQ_MSG_SESSION_NUMBER);
			if (ppm == NULL) {
				printk(KERN_ERR "%s: outof memory\n", __func__);
				return false;
			}
			ppm[ph->_session] = pq;
			pinstance->_zmq_deliver_map[ph->_class_id]
						[ph->_module_id] = ppm;
		}
	}
	else {
		ppm[ph->_session] = pq;
	}
#endif
	return true;
}
static inline zmq_deliver_queue_t * zmq_find_deliver_queue_by_filp(
		struct file *filp)
{
	return (zmq_deliver_queue_t *)filp->private_data;
}

/*
 * register/remove a new session object to deliver map
 */
static bool zmq_add_deliver_queue(
		zmq_instance_t * pinstance,
		zmq_packet_header_t * ph, zmq_session_request_t * psr)
{
	unsigned long flags;
	bool hooked;
	zmq_deliver_queue_t * pq;

	/* prepare the deliver queue anyway */
	pq = (zmq_deliver_queue_t *)zmq_alloc(sizeof(*pq));
	if (pq == NULL) {
		printk(KERN_ERR "%s: cannot allocate deliver queue\n", __func__);
		return false;
	}
	pq->_pinstance		= pinstance;
	pq->_key._val		= ph->_val;
	pq->_peek_window	= psr->_peek_window;
	pq->_peek_mask		= psr->_peek_mask;
	pq->_data_flow_mqin	= psr->_max_queue_items;
	mutex_init(&pq->_readlock);
	init_completion(&pq->_msg_arrival);

	spin_lock_init(&pq->_data_flow_lock);
	init_completion(&pq->_data_flow);
	pq->_data_flow_active	= false;
	pq->_data_flow_ctx	= NULL;
	pq->_data_flow_cb	= NULL;
	pq->_data_flow_qic	= 0;

	pq->_pmaster		= NULL;

	zmq_dtena(pq); /* by default disable trace on objects */

	zmq_msg_cache_init(&pq->_queue);

	/* now update the queue mapping, need protection here */
	hooked = false;
	spin_lock_irqsave(&pinstance->_genlock, flags);
	/* check if the target queue existed */
	if ( zmq_find_deliver_queue_by_packet(pinstance, ph) == NULL ) {
		/* good to go */
		zmq_put_deliver_queue_by_packet(pinstance, ph, pq);
		hooked = true;
	}
	spin_unlock_irqrestore(&pinstance->_genlock, flags);

	if ( !hooked ) {
		zmq_msg_cache_deinit(&pq->_queue);
		zmq_free(pq);
		printk(KERN_ERR "%s:%d: deliver queue for [%d][%d][%d] exists\n"
				, __func__, __LINE__,ph->_class_id
				, ph->_module_id, ph->_session);
		return false;
	}

	zmq_dt(ZMQ_DT_SESSION, ("%d: session@%p registered\n",__LINE__, pq));

	return true;
}
static void zmq_remove_deliver_queue(
		zmq_instance_t * pinstance,
		zmq_packet_header_t * ph)
{
	unsigned long flags;
	zmq_deliver_queue_t * pq;

	spin_lock_irqsave(&pinstance->_genlock, flags);
	pq = zmq_find_deliver_queue_by_packet(pinstance, ph);
	if ( pq != NULL ) {
		zmq_put_deliver_queue_by_packet(pinstance, ph, NULL);
		zmq_msg_cache_deinit(&pq->_queue);
		mutex_destroy(&pq->_readlock);
		zmq_dt(ZMQ_DT_SESSION, ("%d: session@%p unregistered\n",
			__LINE__, pq));
		zmq_free(pq);
	}
	spin_unlock_irqrestore(&pinstance->_genlock, flags);
}
static void zmq_clear_deliver_queues(zmq_instance_t * pinstance)
{
	int cid, mid, sid;
	for (cid = 0; cid < ZMQ_MSG_CLASS_NUMBER; cid++) {
		for (mid = 0; mid < ZMQ_MSG_MODULE_NUMBER; mid++) {
#ifdef ZMQ_USING_DIRECT_DISPATCH_MAPPING
			for (sid = 0; sid < ZMQ_MSG_SESSION_NUMBER; sid++) {
				if (pinstance->_zmq_deliver_map[cid][mid][sid]
						!= NULL) {
					zmq_remove_deliver_queue(pinstance,
						&(pinstance->_zmq_deliver_map
							cid][mid][sid]->_key));
				}
			}
#else
			zmq_deliver_queue_t ** ppm;
			ppm = pinstance->_zmq_deliver_map[cid][mid];
			if (ppm == NULL) continue;

			for (sid = 0; sid < ZMQ_MSG_SESSION_NUMBER; sid++) {
				if (ppm[sid] != NULL) {
					zmq_remove_deliver_queue(pinstance,
						&(ppm[sid]->_key));
				}
			}
			zmq_free(ppm);
			pinstance->_zmq_deliver_map[cid][mid] = NULL;
#endif
		}
	}
}

/*
 * send message implement, all clients send will reach this function in
 * a direct call path. This function is implemented to be thread safe
 */
static int zmq_message_send(
		zmq_instance_t * pinstance,
		zmq_packet_header_t * ppackethdr,
		void* pmsg, u32 len, u32 try_num, bool dolog)
{
	unsigned long flags;
	bool done;
	u32 try_total;
	zmq_shm_t * pshm;
	u32 tx_ptr_nxt = 0;
	u32 rx_ptr = 0;
	u32 tx_ptr = 0;
	zmq_packet_t * ppkt;

	pshm = pinstance->_pshm_map;

	/* input parameter checking */
	if ((pmsg == NULL) || (len > ZMQ_MSG_LEN_MAX)) {
		return -1;
	}

	if (unlikely(pinstance->_dobjcnt.counter <= 0)) {
		printk(KERN_ERR "%s:%d: Accessing while ZSP off\n",
			__func__,__LINE__);
		return -1;
	}

	done = false;
	try_total = try_num;

	while (try_num > 0){

		spin_lock_irqsave(&pinstance->_sendlock, flags);

		tx_ptr = pshm->_key_section._p_tx_ptr;
		rx_ptr = pshm->_key_section._z_rx_ptr;

		tx_ptr_nxt = ZMQ_NEXT_TX_PTR(tx_ptr);
		/* pj4 send and zsp receive */
		/* check if the queue is full */
		if (likely(tx_ptr_nxt != rx_ptr)) {
			/* share mem is not full */
			ppkt = &(pshm->_p_tx_z_rx_queue[tx_ptr_nxt]);
			ppkt->_header._val = ppackethdr->_val;
			if (ppackethdr->_function == 0) {
				if (ZMQ_IS_BIT_SET(pinstance->_test_mode,
					ZMQ_TEST_MODE_LOOPBACK_BIT)) {
					ppkt->_header._function =
						ZMQ_CMD_LOOPBACK;
				}
			}
			ppkt->_header._packet_length = len;
			memcpy(&(ppkt->_message[0]), pmsg, len);
			/* clear the rest */
			if ( len < ZMQ_MSG_LEN_MAX) {
				memset(&(ppkt->_message[len]), 0,
					ZMQ_MSG_LEN_MAX - len);
			}

			/* update transfer pointer for message sending */
			pshm->_key_section._p_tx_ptr = tx_ptr_nxt;

			if (zmq_filter(ZMQ_DT_MSGDUMP))
				zmq_dump_packet(ppkt,true);
			/* notify the PJ4 by IPC */
			/* add memory barrier here to ensure the message
			 * content is ready for zsp reading already once ZSP
			 * is notifyed by IPC
			 */
			mb();
			pzipc_set_interrupt(IPC_MSG_TRANSFER_ID);

			done = true;
		}

		spin_unlock_irqrestore(&pinstance->_sendlock, flags);

		if (done) {
			zmq_dtcond(dolog, ZMQ_DT_SEND,
				("%d: send message on slot[%d], len = %d,"
				"  ok on No.%d time\n",
					__LINE__, tx_ptr_nxt, len,
					try_total-try_num + 1));
			/* no dump here, zsp may change the content */

			return len;
		}

		/* no blocking for sending, delay some times */
		mdelay(50);
		try_num--;
	}

	return -1;
}

/*
 * receive message implement, all clients send will reach this function in
 * a direct call path. This function is implemented to be thread safe
 */
static inline int zmq_message_recv(
		zmq_instance_t * pinstance,
		zmq_deliver_queue_t * pdeliver,
		void* pmsg, u32 len)
{
	int ret;
	/*
	since each session has its own session object we only need to
	ensure that no multiple threads are listening on the queue at
	the same time
	*/

	ret = 0;
	if (mutex_trylock(&pdeliver->_readlock) == 0) {
		printk(KERN_WARNING "%s: cannot listen on the same queue simultaneously\n",
			__func__);
		return -EPERM;
	}
	/* make sure that after delegation, read won't read any thing */
	if ((pdeliver->_pmaster == NULL) || (pdeliver->_pmaster == pdeliver)) {
		ret = zmq_deliver_queue_get_message(pdeliver, pmsg, len);
	} else {
		ret = -EPERM;
	}
	mutex_unlock(&pdeliver->_readlock);

	return ret;
}
/* logger implementation */
typedef struct {
	u32 _edomain;
	u32 _eline;
	u32 _ests;
	u32 _ddomain;
	u32 _dline;
	u32 _dvalue[5];
} zmq_log_entry_t;
static int zmq_logger_thread(void * pctx) {

	zmq_log_entry_t *	pentry;
	zmq_msg_cache_node_t *	ps;
	zmq_instance_t *	pinstance;
	unsigned long		tr;


	pinstance = (zmq_instance_t *)pctx;
	printk(KERN_INFO "%s:%d: ZMQ: logger started\n", __func__, __LINE__);
	while (1) {
		tr = wait_for_completion_timeout(&(pinstance->_logger_wake),
						msecs_to_jiffies(60*1000));
		while (NULL != (ps = zmq_msg_cache_dequeue(
					&pinstance->_logger_cache))) {
			pentry = (zmq_log_entry_t*)zmq_msg_cache_node_payload(ps);
			if (pentry->_edomain != 0)
			{
				printk(KERN_INFO "[ZSPLOG:%u]: err[dm:0x%x|ln:%04d|st_hex:%x]\n",
					jiffies_to_msecs(jiffies),
					pentry->_edomain,
					pentry->_eline,
					pentry->_ests);
			}
			if (pentry->_ddomain != 0)
			{
				printk(KERN_INFO "[ZSPLOG:%u]: dbg[dm:0x%x|ln:%04d|va_hex:%x-%x-%x-%x-%x]\n",
					jiffies_to_msecs(jiffies),
					pentry->_ddomain, pentry->_dline,
					pentry->_dvalue[0],pentry->_dvalue[1],
					pentry->_dvalue[2],
					pentry->_dvalue[3],pentry->_dvalue[4]);
			}
			zmq_msg_cache_free_node(ps);
		}
		if (pinstance->_logger_exit) break;
	}
	printk(KERN_INFO "%s:%d: ZMQ: logger stopped\n", __func__, __LINE__);
	return 0;
}
static void zmq_logger_put_message(zmq_instance_t * pinstance,
	zmq_packet_t * pp)
{
	zmq_msg_cache_node_t * ps;
	int len, pklen;

	len	= sizeof(zmq_log_entry_t);
	pklen	= pp->_header._packet_length;
	ps = zmq_msg_cache_allocate_node();
	if (ps == NULL) {
		printk(KERN_ERR "%s: cannot allocate logger buffer\n", __func__);
		return;
	}
	if (pklen == 0) {
		memcpy(zmq_msg_cache_node_payload(ps),
				&pinstance->_pshm_map->_padding[0], len);
	}
	else if (pklen >= len) {
		memcpy(zmq_msg_cache_node_payload(ps),
				&pp->_message[0], len);
	}
	else {
		memcpy(zmq_msg_cache_node_payload(ps),
				&pp->_message[0], pklen);
		memset((((char*)zmq_msg_cache_node_payload(ps)+pklen)), 0,
					len - pklen);
	}
	zmq_msg_cache_enqueue(&pinstance->_logger_cache, ps);
}
void zmq_processing_log(zmq_instance_t * pinstance, zmq_packet_t * ppkt)
{
	if (ZMQ_IS_BIT_SET(pinstance->_test_mode, ZMQ_TEST_MODE_LOGGER_BIT)) {
		if (pinstance->_logger != NULL) {
			zmq_logger_put_message(pinstance, ppkt);
			complete(&pinstance->_logger_wake);
		}
	}
}
static void zmq_logger_mod(zmq_instance_t * pinstance,
		u32 flags_to_clear, u32 flags_to_set)
{
	zmq_logctl_t logctl;
	bool sendok;
	logctl._flags_clear	= flags_to_clear;
	logctl._flags_set	= flags_to_set;
	ZMQ_ISSUE_CMD(pinstance, 0, logctl, sendok, ZMQ_CMD_LOGCTL, 0, true);
}
static void zmq_load_logger(zmq_instance_t * pinstance)
{
	if (pinstance->_logger == NULL) {
		zmq_firmware_acquire(pinstance);
		zmq_msg_cache_init(&pinstance->_logger_cache);
		init_completion(&pinstance->_logger_wake);
		pinstance->_logger_exit = false;
		pinstance->_logger = kthread_run(zmq_logger_thread,
					pinstance, "zmq_logger");

		/* now send logger command to enable it */
		zmq_logger_mod(pinstance, 0, ZMQ_LOGCTL_FLAGS_ENA);
	}
}
static void zmq_unload_logger(zmq_instance_t * pinstance)
{
	if (pinstance->_logger != NULL) {
		/* now send logger command to disable it */
		zmq_logger_mod(pinstance, ZMQ_LOGCTL_FLAGS_ENA, 0);

		pinstance->_logger_exit = true;
		complete(&pinstance->_logger_wake);
		kthread_stop(pinstance->_logger);
		pinstance->_logger = NULL;
		zmq_msg_cache_deinit(&pinstance->_logger_cache);
		zmq_firmware_release(pinstance);
	}
}

/* mdma proxy part */
#define ZMQ_MDMA_ALIGN(x) ((x)& (~0xf))
#define ZMQ_MDMA_ALIGNED(x) ((x) == ZMQ_MDMA_ALIGN(x))
static int zmq_mdma_proxy_thread(void * pctx) {

	zmq_msg_cache_node_t *	ps;
	zmq_packet_t *		pp;
	zmq_mdma_t *		pentry;
	zmq_instance_t *	pinstance;
	unsigned long		copied;
	int			retry;
	bool			sendok;

	pinstance = (zmq_instance_t *)pctx;
	printk(KERN_INFO "%s:%d: ZMQ: MDMA proxy started\n", __func__, __LINE__);
	while (1) {
		if (!wait_for_completion_timeout(&(pinstance->_mdma_wake),
						msecs_to_jiffies(60*1000)))
			continue;
		zmq_firmware_acquire(pinstance);
		while (NULL != (ps = zmq_msg_cache_dequeue(
					&pinstance->_mdma_cache))) {
			pp = (zmq_packet_t*)zmq_msg_cache_node_payload(ps);
			pentry = (zmq_mdma_t*)&pp->_message[0];
			copied = -1;

			/* fix the zsp TCM address */
			if (pentry->_dir == ZMQ_DATA_FLOW_DIR_P2Z) {
				pentry->_byteaddr_dst =
					ZSP_BADDR_TO_PA(pentry->_byteaddr_dst);
			} else  {
				pentry->_byteaddr_src =
					ZSP_BADDR_TO_PA(pentry->_byteaddr_src);
			}

			if (	ZMQ_MDMA_ALIGNED(pentry->_byteaddr_dst) &&
				ZMQ_MDMA_ALIGNED(pentry->_byteaddr_src) &&
				ZMQ_MDMA_ALIGNED(pentry->_byte_cnt)) {
/*
				zmq_dt(ZMQ_DT_MDMA, ("%d: MDMA copy 0x%08x bytes"
						" from 0x%08x=>0x%08x, ret=%d\n"
					,__LINE__, pentry->_byte_cnt,
					pentry->_byteaddr_src,
					pentry->_byteaddr_dst, (int)copied));
*/
				retry = 10;
				while (retry-- > 0) {
					copied = mdma_pmemcpy(pentry->_byteaddr_dst,
							pentry->_byteaddr_src,
							pentry->_byte_cnt);
					if (	(copied == -ERESTARTSYS) ||
						(copied == pentry->_byte_cnt)) {
						break;
					}
					msleep(10); /* wait for a while */
				}
			} else {
				printk(KERN_ERR "%s:%d: none-aligned MDMA request\n",
					__func__, __LINE__);
			}
			if (copied != pentry->_byte_cnt) { /* failed */
				pentry->_complete = 0;
				ZMQ_ISSUE_CMD(pinstance, pp->_header._val,
						(*pentry),
						sendok, ZMQ_CMD_MDMA_NOTIFY,
						ZMQ_CMD_RESPONSE_ERROR, true);
			} else if (pentry->_complete){ /* reqquires complete */
				ZMQ_ISSUE_CMD(pinstance, pp->_header._val,
						(*pentry),
						sendok, ZMQ_CMD_MDMA_NOTIFY,
						ZMQ_CMD_RESPONSE_OK, true);
			}
			zmq_dt(ZMQ_DT_MDMA,
				("%d: MDMA copy 0x%08x bytes from 0x%08x=>0x%08x, ret=%d\n"
				,__LINE__, pentry->_byte_cnt,
				pentry->_byteaddr_src,
				pentry->_byteaddr_dst, (int)copied));

			zmq_msg_cache_free_node(ps);
		}
		zmq_firmware_release(pinstance);
		if (pinstance->_mdma_exit) break;
	}
	printk(KERN_INFO "%s:%d: ZMQ: MDMA proxy stopped\n", __func__, __LINE__);
	return 0;
}
void zmq_processing_mdma_request(zmq_instance_t * pinstance, zmq_packet_t * pp)
{
	zmq_msg_cache_node_t * ps;
	int len;

	if (pinstance->_mdma == NULL) return;

	len = sizeof(pp->_header) + pp->_header._packet_length;
	ps = zmq_msg_cache_allocate_node();
	if (ps == NULL) {
		printk(KERN_ERR "%s: cannot allocate mdma request buffer\n",
			__func__);
		return;
	}
	memcpy(zmq_msg_cache_node_payload(ps), pp, len);
	zmq_msg_cache_enqueue(&pinstance->_mdma_cache, ps);
	complete(&pinstance->_mdma_wake);
}
static void zmq_load_mdma_proxy(zmq_instance_t * pinstance)
{
	if (pinstance->_mdma == NULL) {
		zmq_msg_cache_init(&pinstance->_mdma_cache);
		init_completion(&pinstance->_mdma_wake);
		pinstance->_mdma_exit = false;
		pinstance->_mdma = kthread_run(zmq_mdma_proxy_thread,
						pinstance, "zmq_mdma_proxy");
	}
}
static void zmq_unload_mdma_proxy(zmq_instance_t * pinstance)
{
	if (pinstance->_mdma != NULL) {
		pinstance->_mdma_exit = true;
		complete(&pinstance->_mdma_wake);
		kthread_stop(pinstance->_mdma);
		pinstance->_mdma = NULL;
		zmq_msg_cache_deinit(&pinstance->_mdma_cache);
	}
}
/* instance construction and destruction */
static bool zmq_init(zmq_instance_t ** ppinstance)
{
	zmq_instance_t * pinstance;

	if (ppinstance == NULL) {
		printk(KERN_ERR "%s: parameter error: wrong instance\n",
			__func__);
		return false;
	}
	/* allocate ZMQ instance */
	pinstance = (zmq_instance_t *)zmq_alloc(sizeof(*pinstance));
	if (pinstance == NULL) {
		printk(KERN_ERR "%s: cannot allocate ZMQ instance memory\n",
			__func__);
		return false;
	}
	/* instance ok */
	pinstance->_pshm_map = (zmq_shm_t *)zsp_get_datawnd();

	if (pinstance->_pshm_map == NULL ) {
		/* free instance */
		zmq_free(pinstance);
		printk(KERN_ERR "%s: cannot map ZMQ SHM\n", __func__);
		return false;
	}

	/* SHM will be initialized by ZSP firmware, don't touch here */

	zmq_bl_init(&pinstance->_bind_list);

	spin_lock_init(&pinstance->_genlock);
	spin_lock_init(&pinstance->_sendlock);

	pinstance->_test_mode = 0;
	pinstance->_logger = NULL;
	pinstance->_mdma = NULL;

	zmq_load_mdma_proxy(pinstance);

	memset(pinstance->_zmq_deliver_map, 0,
		sizeof(pinstance->_zmq_deliver_map));

	pinstance->_dobjcnt.counter = 0;

	*ppinstance = pinstance;

	printk(KERN_INFO "ZSP Message Queue Loaded\n");

	return true;
}
static void zmq_deinit(zmq_instance_t ** ppinstance)
{
	zmq_instance_t * pinstance;
	if (ppinstance == NULL) {
		printk(KERN_ERR "%s: parameter error: wrong instance\n",
			__func__);
		return;
	}
	pinstance = *ppinstance;
	if (pinstance != NULL) {
		zmq_unload_mdma_proxy(pinstance);
		zmq_unload_logger(pinstance);
		zmq_bl_deinit(&(pinstance->_bind_list));
		zmq_clear_deliver_queues(pinstance);
		zmq_free(pinstance);
	}
	*ppinstance = NULL;
	printk(KERN_INFO "ZSP Message Queue Unloaded\n");
}
static inline zmq_instance_t * zmq_get_instance_ex(bool deinit)
{
	static zmq_instance_t * sp_zmq_instance = NULL;
	if (unlikely(deinit))
	{
		if (sp_zmq_instance != NULL) {
			zmq_deinit(&sp_zmq_instance);
		}
		sp_zmq_instance = NULL;
	}
	else if (unlikely(sp_zmq_instance == NULL)) {
		if ( !zmq_init(&sp_zmq_instance) ) {
			sp_zmq_instance = NULL;
		}
	}
	return sp_zmq_instance;
}
#define zmq_get_instance() zmq_get_instance_ex(false)
#define zmq_deinit_instance() zmq_get_instance_ex(true)

/*****************************************************************
 * implementation of the session request protocol
 *****************************************************************
 * the packet header will include the target class_id, module_id
 * session_id is the max session number supported my side
 * when request processed, return message use the same structure
 * the response bit indicates if the request been processed OK.
 * the session_id is the allocated session_id from peer
 * the message content is zmq_session_request_t, which includes the
 * sequence number used to pair the request and response and the peek
 * window position.
 ****************************************************************/
typedef struct {
	zmq_session_request_t	_sr;
	zmq_packet_header_t	_hdr;
	zmq_instance_t * 	_pinstance;
	volatile bool		_changed;
	struct completion	_bine_done;
} zmq_bind_completion_t;
static bool zmq_request_session(zmq_instance_t * pinstance,
	u16 sn, zmq_packet_header_t * phdr)
{
	zmq_bind_node_t *	pbn;
	zmq_bind_completion_t * pbc;
	bool res, sendok;

	if (		(phdr->_class_id  >= ZMQ_MSG_CLASS_NUMBER)
		||	(phdr->_module_id >= ZMQ_MSG_MODULE_NUMBER)
	) {
		printk(KERN_ERR "%s: %d: invalid parameter, cid=%d, mid=%d\n",
				__func__,__LINE__,
				phdr->_class_id,
				phdr->_module_id);
		return false;
	}

	/* prepare completion node for paring the request by sn */
	pbn = zmq_bl_allocate_node(sizeof(*pbc));
	if (pbn == NULL) {
		printk(KERN_ERR "%s: cannot allocate bind completion object\n",
			__func__);
		return false;
	}

	res = true;
	pbc = (zmq_bind_completion_t*) zmq_bl_node_payload(pbn);
	pbc->_pinstance	= pinstance;
	memset(&pbc->_sr, 0, sizeof(pbc->_sr));
	pbc->_sr._sn		= sn;
	pbc->_hdr._val		= phdr->_val;
	pbc->_hdr._session	= ZMQ_MSG_SESSION_NUMBER;
	pbc->_hdr._response	= ZMQ_CMD_RESPONSE_OK;
	pbc->_changed		= false;
	init_completion(&pbc->_bine_done);

	zmq_bl_enqueue(&pinstance->_bind_list, pbn);

	zmq_dt(ZMQ_DT_SESSION, ("%d: registering session for class %d, module %d\n"
			,__LINE__, pbc->_hdr._class_id, pbc->_hdr._module_id));

	/* now issue session request command */
	ZMQ_ISSUE_CMD(pinstance, pbc->_hdr._val, pbc->_sr, sendok,
			ZMQ_CMD_SESSION_REQUEST, 0, true);

	if ( sendok ) {
		/* message send ok */
		int tt;
		tt = wait_for_completion_interruptible_timeout(&pbc->_bine_done,
						msecs_to_jiffies(2000));

		if (tt < 0) {
			res = false;
			goto clean_up;
		}
		/* now the header session part should have been updated */
		if ( !pbc->_changed ) {
			printk(KERN_ERR "%s: %d: abort, ZSP response timeout\n",
				__func__,__LINE__);
			res = false;
			goto clean_up;
		}

		if (pbc->_hdr._response != ZMQ_CMD_RESPONSE_OK) {
			printk(KERN_ERR "%s: %d: abort, reach ZSP limit\n",
				__func__,__LINE__);
			res = false;
			goto clean_up;
		}

		if (pbc->_hdr._session >= ZMQ_MSG_SESSION_NUMBER) {
			printk(KERN_ERR "%s: %d: abort, session number out of range, %d\n",
					__func__,__LINE__, pbc->_hdr._session);
			res = false;
			goto clean_up;
		}

		zmq_dt(ZMQ_DT_SESSION, ("%s: %d: bind\n",__func__,__LINE__));

		/* make sure all values good, clean up internal message part */
		pbc->_hdr._function		= 0;
		pbc->_hdr._response		= 0;
		pbc->_hdr._packet_length	= 0;
		pbc->_hdr._reserved		= 0;

		if ( !zmq_add_deliver_queue(pinstance, &pbc->_hdr, &pbc->_sr)) {
			printk(KERN_ERR "%s: %d: bind abort, cannot add session to map\n",
					__func__,__LINE__);
			res = false;
			goto clean_up;
		}

		phdr->_val = pbc->_hdr._val;

		zmq_dt(ZMQ_DT_SESSION,
			("%d: returned session id %d, pkwnd=%d, msk=0x%08x, queuelen=%d\n"
				,__LINE__, pbc->_hdr._session,
				pbc->_sr._peek_window, pbc->_sr._peek_mask,
				pbc->_sr._max_queue_items));
	}

clean_up:
	/* find and remove the completion node */
	zmq_bl_find(&pinstance->_bind_list, zmq_payload_matched, pbc, true);
	zmq_bl_free_node(pbn);
	return res;
}

static void zmq_end_session(zmq_deliver_queue_t * pdeliver)
{
	bool			sentok;
	zmq_session_end_t	se;
	se._peek_window			= pdeliver->_peek_window;

	ZMQ_ISSUE_CMD(pdeliver->_pinstance, pdeliver->_key._val, se, sentok,
			ZMQ_CMD_SESSION_END, 0, true);

	zmq_dt(ZMQ_DT_SESSION,
		("%d: ending session for class %d, module %d, session %d\n"
			,__LINE__, pdeliver->_key._class_id,
			pdeliver->_key._module_id, pdeliver->_key._session));

}

static bool zmq_request_session_matched(void* p1, void* p2)
{
	zmq_bind_completion_t * pbc1;
	zmq_bind_completion_t * pbc2;
	pbc1 = (zmq_bind_completion_t*)p1;
	pbc2 = (zmq_bind_completion_t*)p2;

	return pbc1->_sr._sn == pbc2->_sr._sn;
}
static void zmq_request_session_complete(zmq_instance_t * pinstance,
	zmq_packet_t * ppkt)
{
	zmq_bind_node_t *	pbn;
	zmq_bind_completion_t *	pbc;
	zmq_bind_completion_t	key;

	memcpy(&key._sr, &ppkt->_message[0], sizeof(key._sr));
	pbn = zmq_bl_find(&pinstance->_bind_list,
			zmq_request_session_matched, &key, false);
	if (pbn == NULL) {
		printk(KERN_ERR "%s: %d: cannot find completion object\n",
			__func__, __LINE__);
		return;
	}

	pbc = (zmq_bind_completion_t*) zmq_bl_node_payload(pbn);

	/* now we copy the results back to the completion node for
	next step processing
	*/
	pbc->_hdr._session	= ppkt->_header._session;
	pbc->_hdr._response	= ppkt->_header._response;
	memcpy(&pbc->_sr, &key._sr, sizeof(key._sr));

	pbc->_changed		= true;
	zmq_dt(ZMQ_DT_SESSION,
		("%d: session request response arrived\n",__LINE__));

	complete(&pbc->_bine_done);

	/* the object will be reclaimed in zmq_request_session */
}

/*****************************************************************
 * implementation of the data flow protocol
 *****************************************************************/

static inline bool zmq_register_data_flow(zmq_deliver_queue_t * pdeliver,
		zmq_pfn_data_flow_cb callback, void * pctx)
{
	unsigned long flags;
	bool ret;
	spin_lock_irqsave(&pdeliver->_data_flow_lock, flags);
	if ( !pdeliver->_data_flow_active) {
		pdeliver->_data_flow_active	= true;
		pdeliver->_data_flow_ctx	= pctx;
		pdeliver->_data_flow_cb		= callback; /* can be null,
							for user client */
		pdeliver->_data_flow_qic	= 0;
		ret = true;
	}
	else ret = false;
	spin_unlock_irqrestore(&pdeliver->_data_flow_lock, flags);

	zmq_dtobj(pdeliver, ZMQ_DT_DF, ("%d: data flow register %s\n"
			, __LINE__, ret?("ok"):("failed")));

	return ret;
}
static inline bool zmq_unregister_data_flow(zmq_deliver_queue_t * pdeliver)
{
	unsigned long flags;
	bool ret;
	spin_lock_irqsave(&pdeliver->_data_flow_lock, flags);
	if ( pdeliver->_data_flow_active) {
		pdeliver->_data_flow_active	= false;
		pdeliver->_data_flow_ctx	= NULL;
		pdeliver->_data_flow_cb		= NULL;
		pdeliver->_data_flow_qic	= 0;
		ret = true;
	}
	else ret = false;
	spin_unlock_irqrestore(&pdeliver->_data_flow_lock, flags);

	zmq_dtobj(pdeliver, ZMQ_DT_DF, ("%d: data flow unregister %s\n"
			, __LINE__, ret?("ok"):("failed")));
	return ret;
}
static bool zmq_data_flow_enqueue(zmq_deliver_queue_t * pdeliver,
		u32 addr, u32 len, u32 low, u32 evts, u32 dir)
{
	unsigned long	flags;
	bool 		stok;
	zmq_data_flow_t	df;
	bool		do_enqueue;

	do_enqueue	= false;
	stok		= false;
	df._dir		= 0;
	spin_lock_irqsave(&pdeliver->_data_flow_lock, flags);
	if (pdeliver->_data_flow_active
		&& (pdeliver->_data_flow_qic < pdeliver->_data_flow_mqin)) {
		pdeliver->_data_flow_qic++;
		do_enqueue = true;
	}
	spin_unlock_irqrestore(&pdeliver->_data_flow_lock, flags);

	if (do_enqueue) {
		df._addr = addr; df._len = len; df._low = low;
		df._evts = evts; df._qclr = 0;
		df._dir = ((dir == ZMQ_DATA_FLOW_DIR_P2Z)?
				ZMQ_DATA_FLOW_DIR_P2Z:
				ZMQ_DATA_FLOW_DIR_Z2P);
		ZMQ_ISSUE_CMD(pdeliver->_pinstance,
				pdeliver->_key._val, df, stok,
				ZMQ_CMD_DATA_FLOW, 0, zmq_objflag(pdeliver));

	}
	zmq_dtobj(pdeliver, ZMQ_DT_DF,
		("%d: data flow descriptor (0x%08x/%d/%d/0x%04x/%d) enqueue %s\n"
			, __LINE__, addr, len, low, evts, df._dir,
			(do_enqueue && stok)?("ok"):("failed")));
	return do_enqueue && stok;
}
static void zmq_data_flow_report(zmq_instance_t * pinstance, zmq_packet_t * ppkt)
{
	unsigned long flags1, flags2;
	zmq_deliver_queue_t * 	pdeliver;
	zmq_data_flow_t		df;
	bool			do_complete;

	do_complete = false;

	spin_lock_irqsave(&pinstance->_genlock, flags1);
	/* ensure no adding/remove deliver queue in progress at the same point */
	pdeliver = zmq_find_deliver_queue_by_packet(pinstance, &ppkt->_header);
	if ( pdeliver != NULL )
	{
		spin_lock_irqsave(&pdeliver->_data_flow_lock, flags2);
		if (pdeliver->_data_flow_active) {
			memcpy(&df, &ppkt->_message[0], sizeof(df));
			if (df._qclr <= pdeliver->_data_flow_qic) {
				pdeliver->_data_flow_qic -= df._qclr;
			}
			else {
				printk(KERN_WARNING "%s: %d: data flow desc"
					" clear number too big %d, should <= %d\n"
						,__func__,__LINE__,df._qclr,
						pdeliver->_data_flow_qic);
				/* clear anyway */
				pdeliver->_data_flow_qic = 0;
			}
			if (pdeliver->_data_flow_cb != NULL) {
				pdeliver->_data_flow_cb(&df,
						pdeliver->_data_flow_ctx);
			}
			else {
				do_complete = true;
			}

			zmq_dtobj(pdeliver, ZMQ_DT_DF,
				("%d: data flow notification (0x%08x/%d/%d/0x%04x/%d)\n"
					, __LINE__, df._addr, df._len,
					df._low, df._evts, df._qclr));
		}
		else {
			printk(KERN_ERR "%s: %d: no data flow bind\n",
				__func__, __LINE__);
		}
		spin_unlock_irqrestore(&pdeliver->_data_flow_lock, flags2);
	}
	else {
		printk(KERN_ERR "%s: %d: data flow notification, no deliver object\n",
				__func__,__LINE__);
	}
	spin_unlock_irqrestore(&pinstance->_genlock, flags1);

	if (do_complete) complete(&(pdeliver->_data_flow));

}
static inline u32 zmq_peek_at_the_window(zmq_deliver_queue_t * pdeliver)
{
	if (pdeliver == NULL) return 0;
	if (pdeliver->_peek_window >= ZMQ_SHM_PEEK_SLOT_NUMBER) return 0;
	return pdeliver->_pinstance->_pshm_map->_peek_slot[pdeliver->_peek_window];
}
static inline void zmq_set_peek_window(zmq_deliver_queue_t * pdeliver, u32 value)
{
	if (pdeliver == NULL) return;
	if (pdeliver->_peek_window >= ZMQ_SHM_PEEK_SLOT_NUMBER) return;
	pdeliver->_pinstance->_pshm_map->_peek_slot[pdeliver->_peek_window] = value;
}
static inline u32 zmq_data_flow_pos(zmq_deliver_queue_t * pdeliver)
{
	/*
	assume that the update has minimum block then
	we may use some of the low bits in this u32 value to mark other
	real-time status.
	*/
	return pdeliver->_peek_mask & zmq_peek_at_the_window(pdeliver);
}
static inline bool zmq_data_flow_wait(zmq_deliver_queue_t * pdeliver, u32* pos)
{
	int result;
	while (1) {
		result = wait_for_completion_interruptible_timeout(
			&(pdeliver->_data_flow), msecs_to_jiffies(60*1000));
		if(result > 0) {
			/* real signal */
			break;
		}
		else if (result == 0) {
			/* timeout restart waiting */
			continue;
		}
		else {
			/* error */
			printk(KERN_ERR "%s: %d: unexpected termination\n",
				__func__, __LINE__);
			return false;
		}
	}

	* pos = zmq_data_flow_pos(pdeliver);
	return true;
}

/*
here is a simple data flow model (basic ring buffer):
1.	on user creation, we allocate a physical contiguous memory,
	and map to user; setup the information like capture/play-back,
	water marker etc.
2.	on user get_buffer, it pass the portion can use by a offset
	and size, user need to fill/fetch data to the portion;
	when no data valid, it triggers a wait; user need to handle
	loop back to start situation.
3.	when user consumed a buffer, it deliver_buffer here; includes
	the information of size of valid data; we will generated a
	descriptor and push to ZSP
4.	when ZSP consumes a buffer and reached water marker, it generates
	a notification and go on; when we received it, we will release
	the pending get_buffer call.
5.	if no no more deliver_buffer is called, and ZSP previous buffer
	all used, it automatically stops and generated a notification
	based on configuration item.
*/
struct zmq_data_flow_br_t_;
typedef void (* zmq_pfn_data_flow_br_cb)(zmq_data_flow_t *,
	struct zmq_data_flow_br_t_ *);
typedef struct zmq_data_flow_br_t_{
	u32			_upos;		/* offset */
	u32			_size;
	u32			_low;
	u32			_start;
	u32			_dir;
	u8 *			_vptr;
	zmq_deliver_queue_t * 	_pdeliver;
	zmq_pfn_data_flow_br_cb _callback;
} zmq_data_flow_br_t;
static void zmq_data_flow_br_cb(zmq_data_flow_t * pdf, void * pctx)
{
	zmq_data_flow_br_t * pdfbr;
	pdfbr = (zmq_data_flow_br_t*)pctx;
	if (pdfbr == NULL) return;
	if (pdfbr->_callback != NULL) {
		pdfbr->_callback(pdf, pdfbr);
	}
}
static void * zmq_br_create(zmq_deliver_queue_t * pdeliver,
		u32 start, u32 size, u32 low, u32 dir, u8 * vptr,
		zmq_pfn_data_flow_br_cb callback)
{
	zmq_data_flow_br_t * pbr;

	if (pdeliver == NULL) {
		printk(KERN_ERR "%s: %d: invalid session\n",__func__,__LINE__);
		return NULL;
	}

	pbr = (zmq_data_flow_br_t *)zmq_alloc(sizeof(*pbr));
	if (pbr == NULL) {
		printk(KERN_ERR "%s: %d: cannot get buffer ring object\n",
			__func__,__LINE__);
		return NULL;
	}
	pbr->_pdeliver	= pdeliver;
	pbr->_start	= start;
	pbr->_dir	= (dir == ZMQ_DATA_FLOW_DIR_P2Z)?
				ZMQ_DATA_FLOW_DIR_P2Z:
				ZMQ_DATA_FLOW_DIR_Z2P;
	pbr->_size	= size;
	pbr->_upos	= 0;
	pbr->_low	= low;
	pbr->_vptr	= vptr;
	pbr->_callback	= callback;

	if ( !zmq_register_data_flow(pdeliver, (callback == NULL)?
						NULL:
						zmq_data_flow_br_cb,
					pbr)) {
		zmq_free(pbr);
		pbr = NULL;
	}

	zmq_set_peek_window(pdeliver, start); /* init the peek value */

	zmq_dtobj(pdeliver, ZMQ_DT_DF_BR,
			("%d: buffer ring %p registered. (0x%08x/%d/%d/%p)\n"
			, __LINE__, pbr, start, size, low, vptr));
	return pbr;
}
static void zmq_br_destory(void * hbr)
{
	zmq_data_flow_br_t * pbr;
	pbr = (zmq_data_flow_br_t *) hbr;
	if (pbr == NULL) return;
	zmq_unregister_data_flow(pbr->_pdeliver);
	zmq_dtobj(pbr->_pdeliver, ZMQ_DT_DF_BR,
		("%d: buffer ring %p unregistered.\n", __LINE__, pbr));
	zmq_free(pbr);
}
static inline u32 zmq_br_zpos(void* hbr)
{
	zmq_data_flow_br_t * pbr;
	u32 pos;
	pbr = (zmq_data_flow_br_t *) hbr;
	if (pbr == NULL) return 0;
	pos = zmq_data_flow_pos(pbr->_pdeliver);
	if ( pos < pbr->_start) {
		printk(KERN_ERR "%s: %d: wrong ZSP pos 0x%08x\n",
			__func__,__LINE__,pos);
	}
	pos = (pos - pbr->_start); /* adjust to 0 based */

	zmq_dtobj(pbr->_pdeliver, ZMQ_DT_DF_BR, ("%d: buffer ring zpos %d.\n",
					__LINE__, pos));
	return pos;
}
static inline u32 zmq_br_delta(u32 upos, u32 zpos, u32 limit) {
	if (unlikely(upos > limit || zpos > limit)) {
		printk(KERN_ERR "%s: %d: invalid parameter\n",
			__func__,__LINE__);
	}
	/*
	since zpos update is based on transfer size count
	if we ensure that the issued size does not cause a over-run
	we can always assume that when two position is the same
	the delta part is the total buffer, here is "limit"
	only when data flow from ZSP to user, if we call get_buffer before
	actually started the transfer and using call back mode we need to
	drop the content.
	*/
	return (upos >= zpos)?(zpos + (limit-upos)):(zpos - upos);
}
static bool zmq_br_get_buffer(void* hbr, u32 * pupos, u32 * pvalidsize)
{
	zmq_data_flow_br_t * pbr;
	u32 zpos;

	pbr = (zmq_data_flow_br_t *) hbr;
	if (pbr == NULL) return false;

	zpos = 0;
	if (pbr->_callback == NULL) {
		if ( !zmq_data_flow_wait(pbr->_pdeliver, &zpos) ) {
			return false;
		}
	}
	zpos = zmq_br_zpos(pbr); /* pos is 0 based */

	/* return the size of usable buffer */
	*pupos		= pbr->_upos;
	*pvalidsize	= zmq_br_delta(pbr->_upos, zpos, pbr->_size);

	zmq_dtobj(pbr->_pdeliver, ZMQ_DT_DF_BR,
		("%d: buffer ring get_buffer (upos:%d/validsize:%d).\n",
		__LINE__, *pupos, *pvalidsize));
	return true;
}
static bool zmq_br_deliver_buffer(void* hbr, u32 size)
{
	zmq_data_flow_br_t * pbr;
	u32 zpos, upos, upos_new, limit, low;

	pbr = (zmq_data_flow_br_t *) hbr;
	if (pbr == NULL) return false;
	if (size > pbr->_size) return false;

	zpos	= zmq_br_zpos(pbr); /* pos is 0 based */
	upos	= pbr->_upos;
	limit	= pbr->_size;
	low	= pbr->_low;
	if (zmq_br_delta(upos, zpos, limit) < size) {
		/* should not be here */
		printk(KERN_ERR "%s: %d: software overflow?\n",
			__func__, __LINE__);
		return false;
	}

	zmq_dtobj(pbr->_pdeliver, ZMQ_DT_DF_BR,
		("%d: limit:%d, upos:%d, zpos:%d, size:%d, low:%d.\n",
		__LINE__, limit, upos, zpos, size, low));

	upos_new = upos + size;
	if (upos_new > limit) {
		/* roll back from start */
		/* a separation is required */
		upos_new -= limit;

		if (size < low) {
			/* smaller than low, end of data?
			end part, convert pos to ptr*/
			zmq_data_flow_enqueue(pbr->_pdeliver, pbr->_start+upos,
					limit - upos, 0, 0, pbr->_dir);
			/* beginning part, put an auto stop marker */
			zmq_data_flow_enqueue(pbr->_pdeliver, pbr->_start,
					upos_new, 0,
					ZMQ_DATA_FLOW_EVTS_AUTO_STOP,
					pbr->_dir);
		}
		else {
			/* total size can hit low */
			if (upos_new <= low) {
				/* low point in the end part */
				zmq_data_flow_enqueue(pbr->_pdeliver,
						pbr->_start + upos,
						limit - upos, low - upos_new,
						ZMQ_DATA_FLOW_EVTS_LOW,
						pbr->_dir);
				/* beginning part has no low setting */
				zmq_data_flow_enqueue(pbr->_pdeliver,
						pbr->_start,
						upos_new, 0, 0, pbr->_dir);
			}
			else {
				/* low point in the beginning part */
				zmq_data_flow_enqueue(pbr->_pdeliver,
						pbr->_start + upos,
						limit - upos, 0, 0, pbr->_dir);
				/* beginning part has no low setting */
				zmq_data_flow_enqueue(pbr->_pdeliver,
						pbr->_start, upos_new, low,
						ZMQ_DATA_FLOW_EVTS_LOW,
						pbr->_dir);
			}
		}
		zmq_dtobj(pbr->_pdeliver, ZMQ_DT_DF_BR,
			("%d: buffer ring deliver_buffer two parts (new_upos:%d/size:%d).\n",
				__LINE__, upos_new, size));
	}
	else {
		if (size < low) {
			/* smaller than low, end of data? */
			zmq_data_flow_enqueue(pbr->_pdeliver,
					pbr->_start + upos,
					size, 0,
					ZMQ_DATA_FLOW_EVTS_AUTO_STOP,
					pbr->_dir);
		}
		else {
			zmq_data_flow_enqueue(pbr->_pdeliver,
					pbr->_start + upos, size, low,
					ZMQ_DATA_FLOW_EVTS_LOW, pbr->_dir);
		}
		if (upos_new == limit) upos_new = 0; /* roll back */

		zmq_dtobj(pbr->_pdeliver, ZMQ_DT_DF_BR,
			("%d: buffer ring deliver_buffer single part (new_upos:%d/size:%d).\n",
				__LINE__, upos_new, size));
	}

	pbr->_upos = upos_new;

	return true;
}


/* internal message processing dispatcher */
static void zmq_processing_internal_messages(zmq_instance_t * pinstance,
		zmq_packet_t * ppkt)
{
	switch (ppkt->_header._function) {
	case ZMQ_REP_DATA_FLOW:
		zmq_data_flow_report(pinstance, ppkt);
		break;
	case ZMQ_REP_MDMA_REQ:
		zmq_processing_mdma_request(pinstance,ppkt);
		break;
	case ZMQ_REP_LOG:
		zmq_processing_log(pinstance, ppkt);
		break;
	case ZMQ_REP_SESSION_REQUEST:
		zmq_request_session_complete(pinstance, ppkt);
		break;
	}
}

/* implementation of the message receiver, it is the entry point of dispatcher */
static void zmq_message_arrival(zmq_instance_t * pinstance)
{
	unsigned long flags;
	zmq_shm_t * pshm;
	zmq_packet_header_t header;
	zmq_deliver_queue_t * pdeliver;
	u32 rx_ptr_nxt;
	u32 tx_ptr = 0;
	u32 rx_ptr = 0;
	bool do_complete;

	do_complete = false;

	/* hazard here, if all client disconnection and still have things
	to read there will be issues */
	if (pinstance->_dobjcnt.counter <= 0) {
		return;
	}

	pshm = pinstance->_pshm_map;
	/* pj4 receive and zsp send */
	tx_ptr = pshm->_key_section._z_tx_ptr;
	rx_ptr = pshm->_key_section._p_rx_ptr;

	zmq_dt(ZMQ_DT_INTR, ("%d: message arrival intr %d-%d\n",
			__LINE__, tx_ptr, rx_ptr));

	if (tx_ptr != rx_ptr){ /*new pakcet available*/
		while ( 1 ){
			rx_ptr_nxt = ZMQ_NEXT_RX_PTR(rx_ptr);
			header._val = pshm->_z_tx_p_rx_queue[rx_ptr_nxt]._header._val;

			if (zmq_filter(ZMQ_DT_MSGDUMP))
				zmq_dump_packet(&pshm->_z_tx_p_rx_queue[rx_ptr_nxt], false);

			if (header._function != 0) {
				/* internal messages */
				zmq_processing_internal_messages(pinstance,
					&pshm->_z_tx_p_rx_queue[rx_ptr_nxt]);
			}
			else {
				/* general message */
				spin_lock_irqsave(&pinstance->_genlock, flags);
				/* ensure no adding/remove deliver queue in
				progress at the same point */
				pdeliver = zmq_find_deliver_queue_by_packet(
						pinstance, &header);
				if ( pdeliver != NULL )
				{
					/* search for the final delegation */
					while (pdeliver->_pmaster != NULL) {
						if (pdeliver == pdeliver->_pmaster) {
							pdeliver->_pmaster = NULL;
							break;
						}
						pdeliver = pdeliver->_pmaster;
					}
					zmq_deliver_queue_put_message(pdeliver,
						&pshm->_z_tx_p_rx_queue[rx_ptr_nxt],
						rx_ptr_nxt);
					do_complete = true;
				}
				else {
					do_complete = false;
					printk("%s: %d: dead message\n",
						__func__,__LINE__);
				}
				spin_unlock_irqrestore(&pinstance->_genlock, flags);

				if (do_complete)
					complete(&pdeliver->_msg_arrival);
			}

			/* consume the message */
			rx_ptr = rx_ptr_nxt;
			pshm->_key_section._p_rx_ptr = rx_ptr_nxt;

			if (rx_ptr == tx_ptr) break; /* empty */
		}
	}
	return;
}

static inline u16 zmq_get_sequence_number(void)
{
	static u16 s_sn = 0;
	return ++s_sn;
}

static zmq_deliver_queue_t * zmq_bind(zmq_instance_t * pinstance, u32 type_id)
{
	unsigned long flags;
	zmq_packet_header_t hdr;
	u16 sn;

	hdr._val		= type_id;
	spin_lock_irqsave(&pinstance->_genlock, flags);
	sn = zmq_get_sequence_number();
	spin_unlock_irqrestore(&pinstance->_genlock, flags);

	zmq_firmware_acquire(pinstance); /* let's do it first turn zsp on*/

	/* session request */
	if ( !zmq_request_session(pinstance, sn, &hdr))
	{
		/* session request failed, let's remove the count*/
		zmq_firmware_release(pinstance);
		return NULL;
	}
	return zmq_find_deliver_queue_by_packet(pinstance, &hdr);
}
static void zmq_unbind(zmq_instance_t * pinstance,
	zmq_deliver_queue_t * pdeliver)
{
	zmq_end_session(pdeliver);
	zmq_remove_deliver_queue(pinstance, &pdeliver->_key);
	zmq_firmware_release(pinstance);
}

/*****************************************************************
 * kernel interface
 ****************************************************************/
/* let's just use zmq_deliver_queue_t * as session handle */
void* kzmq_open(int type_id)
{
	zmq_instance_t * pinstance;
	zmq_deliver_queue_t * pdeliver;

	pinstance = zmq_get_instance();
	if (pinstance == NULL) {
		printk(KERN_ERR "%s: %d: no valid zmq instance\n",
			__func__,__LINE__);
		return NULL;
	}

	if ( (pdeliver = zmq_bind(pinstance, type_id)) == NULL) {
		printk(KERN_ERR "%s: %d: bind failed\n", __func__,__LINE__);
		return NULL;
	}

	return pdeliver;
}
EXPORT_SYMBOL(kzmq_open);

void kzmq_info(void* hkzmq, kzmq_info_t * pinfo)
{
	zmq_deliver_queue_t * pdeliver;
	pdeliver = (zmq_deliver_queue_t *)hkzmq;
	if (pdeliver != NULL && pinfo != NULL) {
		pinfo->_class_id	= pdeliver->_key._class_id;
		pinfo->_module_id	= pdeliver->_key._module_id;
		pinfo->_session		= pdeliver->_key._session;
	}
}
EXPORT_SYMBOL(kzmq_info);

void kzmq_close(void* hkzmq)
{
	zmq_deliver_queue_t * pdeliver;
	pdeliver = (zmq_deliver_queue_t *)hkzmq;
	if (pdeliver != NULL)
		zmq_unbind(pdeliver->_pinstance, pdeliver);
}
EXPORT_SYMBOL(kzmq_close);

void kzmq_dt(void* hkzmq, bool ena)
{
	zmq_deliver_queue_t * pdeliver;
	pdeliver = (zmq_deliver_queue_t *)hkzmq;
	if (pdeliver != NULL) {
		if (ena)
			zmq_dtena(pdeliver);
		else
			zmq_dtdsa(pdeliver);
	}
}

int kzmq_read(void* hkzmq, void *pmsg, int len)
{
	zmq_deliver_queue_t * pdeliver;
	pdeliver = (zmq_deliver_queue_t *)hkzmq;
	if (pdeliver == NULL) {
		return -1;
	}
	return zmq_message_recv(pdeliver->_pinstance, pdeliver, pmsg, len);
}
EXPORT_SYMBOL(kzmq_read);

int kzmq_write(void* hkzmq, void *pmsg, int len)
{
	zmq_packet_t		packet;
	int			result;
	zmq_deliver_queue_t * 	pdeliver;

	pdeliver = (zmq_deliver_queue_t *)hkzmq;
	if (pdeliver == NULL) {
		return -1;
	}

	if ( (pmsg == NULL) || (len > ( sizeof(packet._message))) ) {
		printk(KERN_ERR "%s: %d: message buffer invalid\n",
			__func__,__LINE__);
		return -1;
	}

	packet._header._val		= pdeliver->_key._val;
	packet._header._function	= 0;
	packet._header._response	= 0;
	packet._header._reserved	= 0;
	packet._header._packet_length	= len;

	result = zmq_message_send(pdeliver->_pinstance, &packet._header,
			pmsg, len, ZMQ_MSG_SEND_DEFAULT_RETRY,
			zmq_objflag(pdeliver));

	if (result != len) {
		printk(KERN_ERR "%s: %d: message send failed, length %d/%d\n",
				__func__,__LINE__, result, len);
		return result;
	}
	return result;
}
EXPORT_SYMBOL(kzmq_write);

bool	kzmq_delegate(void* hkzmq, void* hmaster)
{
	zmq_deliver_queue_t * 	pdeliver;
	unsigned long flags;

	pdeliver = (zmq_deliver_queue_t *)hkzmq;
	if (pdeliver == NULL) {
		return false;
	}

	spin_lock_irqsave(&(pdeliver->_pinstance->_genlock), flags);
	pdeliver->_pmaster = (zmq_deliver_queue_t *)hmaster;
	if (pdeliver->_pmaster == pdeliver) {
		pdeliver->_pmaster = NULL;
	}
	spin_unlock_irqrestore(&(pdeliver->_pinstance->_genlock), flags);

	return true;
}
EXPORT_SYMBOL(kzmq_delegate);

bool	kzmq_register_data_flow(void* hkzmq,
	zmq_pfn_data_flow_cb callback, void * pctx)
{
	return zmq_register_data_flow((zmq_deliver_queue_t*)hkzmq,
			callback, pctx);
}
EXPORT_SYMBOL(kzmq_register_data_flow);

bool	kzmq_unregister_data_flow(void* hkzmq)
{
	return zmq_unregister_data_flow((zmq_deliver_queue_t*)hkzmq);
}
EXPORT_SYMBOL(kzmq_unregister_data_flow);

bool	kzmq_data_flow_enqueue(void* hkzmq, u32 addr, u32 len,
	u32 low, u32 evts, u32 dir)
{
	return zmq_data_flow_enqueue((zmq_deliver_queue_t*)hkzmq, addr, len,
			low, evts, dir);
}
EXPORT_SYMBOL(kzmq_data_flow_enqueue);

u32	kzmq_data_flow_pos(void* hkzmq)
{
	return zmq_data_flow_pos((zmq_deliver_queue_t*)hkzmq);
}
EXPORT_SYMBOL(kzmq_data_flow_pos);

bool	kzmq_data_flow_wait(void* hkzmq, u32* pos)
{
	return zmq_data_flow_wait((zmq_deliver_queue_t*)hkzmq, pos);
}
EXPORT_SYMBOL(kzmq_data_flow_wait);

/* buffer ring model */
void * kzmq_br_create(void * hkzmq, u32 start, u32 size, u32 low,
	u32 dir, u8* vptr, kzmq_pfn_data_flow_br_cb callback)
{
	return zmq_br_create((zmq_deliver_queue_t*) hkzmq,
			start, size, low, dir, vptr,
			(zmq_pfn_data_flow_br_cb )callback);
}
EXPORT_SYMBOL(kzmq_br_create);

void kzmq_br_destory(void * hbr)
{
	zmq_br_destory(hbr);
}
EXPORT_SYMBOL(kzmq_br_destory);

u32 kzmq_br_zpos(void* hbr)
{
	return zmq_br_zpos(hbr);
}
EXPORT_SYMBOL(kzmq_br_zpos);

bool kzmq_br_get_buffer(void* hbr, u32 * pupos, u32 * pvalidsize)
{
	return zmq_br_get_buffer(hbr, pupos, pvalidsize);
}
EXPORT_SYMBOL(kzmq_br_get_buffer);

bool kzmq_br_deliver_buffer(void* hbr, u32 size)
{
	return zmq_br_deliver_buffer(hbr, size);
}
EXPORT_SYMBOL(kzmq_br_deliver_buffer);


/*****************************************************************
 * standard driver interface
 *****************************************************************/
static struct miscdevice *gs_zmq_devif_miscdev = NULL;
static int gs_zmq_devif_miscdev_cnt = 0;
static struct miscdevice * zmq_query_dev_by_minor(struct miscdevice * plist,
	int cnt, unsigned minor)
{
	int i;
	for (i = 0; i < cnt; i++) {
		if (plist->minor == minor) {
			return plist;
		}
		plist++;
	}
	return NULL;
}

static int zmq_devif_open( struct inode *inode, struct file *filp )
{
	static struct {
		char *	_name;
		int	_type;
	} s_matchtab[] = {
		{ZMQ_DEVN_CODEC,	ZMQ_MSG_ID_CODEC		},
		{ZMQ_DEVN_PLAYBACK_0,	ZMQ_MSG_ID_RENDER_PLAYBACK_0	},
		{ZMQ_DEVN_CAPTURE_0,	ZMQ_MSG_ID_RENDER_CAPTURE_0	},
		{ZMQ_DEVN_PLAYBACK_1,	ZMQ_MSG_ID_RENDER_PLAYBACK_1	},
		{ZMQ_DEVN_CAPTURE_1,	ZMQ_MSG_ID_RENDER_CAPTURE_1	},
	};
	unsigned n = MINOR(inode->i_rdev);
	struct miscdevice * pdev;

	filp->private_data = NULL;

	pdev = zmq_query_dev_by_minor(gs_zmq_devif_miscdev,
					gs_zmq_devif_miscdev_cnt, n);
	if (pdev != NULL) {
		int i;
		for (i = 0; i < _countof(s_matchtab); i++) {
			if (strcmp(s_matchtab[i]._name, pdev->name) == 0) {
				zmq_instance_t * pinstance;

				pinstance = zmq_get_instance();
				if (pinstance == NULL) {
					printk(KERN_ERR "%s: %d: driver down\n",
						__func__,__LINE__);
					return -ENOENT;
				}
				if ( (filp->private_data =zmq_bind(
					pinstance, s_matchtab[i]._type))
					== NULL) {
					printk(KERN_ERR "%s: %d: bind failed\n",
						__func__,__LINE__);
					return -EBUSY;
				}
				return 0;
			}
		}
		return ENOENT;
	}
	else return ENOENT;

	return 0;
}

static int zmq_devif_release(struct inode *inode, struct file *filp)
{
	zmq_deliver_queue_t * pdeliver;
	pdeliver = zmq_find_deliver_queue_by_filp(filp);
	if (pdeliver != NULL) {
		zmq_unbind(pdeliver->_pinstance, pdeliver);
	}
	return 0;
}

static ssize_t zmq_devif_read(struct file *filp, char *buf,
	size_t count, loff_t *f_pos)
{
	zmq_packet_t		packet;
	zmq_deliver_queue_t *	pdeliver;
	int			len;
	int			remain;

	if (count <= 0) {
		printk(KERN_ERR "%s: %d: zero length buffer\n",
			__func__,__LINE__);
		return -1;
	}

	pdeliver = zmq_find_deliver_queue_by_filp(filp);
	if (pdeliver == NULL) {
		printk(KERN_ERR "%s: %d: wrong fd\n", __func__,__LINE__);
		return -1;
	}

	len = zmq_message_recv(pdeliver->_pinstance, pdeliver,
			&packet._message[0], sizeof(packet._message));

	if ( len == - ERESTARTSYS) {
		return -EINTR;
	}
	else if ( len <= 0) {
		printk(KERN_ERR "%s: %d: read error\n",__func__,__LINE__);
		return len;
	}

	remain = copy_to_user(buf, &packet._message[0], len);

	if (remain != 0) {
		printk(KERN_ERR "%s: %d: copy_to_user less than expected, %d/%d\n",
				__func__,__LINE__, len-remain, len);
		return false;
	}

	return len - remain;
}

static ssize_t zmq_devif_write(struct file *filp, const char *buf,
	size_t count, loff_t *f_pos)
{
	zmq_packet_t		packet;
	zmq_deliver_queue_t *	pdeliver;
	int			result;

	if ( count > ( sizeof(packet._message)) ) {
		printk(KERN_ERR "%s: %d: message exceed max packet length\n",
			__func__,__LINE__);
		return -1;
	}
	if (count <= 0) {
		printk(KERN_ERR "%s: %d: zero length buffer\n",
			__func__,__LINE__);
		return -1;
	}

	pdeliver = zmq_find_deliver_queue_by_filp(filp);
	if (pdeliver == NULL) {
		printk(KERN_ERR "%s: %d: wrong fd\n", __func__,__LINE__);
		return -1;
	}

	packet._header._val		= pdeliver->_key._val;
	packet._header._function	= 0;
	packet._header._response	= 0;
	packet._header._reserved	= 0;
	packet._header._packet_length	= count;

	result = copy_from_user(&packet._message[0], buf, count);
	if (result != 0) {
		printk(KERN_ERR "%s: %d: cannot copy all user buffer, %d/%d\n",
				__func__,__LINE__, count - result, count);
		return -1;
	}

	result = zmq_message_send(pdeliver->_pinstance, &packet._header,
			&packet._message[0], count, ZMQ_MSG_SEND_DEFAULT_RETRY,
			zmq_objflag(pdeliver));
	if (result != count) {
		printk(KERN_ERR "%s: %d: message send failed, length %d/%d\n",
				__func__,__LINE__, result, count);
		return result;
	}

	return result;
}

#define ZMQ_IOCTL_ENTER(param, ret)					\
	ret = copy_from_user(&(param), (void*)arg, sizeof(param));	\
	if (ret != 0) {							\
		printk(KERN_ERR "%s: %d: read %s failed, len %d/%d\n",	\
			__func__,__LINE__,#param, ret, sizeof(param));	\
		return -EFAULT;						\
	}
#define ZMQ_IOCTL_LEAVE(param, ret)					\
	ret = copy_to_user((void*)arg, &(param), sizeof(param));	\
	if (ret != 0) {							\
		printk(KERN_ERR "%s: %d: read %s failed, len %d/%d\n",	\
			__func__,__LINE__,#param, ret, sizeof(param));	\
		return -EFAULT;						\
	}

static int zmq_br_ioctl(zmq_ioc_br_param_t * pbrp,
	zmq_deliver_queue_t * pdeliver)
{
	void * hbr;

	pbrp->_stat = 0;
	if (unlikely(pbrp->_op == ZMQ_IOC_BR_OP_CREATE)) {
		if (pdeliver->_data_flow_ctx != NULL) {
			printk(KERN_ERR "%s: %d: ZMQ_IOC_BR_OP_CREATE:"
				"only one BR allowed on each session\n",
				__func__,__LINE__);
			return -EFAULT;
		}
		hbr = kzmq_br_create(pdeliver, pbrp->_physaddr,
			pbrp->_size, pbrp->_low, pbrp->_dir, NULL, NULL);
		if (hbr == NULL) {
			printk(KERN_ERR "%s: %d: ZMQ_IOC_BR_OP_CREATE:"
				"create internal object failed\n",
				__func__,__LINE__);
			return -EFAULT;
		}
		pbrp->_stat = 1;
		return 0;
	}
	if (pdeliver->_data_flow_ctx == NULL) {
		printk(KERN_ERR "%s: %d: ZMQ_IOC_BR_OP_GET_BUFFER:"
				"no BR registered for this session\n",
				__func__,__LINE__);
		return -EFAULT;
	}
	switch (pbrp->_op) {
	case ZMQ_IOC_BR_OP_GET_BUFFER:
		if (kzmq_br_get_buffer(pdeliver->_data_flow_ctx,
			&pbrp->_upos, &pbrp->_usize)) {
			pbrp->_stat = 1;
			return 0;
		}
		break;
	case ZMQ_IOC_BR_OP_DELIVER_BUFFER:
		if (kzmq_br_deliver_buffer(pdeliver->_data_flow_ctx,
			pbrp->_vsize)) {
			pbrp->_stat = 1;
			return 0;
		}
		break;
	case ZMQ_IOC_BR_OP_DESTORY:
		kzmq_br_destory(pdeliver->_data_flow_ctx);
		pbrp->_stat = 1;
		return 0;
	}
	return -EINVAL;
}

static long zmq_devif_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret;

	ret = 0;
	switch (cmd) {
	case ZMQ_IOC_BR_REQ:
		{
			zmq_ioc_br_param_t param;
			zmq_deliver_queue_t * pdeliver;
			pdeliver = zmq_find_deliver_queue_by_filp(file);
			if (pdeliver == NULL) {
				printk(KERN_ERR  "%s: %d: ZMQ_IOC_BR_REQ:"
					"invalid instance\n",
					__func__,__LINE__);
				return -EFAULT;
			}
			ZMQ_IOCTL_ENTER(param, ret);
			zmq_br_ioctl(&param, pdeliver);
			ZMQ_IOCTL_LEAVE(param, ret);
		}
		break;

	case ZMQ_IOC_MDMA_PMEMCPY:
		{
			zmq_ioc_pmemcpy_param_t	param;

			ZMQ_IOCTL_ENTER(param, ret);
			ret = mdma_pmemcpy(param._output, param._input,
						param._length);
			ZMQ_IOCTL_LEAVE(param, ret);
		}
		break;

	case ZMQ_IOC_BIND:
		{
			zmq_ioc_bind_param_t param;
			zmq_instance_t * pinstance;

			if (file->private_data != NULL) {
				printk(KERN_ERR "%s: %d: ZMQ_IOC_BIND:"
					"already bind\n",
					__func__,__LINE__);
				return -EBUSY;
			}

			ZMQ_IOCTL_ENTER(param, ret);
			pinstance = zmq_get_instance();
			if (pinstance == NULL) {
				printk(KERN_ERR "%s: %d: ZMQ_IOC_BIND:"
					"driver not initialized appropriately\n",
					__func__,__LINE__);
				return -ENOENT;
			}
			if ( (file->private_data = zmq_bind(pinstance,
						param._type_id)) == NULL) {
				printk(KERN_ERR "%s: %d: ZMQ_IOC_BIND: failed\n",
					__func__,__LINE__);
				return -EBUSY;
			}
			ZMQ_IOCTL_LEAVE(param, ret);
		}
		break;

	case ZMQ_IOC_GET_SESSION_HANDLE:
		{
			zmq_ioc_session_handle_t param;
			zmq_deliver_queue_t * pdeliver;

			if (file->private_data == NULL) {
				printk(KERN_ERR "%s: %d: ZMQ_IOC_GET_SESSION_HANDLE:"
					"session does not exist\n",
					__func__,__LINE__);
				return -ENOENT;
			}

			pdeliver = (zmq_deliver_queue_t *)file->private_data;

			ZMQ_IOCTL_ENTER(param, ret);
			param._handle	= pdeliver->_key._val;
			param._handlex	= pdeliver;
			ZMQ_IOCTL_LEAVE(param, ret);
		}
		break;

	case ZMQ_IOC_SET_SESSION_MASTER:
		{
			zmq_ioc_session_handle_t param;
			zmq_deliver_queue_t * pdeliver;
			zmq_packet_header_t hdr;
			unsigned long flags;

			if (file->private_data == NULL) {
				printk(KERN_ERR "%s: %d: ZMQ_IOC_SET_SESSION_MASTER:"
					"session does not exist\n",
					__func__,__LINE__);
				return -ENOENT;
			}

			ZMQ_IOCTL_ENTER(param, ret);
			hdr._val = param._handle;
			ZMQ_IOCTL_LEAVE(param, ret);

			if (		(hdr._class_id >= ZMQ_MSG_CLASS_NUMBER)
				||	(hdr._module_id >= ZMQ_MSG_MODULE_NUMBER)
				||	(hdr._session >= ZMQ_MSG_SESSION_NUMBER)
				)
			{
				printk(KERN_ERR "%s: %d: ZMQ_IOC_SET_SESSION_MASTER:"
					"invalid session\n",
					__func__,__LINE__);
				return -ENOENT;
			}

			pdeliver = (zmq_deliver_queue_t *)file->private_data;
			spin_lock_irqsave(&(pdeliver->_pinstance->_genlock), flags);
			pdeliver->_pmaster = zmq_find_deliver_queue_by_packet(
						pdeliver->_pinstance, &hdr);
			if (pdeliver->_pmaster != param._handlex) {
				printk(KERN_ERR "%s: %d: ZMQ_IOC_SET_SESSION_MASTER:"
					"parameter not match\n",
					__func__,__LINE__);
				pdeliver->_pmaster = NULL;
				ret = -EFAULT;
			}
			else if (pdeliver->_pmaster == pdeliver) {
				pdeliver->_pmaster = NULL;
			}
			spin_unlock_irqrestore(&(pdeliver->_pinstance->_genlock), flags);

		}
		break;

	case ZMQ_IOC_CLR_SESSION_MASTER:
		{
			zmq_deliver_queue_t * pdeliver;
			unsigned long flags;

			if (file->private_data == NULL) {
				printk(KERN_ERR "%s: %d: ZMQ_IOC_CLR_SESSION_MASTER:"
					"session does not exist\n",
					__func__,__LINE__);
				return -ENOENT;
			}

			pdeliver = (zmq_deliver_queue_t *)file->private_data;
			spin_lock_irqsave(&(pdeliver->_pinstance->_genlock), flags);
			pdeliver->_pmaster = NULL;
			spin_unlock_irqrestore(&(pdeliver->_pinstance->_genlock), flags);

		}
		break;

	case ZMQ_IOC_ZSP_SOFT_RESET:
		ret = pzipc_set_interrupt(IPC_SOFT_RESET_ID);
		break;
	default:
		ret = -ENOTTY;
		break;
	}


	return ret;
}
static int zmq_devif_mmap(struct file *file, struct vm_area_struct *vma)
{
	return zsp_mmap_datawnd(vma);
}

static struct file_operations zmq_devif_fops = {
	.open			= zmq_devif_open,
	.read			= zmq_devif_read,
	.write			= zmq_devif_write,
	.release		= zmq_devif_release,
	.unlocked_ioctl		= zmq_devif_ioctl,
	.mmap			= zmq_devif_mmap,
	.owner			= THIS_MODULE
};

#ifdef ZMQ_DIAGNOSE_ENA
/*****************************************************************
 * diagnose interface
 ****************************************************************/
static struct device * gsp_diagnose_dev = NULL;
static int zmq_devif_diagnose_open( struct inode *inode, struct file *filp )
{
	return 0;
}
static int zmq_devif_diagnose_release(struct inode *inode, struct file *filp)
{
	return 0;
}
static ssize_t zmq_devif_diagnose_read(struct file *filp, char *buf,
	size_t count, loff_t *f_pos)
{
	return count;
}
static void zmq_diagnose_session_control(zmq_instance_t * pins)
{

	int tn, sn, redo, sessionmax, typemax;
	kzmq_info_t info;
	static int types[] = {
		ZMQ_MSG_ID_CODEC,
		ZMQ_MSG_ID_RENDER_PLAYBACK_0,
		ZMQ_MSG_ID_RENDER_CAPTURE_0,
		ZMQ_MSG_ID_RENDER_PLAYBACK_1,
		ZMQ_MSG_ID_RENDER_CAPTURE_1,
	};
	void* handles[_countof(types)][ZMQ_MSG_SESSION_NUMBER];
	void* handle;
	printk("[ZMQDIAG]:SESSION TEST\n");
	sessionmax = ZMQ_MSG_SESSION_NUMBER;
	typemax = _countof(types);
	redo = 1;
	while (redo-- >0) {
		for (tn = 0; tn < typemax; tn++) {
			for (sn = 0; sn < sessionmax; sn++) {
				handles[tn][sn] = NULL;
			}
		}
		for (tn = 0; tn < typemax; tn++) {
			for (sn = 0; sn < sessionmax; sn++) {
				handle = kzmq_open(types[tn]);
				if (handle != NULL) {
					kzmq_dt(handle, true);
					kzmq_info(handle, &info);
					printk("\t\t session[%d][%d][%d] %p created\n",
						info._class_id,
						info._module_id,
						info._session, handle);
				}
				else break;
				handles[tn][sn] = handle;
			}
			printk("\t\t type[0x%08x] support max %d sessions\n",
				types[tn], sn);
		}
		for (tn = 0; tn < typemax; tn++) {
			for (sn = 0; sn < sessionmax; sn++) {
				if (handles[tn][sn] != NULL)
					kzmq_close(handles[tn][sn]);
			}
		}
	}
	printk("[ZMQDIAG]:SESSION TEST END\n");
}

enum {
	ZMQ_STREAM_START		= 0,
	ZMQ_STREAM_STOP			= 1,
};
typedef struct
{
	u32			cmd;
/*
	u16			format;
	u16			sample_rate;
	u16			periods;
	u16			frames_per_period;
	u16			routing_target;
*/
} zmq_stream_t;

static void zmq_diagnose_stream(zmq_instance_t * pins, bool dataout)
{
	void* handle;
	void* hbr;
	zmq_stream_t sctx;
	struct completion cmp;
	u32 bufsize, buflow, curpos, cursize, totalsz;
	u8 * pbuffer;
	dma_addr_t dmaphyaddr;

	printk("[ZMQDIAG]:STREAM TEST\n");

	pbuffer	= NULL;
	bufsize	= 0x800;	/* 2kB */
	buflow	= 0x200;	/* low 512B */

	if (gsp_diagnose_dev != NULL) {
		gsp_diagnose_dev->coherent_dma_mask = 0xffffffff;
		pbuffer = dmam_alloc_coherent(gsp_diagnose_dev, bufsize*3,
					&dmaphyaddr, GFP_KERNEL);
		memset(pbuffer, 0xC5, bufsize*3);
	}

	if ((pbuffer != NULL)
		&& (handle = kzmq_open(ZMQ_MSG_ID_STREAM)) != NULL) {

		kzmq_dt(handle, true);
		/* give the middle bufsize range to be the buffer used */
		hbr = kzmq_br_create(handle, dmaphyaddr+bufsize,
					bufsize, buflow,
					dataout?ZMQ_DATA_FLOW_DIR_P2Z:
						ZMQ_DATA_FLOW_DIR_Z2P,
					pbuffer+bufsize, NULL);

		sctx.cmd = ZMQ_STREAM_START;
		kzmq_write(handle, &sctx, sizeof(sctx));
		init_completion(&cmp);
		printk("[ZMQDIAG]:WAIT BEFORE DATA, shadow buffer 0x%08x, size 0x%08x\n",
				dmaphyaddr+bufsize, bufsize);
		wait_for_completion_interruptible_timeout(
				&cmp,msecs_to_jiffies(2*1000));
		printk("[ZMQDIAG]:OK\n");

		totalsz = 0x2000; /* 8kB */

		if (hbr != NULL) {
			/* first time */
			kzmq_br_deliver_buffer(hbr,bufsize);
		}
		while (hbr != NULL) { /* loop until break */
			if (kzmq_br_get_buffer(hbr, &curpos, &cursize)) {
				printk("[ZMQDIAG]: move to pos[0x%04x], size[0x%04x]\n",
				       curpos, cursize);
				if (!dataout) {
					u32 k;
					for (k = 0; k < cursize; k++) {
						if (pbuffer[bufsize+(curpos+k)%bufsize] != 0x16) {
							printk("buffer error %d, %x\n",
								k, pbuffer[bufsize+(curpos+k)%bufsize]);
							break;
						}
					}
				}
				kzmq_br_deliver_buffer(hbr,cursize);
				if (totalsz < cursize) break;
				totalsz -= cursize;
			}
			else {
				printk("[ZMQDIAG]: get buffer error\n");
				break;
			}
		}


		sctx.cmd = ZMQ_STREAM_STOP;
		kzmq_write(handle, &sctx, sizeof(sctx));

		printk("[ZMQDIAG]:WAIT BEFORE EXIT\n");
		wait_for_completion_interruptible_timeout(
					&cmp,msecs_to_jiffies(2*1000));
		printk("[ZMQDIAG]:OK\n");


		if (hbr != NULL) kzmq_br_destory(hbr);
		kzmq_close(handle);
	}
	else {
		printk("[ZMQDIAG]:allocate DMA buffer or create stream object failed\n");
	}

	if (gsp_diagnose_dev != NULL) {
		if (pbuffer != NULL) {
			/* verify nothing changed adjacent the buffer part used */
			int a;
			printk("[ZMQDIAG]:CHECK BUFFER RANGE\n");
			for (a = 0; a < bufsize; a++) { /* before */
				if (pbuffer[a] != 0xC5) {
					printk("buffer underrun %d, %x\n",
						a, pbuffer[a]);
					break;
				}
			}
			if (!dataout) {
				for (a = bufsize; a < bufsize*2; a++) {
					if (pbuffer[a] != 0x16) {
						printk("internal buffer copy back error %d, %x\n",
							a, pbuffer[a]);
						break;
					}
				}
			}
			for (a = bufsize*2; a < bufsize*3; a++) { /* after */
				if (pbuffer[a] != 0xC5) {
					printk("buffer overrun %d, %x\n",
						a, pbuffer[a]);
					break;
				}
			}
			dmam_free_coherent(gsp_diagnose_dev, bufsize*3,
				pbuffer, dmaphyaddr);
		}
	}

	printk("[ZMQDIAG]:STREAM TEST END\n");
}

static void zmq_diagnose_clk(u32 flags)
{
	int ret;
	static int newsrc = MMP_ZSP_ASCLK_22579200;
	static int newspd = MMP_ZSP_SPD_PERFORMACE;
	static const char * spdname[] = {
		"Performance", "Balanced", "Powersaving"
	};
	static const char * srcname[] = {
		"PMU", "AudioPLL"
	};
	static const char * asclkname[] = {
		"44.1", "48"
	};
	struct mmp_zsp_clkcfg cfg;

	flags = flags & (MMP_ZSP_SPD_FLAGS | MMP_ZSP_ASCLK_FLAGS);

	if (flags & MMP_ZSP_ASCLK_FLAGS)
		cfg.asclk = (newsrc++) % MMP_ZSP_ASCLK_NUMBER;

	if (flags & MMP_ZSP_SPD_FLAGS)
		cfg.spd = (newspd++) % MMP_ZSP_SPD_NUMBER;

	ret = zsp_set_clock_preference(flags, &cfg);

	printk("[ZMQDIAG]: set %s, setting: spd:%s, src:%s, asclk:%s\n",
			(ret==0)?"ok":"fail"
			, spdname[cfg.spd]
			, srcname[cfg.src]
			, asclkname[cfg.asclk]
		);
}
static void zmq_diagnoze(zmq_instance_t * pins, char op)
{
	switch (op) {
	case 'a':
		zmq_diagnose_session_control(pins);
		break;
	case 's':
		zmq_diagnose_stream(pins, true);
		break;
	case 'S':
		zmq_diagnose_stream(pins, false);
		break;
	case 'b':
		ZMQ_SET_BITS(pins->_test_mode, ZMQ_TEST_MODE_LOOPBACK_BIT);
		break;
	case 'B':
		ZMQ_CLR_BITS(pins->_test_mode, ZMQ_TEST_MODE_LOOPBACK_BIT);
		break;
	case 'l':
		zmq_load_logger(pins);
		ZMQ_SET_BITS(pins->_test_mode, ZMQ_TEST_MODE_LOGGER_BIT);
		break;
	case 'L':
		ZMQ_CLR_BITS(pins->_test_mode, ZMQ_TEST_MODE_LOGGER_BIT);
		zmq_unload_logger(pins);
		break;
	case 'c':
		zmq_diagnose_clk(MMP_ZSP_ASCLK_FLAGS);
		break;
	case 'f':
		zmq_diagnose_clk(MMP_ZSP_SPD_FLAGS);
		break;
	case 'U': zmq_dt_masks = 0; break;
	case 'V': zmq_dt_masks = ZMQ_DT_SESSION; break;
	case 'v': zmq_dt_masks = ZMQ_DT_MDMA; break;
	case 'W': zmq_dt_masks = ZMQ_DT_INTR; break;
	case 'w': zmq_dt_masks = ZMQ_DT_DF | ZMQ_DT_DF_BR; break;
	case 'X': zmq_dt_masks = ZMQ_DT_SESSION | ZMQ_DT_INTR | ZMQ_DT_MSGDUMP; break;
	case 'Y': zmq_dt_masks = ZMQ_DT_SEND | ZMQ_DT_RECV; break;
	case 'Z': zmq_dt_masks = ZMQ_DT_ALL; break;
	}
}
static ssize_t zmq_devif_diagnose_write(struct file *filp, const char *buf,
	size_t count, loff_t *f_pos)
{
	zmq_instance_t * pins;
	int loop, remain, block, i, j, cpsize;
	char optab[64];
	size_t chcnt;

	chcnt	= count;
	pins	= zmq_get_instance();

	if (pins == NULL || count <= 0) {
		printk("%s: wrong diagnose write buffer or invalid instance\n",
			__func__);
		return -1;
	}

	block	= _countof(optab);
	loop	= count/block;
	remain	= count - loop*block;
	for (j = 0; j < loop; j++) {
		cpsize = copy_from_user(optab, &buf[j*block], block);
		for (i = 0; i < block; i++) {
			zmq_diagnoze(pins, optab[i]);
		}
	}
	if (remain >0) {
		cpsize = copy_from_user(optab, &buf[loop*block], remain);
		for (i = 0; i < remain; i++) {
			zmq_diagnoze(pins, optab[i]);
		}
	}
	return count;
}
static struct file_operations zmq_devif_diagnose_fops = {
	.open			= zmq_devif_diagnose_open,
	.read			= zmq_devif_diagnose_read,
	.write			= zmq_devif_diagnose_write,
	.release		= zmq_devif_diagnose_release,
	.owner			= THIS_MODULE
};
#endif

static struct miscdevice zmq_devif_miscdev[] = {
#ifdef ZMQ_DIAGNOSE_ENA
	{
		.minor			= MISC_DYNAMIC_MINOR,
		.name			= ZMQ_DEVN_DIAGNOSE,
		.fops			= &zmq_devif_diagnose_fops,
	},
#endif
	{
		.minor			= MISC_DYNAMIC_MINOR,
		.name			= ZMQ_DEVN_CODEC,
		.fops			= &zmq_devif_fops,
	},
	{
		.minor			= MISC_DYNAMIC_MINOR,
		.name			= ZMQ_DEVN_PLAYBACK_0,
		.fops			= &zmq_devif_fops,
	},
	{
		.minor			= MISC_DYNAMIC_MINOR,
		.name			= ZMQ_DEVN_CAPTURE_0,
		.fops			= &zmq_devif_fops,
	},
	{
		.minor			= MISC_DYNAMIC_MINOR,
		.name			= ZMQ_DEVN_PLAYBACK_1,
		.fops			= &zmq_devif_fops,
	},
	{
		.minor			= MISC_DYNAMIC_MINOR,
		.name			= ZMQ_DEVN_CAPTURE_1,
		.fops			= &zmq_devif_fops,
	},
};

static irqreturn_t zmq_receive_handler(int irq, void *dev_id)
{
#ifdef ZMQ_USING_WORKQUEUE
	queue_work(sg_recieve_working_queue, &ag_recieve_wq_request);
#else
	zmq_message_arrival(zmq_get_instance());
#endif
	return IRQ_HANDLED;
}

static void zmq_message_dispatcher(struct work_struct *work)
{
	zmq_message_arrival(zmq_get_instance());
}

static int __init zmq_devif_init(void)
{
	u32 ret = 0;
	int i;
	zmq_pool_init(ZMQ_MSG_SLOT_DELIVER_NUM *
			(ZMQ_MSG_SLOT_SIZE + sizeof(struct list_head)));
	gs_zmq_devif_miscdev = zmq_devif_miscdev;
	gs_zmq_devif_miscdev_cnt = _countof(zmq_devif_miscdev);

	for (i = 0; i < _countof(zmq_devif_miscdev); i++) {
		ret = misc_register(&zmq_devif_miscdev[i]);
		if (ret) {
			printk("%s: %d: cannot register device %s\n",
					__func__,__LINE__,
					zmq_devif_miscdev[i].name);
			return ret;
		}
		else {
			zmq_dt(ZMQ_DT_MEM,("%s: %d: register device MINOR(%d) %s\n",
					__func__,__LINE__,
					zmq_devif_miscdev[i].minor,
					zmq_devif_miscdev[i].name));
		}
	}

#ifdef ZMQ_DIAGNOSE_ENA
	gsp_diagnose_dev = zmq_devif_miscdev[1].this_device;
#endif

	zmq_get_instance(); /* trigger init */

	pzipc_add_isr(IPC_MSG_TRANSFER_ID,  zmq_receive_handler);

	sg_recieve_working_queue = create_workqueue("zmq_message_dispatcher");
	INIT_WORK(&ag_recieve_wq_request, zmq_message_dispatcher);

	return 0;
}

static void __exit zmq_devif_exit(void)
{
	int i;
	zmq_pool_uninit();

	pzipc_remove_isr(IPC_MSG_TRANSFER_ID);
	destroy_workqueue(sg_recieve_working_queue);

	zmq_deinit_instance();

	gsp_diagnose_dev = NULL;

	for (i = _countof(zmq_devif_miscdev) - 1; i >=0 ; i--) {
		misc_deregister(&zmq_devif_miscdev[i]);
	}


	if (gs_zmq_memaf_cnt != 0) {
		printk("%s:%d WARNING >> alloc/free does not match %d\n",
				__func__,__LINE__, gs_zmq_memaf_cnt);
	}
}

late_initcall(zmq_devif_init);
module_exit(zmq_devif_exit);

MODULE_AUTHOR("Marvell Technology");
MODULE_DESCRIPTION("MMP Message Queue Driver For ZSP");
MODULE_LICENSE("GPL");
