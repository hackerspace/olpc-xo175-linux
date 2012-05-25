/*
 * mmp3_hsi.h
 *
 * Marvell HSI - Physical Layer
 *
 * Copyright:  (C) Copyright 2011 Marvell International Ltd.
 *              Henry Zhao <xzhao10@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef _MMP3_HSI_H_
#define _MMP3_HSI_H_

#include <linux/types.h>


/**********************/
/* HSI Macros Defines */
/**********************/
#define HSI_LOCK(USER) (mutex_lock(&(hsi_core->hsi_mutex)))
#define HSI_UNLOCK(USER) (mutex_unlock(&(hsi_core->hsi_mutex)))

#define HSI_READ32(OFFSET) (__raw_readl(hsi_core->mmio_hsi_base + (OFFSET)))
#define HSI_WRITE32(OFFSET, VALUE) \
	(__raw_writel((VALUE), hsi_core->mmio_hsi_base + (OFFSET)))
#define HSI_MASK32(MSB, LSB) \
	((~((u32)0-(u32)(1<<(1+(MSB)-(LSB))))) << (LSB))
#define HSI_LSB_MASK32(MSB) (~((u32)0-(u32)(1<<(MSB))))
#define HSI_MODIFY32(SRC, MSB, LSB, VALUE) \
	(((SRC) & (~HSI_MASK32(MSB, LSB))) | \
	(((VALUE)&HSI_LSB_MASK32(1+(MSB)-(LSB))) << (LSB)))
#define HSI_GET32(SRC, MSB, LSB) (((SRC)&HSI_MASK32(MSB, LSB)) >> (LSB))
#define HSI_PWRCLK_READ32(OFFSET) \
	(ioread32(hsi_core->mmio_pwrclk + (OFFSET)))
#define HSI_PWRCLK_WRITE32(OFFSET, VALUE) \
	(iowrite32((VALUE), hsi_core->mmio_pwrclk + (OFFSET)))

#define CMD_CH		0
#define DATA_CH		1

/********************/
/* HSI Regs Defines */
/********************/
#define HSI_CH_NUM				16
#define HSI_TLTC_CH_NUM			HSI_CH_NUM
#define HSI_TX_CH_NUM			HSI_CH_NUM
#define HSI_RX_CH_NUM			HSI_CH_NUM

#define HSI_ENABLE0				0x0000
#define HSI_ENABLE1				0x0004
#define HSI_ENABLE2				0x0008
#define HSI_ENABLE3				0x000C
#define HSI_CNTRL				0x0010
#define HSI_CONFIG1				0x0014
#define HSI_CONFIG2				0x0018
#define HSI_BRK_CONFIG			0x001C
#define HSI_BURST_CONFIG		0x0020
#define HSI_FTO_CONFIG			0x0024
#define HSI_RTBC_CONFIG			0x0028
#define HSI_RIPC				0x002C

#define TLTC_CONFIG				0x0040
#define RX_FIFO_CONFIG			0x0080
#define TX_FIFO_CONFIG			0x00C0
#define HSI_IPC_DATA			0x0100
#define HSI_IPC_DATA_MASK		0x0140
#define CHNL_FIFO_STATUS		0x0180
#define CHNL_STATUS				0x0200
#define CHNL_INT_MASK			0x0240
#define CHNL_INT_STATUS			0x0280
#define HSI_INT_MASK0			0x0300
#define HSI_INT_MASK1			0x0304
#define HSI_INT_MASK2			0x0308
#define HSI_INT_MASK3			0x030C
#define HSI_INT_STATUS0			0x0310
#define HSI_INT_STATUS1			0x0314
#define HSI_INT_STATUS2			0x0318
#define HSI_INT_STATUS3			0x031C

#define RX_DMA_NDPTR			0x0400
#define RX_DMA_DADR				0x0440
#define RX_DMA_DLEN				0x0480
#define RX_DMA_CTRL				0x04C0
#define TX_DMA_NDPTR			0x0500
#define TX_DMA_SADR				0x0540
#define TX_DMA_DLEN				0x0580
#define TX_DMA_CTRL				0x05C0

#define DMA_ARB_MODE			0x0600
#define TX_FIFO					0x0A00
#define RX_FIFO					0x0B00


 /* HSI Enable Register (HSI_ENABLE) */
#define BIT_TX_PAUSE		5
#define BIT_RX_PAUSE		4
#define BIT_HSI_ENABLE		0

 /* HSI Control Register (HSI_CNTRL) */
#define BIT_HSI_RIPC_EN		16
#define BIT_IPC_CMD_ID_MSB	15
#define BIT_IPC_CMD_ID_LSB	12
#define BIT_IPC_EN			8
#define BIT_TX_BIT_RST		4
#define BIT_TX_WRM_RST		3
#define BIT_RX_BIT_RST		2
#define BIT_RX_WRM_RST		1
#define BIT_RCVR_MODE		0

/* HSI Configuration Register	 1 */
#define BIT_TLTC_TO_PRE_MSB	31
#define BIT_TLTC_TO_PRE_LSB	24
#define BIT_NUMB_CHAN_MSB	18
#define BIT_NUMB_CHAN_LSB	16
#define BIT_DATA_FLOW_MSB	11
#define BIT_DATA_FLOW_LSB	10
#define BIT_TRAN_MODE		8
#define BIT_READY_MODE		2
#define BIT_TX_WAKE_IMPL	1
#define BIT_RX_WAKE_IMPL	0

/* HSI Configuration Register	 2 */
#define BIT_RX_CLK_DIV_MSB	31
#define BIT_RX_CLK_DIV_LSB	16
#define BIT_TX_CLK_DIV_MSB	15
#define BIT_TX_CLK_DIV_LSB	0

/* BREAK Configuration Register	 */
#define BIT_BRK_RCVR_CLR	18
#define BIT_BRK_FIFO_CLR	17
#define BIT_TX_BREAK		16
#define BIT_BRK_CNT_MSB		5
#define BIT_BRK_CNT_LSB		0


/* BURST Configuration Register */
#define BIT_FB_ENALBE		16
#define BIT_FB_CNT_MSB		7
#define BIT_FB_CNT_LSB		0

/* FIFO Status Register */
#define BIT_TX_FRM_CNT_MSB	22
#define BIT_TX_FRM_CNT_LSB	16
#define BIT_RX_FRM_CNT_MSB	6
#define BIT_RX_FRM_CNT_LSB	0

/* Trasnmitter FIFO Status Register 0 */
#define BIT_TX_FIFO_FULL	10

/* Receiver FIFO Configuration Register */
#define BIT_RX_FIFO_EN	0
#define BIT_RX_REQ_EN	1

/* Trasnmitter FIFO Configuration Register */
#define BIT_TX_REQ_EN	1
#define BIT_TX_FIFO_EN	0

/* Receiver Interrupt Status Register */
#define HSI_RX_INT_STATUS_RXN_FIFO_THRS_LSB		0
#define HSI_RX_INT_STATUS_RXN_FIFO_THRS_MSB		7
#define HSI_RX_INT_STATUS_RXN_FRM_LOST_LSB		8
#define HSI_RX_INT_STATUS_RXN_FRM_LOST_MSB		15
#define HSI_RX_INT_STATUS_RCVR_ERR			28
#define HSI_RX_INT_STATUS_FTO_EXPIRED			29
#define HSI_RX_INT_STATUS_BRK_RCVD			30

/* Trasnmitter Interrupt Status Register		*/
#define HSI_TX_INT_STATUS_TXN_FIFO_THRS_LSB		0
#define HSI_TX_INT_STATUS_TXN_FIFO_THRS_MSB		7
#define HSI_TX_INT_STATUS_TXN_FRM_LOST_LSB		8
#define HSI_TX_INT_STATUS_TXN_FRM_LOST_MSB		15


/* HSI Interrupt Status Register values */
#define HSI_CHNL_INT_0				16
#define HSI_CHNL_INT_1				17
#define HSI_CHNL_INT_15				31

/* HSI Channel Status Register values */
#define HSI_CH_STATUS_IPC_MSB				31
#define HSI_CH_STATUS_IPC_LSB				24
#define HSI_CH_STATUS_IPC_MASK				0xFF000000
#define HSI_CH_STATUS_IPC_CMD				(0x1 << 20)
#define HSI_CH_STATUS_RX_DMA_ERR			(0x1 << 14)
#define HSI_CH_STATUS_RX_DMA_DONE			(0x1 << 13)
#define HSI_CH_STATUS_RX_FIFO_UNDER			(0x1 << 12)
#define HSI_CH_STATUS_RX_FRM_LOST			(0x1 << 11)
#define HSI_CH_STATUS_RX_FIFO_THRS			(0x1 << 9)
#define HSI_CH_STATUS_TX_DMA_ERR			(0x1 << 6)
#define HSI_CH_STATUS_TX_DMA_DONE			(0x1 << 5)
#define HSI_CH_STATUS_TX_FIFO_UNDER			(0x1 << 4)
#define HSI_CH_STATUS_TX_FRM_LOST			(0x1 << 3)
#define HSI_CH_STATUS_TX_FIFO_THRS			(0x1 << 1)
#define HSI_CH_STATUS_CLR_RX_MASK			0x00007A00
#define HSI_CH_STATUS_CLR_TX_MASK			0x0000007A
#define HSI_CH_STATUS_CLR_ALL_MASK			0xFF107A7A

/* HSI Channel INT Mask Register values */
#define HSI_CH_MSK_IPC_MSB				31
#define HSI_CH_MSK_IPC_LSB				24
#define HSI_CH_MSK_IPC_MASK				0xFF000000
#define HSI_CH_MSK_IPC_DATA				(0x1 << 21)
#define HSI_CH_MSK_IPC_CMD				(0x1 << 20)
#define HSI_CH_MSK_RX_FIFO				(0x1 << 18)
#define HSI_CH_MSK_TX_FIFO				(0x1 << 17)
#define HSI_CH_MKS_TLTC_EXPIRE			(0x1 << 16)
#define HSI_CH_MSK_RX_DMA_ERR			(0x1 << 14)
#define HSI_CH_MSK_RX_DMA_DONE			(0x1 << 13)
#define HSI_CH_MSK_RX_FIFO_UNDER		(0x1 << 12)
#define HSI_CH_MSK_RX_FRM_LOST			(0x1 << 11)
#define HSI_CH_MSK_RX_FIFO_FULL			(0x1 << 10)
#define HSI_CH_MSK_RX_FIFO_THRS			(0x1 << 9)
#define HSI_CH_MSK_RX_FIFO_EMPTY		(0x1 << 8)
#define HSI_CH_MSK_RX_FIFO_N_EMPTY		(0x1 << 7)
#define HSI_CH_MSK_TX_DMA_ERR			(0x1 << 6)
#define HSI_CH_MSK_TX_DMA_DONE			(0x1 << 5)
#define HSI_CH_MSK_TX_FIFO_OVER			(0x1 << 4)
#define HSI_CH_MSK_TX_FRM_LOST			(0x1 << 3)
#define HSI_CH_MSK_TX_FIFO_FULL			(0x1 << 2)
#define HSI_CH_MSK_TX_FIFO_THRS			(0x1 << 1)
#define HSI_CH_MSK_TX_FIFO_EMPTY		(0x1 << 0)
#define HSI_CH_MSK_TX					0x2007F
#define HSI_CH_MSK_RX					0x47F80
#define HSI_CH_MSK_ALL					0x67FFF


/* HSI Main INT Mask Register values */
#define HSI_MSK_CH0_BIT		16


/**************/
/* Enums      */
/**************/
enum hsi_if_arb_mode_enum {
	HSI_IF_ARB_ROUND_ROBIN	= 0,
	HSI_IF_ARB_PRIORITY		= 1,
	HSI_IF_ARB_DUMMY		= 0x7FFFFFFF,
} ;

enum hsi_if_bit_trans_mode_enum {
	HSI_IF_OP_MODE_STREAM	= 0,
	HSI_IF_OP_MODE_FRAME	= 1,
	HSI_IF_OP_MODE_DUMMY	= 0x7FFFFFFF,
};

enum hsi_if_ch_bits_enum {
	HSI_IF_CHANNEL_BITS_0		= 0,
	HSI_IF_CHANNEL_BITS_1		= 1,
	HSI_IF_CHANNEL_BITS_2		= 2,
	HSI_IF_CHANNEL_BITS_3		= 3,
	HSI_IF_CHANNEL_BITS_4		= 4,
	HSI_IF_CHANNEL_BITS_RSVD	= 5
};

enum hsi_if_data_flow_enum {
	HSI_IF_DATA_FLOW_SYNCHRONIZED	= 0,
	HSI_IF_DATA_FLOW_PIPELINED		= 1,
	HSI_IF_DATA_FLOW_REALTIME		= 2,
	HSI_IF_DATA_FLOW_DUMMY			= 0x7FFFFFFF,
};

enum hsi_ready_mode_enum {
	HSI_READY_MODE_V1_1 = 0,
	HSI_READY_MODE_V1_0 = 1
};

enum hsi_tx_wake_impl_enum {
	HSI_TX_WAKE_NOT_IMPL = 0,
	HSI_TX_WAKE_IMPL = 1
};

enum hsi_rx_wake_impl_enum {
	HSI_RX_WAKE_NOT_IMPL = 0,
	HSI_RX_WAKE_IMPL = 1
};

enum hsi_rcv_evt_enum {
	HSI_EVENT_RECEIVE_DATA = 0,
	HSI_EVENT_RECEIVER_ERROR,
	HSI_EVENT_RECEIVER_DUMMY	= 0x7FFFFFFF,
};

enum hsi_fifo_valve_enum {
	HSI_RECV_FIFO_VALVE_DISABLE = 0,
	HSI_RECV_FIFO_VALVE_ENABLE,
};

enum hsi_fifo_req_sel_enum {
	HSI_FIFO_REQ_INT = 0,
	HSI_FIFO_REQ_DMA
};

enum hsi_tltc_mode_enum {
	TLTC_MODE_BURST = 0,
	TLTC_MODE_TIMEOUT
};

enum hsi_if_tx_state_enum {
	HSI_IF_TX_STATE_IDLE = 0,
	HSI_IF_TX_STATE_WAIT,
	HSI_IF_TX_STATE_TRANSMIT,
	HSI_IF_TX_STATE_SLEEP,
	HSI_IF_TX_STATE_DUMMY		= 0x7FFFFFFF
} ;

enum hsi_if_rx_state_enum {
	HSI_IF_RX_STATE_IDLE = 0,
	HSI_IF_RX_STATE_RECEIVING,
	HSI_IF_RX_STATE_ERROR,
	HSI_IF_RX_STATE_TIME_OUT,
	HSI_IF_RX_STATE_SLEEP,
	HSI_IF_RX_STATE_DUMMY		= 0x7FFFFFFF
};

enum hsi_if_rx_err_enum {
	HSI_IF_RX_ERROR_NONE = 0,
	HSI_IF_RX_ERROR_SIGNAL,
	HSI_IF_RX_ERROR_TIME_OUT,
	HSI_IF_RX_ERROR_DUMMY		= 0x7FFFFFFF
};

enum hsi_if_fto_enable_enum {
	FTO_DISABLED = 0,
	FTO_ENABLED,
};

enum hsi_work_mode_enum {
	HSI_BOOTROM_MODE = 0,
	HSI_CPIMAGE_MODE,
};

/**************/
/* Typedefs   */
/**************/
typedef void(*hsi_break_received_handler_ptr)(void);
typedef void(*hsi_rx_thrs_handler_ptr)(u32 channel, u32 data);
typedef void(*hsi_dma_done_handler_ptr)(u32 channel, void *cookie);

/**************/
/* Structs    */
/**************/
struct strt_hsi_brk_cfg {
	bool brk_rcv_clr;
	bool brk_fifo_clr;
	u32 brk_cnt;
};

struct strt_hsi_tltc_chnl_cfg {
	u32 tltc_cnt;
	enum hsi_tltc_mode_enum tltc_mode;
	bool tltc_enable;
};

struct strt_hsi_tx_ch_cfg {
	bool tx_busy;
	bool tx_fifo_req_ena;
	bool tx_fifo_int_ena;
	bool tx_fifo_enable;

	u32 tx_ch_fifo_base;
	u32 tx_ch_fifo_size;
	u32 tx_ch_fifo_thrs;
	u32 tx_ch_priority;
	enum hsi_fifo_req_sel_enum tx_fifo_req_sel;


	u32 *tx_chain_mode_ptr;
	void *dma_cookie;
	hsi_dma_done_handler_ptr tx_dma_handler;
	bool dma_err;

	u32 tx_frame_cnt;
	u32 tx_frame_sent;
	u32 *tx_frame;

	struct completion	tx_thrs_event;
};

struct strt_hsi_rx_ch_cfg {
	bool rx_busy;
	bool rx_fifo_req_ena;
	bool rx_fifo_int_ena;
	bool rx_fifo_enable;
	bool ipc_cmd_rcv_ena;

	u32 rx_ch_fifo_base;
	u32 rx_ch_fifo_size;
	u32 rx_ch_fifo_thrs;
	enum hsi_fifo_valve_enum rx_fifo_valve;
	enum hsi_fifo_req_sel_enum rx_fifo_req_sel;

	u32 *rx_chain_mode_ptr;
	void *dma_cookie;
	hsi_dma_done_handler_ptr rx_dma_handler;
	hsi_rx_thrs_handler_ptr rx_cpu_handler;
	bool dma_err;

	u32 rx_frame_cnt;
	u32 rx_frame_rcv;
	u32 *rx_frame;

	struct completion	rx_thrs_event;
} ;

struct hsi_cfg_strt {
	enum hsi_if_ch_bits_enum ch_bits;
	enum hsi_if_data_flow_enum data_flow;
	enum hsi_if_bit_trans_mode_enum trans_mode;
	enum hsi_ready_mode_enum ready_mode;
	enum hsi_tx_wake_impl_enum tx_wake_impl;
	enum hsi_rx_wake_impl_enum rx_wake_impl;
	enum hsi_work_mode_enum mode;

	u32 tltc_pre_scaler;
	u32 tx_rate_div;
	u32 rx_rate_div;

	enum hsi_if_fto_enable_enum fto_cnt_ena;
	u32 fto_cnt;

	enum hsi_if_arb_mode_enum tx_arb_mode;
	enum hsi_if_arb_mode_enum rx_arb_mode;

	hsi_break_received_handler_ptr rx_brk_handler;

	struct strt_hsi_brk_cfg brk_cfg;
	struct strt_hsi_tltc_chnl_cfg tltc_ch_cfg[HSI_TLTC_CH_NUM];
	struct strt_hsi_tx_ch_cfg tx_chnl_config[HSI_TX_CH_NUM];
	struct strt_hsi_rx_ch_cfg rx_chnl_config[HSI_RX_CH_NUM];
};

struct hsi_desp_strt {
	u32 next_desc_ptr;
	u32 src_addr;
	u32 dest_addr;
	u32 byte_cnt;
} ;

struct mmp3_hsi_core {
	struct device	*dev;
	struct clk		*clk;

	struct resource *mem_res_hsi;
	unsigned long	mmio_base_phys;
	unsigned long	mmio_size;
	void __iomem	*mmio_hsi_base;

	int irq;
	struct mutex hsi_mutex;
	spinlock_t s_irqlock;
	struct hsi_cfg_strt *hsi_config;
	struct hsi_platform_data *plat_data;
};

int hsi_register_rx_break_handler(hsi_break_received_handler_ptr handler);
int hsi_config_cpu_tx_channel(int channel, bool enable_fifo);
int hsi_config_cpu_rx_channel(int channel,
	hsi_rx_thrs_handler_ptr rx_cpu_hdlr, bool enable_fifo);
int hsi_config_dma_tx_channel(int channel,
	hsi_dma_done_handler_ptr tx_dma_hdlr, bool enable_fifo);
int hsi_config_dma_rx_channel(int channel,
	hsi_dma_done_handler_ptr rx_dma_hdlr, bool enable_fifo);

int hsi_transmit(int chnl, u32 *data, u32 byte_cnt);
int hsi_transmit_cmd(int chnl, u32 *data, u32 byte_cnt, u32 *tx_cnt);
int hsi_transmit_nblk(int chnl, u32 *data, u32 byte_cnt, u32 *tx_cnt);
int hsi_transmit_dma(int chnl, u32 *data, u32 byte_cnt, void * cookie);
int hsi_transmit_dma_chain(int channel, u32 **data,
	u32 *byte_cnt, u32 entries, void * cookie);
int hsi_transmit_break(void);

int hsi_receive(int channel, u32 *data, u32 byte_cnt);
int hsi_receive_dma(int chnl, u32 *data, u32 byte_cnt, void * cookie);
int hsi_receive_dma_chain(int channel, u32 **data,
	u32 *byte_cnt, u32 entries, void * cookie);

int hsi_reset_tx(void);
int hsi_reset_rx(void);
int mmp3_hsi_enable_hw(void);
int mmp3_hsi_disable_hw(void);

bool hsi_disable_tx_fifo_int(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg);
bool hsi_disable_tx_fifo_req(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg);
bool hsi_enable_tx_fifo_req(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg);
bool hsi_enable_tx_fifo_cpu(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg);
bool hsi_enable_tx_fifo_dma(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg);

bool hsi_disable_rx_fifo_int(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg);
bool hsi_disable_rx_fifo_req(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg);
bool hsi_enable_rx_fifo_req(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg);
bool hsi_enable_rx_fifo_cpu(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg);
bool hsi_enable_rx_fifo_dma(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg);

int hsi_set_work_mode(enum hsi_work_mode_enum mode);
#endif
