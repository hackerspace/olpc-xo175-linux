/*
 * linux/sound/soc/pxa/mmp-squ-zsp.h
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.h
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Libin Yang <lbyang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef _MMP_SQU_ZSP_H
#define _MMP_SQU_ZSP_H

#include "mmp-zsp-audio.h"

#if 0
struct pxa910_squ_dma_params {
	char *name;			/* stream identifier */
	u32 dcmd;			/* DMA descriptor dcmd field */
	u32 dev_addr;		/* device physical address for DMA */
};
#endif

struct pxa688_adma_registers {
	u32 byte_counter;
	u32 src_addr;
	u32 dest_addr;
	u32 next_desc_ptr;
	u32 ctrl;
	u32 chan_pri;				/* Only used in channel 0 */
	u32 curr_desc_ptr;
	u32 intr_mask;
	u32 intr_status;
};

struct zsp_buffer {
	void *buf;
	dma_addr_t buf_phys;
	u32 zsp_offset;
	u32 app_offset;
	int buf_len;
	int zsp_period_bytes;
};

struct pxa910_runtime_data {
	int dma_ch;
	struct pxa3xx_pcm_dma_params *params;
	void *squ_desc_array;
	dma_addr_t squ_desc_array_phys;

	struct snd_pcm_substream *substream;

	struct pxa688_adma_registers pxa688_adma_saved[4];
	void *squ_desc_array_saved;

	void *zmq_deliver;
	int render_id;
	int stream_id;
	int fmbuf_len;
	struct zsp_buffer zsp_buf;
	struct workqueue_struct *zmq_workqueue;
	struct work_struct zsp_queue;
	struct completion zmq_cmd_completion;
	struct completion zmq_start_completion;
	struct completion zmq_stop_completion;
	adma_config_t zsp_adma_conf;
	int trigger_state;
	struct work_struct trigger_work_start;
	struct work_struct trigger_work_stop;
	struct completion zmq_closed;
	struct mutex trigger_mutex;
#define ZMQ_OPENED 1
#define	ZMQ_CLOSED 0
	int zmq_state;
	int zsp_triggered;
	int zsp_trigger_started;
	int zsp_trigger_stopped;
	atomic_t trigger_numbers;
	spinlock_t lock;
};

extern struct snd_soc_platform pxa910_soc_platform;

#endif
