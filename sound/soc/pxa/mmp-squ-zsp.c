/*
 * linux/sound/soc/pxa/pxa910-squ.c
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Libin Yang<lbyang@marvell.com>
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <linux/memblock.h>

#include <mach/regs-sspa.h>
#include <mach/pxa910-squ.h>
#include <mach/mmp_dma.h>
#include <mach/cputype.h>
#include "mmp-squ-zsp.h"
#include <plat/ssp.h>

#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/bootmem.h>

#define AUDIO_SRAM_START		(0xe0000000)
#define ABUF_MAX			(64 * 1024)
static const struct snd_pcm_hardware mmp2_a0_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 1*1024,
	.period_bytes_max	= 2*1024,
	.periods_min		= 1,
	.periods_max		= 12,
	.buffer_bytes_max	= ABUF_MAX,
	.fifo_size		= 32,
};

static DECLARE_WAIT_QUEUE_HEAD(dma_wq);

static unsigned long ab_phys, ab_size;
static int __init audiobuf_resv_setup(char *p)
{
	unsigned long size, start = 0xff00000;
	size = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);
	pr_info("Reserved audio buffer: %dM at %#x\n",
			(unsigned)size/0x100000, (unsigned)start);
	ab_phys = start;
	ab_size = size;
	memblock_remove(ab_phys, ab_size);
	return 1;
}
__setup("audiobuf_resv=", audiobuf_resv_setup);

static int mmp_zsp_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	int period_bytes, buffer_bytes;

	period_bytes = params_period_bytes(params);
	buffer_bytes = params_buffer_bytes(params);
	prtd->zsp_buf.zsp_period_bytes = period_bytes;

	if ((prtd->stream_id + 1) * ABUF_MAX > ab_size)
		return -ENOMEM;

	prtd->zsp_buf.buf_phys = ab_phys + prtd->stream_id * ABUF_MAX;
	/* remap the max audio buffer for the stream */
	prtd->zsp_buf.buf = ioremap(prtd->zsp_buf.buf_phys, ABUF_MAX);
	if (prtd->zsp_buf.buf == NULL)
		return -ENOMEM;

	prtd->zsp_buf.buf_len = buffer_bytes;

	/* setup adma conf */
	switch (params_format(params)) {
	case  SNDRV_PCM_FORMAT_S8:
		prtd->zsp_adma_conf.sample_size = 0x0;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		prtd->zsp_adma_conf.sample_size = 0x2;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		prtd->zsp_adma_conf.sample_size = 0x3;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		prtd->zsp_adma_conf.sample_size = 0x4;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		prtd->zsp_adma_conf.sample_size = 0x5;
		break;
	default:
		return -EINVAL;
	}

	prtd->zsp_adma_conf.pack_mode = ADMA_CONFIG_PACKMOD_PACK;
	runtime->dma_bytes = totsize;

	return 0;
}

static int mmp_zsp_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;

	if (prtd->zsp_buf.buf != NULL) {
		iounmap(prtd->zsp_buf.buf);
		prtd->zsp_buf.buf = NULL;
	}

	return 0;
}

static void zmq_send_cmd(struct pxa910_runtime_data *prtd,
			int cmdid, int trigger_type, int sync)
{
	struct snd_pcm_substream *substream;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *cpu_dai;
	struct ssp_device *sspa;
	void *pmsg;
	int len;
	cmd_audio_stream_acquire_t cmd_strm_acquire;
	cmd_audio_stream_prepare_t cmd_strm_pre;
	cmd_audio_stream_trigger_t cmd_strm_tri;
	cmd_audio_stream_release_t cmd_strm_release;
	cmd_audio_stream_position_t cmd_strm_pos;

	if (!prtd->zmq_deliver)
		return;
	switch (cmdid) {
	case AUDIO_STREAM_CMDID_ACQUIRE:
		cmd_strm_acquire.command_id = AUDIO_STREAM_CMDID_ACQUIRE;
		cmd_strm_acquire.stream_id = prtd->stream_id;
		pmsg = &cmd_strm_acquire;
		len = sizeof(cmd_audio_stream_acquire_t);
		break;
	case AUDIO_STREAM_CMDID_PREPARE:
		substream = prtd->substream;
		rtd = substream->private_data;
		cpu_dai = rtd->cpu_dai;
		sspa = snd_soc_dai_get_drvdata(cpu_dai);
		cmd_strm_pre.command_id = AUDIO_STREAM_CMDID_PREPARE;
		cmd_strm_pre.stream_id = prtd->stream_id;
		cmd_strm_pre.sspa_conf.fwid = sspa->zsp_sspa_conf.fwid;
		cmd_strm_pre.sspa_conf.fsync_active = \
			sspa->zsp_sspa_conf.fsync_active;
		cmd_strm_pre.sspa_conf.msl_select = \
			sspa->zsp_sspa_conf.msl_select;
		cmd_strm_pre.sspa_conf.fsp = sspa->zsp_sspa_conf.fsp;
		cmd_strm_pre.sspa_conf.ch_num = \
			sspa->zsp_sspa_conf.ch_num;
		cmd_strm_pre.sspa_conf.word_length = \
			sspa->zsp_sspa_conf.word_length;
		cmd_strm_pre.sspa_conf.sample_size = \
			sspa->zsp_sspa_conf.sample_size;
		cmd_strm_pre.sspa_conf.jst = sspa->zsp_sspa_conf.jst;
		cmd_strm_pre.sspa_conf.data_delay = \
			sspa->zsp_sspa_conf.data_delay;
		cmd_strm_pre.sspa_conf.sample_rate = \
			sspa->zsp_sspa_conf.sample_rate;
		cmd_strm_pre.sspa_conf.mclk = sspa->zsp_sspa_conf.mclk;
		cmd_strm_pre.adma_conf.sample_size = \
			prtd->zsp_adma_conf.sample_size;
		cmd_strm_pre.adma_conf.pack_mode = \
			prtd->zsp_adma_conf.pack_mode;
		cmd_strm_pre.adma_conf.fragment_size = 1536;
		pmsg = &cmd_strm_pre;
		len = sizeof(cmd_audio_stream_prepare_t);
		break;
	case AUDIO_STREAM_CMDID_RELEASE:
		cmd_strm_release.command_id = AUDIO_STREAM_CMDID_RELEASE;
		cmd_strm_release.stream_id = prtd->stream_id;
		atomic_inc(&prtd->trigger_numbers);
		pmsg = &cmd_strm_release;
		len = sizeof(cmd_audio_stream_release_t);
		break;
	case AUDIO_STREAM_CMDID_TRIGGER:
		if (trigger_type == AUDIO_STREAM_TRIGGER_START) {
			cmd_strm_tri.command_id = AUDIO_STREAM_CMDID_TRIGGER;
			cmd_strm_tri.stream_id = prtd->stream_id;
			cmd_strm_tri.trigger_type = AUDIO_STREAM_TRIGGER_START;
			cmd_strm_tri.int_type = AUDIO_STREAM_INT_TYPE_THRESHOLD;
			if (prtd->fmbuf_len < 2)
				return;
			else if (prtd->fmbuf_len < 10)
				cmd_strm_tri.int_threshold = \
					prtd->fmbuf_len - 1;
			else
				cmd_strm_tri.int_threshold = \
					prtd->fmbuf_len - 2;
			atomic_inc(&prtd->trigger_numbers);
		} else if (trigger_type == AUDIO_STREAM_TRIGGER_STOP) {
			cmd_strm_tri.command_id = AUDIO_STREAM_CMDID_TRIGGER;
			cmd_strm_tri.stream_id = prtd->stream_id;
			cmd_strm_tri.trigger_type = AUDIO_STREAM_TRIGGER_STOP;
			cmd_strm_tri.int_type = AUDIO_STREAM_STOP_AFTER_FLUSHED;
			atomic_inc(&prtd->trigger_numbers);
		} else
			return;
		pmsg = &cmd_strm_tri;
		len = sizeof(cmd_audio_stream_trigger_t);
		break;
	case AUDIO_STREAM_CMDID_POSITION:
		cmd_strm_pos.command_id = AUDIO_STREAM_CMDID_POSITION;
		cmd_strm_pos.stream_id = prtd->stream_id;
		pmsg = &cmd_strm_pos;
		len = sizeof(cmd_audio_stream_position_t);
		break;
	default:
		return;
	}
	if (sync)
		init_completion(&prtd->zmq_cmd_completion);
	kzmq_write(prtd->zmq_deliver, pmsg, len);
	if (sync)
		wait_for_completion(&prtd->zmq_cmd_completion);
	return;
}

static int mmp_zsp_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;
	unsigned long flags;

	/* in zsp mode, prepare will be handled in trigger hook */
	if (prtd->zsp_triggered) {
		if (!prtd->zsp_trigger_stopped) {
			init_completion(&prtd->zmq_stop_completion);
			wait_for_completion(&prtd->zmq_stop_completion);
		}
		spin_lock_irqsave(&prtd->lock, flags);
		while (atomic_read(&prtd->trigger_numbers) > 0) {
			init_completion(&prtd->zmq_cmd_completion);
			spin_unlock_irqrestore(&prtd->lock, flags);
			wait_for_completion(&prtd->zmq_cmd_completion);
			spin_lock_irqsave(&prtd->lock, flags);
		}
		spin_unlock_irqrestore(&prtd->lock, flags);
#ifdef CONFIG_WAKELOCK
		if (!has_wake_lock(WAKE_LOCK_SUSPEND)) {
			msleep(20);
			return -EPERM;
		}
#endif
		if (!prtd->zmq_deliver) {
			prtd->zmq_deliver = kzmq_open(prtd->render_id);
			if (!prtd->zmq_deliver)
				return -EPERM;
			prtd->zmq_state = ZMQ_OPENED;
			queue_work(prtd->zmq_workqueue,		\
				&prtd->zsp_queue);
			zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_ACQUIRE, 0, 1);
		}
		if (prtd->fmbuf_len == 0)
			zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_POSITION, 0, 1);
		zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_PREPARE, 0, 1);
	}

	prtd->zsp_buf.zsp_offset = 0;
	prtd->zsp_buf.app_offset = 0;

	return 0;
}

static void trigger_delay_work_stop(struct work_struct *work)
{
	struct pxa910_runtime_data *prtd;

	prtd = container_of(work, struct pxa910_runtime_data,
						trigger_work_stop);
	if (!prtd || !prtd->zmq_deliver)
		return;
	if (!prtd->zsp_trigger_started) {
		init_completion(&prtd->zmq_start_completion);
		wait_for_completion(&prtd->zmq_start_completion);
	}
	zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_TRIGGER,
				 AUDIO_STREAM_TRIGGER_STOP, 1);
	zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_RELEASE, 0, 1);
	if (prtd != NULL && prtd->zmq_deliver != NULL)
		prtd->zmq_state = ZMQ_CLOSED;
	prtd->zsp_trigger_stopped = 1;
	complete(&prtd->zmq_stop_completion);
}

static void trigger_delay_work_start(struct work_struct *work)
{
	struct pxa910_runtime_data *prtd;

	prtd = container_of(work, struct pxa910_runtime_data,
						trigger_work_start);
	if (prtd == NULL)
		return;
	if (!prtd->zmq_deliver) {
		prtd->zmq_deliver = kzmq_open(prtd->render_id);
		if (!prtd->zmq_deliver)
			return;
		prtd->zmq_state = ZMQ_OPENED;
		queue_work(prtd->zmq_workqueue,	\
			&prtd->zsp_queue);
		zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_ACQUIRE, 0, 1);
	}
	if (prtd->fmbuf_len == 0)
		zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_POSITION, 0, 1);
	zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_PREPARE, 0, 1);
	zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_TRIGGER,
				 AUDIO_STREAM_TRIGGER_START, 1);
	prtd->zsp_trigger_started = 1;
	complete(&prtd->zmq_start_completion);
}

static int mmp_zsp_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct pxa910_runtime_data *prtd = substream->runtime->private_data;

	prtd->trigger_state = cmd;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!prtd->zsp_triggered) {
			prtd->zsp_triggered = 1;
			schedule_work(&prtd->trigger_work_start);
		} else {
			zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_TRIGGER,
				AUDIO_STREAM_TRIGGER_START, 0);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!prtd->zsp_trigger_started) {
			schedule_work(&prtd->trigger_work_stop);
		} else {
			prtd->zsp_trigger_stopped = 1;
			zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_TRIGGER,
					AUDIO_STREAM_TRIGGER_STOP, 0);
			zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_RELEASE, 0, 0);
			if (prtd != NULL && prtd->zmq_deliver != NULL)
				prtd->zmq_state = ZMQ_CLOSED;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t
mmp_zsp_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;
	snd_pcm_uframes_t x;

	x = prtd->zsp_buf.zsp_offset;
	x = bytes_to_frames(runtime, x);
	if (x == prtd->zsp_buf.buf_len/4)
		x = 0;

	return x;
}

int get_zsp_buf_avail(struct zsp_buffer *buf)
{
	int len;

	if (buf->zsp_offset >= buf->app_offset)
		len = buf->buf_len - buf->zsp_offset;
	else
		len = buf->app_offset - buf->zsp_offset;

	return len;
}

int get_buf_avail(struct zsp_buffer *buf, int capture)
{
	int len;

	len = buf->app_offset - buf->zsp_offset;
	if (len <= 0)
		len += buf->buf_len;

	return len;
}

int get_fw_avail(cmd_audio_stream_datarxtxreq_t *datareq, int capture)
{
	int len;

	if (capture)
		len = datareq->write_index - datareq->read_index;
	else
		len = datareq->read_index - datareq->write_index;

	if (len <= 0)
		len += datareq->buffer_length;

	return len;
}

static void zsp_msg_handler(struct work_struct *work)
{
	struct pxa910_runtime_data *prtd;
	cmd_audio_stream_t zsp_data_received;
	cmd_audio_stream_datarxtx_t cmd_strm_tx;
	ack_audio_stream_release_t *p_zsp_release;
	ack_audio_stream_position_t *p_zsp_pos;
	struct snd_pcm_substream *substream;
	cmd_audio_stream_datarxtxreq_t *p_zsp_req;
	int len1, len2, len3, tsize;
	unsigned long flags;

	prtd = container_of(work, struct pxa910_runtime_data, zsp_queue);
	if (prtd == NULL)
		return;
	if (prtd->zmq_deliver == NULL)
		return;

	substream = prtd->substream;
	while (1) {
		kzmq_read((void *)prtd->zmq_deliver, (void *)&zsp_data_received,
				  sizeof(ack_audio_stream_t));
		switch (zsp_data_received.command_id) {
		case AUDIO_STREAM_CMDID_ACQUIRE:
			/* suppose it is OK, tbd later */
			complete(&prtd->zmq_cmd_completion);
			break;
		case AUDIO_STREAM_CMDID_PREPARE:
			/* tbd */
			complete(&prtd->zmq_cmd_completion);
			break;
		case AUDIO_STREAM_CMDID_TRIGGER:
			spin_lock_irqsave(&prtd->lock, flags);
			atomic_dec(&prtd->trigger_numbers);
			complete(&prtd->zmq_cmd_completion);
			spin_unlock_irqrestore(&prtd->lock, flags);
			break;
		case AUDIO_STREAM_CMDID_POSITION:
			p_zsp_pos = (ack_audio_stream_position_t *)	\
				&zsp_data_received;
			/* save the numbers of 1KB in firmware buf */
			prtd->fmbuf_len = p_zsp_pos->buffer_length / 1024;
			complete(&prtd->zmq_cmd_completion);
			break;
		case AUDIO_STREAM_CMDID_DATARXTXREQ:
			p_zsp_req = (cmd_audio_stream_datarxtxreq_t *)\
				&zsp_data_received;
			len1 = get_zsp_buf_avail(&prtd->zsp_buf);
			len2 = get_fw_avail(p_zsp_req, substream->stream);
			len3 = get_buf_avail(&prtd->zsp_buf, substream->stream);
			tsize = ((len1 < len2) ? len1 : len2);
			if ((tsize == len1) && (len1 == len3) && (tsize >= \
				2 * prtd->zsp_buf.zsp_period_bytes)) {
				tsize -= prtd->zsp_buf.zsp_period_bytes;
			}
			cmd_strm_tx.command_id = AUDIO_STREAM_CMDID_DATARXTX;
			cmd_strm_tx.stream_id = prtd->stream_id;
			cmd_strm_tx.addr = prtd->zsp_buf.buf_phys +	\
				prtd->zsp_buf.zsp_offset;
			cmd_strm_tx.size = tsize;
			kzmq_write(prtd->zmq_deliver, (void *)&cmd_strm_tx,
					   sizeof(cmd_audio_stream_datarxtx_t));
			if ((prtd->zsp_buf.zsp_offset + tsize) >= \
				(prtd->zsp_buf.buf_len))
				prtd->zsp_buf.zsp_offset = 0;
			else
				prtd->zsp_buf.zsp_offset += \
					tsize;
			break;

		case AUDIO_STREAM_CMDID_DATARXTX:
			/* tbd */
			snd_pcm_period_elapsed(substream);
			break;
		case AUDIO_STREAM_CMDID_RELEASE:
			p_zsp_release =
			  (ack_audio_stream_release_t *)&zsp_data_received;
			if (p_zsp_release->return_code == 0) {
				kzmq_close(prtd->zmq_deliver);
				prtd->zmq_deliver = NULL;
				spin_lock_irqsave(&prtd->lock, flags);
				atomic_set(&prtd->trigger_numbers, 0);
				complete(&prtd->zmq_cmd_completion);
				spin_unlock_irqrestore(&prtd->lock, flags);
				return;
			} else
				break;
		default:
			/* unknown command */
			break;
		}
	}

	return;
}

static int mmp_zsp_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &mmp2_a0_pcm_hardware);

	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct pxa910_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	prtd->substream = substream;
	prtd->zsp_buf.zsp_offset = 0;
	prtd->zsp_buf.app_offset = 0;
	prtd->fmbuf_len = 0;
	atomic_set(&prtd->trigger_numbers, 0);
	INIT_WORK(&prtd->zsp_queue, zsp_msg_handler);
	INIT_WORK(&prtd->trigger_work_start, trigger_delay_work_start);
	INIT_WORK(&prtd->trigger_work_stop, trigger_delay_work_stop);
	mutex_init(&prtd->trigger_mutex);
	runtime->private_data = prtd;
	prtd->dma_ch = -1;
	prtd->zsp_triggered = 0;
	prtd->zsp_trigger_started = 0;
	prtd->zsp_trigger_stopped = 0;
	prtd->zmq_workqueue = create_workqueue("zmq_wq");
	init_completion(&prtd->zmq_stop_completion);
	init_completion(&prtd->zmq_start_completion);

	return 0;

 out:
	return ret;
}

static int mmp_zsp_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;
	unsigned long flags;

	cancel_work_sync(&prtd->trigger_work_start);
	cancel_work_sync(&prtd->trigger_work_stop);
	spin_lock_irqsave(&prtd->lock, flags);
	while (atomic_read(&prtd->trigger_numbers) > 0) {
		init_completion(&prtd->zmq_cmd_completion);
		spin_unlock_irqrestore(&prtd->lock, flags);
		wait_for_completion(&prtd->zmq_cmd_completion);
		spin_lock_irqsave(&prtd->lock, flags);
	}
	spin_unlock_irqrestore(&prtd->lock, flags);
	if (prtd->zsp_triggered && prtd->zmq_deliver) {
		zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_TRIGGER,
					 AUDIO_STREAM_TRIGGER_STOP, 1);
		zmq_send_cmd(prtd, AUDIO_STREAM_CMDID_RELEASE, 0, 1);
		if (prtd != NULL && prtd->zmq_deliver != NULL)
			prtd->zmq_state = ZMQ_CLOSED;
	}
	if (prtd != NULL && prtd->zmq_workqueue != NULL)
		destroy_workqueue(prtd->zmq_workqueue);
	iounmap(prtd->squ_desc_array);
	prtd->zsp_buf.buf_phys = (dma_addr_t)0;

	kfree(prtd);
	runtime->private_data = NULL;
	return 0;
}

static int mmp_zsp_pcm_copy(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos,
				void *buf, snd_pcm_uframes_t count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (copy_from_user(prtd->zsp_buf.buf + \
				prtd->zsp_buf.app_offset,
				buf, frames_to_bytes(runtime, count)))
			return -EFAULT;
		if ((prtd->zsp_buf.app_offset + \
			 frames_to_bytes(runtime, count)) >= \
			prtd->zsp_buf.buf_len)
			prtd->zsp_buf.app_offset = 0;
		else
			prtd->zsp_buf.app_offset +=\
				frames_to_bytes(runtime, count);
	} else {
		if (copy_to_user(buf, prtd->zsp_buf.buf +
				prtd->zsp_buf.app_offset,
				frames_to_bytes(runtime, count)))
			return -EFAULT;
		if ((prtd->zsp_buf.app_offset + \
			 frames_to_bytes(runtime, count)) >= \
			prtd->zsp_buf.buf_len)
			prtd->zsp_buf.app_offset = 0;
		else
			prtd->zsp_buf.app_offset += \
				frames_to_bytes(runtime, count);
	}

	return 0;
}

static int mmp_zsp_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;
	unsigned long off = vma->vm_pgoff;
	int ret;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = remap_pfn_range(vma, vma->vm_start,
			__phys_to_pfn(prtd->zsp_buf.buf_phys) + off,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
	return ret;
}

struct snd_pcm_ops mmp_zsp_pcm_ops = {
	.open		= mmp_zsp_pcm_open,
	.close		= mmp_zsp_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mmp_zsp_pcm_hw_params,
	.hw_free	= mmp_zsp_pcm_hw_free,
	.prepare	= mmp_zsp_pcm_prepare,
	.trigger	= mmp_zsp_pcm_trigger,
	.pointer	= mmp_zsp_pcm_pointer,
	.copy		= mmp_zsp_pcm_copy,
	.mmap		= mmp_zsp_pcm_mmap,
};

static void mmp_zsp_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		struct snd_soc_pcm_runtime *rtd;

		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		rtd = substream->private_data;
	}
}

static u64 mmp_zsp_pcm_dmamask = DMA_BIT_MASK(64);

int mmp_zsp_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &mmp_zsp_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(64);

	return 0;
}

struct snd_soc_platform_driver mmp_zsp_soc_platform = {
	.ops = &mmp_zsp_pcm_ops,
	.pcm_new = mmp_zsp_pcm_new,
	.pcm_free = mmp_zsp_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(mmp_zsp_soc_platform);

static int __devinit mmp_zsp_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &mmp_zsp_soc_platform);
}

static int __devexit mmp_zsp_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver mmp_zsp_pcm_driver = {
	.driver = {
		.name = "mmp-zsp-pcm-audio",
		.owner = THIS_MODULE,
	},
	.probe = mmp_zsp_soc_platform_probe,
	.remove = __devexit_p(mmp_zsp_soc_platform_remove),
};

static int __init snd_mmp2_zsp_pcm_init(void)
{
	return platform_driver_register(&mmp_zsp_pcm_driver);
}
module_init(snd_mmp2_zsp_pcm_init);

static void __exit snd_mmp2_zsp_pcm_exit(void)
{
	platform_driver_unregister(&mmp_zsp_pcm_driver);
}
module_exit(snd_mmp2_zsp_pcm_exit);

MODULE_AUTHOR("lbyang@marvell.com");
MODULE_DESCRIPTION("MMP SQU DMA for ZSP module");
MODULE_LICENSE("GPL");
