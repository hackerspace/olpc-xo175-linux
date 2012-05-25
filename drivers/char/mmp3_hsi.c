/*
 * mmp3_hsi.c
 *
 * Marvell HSI - Physical Layer Driver
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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/time.h>

#include <linux/spinlock.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/system.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <mach/memory.h>
#include <mach/hsi_dev.h>


#include <asm/mach-types.h>

#include "mmp3_hsi.h"

#define HSI_DEBUG		0
#define FRAME_PER_FIFO	2
#define MSEC_TO_US	1000
#define SEC_TO_US	1000000


static struct mmp3_hsi_core *hsi_core;

static struct hsi_cfg_strt hsi_config = {
	.ch_bits = HSI_IF_CHANNEL_BITS_2,
	.data_flow = HSI_IF_DATA_FLOW_SYNCHRONIZED,
	.trans_mode = HSI_IF_OP_MODE_FRAME,
	.ready_mode = HSI_READY_MODE_V1_0,
	.tx_wake_impl = HSI_TX_WAKE_NOT_IMPL,
	.rx_wake_impl = HSI_RX_WAKE_NOT_IMPL,
	.tltc_pre_scaler = 0x0,
	.tx_rate_div = 0x0,
	.rx_rate_div = 0x0,
	.fto_cnt_ena = FTO_DISABLED,
	.fto_cnt = 0,
	.tx_arb_mode = HSI_IF_ARB_ROUND_ROBIN,
	.rx_arb_mode = HSI_IF_ARB_ROUND_ROBIN,
	.rx_brk_handler = NULL,
	.brk_cfg = {
		.brk_rcv_clr = false,
		.brk_fifo_clr = false,
		.brk_cnt = 0x25,
	},
	.mode = HSI_CPIMAGE_MODE,
};

static bool read_data_from_fifo(u32 frame_count, int channel)
{
	u32 frame_data, frame_head, frame_channel, cmd_ack = 0;
	bool ret = true;
	struct strt_hsi_rx_ch_cfg *rx_cfg;

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);

	while (frame_count != 0) {
		frame_data = HSI_READ32(RX_FIFO + (channel << 3));
		frame_head = frame_data >> 28;
		/* reply ACK if it's open connection */
		if (frame_head == 0x07) {
			frame_channel = (frame_data >> 24) & 0xf;
			if (frame_channel > HSI_CH_NUM) {
				pr_err("channel number out of range: %d\n",
						frame_channel);
				ret = false;
				break;
			}

			cmd_ack = (0xb << 28) | (frame_channel << 24);
			/* send ACK */
			HSI_WRITE32(TX_FIFO + (channel << 3), cmd_ack);
		} else if (frame_head == 0x0b) {
		/* ACK is processed by IPC automatically */
			if (hsi_core->hsi_config->mode == HSI_CPIMAGE_MODE) {
				pr_err("%s: IPC err.\n", __func__);
				continue;
			}
		}

		if (rx_cfg->rx_cpu_handler)
			rx_cfg->rx_cpu_handler(channel, frame_data);

		if (rx_cfg->rx_frame == NULL) {
			frame_count--;
			continue;
		}

		if (rx_cfg->rx_frame_rcv < rx_cfg->rx_frame_cnt) {
			*rx_cfg->rx_frame = frame_data;
			rx_cfg->rx_frame++;
			rx_cfg->rx_frame_rcv++;
			if (rx_cfg->rx_frame_rcv == rx_cfg->rx_frame_cnt) {
				hsi_disable_rx_fifo_req(channel, rx_cfg);
				complete_all(&rx_cfg->rx_thrs_event);
				ret = false;
				break;
			}
		} else {
			dev_err(hsi_core->dev, "rx pio err\n");
			hsi_disable_rx_fifo_req(channel, rx_cfg);
			complete_all(&rx_cfg->rx_thrs_event);
			ret = false;
			break;
		}
		frame_count--;
	}

	return ret;
}

static int hsi_handle_rx_thrs_int(int channel)
{
	u32 frame_count;
	struct strt_hsi_rx_ch_cfg *rx_cfg;
	bool loop;

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);

	if (rx_cfg->rx_frame == NULL && rx_cfg->rx_cpu_handler == NULL)
		hsi_disable_rx_fifo_req(channel, rx_cfg);

	frame_count = HSI_READ32(CHNL_FIFO_STATUS + (channel << 2));
	frame_count = HSI_GET32(frame_count,
		BIT_RX_FRM_CNT_MSB, BIT_RX_FRM_CNT_LSB);
	while (frame_count != 0) {
		loop = read_data_from_fifo(frame_count, channel);
		if (loop == false)
			break;
		frame_count = HSI_GET32(
			HSI_READ32(CHNL_FIFO_STATUS + (channel << 2)),
			BIT_RX_FRM_CNT_MSB, BIT_RX_FRM_CNT_LSB);
	}

	return 0;
}

static int hsi_ipc_cmd_rcv_handler(int channel)
{
	/* do nothing here */
	return 0;
}

static int hsi_dma_rx_done_handler(int channel)
{
	struct strt_hsi_rx_ch_cfg *rx_cfg;

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);

	hsi_disable_rx_fifo_req(channel, rx_cfg);
	hsi_disable_rx_fifo_int(channel, rx_cfg);

	rx_cfg->rx_busy = false;

	if (rx_cfg->rx_dma_handler)
		rx_cfg->rx_dma_handler(channel, rx_cfg->dma_cookie);

	return 0;
}

static int hsi_dma_rx_err_handler(int channel)
{
	struct strt_hsi_rx_ch_cfg *rx_cfg;

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);

	rx_cfg->dma_err = true;

	dev_err(hsi_core->dev, "rx dma err\n");

	return 0;
}

static int hsi_rx_frm_lost_handler(int channel)
{
	dev_warn(hsi_core->dev, "rx frame lost err\n");

	return 0;
}

static void hsi_bootrom_delay(int time_us)
{
	struct timeval tv_tick;
	long end_secs, end_usecs;
	long start_secs, start_usecs;
	long total_secs, total_usecs;

	do_gettimeofday(&tv_tick);
	start_secs = tv_tick.tv_sec;
	start_usecs = tv_tick.tv_usec;

	do {
		do_gettimeofday(&tv_tick);
		end_secs = tv_tick.tv_sec;
		end_usecs = tv_tick.tv_usec;

		total_secs = (end_secs - start_secs);
		if (total_secs < 0)
			total_secs = 0;
		total_usecs = (end_usecs - start_usecs);
		if (total_usecs < 0)
			total_usecs = 0;
	} while ((total_secs * SEC_TO_US + total_usecs) < time_us);

	return;
}

static int hsi_handle_tx_thrs_int(int channel)
{
	struct strt_hsi_tx_ch_cfg *tx_cfg;
	int burst_size, burst_cnt;
	u32 frame_num;

	tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[channel]);

	if (tx_cfg->tx_frame == NULL) {
		hsi_disable_tx_fifo_req(channel, tx_cfg);
		return 0;
	}

	switch (hsi_core->hsi_config->mode) {
	case HSI_BOOTROM_MODE:
		hsi_bootrom_delay(1 * MSEC_TO_US);
		break;
	case HSI_CPIMAGE_MODE:
		break;
	default:
		break;
	}

	frame_num = HSI_READ32(CHNL_FIFO_STATUS + (channel << 2));
	frame_num = HSI_GET32(frame_num,
		BIT_TX_FRM_CNT_MSB, BIT_TX_FRM_CNT_LSB);

	burst_size = tx_cfg->tx_ch_fifo_size * FRAME_PER_FIFO - frame_num;
	burst_cnt = burst_size;
	while (burst_cnt && tx_cfg->tx_frame_cnt) {
		HSI_WRITE32(TX_FIFO + (channel << 3), *(tx_cfg->tx_frame++));
		burst_cnt--;
		tx_cfg->tx_frame_sent++;
		tx_cfg->tx_frame_cnt--;
	}

	if (tx_cfg->tx_frame_cnt == 0) {
		hsi_disable_tx_fifo_req(channel, tx_cfg);
		complete_all(&tx_cfg->tx_thrs_event);
	}

	return 0;
}

static int hsi_dma_tx_err_handler(int channel)
{
	struct strt_hsi_tx_ch_cfg *tx_cfg;

	tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[channel]);

	tx_cfg->dma_err = true;

	dev_err(hsi_core->dev, "tx dma err\n");

	return 0;
}

/* we don't need dma tx done and don't notify data link driver.
 * only enable for debug purpose */
static int hsi_dma_tx_done_handler(int channel)
{
#if HSI_DEBUG
	struct strt_hsi_tx_ch_cfg *tx_cfg;

	tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[channel]);

	hsi_disable_tx_fifo_req(channel, tx_cfg);
	hsi_disable_tx_fifo_int(channel, tx_cfg);

	tx_cfg->tx_busy = false;

	if (tx_cfg->tx_dma_handler)
		tx_cfg->tx_dma_handler(channel, tx_cfg->dma_cookie);
#endif

	return 0;
}

static int hsi_channel_int_handler(int channel)
{
	int ret = 0;
	u32 channel_status;


	if (channel > HSI_CH_NUM - 1) {
		ret = -1;
		goto exit;
	}

	channel_status = HSI_READ32(CHNL_STATUS + (channel << 2));

	if (channel_status & HSI_CH_STATUS_TX_FIFO_THRS)
		ret = hsi_handle_tx_thrs_int(channel);
	if (channel_status & HSI_CH_STATUS_TX_DMA_DONE)
		ret = hsi_dma_tx_done_handler(channel);
	if (channel_status & HSI_CH_STATUS_TX_DMA_ERR)
		ret = hsi_dma_tx_err_handler(channel);
	if (channel_status & HSI_CH_STATUS_RX_FIFO_THRS)
		ret = hsi_handle_rx_thrs_int(channel);
	if (channel_status & HSI_CH_STATUS_RX_DMA_DONE)
		ret = hsi_dma_rx_done_handler(channel);
	if (channel_status & HSI_CH_STATUS_RX_DMA_ERR)
		ret = hsi_dma_rx_err_handler(channel);
	if (channel_status & HSI_CH_STATUS_RX_FRM_LOST)
		ret = hsi_rx_frm_lost_handler(channel);
	if (channel_status & HSI_CH_STATUS_IPC_CMD)
		ret = hsi_ipc_cmd_rcv_handler(channel);

exit:
	channel_status &= HSI_CH_STATUS_CLR_ALL_MASK;
	HSI_WRITE32(CHNL_STATUS + (channel << 2), channel_status);

	return ret;
}

static irqreturn_t hsi_interrupt_handler(int irq, void *dev_id)
{
	u32 hsi_status;
	int ch;
	unsigned long flags;

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);

	hsi_status = HSI_READ32(HSI_INT_STATUS0);

	for (ch = HSI_CHNL_INT_0; ch <= HSI_CHNL_INT_1; ch++) {
		if ((hsi_status & (0x1 << ch)) != 0)
			hsi_channel_int_handler(ch - HSI_CHNL_INT_0);
	}

	if (hsi_status & 0x1) {
		/* Receiver Break Received */
		if (hsi_core->hsi_config->rx_brk_handler != NULL)
			hsi_core->hsi_config->rx_brk_handler();
	}

	HSI_WRITE32(HSI_INT_STATUS0, (hsi_status & HSI_CH_MSK_ALL));
	HSI_WRITE32(HSI_INT_STATUS1, (hsi_status & HSI_CH_MSK_ALL));
	HSI_WRITE32(HSI_INT_STATUS2, (hsi_status & HSI_CH_MSK_ALL));
	HSI_WRITE32(HSI_INT_STATUS3, (hsi_status & HSI_CH_MSK_ALL));

	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	return IRQ_HANDLED;
}

long hsi_ioctl(struct file *fp, unsigned int cmd, unsigned long param)
{
	return 0;
}


static const struct file_operations hsi_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= hsi_ioctl,
};


static struct miscdevice hsi_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "hsi",
	.fops		= &hsi_fops,
};

static int mmp3_hsi_reset_modules(void)
{
	u32 reg;

	reg = HSI_READ32(HSI_CNTRL);
	reg |= 0x1E;
	HSI_WRITE32(HSI_CNTRL, reg);
	reg = HSI_READ32(HSI_CNTRL);
	reg &= ~0x1E;
	HSI_WRITE32(HSI_CNTRL, reg);
	reg = HSI_READ32(HSI_CNTRL);

	return 0;
}

static int mmp3_hsi_config_hw(struct hsi_cfg_strt *hsi_config)
{
	u32 reg;
	int ch;
	int ret = 0;
	struct hsi_cfg_strt *hsi_setting;
	struct strt_hsi_tltc_chnl_cfg *tltc_cfg;
	struct strt_hsi_tx_ch_cfg *tx_cfg;
	struct strt_hsi_rx_ch_cfg *rx_cfg;

	HSI_LOCK(0);

	mmp3_hsi_reset_modules();

	hsi_setting = hsi_core->hsi_config;

	/* Set up configuration 1 register */
	reg = 0;
	reg = HSI_MODIFY32(reg, 31, 24, hsi_setting->tltc_pre_scaler);
	switch (hsi_setting->ch_bits) {
	case HSI_IF_CHANNEL_BITS_0:
		reg = HSI_MODIFY32(reg, 18, 16, 0);
		break;
	case HSI_IF_CHANNEL_BITS_1:
		reg = HSI_MODIFY32(reg, 18, 16, 1);
		break;
	case HSI_IF_CHANNEL_BITS_2:
		reg = HSI_MODIFY32(reg, 18, 16, 2);
		break;
	case HSI_IF_CHANNEL_BITS_3:
		reg = HSI_MODIFY32(reg, 18, 16, 3);
		break;
	case HSI_IF_CHANNEL_BITS_4:
		reg = HSI_MODIFY32(reg, 18, 16, 4);
		break;
	default:
		ret = -ENXIO;
		goto error;
	}
	switch (hsi_setting->data_flow) {
	case HSI_IF_DATA_FLOW_SYNCHRONIZED:
		reg = HSI_MODIFY32(reg, 11, 10, 0);
		break;
	case HSI_IF_DATA_FLOW_PIPELINED:
		reg = HSI_MODIFY32(reg, 11, 10, 1);
		break;
	case HSI_IF_DATA_FLOW_REALTIME:
		reg = HSI_MODIFY32(reg, 11, 10, 2);
		break;
	case HSI_IF_DATA_FLOW_DUMMY:
		reg = HSI_MODIFY32(reg, 11, 10, 3);
		break;
	default:
		ret = -ENXIO;
		goto error;
	}
	switch (hsi_setting->trans_mode) {
	case HSI_IF_OP_MODE_STREAM:
		reg = HSI_MODIFY32(reg, 8, 8, 0);
		break;
	case HSI_IF_OP_MODE_FRAME:
		reg = HSI_MODIFY32(reg, 8, 8, 1);
		break;
	default:
		ret = -ENXIO;
		goto error;
	}
	switch (hsi_setting->ready_mode) {
	case HSI_READY_MODE_V1_1:
		reg = HSI_MODIFY32(reg, 2, 2, 0);
		break;
	case HSI_READY_MODE_V1_0:
		reg = HSI_MODIFY32(reg, 2, 2, 1);
		break;
	default:
		ret = -ENXIO;
		goto error;
	}
	switch (hsi_setting->tx_wake_impl) {
	case HSI_TX_WAKE_NOT_IMPL:
		reg = HSI_MODIFY32(reg, 1, 1, 0);
		break;
	case HSI_TX_WAKE_IMPL:
		reg = HSI_MODIFY32(reg, 1, 1, 1);
		break;
	default:
		ret = -ENXIO;
		goto error;
	}
	switch (hsi_setting->rx_wake_impl) {
	case HSI_RX_WAKE_NOT_IMPL:
		reg = HSI_MODIFY32(reg, 0, 0, 0);
		break;
	case HSI_RX_WAKE_IMPL:
		reg = HSI_MODIFY32(reg, 0, 0, 1);
		break;
	default:
		ret = -ENXIO;
		goto error;
	}
	HSI_WRITE32(HSI_CONFIG1, reg);

	/* Set up configuration 2 register */
	reg = 0;
	reg = HSI_MODIFY32(reg, 31, 16, hsi_setting->rx_rate_div);
	reg = HSI_MODIFY32(reg, 15, 0, hsi_setting->tx_rate_div);
	HSI_WRITE32(HSI_CONFIG2, reg);

	/* Set up break configuration register */
	reg = 0;
	if (hsi_setting->brk_cfg.brk_rcv_clr)
		reg = HSI_MODIFY32(reg, 18, 18, 1);
	else
		reg = HSI_MODIFY32(reg, 18, 18, 0);
	if (hsi_setting->brk_cfg.brk_fifo_clr)
		reg = HSI_MODIFY32(reg, 17, 17, 1);
	else
		reg = HSI_MODIFY32(reg, 17, 17, 0);
	reg = HSI_MODIFY32(reg, 5, 0, hsi_setting->brk_cfg.brk_cnt);
	HSI_WRITE32(HSI_BRK_CONFIG, reg);

	/* Set up burst configuration register */
	reg = 0;
	reg = HSI_MODIFY32(reg, 16, 16, 0);
	reg = HSI_MODIFY32(reg, 7, 0, 0);
	HSI_WRITE32(HSI_BURST_CONFIG, reg);

	/* Set up frame timeout configuration register */
	reg = 0;
	if (hsi_setting->fto_cnt_ena)
		reg = HSI_MODIFY32(reg, 16, 16, 1);
	else
		reg = HSI_MODIFY32(reg, 16, 16, 0);
	reg = HSI_MODIFY32(reg, 15, 0, hsi_setting->fto_cnt);
	HSI_WRITE32(HSI_FTO_CONFIG, reg);

	/* Set up tltc configuration register */
	for (ch = 0; ch < HSI_TLTC_CH_NUM; ch++) {
		tltc_cfg = &(hsi_core->hsi_config->tltc_ch_cfg[ch]);
		reg = 0;
		reg = HSI_MODIFY32(reg, 31, 16, tltc_cfg->tltc_cnt);
		switch (tltc_cfg->tltc_mode) {
		case TLTC_MODE_BURST:
			reg = HSI_MODIFY32(reg, 1, 1, 0);
			break;
		case TLTC_MODE_TIMEOUT:
			reg = HSI_MODIFY32(reg, 1, 1, 1);
			break;
		default:
			ret = -EINVAL;
			goto error;
		}
		if (tltc_cfg->tltc_enable)
			reg = HSI_MODIFY32(reg, 0, 0, 1);
		else
			reg = HSI_MODIFY32(reg, 0, 0, 0);
		HSI_WRITE32(TLTC_CONFIG + (ch << 2), reg);
	}

	/* Set up RX FIFO configuration register */
	for (ch = 0; ch < HSI_RX_CH_NUM; ch++) {
		rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[ch]);
		reg = 0;
		reg = HSI_MODIFY32(reg, 23, 18, rx_cfg->rx_ch_fifo_thrs);
		reg = HSI_MODIFY32(reg, 17, 12, rx_cfg->rx_ch_fifo_size);
		reg = HSI_MODIFY32(reg, 31, 24, rx_cfg->rx_ch_fifo_base);
		switch (rx_cfg->rx_fifo_valve) {
		case HSI_RECV_FIFO_VALVE_DISABLE:
			reg = HSI_MODIFY32(reg, 3, 3, 0);
			break;
		case HSI_RECV_FIFO_VALVE_ENABLE:
			reg = HSI_MODIFY32(reg, 3, 3, 1);
			break;
		default:
			ret = -EINVAL;
			goto error;
		}
		switch (rx_cfg->rx_fifo_req_sel) {
		case HSI_FIFO_REQ_INT:
			reg = HSI_MODIFY32(reg, 2, 2, 0);
			break;
		case HSI_FIFO_REQ_DMA:
			reg = HSI_MODIFY32(reg, 2, 2, 1);
			break;
		default:
			ret = -ENXIO;
			goto error;
		}

		if (rx_cfg->rx_fifo_req_ena)
			reg = HSI_MODIFY32(reg, 1, 1, 1);
		else
			reg = HSI_MODIFY32(reg, 1, 1, 0);

		if (rx_cfg->rx_fifo_enable)
			reg = HSI_MODIFY32(reg, 0, 0, 1);
		else
			reg = HSI_MODIFY32(reg, 0, 0, 0);

		HSI_WRITE32(RX_FIFO_CONFIG + (ch << 2), reg);
	}

	/* Set up TX FIFO configuration register */
	for (ch = 0; ch < HSI_TX_CH_NUM; ch++) {
		tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[ch]);
		reg = 0;
		reg = HSI_MODIFY32(reg, 23, 18, tx_cfg->tx_ch_fifo_thrs);
		reg = HSI_MODIFY32(reg, 17, 12, tx_cfg->tx_ch_fifo_size);
		reg = HSI_MODIFY32(reg, 31, 24, tx_cfg->tx_ch_fifo_base);
		reg = HSI_MODIFY32(reg, 4, 3, tx_cfg->tx_ch_priority);
		switch (tx_cfg->tx_fifo_req_sel) {
		case HSI_FIFO_REQ_INT:
			reg = HSI_MODIFY32(reg, 2, 2, 0);
			break;
		case HSI_FIFO_REQ_DMA:
			reg = HSI_MODIFY32(reg, 2, 2, 1);
			break;
		default:
			ret = -ENXIO;
			goto error;
		}

		if (tx_cfg->tx_fifo_req_ena)
			reg = HSI_MODIFY32(reg, 1, 1, 1);
		else
			reg = HSI_MODIFY32(reg, 1, 1, 0);

		if (tx_cfg->tx_fifo_enable)
			reg = HSI_MODIFY32(reg, 0, 0, 1);
		else
			reg = HSI_MODIFY32(reg, 0, 0, 0);
		HSI_WRITE32(TX_FIFO_CONFIG + (ch << 2), reg);
	}

	/* Set up interrupt mask register for general INT */
	reg = 0;
	reg = HSI_MODIFY32(reg, 2, 2, 1);
	reg = HSI_MODIFY32(reg, 1, 1, 1);
	reg = HSI_MODIFY32(reg, 0, 0, 1);
	HSI_WRITE32(HSI_INT_MASK0, reg);
	HSI_WRITE32(HSI_INT_MASK1, reg);
	HSI_WRITE32(HSI_INT_MASK2, reg);
	HSI_WRITE32(HSI_INT_MASK3, reg);

error:
	HSI_UNLOCK(0);
	return ret;
}

int mmp3_hsi_enable_hw(void)
{
	HSI_WRITE32(HSI_ENABLE0, 0x1);
	HSI_WRITE32(HSI_ENABLE1, 0x1);
	HSI_WRITE32(HSI_ENABLE2, 0x1);
	HSI_WRITE32(HSI_ENABLE3, 0x1);

	return 0;
}
EXPORT_SYMBOL(mmp3_hsi_enable_hw);

int mmp3_hsi_disable_hw(void)
{
	HSI_WRITE32(HSI_ENABLE0, 0x0);
	HSI_WRITE32(HSI_ENABLE1, 0x0);
	HSI_WRITE32(HSI_ENABLE2, 0x0);
	HSI_WRITE32(HSI_ENABLE3, 0x0);

	return 0;
}
EXPORT_SYMBOL(mmp3_hsi_disable_hw);

static int mmp3_hsi_init_database(void)
{
	int ch;
	struct strt_hsi_tltc_chnl_cfg *tltc_cfg;
	struct strt_hsi_tx_ch_cfg *tx_config;
	struct strt_hsi_rx_ch_cfg *rx_config;

	hsi_core->hsi_config = &hsi_config;
	for (ch = 0; ch < HSI_TLTC_CH_NUM; ch++) {
		tltc_cfg = &hsi_config.tltc_ch_cfg[ch];
		tltc_cfg->tltc_cnt = 0;
		tltc_cfg->tltc_enable = false;
		tltc_cfg->tltc_mode = TLTC_MODE_TIMEOUT;
	}
	for (ch = 0; ch < HSI_TX_CH_NUM; ch++) {
		tx_config = &hsi_config.tx_chnl_config[ch];
		tx_config->tx_busy = false;
		tx_config->tx_fifo_req_ena = false;
		tx_config->tx_fifo_int_ena = false;
		tx_config->tx_fifo_enable = false;
		tx_config->tx_ch_fifo_base = 0;
		tx_config->tx_ch_fifo_size = 0;
		tx_config->tx_ch_fifo_thrs = 0;
		tx_config->tx_ch_priority = 0;
		tx_config->tx_fifo_req_sel = HSI_FIFO_REQ_INT;
		tx_config->tx_chain_mode_ptr = NULL;
		tx_config->dma_cookie = NULL;
		tx_config->tx_dma_handler = NULL;
		tx_config->tx_frame_cnt = 0;
		tx_config->tx_frame_sent = 0;
		tx_config->tx_frame = NULL;
		init_completion(&(tx_config->tx_thrs_event));
	}
	tx_config = &hsi_config.tx_chnl_config[CMD_CH];
	tx_config->tx_ch_fifo_base = 0;
	tx_config->tx_ch_fifo_size = 32;
	tx_config->tx_ch_fifo_thrs = 1;
	tx_config->tx_ch_priority = 1;
	tx_config = &hsi_config.tx_chnl_config[DATA_CH];
	tx_config->tx_ch_fifo_base = 64;
	tx_config->tx_ch_fifo_size = 32;
	tx_config->tx_ch_fifo_thrs = 32;
	tx_config->tx_ch_priority = 1;

	for (ch = 0; ch < HSI_RX_CH_NUM; ch++) {
		rx_config = &hsi_config.rx_chnl_config[ch];
		rx_config->rx_busy = false;
		rx_config->rx_fifo_req_ena = false;
		rx_config->rx_fifo_int_ena = false;
		rx_config->rx_fifo_enable = false;
		rx_config->ipc_cmd_rcv_ena = false;
		rx_config->rx_ch_fifo_base = 0;
		rx_config->rx_ch_fifo_size = 0;
		rx_config->rx_ch_fifo_thrs = 0;
		rx_config->rx_fifo_valve = HSI_RECV_FIFO_VALVE_ENABLE;
		rx_config->rx_fifo_req_sel = HSI_FIFO_REQ_INT;
		rx_config->rx_chain_mode_ptr = NULL;
		rx_config->dma_cookie = NULL;
		rx_config->rx_dma_handler = NULL;
		rx_config->rx_frame_cnt = 0;
		rx_config->rx_frame_rcv = 0;
		rx_config->rx_frame = NULL;
		init_completion(&(rx_config->rx_thrs_event));
	}
	rx_config = &hsi_config.rx_chnl_config[CMD_CH];
	rx_config->rx_ch_fifo_base = 0;
	rx_config->rx_ch_fifo_size = 32;
	rx_config->rx_ch_fifo_thrs = 1;
	rx_config = &hsi_config.rx_chnl_config[DATA_CH];
	rx_config->rx_ch_fifo_base = 64;
	rx_config->rx_ch_fifo_size = 32;
	rx_config->rx_ch_fifo_thrs = 1;

	return 0;
}

static int __devinit mmp3_hsi_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct resource *res;
	int size;
	int ret = -EINVAL, irq;

	hsi_core = kzalloc(sizeof(struct mmp3_hsi_core), GFP_KERNEL);
	if (hsi_core == NULL) {
		dev_err(&pdev->dev, "core failed\n");
		ret = -ENOMEM;
		goto failed_alloc_config;
	}

	hsi_core->dev = &pdev->dev;
	hsi_core->plat_data = pdev->dev.platform_data;
	/* Get HSI clocks */
	clk = clk_get(hsi_core->dev, "hsi-clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "clk_get failed\n");
		ret = -EINVAL;
		goto failed_clk;
	}
	hsi_core->clk = clk;

	/* Get HSI function register base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size = res->end - res->start + 1;
	hsi_core->mem_res_hsi =
		request_mem_region(res->start, size, pdev->name);
	if (hsi_core->mem_res_hsi == NULL) {
		dev_err(&pdev->dev, "failed to request register memory\n");
		ret = -EBUSY;
		goto failed_req_mem;
	}
	hsi_core->mmio_size = size;
	hsi_core->mmio_hsi_base =
		ioremap_nocache(hsi_core->mem_res_hsi->start, size);
	if (hsi_core->mmio_hsi_base == NULL) {
		release_mem_region(hsi_core->mem_res_hsi->start, size);
		hsi_core->mem_res_hsi = NULL;
		dev_err(&pdev->dev, "failed to ioremap registers\n");
		ret = -ENXIO;
		goto failed_remap;
	}

	/* Get HSI IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq >= 0) {
		ret = request_irq(irq, hsi_interrupt_handler,
			IRQF_DISABLED, "mmp3-hsi", hsi_core);
		disable_irq(irq);
	}
	if (irq < 0 || ret < 0) {
		ret = -ENXIO;
		goto failed_getirqs;
	}
	hsi_core->irq = irq;

	spin_lock_init(&hsi_core->s_irqlock);
	mutex_init(&(hsi_core->hsi_mutex));
	mmp3_hsi_init_database();
	platform_set_drvdata(pdev, hsi_core);

	ret = clk_enable(hsi_core->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk_enable hsi failed\n");
		goto failed_clk_on;
	}

	mmp3_hsi_config_hw(hsi_core->hsi_config);
	if (hsi_core->plat_data && hsi_core->plat_data->hsi_config_int)
		hsi_core->plat_data->hsi_config_int(NULL);
	enable_irq(irq);

	mmp3_hsi_enable_hw();

	return 0;

failed_clk_on:
failed_getirqs:
	iounmap(hsi_core->mmio_hsi_base);
failed_remap:
	release_mem_region(hsi_core->mem_res_hsi->start, hsi_core->mmio_size);
failed_req_mem:
	clk_put(hsi_core->clk);
failed_clk:
	kfree(hsi_core);
	hsi_core = NULL;
failed_alloc_config:
	return ret;
}

bool hsi_disable_rx_fifo_req(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 reg;
	bool rx_fifo_req_ena = rx_cfg->rx_fifo_req_ena;

	if (rx_fifo_req_ena == true) {
		reg  = HSI_READ32(RX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 1, 1, 0);
		HSI_WRITE32(RX_FIFO_CONFIG + (channel << 2), reg);

		rx_cfg->rx_fifo_req_ena = false;
	}

	return rx_fifo_req_ena;
}

bool hsi_enable_rx_fifo_req(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 reg;
	bool rx_fifo_req_ena = rx_cfg->rx_fifo_req_ena;

	if (rx_fifo_req_ena == false) {
		rx_cfg->rx_fifo_req_ena = true;
		reg  = HSI_READ32(RX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 1, 1, 1);
		HSI_WRITE32(RX_FIFO_CONFIG + (channel << 2), reg);
	}

	return rx_fifo_req_ena;
}

bool hsi_disable_ipc_cmd_rcv_int(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 channel_mask;
	bool ipc_cmd_rcv_ena = rx_cfg->ipc_cmd_rcv_ena;

	if (channel != DATA_CH)
		return false;

	if (ipc_cmd_rcv_ena == true) {
		channel_mask = HSI_READ32(CHNL_INT_MASK + (channel << 2));
		channel_mask &= ~HSI_CH_MSK_IPC_CMD;
		HSI_WRITE32(CHNL_INT_MASK + (channel << 2), channel_mask);
		rx_cfg->ipc_cmd_rcv_ena = false;
	}

	return ipc_cmd_rcv_ena;
}

bool hsi_enable_ipc_cmd_rcv_int(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 channel_mask;
	bool ipc_cmd_rcv_ena = rx_cfg->ipc_cmd_rcv_ena;

	if (channel != DATA_CH)
		return false;

	if (ipc_cmd_rcv_ena == false) {
		pr_info("enable ipc cmd rcv int\n");
		channel_mask = HSI_READ32(CHNL_INT_MASK + (channel << 2));
		channel_mask |= HSI_CH_MSK_IPC_CMD;
		HSI_WRITE32(CHNL_INT_MASK + (channel << 2), channel_mask);
		rx_cfg->ipc_cmd_rcv_ena = true;
	}

	return ipc_cmd_rcv_ena;
}

bool hsi_disable_rx_fifo_int(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 channel_mask;
	bool rx_fifo_int_ena = rx_cfg->rx_fifo_int_ena;

	if (rx_fifo_int_ena == true) {
		channel_mask = HSI_READ32(CHNL_INT_MASK + (channel << 2));
		channel_mask &= ~HSI_CH_MSK_RX;
		HSI_WRITE32(CHNL_INT_MASK + (channel << 2), channel_mask);
		rx_cfg->rx_fifo_int_ena = false;
	}

	return rx_fifo_int_ena;
}

bool hsi_enable_rx_fifo_int(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 channel_mask;
	u32 channel_status;
	u32 reg, ch_offset;
	bool rx_fifo_int_ena = rx_cfg->rx_fifo_int_ena;

	if (rx_fifo_int_ena == false) {
		/* Clear channel status, W1CLR */
		channel_status = HSI_CH_STATUS_CLR_RX_MASK;
		HSI_WRITE32(CHNL_STATUS + (channel << 2), channel_status);

		/* Set channel INT mask*/
		channel_mask = HSI_READ32(CHNL_INT_MASK + (channel << 2));
		channel_mask &= ~HSI_CH_MSK_RX;
		if (rx_cfg->rx_fifo_req_sel == HSI_FIFO_REQ_INT)
			channel_mask |= HSI_CH_MSK_RX_FIFO_THRS;
		else {
			channel_mask |= HSI_CH_MSK_RX_DMA_DONE;
			channel_mask |= HSI_CH_MSK_RX_DMA_ERR;
			channel_mask |= HSI_CH_MSK_RX_FRM_LOST;
		}
		HSI_WRITE32(CHNL_INT_MASK + (channel << 2), channel_mask);

		/* Set main INT mask*/
		reg = HSI_READ32(HSI_INT_MASK0);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK0, reg);
		reg = HSI_READ32(HSI_INT_MASK1);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK1, reg);
		reg = HSI_READ32(HSI_INT_MASK2);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK2, reg);
		reg = HSI_READ32(HSI_INT_MASK3);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK3, reg);

		rx_cfg->rx_fifo_int_ena = true;
	}

	return rx_fifo_int_ena;
}


bool hsi_enable_rx_fifo_cpu(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 reg;
	bool rx_fifo_enable = rx_cfg->rx_fifo_enable;

	if (rx_fifo_enable == false) {
		/* Enable FIFO HW and disable request*/
		reg  = HSI_READ32(RX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 2, 2, 0);
		reg = HSI_MODIFY32(reg, 1, 1, 0);
		reg = HSI_MODIFY32(reg, 0, 0, 1);
		HSI_WRITE32(RX_FIFO_CONFIG + (channel << 2), reg);

		rx_cfg->rx_fifo_enable = true;
		rx_cfg->rx_fifo_req_ena = false;
		rx_cfg->rx_fifo_req_sel = HSI_FIFO_REQ_INT;
	}

	return rx_fifo_enable;
}

bool hsi_enable_rx_fifo_dma(int channel, struct strt_hsi_rx_ch_cfg *rx_cfg)
{
	u32 reg;
	bool rx_fifo_enable = rx_cfg->rx_fifo_enable;

	if (rx_fifo_enable == false) {
		/* Enable FIFO HW and disable request*/
		reg  = HSI_READ32(RX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 2, 2, 1);
		reg = HSI_MODIFY32(reg, 1, 1, 0);
		reg = HSI_MODIFY32(reg, 0, 0, 1);
		HSI_WRITE32(RX_FIFO_CONFIG + (channel << 2), reg);

		rx_cfg->rx_fifo_enable = true;
		rx_cfg->rx_fifo_req_ena = false;
		rx_cfg->rx_fifo_req_sel = HSI_FIFO_REQ_DMA;
	}

	return rx_fifo_enable;
}

int hsi_config_cpu_rx_channel(int channel,
	hsi_rx_thrs_handler_ptr rx_cpu_hdlr, bool enable_fifo)
{
	u32 reg;
	int ret = 0;
	struct strt_hsi_rx_ch_cfg *rx_cfg;
	unsigned long flags;

	HSI_LOCK(0);

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	if (rx_cfg->rx_busy) {
		ret = -EBUSY;
		spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);
		goto exit;
	}
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	rx_cfg->rx_dma_handler = NULL;
	rx_cfg->rx_cpu_handler = rx_cpu_hdlr;

	if (enable_fifo == false) {
		/* Disable request and FIFO HW */
		reg  = HSI_READ32(RX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 0, 0, 0);
		HSI_WRITE32(RX_FIFO_CONFIG + (channel << 2), reg);
		rx_cfg->rx_fifo_enable = false;

		hsi_disable_rx_fifo_int(channel, rx_cfg);

		goto exit;
	}

	hsi_enable_rx_fifo_cpu(channel, rx_cfg);
	if (rx_cfg->rx_cpu_handler)
		hsi_enable_rx_fifo_req(channel, rx_cfg);

	hsi_enable_rx_fifo_int(channel, rx_cfg);

exit:
	HSI_UNLOCK(0);

	return ret;
}
EXPORT_SYMBOL(hsi_config_cpu_rx_channel);

int hsi_config_dma_rx_channel(int channel,
	hsi_dma_done_handler_ptr rx_dma_hdlr, bool enable_fifo)
{
	u32 ret = 0, reg;
	struct strt_hsi_rx_ch_cfg *rx_cfg;
	unsigned long flags;

	HSI_LOCK(0);

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	if (rx_cfg->rx_busy) {
		ret = -EBUSY;
		spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);
		goto exit;
	}
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	rx_cfg->rx_dma_handler = rx_dma_hdlr;
	rx_cfg->rx_cpu_handler = NULL;

	if (enable_fifo == false) {
		/* Disable request and FIFO HW */
		reg  = HSI_READ32(RX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 0, 0, 0);
		HSI_WRITE32(RX_FIFO_CONFIG + (channel << 2), reg);
		rx_cfg->rx_fifo_enable = false;

		hsi_disable_rx_fifo_int(channel, rx_cfg);

		goto exit;
	}

	hsi_enable_rx_fifo_dma(channel, rx_cfg);
	hsi_enable_rx_fifo_int(channel, rx_cfg);
exit:
	HSI_UNLOCK(0);

	return ret;
}
EXPORT_SYMBOL(hsi_config_dma_rx_channel);

int hsi_receive(int channel, u32 *data, u32 byte_count)
{
	u32 frame_rcv;
	int ret = 0;
	unsigned long flags;
	struct strt_hsi_rx_ch_cfg *rx_cfg;
	long rc = 0;
	u32 frame_count;

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);
	frame_count = byte_count >> 2;

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	if (rx_cfg->rx_busy) {
		spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);
		return -EBUSY;
	} else
		rx_cfg->rx_busy = true;
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	HSI_LOCK(0);

	rx_cfg->rx_frame = data;
	rx_cfg->rx_frame_cnt = frame_count;
	rx_cfg->rx_frame_rcv = 0;

	hsi_enable_rx_fifo_cpu(channel, rx_cfg);
	hsi_enable_rx_fifo_req(channel, rx_cfg);
	hsi_enable_rx_fifo_int(channel, rx_cfg);

	frame_rcv = rx_cfg->rx_frame_rcv;
	while (1) {
		rc = wait_for_completion_timeout(&rx_cfg->rx_thrs_event, 500);
		INIT_COMPLETION(rx_cfg->rx_thrs_event);
		if (rc == 0) {
			if (frame_rcv == rx_cfg->rx_frame_rcv) {
				dev_warn(hsi_core->dev, "hsi receive timeout\n");
				ret = -ETIMEDOUT;
				goto exit;
			} else
				frame_rcv = rx_cfg->rx_frame_rcv;
		} else
			break;
	}

	if (rx_cfg->rx_frame_rcv != rx_cfg->rx_frame_cnt)
		ret = -ENXIO;

exit:
	hsi_disable_rx_fifo_req(channel, rx_cfg);
	hsi_disable_rx_fifo_int(channel, rx_cfg);

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	rx_cfg->rx_busy = false;
	rx_cfg->rx_frame = NULL;
	rx_cfg->rx_frame_cnt = 0;
	rx_cfg->rx_frame_rcv = 0;
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	HSI_UNLOCK(0);

	return ret;
}
EXPORT_SYMBOL(hsi_receive);

int hsi_receive_dma(int channel, u32 *paddr, u32 byte_count, void * cookie)
{
	unsigned long flags;
	struct strt_hsi_rx_ch_cfg *rx_cfg;
	u32 reg;
	int burst_size;

	rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[channel]);

	burst_size = (rx_cfg->rx_ch_fifo_size * 2
		- rx_cfg->rx_ch_fifo_thrs) * 4;
	if (burst_size <= 0)
		return -EINVAL;

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	if (rx_cfg->rx_busy) {
		spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);
		return -EBUSY;
	} else {
		rx_cfg->rx_busy = true;
		rx_cfg->dma_err = false;
	}
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	HSI_LOCK(0);

	rx_cfg->dma_cookie = cookie;
	hsi_enable_rx_fifo_dma(channel, rx_cfg);
	hsi_enable_rx_fifo_int(channel, rx_cfg);

	HSI_WRITE32(RX_DMA_DADR + (channel << 2), (u32)paddr);
	HSI_WRITE32(RX_DMA_DLEN + (channel << 2), byte_count & 0xFFFF);
	reg = HSI_READ32(RX_DMA_CTRL + (channel << 2));
	reg = HSI_MODIFY32(reg, 16, 16, 0);
	if (burst_size >= 64)
		reg = HSI_MODIFY32(reg, 10, 8, 0x7);
	else if (burst_size >= 32)
		reg = HSI_MODIFY32(reg, 10, 8, 0x6);
	else if (burst_size >= 16)
		reg = HSI_MODIFY32(reg, 10, 8, 0x3);
	else if (burst_size >= 8)
		reg = HSI_MODIFY32(reg, 10, 8, 0x1);
	else if (burst_size >= 4)
		reg = HSI_MODIFY32(reg, 10, 8, 0x0);
	reg = HSI_MODIFY32(reg, 0, 0, 1);
	HSI_WRITE32(RX_DMA_CTRL + (channel << 2), reg);

	hsi_enable_rx_fifo_req(channel, rx_cfg);

	reg = HSI_MODIFY32(reg, 24, 24, 1);
	HSI_WRITE32(RX_DMA_CTRL + (channel << 2), reg);

	HSI_UNLOCK(0);

	return 0;
}
EXPORT_SYMBOL(hsi_receive_dma);

int hsi_receive_dma_chain(int channel, u32 **data,
	u32 *frame_count, u32 entriesi, void * cookie)
{
	return -EINVAL;
}
EXPORT_SYMBOL(hsi_receive_dma_chain);


bool hsi_disable_tx_fifo_req(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg)
{
	u32 reg;
	bool tx_fifo_req_ena = tx_cfg->tx_fifo_req_ena;

	if (tx_fifo_req_ena == true) {
		reg  = HSI_READ32(TX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 1, 1, 0);
		HSI_WRITE32(TX_FIFO_CONFIG + (channel << 2), reg);

		tx_cfg->tx_fifo_req_ena = false;
	}

	return tx_fifo_req_ena;
}

bool hsi_enable_tx_fifo_req(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg)
{
	u32 reg;
	bool tx_fifo_req_ena = tx_cfg->tx_fifo_req_ena;

	if (tx_fifo_req_ena == false) {
		tx_cfg->tx_fifo_req_ena = true;
		reg  = HSI_READ32(TX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 1, 1, 1);
		HSI_WRITE32(TX_FIFO_CONFIG + (channel << 2), reg);
	}

	return tx_fifo_req_ena;
}

bool hsi_disable_tx_fifo_int(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg)
{
	u32 channel_mask;
	bool tx_fifo_int_ena = tx_cfg->tx_fifo_int_ena;

	tx_cfg->tx_fifo_int_ena = false;

	if (tx_fifo_int_ena == true) {
		channel_mask = HSI_READ32(CHNL_INT_MASK + (channel << 2));
		channel_mask &= ~HSI_CH_MSK_TX;
		HSI_WRITE32(CHNL_INT_MASK + (channel << 2), channel_mask);
		tx_cfg->tx_fifo_int_ena = false;
	}

	return tx_fifo_int_ena;
}

bool hsi_enable_tx_fifo_int(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg)
{
	u32 channel_mask;
	u32 channel_status;
	u32 reg, ch_offset;
	bool tx_fifo_int_ena = tx_cfg->tx_fifo_int_ena;

	if (false == tx_fifo_int_ena) {
		/* Clear channel status, W1CLR */
		channel_status = HSI_CH_STATUS_CLR_TX_MASK;
		HSI_WRITE32(CHNL_STATUS + (channel << 2), channel_status);

		/* Set channel INT mask*/
		channel_mask = HSI_READ32(CHNL_INT_MASK + (channel << 2));
		channel_mask &= ~HSI_CH_MSK_TX;
		if (tx_cfg->tx_fifo_req_sel == HSI_FIFO_REQ_INT)
			channel_mask |= HSI_CH_MSK_TX_FIFO_THRS;
		else {
			channel_mask |= HSI_CH_MSK_TX_DMA_ERR;
		}
		HSI_WRITE32(CHNL_INT_MASK + (channel << 2), channel_mask);

		/* Set main INT mask*/
		reg = HSI_READ32(HSI_INT_MASK0);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK0, reg);
		reg = HSI_READ32(HSI_INT_MASK1);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK1, reg);
		reg = HSI_READ32(HSI_INT_MASK2);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK2, reg);
		reg = HSI_READ32(HSI_INT_MASK3);
		ch_offset = HSI_MSK_CH0_BIT + channel;
		reg = HSI_MODIFY32(reg, ch_offset, ch_offset, 0x1);
		HSI_WRITE32(HSI_INT_MASK3, reg);
		tx_cfg->tx_fifo_int_ena = true;
	}

	return tx_fifo_int_ena;
}


bool hsi_enable_tx_fifo_cpu(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg)
{
	u32 reg;
	bool tx_fifo_enable = tx_cfg->tx_fifo_enable;

	if (tx_fifo_enable == false) {
		reg  = HSI_READ32(TX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 2, 2, 0);
		reg = HSI_MODIFY32(reg, 1, 1, 0);
		reg = HSI_MODIFY32(reg, 0, 0, 1);
		HSI_WRITE32(TX_FIFO_CONFIG + (channel << 2), reg);

		tx_cfg->tx_fifo_enable = true;
		tx_cfg->tx_fifo_req_ena = false;
		tx_cfg->tx_fifo_req_sel = HSI_FIFO_REQ_INT;
	}

	return tx_fifo_enable;
}

bool hsi_enable_tx_fifo_dma(int channel, struct strt_hsi_tx_ch_cfg *tx_cfg)
{
	u32 reg;
	bool tx_fifo_enable = tx_cfg->tx_fifo_enable;

	if (tx_fifo_enable == false) {
		/* Enable request and FIFO HW */
		reg  = HSI_READ32(TX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 2, 2, 1);
		reg = HSI_MODIFY32(reg, 1, 1, 0);
		reg = HSI_MODIFY32(reg, 0, 0, 1);
		HSI_WRITE32(TX_FIFO_CONFIG + (channel << 2), reg);

		tx_cfg->tx_fifo_enable = true;
		tx_cfg->tx_fifo_req_ena = false;
		tx_cfg->tx_fifo_req_sel = HSI_FIFO_REQ_DMA;
	}

	return tx_fifo_enable;
}

int hsi_config_cpu_tx_channel(int channel, bool enable_fifo)
{
	u32 reg;
	struct strt_hsi_tx_ch_cfg *tx_cfg;
	int ret = 0;
	unsigned long flags;

	HSI_LOCK(0);

	tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[channel]);

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	if (tx_cfg->tx_busy) {
		ret = -EBUSY;
		spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);
		goto exit;
	}
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	tx_cfg->tx_dma_handler = NULL;

	if (enable_fifo == false) {
		/* Disable request and FIFO HW */
		reg  = HSI_READ32(TX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 0, 0, 0);
		HSI_WRITE32(TX_FIFO_CONFIG + (channel << 2), reg);
		tx_cfg->tx_fifo_enable = false;

		hsi_disable_tx_fifo_int(channel, tx_cfg);

		goto exit;
	}

	hsi_enable_tx_fifo_cpu(channel, tx_cfg);
	hsi_enable_tx_fifo_int(channel, tx_cfg);
exit:
	HSI_UNLOCK(0);
	return ret;
}
EXPORT_SYMBOL(hsi_config_cpu_tx_channel);

int hsi_register_rx_break_handler(hsi_break_received_handler_ptr handler)
{
	hsi_core->hsi_config->rx_brk_handler = handler;
	return 0;
}
EXPORT_SYMBOL(hsi_register_rx_break_handler);

int hsi_config_dma_tx_channel(int channel,
	hsi_dma_done_handler_ptr tx_dma_hdlr, bool enable_fifo)
{
	struct strt_hsi_tx_ch_cfg *tx_cfg;
	unsigned long flags;
	int ret = 0;
	u32 reg;

	HSI_LOCK(0);

	tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[channel]);

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	if (tx_cfg->tx_busy) {
		spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);
		ret = -EBUSY;
		goto exit;
	}
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	tx_cfg->tx_dma_handler = tx_dma_hdlr;

	if (enable_fifo == false) {
		/* Disable request and FIFO HW */
		reg  = HSI_READ32(TX_FIFO_CONFIG + (channel << 2));
		reg = HSI_MODIFY32(reg, 0, 0, 0);
		HSI_WRITE32(TX_FIFO_CONFIG + (channel << 2), reg);

		tx_cfg->tx_fifo_enable = false;

		hsi_disable_tx_fifo_int(channel, tx_cfg);

		goto exit;
	}

	hsi_enable_tx_fifo_dma(channel, tx_cfg);
	hsi_enable_tx_fifo_int(channel, tx_cfg);

exit:
	HSI_UNLOCK(0);
	return ret;
}
EXPORT_SYMBOL(hsi_config_dma_tx_channel);

int hsi_transmit(int channel, u32 *data, u32 byte_count)
{
	if (byte_count == 4)
		return hsi_transmit_cmd(channel, data, byte_count, NULL);
	if (byte_count > 4)
		return hsi_transmit_nblk(channel, data, byte_count, NULL);
	else
		return -EINVAL;
}
EXPORT_SYMBOL(hsi_transmit);

int hsi_transmit_cmd(int channel, u32 *data, u32 byte_count, u32 *tx_cnt)
{
	HSI_LOCK(0);
	HSI_WRITE32(TX_FIFO + (channel << 3), *data);
	HSI_UNLOCK(0);

	return 0;
}
EXPORT_SYMBOL(hsi_transmit_cmd);

int hsi_transmit_nblk(int channel, u32 *data, u32 byte_count, u32 *tx_cnt)
{
	unsigned long flags;
	struct strt_hsi_tx_ch_cfg *tx_cfg;
	long rc = 0;
	int ret = 0;
	int burst_size, burst_cnt, frame_left;
	u32 frame_num, frame_count;

	tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[channel]);

	frame_count = byte_count >> 2;
	frame_num = HSI_READ32(CHNL_FIFO_STATUS + (channel << 2));
	frame_num = HSI_GET32(frame_num,
		BIT_TX_FRM_CNT_MSB, BIT_TX_FRM_CNT_LSB);
	burst_size = tx_cfg->tx_ch_fifo_size * FRAME_PER_FIFO - frame_num;

	if (burst_size <= 0)
		return -EINVAL;

	if (tx_cnt)
		*tx_cnt = 0;

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	if (tx_cfg->tx_busy) {
		spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);
		return -EBUSY;
	} else
		tx_cfg->tx_busy = true;
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	HSI_LOCK(0);
	hsi_enable_tx_fifo_cpu(channel, tx_cfg);
	hsi_enable_tx_fifo_int(channel, tx_cfg);

	tx_cfg->tx_frame = data;
	tx_cfg->tx_frame_cnt = frame_count;
	tx_cfg->tx_frame_sent = 0;

	burst_cnt = burst_size;
	while (burst_cnt && tx_cfg->tx_frame_cnt) {
		HSI_WRITE32(TX_FIFO + (channel << 3), *(tx_cfg->tx_frame++));
		burst_cnt--;
		tx_cfg->tx_frame_cnt--;
		tx_cfg->tx_frame_sent++;
	}

	frame_num = HSI_READ32(CHNL_FIFO_STATUS + (channel << 2));
	frame_num = HSI_GET32(frame_num,
		BIT_TX_FRM_CNT_MSB, BIT_TX_FRM_CNT_LSB);
	if (tx_cfg->tx_frame_cnt > 0) {
		frame_left = tx_cfg->tx_frame_cnt;
		hsi_enable_tx_fifo_req(channel, tx_cfg);

		while (1) {
			rc = wait_for_completion_timeout(
				&tx_cfg->tx_thrs_event, 500);
			INIT_COMPLETION(tx_cfg->tx_thrs_event);
			if (rc == 0) {
				if (frame_left == tx_cfg->tx_frame_cnt) {
					dev_warn(hsi_core->dev, "hsi transmit timeout\n");
					ret = -ETIMEDOUT;
					goto exit;
				} else
					frame_left = tx_cfg->tx_frame_cnt;
			} else
				break;
		}
	}

exit:
	hsi_disable_tx_fifo_req(channel, tx_cfg);

	if (tx_cnt)
		*tx_cnt = tx_cfg->tx_frame_sent;

	hsi_disable_tx_fifo_int(channel, tx_cfg);

	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	tx_cfg->tx_busy = false;
	tx_cfg->tx_frame = NULL;
	tx_cfg->tx_frame_cnt = 0;
	tx_cfg->tx_frame_sent = 0;
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	HSI_UNLOCK(0);
	return 0;
}
EXPORT_SYMBOL(hsi_transmit_nblk);

int hsi_transmit_dma(int channel, u32 *paddr, u32 byte_count, void *cookie)
{
	struct strt_hsi_tx_ch_cfg *tx_cfg;
	u32 reg;
	int burst_size;

	tx_cfg = &(hsi_core->hsi_config->tx_chnl_config[channel]);
	burst_size = (tx_cfg->tx_ch_fifo_size * FRAME_PER_FIFO
		- tx_cfg->tx_ch_fifo_thrs) * 4;
	if (burst_size <= 0)
		return -EINVAL;

	HSI_LOCK(0);

	tx_cfg->dma_cookie = cookie;
	hsi_enable_tx_fifo_dma(channel, tx_cfg);
	hsi_enable_tx_fifo_int(channel, tx_cfg);

	HSI_WRITE32(TX_DMA_SADR + (channel << 2), (u32)paddr);

	reg = HSI_READ32(TX_DMA_DLEN + (channel << 2));
	reg = HSI_MODIFY32(reg, 15, 0, byte_count & 0xFFFF);
	HSI_WRITE32(TX_DMA_DLEN + (channel << 2), reg);

	reg = HSI_READ32(TX_DMA_CTRL + (channel << 2));
	reg = HSI_MODIFY32(reg, 16, 16, 0);
	if (burst_size >= 64)
		reg = HSI_MODIFY32(reg, 10, 8, 0x7);
	else if (burst_size >= 32)
		reg = HSI_MODIFY32(reg, 10, 8, 0x6);
	else if (burst_size >= 16)
		reg = HSI_MODIFY32(reg, 10, 8, 0x3);
	else if (burst_size >= 8)
		reg = HSI_MODIFY32(reg, 10, 8, 0x1);
	else if (burst_size >= 4)
		reg = HSI_MODIFY32(reg, 10, 8, 0x0);
	reg = HSI_MODIFY32(reg, 0, 0, 1);
	HSI_WRITE32(TX_DMA_CTRL + (channel << 2), reg);

	hsi_enable_tx_fifo_req(channel, tx_cfg);

	reg = HSI_READ32(TX_DMA_CTRL + (channel << 2));
	reg = HSI_MODIFY32(reg, 24, 24, 1);
	HSI_WRITE32(TX_DMA_CTRL + (channel << 2), reg);

	HSI_UNLOCK(0);

	return 0;
}
EXPORT_SYMBOL(hsi_transmit_dma);

int hsi_transmit_dma_chain(int channel, u32 **data,
	u32 *byte_count, u32 entries, void * cookie)
{
	return -EINVAL;
}
EXPORT_SYMBOL(hsi_transmit_dma_chain);

int hsi_transmit_break(void)
{
	u32 reg;

	HSI_LOCK(0);

	reg = HSI_READ32(HSI_BRK_CONFIG);
	if (HSI_GET32(reg, BIT_BRK_CNT_MSB, BIT_BRK_CNT_LSB) < 0x25) {
		reg = HSI_MODIFY32(reg, BIT_BRK_CNT_MSB, BIT_BRK_CNT_LSB, 0x25);
		HSI_WRITE32(HSI_BRK_CONFIG, reg);
	}
	reg = HSI_MODIFY32(reg, BIT_TX_BREAK, BIT_TX_BREAK, 1);
	HSI_WRITE32(HSI_BRK_CONFIG, reg);

	HSI_UNLOCK(0);
	return 0;
}
EXPORT_SYMBOL(hsi_transmit_break);

int hsi_reset_tx(void)
{
	u32 reg;
	int ch;
	unsigned long flags;

	HSI_LOCK(0);
	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	for (ch = 0; ch < HSI_TX_CH_NUM; ch++)
		hsi_core->hsi_config->tx_chnl_config[ch].tx_busy = 0;
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	reg = HSI_READ32(HSI_CNTRL);
	reg = HSI_MODIFY32(reg, BIT_TX_BIT_RST, BIT_TX_WRM_RST, 1);
	HSI_WRITE32(HSI_CNTRL, reg);
	HSI_UNLOCK(0);

	return 0;
}
EXPORT_SYMBOL(hsi_reset_tx);

int hsi_reset_rx(void)
{
	u32 reg;
	int ch;
	unsigned long flags;

	HSI_LOCK(0);
	spin_lock_irqsave(&hsi_core->s_irqlock, flags);
	for (ch = 0; ch < HSI_RX_CH_NUM; ch++)
		hsi_core->hsi_config->rx_chnl_config[ch].rx_busy = false;
	spin_unlock_irqrestore(&hsi_core->s_irqlock, flags);

	reg = HSI_READ32(HSI_CNTRL);
	reg = HSI_MODIFY32(reg, BIT_RX_BIT_RST, BIT_RX_WRM_RST, 1);
	HSI_WRITE32(HSI_CNTRL, reg);

	HSI_UNLOCK(0);

	return 0;
}
EXPORT_SYMBOL(hsi_reset_rx);

int hsi_set_work_mode(enum hsi_work_mode_enum mode)
{
	u32 reg;
	struct strt_hsi_rx_ch_cfg *rx_cfg;
	switch (mode) {
	case HSI_BOOTROM_MODE:
		hsi_core->hsi_config->ch_bits = HSI_IF_CHANNEL_BITS_1;
		hsi_core->hsi_config->trans_mode = HSI_IF_OP_MODE_STREAM;
		hsi_core->hsi_config->tx_rate_div = 0x7;
		hsi_core->hsi_config->mode = HSI_BOOTROM_MODE;
		reg = HSI_READ32(HSI_CONFIG1);
		/* Channel BIT set to 1 */
		reg = HSI_MODIFY32(reg, 18, 16, hsi_core->hsi_config->ch_bits);
		/* Use stream mode */
		reg = HSI_MODIFY32(reg, 8, 8, hsi_core->hsi_config->trans_mode);
		HSI_WRITE32(HSI_CONFIG1, reg);

		/* Set TX divider */
		reg = HSI_READ32(HSI_CONFIG2);
		reg = HSI_MODIFY32(reg, 15, 0, hsi_core->hsi_config->tx_rate_div);
		HSI_WRITE32(HSI_CONFIG2, reg);
		break;
	case HSI_CPIMAGE_MODE:
		hsi_core->hsi_config->ch_bits = HSI_IF_CHANNEL_BITS_2;
		hsi_core->hsi_config->trans_mode = HSI_IF_OP_MODE_FRAME;
		hsi_core->hsi_config->tx_rate_div = 0x1;
		hsi_core->hsi_config->mode = HSI_CPIMAGE_MODE;
		reg = HSI_READ32(HSI_CONFIG1);
		/* Channel BIT set to 2 */
		reg = HSI_MODIFY32(reg, 18, 16, hsi_core->hsi_config->ch_bits);
		/* Use frame mode */
		reg = HSI_MODIFY32(reg, 8, 8, hsi_core->hsi_config->trans_mode);
		HSI_WRITE32(HSI_CONFIG1, reg);

		/* Set TX divider */
		reg = HSI_READ32(HSI_CONFIG2);
		reg = HSI_MODIFY32(reg, 15, 0, hsi_core->hsi_config->tx_rate_div);
		HSI_WRITE32(HSI_CONFIG2, reg);

		/* enable IPC for ACK */
		reg = HSI_READ32(HSI_CNTRL);
		reg = HSI_MODIFY32(reg, 15, 12, 0xB);
		reg = HSI_MODIFY32(reg, 8, 8, 1);
		HSI_WRITE32(HSI_CNTRL, reg);
		rx_cfg = &(hsi_core->hsi_config->rx_chnl_config[DATA_CH]);
		hsi_enable_ipc_cmd_rcv_int(DATA_CH, rx_cfg);
		break;
	default:
		break;
	}

	hsi_reset_rx();
	hsi_reset_tx();

	return 0;
}
EXPORT_SYMBOL(hsi_set_work_mode);

static int __devexit mmp3_hsi_remove(struct platform_device *pdev)
{
	disable_irq(hsi_core->irq);
	free_irq(hsi_core->irq, hsi_core);
	iounmap(hsi_core->mmio_hsi_base);
	release_mem_region(hsi_core->mem_res_hsi->start, hsi_core->mmio_size);
	clk_put(hsi_core->clk);

	kfree(hsi_core);
	hsi_core = NULL;

	return 0;
}

struct platform_driver mmp3_hsi_driver = {
	.driver		= {
		.name	= "mmp3_hsi",
		.owner  = THIS_MODULE,
	},
	.probe		= mmp3_hsi_probe,
	.remove		= __devexit_p(mmp3_hsi_remove),
};

static int __init mmp3_hsi_init(void)
{
	int ret;

	ret = misc_register(&hsi_miscdev);
	if (ret != 0)
		return -EBUSY;

	return platform_driver_register(&mmp3_hsi_driver);
}

static void __exit mmp3_hsi_exit(void)
{
	platform_driver_unregister(&mmp3_hsi_driver);
	misc_deregister(&hsi_miscdev);
}


module_init(mmp3_hsi_init);
module_exit(mmp3_hsi_exit);

MODULE_AUTHOR("Marvell Technology Ltd.");
MODULE_DESCRIPTION("HSI driver");
MODULE_LICENSE("GPL");
