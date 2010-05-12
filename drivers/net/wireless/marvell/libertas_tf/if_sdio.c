/*
 *  linux/drivers/net/wireless/libertas_tf/if_sdio.c
 *
 *  Copyright (C) 2010, cozybit Inc.
 *  
 * Portions Copyright 2007-2008 Pierre Ossman
 * Inspired by if_cs.c, Copyright 2007 Holger Schurig
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>

#define DRV_NAME "lbtf_sdio"

#include "deb_defs.h"
#include "libertas_tf.h"
// #include "host.h"
// #include "decl.h"
// #include "defs.h"
// #include "dev.h"
// #include "cmd.h"
#include "if_sdio.h"

static char *lbtf_helper_name = NULL;
module_param_named(helper_name, lbtf_helper_name, charp, 0644);

static char *lbtf_fw_name = NULL;
module_param_named(fw_name, lbtf_fw_name, charp, 0644);

static const struct sdio_device_id if_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_MARVELL,
			SDIO_DEVICE_ID_MARVELL_LIBERTAS) },
	{ SDIO_DEVICE(SDIO_VENDOR_ID_MARVELL,
			SDIO_DEVICE_ID_MARVELL_8688WLAN) },
	{ /* end: all zeroes */				},
};

MODULE_DEVICE_TABLE(sdio, if_sdio_ids);

struct if_sdio_model {
	int model;
	const char *helper;
	const char *firmware;
};

static struct if_sdio_model if_sdio_models[] = {
	{
		/* 8686 */
		.model = IF_SDIO_MODEL_8686,
		.helper = "libertas/sd8686_v9_helper.bin",
		.firmware = "libertas/lbtf_sdio.bin",
	},
};
MODULE_FIRMWARE("libertas/sd8686_v9_helper.bin");
MODULE_FIRMWARE("libertas/lbtf_sdio.bin");

struct if_sdio_packet {
	struct if_sdio_packet	*next;
	u16			nb;
	u8			buffer[0] __attribute__((aligned(4)));
};

struct if_sdio_card {
	struct sdio_func	*func;
	struct lbtf_private	*priv;

	int			model;
	unsigned long		ioport;
	unsigned int		scratch_reg;

	const char		*helper;
	const char		*firmware;

	u8			buffer[65536];

	spinlock_t		lock;
	struct if_sdio_packet	*packets;

	struct workqueue_struct	*workqueue;
	struct work_struct	packet_worker;

	u8			rx_unit;
};

/*
 *  For SD8385/SD8686, this function reads firmware status after
 *  the image is downloaded, or reads RX packet length when
 *  interrupt (with IF_SDIO_H_INT_UPLD bit set) is received.
 *  For SD8688, this function reads firmware status only.
 */
static u16 if_sdio_read_scratch(struct if_sdio_card *card, int *err)
{
	int ret;
	u16 scratch;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	scratch = sdio_readb(card->func, card->scratch_reg, &ret);
	if (!ret)
		scratch |= sdio_readb(card->func, card->scratch_reg + 1,
					&ret) << 8;

	if (err)
		*err = ret;

	if (ret)
		return 0xffff;

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "scratch %x", scratch);
	return scratch;
}

/********************************************************************/
/* I/O                                                              */
/********************************************************************/
static int if_sdio_handle_cmd(struct if_sdio_card *card,
		u8 *buffer, unsigned size)
{
	struct lbtf_private *priv = card->priv;
	int ret;

	lbtf_deb_enter(LBTF_DEB_SDIO);


	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);
	return ret;
}

static int if_sdio_handle_data(struct if_sdio_card *card,
		u8 *buffer, unsigned size)
{
	int ret;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_handle_event(struct if_sdio_card *card,
		u8 *buffer, unsigned size)
{
	int ret = 0;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_wait_status(struct if_sdio_card *card, const u8 condition)
{
	u8 status;
	unsigned long timeout;
	int ret = 0;

//	lbtf_deb_enter(LBTF_DEB_SDIO);

	timeout = jiffies + HZ;
	while (1) {
		status = sdio_readb(card->func, IF_SDIO_STATUS, &ret);
		if (ret)
			return ret;
		if ((status & condition) == condition)
			break;
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		mdelay(1);
	}

//	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);
	return ret;
}

static int if_sdio_card_to_host(struct if_sdio_card *card)
{
	int ret;

	lbtf_deb_enter(LBTF_DEB_SDIO);


	if (ret)
		pr_err("problem fetching packet from firmware\n");

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static void if_sdio_host_to_card_worker(struct work_struct *work)
{
	struct if_sdio_card *card;
	struct if_sdio_packet *packet;
	int ret;
	unsigned long flags;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	card = container_of(work, struct if_sdio_card, packet_worker);

	while (1) {
		spin_lock_irqsave(&card->lock, flags);
		packet = card->packets;
		if (packet)
			card->packets = packet->next;
		spin_unlock_irqrestore(&card->lock, flags);

		if (!packet)
			break;

		sdio_claim_host(card->func);

		ret = if_sdio_wait_status(card, IF_SDIO_IO_RDY);
		if (ret == 0) {
			ret = sdio_writesb(card->func, card->ioport,
					   packet->buffer, packet->nb);
		}

		if (ret)
			pr_err("error %d sending packet to firmware\n", ret);

		sdio_release_host(card->func);

		kfree(packet);
	}

	lbtf_deb_leave(LBTF_DEB_SDIO);
}

/********************************************************************/
/* Firmware                                                         */
/********************************************************************/

#define FW_DL_READY_STATUS (IF_SDIO_IO_RDY | IF_SDIO_DL_RDY)

static int if_sdio_prog_helper(struct if_sdio_card *card)
{
	int ret;
	const struct firmware *fw;
	unsigned long timeout;
	u8 *chunk_buffer;
	u32 chunk_size;
	const u8 *firmware;
	size_t size;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	ret = request_firmware(&fw, card->helper, &card->func->dev);

	if (ret) {
		pr_err("failed to load helper firmware\n");
		goto out;
	}

	chunk_buffer = kzalloc(64, GFP_KERNEL);
	if (!chunk_buffer) {
		ret = -ENOMEM;
		goto release_fw;
	}

	sdio_claim_host(card->func);

	ret = sdio_set_block_size(card->func, 32);
	if (ret)
		goto release;

	firmware = fw->data;
	size = fw->size;

	lbtf_deb_sdio("Helper size: %d", size);

	while (size) {
		ret = if_sdio_wait_status(card, FW_DL_READY_STATUS);
		if (ret)
			goto release;

		/* On some platforms (like Davinci) the chip needs more time
		 * between helper blocks.
		 */
		mdelay(2);

		chunk_size = min(size, (size_t)60);

		*((__le32*)chunk_buffer) = cpu_to_le32(chunk_size);
		memcpy(chunk_buffer + 4, firmware, chunk_size);

		// lbtf_deb_sdio("sending %d bytes chunk\n", chunk_size);

		ret = sdio_writesb(card->func, card->ioport,
				chunk_buffer, 64);
		if (ret)
			goto release;

		firmware += chunk_size;
		size -= chunk_size;
	}

	/* an empty block marks the end of the transfer */
	memset(chunk_buffer, 0, 4);
	ret = sdio_writesb(card->func, card->ioport, chunk_buffer, 64);
	if (ret)
		goto release;

	lbtf_deb_sdio("waiting for helper to boot...\n");

	/* wait for the helper to boot by looking at the size register */
	timeout = jiffies + HZ;
	while (1) {
		u16 req_size;

		req_size = sdio_readb(card->func, IF_SDIO_RD_BASE, &ret);
		if (ret)
			goto release;

		req_size |= sdio_readb(card->func, IF_SDIO_RD_BASE + 1, &ret) << 8;
		if (ret)
			goto release;

		if (req_size != 0)
			break;

		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			goto release;
		}

		msleep(10);
	}

	ret = 0;

release:
	sdio_release_host(card->func);
	kfree(chunk_buffer);
release_fw:
	release_firmware(fw);

out:
	if (ret)
		pr_err("failed to load helper firmware\n");

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_prog_real(struct if_sdio_card *card)
{
	int ret;
	const struct firmware *fw;
	unsigned long timeout;
	u8 *chunk_buffer;
	u32 chunk_size;
	const u8 *firmware;
	size_t size, req_size;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	ret = request_firmware(&fw, card->firmware, &card->func->dev);
	if (ret) {
		pr_err("can't load firmware\n");
		goto out;
	}

	chunk_buffer = kzalloc(512, GFP_KERNEL);
	if (!chunk_buffer) {
		ret = -ENOMEM;
		goto release_fw;
	}

	sdio_claim_host(card->func);

	ret = sdio_set_block_size(card->func, 32);
	if (ret)
		goto release;

	firmware = fw->data;
	size = fw->size;

	lbtf_deb_sdio("Firmware size: %d", size);

	while (size) {
		ret = if_sdio_wait_status(card, FW_DL_READY_STATUS);
		if (ret)
			goto release;

		req_size = sdio_readb(card->func, IF_SDIO_RD_BASE, &ret);
		if (ret)
			goto release;

		req_size |= sdio_readb(card->func, IF_SDIO_RD_BASE + 1, &ret) << 8;
		if (ret)
			goto release;
/*
		lbtf_deb_sdio("firmware wants %d bytes\n", (int)req_size);
*/
		if (req_size == 0) {
			lbtf_deb_sdio("firmware helper gave up early\n");
			ret = -EIO;
			goto release;
		}

		if (req_size & 0x01) {
			lbtf_deb_sdio("firmware helper signalled error\n");
			ret = -EIO;
			goto release;
		}

		if (req_size > size)
			req_size = size;

		while (req_size) {
			chunk_size = min(req_size, (size_t)512);

			memcpy(chunk_buffer, firmware, chunk_size);
/*
			lbtf_deb_sdio("sending %d bytes (%d bytes) chunk\n",
				chunk_size, (chunk_size + 31) / 32 * 32);
*/
			ret = sdio_writesb(card->func, card->ioport,
				chunk_buffer, roundup(chunk_size, 32));
			if (ret)
				goto release;

			firmware += chunk_size;
			size -= chunk_size;
			req_size -= chunk_size;
		}
	}

	ret = 0;

	lbtf_deb_sdio("waiting for firmware to boot...\n");

	/* wait for the firmware to boot */
	timeout = jiffies + HZ;
	while (1) {
		u16 scratch;

		scratch = if_sdio_read_scratch(card, &ret);
		if (ret)
			goto release;

		if (scratch == IF_SDIO_FIRMWARE_OK)
			break;

		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			goto release;
		}

		msleep(10);
	}

	ret = 0;

release:
	sdio_release_host(card->func);
	kfree(chunk_buffer);
release_fw:
	release_firmware(fw);

out:
	if (ret)
		pr_err("failed to load firmware\n");

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_prog_firmware(struct if_sdio_card *card)
{
	int ret;
	u16 scratch;

	lbtf_deb_enter(LBTF_DEB_SDIO);
	sdio_claim_host(card->func);
	scratch = if_sdio_read_scratch(card, &ret);
	sdio_release_host(card->func);

	if (ret)
		goto out;

	lbtf_deb_sdio("firmware status = %#x\n", scratch);

	if (scratch == IF_SDIO_FIRMWARE_OK) {
		lbtf_deb_sdio("firmware already loaded\n");
		goto success;
	}

	ret = if_sdio_prog_helper(card);
	if (ret)
		goto out;

	lbtf_deb_sdio("Helper firmware loaded\n");

	ret = if_sdio_prog_real(card);
	if (ret)
		goto out;

	lbtf_deb_sdio("Firmware loaded\n");

success:
	sdio_claim_host(card->func);
	sdio_set_block_size(card->func, IF_SDIO_BLOCK_SIZE);

	/*
	 * Enable interrupts now that everything is set up
	 */
	sdio_writeb(card->func, 0x0f, IF_SDIO_H_INT_MASK, &ret);
	sdio_release_host(card->func);
	if (ret)
		pr_err("Error enabling interrupts: %d", ret);

out:
	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

/*******************************************************************/
/* Libertas callbacks                                              */
/*******************************************************************/

static int if_sdio_host_to_card(struct lbtf_private *priv,
		u8 type, u8 *buf, u16 nb)
{
	int ret;


	lbtf_deb_enter_args(LBTF_DEB_SDIO, "type %d, bytes %d", type, nb);


	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_enter_deep_sleep(struct lbtf_private *priv)
{
	int ret = -1;


	return ret;
}

static int if_sdio_exit_deep_sleep(struct lbtf_private *priv)
{
	struct if_sdio_card *card = priv->card;
	int ret = -1;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);
	return ret;
}

static int if_sdio_reset_deep_sleep_wakeup(struct lbtf_private *priv)
{
	struct if_sdio_card *card = priv->card;
	int ret = -1;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);
	return ret;

}

static int if_sdio_reset_device(struct if_sdio_card *card)
{
	int ret;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	ret = 0;

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

/*******************************************************************/
/* SDIO callbacks                                                  */
/*******************************************************************/

static void if_sdio_interrupt(struct sdio_func *func)
{
	int ret;

//	lbtf_deb_enter(LBTF_DEB_SDIO);


	ret = 0;

//	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);
}

static int if_sdio_probe(struct sdio_func *func,
		const struct sdio_device_id *id)
{
	struct if_sdio_card *card;
	struct lbtf_private *priv;
	int ret, i;
	unsigned int model;
	struct if_sdio_packet *packet;
	struct mmc_host *host = func->card->host;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	for (i = 0;i < func->card->num_info;i++) {
		if (sscanf(func->card->info[i],
				"802.11 SDIO ID: %x", &model) == 1)
			break;
		if (sscanf(func->card->info[i],
				"ID: %x", &model) == 1)
			break;
		if (!strcmp(func->card->info[i], "IBIS Wireless SDIO Card")) {
			model = IF_SDIO_MODEL_8385;
			break;
		}
	}

	if (i == func->card->num_info) {
		pr_err("unable to identify card model\n");
		return -ENODEV;
	}

	lbtf_deb_sdio("Found model: 0x%x", model);

	card = kzalloc(sizeof(struct if_sdio_card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->func = func;
	card->model = model;

	switch (card->model) {
	case IF_SDIO_MODEL_8385:
		card->scratch_reg = IF_SDIO_SCRATCH_OLD;
		break;
	case IF_SDIO_MODEL_8686:
		lbtf_deb_sdio("Found Marvel 8686");
		card->scratch_reg = IF_SDIO_SCRATCH;
		break;
	case IF_SDIO_MODEL_8688:
	default: /* for newer chipsets */
		card->scratch_reg = IF_SDIO_FW_STATUS;
		break;
	}

	spin_lock_init(&card->lock);
	card->workqueue = create_workqueue("libertas_tf_sdio");
	INIT_WORK(&card->packet_worker, if_sdio_host_to_card_worker);

	for (i = 0;i < ARRAY_SIZE(if_sdio_models);i++) {
		if (card->model == if_sdio_models[i].model)
			break;
	}

	if (i == ARRAY_SIZE(if_sdio_models)) {
		pr_err("unknown card model 0x%x\n", card->model);
		ret = -ENODEV;
		goto free;
	}

	card->helper = if_sdio_models[i].helper;
	card->firmware = if_sdio_models[i].firmware;

	if (lbtf_helper_name) {
		lbtf_deb_sdio("overriding helper firmware: %s\n",
			lbtf_helper_name);
		card->helper = lbtf_helper_name;
	}

	if (lbtf_fw_name) {
		lbtf_deb_sdio("overriding firmware: %s\n", lbtf_fw_name);
		card->firmware = lbtf_fw_name;
	}

	sdio_claim_host(func);

	ret = sdio_enable_func(func);
	if (ret)
		goto release;

	ret = sdio_claim_irq(func, if_sdio_interrupt);
	if (ret)
		goto disable;

	/* For 1-bit transfers to the 8686 model, we need to enable the
	 * interrupt flag in the CCCR register. Set the MMC_QUIRK_LENIENT_FN0
	 * bit to allow access to non-vendor registers. */
	if ((card->model == IF_SDIO_MODEL_8686) &&
	    (host->caps & MMC_CAP_SDIO_IRQ) &&
	    (host->ios.bus_width == MMC_BUS_WIDTH_1)) {
		u8 reg;

		func->card->quirks |= MMC_QUIRK_LENIENT_FN0;
		reg = sdio_f0_readb(func, SDIO_CCCR_IF, &ret);
		if (ret)
			goto release_int;

		reg |= SDIO_BUS_ECSI;
		sdio_f0_writeb(func, reg, SDIO_CCCR_IF, &ret);
		if (ret)
			goto release_int;
	}

	card->ioport = sdio_readb(func, IF_SDIO_IOPORT, &ret);
	if (ret)
		goto release_int;

	card->ioport |= sdio_readb(func, IF_SDIO_IOPORT + 1, &ret) << 8;
	if (ret)
		goto release_int;

	card->ioport |= sdio_readb(func, IF_SDIO_IOPORT + 2, &ret) << 16;
	if (ret)
		goto release_int;

	sdio_release_host(func);
	sdio_set_drvdata(func, card);

	lbtf_deb_sdio("class = 0x%X, vendor = 0x%X, "
			"device = 0x%X, model = 0x%X, ioport = 0x%X\n",
			func->class, func->vendor, func->device,
			model, (unsigned)card->ioport);

	priv = lbtf_add_card(card, &func->dev);
	if (!priv) {
		ret = -ENOMEM;
		goto reclaim;
	}

	card->priv = priv;
	priv->card = card;

	priv->hw_host_to_card = if_sdio_host_to_card;
	priv->hw_prog_firmware = if_sdio_prog_firmware;
	priv->enter_deep_sleep = if_sdio_enter_deep_sleep;
	priv->exit_deep_sleep = if_sdio_exit_deep_sleep;
	priv->reset_deep_sleep_wakeup = if_sdio_reset_deep_sleep_wakeup;
	priv->hw_reset_device = if_sdio_reset_device;

	sdio_claim_host(func);
// 	/*
// 	 * Get rx_unit if the chip is SD8688 or newer.
// 	 * SD8385 & SD8686 do not have rx_unit.
// 	 */
// 	if ((card->model != IF_SDIO_MODEL_8385)
// 			&& (card->model != IF_SDIO_MODEL_8686))
// 		card->rx_unit = if_sdio_read_rx_unit(card);
// 	else
		card->rx_unit = 0;

	sdio_release_host(func);

out:
	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;

err_activate_card:
	lbtf_deb_sdio("prob error jump: err_activate_card");
	flush_workqueue(card->workqueue);
	lbtf_remove_card(priv);
reclaim:
	lbtf_deb_sdio("prob error jump: reclaim");
	sdio_claim_host(func);
release_int:
	lbtf_deb_sdio("prob error jump: release_int");
	sdio_release_irq(func);
disable:
	lbtf_deb_sdio("prob error jump: disable");
	sdio_disable_func(func);
release:
	lbtf_deb_sdio("prob error jump: release");
	sdio_release_host(func);
free:
	lbtf_deb_sdio("prob error jump: free");
	destroy_workqueue(card->workqueue);
	while (card->packets) {
		packet = card->packets;
		card->packets = card->packets->next;
		kfree(packet);
	}

	kfree(card);

	goto out;
}

static void if_sdio_remove(struct sdio_func *func)
{
	struct if_sdio_card *card;
	struct if_sdio_packet *packet;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	card = sdio_get_drvdata(func);

	lbtf_deb_sdio("call remove card\n");
//	lbtf_stop_card(card->priv);
	lbtf_remove_card(card->priv);
	card->priv->surpriseremoved = 1;

	flush_workqueue(card->workqueue);
	destroy_workqueue(card->workqueue);

	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	while (card->packets) {
		packet = card->packets;
		card->packets = card->packets->next;
		kfree(packet);
	}

	kfree(card);

	lbtf_deb_leave(LBTF_DEB_SDIO);
}

static int if_sdio_suspend(struct device *dev)
{
	return 0;
}

static int if_sdio_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops if_sdio_pm_ops = {
	.suspend	= if_sdio_suspend,
	.resume		= if_sdio_resume,
};

static struct sdio_driver if_sdio_driver = {
	.name		= "libertas_tf_sdio",
	.id_table	= if_sdio_ids,
	.probe		= if_sdio_probe,
	.remove		= if_sdio_remove,
	.drv = {
		.pm = &if_sdio_pm_ops,
	},
};

/*******************************************************************/
/* Module functions                                                */
/*******************************************************************/

static int __init if_sdio_init_module(void)
{
	int ret = 0;

	lbtf_deb_enter(LBTF_DEB_SDIO);

	printk(KERN_INFO "libertas_tf_sdio: Libertas Thinfirmware SDIO driver\n");
	printk(KERN_INFO "libertas_tf_sdio: Copyright cozybit Inc.\n");
	printk(KERN_INFO "libertas_tf_sdio: buildstamp: 6\n");

	ret = sdio_register_driver(&if_sdio_driver);

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static void __exit if_sdio_exit_module(void)
{
	lbtf_deb_enter(LBTF_DEB_SDIO);


	sdio_unregister_driver(&if_sdio_driver);

	lbtf_deb_leave(LBTF_DEB_SDIO);
}

module_init(if_sdio_init_module);
module_exit(if_sdio_exit_module);

MODULE_DESCRIPTION("Libertas_tf SDIO WLAN Driver");
MODULE_AUTHOR("Steve deRosier");
MODULE_LICENSE("GPL");
