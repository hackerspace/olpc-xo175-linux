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


	if (ret)
		pr_err("failed to load helper firmware\n");

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_prog_real(struct if_sdio_card *card)
{
	int ret;

	lbtf_deb_enter(LBTF_DEB_SDIO);


	if (ret)
		pr_err("failed to load firmware\n");

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_prog_firmware(struct if_sdio_card *card)
{
	int ret;

	lbtf_deb_enter(LBTF_DEB_SDIO);



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

/*******************************************************************/
/* SDIO callbacks                                                  */
/*******************************************************************/

static void if_sdio_interrupt(struct sdio_func *func)
{
	int ret;

	lbtf_deb_enter(LBTF_DEB_SDIO);


	ret = 0;

	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);
}

static int if_sdio_probe(struct sdio_func *func,
		const struct sdio_device_id *id)
{
	int ret, i;

	lbtf_deb_enter(LBTF_DEB_SDIO);


	lbtf_deb_leave_args(LBTF_DEB_SDIO, "ret %d", ret);

	return ret;

}

static void if_sdio_remove(struct sdio_func *func)
{
	lbtf_deb_enter(LBTF_DEB_SDIO);



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
	printk(KERN_INFO "libertas_tf_sdio: Copyright cozybit\n");

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
