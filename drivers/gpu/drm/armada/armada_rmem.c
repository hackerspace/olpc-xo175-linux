// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2017 Russell King
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>

static int armada_rmem_dev_init(struct reserved_mem *rmem, struct device *dev)
{
	struct resource *r;

	if (dev->platform_data)
		return -EBUSY;

	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	r->start = rmem->base;
	r->end = rmem->base + rmem->size - 1;
	r->flags = IORESOURCE_MEM;

	rmem->priv = r;
	dev->platform_data = r;

	return 0;
}

static void armada_rmem_dev_release(struct reserved_mem *rmem,
	struct device *dev)
{
	kfree(rmem->priv);
	rmem->priv = NULL;
	dev->platform_data = NULL;
}

static const struct reserved_mem_ops armada_rmem_ops = {
	.device_init = armada_rmem_dev_init,
	.device_release = armada_rmem_dev_release,
};

static int __init armada_rmem_init(struct reserved_mem *rmem)
{
	rmem->ops = &armada_rmem_ops;
	return 0;
}

RESERVEDMEM_OF_DECLARE(armada_rmem, "marvell,dove-framebuffer",
			armada_rmem_init);
