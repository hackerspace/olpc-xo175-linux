/*
 *  linux/arch/arm/mach-mmp/sram.c
 *
 *  based on mach-davinci/sram.c - DaVinci simple SRAM allocator
 *
 *  Copyright (c) 2011 Marvell Semiconductors Inc.
 *  All Rights Reserved
 *
 *  Add for mmp2 audio sram support - Leo Yan <leoy@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <mach/sram.h>

static DEFINE_MUTEX(sram_lock);
static LIST_HEAD(sram_list);

static struct sram_bank *sram_get_avail(char *pool_name)
{
	struct sram_bank *sram_bank_slot;

	mutex_lock(&sram_lock);
	if (list_empty(&sram_list)) {
		pr_err("no available sram\n");
		mutex_unlock(&sram_lock);
		return NULL;
	}

	list_for_each_entry(sram_bank_slot, &sram_list,
			    sram_list) {
		if (memcmp(sram_bank_slot->pool_name, pool_name, 10) == 0)
			break;
	}

	if (&sram_bank_slot->sram_list == &sram_list) {
		pr_err("no matched sram found\n");
		mutex_unlock(&sram_lock);
		return NULL;
	}

	mutex_unlock(&sram_lock);
	return sram_bank_slot;
}

void *sram_alloc(char *pool_name, size_t len, dma_addr_t * dma)
{
	unsigned long vaddr;
	struct sram_bank *sram_bank_slot;
	dma_addr_t dma_base;

	sram_bank_slot = sram_get_avail(pool_name);
	if (sram_bank_slot == NULL) {
		pr_err("failed to alloc sram\n");
		return NULL;
	}

	dma_base = (dma_addr_t) sram_bank_slot->sram_phys;

	if (dma)
		*dma = 0;
	if (!sram_bank_slot->pool || (dma && !dma_base))
		return NULL;

	vaddr = gen_pool_alloc(sram_bank_slot->pool, len);
	if (!vaddr)
		return NULL;

	if (dma)
		*dma = dma_base + (vaddr -
				(unsigned long)sram_bank_slot->sram_virt);
	return (void *)vaddr;
}
EXPORT_SYMBOL(sram_alloc);

void sram_free(char *pool_name, void *addr, size_t len)
{
	struct sram_bank *sram_bank_slot;

	sram_bank_slot = sram_get_avail(pool_name);
	if (sram_bank_slot == NULL) {
		pr_err("failed to free sram\n");
		return;
	}

	gen_pool_free(sram_bank_slot->pool, (unsigned long)addr, len);
}
EXPORT_SYMBOL(sram_free);

static int __devinit sram_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct sram_bank *sram_bank_slot;
	int ret = 0;

	sram_bank_slot = pdev->dev.platform_data;
	if (sram_bank_slot == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto out;
	}
	sram_bank_slot->sram_phys = res->start;
	sram_bank_slot->sram_size = res->end - res->start + 1;
	sram_bank_slot->sram_virt =
	    ioremap(sram_bank_slot->sram_phys, sram_bank_slot->sram_size);

	sram_bank_slot->pool = gen_pool_create(ilog2(sram_bank_slot->step), -1);
	if (!sram_bank_slot->pool) {
		dev_err(&pdev->dev, "create pool failed\n");
		ret = -ENOMEM;
		goto create_pool_err;
	}

	ret = gen_pool_add(sram_bank_slot->pool,
			 (unsigned long)sram_bank_slot->sram_virt,
			 sram_bank_slot->sram_size, -1);
	if (ret < 0) {
		dev_err(&pdev->dev, "add new chunk failed\n");
		ret = -ENOMEM;
		goto add_chunk_err;
	}

	mutex_lock(&sram_lock);
	list_add(&sram_bank_slot->sram_list, &sram_list);
	mutex_unlock(&sram_lock);

	return 0;

add_chunk_err:
	gen_pool_destroy(sram_bank_slot->pool);
create_pool_err:
	iounmap(sram_bank_slot->sram_virt);
out:
	return ret;
}

static int __devexit sram_remove(struct platform_device *pdev)
{
	struct sram_bank *sram_bank_slot = pdev->dev.platform_data;
	if (sram_bank_slot == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	gen_pool_destroy(sram_bank_slot->pool);
	iounmap(sram_bank_slot->sram_virt);
	list_del(&sram_bank_slot->sram_list);

	return 0;
}

static struct platform_driver sram_driver = {
	.probe = sram_probe,
	.remove = sram_remove,
	.driver = {
		   .name = "mmp-sram",
		   },
};

static int __init sram_init(void)
{
	return platform_driver_register(&sram_driver);
}

static void __exit sram_exit(void)
{
	platform_driver_unregister(&sram_driver);
}

module_init(sram_init);
module_exit(sram_exit);
MODULE_LICENSE("GPL");
