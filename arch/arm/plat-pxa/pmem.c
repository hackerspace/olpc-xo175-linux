/*
 *  arch/arm/plat-pxa/pmem.c
 *
 *  Buffer Management Module
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
#include <linux/memblock.h>

#ifdef CONFIG_ANDROID_PMEM
#include <plat/pmem.h>

/*default reserve size:16MB*/
static size_t __initdata pmem_reserve_size = 0x1000000;
static unsigned long __initdata pmem_reserve_pa;

static int __init pxa_reserve_early_init(char *arg)
{
	pmem_reserve_size = memparse(arg, NULL);
	return 0;
}
early_param("reserve_pmem", pxa_reserve_early_init);

static void __init  __pxa_add_pmem(char *name, size_t size, int no_allocator,
				int cached, int buffered)
{
	struct platform_device *android_pmem_device;
	struct android_pmem_platform_data *android_pmem_pdata;
	static int id;

	if (size > PAGE_SIZE && size > pmem_reserve_size)
		return;

	android_pmem_device = kzalloc(sizeof(struct platform_device),
					GFP_KERNEL);
	if (android_pmem_device == NULL)
		return ;

	android_pmem_pdata = kzalloc(sizeof(struct android_pmem_platform_data),
					GFP_KERNEL);
	if (android_pmem_pdata == NULL) {
		kfree(android_pmem_device);
		return ;
	}

	if (pmem_reserve_pa == 0)
		return;

	if (size > PAGE_SIZE) {
		android_pmem_pdata->start = pmem_reserve_pa;
		android_pmem_pdata->size = size;
		pmem_reserve_pa += size;
		pmem_reserve_size -= size;
	} else {
		android_pmem_pdata->start = size;
		android_pmem_pdata->size = 0;
	}

	android_pmem_pdata->name = name;
	android_pmem_pdata->no_allocator = no_allocator;
	android_pmem_pdata->cached = cached;
	android_pmem_pdata->buffered = buffered;

	android_pmem_device->name = "android_pmem";
	android_pmem_device->id = id++;
	android_pmem_device->dev.platform_data = android_pmem_pdata;

	platform_device_register(android_pmem_device);
	printk(KERN_INFO "pmem register %s reserve pa(0x%lx), request size=0x%x\n",
		name, pmem_reserve_pa, size);
}

void __init pxa_reserve_pmem_memblock(void)
{
	pmem_reserve_pa = memblock_alloc(pmem_reserve_size, PAGE_SIZE);
	if (!pmem_reserve_pa) {
		pr_err("%s: failed to reserve %x bytes\n",
				__func__, pmem_reserve_size);
		return;
	}
	/* FIXME:
	 * - memblock_free: remove the allocated buffer from the reserved
	 *   region, Which means it will be not reserved.
	 * - memblock_remove: remove the allocated buffer from the memory
	 *   available to kernel, and the pages will not be mapped.
	 *
	 * ARM DMA APIs requires the DMA buffer pages to be mapped in kernel,
	 * or it will do cache flush on the invalid virtual address.
	 */
#if 0
	memblock_free(pmem_reserve_pa, pmem_reserve_size);
	memblock_remove(pmem_reserve_pa, pmem_reserve_size);
#endif
}

void __init pxa_add_pmem(void)
{
	__pxa_add_pmem("pmem", pmem_reserve_size, 0, 1, 1);
	__pxa_add_pmem("pmem_adsp", 0, 0, 0, 0);
}

#endif
