/*
 * PMU IRQ registration for MMP PMU families.
 *
 * (C) Copyright 2011 Marvell International Ltd.
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/pmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>

/* NOTE: Order fixed as mp1/mp2/mm core id = 0, 1, 2 */
static struct resource pmu_resource_mmp3[] = {
	/* MP1 = 0 */
	{
		.start	= IRQ_MMP3_PMU_INT0,
		.end	= IRQ_MMP3_PMU_INT0,
		.flags	= IORESOURCE_IRQ,
	},
	/* MP2 = 1*/
	{
		.start	= IRQ_MMP3_PMU_INT1,
		.end	= IRQ_MMP3_PMU_INT1,
		.flags	= IORESOURCE_IRQ,
	},
	/* mm  = 2*/
	{
		.start	= IRQ_MMP3_PMU_INT2,
		.end	= IRQ_MMP3_PMU_INT2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
};

static int __init pxa_pmu_init(void)
{
	if (cpu_is_mmp3()) {
		pmu_device.resource = pmu_resource_mmp3;
		pmu_device.num_resources = ARRAY_SIZE(pmu_resource_mmp3);
	} else {
		printk(KERN_WARNING "unsupported Soc for PMU");
		return -EIO;
	}

	platform_device_register(&pmu_device);
	return 0;
}
arch_initcall(pxa_pmu_init);
