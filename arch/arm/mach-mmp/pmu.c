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
#include <asm/io.h>
#include <mach/addr-map.h>
#include <mach/cputype.h>
#include <mach/irqs.h>

#define CORESIGHT_VIRT_BASE	(APB_VIRT_BASE + 0x100000)
#define CTI_CORE0_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x18000)
#define CTI_CORE1_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x19000)
#define CTI_SOC_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x6000)

#define CTI_CTRL		0x0
#define CTI_EN_MASK		0x0F
#define CTI_EN_IN0		0x20
#define CTI_EN_IN1		0x24
#define CTI_EN_IN2		0x28
#define CTI_EN_IN3		0x2C
#define CTI_EN_IN4		0x30
#define CTI_EN_IN5		0x34
#define CTI_EN_IN6		0x38
#define CTI_EN_IN7		0x3C
#define CTI_EN_OUT0		0xA0
#define CTI_EN_OUT1		0xA4
#define CTI_EN_OUT2		0xA8
#define CTI_EN_OUT3		0xAC
#define CTI_EN_OUT4		0xB0
#define CTI_EN_OUT5		0xB4
#define CTI_EN_OUT6		0xB8
#define CTI_EN_OUT7		0xBC


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

static struct resource pmu_resource_pxa988[] = {
	/* core0 */
	{
		.start	= IRQ_PXA988_CORESIGHT,
		.end	= IRQ_PXA988_CORESIGHT,
		.flags	= IORESOURCE_IRQ,
	},
	/* core1*/
	{
		.start	= IRQ_PXA988_CORESIGHT2,
		.end	= IRQ_PXA988_CORESIGHT2,
		.flags	= IORESOURCE_IRQ,
	},
};


static struct platform_device pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
};

static void __init pxa988_cti_init(void)
{
	u32 tmp;

	/* enable CTI for core0 & core1*/
	__raw_writel(0x1, CTI_CORE0_VIRT_BASE + CTI_CTRL);
	__raw_writel(0x1, CTI_CORE1_VIRT_BASE + CTI_CTRL);

	/* enable CTI for SOC */
	__raw_writel(0x1, CTI_SOC_VIRT_BASE + CTI_CTRL);

	/*
	 * enable core0 CTI triger in 1 from PMU0 irq to CTM channel 3
	 * and enable the CTM channel 3 route to core0 CTI trigger out 6
	 */
	tmp = __raw_readl(CTI_CORE0_VIRT_BASE + CTI_EN_IN1);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x8;
	__raw_writel(tmp, CTI_CORE0_VIRT_BASE + CTI_EN_IN1);

	tmp = __raw_readl(CTI_CORE0_VIRT_BASE + CTI_EN_OUT6);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x8;
	__raw_writel(tmp, CTI_CORE0_VIRT_BASE + CTI_EN_OUT6);

	/*
	 * enable core1 CTI triger in 1 from PMU1 irq to CTM channel 0
	 * and enable the CTM channel 0 route to core1 CTI trigger out 6
	 */
	tmp = __raw_readl(CTI_CORE1_VIRT_BASE + CTI_EN_IN1);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x1;
	__raw_writel(tmp, CTI_CORE1_VIRT_BASE + CTI_EN_IN1);

	tmp = __raw_readl(CTI_CORE1_VIRT_BASE + CTI_EN_OUT6);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x1;
	__raw_writel(tmp, CTI_CORE1_VIRT_BASE + CTI_EN_OUT6);
}

static int __init pxa_pmu_init(void)
{
	if (cpu_is_mmp3()) {
		pmu_device.resource = pmu_resource_mmp3;
		pmu_device.num_resources = ARRAY_SIZE(pmu_resource_mmp3);
	} else if (cpu_is_pxa988()) {
		pmu_device.resource = pmu_resource_pxa988;
		pmu_device.num_resources = ARRAY_SIZE(pmu_resource_pxa988);

		/* Need to init CTI irq line */
		pxa988_cti_init();
	} else {
		printk(KERN_WARNING "unsupported Soc for PMU");
		return -EIO;
	}

	platform_device_register(&pmu_device);
	return 0;
}
arch_initcall(pxa_pmu_init);
