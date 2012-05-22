/*
 *  linux/arch/arm/mach-mmp/common.c
 *
 *  Code common to PXA168 processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/page.h>
#include <asm/mach/map.h>
#include <mach/addr-map.h>
#include <mach/cputype.h>

#ifdef CONFIG_CPU_MMP2
#include <mach/mmp2_pm.h>
#endif
#ifdef CONFIG_CPU_MMP3
#include <mach/mmp_audisland.h>
#endif

#include "common.h"

#define MMP_CHIPID	(AXI_VIRT_BASE + 0x82c00)
#define MMP_FUSE_95_64	(AXI_VIRT_BASE + 0x1498)
#define MMP_FUSE_127_96	(AXI_VIRT_BASE + 0x149c)

unsigned int mmp_chip_id;
EXPORT_SYMBOL(mmp_chip_id);

unsigned int mmp_fuse_id;
EXPORT_SYMBOL(mmp_fuse_id);

unsigned int mmp_1g_svc;
EXPORT_SYMBOL(mmp_1g_svc);

int mmp2_platform_version;

static struct map_desc standard_io_desc[] __initdata = {
	{
		.pfn		= __phys_to_pfn(APB_PHYS_BASE),
		.virtual	= APB_VIRT_BASE,
		.length		= APB_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn		= __phys_to_pfn(AXI_PHYS_BASE),
		.virtual	= AXI_VIRT_BASE,
		.length		= AXI_PHYS_SIZE,
		.type		= MT_DEVICE,
#ifdef CONFIG_CPU_MMP2
	}, {
		.pfn		= __phys_to_pfn(FC_PHYS_BASE),
		.virtual	= FC_VIRT_BASE,
		.length		= FC_PHYS_SIZE,
		.type		= MT_MEMORY_NONCACHED,
#endif
	}, {
		.pfn		= __phys_to_pfn(DMCU_PHYS_BASE),
		.virtual	= DMCU_VIRT_BASE,
		.length		= DMCU_PHYS_SIZE,
		.type		= MT_DEVICE,
#ifdef CONFIG_CPU_MMP3
	}, {
		.pfn		= __phys_to_pfn(PGU_PHYS_BASE),
		.virtual	= PGU_VIRT_BASE,
		.length		= PGU_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn            = __phys_to_pfn(AUD_PHYS_BASE),
		.virtual        = AUD_VIRT_BASE,
		.length         = AUD_PHYS_SIZE,
		.type           = MT_DEVICE,
	}, {
		.pfn            = __phys_to_pfn(AUD_PHYS_BASE2),
		.virtual        = AUD_VIRT_BASE2,
		.length         = AUD_PHYS_SIZE2,
		.type           = MT_DEVICE,
	}, {
		.pfn            = __phys_to_pfn(TZ_HV_PHYS_BASE),
		.virtual        = TZ_HV_VIRT_BASE,
		.length         = TZ_HV_PHYS_SIZE,
		.type           = MT_MEMORY_NONCACHED,
#endif
	},
};

void __init mmp_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));

	/* this is early, initialize mmp_chip_id here */
	mmp_chip_id = __raw_readl(MMP_CHIPID);
#ifdef CONFIG_CPU_PXA910
	mmp_fuse_id = __raw_readl(MMP_FUSE_95_64);
	mmp_1g_svc = ((__raw_readl(MMP_FUSE_127_96) & 0x00003FFF) << 6)
			| ((__raw_readl(MMP_FUSE_95_64) & 0xFC000000) >> 26);
#endif
}
