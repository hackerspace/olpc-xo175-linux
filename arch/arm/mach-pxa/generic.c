/*
 *  linux/arch/arm/mach-pxa/generic.c
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 * Code common to all PXA machines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Since this file should be linked before any other machine specific file,
 * the __initcall() here will be executed first.  This serves as default
 * initialization stuff for PXA machines which can be overridden later if
 * need be.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <mach/reset.h>
#include <mach/gpio.h>
#include <mach/smemc.h>
#include <mach/pxa3xx-regs.h>
#include <mach/part_table.h>

#include <plat/pxa3xx_onenand.h>

#include "generic.h"

/* chip id is introduced from PXA95x */
unsigned int pxa_chip_id;
EXPORT_SYMBOL(pxa_chip_id);

void clear_reset_status(unsigned int mask)
{
	if (cpu_is_pxa2xx())
		pxa2xx_clear_reset_status(mask);
	else {
		/* RESET_STATUS_* has a 1:1 mapping with ARSR */
		ARSR = mask;
	}
}

unsigned long get_clock_tick_rate(void)
{
	unsigned long clock_tick_rate;

	if (cpu_is_pxa25x())
		clock_tick_rate = 3686400;
	else if (machine_is_mainstone())
		clock_tick_rate = 3249600;
	else
		clock_tick_rate = 3250000;

	return clock_tick_rate;
}
EXPORT_SYMBOL(get_clock_tick_rate);

/*
 * Get the clock frequency as reflected by CCCR and the turbo flag.
 * We assume these values have been applied via a fcs.
 * If info is not 0 we also display the current settings.
 */
unsigned int get_clk_frequency_khz(int info)
{
	if (cpu_is_pxa25x())
		return pxa25x_get_clk_frequency_khz(info);
	else if (cpu_is_pxa27x())
		return pxa27x_get_clk_frequency_khz(info);
	return 0;
}
EXPORT_SYMBOL(get_clk_frequency_khz);

/*
 * Intel PXA2xx internal register mapping.
 *
 * Note 1: virtual 0xfffe0000-0xffffffff is reserved for the vector table
 *       and cache flush area.
 *
 * Note 2: virtual 0xfb000000-0xfb00ffff is reserved for PXA95x
 *
 */
static struct map_desc common_io_desc[] __initdata = {
  	{	/* Devs */
		.virtual	=  0xf2000000,
		.pfn		= __phys_to_pfn(0x40000000),
		.length		= 0x02000000,
		.type		= MT_DEVICE
	}, {	/* Sys */
		.virtual	= 0xfb000000,
		.pfn		= __phys_to_pfn(0x46000000),
		.length		= 0x00010000,
		.type		= MT_DEVICE,
	}, {	/* UNCACHED_PHYS_0 */
		.virtual	= 0xff000000,
		.pfn		= __phys_to_pfn(0x00000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {
		/* Mem Ctl */
		.virtual	= SMEMC_VIRT,
		.pfn		= __phys_to_pfn(PXA3XX_SMEMC_BASE),
		.length		= 0x00200000,
		.type		= MT_DEVICE
	}
};

#if (defined(CONFIG_MTD_ONENAND) || defined(CONFIG_MTD_ONENAND_MODULE))

extern void onenand_mmcontrol_smc_cfg(void);
extern void onenand_sync_clk_cfg(void);

static void __attribute__ ((unused)) onenand_mmcontrol(struct mtd_info *mtd, int sync_read)
{
	struct onenand_chip *this = mtd->priv;
	unsigned int syscfg;

	if (sync_read) {
		onenand_mmcontrol_smc_cfg();
		syscfg = this->read_word(this->base + ONENAND_REG_SYS_CFG1);
		syscfg &= (~(0x07<<9));
		/* 16 words for one burst */
		syscfg |= 0x03<<9;
		this->write_word((syscfg | sync_read), this->base + ONENAND_REG_SYS_CFG1);
	} else {
		syscfg = this->read_word(this->base + ONENAND_REG_SYS_CFG1);
		this->write_word((syscfg & ~sync_read), this->base + ONENAND_REG_SYS_CFG1);
	}
}

static struct pxa3xx_onenand_platform_data onenand_platinfo;
static int set_partition_info(u32 flash_size, u32 page_size, struct pxa3xx_onenand_platform_data *pdata)
{
	int found = -EINVAL;
	if (256 == flash_size) {
		pdata->parts = android_256m_4k_page_partitions;
		pdata->nr_parts = ARRAY_SIZE(android_256m_4k_page_partitions);
		found = 0;
	} else if (512 == flash_size) {
		pdata->parts = android_512m_4k_page_partitions;
		pdata->nr_parts = ARRAY_SIZE(android_512m_4k_page_partitions);
		found = 0;
	}

	if (0 != found)
		printk(KERN_ERR"***************no proper partition table *************\n");

	return found;
}
void onenand_init(int sync_enable)
{
	u32 temp;
	if (sync_enable) {
		onenand_sync_clk_cfg();
		temp  = ACCR;
		/*
		* bit25~bit23
		* 000 78Mhz, 010 104Mhz, 100 156Mhz
		*/
		temp &= (~(7 << 23));
		temp |= 0x02<<23;
		ACCR = temp;  /*106Mhz*/
		onenand_platinfo.mmcontrol = onenand_mmcontrol;
	}
	onenand_platinfo.set_partition_info = set_partition_info;
	pxa3xx_set_onenand_info(&onenand_platinfo);
}
#else
void onenand_init(int sync_enable) {}
#endif


/* Board ID based on BOAR= cmdline token get from OBM */
static long g_board_id = -1;
static int __init set_board_id(char *p)
{
	int ret;
	ret = strict_strtol(p, 16, &g_board_id);
	if (ret < 0) {
		printk(KERN_ERR "%s g_board_id is not right\n", __func__);
		return ret;
	}
	printk(KERN_INFO "%s g_board_id = %ld\n", __func__, g_board_id);
	return 1;
}
__setup("BOAR=", set_board_id);

long get_board_id(void)
{
	return g_board_id;
}
EXPORT_SYMBOL(get_board_id);

void __init pxa_map_io(void)
{
	iotable_init(ARRAY_AND_SIZE(common_io_desc));

	if (!cpu_is_pxa2xx() || !cpu_is_pxa3xx() || !cpu_is_pxa93x())
		pxa_chip_id = __raw_readl(0xfb00ff80);
}
