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
#include <linux/clk.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <mach/reset.h>
#include <mach/gpio.h>
#include <mach/smemc.h>
#include <mach/pxa3xx-regs.h>
#include <mach/part_table.h>
#include <mach/audio.h>

#include <plat/pxa3xx_onenand.h>
#include <plat/pxa3xx_nand.h>
#include <linux/delay.h>
#include <plat/mfp.h>
#include "generic.h"

void (*abu_mfp_init_func)(bool);
void (*ssp3_mfp_init_func)(bool);

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
	}, {
		/* Mem Ctl */
		.virtual	= SMEMC_VIRT,
		.pfn		= __phys_to_pfn(PXA3XX_SMEMC_BASE),
		.length		= 0x00200000,
		.type		= MT_DEVICE
	}
};

#if defined(CONFIG_MTD_NAND)
static struct pxa3xx_nand_platform_data nand_info = {
	.attr = ARBI_EN | NAKED_CMD,
	.num_cs = 1,
	.parts[0] = android_512m_4k_page_partitions,
	.nr_parts[0] = ARRAY_SIZE(android_512m_4k_page_partitions),
};


void nand_init(void)
{
	pxa3xx_set_nand_info(&nand_info);
}
#else
void nand_init(void) {}
#endif /* CONFIG_MTD_NAND */

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
	if (sync_enable) {
		onenand_sync_clk_cfg();
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

/* PMIC ID based on MICC= cmdline token get from OBM */
static long g_pmic_id = -1;
static int __init set_pmic_id(char *p)
{
	int ret;
	ret = strict_strtol(p, 16, &g_pmic_id);
	if (ret < 0) {
		printk(KERN_ERR "%s g_pmic_id is not right\n", __func__);
		return ret;
	}
	printk(KERN_INFO "%s g_pmic_id = %lx\n", __func__, g_pmic_id);
	return 1;
}
__setup("MICC=", set_pmic_id);

long get_pmic_id(void)
{
	return g_pmic_id;
}
EXPORT_SYMBOL(get_pmic_id);

void __init pxa_map_io(void)
{
	iotable_init(ARRAY_AND_SIZE(common_io_desc));

	if (!cpu_is_pxa2xx() || !cpu_is_pxa3xx() || !cpu_is_pxa93x())
		pxa_chip_id = __raw_readl(0xfb00ff80);
}

void __init set_abu_init_func(void (*func)(bool))
{
	if (func)
		abu_mfp_init_func = func;
	else
		printk(KERN_ERR "%s called with NULL pointer\n", __func__);
}

void pxa95x_abu_mfp_init(bool abu)
{
	if (abu_mfp_init_func)
		abu_mfp_init_func(abu);
	else
		panic("pxa95x_abu_mfp_init called with NULL pointer!\n");
}

void __init set_ssp_init_func(void (*func)(bool))
{
	if (func)
		ssp3_mfp_init_func = func;
	else
		printk(KERN_ERR "%s called with NULL pointer\n", __func__);
}

void pxa95x_ssp_mfp_init(bool bssp)
{
	if (ssp3_mfp_init_func)
		ssp3_mfp_init_func(bssp);
	else
		panic("pxa95x_abu_mfp_init called with NULL pointer!\n");
}


static unsigned int rfic_reset_gpio_pin = EINVAL;

void update_rfreset_gpio_num(unsigned int gpio_pin_num)
{
	rfic_reset_gpio_pin = gpio_pin_num;
}

void pxa9xx_platform_rfic_reset(unsigned short in_len,  void *in_buf,
				unsigned short out_len, void *out_buf)
{
	int uDelay = *((int *)in_buf);
	pr_info("%s: RFIC RPC gpio_request recieved, delay=%d!\n",
	__func__, uDelay);

	if (rfic_reset_gpio_pin == EINVAL) {
		pr_err("%s: invalid value of rfic_reset_gpio_pin\n",
		__func__);
		return;
	}
	if (gpio_request(rfic_reset_gpio_pin, "RFIC reset RPC")) {
		pr_err("RFIC RPC gpio_request: failed!\n");
		return;
	}
	gpio_direction_output(rfic_reset_gpio_pin, 0);
	udelay(uDelay);
	gpio_direction_output(rfic_reset_gpio_pin, 1);
	gpio_free(rfic_reset_gpio_pin);
}
EXPORT_SYMBOL(pxa9xx_platform_rfic_reset);
#ifdef CONFIG_PROC_FS

#define GEN_REG3		__REG(0x42404008)

static struct regulator *g_gps_regulator;
static int g_gps_reset, g_gps_on;
static int g_is_gps_on;
static struct clk *clk_tout_s0;

static void gps_eclk(int flag);

/* GPS: power on/off control */
static void gps_power_on(void)
{
	if (g_is_gps_on) {
		pr_err("%s: gps driver already on\n", __func__);
		return;
	}

	switch (get_board_id()) {
	case OBM_SAAR_B_MG1_C0_V12_BOARD:
	case OBM_SAAR_B_MG2_A0_V13_BOARD:
	case OBM_SAAR_B_MG2_A0_V14_BOARD:
	case OBM_SAAR_B_MG2_B0_V15_BOARD:
	case OBM_SAAR_B_MG2_C0_V26_BOARD:
		g_gps_reset = mfp_to_gpio(MFP_PIN_GPIO61);
		g_gps_on = mfp_to_gpio(MFP_PIN_GPIO62);
		break;
	case OBM_EVB_NEVO_1_2_BOARD:
	case OBM_SAAR_C3_NEVO_C0_V10_BOARD:
	case OBM_SAAR_C3_NEVO_C0_V10_BOARD_533MHZ:
	case OBM_DKB_2_NEVO_C0_BOARD:
		g_gps_reset = mfp_to_gpio(MFP_PIN_GPIO106);
		g_gps_on = mfp_to_gpio(MFP_PIN_GPIO104);
		break;
	default:
		g_gps_reset = 0;
		g_gps_on = 0;
		pr_err("%s: Unknown board type!\n", __func__);
		break;
	}

	if (gpio_request(g_gps_reset, "gpio_gps_reset")) {
		pr_err("%s: g_gps_reset request failed: %d\n", __func__, g_gps_reset);
		return;
	}
	pr_info("%s: g_gps_reset ok: %d\n", __func__, g_gps_reset);

	if (gpio_request(g_gps_on, "gpio_gps_on")) {
		pr_err("%s: gpio_gps_on request failed: %d\n", __func__, g_gps_on);
		goto exit1;
	}
	pr_info("%s: gpio_gps_on ok: %d\n", __func__, g_gps_on);

	g_gps_regulator = regulator_get(NULL, "v_gps");
	if (!g_gps_regulator) {
		pr_err("%s: regulator_get failed: v_gps\n", __func__);
		goto exit2;
	}
	pr_info("%s: regulator_get ok: v_gps\n", __func__);

	gpio_direction_output(g_gps_reset, 1);
	gpio_direction_output(g_gps_on, 0);
	gps_eclk(0);

	clk_tout_s0 = clk_get(NULL, "CLK_TOUT_S0");
	if (IS_ERR(clk_tout_s0)) {
		pr_err("%s: unable to get tout_s0 clock\n", __func__);
		goto exit3;
	}
	clk_enable(clk_tout_s0);

	if (regulator_enable(g_gps_regulator) < 0) {
		pr_err("%s: regulator_enable failed: v_gps\n", __func__);
		goto exit4;
	}

	g_is_gps_on = 1;
	pr_info("%s: sirf gps chip powered on\n", __func__);
	return;

exit4:
	regulator_put(g_gps_regulator);
	g_gps_regulator = NULL;
exit3:
	clk_put(clk_tout_s0);
	clk_tout_s0 = NULL;
exit2:
	gpio_free(g_gps_on);
	g_gps_on = (int) NULL;
exit1:
	gpio_free(g_gps_reset);
	g_gps_reset = (int) NULL;
}

static void gps_power_off(void)
{
	if (!g_is_gps_on) {
		pr_warning("%s: gps driver already off\n", __func__);
		/* In this case, do not return, release all resources. */
	}

	gps_eclk(0);

	if (clk_tout_s0) {
		clk_disable(clk_tout_s0);
		clk_put(clk_tout_s0);
	}

	if (g_gps_reset) {
		gpio_direction_input(g_gps_reset);
		gpio_free(g_gps_reset);
		g_gps_reset = (int) NULL;
	}

	if (g_gps_on) {
		gpio_direction_input(g_gps_on);
		gpio_free(g_gps_on);
		g_gps_on = (int) NULL;
	}

	if (g_gps_regulator) {
		if (regulator_disable(g_gps_regulator) < 0)
			pr_err("%s: regulator_disable failed: g_gps_regulator\n", __func__);
		regulator_put(g_gps_regulator);
		g_gps_regulator = NULL;
	}

	g_is_gps_on = 0;
	pr_info("%s: sirf gps chip powered off\n", __func__);
}

static void gps_reset(int flag)
{
	if (!g_gps_reset) {
		pr_err("%s: illegal handle, g_gps_reset: %d\n", __func__, g_gps_reset);
		return;
	}
	if ((flag != 0) && (flag != 1)) {
		pr_err("%s: illegal value, flag: %d\n", __func__, flag);
		return;
	}

	gpio_direction_output(g_gps_reset, flag);
	/*pr_info("%s: sirf gps chip reset\n", __func__);*/
}

static void gps_on_off(int flag)
{
	if (!g_gps_on) {
		pr_err("%s: illegal handle, g_gps_on: %d\n", __func__, g_gps_on);
		return;
	}
	if ((flag != 0) && (flag != 1)) {
		pr_err("%s: illegal value, flag: %d\n", __func__, flag);
		return;
	}
	gpio_direction_output(g_gps_on, flag);
	/*pr_info("%s: sirf gps chip offon\n", __func__);*/
}

static void gps_eclk(int flag)
{
	unsigned long reg_image;
	unsigned long cpsr;

	if ((flag != 0) && (flag != 1)) {
		pr_err("%s: illegal value, flag: %d\n", __func__, flag);
		return;
	}

	/* lock interrupts */
	local_irq_save(cpsr);

	/* read GEN_REG3 register */
	reg_image = GEN_REG3;

	/* ignore (clear) all bits, accept bits 0 & 1 () */
	reg_image &= 0x00000003;

	/* modify the value of CKRSW1 bit */
	if (flag)
		reg_image |= (1 << 16); /* set bit 16 (CKRSW1) */
	else
		reg_image &= ~(1 << 16); /* clear bit 16 (CKRSW1) */

	/* write GEN_REG3 register */
	GEN_REG3 = reg_image;

	/* unlock interrupts */
	local_irq_restore(cpsr);

	/*pr_info("%s: sirf gps chip eclk\n", __func__);*/
}

#define SIRF_STATUS_LEN	16
static char sirf_status[SIRF_STATUS_LEN] = "off";

static ssize_t sirf_read_proc(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len = strlen(sirf_status);

	sprintf(page, "%s\n", sirf_status);
	return len + 1;
}

static ssize_t sirf_write_proc(struct file *filp,
				const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];

	if (len > 255)
		len = 255;

	memset(messages, 0, sizeof(messages));

	if (!buff || copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strlen(messages) > (SIRF_STATUS_LEN - 1)) {
		pr_warning("%s: messages too long, (%d) %s\n",
			__func__, strlen(messages), messages);
		return -EFAULT;
	}

	if (strncmp(messages, "off", 3) == 0) {
		strcpy(sirf_status, "off");
		gps_power_off();
	} else if (strncmp(messages, "on", 2) == 0) {
		strcpy(sirf_status, "on");
		gps_power_on();
	} else if (strncmp(messages, "reset", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_reset(flag);
	} else if (strncmp(messages, "sirfon", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_on_off(flag);
	} else if (strncmp(messages, "eclk", 4) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_eclk(flag);
	} else
		pr_info("usage: echo [off|on|reset|sirfon|eclk] [0|1] > /proc/driver/sirf\n");

	return len;
}

void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file = NULL;

	sirf_proc_file = create_proc_entry("driver/sirf", 0644, NULL);
	if (!sirf_proc_file) {
		pr_err("%s: create proc file failed\n", __func__);
		return;
	}

	sirf_proc_file->read_proc = sirf_read_proc;
	sirf_proc_file->write_proc = (write_proc_t *)sirf_write_proc;
}
#endif
