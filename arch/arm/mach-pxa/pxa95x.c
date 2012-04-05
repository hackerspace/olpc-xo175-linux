/*
 * linux/arch/arm/mach-pxa/pxa95x.c
 *
 * code specific to PXA95x aka MGx
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/i2c/pxa-i2c.h>
#include <linux/irq.h>
#include <linux/syscore_ops.h>
#include <linux/memblock.h>
#include <linux/sysdev.h>

#include <asm/hardware/cache-tauros2.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa930.h>
#include <mach/reset.h>
#include <mach/dma.h>
#include <mach/regs-intc.h>
#include <mach/soc_vmeta.h>
#include <mach/usb-regs.h>
#include <mach/pxa95x_dvfm.h>

#include <linux/uio_vmeta.h>

#include <plat/pmem.h>

#include "generic.h"
#include "devices.h"

#define PECR_IE(n)	((1 << ((n) * 2)) << 28)
#define PECR_IS(n)	((1 << ((n) * 2)) << 29)

static int boot_flash_type;
int pxa_boot_flash_type_get(void)
{
	return boot_flash_type;
}

static int __init setup_boot_flash_type(char *p)
{
	boot_flash_type  = memparse(p, &p);
	printk(KERN_INFO "setup_boot_flash_type: boot_flash_type=%d",
		boot_flash_type);
	return 1;
}
__setup("FLAS=", setup_boot_flash_type);

static struct mfp_addr_map pxa95x_mfp_addr_map[] __initdata = {

	MFP_ADDR(GPIO0, 0x02e0),
	MFP_ADDR(GPIO1, 0x02dc),
	MFP_ADDR(GPIO2, 0x02e8),
	MFP_ADDR(GPIO3, 0x02d8),
	MFP_ADDR(GPIO4, 0x02e4),
	MFP_ADDR(GPIO5, 0x02ec),
	MFP_ADDR(GPIO6, 0x02f8),
	MFP_ADDR(GPIO7, 0x02fc),
	MFP_ADDR(GPIO8, 0x0300),
	MFP_ADDR(GPIO9, 0x02d4),
	MFP_ADDR(GPIO10, 0x02f4),
	MFP_ADDR(GPIO11, 0x02f0),
	MFP_ADDR(GPIO12, 0x0304),
	MFP_ADDR(GPIO13, 0x0310),
	MFP_ADDR(GPIO14, 0x0308),
	MFP_ADDR(GPIO15, 0x030c),
	MFP_ADDR(GPIO16, 0x04e8),
	MFP_ADDR(GPIO17, 0x04f4),
	MFP_ADDR(GPIO18, 0x04f8),
	MFP_ADDR(GPIO19, 0x04fc),
	MFP_ADDR(GPIO20, 0x0518),
	MFP_ADDR(GPIO21, 0x051c),
	MFP_ADDR(GPIO22, 0x04ec),
	MFP_ADDR(GPIO23, 0x0500),
	MFP_ADDR(GPIO24, 0x04f0),
	MFP_ADDR(GPIO25, 0x0504),
	MFP_ADDR(GPIO26, 0x0510),
	MFP_ADDR(GPIO27, 0x0514),
	MFP_ADDR(GPIO28, 0x0520),
	MFP_ADDR(GPIO29, 0x0600),
	MFP_ADDR(GPIO30, 0x0618),
	MFP_ADDR(GPIO31, 0x0610),
	MFP_ADDR(GPIO32, 0x060c),
	MFP_ADDR(GPIO33, 0x061c),
	MFP_ADDR(GPIO34, 0x0620),
	MFP_ADDR(GPIO35, 0x0628),
	MFP_ADDR(GPIO36, 0x062c),
	MFP_ADDR(GPIO37, 0x0630),
	MFP_ADDR(GPIO38, 0x0634),
	MFP_ADDR(GPIO39, 0x0638),
	MFP_ADDR(GPIO40, 0x063c),
	MFP_ADDR(GPIO41, 0x0614),
	MFP_ADDR(GPIO42, 0x0624),
	MFP_ADDR(GPIO43, 0x0608),
	MFP_ADDR(GPIO44, 0x0604),
	MFP_ADDR(GPIO45, 0x050c),
	MFP_ADDR(GPIO46, 0x0508),
	MFP_ADDR(GPIO47, 0x02bc),
	MFP_ADDR(GPIO48, 0x02b4),
	MFP_ADDR(GPIO49, 0x02b8),
	MFP_ADDR(GPIO50, 0x02c8),
	MFP_ADDR(GPIO51, 0x02c0),
	MFP_ADDR(GPIO52, 0x02c4),
	MFP_ADDR(GPIO53, 0x02d0),
	MFP_ADDR(GPIO54, 0x02cc),
	MFP_ADDR(GPIO55, 0x029c),
	MFP_ADDR(GPIO56, 0x02a0),
	MFP_ADDR(GPIO57, 0x0294),
	MFP_ADDR(GPIO58, 0x0298),
	MFP_ADDR(GPIO59, 0x02a4),
	MFP_ADDR(GPIO60, 0x02a8),
	MFP_ADDR(GPIO61, 0x02b0),
	MFP_ADDR(GPIO62, 0x02ac),
	MFP_ADDR(GPIO63, 0x0640),
	MFP_ADDR(GPIO64, 0x065c),
	MFP_ADDR(GPIO65, 0x0648),
	MFP_ADDR(GPIO66, 0x0644),
	MFP_ADDR(GPIO67, 0x0674),
	MFP_ADDR(GPIO68, 0x0658),
	MFP_ADDR(GPIO69, 0x0654),
	MFP_ADDR(GPIO70, 0x0660),
	MFP_ADDR(GPIO71, 0x0668),
	MFP_ADDR(GPIO72, 0x0664),
	MFP_ADDR(GPIO73, 0x0650),
	MFP_ADDR(GPIO74, 0x066c),
	MFP_ADDR(GPIO75, 0x064c),
	MFP_ADDR(GPIO76, 0x0670),
	MFP_ADDR(GPIO77, 0x0678),
	MFP_ADDR(GPIO78, 0x067c),
	MFP_ADDR(GPIO79, 0x0694),
	MFP_ADDR(GPIO80, 0x069c),
	MFP_ADDR(GPIO81, 0x06a0),
	MFP_ADDR(GPIO82, 0x06a4),
	MFP_ADDR(GPIO83, 0x0698),
	MFP_ADDR(GPIO84, 0x06bc),
	MFP_ADDR(GPIO85, 0x06b4),
	MFP_ADDR(GPIO86, 0x06b0),
	MFP_ADDR(GPIO87, 0x06c0),
	MFP_ADDR(GPIO88, 0x06c4),
	MFP_ADDR(GPIO89, 0x06ac),
	MFP_ADDR(GPIO90, 0x0680),
	MFP_ADDR(GPIO91, 0x0684),
	MFP_ADDR(GPIO92, 0x0688),
	MFP_ADDR(GPIO93, 0x0690),
	MFP_ADDR(GPIO94, 0x068c),
	MFP_ADDR(GPIO95, 0x06a8),
	MFP_ADDR(GPIO96, 0x06b8),
	MFP_ADDR(GPIO97, 0x0418),
	MFP_ADDR(GPIO98, 0x0410),
	MFP_ADDR(GPIO99, 0x041c),
	MFP_ADDR(GPIO100, 0x0414),
	MFP_ADDR(GPIO101, 0x0408),
	MFP_ADDR(GPIO102, 0x0324),
	MFP_ADDR(GPIO103, 0x040c),
	MFP_ADDR(GPIO104, 0x0400),
	MFP_ADDR(GPIO105, 0x0328),
	MFP_ADDR(GPIO106, 0x0404),

	MFP_ADDR(GPIO159, 0x0524),
	MFP_ADDR(GPIO163, 0x0534),
	MFP_ADDR(GPIO167, 0x0544),
	MFP_ADDR(GPIO168, 0x0548),
	MFP_ADDR(GPIO169, 0x054c),
	MFP_ADDR(GPIO170, 0x0550),
	MFP_ADDR(GPIO171, 0x0554),
	MFP_ADDR(GPIO172, 0x0558),
	MFP_ADDR(GPIO173, 0x055c),

	MFP_ADDR(nXCVREN, 0x0204),
	MFP_ADDR(DF_CLE_nOE, 0x020c),
	MFP_ADDR(DF_nADV1_ALE, 0x0218),
	MFP_ADDR(DF_SCLK_E, 0x0214),
	MFP_ADDR(DF_SCLK_S, 0x0210),
	MFP_ADDR(nBE0, 0x021c),
	MFP_ADDR(nBE1, 0x0220),
	MFP_ADDR(DF_nADV2_ALE, 0x0224),
	MFP_ADDR(DF_INT_RnB, 0x0228),
	MFP_ADDR(DF_nCS0, 0x022c),
	MFP_ADDR(DF_nCS1, 0x0230),
	MFP_ADDR(nLUA, 0x0254),
	MFP_ADDR(nLLA, 0x0258),
	MFP_ADDR(DF_nWE, 0x0234),
	MFP_ADDR(DF_nRE_nOE, 0x0238),
	MFP_ADDR(DF_ADDR0, 0x024c),
	MFP_ADDR(DF_ADDR1, 0x0250),
	MFP_ADDR(DF_ADDR2, 0x025c),
	MFP_ADDR(DF_ADDR3, 0x0260),
	MFP_ADDR(DF_IO0, 0x023c),
	MFP_ADDR(DF_IO1, 0x0240),
	MFP_ADDR(DF_IO2, 0x0244),
	MFP_ADDR(DF_IO3, 0x0248),
	MFP_ADDR(DF_IO4, 0x0264),
	MFP_ADDR(DF_IO5, 0x0268),
	MFP_ADDR(DF_IO6, 0x026c),
	MFP_ADDR(DF_IO7, 0x0270),
	MFP_ADDR(DF_IO8, 0x0274),
	MFP_ADDR(DF_IO9, 0x0278),
	MFP_ADDR(DF_IO10, 0x027c),
	MFP_ADDR(DF_IO11, 0x0280),
	MFP_ADDR(DF_IO12, 0x0284),
	MFP_ADDR(DF_IO13, 0x0288),
	MFP_ADDR(DF_IO14, 0x028c),
	MFP_ADDR(DF_IO15, 0x0290),

	MFP_ADDR(GSIM_UIO, 0x0314),
	MFP_ADDR(GSIM_UCLK, 0x0318),
	MFP_ADDR(GSIM_UDET, 0x031c),
	MFP_ADDR(GSIM_nURST, 0x0320),

	MFP_ADDR(PMIC_INT, 0x06c8),

	MFP_ADDR(RDY, 0x0200),

	MFP_ADDR_END,
};

static struct mfp_addr_map pxa978_mfp_addr_map[] __initdata = {
	/*PMIC interrupt pin*/
	MFP_ADDR(PMIC_INT, 0x204),

	/* MFP Pins*/
	MFP_ADDR_X(GPIO0, GPIO132, 0x208),

	/* RF MFP Pins */
	MFP_ADDR_X(RF_MFP0, RF_MFP30, 0x460),

	/* MEM MFP Pins */
	MFP_ADDR_X(MEM_MFP0, MEM_MFP39, 0x500),

	MFP_ADDR_END,
};

static void pxa_ack_ext_wakeup(struct irq_data *d)
{
	PECR |= PECR_IS(d->irq - IRQ_WAKEUP0);
}

static void pxa_mask_ext_wakeup(struct irq_data *d)
{
	ICMR2 &= ~(1 << ((d->irq - PXA_IRQ(0)) & 0x1f));
	PECR &= ~PECR_IE(d->irq - IRQ_WAKEUP0);
}

static void pxa_unmask_ext_wakeup(struct irq_data *d)
{
	ICMR2 |= 1 << ((d->irq - PXA_IRQ(0)) & 0x1f);
	PECR |= PECR_IE(d->irq - IRQ_WAKEUP0);
}

static int pxa_set_ext_wakeup_type(struct irq_data *d, unsigned int flow_type)
{
	if (flow_type & IRQ_TYPE_EDGE_RISING)
		PWER |= 1 << (d->irq - IRQ_WAKEUP0);

	if (flow_type & IRQ_TYPE_EDGE_FALLING)
		PWER |= 1 << (d->irq - IRQ_WAKEUP0 + 2);

	return 0;
}

static struct irq_chip pxa_ext_wakeup_chip = {
	.name           = "WAKEUP",
	.irq_ack            = pxa_ack_ext_wakeup,
	.irq_mask           = pxa_mask_ext_wakeup,
	.irq_unmask         = pxa_unmask_ext_wakeup,
	.irq_set_type       = pxa_set_ext_wakeup_type,
};

/*
 *  * For pxa95x, it's not necessary to use irq to wakeup system.
 *   */
int pxa95x_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}

static void __init pxa_init_ext_wakeup_irq(set_wake_t fn)
{
	irq_set_chip_and_handler(IRQ_WAKEUP0, &pxa_ext_wakeup_chip,
			handle_edge_irq);
	set_irq_flags(IRQ_WAKEUP0, IRQF_VALID);
	pxa_ext_wakeup_chip.irq_set_wake = fn;
}

void __init pxa95x_init_irq(void)
{
	pxa_init_irq(96, pxa95x_set_wake);
	if (!cpu_is_pxa978())
		pxa_init_ext_wakeup_irq(pxa95x_set_wake);
	pxa_init_gpio(IRQ_GPIO_2_x, 2, 191, pxa95x_set_wake);
}

/*
 * device registration specific to PXA93x.
 */

void __init pxa95x_set_i2c_power_info(struct i2c_pxa_platform_data *info)
{
	pxa_register_device(&pxa3xx_device_i2c_power, info);
}

#if defined(CONFIG_UIO_VMETA)
static struct vmeta_plat_data vmeta_plat_data = {
	.bus_irq_handler = pxa95x_vmeta_bus_irq_handler,
	.axi_clk_available = 1,
	.power_down_ms = 10,
};
#endif /*(CONFIG_UIO_VMETA)*/

static struct platform_device *devices[] __initdata = {
	&sa1100_device_rtc,
	&pxa_device_rtc,
	&pxa27x_device_ssp1,
	&pxa27x_device_ssp2,
	&pxa27x_device_ssp3,
	&pxa3xx_device_ssp4,
	&pxa27x_device_pwm0,
	&pxa27x_device_pwm1,
	&pxa95x_device_pwm4,
	&pxa95x_device_pwm5,
	&pxa95x_device_pwm6,
	&pxa95x_device_pwm7,
	&pxa_device_asoc_abu,
	&pxa_device_asoc_ssp2,
	&pxa_device_asoc_ssp3,
	&pxa_device_asoc_abu_platform,
	&pxa_device_asoc_platform,
	&pxa_device_asoc_hdmi_codec,
#if defined(CONFIG_PXA9XX_ACIPC)
	&pxa930_acipc_device,
#endif /*CONFIG_PXA9XX_ACIPC*/
#if defined(CONFIG_TOUCHSCREEN_VNC)
	&vnc_device,
#endif
};

struct pxa95x_freq_mach_info freq_mach_info = {
	.flags = PXA95x_USE_POWER_I2C,
};

static int __init pxa95x_init(void)
{
	int ret = 0;

	/* dvfm device */
#ifdef CONFIG_PXA95x_DVFM
	set_pxa95x_freq_info(&freq_mach_info);
#endif

#ifdef CONFIG_CACHE_TAUROS2
	if (!cpu_is_pxa978_Cx())
		tauros2_init();
#endif

#ifdef CONFIG_CACHE_L2X0
	if (cpu_is_pxa978_Cx()) {
		void *l2x0_base = ioremap_nocache(0x58120000, 0x1000);
		if (!l2x0_base)
			return -ENOMEM;
		/* Args 1,2: don't change AUX_CTRL */
		l2x0_init(l2x0_base, 0, ~0);
	}
#endif
	mfp_init_base(io_p2v(MFPR_BASE));
	if (cpu_is_pxa978())
		mfp_init_addr(pxa978_mfp_addr_map);
	else
		mfp_init_addr(pxa95x_mfp_addr_map);

	reset_status = ARSR;

	/*
	 * clear RDH bit every time after reset
	 *
	 * Note: the last 3 bits DxS are write-1-to-clear so carefully
	 * preserve them here in case they will be referenced later
	 */
	ASCR &= ~(ASCR_RDH | ASCR_D1S | ASCR_D2S | ASCR_D3S);

	if ((ret = pxa_init_dma(IRQ_DMA, 32)))
		return ret;

	register_syscore_ops(&pxa_irq_syscore_ops);
	register_syscore_ops(&pxa_gpio_syscore_ops);

	ret = platform_add_devices(devices, ARRAY_SIZE(devices));

	pxa_set_ffuart_info(NULL);
	pxa_set_stuart_info(NULL);

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif
#if defined(CONFIG_UIO_VMETA)
	pxa95x_set_vmeta_info(&vmeta_plat_data);
#endif
	return ret;
}

postcore_initcall(pxa95x_init);

#ifdef CONFIG_USB_PXA_U2O
unsigned u2o_get(unsigned base, unsigned offset)
{
	return readl(base + offset);
}

void u2o_set(unsigned base, unsigned offset, unsigned value)
{
	volatile unsigned int reg;
	reg = readl(base + offset);
	reg |= value;
	writel(reg, base + offset);
	__raw_readl(base + offset);
}

void u2o_clear(unsigned base, unsigned offset, unsigned value)
{
	volatile unsigned int reg;
	reg = readl(base + offset);
	reg &= ~value;
	writel(reg, base + offset);
	__raw_readl(base + offset);
}

void u2o_write(unsigned base, unsigned offset, unsigned value)
{
	writel(value, base + offset);
	__raw_readl(base + offset);
}

int pxa9xx_usb_phy_init(unsigned int base)
{
	/* linux kernel is not allowed to override usb phy settings */
	/* these settings configured only by obm (or bootrom)       */
	static int init_done;
	unsigned int ulTempAccr1;
	unsigned int ulTempCkenC;

	if (init_done)
		printk(KERN_DEBUG "re-init phy\n");

	ulTempAccr1 = ACCR1;
	ulTempCkenC = CKENC;

	ACCR1 |= (1<<10);
	CKENC |= (1<<10);

	/* Not safe. Sync risk */
	if ((ulTempAccr1 & (1<<10)) == 0)
		ACCR1 &= ~(1<<10);

	if ((ulTempCkenC & (1<<10)) == 0)
		CKENC &= ~(1<<10);

	/* override usb phy setting to mach values set by obm */
	u2o_write(base, U2PPLL, 0xfe819eeb);
	u2o_write(base, U2PTX, 0x41c10fc2);
	u2o_write(base, U2PRX, 0xe31d02e9);
	u2o_write(base, U2IVREF, 0x2000017e);

	u2o_write(base, U2PRS, 0x00008000);

	init_done = 1;
	return 0;
}
#endif
/*
return: -1 -- failure:exceed limit; >=0 -- success;
These two functions shall be different on different platforms.
*/

int pxa95x_vmeta_increase_core_freq(const struct vmeta_instance *vi,
						const int step)
{
	if (vi->vop >= VMETA_OP_VGA
	&& vi->vop <= (VMETA_OP_VGA+2-step)) { /* VGA:1,2,3 */
		return vi->vop+step;
	} else if (vi->vop >= VMETA_OP_720P
		&& vi->vop <= (VMETA_OP_720P+2-step)) {/* 720p: 8,9,10 */
		return vi->vop+step;
	}

	return -1;
}

int pxa95x_vmeta_decrease_core_freq(const struct vmeta_instance *vi,
					const int step)
{
	if (vi->vop >= (VMETA_OP_VGA+step) && vi->vop <= VMETA_OP_VGA+2) {
		return vi->vop - step;
	} else if (vi->vop >= (VMETA_OP_720P+step)
		&& vi->vop <= VMETA_OP_720P+2) {
		return vi->vop - step;
	}

	return -1;
}

int pxa95x_vmeta_clean_dvfm_constraint(struct vmeta_instance *vi, int idx)
{
	dvfm_disable_op_name("208M_HF", idx);
	dvfm_disable_op_name("416M_VGA", idx);
	dvfm_enable_op_name("416M", idx);
	return 0;
}

int pxa95x_vmeta_init_dvfm_constraint(struct vmeta_instance *vi, int idx)
{
	dvfm_disable_op_name("208M_HF", idx);
	dvfm_disable_op_name("416M_VGA", idx);
	return 0;
}

/*
resolution <= VGA          -- 1~3	208M_HF, 416M_VGA, 624M
VGA < resolution <=720p    -- 8~10	416M, 624M, 806M
resolution > 720p          -- 806M
*/
int pxa95x_vmeta_set_dvfm_constraint(struct vmeta_instance *vi, int idx)
{
	if ((vi->vop < VMETA_OP_MIN || vi->vop > VMETA_OP_MAX)
		&& vi->vop != VMETA_OP_INVALID) {
		printk(KERN_ERR "unsupport vmeta vop=%d\n", vi->vop);
		return -1;
	}

	vi->vop_real = vi->vop;

	if (!cpu_is_pxa978()) {
		dvfm_disable_op_name("156M", idx);
		dvfm_disable_op_name("156M_HF", idx);
		dvfm_disable_op_name("988M", idx);

		switch (vi->vop_real) {
		case VMETA_OP_VGA:
			dvfm_enable_op_name("208M_HF", idx);
			dvfm_enable_op_name("416M_VGA", idx);
			dvfm_disable_op_name("416M", idx);
			break;
		case VMETA_OP_VGA+1:
			dvfm_disable_op_name("416M", idx);
			dvfm_enable_op_name("416M_VGA", idx);
			dvfm_disable_op_name("208M_HF", idx);
			break;
		case VMETA_OP_VGA+2:
			dvfm_disable_op_name("416M", idx);
			dvfm_disable_op_name("208M_HF", idx);
			dvfm_disable_op_name("416M_VGA", idx);
			break;
		case VMETA_OP_720P:
		case VMETA_OP_INVALID:
			dvfm_disable_op_name("208M_HF", idx);
			dvfm_disable_op_name("416M_VGA", idx);
			break;
		case VMETA_OP_720P+1:
			dvfm_disable_op_name("208M_HF", idx);
			dvfm_disable_op_name("416M_VGA", idx);
			dvfm_disable_op_name("416M", idx);
			break;
		case VMETA_OP_720P+2:
		default:
			dvfm_disable_op_name("208M_HF", idx);
			dvfm_disable_op_name("416M_VGA", idx);
			dvfm_disable_op_name("416M", idx);
			dvfm_disable_op_name("624M", idx);
			break;
		}
	} else {
		dvfm_disable_op_name("624M", idx);
		dvfm_disable_op_name("312M", idx);
		dvfm_disable_op_name("156M", idx);
	}

	return 0;
}

int pxa95x_vmeta_unset_dvfm_constraint(struct vmeta_instance *vi, int idx)
{
	if (!cpu_is_pxa978()) {
		dvfm_enable_op_name("156M", idx);
		dvfm_enable_op_name("156M_HF", idx);
		dvfm_enable_op_name("624M", idx);
		dvfm_enable_op_name("988M", idx);

		/* It's already power off, e.g. in pause case */
		if (vi->power_status == 0)
			vi->vop_real = VMETA_OP_INVALID;

		switch (vi->vop_real) {
		case VMETA_OP_VGA:
		case VMETA_OP_VGA+1:
		case VMETA_OP_VGA+2:
			dvfm_disable_op_name("416M", idx);
			dvfm_enable_op_name("208M_HF", idx);
			dvfm_enable_op_name("416M_VGA", idx);
			break;
		case VMETA_OP_720P:
		case VMETA_OP_720P+1:
		case VMETA_OP_720P+2:
		case VMETA_OP_INVALID:
		default:
			dvfm_disable_op_name("208M_HF", idx);
			dvfm_disable_op_name("416M_VGA", idx);
			dvfm_enable_op_name("416M", idx);
			break;
		}
	} else {
		dvfm_enable_op_name("156M", idx);
		dvfm_enable_op_name("312M", idx);
		dvfm_enable_op_name("624M", idx);
	}
	vi->vop_real = VMETA_OP_INVALID;

	return 0;
}

irqreturn_t pxa95x_vmeta_bus_irq_handler(int irq, void *dev_id)
{
	struct vmeta_instance *vi = (struct vmeta_instance *)dev_id;

	printk(KERN_ERR "VMETA: bus error detected\n");
	uio_event_notify(&vi->uio_info);
	return IRQ_HANDLED;
}

#define CP_MEM_MAX_SEGMENTS 2
unsigned _cp_area_addr[CP_MEM_MAX_SEGMENTS];
unsigned _cp_area_size[CP_MEM_MAX_SEGMENTS+1]; /* last entry 0 */
static int __init setup_cpmem(char *p)
{
	unsigned long size, start = 0xa7000000;
	int seg;

	size  = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);

	for (seg = 0; seg < CP_MEM_MAX_SEGMENTS; seg++)
		if (!_cp_area_size[seg])
			break;
	BUG_ON(seg == CP_MEM_MAX_SEGMENTS);
	_cp_area_addr[seg] = (unsigned)start;
	_cp_area_size[seg] = (unsigned)size;
	return 0;
}
early_param("cpmem", setup_cpmem);

unsigned cp_area_addr(void)
{
	/* _cp_area_addr[] contain actual CP region addresses for reservation.
	This function returns the address of the first region, which is
	the main one used for AP-CP interface, aligned to 16MB.
	The AP-CP interface code takes care of the offsets inside the region,
	including the non-CP area at the beginning of the 16MB aligned range. */
	return _cp_area_addr[0]&0xFF000000;
}
EXPORT_SYMBOL(cp_area_addr);

void pxa95x_cpmem_reserve(void)
{
	int seg;

	/* reserve cpmem */
	for (seg = 0; seg < CP_MEM_MAX_SEGMENTS; seg++) {
		if (_cp_area_size[seg] != 0) {
			BUG_ON(memblock_reserve(_cp_area_addr[seg], _cp_area_size[seg]));
			memblock_free(_cp_area_addr[seg], _cp_area_size[seg]);
			memblock_remove(_cp_area_addr[seg], _cp_area_size[seg]);
			printk(KERN_INFO "Reserving CP memory: %dM at %.8x\n",
				(unsigned)_cp_area_size[seg]/0x100000,
				(unsigned)_cp_area_addr[seg]);
		}
	}

#ifdef CONFIG_ANDROID_PMEM
	/* reserve pmem */
	pxa_reserve_pmem_memblock();
#endif
}

void pxa95x_mem_reserve(void)
{
	pxa95x_cpmem_reserve();
}

void pxa_boot_flash_init(int sync_mode)
{
	int boot_flash_type;

	/* Get boot flash type from OBM */
	boot_flash_type = pxa_boot_flash_type_get();
	switch (boot_flash_type) {
	case NAND_FLASH:
#ifdef CONFIG_MTD_NAND
		nand_init();
#endif
		break;
	case ONENAND_FLASH:
#ifdef CONFIG_MTD_ONENAND
		/* 1 sync read, 0  async read */
		onenand_init(sync_mode);
#endif
		break;
	case SDMMC_FLASH:
		/* ShukiZ: TODO, check how to init
			eMMC vs. external MMC device */
		break;
	default:
		printk(KERN_ERR "boot flash type not supported: %d",
			boot_flash_type);
	}
}
EXPORT_SYMBOL(pxa_boot_flash_init);

static struct dvfm_lock dvfm_lock = {
	.lock = __SPIN_LOCK_UNLOCKED(dvfm_lock.lock),
	.dev_idx = -1,
	.count = 0,
};

static void vmeta_work_handler(struct work_struct *work)
{
	int ret;
	struct vmeta_instance *vi = container_of(work,
						 struct vmeta_instance,
						 unset_op_work.work);
	spin_lock(&dvfm_lock.lock);
	if (dvfm_lock.count == 0) {
		spin_unlock(&dvfm_lock.lock);
		mutex_lock(&vi->mutex);
		ret = pxa95x_vmeta_unset_dvfm_constraint(vi, dvfm_lock.dev_idx);
		if (ret) {
			printk(KERN_ERR "vmeta dvfm enable error with %d\n",
			       ret);
		}
		vi->power_constraint = 0;
		mutex_unlock(&vi->mutex);
		return;
	}
	spin_unlock(&dvfm_lock.lock);
}
int vmeta_runtime_constraint(struct vmeta_instance *vi, int on)
{
	int ret = 0;
	if (1 == on) {
		spin_lock(&dvfm_lock.lock);
		if (dvfm_lock.count++ == 0) {
			spin_unlock(&dvfm_lock.lock);
		/* Disable dvfm for now for MG1,
		 * todo: later should try to restore to optimize for power */
			ret = pxa95x_vmeta_set_dvfm_constraint(vi,
						dvfm_lock.dev_idx);
			if (ret)
				printk(KERN_ERR
				"vmeta dvfm disable error with %d\n", ret);
			vi->power_constraint = 1;
			cancel_delayed_work(&vi->unset_op_work);
		} else {
			dvfm_lock.count--;
			spin_unlock(&dvfm_lock.lock);
		}
	} else if (0 == on) {
		spin_lock(&dvfm_lock.lock);
		if (dvfm_lock.count == 0) {
			spin_unlock(&dvfm_lock.lock);
			return 0;
		}
		if (--dvfm_lock.count == 0) {
			if (timer_pending(&vi->unset_op_work.timer))
				mod_timer(&vi->unset_op_work.timer,
					  jiffies + msecs_to_jiffies
					  (vmeta_plat_data.power_down_ms));
			else
				schedule_delayed_work_on(0,
							 &vi->unset_op_work,
							 msecs_to_jiffies(
							 vmeta_plat_data.
							 power_down_ms));
		} else
			dvfm_lock.count++;
		spin_unlock(&dvfm_lock.lock);
	}
	return 0;
}
int vmeta_init_constraint(struct vmeta_instance *vi)
{
	int ret;
	INIT_DELAYED_WORK(&vi->unset_op_work, vmeta_work_handler);
	vmeta_plat_data.power_down_ms = 10;

	ret = dvfm_register("VMETA", &dvfm_lock.dev_idx);
	if (ret)
		printk(KERN_ERR "vmeta dvfm register fail(%d)\n", ret);

	pxa95x_vmeta_init_dvfm_constraint(vi, dvfm_lock.dev_idx);
	return 0;
}
int vmeta_clean_constraint(struct vmeta_instance *vi)
{
	pxa95x_vmeta_clean_dvfm_constraint(NULL, dvfm_lock.dev_idx);
	dvfm_unregister("VMETA", &dvfm_lock.dev_idx);
	return 0;
}
int vmeta_freq_change(struct vmeta_instance *vi, int step)
{
	int ret = 0;
	if (step > 0)
		ret = pxa95x_vmeta_increase_core_freq(vi, step);
	else if (step < 0)
		ret = pxa95x_vmeta_decrease_core_freq(vi, 0 - step);
	return ret;
}
void vmeta_power_switch(unsigned int enable)
{
	if (VMETA_PWR_ENABLE == enable) {
		dvfm_disable_lowpower(dvfm_lock.dev_idx);
		vmeta_pwr(VMETA_PWR_ENABLE);
	} else if (VMETA_PWR_DISABLE == enable) {
		vmeta_pwr(VMETA_PWR_DISABLE);
		pxa95x_vmeta_clean_dvfm_constraint(NULL, dvfm_lock.dev_idx);
		printk(KERN_INFO "vmeta op clean up\n");
		dvfm_enable_lowpower(dvfm_lock.dev_idx);
	}
}
