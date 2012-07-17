/*
 * linux/arch/arm/mach-mmp/pxa988.c
 *
 * code name PXA988
 *
 * Copyright (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/memblock.h>
#include <linux/platform_device.h>

#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/cacheflush.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-pmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/pxa988.h>
#include <mach/regs-timers.h>
#include <mach/pxa910-squ.h>
#include <mach/soc_coda7542.h>
#include <mach/reset-pxa988.h>
#include <mach/regs-usb.h>
#include <mach/gpio-edge.h>

#include <plat/mfp.h>
#include <plat/pmem.h>
#include <plat/pm.h>

#include "common.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)
#define RIPC0_VIRT_BASE	(APB_VIRT_BASE + 0x3D000)
#define GPIOE_VIRT_BASE	(APB_VIRT_BASE + 0x19800)
#define RIPC0_STATUS	(RIPC0_VIRT_BASE + 0x00)

static struct mfp_addr_map pxa988_addr_map[] __initdata = {

	MFP_ADDR_X(GPIO0, GPIO54, 0xdc),
	MFP_ADDR_X(GPIO67, GPIO98, 0x1b8),
	MFP_ADDR_X(GPIO100, GPIO109, 0x238),
	MFP_ADDR_X(GPIO110, GPIO116, 0x298),

	MFP_ADDR(DF_IO0, 0x40),
	MFP_ADDR(DF_IO1, 0x3c),
	MFP_ADDR(DF_IO2, 0x38),
	MFP_ADDR(DF_IO3, 0x34),
	MFP_ADDR(DF_IO4, 0x30),
	MFP_ADDR(DF_IO5, 0x2c),
	MFP_ADDR(DF_IO6, 0x28),
	MFP_ADDR(DF_IO7, 0x24),
	MFP_ADDR(DF_IO8, 0x20),
	MFP_ADDR(DF_IO9, 0x1c),
	MFP_ADDR(DF_IO10, 0x18),
	MFP_ADDR(DF_IO11, 0x14),
	MFP_ADDR(DF_IO12, 0x10),
	MFP_ADDR(DF_IO13, 0xc),
	MFP_ADDR(DF_IO14, 0x8),
	MFP_ADDR(DF_IO15, 0x4),

	MFP_ADDR(DF_nCS0_SM_nCS2, 0x44),
	MFP_ADDR(DF_nCS1_SM_nCS3, 0x48),
	MFP_ADDR(SM_nCS0, 0x4c),
	MFP_ADDR(SM_nCS1, 0x50),
	MFP_ADDR(DF_WEn, 0x54),
	MFP_ADDR(DF_REn, 0x58),
	MFP_ADDR(DF_CLE_SM_OEn, 0x5c),
	MFP_ADDR(DF_ALE_SM_WEn, 0x60),
	MFP_ADDR(SM_SCLK, 0x64),
	MFP_ADDR(DF_RDY0, 0x68),
	MFP_ADDR(SM_BE0, 0x6c),
	MFP_ADDR(SM_BE1, 0x70),
	MFP_ADDR(SM_ADV, 0x74),
	MFP_ADDR(DF_RDY1, 0x78),
	MFP_ADDR(SM_ADVMUX, 0x7c),
	MFP_ADDR(SM_RDY, 0x80),
	MFP_ADDR(ANT_SW4, 0x26c),

	MFP_ADDR_X(MMC1_DAT7, MMC1_WP, 0x84),

	MFP_ADDR(GPIO124, 0xd0),

	MFP_ADDR(CLK_REQ, 0xcc),

	MFP_ADDR_END,
};

/* GC power control */
#define GC_CLK_EN	\
	((0x1 << 3) | (0x1 << 4) | (0x1 << 5))

#define GC_ACLK_RST	(0x1 << 0)
#define GC_FCLK_RST	(0x1 << 1)
#define GC_HCLK_RST	(0x1 << 2)
#define GC_CLK_RST	\
	(GC_ACLK_RST | GC_FCLK_RST | GC_HCLK_RST)

#define GC_ISOB		(0x1 << 8)
#define GC_PWRON1	(0x1 << 9)
#define GC_PWRON2	(0x1 << 10)

#define GC_FCLK_SEL_MASK	(0x3 << 6)
#define GC_ACLK_SEL_MASK	(0x3 << 20)
#define GC_FCLK_DIV_MASK	(0x7 << 12)
#define GC_ACLK_DIV_MASK	(0x7 << 17)
#define GC_FCLK_REQ		(0x1 << 15)
#define GC_ACLK_REQ		(0x1 << 16)

#define GC_CLK_SEL_WIDTH	(2)
#define GC_CLK_DIV_WIDTH	(3)
#define GC_FCLK_SEL_SHIFT	(6)
#define GC_ACLK_SEL_SHIFT	(20)
#define GC_FCLK_DIV_SHIFT	(12)
#define GC_ACLK_DIV_SHIFT	(17)

#define GC_REG_WRITE(val)	{	\
	__raw_writel(val, APMU_GC);	\
}

static DEFINE_SPINLOCK(gc_pwr_lock);
/* used for GC LPM constraint */
static struct pm_qos_request_list gc_lpm_cons;
static bool gc_qos_list_inited;

void gc_pwr(int power_on)
{
	unsigned int val = __raw_readl(APMU_GC);

	spin_lock(&gc_pwr_lock);
	/* initialize the qos list at the first time */
	if (unlikely(!gc_qos_list_inited)) {
		pm_qos_add_request(&gc_lpm_cons,
			PM_QOS_CPUIDLE_KEEP_DDR, PM_QOS_DEFAULT_VALUE);
		gc_qos_list_inited = true;
	}

	if (power_on) {
		/* block LPM deeper than D1 */
		pm_qos_update_request(&gc_lpm_cons, PM_QOS_CONSTRAINT);

		/* enable bus and function clock  */
		val |= GC_CLK_EN;
		GC_REG_WRITE(val);

		/* enable power_on1, wait at least 200us */
		val |= GC_PWRON1;
		GC_REG_WRITE(val);
		udelay(200);

		/* enable power_on2 */
		val |= GC_PWRON2;
		GC_REG_WRITE(val);

		/* fRst release */
		val |= GC_FCLK_RST;
		GC_REG_WRITE(val);
		udelay(100);

		/* aRst hRst release at least 48 cycles later than fRst */
		val |= (GC_ACLK_RST | GC_HCLK_RST);
		GC_REG_WRITE(val);

		/* disable isolation */
		val |= GC_ISOB;
		GC_REG_WRITE(val);
	} else {
		/* enable isolation */
		val &= ~GC_ISOB;
		GC_REG_WRITE(val);

		/* disable power_on2 */
		val &= ~GC_PWRON2;
		GC_REG_WRITE(val);

		/* disable power_on1 */
		val &= ~GC_PWRON1;
		GC_REG_WRITE(val);

		/* fRst aRst hRst */
		val &= ~(GC_CLK_RST | GC_CLK_EN);
		GC_REG_WRITE(val);
		udelay(100);

		/* release D1 LPM constraint */
		pm_qos_update_request(&gc_lpm_cons, PM_QOS_DEFAULT_VALUE);
	}
	spin_unlock(&gc_pwr_lock);
}
EXPORT_SYMBOL(gc_pwr);

#define VPU_HW_MODE	(0x1 << 19)
#define VPU_AUTO_PWR_ON	(0x1 << 2)
#define VPU_PWR_STAT	(0x1 << 2)

void coda7542_power_switch(int on)
{
	unsigned int val;
	int timeout = 5000;

	/* HW mode power on */
	if (on) {
		/* set VPU HW on/off mode  */
		val = __raw_readl(APMU_VPU_CLK_RES_CTRL);
		val |= VPU_HW_MODE;
		__raw_writel(val, APMU_VPU_CLK_RES_CTRL);
		/* on1, on2, off timer */
		__raw_writel(0x20001fff, APMU_PWR_BLK_TMR_REG);

		/* VPU auto power on */
		val = __raw_readl(APMU_PWR_CTRL_REG);
		val |= VPU_AUTO_PWR_ON;
		__raw_writel(val, APMU_PWR_CTRL_REG);

		/* polling VPU_PWR_STAT bit */
		while (!(__raw_readl(APMU_PWR_STATUS_REG) & VPU_PWR_STAT)) {
			udelay(500);
			timeout -= 500;
			if (timeout < 0) {
				pr_err("%s: VPU power on timeout\n", __func__);
				return;
			}
		}
	/* HW mode power off */
	} else {
		/* VPU auto power off */
		val = __raw_readl(APMU_PWR_CTRL_REG);
		val &= ~VPU_AUTO_PWR_ON;
		__raw_writel(val, APMU_PWR_CTRL_REG);

		/* polling VPU_PWR_STAT bit */
		while ((__raw_readl(APMU_PWR_STATUS_REG) & VPU_PWR_STAT)) {
			udelay(500);
			timeout -= 500;
			if (timeout < 0) {
				pr_err("%s: VPU power off timeout\n", __func__);
				return;
			}
		}
	}
}

#ifdef CONFIG_SMP
#define PM_RESERVE_SIZE	(1024 * 1024)
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
static void pxa988_ram_console_mem_reserve(void)
{
	unsigned int pa, ret;

	pa = __virt_to_phys(CONFIG_ANDROID_RAM_CONSOLE_EARLY_ADDR);

	if (!pa || CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE == 0)
		return;

	ret = memblock_reserve(pa, CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE);
	if (ret)
		pr_err("Failed to reserve ram console memory, ret 0x%x\n", ret);
	else
		pr_info("Reserve 0x%x at 0x%x (va 0x%x) for ram console\n",
				CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE, pa,
				CONFIG_ANDROID_RAM_CONSOLE_EARLY_ADDR);
}
#endif

static void __init pxa988_reserve_cpmem(void)
{
	u32 cp_area_addr = 0x07000000;
	u32 cp_area_size = 0x01000000;

	/* Reserve 16MB memory for CP */
	BUG_ON(memblock_reserve(cp_area_addr, cp_area_size) != 0);
	memblock_free(cp_area_addr, cp_area_size);
	memblock_remove(cp_area_addr, cp_area_size);
	pr_info("Reserved CP memory: 0x%x@0x%x\n", cp_area_size, cp_area_addr);
}

void __init pxa988_reserve(void)
{
	/*
	 * reserve the first 1MB physical ddr memory for obm. when use EMMD
	 * (Enhanced Marvell Memory Dump), kernel should not make use of this
	 * memory, since it'll be corrupted by next reboot by obm.
	 */
	BUG_ON(memblock_reserve(PLAT_PHYS_OFFSET, 0x100000));

	pxa988_reserve_cpmem();

#ifdef CONFIG_ANDROID_PMEM
	/*reserve memory for pmem*/
	pxa_reserve_pmem_memblock();
#endif
#ifdef CONFIG_SMP
	pm_reserve_pa = memblock_alloc(PM_RESERVE_SIZE, PAGE_SIZE);
	if (!pm_reserve_pa) {
		pr_err("%s: failed to reserve memory for PM\n", __func__);
		BUG();
	}
	BUG_ON(memblock_free(pm_reserve_pa, PM_RESERVE_SIZE));
	BUG_ON(0 != memblock_remove(pm_reserve_pa, PM_RESERVE_SIZE));
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
	pxa988_ram_console_mem_reserve();
#endif
}

void __init pxa988_init_irq(void)
{
	void __iomem *dist_base = (void __iomem *)GIC_DIST_VIRT_BASE;
	void __iomem *cpu_base = (void __iomem *)GIC_CPU_VIRT_BASE;

	mmp_wakeupgen_init();

	gic_init(0, 29, dist_base, cpu_base);
}

void pxa988_ripc_lock(void)
{
	while (__raw_readl(RIPC0_STATUS))
		cpu_relax();
}

int pxa988_ripc_trylock(void)
{
	return !__raw_readl(RIPC0_STATUS);
}

void pxa988_ripc_unlock(void)
{
	__raw_writel(1, RIPC0_STATUS);
}
#ifdef CONFIG_CACHE_L2X0

#ifdef CONFIG_PM
static inline void l2x0_save_phys_reg_addr(u32 *addr_ptr, u32 addr)
{
	BUG_ON(!addr_ptr);
	*addr_ptr = addr;
	flush_cache_all();
	outer_clean_range(virt_to_phys(addr_ptr),
		virt_to_phys(addr_ptr) + sizeof(*addr_ptr));
}
#endif

static void pxa988_l2_cache_init(void)
{
	void __iomem *l2x0_base;

	l2x0_base = ioremap(SL2C_PHYS_BASE, SZ_4K);
	BUG_ON(!l2x0_base);

	/*
	 * TAG, Data Latency Control
	 * The default value for pxa988 is all 8 cycles which should be safe.
	 * But we may need to change it after do some test on real silicon.
	 * writel_relaxed(0x010, l2x0_base + L2X0_TAG_LATENCY_CTRL);
	 * writel_relaxed(0x010, l2x0_base + L2X0_DATA_LATENCY_CTRL);
	 */

	/* L2X0 Power Control  */
	writel_relaxed(0x3, l2x0_base + L2X0_POWER_CTRL);

	/*
	 * use the default value here
	 * for prefetch feature, it can be handled by prefetch register.
	 */
	l2x0_init(l2x0_base, 0, ~0);
#ifdef CONFIG_PM
	l2x0_saved_regs.phy_base = SL2C_PHYS_BASE;
	l2x0_save_phys_reg_addr(&l2x0_regs_phys,
				l2x0_saved_regs_phys_addr);
#endif
}
#else
#define pxa988_l2_cache_init()
#endif

static void __init pxa988_timer_init(void)
{
	uint32_t clk_rst;

	__raw_writel(APBC_APBCLK | APBC_RST, APBC_PXA988_TIMERS);
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
	__raw_writel(clk_rst, APBC_PXA988_TIMERS);

	stimer_source_select(2, TIMER_RATE_32K);
	stimer_event_config(0, 0, IRQ_PXA988_AP_TIMER1, 6500000);
	stimer_event_config(1, 1, IRQ_PXA988_AP_TIMER2_3, 6500000);
	stimer_device_init(TIMERS1_VIRT_BASE);
}

struct sys_timer pxa988_timer = {
	.init   = pxa988_timer_init,
};

void pxa988_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = APMU_PXA988_KP_WAKE_CLR;

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val | mask, APMU_WAKE_CLR);
}

#ifdef CONFIG_USB_PXA_U2O
static DEFINE_SPINLOCK(phy_lock);
static int phy_init_cnt;

static int usb_phy_init_internal(unsigned int base)
{
	struct pxa988_usb_phy *phy = (struct pxa988_usb_phy *)base;
	int i;
	u32 phy_old, phy_power;

	pr_debug("init usb phy.\n");

	/*
	 * power up PHY by PIN.
	 * From the datasheet, it can be controlled by current regiter,
	 * but not pin.
	 * Will remove it after debug.
	 */
	phy_old = (u32)ioremap_nocache(0xD4207100, 0x10);
	phy_power = phy_old + 0x4;
	writel(0x10901003, phy_power);

	/* enable usb device PHY */
	writew(PLLVDD18(0x1) | REFDIV(0xe) | FBDIV(0x1f0),
		&phy->utmi_pll_reg0);
	writew(PU_PLL | PLL_LOCK_BYPASS | ICP(0x3) | KVCO(0x3) | PLLCAL12(0x3),
		&phy->utmi_pll_reg1);
	writew(IMPCAL_VTH(0x2) | EXT_HS_RCAL(0x8) | EXT_FS_RCAL(0x8),
		&phy->utmi_tx_reg0);
	writew(TXVDD15(0x1) | TXVDD12(0x3) | LOWVDD_EN |
		AMP(0x4) | CK60_PHSEL(0x4),
		&phy->utmi_tx_reg1);
	writew(DRV_SLEWRATE(0x3) | IMP_CAL_DLY(0x2) |
		FSDRV_EN(0xf) | HSDEV_EN(0xf),
		&phy->utmi_tx_reg2);
	writew(PHASE_FREEZE_DLY | ACQ_LENGTH(0x2) | SQ_LENGTH(0x2) |
		DISCON_THRESH(0x2) | SQ_THRESH(0xa) | INTPI(0x1),
		&phy->utmi_rx_reg0);
	writew(EARLY_VOS_ON_EN | RXDATA_BLOCK_EN | EDGE_DET_EN |
		RXDATA_BLOCK_LENGTH(0x2) | EDGE_DET_SEL(0x1) |
		S2TO3_DLY_SEL(0x2),
		&phy->utmi_rx_reg1);
	writew(USQ_FILTER | SQ_BUFFER_EN | RXVDD18(0x1) | RXVDD12(0x1),
		&phy->utmi_rx_reg2);
	writew(BG_VSEL(0x1) | TOPVDD18(0x1),
		&phy->utmi_ana_reg0);
	writew(PU_ANA | SEL_LPFR | V2I(0x6) | R_ROTATE_SEL,
		&phy->utmi_ana_reg1);
	writew(FS_EOP_MODE | FORCE_END_EN | SYNCDET_WINDOW_EN |
		CLK_SUSPEND_EN | FIFO_FILL_NUM(0x6),
		&phy->utmi_dig_reg0);
	writew(FS_RX_ERROR_MODE2 | FS_RX_ERROR_MODE1 |
		FS_RX_ERROR_MODE | ARC_DPDM_MODE,
		&phy->utmi_dig_reg1);
	writew(0x0, &phy->utmi_charger_reg0);

	for (i = 0; i < 0x80; i = i + 4)
		pr_debug("[0x%x] = 0x%x\n", base + i,
			readw(base + i));

	iounmap((void __iomem *)phy_old);
	return 0;
}

static int usb_phy_deinit_internal(unsigned int base)
{
	u32 phy_old, phy_power;
	struct pxa988_usb_phy *phy = (struct pxa988_usb_phy *)base;
	u16 val;

	pr_debug("Deinit usb phy.\n");

	/* power down PHY PLL */
	val = readw(&phy->utmi_pll_reg1);
	val &= ~PU_PLL;
	writew(val, &phy->utmi_pll_reg1);

	/* power down PHY Analog part */
	val = readw(&phy->utmi_ana_reg1);
	val &= ~PU_ANA;
	writew(val, &phy->utmi_ana_reg1);

	/* power down PHY by PIN.
	 * From the datasheet, it can be controlled by current regiter,
	 * but not pin.
	 * Will remove it after debug.
	 */
	phy_old = (u32)ioremap_nocache(0xD4207100, 0x10);
	phy_power = phy_old + 0x4;
	writel(0x10901000, phy_power);

	iounmap((void __iomem *)phy_old);
	return 0;
}

int pxa_usb_phy_init(unsigned int base)
{
	unsigned long flags;

	spin_lock_irqsave(&phy_lock, flags);
	if (phy_init_cnt++ == 0)
		usb_phy_init_internal(base);
	spin_unlock_irqrestore(&phy_lock, flags);
	return 0;
}

void pxa_usb_phy_deinit(unsigned int base)
{
	unsigned long flags;

	WARN_ON(phy_init_cnt == 0);

	spin_lock_irqsave(&phy_lock, flags);
	if (--phy_init_cnt == 0)
		usb_phy_deinit_internal(base);
	spin_unlock_irqrestore(&phy_lock, flags);
}
#endif /* CONFIG_USB_PXA_U2O */

struct platform_device pxa988_device_asoc_ssp1 = {
	.name		= "pxa-ssp-dai",
	.id		= 1,
};

struct platform_device pxa988_device_asoc_squ = {
	.name		= "pxa910-squ-audio",
	.id		= -1,
};

#ifdef CONFIG_USB_PXA_U2O
static u64 usb_dma_mask = ~(u32)0;

struct resource pxa988_udc_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA988_UDC_REGBASE + PXA988_UDC_CAPREGS_RANGE,
		.end	= PXA988_UDC_REGBASE + PXA988_UDC_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA988_UDC_PHYBASE,
		.end	= PXA988_UDC_PHYBASE + PXA988_UDC_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA988_USB1,
		.end	= IRQ_PXA988_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa988_device_udc = {
	.name		= "pxa-u2o",
	.id		= -1,
	.resource	= pxa988_udc_resources,
	.num_resources	= ARRAY_SIZE(pxa988_udc_resources),
	.dev		=  {
		.dma_mask	= &usb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};
#endif /* CONFIG_USB_PXA_U2O */

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x09c)
static void __init pxa988_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_PXA988_GPIO);

	/* unmask GPIO edge detection for all 4 banks -- APMASKx */
	for (i = 0; i < 4; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_PXA988_GPIO_AP, 0, 127, NULL);
}

static int __init pxa988_init(void)
{
	pxa988_l2_cache_init();

	mfp_init_base(MFPR_VIRT_BASE);
	mfp_init_addr(pxa988_addr_map);

	pxa988_init_gpio();

	mmp_gpio_edge_init(GPIOE_VIRT_BASE, 128);

	/* would remove such pxa910 interface when kernel upgrade */
	pxa910_init_squ(2);
	/* add ssp and squ device for hifi audio */
	platform_device_register(&pxa988_device_asoc_ssp1);
	platform_device_register(&pxa988_device_asoc_squ);

	return 0;
}

postcore_initcall(pxa988_init);

/* on-chip devices */
PXA988_DEVICE(uart0, "pxa2xx-uart", 0, UART0, 0xd4036000, 0x30, 4, 5);
PXA988_DEVICE(uart1, "pxa2xx-uart", 1, UART1, 0xd4017000, 0x30, 21, 22);
PXA988_DEVICE(uart2, "pxa2xx-uart", 2, UART2, 0xd4018000, 0x30, 23, 24);
PXA988_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
PXA988_DEVICE(twsi0, "pxa910-i2c", 0, I2C_AP,   0xd4011000, 0x40);
PXA988_DEVICE(twsi1, "pxa910-i2c", 1, I2C_INT,  0xd4037000, 0x40);
PXA988_DEVICE(twsi2, "pxa910-i2c", 2, I2C_CP,   0xd4010800, 0x40);
PXA988_DEVICE(sdh1, "sdhci-pxa", 0, MMC, 0xd4280000, 0x120);
PXA988_DEVICE(sdh2, "sdhci-pxa", 1, MMC, 0xd4280800, 0x120);
PXA988_DEVICE(sdh3, "sdhci-pxa", 2, MMC, 0xd4281000, 0x120);
PXA988_DEVICE(ssp0, "pxa910-ssp", 0, SSP0, 0xd401b000, 0x90, 52, 53);
PXA988_DEVICE(ssp1, "pxa910-ssp", 1, SSP1, 0xd42a0c00, 0x90, 1, 2);
PXA988_DEVICE(ssp2, "pxa910-ssp", 2, SSP2, 0xd401C000, 0x90, 60, 61);
PXA988_DEVICE(asram, "mmp-sram", 0, NONE, SRAM_AUDIO_BASE, SRAM_AUDIO_SIZE);
PXA988_DEVICE(vsram, "mmp-sram", 1, NONE, SRAM_VIDEO_BASE, SRAM_VIDEO_SIZE);
PXA988_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x1ec);
PXA988_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x1ec);
PXA988_DEVICE(camera, "mv-camera", 0, CI, 0xd420a000, 0xfff);

/* TODO Fake implementation for bring up */
void handle_coherency_maint_req(void *p) {};

