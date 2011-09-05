/*
 * linux/arch/arm/mach-mmp/devices.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <mach/devices.h>
#include <mach/cputype.h>
#include <mach/regs-usb.h>
#include <mach/soc_vmeta.h>
#include <mach/regs-pmu.h>

int __init pxa_register_device(struct pxa_device_desc *desc,
				void *data, size_t size)
{
	struct platform_device *pdev;
	struct resource res[2 + MAX_RESOURCE_DMA];
	int i, ret = 0, nres = 0;

	pdev = platform_device_alloc(desc->drv_name, desc->id);
	if (pdev == NULL)
		return -ENOMEM;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	memset(res, 0, sizeof(res));

	if (desc->start != -1ul && desc->size > 0) {
		res[nres].start	= desc->start;
		res[nres].end	= desc->start + desc->size - 1;
		res[nres].flags	= IORESOURCE_MEM;
		nres++;
	}

	if (desc->irq != NO_IRQ) {
		res[nres].start	= desc->irq;
		res[nres].end	= desc->irq;
		res[nres].flags	= IORESOURCE_IRQ;
		nres++;
	}

	for (i = 0; i < MAX_RESOURCE_DMA; i++, nres++) {
		if (desc->dma[i] == 0)
			break;

		res[nres].start	= desc->dma[i];
		res[nres].end	= desc->dma[i];
		res[nres].flags	= IORESOURCE_DMA;
	}

	ret = platform_device_add_resources(pdev, res, nres);
	if (ret) {
		platform_device_put(pdev);
		return ret;
	}

	if (data && size) {
		ret = platform_device_add_data(pdev, data, size);
		if (ret) {
			platform_device_put(pdev);
			return ret;
		}
	}

	return platform_device_add(pdev);
}

#if defined(CONFIG_PXA9XX_ACIPC)
/* PXA910 AC-IPC */
static struct resource pxa910_resource_acipc[] = {
	[0] = {
		.start  = 0xD401D000,
		.end    = 0xD401D0ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_PXA910_IPC_AP0,
		.end    = IRQ_PXA910_IPC_AP0,
		.flags  = IORESOURCE_IRQ,
		.name   = "IPC_AP_DATAACK",
	},
	[2] = {
		.start  = IRQ_PXA910_IPC_AP1,
		.end    = IRQ_PXA910_IPC_AP1,
		.flags  = IORESOURCE_IRQ,
		.name   = "IPC_AP_SET_CMD",
	},
	[3] = {
		.start  = IRQ_PXA910_IPC_AP2,
		.end    = IRQ_PXA910_IPC_AP2,
		.flags  = IORESOURCE_IRQ,
		.name   = "IPC_AP_SET_MSG",
	},
};

struct platform_device pxa910_device_acipc = {
	.name           = "pxa9xx-acipc",
	.id             = -1,
	.resource       = pxa910_resource_acipc,
	.num_resources  = ARRAY_SIZE(pxa910_resource_acipc),
};
#endif

#if defined(CONFIG_USB) || defined(CONFIG_USB_GADGET)

/*****************************************************************************
 * The registers read/write routines
 *****************************************************************************/

unsigned int u2o_get(unsigned int base, unsigned int offset)
{
	return readl(base + offset);
}

void u2o_set(unsigned int base, unsigned int offset, unsigned int value)
{
	volatile unsigned int reg;

	reg = readl(base + offset);
	reg |= value;
	writel(reg, base + offset);
	__raw_readl(base + offset);

}

void u2o_clear(unsigned int base, unsigned int offset, unsigned int value)
{
	volatile unsigned int reg;

	reg = readl(base + offset);
	reg &= ~value;
	writel(reg, base + offset);
	__raw_readl(base + offset);
}

void u2o_write(unsigned int base, unsigned int offset, unsigned int value)
{
	writel(value, base + offset);
	__raw_readl(base + offset);

}

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)

#ifdef CONFIG_CPU_MMP3

int pxa_usb_phy_init(unsigned int base)
{
	static int init_done = 0;
	int loops = 0;
	u32 temp;

	if (init_done) {
		printk(KERN_INFO "re-init phy\n");
		return 0;
	}

	temp = __raw_readl(PMUA_REG(0x100));
	temp &= ~0xF00;
	temp |= 0xd00;
	__raw_writel(temp, PMUA_REG(0x100));

	udelay(100);

	u2o_clear(base, USB2_PLL_REG0, (USB2_PLL_FBDIV_MASK_MMP3
		| USB2_PLL_REFDIV_MASK_MMP3));
	/*
	 * Still need to decide values for PLLVDD18 and PLLVDD12 for MMP3.
	 * Set to default for now.
	 */
	u2o_set(base, USB2_PLL_REG0, 0x1 << USB2_PLL_VDD18_SHIFT_MMP3
		| 0x1 << USB2_PLL_VDD12_SHIFT_MMP3
		| 0xf0 << USB2_PLL_FBDIV_SHIFT_MMP3
		| 0xd << USB2_PLL_REFDIV_SHIFT_MMP3);

	u2o_clear(base, USB2_PLL_REG1, USB2_PLL_PU_PLL_MASK
		| USB2_PLL_CALI12_MASK_MMP3
		| USB2_PLL_KVCO_MASK_MMP3
		| USB2_PLL_ICP_MASK_MMP3);

	u2o_set(base, USB2_PLL_REG1, 1 << USB2_PLL_PU_PLL_SHIFT_MMP3
		| 3 << USB2_PLL_CAL12_SHIFT_MMP3 | 2 << USB2_PLL_ICP_SHIFT_MMP3
		| 3 << USB2_PLL_KVCO_SHIFT_MMP3
		| 1 << USB2_PLL_LOCK_BYPASS_SHIFT_MMP3);

	u2o_clear(base, USB2_TX_REG0, USB2_TX_IMPCAL_VTH_MASK_MMP3);
	u2o_set(base, USB2_TX_REG0, 2 << USB2_TX_IMPCAL_VTH_SHIFT_MMP3);

	u2o_clear(base, USB2_TX_REG1, USB2_TX_CK60_PHSEL_MASK_MMP3
		| USB2_TX_AMP_MASK_MMP3
		| USB2_TX_VDD12_MASK_MMP3);

	u2o_set(base, USB2_TX_REG1,  4 << USB2_TX_CK60_PHSEL_SHIFT_MMP3
		| 4 << USB2_TX_AMP_SHIFT_MMP3
		| 3 << USB2_TX_VDD12_SHIFT_MMP3);

	u2o_clear(base, USB2_TX_REG2, 3 << USB2_TX_DRV_SLEWRATE_SHIFT);
	u2o_set(base, USB2_TX_REG2, 3 << USB2_TX_DRV_SLEWRATE_SHIFT);

	u2o_clear(base, USB2_RX_REG0, USB2_RX_SQ_THRESH_MASK_MMP3
		| USB2_RX_SQ_LENGTH_MASK_MMP3);
	u2o_set(base, USB2_RX_REG0, 0xa << USB2_RX_SQ_THRESH_SHIFT_MMP3
		| 2 << USB2_RX_SQ_LENGTH_SHIFT_MMP3);

	u2o_set(base, USB2_ANA_REG1, 0x1 << USB2_ANA_PU_ANA_SHIFT_MMP3);

	u2o_set(base, USB2_OTG_REG0, 0x1 << USB2_OTG_PU_OTG_SHIFT_MMP3);

	/* PLL Power up has been done in usb2CIEnableAppSubSysClocks routine
	*  PLL VCO and TX Impedance Calibration Timing for Eshel & MMP3
	*  PU   __________|-------------------------------|
	*  VCOCAL START	___________________|----------------------|
	*  REG_RCAL_START_____________________________|--|_________|
	*			| 200us  | 400us   |40| 400us   | USB PHY READY
	* ------------------------------------------------------------------
	*/
	/* Calibrate PHY */

	udelay(200);
	u2o_set(base, USB2_PLL_REG1, 1 << USB2_PLL_VCOCAL_START_SHIFT_MMP3);
	udelay(200);
	u2o_set(base, USB2_TX_REG0, 1 << USB2_TX_RCAL_START_SHIFT_MMP3);
	udelay(40);
	u2o_clear(base, USB2_TX_REG0, 1 << USB2_TX_RCAL_START_SHIFT_MMP3);
	udelay(400);

	/* Make sure PHY PLL is ready */
	loops = 0;
	while ((u2o_get(base, USB2_PLL_REG1) & USB2_PLL_READY_MASK_MMP3) == 0) {
		mdelay(1);
		loops++;
		if (loops > 100) {
			printk(KERN_WARNING "PLL_READY not set after 100mS.");
			break;
		}
	}
	init_done = 1;
	return 0;
}
#endif

#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_PXA168) \
	|| defined(CONFIG_CPU_PXA910)

int pxa_usb_phy_init(unsigned int base)
{
	static int init_done;
	int loops;

	if (init_done) {
		printk(KERN_DEBUG "re-init phy\n\n");
		/* return; */
	}

	/* Initialize the USB PHY power */
	if (cpu_is_pxa910()) {
		u2o_set(base, UTMI_CTRL, (1<<UTMI_CTRL_INPKT_DELAY_SOF_SHIFT)
			| (1<<UTMI_CTRL_PU_REF_SHIFT));
	}

	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);
	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);

	/* UTMI_PLL settings */
	u2o_clear(base, UTMI_PLL, UTMI_PLL_PLLVDD18_MASK
		| UTMI_PLL_PLLVDD12_MASK | UTMI_PLL_PLLCALI12_MASK
		| UTMI_PLL_FBDIV_MASK | UTMI_PLL_REFDIV_MASK
		| UTMI_PLL_ICP_MASK | UTMI_PLL_KVCO_MASK);

	u2o_set(base, UTMI_PLL, 0xee<<UTMI_PLL_FBDIV_SHIFT
		| 0xb<<UTMI_PLL_REFDIV_SHIFT | 3<<UTMI_PLL_PLLVDD18_SHIFT
		| 3<<UTMI_PLL_PLLVDD12_SHIFT | 3<<UTMI_PLL_PLLCALI12_SHIFT
		| 1<<UTMI_PLL_ICP_SHIFT | 3<<UTMI_PLL_KVCO_SHIFT);

	/* UTMI_TX */
	u2o_clear(base, UTMI_TX, UTMI_TX_REG_EXT_FS_RCAL_EN_MASK
		| UTMI_TX_TXVDD12_MASK | UTMI_TX_CK60_PHSEL_MASK
		| UTMI_TX_IMPCAL_VTH_MASK | UTMI_TX_REG_EXT_FS_RCAL_MASK
		| UTMI_TX_AMP_MASK);
	u2o_set(base, UTMI_TX, 3<<UTMI_TX_TXVDD12_SHIFT
		| 4<<UTMI_TX_CK60_PHSEL_SHIFT | 4<<UTMI_TX_IMPCAL_VTH_SHIFT
		| 8<<UTMI_TX_REG_EXT_FS_RCAL_SHIFT | 3<<UTMI_TX_AMP_SHIFT);

	/* UTMI_RX */
	u2o_clear(base, UTMI_RX, UTMI_RX_SQ_THRESH_MASK
		| UTMI_REG_SQ_LENGTH_MASK);
	u2o_set(base, UTMI_RX, 7<<UTMI_RX_SQ_THRESH_SHIFT
		| 2<<UTMI_REG_SQ_LENGTH_SHIFT);

	/* UTMI_IVREF */
	if (cpu_is_pxa168() || cpu_is_mmp2())
		/* fixing Microsoft Altair board interface with NEC hub issue -
		 * Set UTMI_IVREF from 0x4a3 to 0x4bf */
		u2o_write(base, UTMI_IVREF, 0x4bf);

	/* calibration */
	loops = 0;
	while ((u2o_get(base, UTMI_PLL) & PLL_READY) == 0) {
		mdelay(1);
		loops++;
		if (loops > 100) {
			printk(KERN_WARNING "calibrate timeout, UTMI_PLL %x\n",
				u2o_get(base, UTMI_PLL));
			break;
		}
	}

	/* toggle VCOCAL_START bit of UTMI_PLL */
	udelay(200);
	u2o_set(base, UTMI_PLL, VCOCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_PLL, VCOCAL_START);

	/* toggle REG_RCAL_START bit of UTMI_TX */
	udelay(200);
	u2o_set(base, UTMI_TX, REG_RCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_TX, REG_RCAL_START);
	udelay(200);

	/* Make sure PHY PLL is ready */
	loops = 0;
	while ((u2o_get(base, UTMI_PLL) & PLL_READY) == 0) {
		mdelay(1);
		loops++;
		if (loops > 100) {
			printk(KERN_WARNING "calibrate timeout, UTMI_PLL %x\n",
				u2o_get(base, UTMI_PLL));
			break;
		}
	}

	if (cpu_is_pxa168() || cpu_is_mmp2()) {
		u2o_set(base, UTMI_RESERVE, 1 << 5);
		/* Turn on UTMI PHY OTG extension */
		u2o_write(base, UTMI_OTG_ADDON, 1);
	}

	init_done = 1;
	return 0;
}
#endif
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
#ifdef CONFIG_CPU_MMP3
int mmp3_hsic_phy_init(unsigned int base)
{
	u32 val;
	unsigned int otgphy;

	otgphy = (unsigned int) ioremap_nocache(PXA168_U2O_PHYBASE,
						USB_PHY_RANGE);
	if (otgphy == 0)
		printk(KERN_ERR "%s: ioremap error\n", __func__);
	pxa_usb_phy_init(otgphy);

	printk(KERN_INFO "%s: init\n", __func__);
	/* Enable hsic phy */
	val = __raw_readl(base + HSIC_CTRL);
	val |= (HSIC_CTRL_HSIC_ENABLE | HSIC_CTRL_PLL_BYPASS);
	__raw_writel(val, base + HSIC_CTRL);

	iounmap((void __iomem *)otgphy);
	return 0;
}
#endif
#endif

#endif

#ifdef CONFIG_USB_SUPPORT
static u64 usb_dma_mask = ~(u32)0;

#ifdef CONFIG_USB_PXA_U2O
struct resource pxa168_u2o_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2o = {
	.name		= "pxa-u2o",
	.id		= -1,
	.resource	= pxa168_u2o_resources,
	.num_resources	= ARRAY_SIZE(pxa168_u2o_resources),
	.dev		=  {
		.dma_mask	= &usb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};

#endif /* CONFIG_USB_PXA_U2O */

#ifdef CONFIG_USB_EHCI_PXA
#ifdef CONFIG_USB_EHCI_PXA_U2O
struct resource pxa168_u2oehci_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2oehci = {
	.name		= "pxa-u2oehci",
	.id		= 0,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(pxa168_u2oehci_resources),
	.resource	= pxa168_u2oehci_resources,
};

#ifdef CONFIG_UIO_VMETA
static struct resource mmp_vmeta_resources[3] = {
	[0] = {
		.start = 0xF0400000,
		.end   = 0xF07FFFFF,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MMP3_VMETA,
		.end   = IRQ_MMP3_VMETA,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mmp_device_vmeta = {
	.name           = UIO_VMETA_NAME,
	.id             = 0,
	.dev            = {
		.dma_mask = DMA_BIT_MASK(32),
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource       = mmp_vmeta_resources,
	.num_resources  = ARRAY_SIZE(mmp_vmeta_resources),
};

void __init mmp_register_vmeta(struct platform_device *dev, void *data)
{
	int ret;

	dev->dev.platform_data = data;

	ret = platform_device_register(dev);
	if (ret)
		dev_err(&dev->dev, "unable to register vmeta device: %d\n", ret);
}

void __init mmp_set_vmeta_info(void* info)
{
	mmp_register_vmeta(&mmp_device_vmeta, info);
}
#endif
#if defined(CONFIG_USB_PXA_U2O_OTG)
struct resource pxa168_u2ootg_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2ootg = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev  = {
		.dma_mask          = &usb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(pxa168_u2ootg_resources),
	.resource      = pxa168_u2ootg_resources,
};
#endif /* CONFIG_USB_PXA_U2O_OTG */

#endif
#endif
#endif

