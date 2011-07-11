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
#include <mach/regs-usb.h>
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

#endif
#endif
