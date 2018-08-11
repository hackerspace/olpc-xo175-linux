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
#include "irqs.h"
#include "devices.h"
#include "cputype.h"
#include "regs-usb.h"

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

#if IS_ENABLED(CONFIG_USB_SUPPORT)
static u64 __maybe_unused usb_dma_mask = ~(u32)0;

#if IS_ENABLED(CONFIG_PHY_PXA_USB)
struct resource pxa168_usb_phy_resources[] = {
	[0] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa168_device_usb_phy = {
	.name		= "pxa-usb-phy",
	.id		= -1,
	.resource	= pxa168_usb_phy_resources,
	.num_resources	= ARRAY_SIZE(pxa168_usb_phy_resources),
	.dev		=  {
		.dma_mask	= &usb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};
#endif /* CONFIG_PHY_PXA_USB */

#if IS_ENABLED(CONFIG_USB_MV_UDC)
struct resource pxa168_u2o_resources[] = {
	[0] = {
		.start	= PXA168_U2O_REGBASE,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2o = {
	.name		= "mv-udc",
	.id		= -1,
	.resource	= pxa168_u2o_resources,
	.num_resources	= ARRAY_SIZE(pxa168_u2o_resources),
	.dev		=  {
		.dma_mask	= &usb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};
#endif /* CONFIG_USB_MV_UDC */

#if IS_ENABLED(CONFIG_USB_EHCI_MV_U2O)
struct resource pxa168_u2oehci_resources[] = {
	[0] = {
		.start	= PXA168_U2O_REGBASE,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2oehci = {
	.name		= "pxa-u2oehci",
	.id		= -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(pxa168_u2oehci_resources),
	.resource	= pxa168_u2oehci_resources,
};
#endif

#if IS_ENABLED(CONFIG_USB_MV_OTG)
struct resource pxa168_u2ootg_resources[] = {
	[0] = {
		.start	= PXA168_U2O_REGBASE,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2ootg = {
	.name		= "mv-otg",
	.id		= -1,
	.dev  = {
		.dma_mask          = &usb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(pxa168_u2ootg_resources),
	.resource      = pxa168_u2ootg_resources,
};
#endif /* CONFIG_USB_MV_OTG */

#endif
