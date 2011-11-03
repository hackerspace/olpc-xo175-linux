/*
 * Copyright (C) 2011 Marvell International Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#ifdef CONFIG_USB_OTG
#include <linux/usb/otg.h>
#endif
#include <linux/usb/mv_usb.h>

#include <plat/usb.h>

struct ehci_hcd_mv {
	struct usb_hcd			*hcd;

	/* Which mode does this ehci running OTG/Host ? */
	int				mode;

	struct mv_op_regs __iomem	*op_regs;
	struct mv_cap_regs __iomem	*cap_regs;
	unsigned int			phy_regs;

	struct otg_transceiver		*otg;

	struct mv_usb_platform_data	*pdata;

	/* clock source and total clock number */
	unsigned int			clknum;
	struct clk			*clk[0];
};

static void ehci_clock_enable(struct ehci_hcd_mv *ehci_mv)
{
	unsigned int i;

	for (i = 0; i < ehci_mv->clknum; i++)
		clk_enable(ehci_mv->clk[i]);
}

static void ehci_clock_disable(struct ehci_hcd_mv *ehci_mv)
{
	unsigned int i;

	for (i = 0; i < ehci_mv->clknum; i++)
		clk_disable(ehci_mv->clk[i]);
}

static int mv_ehci_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct device *dev = hcd->self.controller;
	struct ehci_hcd_mv *ehci_mv = dev_get_drvdata(dev);
	int retval;

	if (ehci_mv == NULL) {
		dev_err(dev, "Can not find private ehci data\n");
		return -ENODEV;
	}

	/*
	 * data structure init
	 */
	retval = ehci_init(hcd);
	if (retval) {
		dev_err(dev, "ehci_init failed %d\n", retval);
		return retval;
	}

	hcd->has_tt = 1;
	ehci->sbrn = 0x20;

	retval = ehci_reset(ehci);
	if (retval) {
		dev_err(dev, "ehci_reset failed %d\n", retval);
		return retval;
	}

	return 0;
}

static const struct hc_driver mv_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "Marvell EHCI",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset			= mv_ehci_reset,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
};

static int mv_ehci_probe(struct platform_device *dev)
{
	struct mv_usb_platform_data *pdata = dev->dev.platform_data;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct ehci_hcd_mv *ehci_mv;
	int irq, clk_i, retval = -ENODEV;
	struct resource *r;

	if (!pdata) {
		dev_err(&dev->dev, "missing platform_data\n");
		goto err_pdata;
	}

	if (usb_disabled())
		goto err_disabled;

	hcd = usb_create_hcd(&mv_ehci_hc_driver, &dev->dev, "mv ehci");
	if (!hcd) {
		retval = -ENOMEM;
		goto err_create_hcd;
	}

	ehci_mv = kzalloc(sizeof(struct ehci_hcd_mv)
			+ sizeof(struct clk *) * pdata->clknum, GFP_KERNEL);
	if (ehci_mv == NULL) {
		dev_err(&dev->dev, "cannot allocate ehci_hcd_mv\n");
		retval = -ENOMEM;
		goto err_alloc_private;
	}
	platform_set_drvdata(dev, ehci_mv);
	ehci_mv->pdata = pdata;
	ehci_mv->hcd = hcd;

	ehci_mv->clknum = pdata->clknum;
	for (clk_i = 0; clk_i < ehci_mv->clknum; clk_i++) {
		ehci_mv->clk[clk_i] = clk_get(&dev->dev, pdata->clkname[clk_i]);
		if (IS_ERR(ehci_mv->clk[clk_i])) {
			dev_err(&dev->dev, "error get clck \"%s\"\n",
				pdata->clkname[clk_i]);
			retval = PTR_ERR(ehci_mv->clk[clk_i]);
			goto err_get_clk;
		}
	}

	r = platform_get_resource_byname(dev, IORESOURCE_MEM, "capregs");
	if (!r) {
		dev_err(&dev->dev, "no I/O memory resource defined\n");
		retval = -ENODEV;
		goto err_get_cap_regs;
	}

	ehci_mv->cap_regs = (struct mv_cap_regs __iomem *)
		ioremap(r->start, resource_size(r));
	if (ehci_mv->cap_regs == NULL) {
		dev_err(&dev->dev, "failed to map I/O memory\n");
		retval = -ENOMEM;
		goto err_map_cap_regs;
	}

	r = platform_get_resource_byname(dev, IORESOURCE_MEM, "phyregs");
	if (r == NULL) {
		dev_err(&dev->dev, "no phy I/O memory resource defined\n");
		retval = -ENODEV;
		goto err_get_phy_regs;
	}

	ehci_mv->phy_regs = (unsigned int)ioremap(r->start, resource_size(r));
	if (ehci_mv->phy_regs == 0) {
		dev_err(&dev->dev, "failed to map phy I/O memory\n");
		retval = -EBUSY;
		goto err_map_phy_regs;
	}

	ehci_clock_enable(ehci_mv);
	if (pdata->phy_init) {
		retval = pdata->phy_init(ehci_mv->phy_regs);
		if (retval) {
			dev_err(&dev->dev, "init phy error %d\n", retval);
			goto err_ehci_enable;
		}
	}

	ehci_mv->op_regs = (struct mv_op_regs __iomem *)((u32)ehci_mv->cap_regs
		+ (readl(&ehci_mv->cap_regs->caplength_hciversion)
			& CAPLENGTH_MASK));

	hcd->rsrc_start = r->start;
	hcd->rsrc_len = r->end - r->start + 1;
	hcd->regs = ehci_mv->op_regs;

	irq = platform_get_irq(dev, 0);
	if (!irq) {
		dev_err(&dev->dev, "Cannot get irq %x\n", irq);
		retval = -ENODEV;
		goto err_get_irq;
	}
	hcd->irq = irq;

	ehci = hcd_to_ehci(hcd);
	ehci->caps = (struct ehci_caps *)ehci_mv->cap_regs;
	ehci->regs = (struct ehci_regs *)ehci_mv->op_regs;
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	ehci_mv->mode = pdata->mode;
	if (ehci_mv->mode == MV_USB_MODE_OTG) {
#ifdef CONFIG_USB_OTG
		ehci_mv->otg = otg_get_transceiver();
		if (!ehci_mv->otg) {
			dev_err(&dev->dev, "unable to find transceiver\n");
			retval = -ENODEV;
			goto err_get_transceiver;
		}

		retval = otg_set_host(ehci_mv->otg, &hcd->self);
		if (retval < 0) {
			dev_err(&dev->dev,
				"unable to register with transceiver\n");
			retval = -ENODEV;
			goto err_set_host;
		}
		/* otg will enable clock before use as host */
		ehci_clock_disable(ehci_mv);
		if (pdata->phy_deinit)
			pdata->phy_deinit(ehci_mv->phy_regs);
#else
		dev_info(&dev->dev,
			"MV_USB_MODE_OTG must have CONFIG_USB_OTG enabled\n");
		goto err_get_irq;
#endif
	} else {
		if (pdata->set_vbus)
			pdata->set_vbus(1);
		retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
		if (retval) {
			dev_err(&dev->dev, "failed to add hcd with err %d\n",
				retval);
			goto err_add_hcd;
		}
	}

	if (pdata->private_init)
		pdata->private_init(ehci_mv->op_regs, ehci_mv->phy_regs);

	dev_info(&dev->dev, "successful find EHCI device with regs 0x%p irq %d"
		" working in %s mode\n", hcd->regs, irq,
		ehci_mv->mode == MV_USB_MODE_OTG ? "OTG" : "Host");

	return 0;

err_add_hcd:
	if (pdata->set_vbus)
		pdata->set_vbus(0);
	if (ehci_mv->otg)
		otg_set_host(ehci_mv->otg, NULL);
err_set_host:
	if (ehci_mv->otg)
		otg_put_transceiver(ehci_mv->otg);
err_get_transceiver:
err_get_irq:
	ehci_clock_disable(ehci_mv);
err_ehci_enable:
	iounmap((void *)ehci_mv->phy_regs);
err_map_phy_regs:
err_get_phy_regs:
	iounmap(ehci_mv->cap_regs);
err_map_cap_regs:
err_get_cap_regs:
err_get_clk:
	for (clk_i--; clk_i >= 0; clk_i--)
		clk_put(ehci_mv->clk[clk_i]);
	kfree(ehci_mv);
err_alloc_private:
	usb_put_hcd(hcd);
err_create_hcd:
err_disabled:
err_pdata:
	return retval;
}

static int mv_ehci_remove(struct platform_device *dev)
{
	struct ehci_hcd_mv *ehci_mv = platform_get_drvdata(dev);
	struct usb_hcd *hcd = ehci_mv->hcd;
	int clk_i;

	if (hcd->rh_registered)
		usb_remove_hcd(hcd);

	if (ehci_mv->otg) {
		otg_set_host(ehci_mv->otg, NULL);
		otg_put_transceiver(ehci_mv->otg);
	}

	if (ehci_mv->mode == MV_USB_MODE_HOST) {
		if (ehci_mv->pdata->set_vbus)
			ehci_mv->pdata->set_vbus(0);

		ehci_clock_disable(ehci_mv);
	}

	iounmap(ehci_mv->cap_regs);
	iounmap((void *)ehci_mv->phy_regs);

	for (clk_i = 0; clk_i < ehci_mv->clknum; clk_i++)
		clk_put(ehci_mv->clk[clk_i]);

	kfree(ehci_mv);
	usb_put_hcd(hcd);
	return 0;
}

MODULE_ALIAS("mv-ehci");

static const struct platform_device_id ehci_id_table[] = {
	{ "pxa-u2oehci",	PXA_U2OEHCI },
	{ "pxa-sph",		PXA_SPH },
	{ "mmp3-hsic",		MMP3_HSIC },
	{ "mmp3-fsic",		MMP3_FSIC },
	{ },
};

static void mv_ehci_shutdown(struct platform_device *pdev)
{
	struct ehci_hcd_mv *ehci_mv = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_mv->hcd;

	if (!hcd->rh_registered)
		return;

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static struct platform_driver mv_ehci_driver = {
	.probe		= mv_ehci_probe,
	.remove		= mv_ehci_remove,
	.shutdown	= mv_ehci_shutdown,
	.driver		= {
		.name	= "pxa-ehci",
		.bus	= &platform_bus_type,
	},
	.id_table	= ehci_id_table,
};
