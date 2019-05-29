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

	/* record the clock status */
	unsigned int			active;

	unsigned int			port_speed;
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

static int mv_ehci_enable(struct ehci_hcd_mv *ehci_mv)
{
	int retval;

	if (ehci_mv->active) {
		pr_info("mv ehci is enabled already...\n");
		return 0;
	}

	ehci_clock_enable(ehci_mv);
	if (ehci_mv->pdata->phy_init) {
		retval = ehci_mv->pdata->phy_init(ehci_mv->phy_regs);
		if (retval) {
			pr_err("Host: init phy error %d\n", retval);
			ehci_clock_disable(ehci_mv);
			return retval;
		}
	}

	ehci_mv->active = 1;

	return 0;
}

static void mv_ehci_disable(struct ehci_hcd_mv *ehci_mv)
{
	if (ehci_mv->active) {
		if (ehci_mv->pdata->phy_deinit)
			ehci_mv->pdata->phy_deinit(ehci_mv->phy_regs);
		ehci_clock_disable(ehci_mv);
		ehci_mv->active = 0;
	}
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
	ehci_port_power(ehci, 1);

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
	.endpoint_reset		= ehci_endpoint_reset,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,

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

	retval = mv_ehci_enable(ehci_mv);
	if (retval) {
		dev_err(&dev->dev, "init phy error %d\n", retval);
		goto err_ehci_enable;
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
		mv_ehci_disable(ehci_mv);
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
	mv_ehci_disable(ehci_mv);
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

		mv_ehci_disable(ehci_mv);
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

#ifdef CONFIG_PM
static int mv_ehci_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	struct ehci_hcd_mv *ehci_mv = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_mv->hcd;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct ehci_regs __iomem *hc_reg = ehci->regs;
	unsigned long flags;
	int    rc = 0;

	/* For OTG, the following will be done in OTG driver*/
	if (!strcmp(pdev->name, "pxa-u2oehci"))
		return 0;

	/* The following is for HSIC host */
	if (time_before(jiffies, ehci->next_statechange))
		msleep(20);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}

	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	/* Store port speed before suspend */
	ehci_mv->port_speed =
		(ehci_readl(ehci, &hc_reg->port_status[0]) >> 26) & 0x3;


	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

bail:
	spin_unlock_irqrestore(&ehci->lock, flags);

	if (ehci_mv->pdata->set_vbus)
		ehci_mv->pdata->set_vbus(0);

	mv_ehci_disable(ehci_mv);

	return rc;
}

static int mv_ehci_resume(struct platform_device *pdev)
{
	struct ehci_hcd_mv *ehci_mv = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_mv->hcd;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct ehci_regs __iomem *hc_reg = ehci->regs;
	int retval = 0;
	u32 val;

	/* For OTG, the following will be done in OTG driver*/
	if (!strcmp(pdev->name, "pxa-u2oehci"))
		return 0;

	/* The following is for HSIC host */
	retval = mv_ehci_enable(ehci_mv);
	if (retval) {
		dev_err(&pdev->dev, "init phy error %d\n", retval);
		return 0;
	}

	if (ehci_mv->pdata->set_vbus) {
		retval = ehci_mv->pdata->set_vbus(1);
		if (retval) {
			dev_err(&pdev->dev, "Failed set vbus: %d\n", retval);
			goto out;
		}
	}

	if (ehci_mv->pdata->private_init)
		ehci_mv->pdata->private_init(ehci_mv->op_regs,
					ehci_mv->phy_regs);

	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* Mark hardware accessible again */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	/* Enable USB Port Power */
	val = ehci_readl(ehci, &hc_reg->port_status[0]);
	val |= PORT_POWER;
	ehci_writel(ehci, val, &hc_reg->port_status[0]);
	udelay(20);

	/* Force USB contorller to port_speed before suspend */
	if (!ehci_readl(ehci, &hc_reg->async_next)) {
		val = ehci_readl(ehci, &hc_reg->port_status[0]) & ~(0xf << 16);
		switch (ehci_mv->port_speed) {
		case USB_PORT_SPEED_HIGH:
			val |= 0x5 << 16;
			break;
		case USB_PORT_SPEED_FULL:
			val |= 0x6 << 16;
			break;
		case USB_PORT_SPEED_LOW:
			val |= 0x7 << 16;
			break;
		default:
			dev_err(&pdev->dev, "No such type speed support.");
			goto out;
		}

		ehci_writel(ehci, val, &hc_reg->port_status[0]);
		udelay(20);

		/* Disable Test Mode */
		val = ehci_readl(ehci, &hc_reg->port_status[0]);
		val &= ~(0xf << 16);
		ehci_writel(ehci, val, &hc_reg->port_status[0]);
		udelay(20);
	}

	/* Wait for PORT_CONNECT assert */
	if (handshake(ehci, &hc_reg->port_status[0],
			PORT_CONNECT, PORT_CONNECT, 5000)) {
		pr_err("%s:waiting for PORT_CONNECT timeout!\n", __func__);
		goto out;
	}

	/* Wait for PORT_PE assert */
	if (handshake(ehci, &hc_reg->port_status[0],
			PORT_PE, PORT_PE, 5000)) {
		pr_err("%s:waiting for PORT_PE timeout!\n", __func__);
		goto out;
	}

	/* Port change detect */
	val = ehci_readl(ehci, &hc_reg->status);
	val |= STS_PCD;
	ehci_writel(ehci, val, &hc_reg->status);

	/* Place USB Controller in Suspend Mode, will resume later */
	val = ehci_readl(ehci, &hc_reg->port_status[0]);
	if ((val & PORT_POWER) && (val & PORT_PE)) {
		val |= PORT_SUSPEND;
		ehci_writel(ehci, val, &hc_reg->port_status[0]);
		/* Wait for PORT_SUSPEND assert */
		if (handshake(ehci, &hc_reg->port_status[0],
				PORT_SUSPEND, PORT_SUSPEND, 5000)) {
			pr_err("%s:waiting for PORT_SUSPEND timeout!\n",
					__func__);
			goto out;
		}
	}

out:
	return 0;
}
#endif

static struct platform_driver mv_ehci_driver = {
	.probe		= mv_ehci_probe,
	.remove		= mv_ehci_remove,
	.shutdown	= mv_ehci_shutdown,
#ifdef CONFIG_PM
	.suspend	= mv_ehci_suspend,
	.resume		= mv_ehci_resume,
#endif
	.driver		= {
		.name	= "pxa-ehci",
		.bus	= &platform_bus_type,
	},
	.id_table	= ehci_id_table,
};
