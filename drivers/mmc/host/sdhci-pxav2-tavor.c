/*
 * Copyright (C) 2010 Marvell International Ltd.
 *		Zhangfei Gao <zhangfei.gao@marvell.com>
 *		Kevin Wang <dwang4@marvell.com>
 *		Jun Nie <njun@marvell.com>
 *		Qiming Wu <wuqm@marvell.com>
 *		Philip Rakity <prakity@marvell.com>
 *		Raymond Wu <xywu@marvell.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/slab.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <mach/dvfm.h>

#include "sdhci.h"
#include "sdhci-pltfm.h"

static void sdhci_pxa_notify_change(struct platform_device *pdev, int state)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	unsigned long flags;
	static int old_state = 0;

	if (host) {
		spin_lock_irqsave(&host->lock, flags);
		if (state && !old_state) {
			old_state = state;
			dev_dbg(&pdev->dev, "card inserted.\n");
			host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
			spin_unlock_irqrestore(&host->lock, flags);
			if (host->vmmc) {
				pr_info("mmc power on vmmc=0x%x\n",
					(int)host->vmmc);
				regulator_enable(host->vmmc);
			}
		} else if(!state && old_state) {
			old_state = state;
			dev_dbg(&pdev->dev, "card removed.\n");
			host->quirks &= ~SDHCI_QUIRK_BROKEN_CARD_DETECTION;

			if (host->mrq) {
				printk(KERN_ERR
				       "%s: Card removed during transfer!\n",
				       mmc_hostname(host->mmc));

				host->mrq->cmd->error = -ENOMEDIUM;
				tasklet_schedule(&host->finish_tasklet);
			}

			spin_unlock_irqrestore(&host->lock, flags);
			if (host->vmmc) {
				pr_info("mmc power off vmmc=0x%x\n",
					(int)host->vmmc);
				regulator_disable(host->vmmc);
			}
		} else
			spin_unlock_irqrestore(&host->lock, flags);

		mmc_detect_change(host->mmc, msecs_to_jiffies(200));
	}
}

static inline int ext_cd_val(int gpio, int invert)
{
	int status;

	status = gpio_get_value(gpio);
	status &= (1 << (gpio % 32));
	status = !!status;

	if (invert)
		status = !status;

	return status;
}

#define WAKE_LOCK_CD_TIMEOUT (15*HZ)
static struct wake_lock cd_wake_lock;

static irqreturn_t sdhci_pxa_cd_irq_thread(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	int status;

	wake_lock_timeout(&cd_wake_lock, WAKE_LOCK_CD_TIMEOUT);

	msleep(600);
	status = ext_cd_val(pdata->ext_cd_gpio, pdata->ext_cd_gpio_invert);
	sdhci_pxa_notify_change(pdev, status);

	return IRQ_HANDLED;
}

static int ext_cd_init(void *data)
{
	struct platform_device *pdev = data;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	int err, cd_irq, ext_cd_gpio;
	int status;

	cd_irq = gpio_to_irq(pdata->ext_cd_gpio);
	ext_cd_gpio = pdata->ext_cd_gpio;
	/* Catch wake lock when card is inserted or removed */
	wake_lock_init(&cd_wake_lock, WAKE_LOCK_SUSPEND, "sd_card_detect");

	/*
	 * setup GPIO for saarb MMC controller
	 */
	err = gpio_request(ext_cd_gpio, "mmc card detect");
	if (err) {
		printk(KERN_ERR "gpio_request err =%d\n", err);
		goto err_request_cd;
	}
	gpio_direction_input(ext_cd_gpio);

	err = request_threaded_irq(cd_irq, NULL, sdhci_pxa_cd_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				   "MMC card detect", pdev);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
		       "can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	status = ext_cd_val(pdata->ext_cd_gpio, pdata->ext_cd_gpio_invert);
	sdhci_pxa_notify_change(pdev, status);

	return 0;

err_request_irq:
	gpio_free(ext_cd_gpio);
err_request_cd:
	return -1;
}

static int pxav2_mmc_set_width(struct sdhci_host *host, int width)
{
	u8 ctrl;

	ctrl = readb(host->ioaddr + SDHCI_HOST_CONTROL);
	if (width == MMC_BUS_WIDTH_8) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (width == MMC_BUS_WIDTH_4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}
	writeb(ctrl, host->ioaddr + SDHCI_HOST_CONTROL);

	return 0;
}

static u32 pxav2_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return clk_get_rate(pltfm_host->clk);
}

static void pxav2_access_constrain(struct sdhci_host *host, unsigned int ac)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;

	if (!ac && pxa->clk_enable) {
		clk_disable(pltfm_host->clk);
		pxa->clk_enable = 0;
	} else if (ac && !pxa->clk_enable) {
		clk_enable(pltfm_host->clk);
		pxa->clk_enable = 1;
	}

	if(ac && (pltfm_host->dvfm_dev_idx > 0))
		dvfm_disable_lowpower(pltfm_host->dvfm_dev_idx);
	else if(!ac && (pltfm_host->dvfm_dev_idx > 0))
		dvfm_enable_lowpower(pltfm_host->dvfm_dev_idx);
}

static struct sdhci_ops pxav2_sdhci_ops = {
	.get_max_clock = pxav2_get_max_clock,
	.platform_8bit_width = pxav2_mmc_set_width,
	.access_constrain = pxav2_access_constrain,
};

static int __devinit sdhci_pxav2_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host = NULL;
	struct sdhci_pxa *pxa = NULL;
	int ret;
	struct clk *clk;

	pxa = kzalloc(sizeof(struct sdhci_pxa), GFP_KERNEL);
	if (!pxa)
		return -ENOMEM;

	host = sdhci_pltfm_init(pdev, NULL);
	if (IS_ERR(host)) {
		kfree(pxa);
		return PTR_ERR(host);
	}
	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = pxa;

	clk = clk_get(dev, "PXA-SDHCLK");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(clk);
		goto err_clk_get;
	}
	pltfm_host->clk = clk;

	ret = dvfm_register((char *)mmc_hostname(host->mmc), &pltfm_host->dvfm_dev_idx);
	if(ret) {
		pr_err("Error %d: Fails to register %s into dvfm.\n",
				ret, mmc_hostname(host->mmc));
		goto err_clk_get;
	}

	pxav2_access_constrain(host, 1);

	host->quirks = SDHCI_QUIRK_BROKEN_ADMA
	    | SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;

	if (pdata) {
		if (pdata->flags & PXA_FLAG_CARD_PERMANENT) {
			/* on-chip device */
			host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
			host->mmc->caps |= MMC_CAP_NONREMOVABLE;
		}

		/* If slot design supports 8 bit data, indicate this to MMC. */
		if (pdata->flags & PXA_FLAG_SD_8_BIT_CAPABLE_SLOT)
			host->mmc->caps |= MMC_CAP_8_BIT_DATA;

		if (pdata && pdata->flags & PXA_FLAG_ACITVE_IN_SUSPEND) {
			host->mmc->pm_flags |= MMC_PM_ALWAYS_ACTIVE;
		}

		if (!(pdata->flags & PXA_FLAG_ENABLE_CLOCK_GATING))
			host->mmc->caps |= MMC_CAP_DISABLE_BUS_CLK_GATING;

		if (pdata->quirks)
			host->quirks |= pdata->quirks;
		if (pdata->host_caps)
			host->mmc->caps |= pdata->host_caps;
		if (pdata->pm_caps)
			host->mmc->pm_caps |= pdata->pm_caps;
	}

	host->ops = &pxav2_sdhci_ops;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "failed to add host\n");
		goto err_add_host;
	}

	platform_set_drvdata(pdev, host);
	pxa->pdata = pdata;

	if (pdata && pdata->ext_cd_gpio)
		ext_cd_init(pdev);

	device_init_wakeup(&pdev->dev, 0);
#ifdef CONFIG_SD8XXX_RFKILL
	if (pxa->pdata->pmmc)
		*pxa->pdata->pmmc = host->mmc;
#endif

	pxav2_access_constrain(host, 0);

	return 0;

err_add_host:
	pxav2_access_constrain(host, 0);
	clk_put(clk);
err_clk_get:
	sdhci_pltfm_free(pdev);
	kfree(pxa);
	return ret;
}

static int __devexit sdhci_pxav2_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;

	if (pxa->pdata && pxa->pdata->ext_cd_gpio) {
		int cd_irq = gpio_to_irq(pxa->pdata->ext_cd_gpio);
		if (cd_irq)
			free_irq(cd_irq, pdev);

		if (gpio_is_valid(pxa->pdata->ext_cd_gpio))
			gpio_free(pxa->pdata->ext_cd_gpio);
	}

	pxav2_access_constrain(host, 1);
	sdhci_remove_host(host, 1);

	pxav2_access_constrain(host, 0);
	clk_put(pltfm_host->clk);
	dvfm_unregister((char *)mmc_hostname(host->mmc), &pltfm_host->dvfm_dev_idx);
	sdhci_pltfm_free(pdev);
	kfree(pxa);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver sdhci_pxav2_tavor_driver = {
	.driver = {
		   .name = "sdhci-pxa",
		   .owner = THIS_MODULE,
		   },
	.probe = sdhci_pxav2_probe,
	.remove = __devexit_p(sdhci_pxav2_remove),
#ifdef CONFIG_PM
	.suspend = sdhci_pltfm_suspend,
	.resume = sdhci_pltfm_resume,
#endif
};

static int __init sdhci_pxav2_tavor_init(void)
{
	return platform_driver_register(&sdhci_pxav2_tavor_driver);
}

static void __exit sdhci_pxav2_tavor_exit(void)
{
	platform_driver_unregister(&sdhci_pxav2_tavor_driver);
}

module_init(sdhci_pxav2_tavor_init);
module_exit(sdhci_pxav2_tavor_exit);

MODULE_DESCRIPTION("SDHCI driver for pxav2-tavor");
MODULE_AUTHOR("Marvell International Ltd.");
MODULE_LICENSE("GPL v2");
