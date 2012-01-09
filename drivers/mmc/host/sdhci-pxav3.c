/*
 * Copyright (C) 2010 Marvell International Ltd.
 *		Zhangfei Gao <zhangfei.gao@marvell.com>
 *		Kevin Wang <dwang4@marvell.com>
 *		Mingwei Wang <mwwang@marvell.com>
 *		Philip Rakity <prakity@marvell.com>
 *		Mark Brown <markb@marvell.com>
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
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include "sdhci.h"
#include "sdhci-pltfm.h"
#include <plat/pm.h>

#define SD_CLOCK_BURST_SIZE_SETUP		0x10A
#define SDCLK_SEL	0x100
#define SDCLK_DELAY_SHIFT	9
#define SDCLK_DELAY_MASK	0x1f

#define SD_CFG_FIFO_PARAM       0x100
#define SDCFG_GEN_PAD_CLK_ON	(1<<6)
#define SDCFG_GEN_PAD_CLK_CNT_MASK	0xFF
#define SDCFG_GEN_PAD_CLK_CNT_SHIFT	24

#define SD_SPI_MODE          0x108
#define SD_CE_ATA_1          0x10C

#define SD_CE_ATA_2          0x10E
#define SDCE_MISC_INT		(1<<2)
#define SDCE_MISC_INT_EN	(1<<1)

static void pxav3_set_private_registers(struct sdhci_host *host, u8 mask)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (mask == SDHCI_RESET_ALL) {
		/*
		 * tune timing of read data/command when crc error happen
		 * no performance impact
		 */
		if (pdata && 0 != pdata->clk_delay_cycles) {
			u16 tmp;

			tmp = readw(host->ioaddr + SD_CLOCK_BURST_SIZE_SETUP);
			tmp |= (pdata->clk_delay_cycles & SDCLK_DELAY_MASK)
				<< SDCLK_DELAY_SHIFT;
			tmp |= SDCLK_SEL;
			writew(tmp, host->ioaddr + SD_CLOCK_BURST_SIZE_SETUP);
		}
	}
}

static int sdhci_pxa_safe_regulator_on(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_pxa *pxa;
	int ret = 0;

	pltfm_host = sdhci_priv(host);
	pxa = pltfm_host->priv;

	if (pxa && pxa->pdata && pxa->pdata->check_short_circuit)
		if (pxa->pdata->check_short_circuit(host,
			pxa->pdata->mfp_start,
			pxa->pdata->mfp_num,
			pxa->pdata->pull_up)) {

			pr_info("WARNING SD short circuit detected\n");
			ret = 1;
		}

	if (!ret) {
		if (pxa && pxa->pdata && pxa->pdata->safe_regulator_on)
			pxa->pdata->safe_regulator_on(host,
				pxa->pdata->mfp_start,
				pxa->pdata->mfp_num);
		else
			regulator_enable(host->vmmc);
	}

	return ret;
}

#define MAX_WAIT_COUNT 5
static void pxav3_gen_init_74_clocks(struct sdhci_host *host, u8 power_mode)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	u16 tmp;
	int count;

	if (pxa->power_mode == MMC_POWER_UP
			&& power_mode == MMC_POWER_ON) {

		dev_dbg(mmc_dev(host->mmc),
				"%s: slot->power_mode = %d,"
				"ios->power_mode = %d\n",
				__func__,
				pxa->power_mode,
				power_mode);

		/* set we want notice of when 74 clocks are sent */
		tmp = readw(host->ioaddr + SD_CE_ATA_2);
		tmp |= SDCE_MISC_INT_EN;
		writew(tmp, host->ioaddr + SD_CE_ATA_2);

		/* start sending the 74 clocks */
		tmp = readw(host->ioaddr + SD_CFG_FIFO_PARAM);
		tmp |= SDCFG_GEN_PAD_CLK_ON;
		writew(tmp, host->ioaddr + SD_CFG_FIFO_PARAM);

		/* slowest speed is about 100KHz or 10usec per clock */
		udelay(740);
		count = 0;

		while (count++ < MAX_WAIT_COUNT) {
			if ((readw(host->ioaddr + SD_CE_ATA_2)
						& SDCE_MISC_INT) == 0)
				break;
			udelay(10);
		}

		if (count == MAX_WAIT_COUNT)
			dev_warn(mmc_dev(host->mmc), "74 clock interrupt not cleared\n");

		/* clear the interrupt bit if posted */
		tmp = readw(host->ioaddr + SD_CE_ATA_2);
		tmp |= SDCE_MISC_INT;
		writew(tmp, host->ioaddr + SD_CE_ATA_2);
	}
	pxa->power_mode = power_mode;
}

static int pxav3_set_uhs_signaling(struct sdhci_host *host, unsigned int uhs)
{
	u16 ctrl_2;

	/*
	 * Set V18_EN -- UHS modes do not work without this.
	 * does not change signaling voltage
	 */
	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50 | SDHCI_CTRL_VDD_180;
		break;
	case MMC_TIMING_UHS_SDR104:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104 | SDHCI_CTRL_VDD_180;
		break;
	case MMC_TIMING_UHS_DDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50 | SDHCI_CTRL_VDD_180;
		break;
	}

	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);
	dev_dbg(mmc_dev(host->mmc),
		"%s uhs = %d, ctrl_2 = %04X\n",
		__func__, uhs, ctrl_2);

	return 0;
}

static void pxav3_signal_vol_change(struct sdhci_host *host, u8 vol)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (pdata && pdata->signal_1v8)
		pdata->signal_1v8(MMC_SIGNAL_VOLTAGE_180 == vol);
}

static void pxav3_access_constrain(struct sdhci_host *host, unsigned int ac)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (!pdata)
		return;

#ifdef CONFIG_WAKELOCK
	if (ac)
		wake_lock(&pdata->idle_lock);
	else
		wake_unlock(&pdata->idle_lock);
#endif
	if (ac)
		pm_qos_update_request(&pdata->qos_idle, PM_QOS_CONSTRAINT);
	else
		pm_qos_update_request(&pdata->qos_idle, PM_QOS_DEFAULT_VALUE);
}

static void ext_cd_notify_change(struct platform_device *pdev, int state)
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

static int pxav3_ext_cd_status(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_pxa *pxa;

	pltfm_host = sdhci_priv(host);
	pxa = pltfm_host->priv;
	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)
		return 1;
	else
		return ext_cd_val(pxa->pdata->ext_cd_gpio,pxa->pdata->ext_cd_gpio_invert);
}

static irqreturn_t ext_cd_irq_thread(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	int status;

	pr_info("sdcard gpio detected\n");

	msleep(600);
	status = ext_cd_val(pdata->ext_cd_gpio, pdata->ext_cd_gpio_invert);
	ext_cd_notify_change(pdev, status);

	return IRQ_HANDLED;
}

static struct sdhci_ops pxav3_sdhci_ops = {
	.platform_reset_exit = pxav3_set_private_registers,
	.set_uhs_signaling = pxav3_set_uhs_signaling,
	.platform_send_init_74_clocks = pxav3_gen_init_74_clocks,
	.signal_vol_change = pxav3_signal_vol_change,
	.access_constrain = pxav3_access_constrain,
	.safe_regulator_on = sdhci_pxa_safe_regulator_on,
};

static int ext_cd_init(void *data)
{
	struct platform_device *pdev = data;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	int err, cd_irq, ext_cd_gpio;
	int status;

	cd_irq = gpio_to_irq(pdata->ext_cd_gpio);
	ext_cd_gpio = pdata->ext_cd_gpio;

	/*
	 * setup GPIO for saarb MMC controller
	 */
	err = gpio_request(ext_cd_gpio, "mmc card detect");
	if (err) {
		printk(KERN_ERR "gpio_request err =%d\n", err);
		goto err_request_cd;
	}
	gpio_direction_input(ext_cd_gpio);

	err = request_threaded_irq(cd_irq, NULL, ext_cd_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				   "MMC card detect", pdev);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
		       "can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	pr_info("sdcard gpio detect init done\n");
	status = ext_cd_val(pdata->ext_cd_gpio, pdata->ext_cd_gpio_invert);
	ext_cd_notify_change(pdev, status);

	pxav3_sdhci_ops.is_present = pxav3_ext_cd_status;

	return 0;

err_request_irq:
	gpio_free(ext_cd_gpio);
err_request_cd:
	return -1;
}


static int sdhci_pxav3_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	int ret = 0;

	if(pxa->pdata->flags & PXA_FLAG_KEEP_POWER_IN_SUSPEND)
		return ret;

	if (atomic_read(&host->mmc->suspended)) {
		printk(KERN_WARNING"%s already suspended\n", mmc_hostname(host->mmc));
		return ret;
	}
	atomic_inc(&host->mmc->suspended);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(host->irq);

	ret = sdhci_suspend_host(host, state);
	if (ret) {
		atomic_dec(&host->mmc->suspended);
		return ret;
	}

	if (pdata->lp_switch) {
		ret = pdata->lp_switch(1, (int)host->mmc->card);
		if (ret) {
			atomic_dec(&host->mmc->suspended);
			sdhci_resume_host(host);
			dev_err(&pdev->dev, "fail to switch gpio, resume..\n");
		}
	}

	return ret;
}

static int sdhci_pxav3_resume(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	int ret = 0;

	if(pxa->pdata->flags & PXA_FLAG_KEEP_POWER_IN_SUSPEND)
		return ret;

	if (pdata->lp_switch)
		pdata->lp_switch(0, (int)host->mmc->card);

	ret = sdhci_resume_host(host);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(host->irq);

	atomic_dec(&host->mmc->suspended);

	return ret;
}

static int __devinit sdhci_pxav3_probe(struct platform_device *pdev)
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
	clk_enable(clk);

	host->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL
		| SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;

	if (pdata) {
		if (pdata->flags & PXA_FLAG_CARD_PERMANENT) {
			/* on-chip device */
			host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
			host->mmc->caps |= MMC_CAP_NONREMOVABLE;
		}

		/* If slot design supports 8 bit data, indicate this to MMC. */
		if (pdata->flags & PXA_FLAG_SD_8_BIT_CAPABLE_SLOT)
			host->mmc->caps |= MMC_CAP_8_BIT_DATA;

		if (pdata->flags & PXA_FLAG_ENABLE_CLOCK_GATING)
			host->mmc->caps |= MMC_CAP_ENABLE_BUS_CLK_GATING;

		if (pdata->handle_cdint)
			pxav3_sdhci_ops.handle_cdint = pdata->handle_cdint;

		if (pdata->quirks)
			host->quirks |= pdata->quirks;
		if (pdata->host_caps)
			host->mmc->caps |= pdata->host_caps;
		if (pdata->pm_caps)
			host->mmc->pm_caps |= pdata->pm_caps;

	pm_qos_add_request(&pdata->qos_idle, PM_QOS_CPU_DMA_LATENCY,
			PM_QOS_DEFAULT_VALUE);
	#ifdef CONFIG_WAKELOCK
		wake_lock_init(&pdata->idle_lock, WAKE_LOCK_IDLE,
			(const char *)mmc_hostname(host->mmc));
	#endif
	}

	host->ops = &pxav3_sdhci_ops;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "failed to add host\n");
		goto err_add_host;
	}

	platform_set_drvdata(pdev, host);

	if (pdata->flags & PXA_FLAG_WAKEUP_HOST)
		device_init_wakeup(&pdev->dev, 1);
	else
		device_init_wakeup(&pdev->dev, 0);

	if (pdata && pdata->ext_cd_gpio)
		ext_cd_init(pdev);

	pxa->pdata = pdata;
#ifdef CONFIG_SD8XXX_RFKILL
	if (pxa->pdata->pmmc)
		*pxa->pdata->pmmc = host->mmc;
#endif

	return 0;

err_add_host:
	clk_disable(clk);
	clk_put(clk);
err_clk_get:
	pm_qos_remove_request(&pdata->qos_idle);
#ifdef CONFIG_WAKELOCK
	wake_lock_destroy(&pdata->idle_lock);
#endif
	sdhci_pltfm_free(pdev);
	kfree(pxa);
	return ret;
}

static int __devexit sdhci_pxav3_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;

	sdhci_remove_host(host, 1);

	pm_qos_remove_request(&pxa->pdata->qos_idle);
#ifdef CONFIG_WAKELOCK
	wake_lock_destroy(&pxa->pdata->idle_lock);
#endif

	clk_disable(pltfm_host->clk);
	clk_put(pltfm_host->clk);
	sdhci_pltfm_free(pdev);
	kfree(pxa);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver sdhci_pxav3_driver = {
	.driver		= {
		.name	= "sdhci-pxa",
		.owner	= THIS_MODULE,
	},
	.probe		= sdhci_pxav3_probe,
	.remove		= __devexit_p(sdhci_pxav3_remove),
#ifdef CONFIG_PM
	.suspend	= sdhci_pxav3_suspend,
	.resume		= sdhci_pxav3_resume,
#endif
};
static int __init sdhci_pxav3_init(void)
{
	return platform_driver_register(&sdhci_pxav3_driver);
}

static void __exit sdhci_pxav3_exit(void)
{
	platform_driver_unregister(&sdhci_pxav3_driver);
}

module_init(sdhci_pxav3_init);
module_exit(sdhci_pxav3_exit);

MODULE_DESCRIPTION("SDHCI driver for pxav3");
MODULE_AUTHOR("Marvell International Ltd.");
MODULE_LICENSE("GPL v2");

