/*
 * Copyright 2012 Marvell Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/* MMP3 Thermal Implementation */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/io.h>
#include <linux/syscalls.h>
#include <linux/cpufreq.h>
#include <linux/smp.h>
#include <mach/regs-apbc.h>

/* This function decode 4bit long number of gray code into original binary */
static int gray_decode(unsigned int gray)
{
	int num, i, tmp;

	if (gray >= 16)
		return 0;

	num = gray & 0x8;
	tmp = num >> 3;
	for (i = 2; i >= 0; i --) {
		tmp = ((gray & (1 << i)) >> i) ^ tmp;
		num |= tmp << i;
	}

	return num;
}

unsigned long read_temperature_sensor(int index)
{
	int i;
	unsigned long data,lower_temp;

	__raw_writel(readl(CPBC_REG(index * 4)) | (1 << 27),
			CPBC_REG(index * 4));
	__raw_writel(readl(CPBC_REG(index * 4)) | (1 << 30),
			CPBC_REG(index * 4));
	for (i = 0; i < 1000; i++) {
		if (readl(CPBC_REG(index * 4)) & (1 << 29))
			break;
		udelay(1000);
	}
	if (i == 1000)
		printk(KERN_EMERG "timeout to get sensor %d temperature!\n",
				index);

	data = readl(CPBC_REG(index * 4)) & 0xf;
	lower_temp = gray_decode(data) * 5 / 2 + 26;

	if ( lower_temp > 60 ){
       		__raw_writel(readl(CPBC_REG(index * 4)) & ~(1 << 27),CPBC_REG(index * 4));
                __raw_writel(readl(CPBC_REG(index * 4)) | (1 << 30),CPBC_REG(index * 4));
	        for (i = 0; i < 1000; i++) {
			if (readl(CPBC_REG(index * 4)) & (1 << 29))
				break;
			udelay(1000);
		}
		if (i == 1000)
		printk(KERN_EMERG "timeout to get sensor %d temperature!\n",index);
		data = readl(CPBC_REG(index * 4)) & 0xf;
		if (data == 0) return lower_temp;
		else return gray_decode(data) * 5 / 2 + 78;
	}else {
		return lower_temp;
	}
}
EXPORT_SYMBOL(read_temperature_sensor);

static int th_sys_get_temp(struct thermal_zone_device *thermal,
		unsigned long *temp)
{
/* return MAX termerature */	
	*temp = (read_temperature_sensor(0) > read_temperature_sensor(1)) ? read_temperature_sensor(0) : read_temperature_sensor(1);
	*temp = (*temp > read_temperature_sensor(2)) ? *temp : read_temperature_sensor(2);
/*	
	*temp = (read_temperature_sensor(0) +
			read_temperature_sensor(1) +
			read_temperature_sensor(2)) / 3;
*/
	return 0;
}

static int th_sys_get_mode(struct thermal_zone_device *thermal,
		enum thermal_device_mode *mode)
{
	*mode = THERMAL_DEVICE_ENABLED;
	return 0;
}

static int th_sys_get_trip_type(struct thermal_zone_device *thermal, int trip,
		enum thermal_trip_type *type)
{
	return 0;
}

static int th_sys_get_trip_temp(struct thermal_zone_device *thermal, int trip,
		unsigned long *temp)
{
	return 0;
}

static int th_sys_get_crit_temp(struct thermal_zone_device *thermal,
		unsigned long *temp)
{
	return -EINVAL;
}

static struct thermal_zone_device_ops g_dev_ops = {
	.get_temp = th_sys_get_temp,
	.get_mode = th_sys_get_mode,
	.get_trip_type = th_sys_get_trip_type,
	.get_trip_temp = th_sys_get_trip_temp,
	.get_crit_temp = th_sys_get_crit_temp,
};

static irqreturn_t thermal_irq(int irq, void *devid)
{
	int i;
	for (i = 0; i < 4; i ++)
		__raw_writel(readl(CPBC_REG(i * 4)) | 0x92000000,
				CPBC_REG(i *4));
	return IRQ_HANDLED;
}

#define MMP3_THERMAL_POLLING_FREQUENCY_MS 1000
static struct thermal_zone_device      *therm_dev;
static int mmp3_thermal_probe(struct platform_device *pdev)
{
	int ret = 0, irq;
	struct clk* therclk;

	therclk = clk_get(NULL, "THERMALCLK");
	if (IS_ERR(therclk)) {
		printk(KERN_ERR "Could not get thermal clock\n");
		return PTR_ERR(therclk);
	}

	clk_enable(therclk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		return -ENXIO;
	}

	ret = request_irq(irq, thermal_irq, IRQF_DISABLED,
			  pdev->name, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		return -ENXIO;
	}

	therm_dev = thermal_zone_device_register(
			"mmp3-temp_sens", 4,
			NULL, &g_dev_ops, 0, 0, MMP3_THERMAL_POLLING_FREQUENCY_MS, 0);

	if (IS_ERR(therm_dev)) {
		pr_err("Failed to register thermal zone device\n");
		ret = -EINVAL;
	}

	return ret;
}

static int mmp3_thermal_remove(struct platform_device *pdev)
{
	thermal_zone_device_unregister(therm_dev);
	pr_info("MMP3: Kernel Thermal management unregistered\n");

	return 0;
}

static struct platform_driver mmp3_thermal_driver = {
	.driver = {
		.name	= "mmp-thermal",
		.owner	= THIS_MODULE,
	},
	.probe		= mmp3_thermal_probe,
	.remove		= mmp3_thermal_remove,
};

static int __init mmp3_thermal_init(void)
{
	return platform_driver_register(&mmp3_thermal_driver);
}

static void __exit mmp3_thermal_exit(void)
{
	platform_driver_unregister(&mmp3_thermal_driver);
}

module_init(mmp3_thermal_init);
module_exit(mmp3_thermal_exit);

MODULE_AUTHOR("Marvell Semiconductor");
MODULE_DESCRIPTION("MMP3 SoC thermal driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mmp3-thermal");
