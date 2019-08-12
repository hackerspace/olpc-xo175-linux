/*
 * Copyright 2012 Marvell Semiconductor, Inc.
 * Copyright 2012 One Laptop per Child
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
#include <linux/of_address.h>

void mmp3_thermal_modify_watchdog(void);

static int param_set_watchdog(const char *val, const struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	mmp3_thermal_modify_watchdog();

	return 0;
}

static const struct kernel_param_ops param_ops_watchdog = {
	.set = param_set_watchdog,
	.get = param_get_int,
};


/* the value of the "enable_watchdog" module parameter is 0 to
 * disable, otherwise it's an index into gray_hirange[], below.
 */
#define DEFAULT_WDOG_TEMP_LIMIT		13	// 100.5
static int enable_watchdog = DEFAULT_WDOG_TEMP_LIMIT;
module_param_cb(enable_watchdog, &param_ops_watchdog, &enable_watchdog, 0644);

// debug level:
//	bit 0 to show all reads,
//	bit 1 to show final filtered sensor values
static int debug;
module_param(debug, int, 0644);

static struct thermal_zone_device *therm_dev;
static void __iomem *therm_base;

#define THERM_BASE(x)	(therm_base + (x))

#define TEMP_RESERVED		(1 << 31) // undoc'd, appears in original marvell code
#define TEMP_START		(1 << 30)
#define TEMP_STATUS		(1 << 29)
#define TEMP_OVER_INT		(1 << 28)
#define TEMP_LOWRANGE		(1 << 27)
#define TEMP_EN_WDOG		(1 << 26)
#define TEMP_TEMP_INT		(1 << 25)
#define TEMP_EN_OVER_INT	(1 << 24)
#define TEMP_EN_TEMP_INT	(1 << 23)
#define TEMP_AUTO_READ		(1 << 21) // undoc'd, useful for watchdog but causes interrupt storm
#define TEMP_WDOG_THSHLD	0x0f00
#define TEMP_WDOG_SHIFT		8
#define TEMP_INT_TSHLD_MASK	0x00f0
#define TEMP_TEMP_VALUE_MASK	0x000f

#define MMP3_THERMAL_POLLING_PERIOD_MS 1000

#define LORANGE 0
#define HIRANGE 1

static int gray_hirange[] = {
     780,  805,  855,  830,  955,  930,  880,  905,
       0, 1130, 1080, 1105,  980, 1005, 1055, 1030,
};

static int gray_lorange[] = {
    260, 285, 335, 310, 435, 410, 360, 385,
    0,	 610, 560, 585, 460, 485, 535, 510,
};

static unsigned long gray_to_temp(int range, int gray)
{
	gray &= 0xf;
	if (range == HIRANGE)
		return gray_hirange[gray];
	else
		return gray_lorange[gray];
}

static unsigned int
mmp3_thermal_get_bits(int index, unsigned int bits)
{
	return readl(THERM_BASE(index * 4)) & bits;
}

static void
mmp3_thermal_set_bits(int index, unsigned int bits)
{
	__raw_writel(readl(THERM_BASE(index * 4)) | bits, THERM_BASE(index * 4));
}

static void
mmp3_thermal_clear_bits(int index, unsigned int bits)
{
	__raw_writel(readl(THERM_BASE(index * 4)) & ~bits, THERM_BASE(index * 4));
}

// return the median of an array of values.  this will both
// reject outliers and, if there are only two values (likely),
// choose the one in the majority.
#define MEDIANCOUNT 3  // must be odd
unsigned long medianfilter(unsigned long *vals)
{
#if MEDIANCOUNT > 3
	// note:  the array is sorted in place.
	int i, j
	unsigned long t;

	for (i = 0; i < MEDIANCOUNT - 1; i++) {
		for (j = i + 1; j < MEDIANCOUNT; j++) {
			if (vals[i] > vals[j]) {
				t = vals[i];
				vals[i] = vals[j];
				vals[j] = t;
			}
		}
	}
	return vals[MEDIANCOUNT / 2];
#else
	unsigned long a = vals[0];
	unsigned long b = vals[1];
	unsigned long c = vals[2];

	// sort the values 
	if (a > b) swap(a, b);
	if (b > c) swap(b, c);
	if (a > b) swap(a, b);

	// return the middle value 
	return b;

#endif
}

DEFINE_MUTEX(mmp3_thermal_sensor_mutex);

static unsigned long read_temperature_sensor(int range, int index)
{
	int i, j, gray;
	unsigned long temp;

	mutex_lock(&mmp3_thermal_sensor_mutex);

	mmp3_thermal_clear_bits(index, TEMP_START);
	if (range == HIRANGE)
		mmp3_thermal_clear_bits(index, TEMP_LOWRANGE);
	else
		mmp3_thermal_set_bits(index, TEMP_LOWRANGE);

	// delay between setting range and starting read
	usleep_range(300, 1000);

	for (i = 0; i < 10; i++) {
		mmp3_thermal_set_bits(index, TEMP_START);
		for (j = 0; j < 10; j++) {
			usleep_range(300, 1000);
			if (mmp3_thermal_get_bits(index, TEMP_STATUS))
				goto done;
		}
		usleep_range(300, 1000);
		mmp3_thermal_clear_bits(index, TEMP_START);
	}
    done:

	if (i == 10) {
		printk(KERN_ERR "mmp3_thermal: timeout reading sensor %d, range %d\n",
			index, range);
		mmp3_thermal_clear_bits(index, TEMP_START);
		temp = 0;  // filtering may fix this
	} else {
		gray = readl(THERM_BASE(index * 4)) & TEMP_TEMP_VALUE_MASK;
		temp = gray_to_temp(range, gray);
	}

	mutex_unlock(&mmp3_thermal_sensor_mutex);

	return temp;	// return value is tenths, C
}


static unsigned long read_filtered_temp(int range, int sensor)
{
	int i;
	unsigned long tvals[MEDIANCOUNT];

	for (i = 0; i < MEDIANCOUNT; i++)
		tvals[i] = read_temperature_sensor(range, sensor);

	if (debug & 1) {
		printk("mmp3_thermal: s %d range %d, ", sensor, range);
		for (i = 0; i < MEDIANCOUNT; i++)
			printk("%3ld ", tvals[i]);
		printk("\n");
	}
	return medianfilter(tvals);
}

static int th_sys_get_temp(struct thermal_zone_device *thermal, int *temp)
{
	unsigned long t, t0, t1, t2;

	t0 = read_filtered_temp(HIRANGE, 0);
	t1 = read_filtered_temp(HIRANGE, 1);
	t2 = read_filtered_temp(HIRANGE, 2);

	t = max(t0,t1);
	t = max(t,t2);

	if (t < 800) {
		t0 = read_filtered_temp(LORANGE, 0);
		t2 = read_filtered_temp(LORANGE, 2);
		t = max(t0,t2);

		// when sensor 0 is doing watchdog duty, don't change ranges
		if (!enable_watchdog) {
			t1 = read_filtered_temp(LORANGE, 1);
			t = max(t,t1);
		}

		// the sensors can't report values between 60 and 80
		if (t > 600)
			t = 700;
	}

	if (debug & 2) {
		printk("mmp3_thermal: %ld %ld %ld\n", t0, t1, t2);
	}

	*temp = t * 100; // return value is in millidegrees C

	return 0;
}


#define MMP3_THERMAL_NUM_TRIPS 0

static struct thermal_zone_device_ops g_dev_ops = {
	.get_temp = th_sys_get_temp,
};


void
mmp3_thermal_modify_watchdog(void)
{
	mmp3_thermal_clear_bits(1, TEMP_EN_WDOG);
	if (!enable_watchdog || gray_hirange[enable_watchdog & 0xf] == 0)
		return;

	enable_watchdog &= 0xf;

	// watchdog will always be in the high range

	printk(KERN_INFO "mmp3_thermal: setting thermal watchdog to %d.%dC\n",
		gray_hirange[enable_watchdog] / 10,
		gray_hirange[enable_watchdog] % 10);

	mmp3_thermal_clear_bits(1, TEMP_LOWRANGE|(0xf << TEMP_WDOG_SHIFT));
	mmp3_thermal_set_bits(1, enable_watchdog << TEMP_WDOG_SHIFT);
	mmp3_thermal_set_bits(1, TEMP_AUTO_READ);
	mmp3_thermal_set_bits(1, TEMP_EN_WDOG);
}

static int mmp3_thermal_probe(struct platform_device *pdev)
{
	int ret = 0, i;
	struct clk *tclk1, *tclk2, *tclk3, *tclk4;
	struct device_node *np;
	struct resource r;

	np = pdev->dev.of_node;
	if (np)
		return -ENODEV;


	ret = of_address_to_resource(np, 0, &r);
	if (ret != 0) {
		printk(KERN_ERR "mmp3_thermal: Could not get i/o memory\n");
		return ret;
	}
	therm_base = ioremap(r.start, r.end - r.start + 1);
	if (therm_base == NULL) {
		return -ENOMEM;
	}
	tclk1 = clk_get(&pdev->dev, "THSENS1");
	tclk2 = clk_get(&pdev->dev, "THSENS2");
	tclk3 = clk_get(&pdev->dev, "THSENS3");
	tclk4 = clk_get(&pdev->dev, "THSENS4");
	if (IS_ERR(tclk1) || IS_ERR(tclk2) || IS_ERR(tclk3) || IS_ERR(tclk4)) {
		printk(KERN_ERR "mmp3_thermal: Could not get all thermal clocks\n");
		return PTR_ERR(tclk1);
	}
	clk_enable(tclk1);
	clk_enable(tclk2);
	clk_enable(tclk3);
	clk_enable(tclk4);

	for (i = 0; i < 3; i++) {
		mmp3_thermal_clear_bits(i, TEMP_AUTO_READ|TEMP_EN_WDOG|
			TEMP_EN_OVER_INT|TEMP_EN_TEMP_INT);
	}

	mmp3_thermal_modify_watchdog();

	therm_dev = thermal_zone_device_register(
			"mmp3-temp_sens", MMP3_THERMAL_NUM_TRIPS,
			0, &g_dev_ops, 0, 0,
			MMP3_THERMAL_POLLING_PERIOD_MS, 0);

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

static const struct of_device_id mmp3_thermal_driver_dt_ids[] = {
	{ .compatible = "marvell,mmp3-thermal", },
	{}
};
MODULE_DEVICE_TABLE(of, mmp3_thermal_driver_dt_ids);

static struct platform_driver mmp3_thermal_driver = {
	.driver = {
		.name	= "mmp-thermal",
		.owner	= THIS_MODULE,
		.of_match_table = mmp3_thermal_driver_dt_ids,
	},
	.probe		= mmp3_thermal_probe,
	.remove		= mmp3_thermal_remove,
};

module_platform_driver(mmp3_thermal_driver);

MODULE_AUTHOR("Marvell Semiconductor");
MODULE_DESCRIPTION("MMP3 SoC thermal driver");
MODULE_LICENSE("GPL");
