/*
 * SIM control support
 *
 * Copyright (C) 2011 Marvell Internation Ltd.
 *
 * Michael Zaidman <zmichael@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG	1
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/namei.h>
#include <linux/path.h>
#include <linux/fs.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <mach/gpio.h>
#include <mach/mfp.h>
#include <plat/pxa9xx_sim.h>

#ifdef	DEBUG
#define dbg(dev, fmt, args...)	dev_info(dev, fmt , ##args)
#define dbg_mfp_print(pdev, bgn, end, prefix) \
				sim_mfp_print(pdev, bgn, end, prefix)
#define dbg_gpio_print(pdev, bgn, end) sim_gpio_print(pdev, bgn, end)
#define sim_connect(x, y)	ioctl_rpc_sim_commutate(RPC_SIM_CONN_CMD, y)
#define sim_disconnect(x, y)	ioctl_rpc_sim_commutate(RPC_SIM_DISC_CMD, y)
#else
#define dbg(dev, fmt, args...)
#define dbg_mfp_print(pdev, bgn, end, prefix)
#define dbg_gpio_print(pdev, bgn, end)
#define sim_connect(x, y)	ioctl_sim_connect(x, y)
#define sim_disconnect(x, y)	ioctl_sim_disconnect(x, y)
#endif

#define REG_NUM(i)		(pdata->sim_mfp_cfg[i].reg_num)
#define OBM_VAL(i)		(pdata->sim_mfp_cfg[i].obm_val)
#define NEW_VAL(i)		(pdata->sim_mfp_cfg[i].new_val)

#define FUN			__func__

#define MAX_MINORS		1
#define VERSION			"1.0.1.1"

#define GSIM_PINS		3
#ifdef CONFIG_PXA9XX_SIM_DUAL_FIXED
#define USIM_MAX		2
#else
#define USIM_MAX		1
#endif
#define MFPR_MAX_NUM		(GSIM_PINS * USIM_MAX)
#define DSLP			0x100	/* DATA_SLEEP bit in MFPR */

enum uevent_update_reason {
	UEVENT_SIM1_INSERTED,
	UEVENT_SIM1_REMOVED,
};

/* Format for RPC SIM connect/disconnect commands */
enum {
	RPC_GSIM_CMD,
	RPC_GSIM_NUM,		/* 0 - GSIM1, 1 - GSIM2 */
	RPC_USIM_NUM,		/* 0 - SIM_1, 1 - SIM_2 */
	RPC_BUF_LEN,
};

/* Possible values for RPC_GSIM_CMD byte */
#define RPC_SIM_CONN_CMD	0xA9	/* RPC SIM connect command */
#define RPC_SIM_DISC_CMD	0x56	/* RPC SIM disconnect command */

struct rpc_statistics {
	u16 commutate_done;
	u16 commutate_errs;
};

struct sim_device {
	struct cdev cdev;
	struct device *dev;
	struct class *class;
	spinlock_t sim_lock;
	struct mutex update_lock;
	struct wake_lock sim_wake_lock;
	struct sim_platform_data *platform_data;
	struct {
		char clk_d2_cfg_done;
	} sim[2];
	unsigned sim_init_done:1;
#ifdef CONFIG_PXA9XX_SIM_DUAL_FIXED
	unsigned sim1_detected:1;
	unsigned sim_selected:1;
#endif
};

static int major;
static struct rpc_statistics rpc;

#ifdef DEBUG
static void sim_mfp_print(struct sim_device *pdev,
		int bgn, int end, char *prefix)
{
	struct sim_platform_data *pdata = pdev->platform_data;
	int i, tmp;

	for (i = bgn; i < end; i++) {
		tmp = mfp_read(REG_NUM(i));
		dev_info(pdev->dev,
			"%s: %s%d: obm_val = 0x%x, mfp_read(0x%x) = 0x%x\n",
			FUN, prefix, i, OBM_VAL(i), REG_NUM(i), tmp);
	}
}

static void sim_gpio_print (struct sim_device *pdev, int bgn, int end)
{
	struct sim_platform_data *pdata = pdev->platform_data;
	int i, gpio;

	for (i = bgn; i < end; i++) {
		gpio = mfp_to_gpio(REG_NUM(i));
		dev_info(pdev->dev, "%s: %d: gpio-get(0x%x) = 0x%x\n",
				FUN, i, gpio, __gpio_get_value(gpio));
	}
}
#endif

static void sim_gpio_set(struct sim_device *pdev,
		int bgn, int end, int level)
{
	struct sim_platform_data *pdata = pdev->platform_data;
	int i, reg_num;

	for (i = bgn; i < end; i++) {
		reg_num = REG_NUM(i);
		gpio_direction_output(mfp_to_gpio(reg_num), level);
	}
}

static int set_clk_data_sleep_level(struct sim_device *pdev,
		unsigned int cmd, union sim_ioctl_args *args)
{
	char name[6];
	int i, reg_num, res = 0;
	int bgn = args->a.sim_num * GSIM_PINS;
	int end = bgn + GSIM_PINS;
	u32 tmp;

	struct sim_platform_data *pdata = pdev->platform_data;

	mutex_lock(&pdev->update_lock);

	for (i = bgn; i < end; i++) {
		reg_num = REG_NUM(i);
		sprintf(name, "%x ", reg_num);
		res = gpio_request(mfp_to_gpio(reg_num), name);
		if (res) {
			mutex_unlock(&pdev->update_lock);
			dev_err(pdev->dev,
				"%s: gpio_request %s failed!\n", FUN, name);

			while (i > bgn)
				gpio_free(mfp_to_gpio(REG_NUM(--i)));

			return res;
		}
	}

	if (args->a.data_sleep_level == LPM_UCLK_HI) {
		sim_gpio_set(pdev, bgn, end, 1);
	} else {
		sim_gpio_set(pdev, bgn, bgn+1, 0);
		sim_gpio_set(pdev, bgn+1, end, 1);
	}

	for (i = bgn; i < end; i++)
		gpio_free(mfp_to_gpio(REG_NUM(i)));

	pdev->sim[args->a.sim_num].clk_d2_cfg_done = 1;

	mutex_unlock(&pdev->update_lock);

	if (args->a.data_sleep_level == LPM_UCLK_HI) {
		/* Set GSIM_UCLK to output high level while
		 * in low power mode.
		 */
		reg_num = REG_NUM(bgn);
		spin_lock_irq(&pdev->sim_lock);
		OBM_VAL(bgn) |= DSLP;
		NEW_VAL(bgn) |= DSLP;

		tmp = mfp_read(reg_num);
		tmp |= DSLP;
		mfp_write(reg_num, tmp);
		spin_unlock_irq(&pdev->sim_lock);
	} else if (args->a.data_sleep_level == LPM_UCLK_LO) {
		/* Set GSIM_UCLK to output low level while
		 * in low power mode.
		 */
		reg_num = REG_NUM(bgn);
		spin_lock_irq(&pdev->sim_lock);
		OBM_VAL(bgn) &= ~DSLP;
		NEW_VAL(bgn) &= ~DSLP;
		tmp = mfp_read(reg_num);
		tmp &= ~DSLP;
		mfp_write(reg_num, tmp);
		spin_unlock_irq(&pdev->sim_lock);
	} else {
		dev_err(pdev->dev, "%s: invalid level value\n", FUN);
		res = -EINVAL;
	}

	dbg_gpio_print(pdev, 0, GSIM_PINS * USIM_MAX);

	return res;
}

static int get_clk_data_sleep_level(struct sim_device *pdev, unsigned int cmd,
		union sim_ioctl_args *args, unsigned long arg)
{
	struct sim_platform_data *pdata = pdev->platform_data;
	int reg_num = REG_NUM(args->a.sim_num * GSIM_PINS);

	args->a.data_sleep_level =
			(mfp_read(reg_num) & DSLP) ? LPM_UCLK_HI : LPM_UCLK_LO;

	dbg(pdev->dev, "%s: mfp_read(0x%x) = 0x%x\n",
			FUN, reg_num, (unsigned int)mfp_read(reg_num));

	if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd)) ||
			__put_user(args->ul32, (unsigned long __user *)arg))
		return -EFAULT;

	return 0;
}

static int sim_num_check(struct sim_device *pdev, int sim_num)
{
	if ((sim_num == SIM_1) ||
		((sim_num == SIM_2) && (USIM_MAX == 2))) {

		return 0;
	}
	dev_err(pdev->dev, "%s: invalid SIM number %d\n", FUN, sim_num);

	return -EINVAL;
}

static int rpc_get_pdev(struct sim_device **pdev)
{
	struct path path;
	struct inode *inode;

	int res = kern_path("/dev/"SIM_DEVNAME, 0, &path);
	if (res) {
		pr_err("%s:%s: can not resolve dev path\n", SIM_DEVNAME, FUN);
		return res;
	}
	inode = path.dentry->d_inode;

	if (inode->i_cdev) {
		*pdev = container_of(inode->i_cdev, struct sim_device, cdev);
		return 0;
	}
	pr_err("%s:%s: RPC call to uninitialized cdev!\n", SIM_DEVNAME, FUN);
	return -1;
}

static int ioctl_sim_connect(struct sim_device *pdev, int sim_num)
{
	unsigned long flags;
	int sim_pins, i;
	struct sim_platform_data *pdata = pdev->platform_data;

	dbg(pdev->dev, "%s\n", FUN);

	if (!pdev->sim[sim_num].clk_d2_cfg_done) {
		dev_err(pdev->dev, "%s: no gpio configured!\n", FUN);
		return -EPERM;
	}
	sim_pins = MFPR_SIM1 + (sim_num * GSIM_PINS);

	spin_lock_irqsave(&pdev->sim_lock, flags);
	for (i = sim_pins; i < (sim_pins + GSIM_PINS); i++) {
		mfp_write(REG_NUM(i), OBM_VAL(i));
		mfp_read(REG_NUM(i));
	}
	spin_unlock_irqrestore(&pdev->sim_lock, flags);

	dbg_mfp_print(pdev, sim_pins, sim_pins + GSIM_PINS, "CONNECT-");

	return 0;
}

static int ioctl_sim_disconnect(struct sim_device *pdev, int sim_num)
{
	unsigned long flags;
	int sim_pins, i;
	struct sim_platform_data *pdata = pdev->platform_data;

	dbg(pdev->dev, "%s\n", FUN);

	if (!pdev->sim[sim_num].clk_d2_cfg_done) {
		dev_err(pdev->dev, "%s: no gpio configured!\n", FUN);
		return -EPERM;
	}
	sim_pins = i = MFPR_SIM1 + (sim_num * GSIM_PINS);

	spin_lock_irqsave(&pdev->sim_lock, flags);
	/* CLK */
	mfp_write(REG_NUM(i), ((OBM_VAL(i) & DSLP) ? 0xc940 : 0xa840));
	mfp_read(REG_NUM(i++));
	/* DATA */
	mfp_write(REG_NUM(i), 0xc940);
	mfp_read(REG_NUM(i++));
	/* RST */
	mfp_write(REG_NUM(i), 0xc940);
	mfp_read(REG_NUM(i));
	spin_unlock_irqrestore(&pdev->sim_lock, flags);

	dbg_mfp_print(pdev, sim_pins, sim_pins + GSIM_PINS, "DISCONN-");

	return 0;
}

unsigned long rpc_sim_commutate(
		unsigned short in_len,  void *in_buf,
		unsigned short out_len, void *out_buf)
{
	int res;
	char *param = in_buf;
	char *pret  = out_buf;
	struct sim_device *pdev;
	int(*commutate)(struct sim_device *, int);

	*pret = -1;

	res = rpc_get_pdev(&pdev);
	if (res)
		goto rpc_err;

	if (in_len != RPC_BUF_LEN) {
		dev_err(pdev->dev, "%s: invalid in_len!\n", FUN);
		res = -EINVAL;
		goto rpc_err;
	}

	if (param[RPC_GSIM_CMD] == RPC_SIM_CONN_CMD) {
		commutate = ioctl_sim_connect;
	} else if (param[RPC_GSIM_CMD] == RPC_SIM_DISC_CMD) {
		commutate = ioctl_sim_disconnect;
	} else {
		dev_err(pdev->dev, "%s: invalid command!\n", FUN);
		res = -EINVAL;
		goto rpc_err;
	}

	res = sim_num_check(pdev, param[RPC_USIM_NUM]);
	if (res)
		goto rpc_err;

	if (!pdev->sim_init_done) {
		dev_err(pdev->dev, "%s: no sim_init!\n", FUN);
		res = -EPERM;
		goto rpc_err;
	}

	*pret = 0;

	res = commutate(pdev, param[RPC_USIM_NUM]);
	if (res)
		goto rpc_err;

	rpc.commutate_done++;
	dev_info(pdev->dev, "%s done=%d, errs=%d\n",
		 FUN, rpc.commutate_done, rpc.commutate_errs);

	return 0;

rpc_err:
	rpc.commutate_errs++;
	/* Don't use dev_err since pdev->dev can be uninitialized yet */
	pr_err(SIM_DEVNAME ": %s done=%d, errs=%d\n",
	       FUN, rpc.commutate_done, rpc.commutate_errs);
	return res;
}
EXPORT_SYMBOL(rpc_sim_commutate);

#ifdef DEBUG
static int ioctl_rpc_sim_commutate(int cmd, int sim_num)
{
	char in_buf[RPC_BUF_LEN];
	char out_buf[sizeof(char)];

	in_buf[RPC_GSIM_CMD] = cmd;
	in_buf[RPC_GSIM_NUM] = 0;
	in_buf[RPC_USIM_NUM] = sim_num;

	return rpc_sim_commutate(RPC_BUF_LEN, in_buf, sizeof(char), out_buf);
}
#endif

#ifdef CONFIG_PXA9XX_SIM_DUAL_FIXED
static int swap_dual_sim(struct sim_device *pdev, int sim_num)
{
	int from, to, i;
	struct sim_platform_data *pdata = pdev->platform_data;

	dbg(pdev->dev, "%s:\n", FUN);

	if (sim_num == SIM_1) {
		from = MFPR_SIM2;
		to = MFPR_SIM1;
	} else {
		from = MFPR_SIM1;
		to = MFPR_SIM2;
	}

	dbg(pdev->dev, "%s: switch to SIM_%d, (to = 0x%x, from = 0x%x)\n",
			FUN, sim_num + 1, to, from);

	dbg_mfp_print(pdev, MFPR_SIM1, MFPR_SIM2 + GSIM_PINS, "SIM_SEL_OLD-");

	spin_lock_irq(&pdev->sim_lock);

	for (i = from; i < (from + GSIM_PINS); i++) {
		/*
		 * When pad is not used as SIM function
		 * the register data should be set to 0xa040
		 */
		mfp_write(REG_NUM(i), 0xa040);
		/*
		 * From the MG2 specification:
		 * A significant amount of operations can be queued
		 * between software and the actual write of the
		 * multi-function register. It is important that
		 * the software be able to find out when the last
		 * operation has completed and taken effect. This
		 * is done by performing a read operation to the
		 * address of the last operation that was performed.
		 * No further operation can be performed to the PCU
		 * until this read response has returned, which
		 * might take a significant amount of time (ns).
		 */
		mfp_read(REG_NUM(i));
	}

	for (i = to; i < (to + GSIM_PINS); i++) {
		mfp_write(REG_NUM(i), NEW_VAL(i));
		mfp_read(REG_NUM(i));
	}
	pdev->sim_selected = sim_num;

	spin_unlock_irq(&pdev->sim_lock);

	dbg_mfp_print(pdev, MFPR_SIM1, MFPR_SIM2 + GSIM_PINS, "SIM_SEL_NEW-");

	return 0;
}

static int get_sim1_card_detection_status(
		struct sim_device *pdev, unsigned int cmd,
		union sim_ioctl_args *args, unsigned long arg)
{
	args->a.sim1_status = pdev->sim1_detected;
	args->a.sim_num = pdev->sim_selected;

	dbg(pdev->dev, "%s: SIM1 is %sdetected\n", FUN,
			(pdev->sim1_detected ? "" : "un"));

	if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd)) ||
			__put_user(args->ul32, (unsigned long __user *)arg))
		return -EFAULT;

	return 0;
}
#endif

static int sim_open(struct inode *inode, struct file *filp)
{
	struct sim_device *pdev;
	int res = 0;

	pdev = container_of(inode->i_cdev, struct sim_device, cdev);

	dbg(pdev->dev, "%s\n", FUN);
	res = nonseekable_open(inode, filp);
	if (res)
		return res;

	filp->private_data = pdev;

	return 0;
}

static long sim_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int res = 0;
	union sim_ioctl_args args;
	struct sim_device *pdev = filp->private_data;

	if (_IOC_TYPE(cmd) != SIM_IOC_MAGIC)
		return -EINVAL;

	if (_IOC_NR(cmd) > SIM_IOC_MAXNR)
		return -EINVAL;

	if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd)) ||
			__get_user(args.ul32, (unsigned long __user *)arg))
		return -EFAULT;

	dbg(pdev->dev, "%s: cmd = 0x%x, sim_num = %d, data_sleep = %d\n",
			FUN, cmd, args.a.sim_num, args.a.data_sleep_level);

	res = sim_num_check(pdev, args.a.sim_num);
	if (res)
		return res;

	switch (cmd) {

	case SIM_D2_LVL_SET:
		res = set_clk_data_sleep_level(pdev, cmd, &args);
		break;

	case SIM_D2_LVL_GET:
		res = get_clk_data_sleep_level(pdev, cmd, &args, arg);
		break;

	case SIM_DISCONNECT:
		res = sim_disconnect(pdev, args.a.sim_num);
		break;

	case SIM_CONNECT:
		res = sim_connect(pdev, args.a.sim_num);
		break;

#ifdef CONFIG_PXA9XX_SIM_DUAL_FIXED
	case SIM_SELECT:
		res = swap_dual_sim(pdev, args.a.sim_num);
		break;

	case SIM1_STATUS_GET:
		res = get_sim1_card_detection_status(pdev, cmd, &args, arg);
		break;
#endif
	default:
		dev_err(pdev->dev, "%s: illegal command!\n", FUN);
		res = -1;
	}

	return res;
}

static const struct file_operations sim_fops = {
	.open  = sim_open,
	.unlocked_ioctl = sim_ioctl,
	.owner = THIS_MODULE,
};

#ifdef CONFIG_PXA9XX_SIM_DUAL_FIXED
static int ext_sim_check(int gpio, int invert)
{
	int status;

	status = gpio_get_value(gpio);
	status &= (1 << (gpio%32));
	status = !!status;

	if (invert)
		status = !status;

	return status;
}


static void sim_generate_event(struct platform_device *pdev,
		enum uevent_update_reason reason)
{
	char *envp[2];

	switch (reason) {
	case UEVENT_SIM1_INSERTED:
		envp[0] = SIM1_INSERT_UEVENT_STR;
		break;
	case UEVENT_SIM1_REMOVED:
		envp[0] = SIM1_REMOVE_UEVENT_STR;
		break;
	default:
		envp[0] = "ERROR";
		break;
	}
	envp[1] = NULL;
	dbg(&pdev->dev, "SIM1 uevent reason: %s\n", envp[0]);
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
}

static void sim1_detect_change_notify(struct platform_device *pdev)
{
	int status;
	struct sim_device *sim_dev = platform_get_drvdata(pdev);

	if (sim_dev->sim1_detected)
		status = UEVENT_SIM1_INSERTED;
	else
		status = UEVENT_SIM1_REMOVED;

	sim_generate_event(pdev, status);
}

static irqreturn_t sim1_detect_irq_thread(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct sim_device *sim_dev = platform_get_drvdata(pdev);
	struct sim_platform_data *pdata = pdev->dev.platform_data;
	int status;

	msleep(500);

	status = ext_sim_check(pdata->sim1_cd_gpio, pdata->sim1_cd_invert);
	if (status != sim_dev->sim1_detected) {
		sim_dev->sim1_detected = status;
		wake_lock_timeout(&sim_dev->sim_wake_lock, (10*HZ));
		sim1_detect_change_notify(pdev);
	}

	return IRQ_HANDLED;
}

static int __devinit sim1_detect_init(struct platform_device *pdev)
{
	int res, sim1_detect_irq;
	struct sim_platform_data *pdata = pdev->dev.platform_data;
	struct sim_device *sim_dev = platform_get_drvdata(pdev);

	res = gpio_is_valid(pdata->sim1_cd_gpio);
	if (!res) {
		dev_err(&pdev->dev, "%s: gpio_is not valid!\n", FUN);
		return res;
	}
	res = gpio_request(pdata->sim1_cd_gpio, "SIM1 detect gpio");
	if (res) {
		dev_err(&pdev->dev, "%s: gpio_request failed!\n", FUN);
		return res;
	}
	gpio_direction_input(pdata->sim1_cd_gpio);

	sim1_detect_irq = gpio_to_irq(pdata->sim1_cd_gpio);
	res = request_threaded_irq(
			sim1_detect_irq, NULL, sim1_detect_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			SIM_DRVNAME, pdev);
	if (res) {
		gpio_free(pdata->sim1_cd_gpio);
		dev_err(&pdev->dev, "%s: irq_request failed!\n", FUN);
		return res;
	}

	wake_lock_init(&sim_dev->sim_wake_lock,
			WAKE_LOCK_SUSPEND, "sim_card_detect");

	res = ext_sim_check(pdata->sim1_cd_gpio, pdata->sim1_cd_invert);
	sim_dev->sim1_detected = res;

	res = swap_dual_sim(sim_dev, (sim_dev->sim1_detected ? SIM_1 : SIM_2));

	return res;
}

static void __devexit sim1_remove(struct platform_device *pdev)
{
	struct sim_device *sim_dev = platform_get_drvdata(pdev);
	struct sim_platform_data *pdata = sim_dev->platform_data;

	free_irq(gpio_to_irq(pdata->sim1_cd_gpio), pdev);
	gpio_free(pdata->sim1_cd_gpio);
	wake_lock_destroy(&sim_dev->sim_wake_lock);
}
#endif

static void sim_mfp_init(struct sim_device *pdev)
{
	int i;
	struct sim_platform_data *pdata = pdev->platform_data;

	dbg(pdev->dev, "%s:\n", FUN);

	/* Upon startup read and save configured by OBM mfpr values */
	for (i = 0; i < MFPR_MAX_NUM; i++) {
		OBM_VAL(i) = (ushort)mfp_read(REG_NUM(i));

		dbg(pdev->dev, "%s: mfp_read(0x%x) = 0x%x\n",
				FUN, REG_NUM(i), OBM_VAL(i));
	}

	pdev->sim_init_done = 1;

	return;
}

static int __devinit sim_probe(struct platform_device *pdev)
{
	int res;
	dev_t dev_id;
	struct sim_device *sim_dev;
	struct sim_platform_data *pdata = pdev->dev.platform_data;

	dev_info(&pdev->dev, "Ver. %s:\n", VERSION);

	if (pdata == NULL) {
		res = -EFAULT;
		dev_err(&pdev->dev, "%s: no platform data!\n", FUN);
		goto out;
	}

	sim_dev = kzalloc(sizeof(*sim_dev), GFP_KERNEL);
	if (sim_dev == NULL) {
		res = -ENOMEM;
		dev_err(&pdev->dev, "%s: unable to allocate sim_dev!\n", FUN);
		goto out;
	}

	sim_dev->platform_data = pdata;

	spin_lock_init(&sim_dev->sim_lock);
	mutex_init(&sim_dev->update_lock);

	if (major) {
		dev_id = MKDEV(major, 0);
		res = register_chrdev_region(dev_id, MAX_MINORS, SIM_DEVNAME);
	} else {
		res = alloc_chrdev_region(&dev_id, 0, MAX_MINORS, SIM_DEVNAME);
		major = MAJOR(dev_id);
	}
	if (res) {
		dev_err(&pdev->dev, "unable to get major %d\n", major);
		goto out_kzalloc;
	}

	major = MAJOR(dev_id);

	cdev_init(&sim_dev->cdev, &sim_fops);
	res = cdev_add(&sim_dev->cdev, MKDEV(major, 0), MAX_MINORS);
	if (res) {
		dev_err(&pdev->dev, "cdev_add returned %d\n", res);
		goto out_chrdev_region;
	}

	sim_dev->class = class_create(THIS_MODULE, SIM_DEVNAME);
	if (IS_ERR(sim_dev->class)) {
		res = PTR_ERR(sim_dev->class);
		goto out_cdev_add;
	}

	sim_dev->dev = device_create(sim_dev->class, &pdev->dev,
			MKDEV(major, 0), sim_dev, SIM_DEVNAME);
	if (IS_ERR(sim_dev->dev)) {
		res = PTR_ERR(sim_dev->dev);
		goto out_class_create;
	}

	platform_set_drvdata(pdev, sim_dev);

	sim_mfp_init(sim_dev);

#ifdef CONFIG_PXA9XX_SIM_DUAL_FIXED
	res = sim1_detect_init(pdev);
	if (res) {
		dev_err(&pdev->dev,
			"%s: sim1_detect_init failed!\n", FUN);
		goto out_device_create;
	}

	return 0;

out_device_create:
	device_destroy(sim_dev->class, MKDEV(major, 0));
#else
	return 0;
#endif

out_class_create:
	class_destroy(sim_dev->class);

out_cdev_add:
	cdev_del(&sim_dev->cdev);

out_chrdev_region:
	unregister_chrdev_region(MKDEV(major, 0), MAX_MINORS);

out_kzalloc:
	kfree(sim_dev);

out:
	return res;
}

static int __devexit sim_remove(struct platform_device *pdev)
{
	struct sim_device *sim_dev = platform_get_drvdata(pdev);

	if (sim_dev) {
#ifdef CONFIG_PXA9XX_SIM_DUAL_FIXED
		sim1_remove(pdev);
#endif
		device_destroy(sim_dev->class, MKDEV(major, 0));
		class_destroy(sim_dev->class);
		cdev_del(&sim_dev->cdev);
		unregister_chrdev_region(MKDEV(major, 0), MAX_MINORS);
		kfree(sim_dev);
	}

	return 0;
}

static struct platform_driver sim_driver = {
	.probe		= sim_probe,
	.remove		= sim_remove,
	.driver		= {
		.name	= SIM_DRVNAME,
	},
};

static int __init sim_init(void)
{
	return platform_driver_register(&sim_driver);
}

static void __exit sim_exit(void)
{
	platform_driver_unregister(&sim_driver);
}

module_init(sim_init);
module_exit(sim_exit);

MODULE_AUTHOR("Michael Zaidman <zmichael@marvell.com>");
MODULE_DESCRIPTION("PXA SIM Control");
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:"SIM_DRVNAME);
