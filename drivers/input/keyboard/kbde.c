/*
 keyboard emulator project

 Copyright (C) 1998-2007 Reznic Valery <valery_reznic@users.sourceforge.net>

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

*/

/*
 * Modified to work on kernel 2.6 by Gunnar Teege
 * (gunnar.teege@unibw-muenchen.de)
 * Successfully tested on SuSE Linux 9.1, Kernel 2.6.5.
 */

#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/serio.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>

static int kbde_open(struct inode *inode, struct file *filp);
static void kbde_dev_release(struct device *dev);
static int kbde_probe(struct platform_device *dev);
static ssize_t kbde_write(struct file *file, const char *buf, size_t length,
			  loff_t *ppos);

/*
 * This is called by serio_open when connecting to the keyboard driver.
 * We need no additional actions here and return success.
 */
static int serio_kbde_open(struct serio *port)
{
	return 0;
}

/*
 * This is called by serio_close when the keyboard driver disconnects.
 * We need no actions here.
 */
static void serio_kbde_close(struct serio *port)
{
}

/* serio should be kmalloce'ed, or serio_unregister_port will segfault :( */
static struct serio *kbde_port;

struct kbdechar_dev {
	struct semaphore sem;	/* mutual exclusion semaphore */
	struct cdev cdev;	/* Char device structure */
};

static struct file_operations kbde_fops = {
	.open = kbde_open,
	.write = kbde_write,
	.owner = THIS_MODULE
};

static struct platform_device kbde_device = {
	.name = "marvell-kbde",
	.id = 0,
	.dev = {
		.release = kbde_dev_release,
		},
};

static struct platform_driver kbde_driver = {
	.probe = kbde_probe,
	.driver = {
		   .name = "marvell-kbde",
		   }
};

static struct miscdevice kbde_miscdev = {
	MISC_DYNAMIC_MINOR,
	"kbde",
	&kbde_fops,
};

static int kbde_open(struct inode *inode, struct file *filp)
{
	struct kbdechar_dev *dev;

	dev = container_of(inode->i_cdev, struct kbdechar_dev, cdev);

	filp->private_data = dev;

	return nonseekable_open(inode, filp);
}

static int kbde_probe(struct platform_device *dev)
{
	int err;

	err = misc_register(&kbde_miscdev);

	kbde_port = kmalloc(sizeof(struct serio), GFP_KERNEL);
	if (kbde_port == NULL)
		return -ENOMEM;

	memset(kbde_port, 0, sizeof(struct serio));
	kbde_port->open = serio_kbde_open;
	kbde_port->close = serio_kbde_close;
	strcpy(kbde_port->name, "Kbd Emulator Port");
	strcpy(kbde_port->phys, "Keyboard Emulator");
	kbde_port->id.type = SERIO_8042_XL;
	kbde_port->id.proto = SERIO_ANY;
	kbde_port->id.id = SERIO_ANY;
	kbde_port->id.extra = SERIO_ANY;

	/* register this driver as a serial io port */
	serio_register_port(kbde_port);

	return err;
}

static ssize_t kbde_write(struct file *file, const char *buf, size_t length,
			  loff_t *ppos)
{
	int retval = 0;
	unsigned char scancode = 0;

	retval = access_ok(VERIFY_READ, buf, length);
	if (retval == 0)
		return -EFAULT;

	for (retval = 0; retval < length; retval++) {
		get_user(scancode, (char *)buf++);
		serio_interrupt(kbde_port, scancode, 0);
	}

	return 0;
}

static void kbde_dev_release(struct device *dev)
{
	return;
}

static int __init kbde_init(void)
{
	int ret;

	pr_info("kbde module has been initialized\n");

	ret = platform_device_register(&kbde_device);
	if (!ret) {
		ret = platform_driver_register(&kbde_driver);
		if (ret)
			platform_device_unregister(&kbde_device);
	} else {
		pr_err("Cannot register KBDE platform device\n");
	}

	return ret;

};

static void __exit kbde_exit(void)
{
	platform_driver_unregister(&kbde_driver);
	platform_device_unregister(&kbde_device);

	serio_unregister_port(kbde_port);
}

module_init(kbde_init);
module_exit(kbde_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Shmuel Gandin shmuel.gandin@marvell.com");
MODULE_DESCRIPTION("Keyboard (i386) emulator");
