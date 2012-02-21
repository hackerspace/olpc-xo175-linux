/*
 *  Do flush operation when panic to save those stuff still in cache to mem
 *
 *  Copyright (C) 2012 Marvell International Ltd.
 *  Lei Wen <leiwen@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/regdump_ops.h>
#include <linux/notifier.h>
#include <linux/kexec.h>
#include <linux/kdebug.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/setup.h>

static int has_died;
static int crash_enable;
static void *indicator;
static int
panic_flush(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct membank *bank;
	int i;
	if (!crash_enable)
		return NOTIFY_DONE;

	memset(indicator, 0 ,PAGE_SIZE);
	*(unsigned long *)indicator = 0x454d4d44;

	dump_reg_to_mem();
	flush_cache_all();
	for (i = 0; i < meminfo.nr_banks; i ++) {
		bank = &meminfo.bank[i];
		if (bank->size)
			outer_flush_range(bank->start, bank->size);
	}

	if (!has_died)
		crash_update(NULL);
	return NOTIFY_DONE;
}

static struct notifier_block panic_flush_block = {
	.notifier_call = panic_flush,
};

static int dump_reg_handler(struct notifier_block *self,
			     unsigned long val,
			     void *data)
{
	struct die_args *args = data;

	if (!crash_enable)
		return 0;

	crash_update(args->regs);
	has_died = 1;
	return 0;
}

static struct notifier_block die_dump_reg_notifier = {
	.notifier_call = dump_reg_handler,
	.priority = 200
};

static ssize_t panic_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	int len = 0;
	len = sprintf(buf, "Current crash enable is: %x\n", crash_enable);

	return len;
}

static ssize_t panic_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t len)
{
	char _buf[80];

	if (strncmp(buf, "PID", 3) == 0) {
		snprintf(_buf, 80, "\nUser Space Panic:%s", buf);
		panic(_buf);
		goto OUT;
	}
	if (strncmp(buf, "CRASH_ENABLE", 12) == 0) {
		crash_enable =  1;
		goto OUT;
	}

	printk(KERN_WARNING "Not valid value!!!\n");
OUT:
	return len;
}

static struct kobj_attribute panic_attr = {
	.attr	= {
		.name = __stringify(panic),
		.mode = 0644,
	},
	.show	= panic_show,
	.store	= panic_store,
};

static struct attribute * g[] = {
	&panic_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int __init pxa_panic_notifier(void)
{
	struct page *page;
	crash_enable = 0;
	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;

	page = pfn_to_page(crashk_res.end >> PAGE_SHIFT);
	indicator = page_address(page);
	register_die_notifier(&die_dump_reg_notifier);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_flush_block);
	has_died = 0;
	return 0;
}

late_initcall(pxa_panic_notifier);
