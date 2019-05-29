/*
 *  PM Logger
 *
 *  Support for Power management related event logger over PXAxxx
 *
 *  Author:	Shay Pathov, Moran Raviv
 *  Created:	Dec 15, 2010
 *  Copyright:	(C) Copyright 2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/pgtable.h>
#include <linux/semaphore.h>
#include <mach/pxa9xx_pm_logger.h>

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "MARVELL"
#define DRIVER_DESC "Power management event Logger"

/* module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/* global logger descriptor */
struct pm_logger_descriptor pm_logger_app;

/* database for parser print */
/* pay attention: when adding new string to the database, make sure to
 *  update MAX_DATA_NUM so we'll not exceed it */
u8 *pm_logger_app_db[][MAX_DATA_NUM] = {
	{"D1 ENTRY", "PWRMODE", "CKENA", "CKENB", "CKENC",
		"MIPI", "CPUPWR"},
	{"D2 ENTRY", "PWRMODE", "CKENA", "CKENB", "CKENC",
		"OSCC", "MIPI", "CPUPWR"},
	{"CGM ENTRY", "PWRMODE", "CKENA", "CKENB", "CKENC",
		"MIPI", "CPUPWR"},
	{"C1 ENTRY", "CPUPWR"},
	{"D1 EXIT", "AD1D0SR"},
	{"D2 EXIT", "AD2D0SR"},
	{"CGM EXIT", "ACGD0SR"},
	{"C1 EXIT"},
	{"OP REQ", "OP"},
	{"OP EN", "OP", "DEV"},
	{"OP DIS", "OP", "DEV"},
	{"OP EN NAME", "OP", "DEV"},
	{"OP DIS NAME", "OP", "DEV"},
	{"OP EN NO CHANGE", "OP", "DEV"},
	{"OP DIS NO CHANGE", "OP", "DEV"},
	{"OP SET", "FREQ", "ACCR", "ACSR"},
	{"C2 ENTRY"},
	{"C2 EXIT", "ICHP"},
	{"INFO"},
	{"WAKEUP GPIO", "GWSR1", "GWSR2", "GWSR3",
		"GWSR4", "GWSR5", "GWSR6"}
};

/* var to avoid a print because of previous print */
static unsigned int pm_logger_print = 1;

static void pm_logger_wq_fnc(struct work_struct *work);

static DECLARE_DELAYED_WORK(pm_logger_wq, pm_logger_wq_fnc);

/*
 * Work queue function for long wakeup debug.
 */
static void pm_logger_wq_fnc(struct work_struct *work)
{
	pr_info("Long active time, printing pm logger\n");
	pm_parser_display_log(1);
	pm_logger_print = 0;
}

/*
 * Add logger trace with 2 parameters.
 */
void pm_logger_app_add_trace_short(int event,
				   unsigned int timeStamp,
				   unsigned int arg1,
				   unsigned int arg2)
{
	register unsigned int entry = pm_logger_app.current_entry;

	if (pm_logger_app.enabled == PM_LOGGER_STOP) /* do noting */
		return;

	/* if end of buffer and ONESHOT mode, stop */
	if ((pm_logger_app.mode == PM_LOGGER_ONESHOT_MODE) &&
		(entry+4 > pm_logger_app.buffSize-1)) {
		pr_info("PM LOGGER APP: buffer is full\n");
		pm_logger_app_stop();
		return;
	}

	/* add 24 bits of sync pattern and 8 bit of event num */
	pm_logger_app.buffer[entry++] =
	(event & PM_LOG_EVENT_MASK) | PM_LOG_SYNC_PATTERN;
	entry = entry & (pm_logger_app.buffSize - 1); /* like modulo */

	/* add time stamp */
	pm_logger_app.buffer[entry++] = timeStamp;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add arguements */
	pm_logger_app.buffer[entry++] = arg1;
	entry = entry & (pm_logger_app.buffSize - 1);

	pm_logger_app.buffer[entry++] = arg2;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* update current entry */
	pm_logger_app.current_entry = entry;
}
EXPORT_SYMBOL(pm_logger_app_add_trace_short);

/*
 * Add logger trace with unknown number of parameters.
 */
void pm_logger_app_add_trace(unsigned int num_args,
			     int event,
			     unsigned int timeStamp,
			     ...)
{
	va_list ap;
	int i = 0;
	unsigned int entry = pm_logger_app.current_entry;

	if (pm_logger_app.enabled == PM_LOGGER_STOP) /* do noting */
		return;

	/* if end of buffer and ONESHOT mode, stop */
	if ((pm_logger_app.mode == PM_LOGGER_ONESHOT_MODE) &&
		(entry+2+num_args > pm_logger_app.buffSize-1)) {
		pr_info("PM LOGGER APP: buffer is full\n");
		pm_logger_app_stop();
		return;
	}

	/* add 24 bits of sync pattern and 8 bit of event num */
	pm_logger_app.buffer[entry++] =
	(event & PM_LOG_EVENT_MASK) | PM_LOG_SYNC_PATTERN;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add time stamp */
	pm_logger_app.buffer[entry++] = timeStamp;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add arguements */
	va_start(ap, timeStamp);
	while (i < num_args) {
		pm_logger_app.buffer[entry++] = va_arg(ap, unsigned int);
		entry = entry & (pm_logger_app.buffSize - 1);
		i++;
	}
	va_end(ap);

	/* update current entry */
	pm_logger_app.current_entry = entry;
}
EXPORT_SYMBOL(pm_logger_app_add_trace);

/*
 * Clear buffer traces, and reset entry to index 0.
 */
void pm_logger_app_clear(void)
{
	/* clear buffer */
	if (pm_logger_app.buffer)
		memset(pm_logger_app.buffer, 0x00,
			pm_logger_app.buffSize * sizeof(unsigned int));

	/* update current entry */
	pm_logger_app.current_entry = 0;
}

/*
 * Allocate buffer to buffSize.
 */
int pm_logger_app_alloc_buffer(void)
{
	pm_logger_app.buffer =
		kzalloc(pm_logger_app.buffSize * sizeof(unsigned int),
		GFP_KERNEL);
	if (pm_logger_app.buffer == NULL) {
		pr_info("PM Logger: failed to allocate buffer\n");
		return -1;
	}

	return 0;
}

/*
 * Change buffer size.
 */
int pm_logger_app_change_buffSize(unsigned int new_buffSize)
{
	int ret;
	unsigned long flags;

	if (new_buffSize == pm_logger_app.buffSize)
		return 0;

	/* verify that buffer size is legal */
	if ((new_buffSize < PM_LOGGER_BUFFER_SZ_MIN) ||
		(new_buffSize > PM_LOGGER_BUFFER_SZ_MAX)) {
		pr_info("PM Logger: invalid buffer size\n");
		return -1;
	}

	/* verify the buffer size is a power of 2 */
	if (is_valid_size(new_buffSize) == false) {
		pr_info("PM Logger: buffer size must be power of 2\n");
		return -1;
	}

	/* disable interrupts, so no write to buffer will occure */
	local_fiq_disable();
	local_irq_save(flags);

	/* free old buffer */
	kfree(pm_logger_app.buffer);

	/* update buffer size var */
	pm_logger_app.buffSize = new_buffSize;

	/* allocate new buffer with new size and update current entry 0 */
	ret = pm_logger_app_alloc_buffer();
	if (ret == -1)
		return -1;
	pm_logger_app.current_entry = 0;

	/* enable interrupts */
	local_irq_restore(flags);
	local_fiq_enable();

	return 0;
}

/*
 * Enable logger tracing, allocate if needed.
 */
void pm_logger_app_start(void)
{
	int ret;

	/* allocate buffer if needed */
	if (pm_logger_app.buffer == NULL) {
		ret = pm_logger_app_alloc_buffer();
		if (ret == -1)
			return;
	}

	/* enable buffer traces */
	pm_logger_app.enabled = PM_LOGGER_START;
}

/*
 * Disable logger tracing.
 */
void pm_logger_app_stop(void)
{
	pm_logger_app.enabled = PM_LOGGER_STOP;
}

/*
 * Set logger mode to reg or oneshot mode.
 */
void set_pm_logger_app_mode(int mode)
{
	pm_logger_app_stop();
	pm_logger_app_clear();
	pm_logger_app.mode = mode;
	pm_logger_app_start();
}

/*
 * Get logger mode.
 */
int get_pm_logger_app_mode(void)
{
	return pm_logger_app.mode;
}

/*
 * Get logger buffer pointer.
 */
u32 *get_pm_logger_app_buffer(void)
{
	return pm_logger_app.buffer;
}

/*
 * Get logger size.
 */
int get_pm_logger_app_buffSize(void)
{
	return pm_logger_app.buffSize;
}

/*
 * Get current entry.
 */
int get_pm_logger_app_current_entry(void)
{
	return pm_logger_app.current_entry;
}

/*
 * Get logger database pointer.
 */
u8 ***get_pm_logger_app_db(void)
{
	return (u8 ***)pm_logger_app_db;
}

/*
 * Get logger status stop or start.
 */
int get_pm_logger_app_status(void)
{
	return pm_logger_app.enabled;
}

/*
 * Check if buffer size is power of 2.
 */
int is_valid_size(unsigned int size)
{
	return ((size != 0) && !(size & (size - 1)));
}

/*
 * Init - set default parameters.
 */
static int __init pm_logger_app_init(void)
{
	/* init logger parameters */
	pm_logger_app.buffSize = PM_LOGGER_BUFFER_SZ_DEFAULT;
	pm_logger_app.enabled = PM_LOGGER_STOP;
	pm_logger_app.debug_length_in_msec = 0;
	pm_logger_app.mode = PM_LOGGER_REG_MODE;
	pm_logger_app.current_entry = 0;
	pm_logger_app.buffer = NULL;

	/* logger default on for mipsram dump.
	 * comment this line for logger default off */
	pm_logger_app_start();

	return 0;
}

/*
 * Exit - free buffer.
 */
static void __exit pm_logger_app_exit(void)
{
	kfree(pm_logger_app.buffer);
	pm_logger_app.buffer = NULL;
}

/*
 * Set debug_length_in_msec value for long wakeup debug.
 */
void pm_logger_app_set_debug_length_in_msec(unsigned int msec)
{
	pm_logger_app.debug_length_in_msec = msec;
}

/*
 * Check if active time length was valid.
 */
void debug_check_active_time(unsigned int active_time_stop,
	unsigned int active_time_start)
{
	if (pm_logger_app.debug_length_in_msec &&
		((active_time_stop - active_time_start) * 1000 / 32768) >
			pm_logger_app.debug_length_in_msec &&
			pm_logger_print)
		schedule_delayed_work(&pm_logger_wq, msecs_to_jiffies(1000));
}

/*
 * Set pm_logger_print var to be 1.
 */
void turn_on_pm_logger_print(void)
{
	pm_logger_print = 1;
}

/*
 * Add string aligned to 32-bit, ending with 0.
 */
static unsigned int pm_logger_app_add_string(unsigned int entry, char *string)
{
	size_t size;
	int word_size;

	size = strlen(string) + 1;
	word_size = ((size-1) / sizeof(unsigned int)) + 1;

	/* split string */
	if ((entry + word_size) > pm_logger_app.buffSize) {
		int first_size = (pm_logger_app.buffSize - entry)*
				sizeof(unsigned int);
		memcpy((void *)(&pm_logger_app.buffer[entry]),
				string, first_size);
		string += first_size;
		memset((void *)(&pm_logger_app.buffer[0]), 0,
				(word_size*sizeof(unsigned int)) - first_size);
		memcpy((void *)(&pm_logger_app.buffer[0]),
				string, size - first_size);
	} /* regular string copy */
	else {
		memset((void *)(&pm_logger_app.buffer[entry]), 0,
				word_size*sizeof(unsigned int));
		memcpy((void *)(&pm_logger_app.buffer[entry]),
				string, size);
	}

	return word_size;
}

/*
 * Add string trace, arguments recieved in array.
 */
void pm_logger_app_add_temp_trace_array(unsigned int num_args,
				unsigned int timeStamp,
				char *string,
				unsigned int *var_array)
{
	int i = 0;
	unsigned int entry = pm_logger_app.current_entry;

	if (pm_logger_app.enabled == PM_LOGGER_STOP) /* do noting */
		return;

	/* if end of buffer and ONESHOT mode, stop */
	if ((pm_logger_app.mode == PM_LOGGER_ONESHOT_MODE) &&
		(entry+2+num_args > pm_logger_app.buffSize-1)) {
		pr_info("PM LOGGER APP: buffer is full\n");
		pm_logger_app_stop();
		return;
	}

	/* add 24 bits of sync pattern and 8 bit of event num */
	pm_logger_app.buffer[entry++] =
	(PM_STRING & PM_LOG_EVENT_MASK) | PM_LOG_SYNC_PATTERN;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add time stamp */
	pm_logger_app.buffer[entry++] = timeStamp;
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add string */
	entry += pm_logger_app_add_string(entry, string);
	entry = entry & (pm_logger_app.buffSize - 1);

	/* add arguements */
	while (i < num_args) {
		pm_logger_app.buffer[entry++] = var_array[i];
		entry = entry & (pm_logger_app.buffSize - 1);
		i++;
	}

	/* update current entry */
	pm_logger_app.current_entry = entry;
}
EXPORT_SYMBOL(pm_logger_app_add_temp_trace_array);

/*
 * Add string trace with given num of parameters.
 */
void pm_logger_app_add_temp_trace(unsigned int num_args,
				unsigned int timeStamp,
				char *string,
				...)
{
	va_list ap;
	unsigned int *var_array;
	int i = 0;

	var_array = kzalloc(num_args*sizeof(unsigned int), GFP_KERNEL);
	if (var_array == ZERO_SIZE_PTR) {
		pr_info("\nCan't allocate buffer\n");
		return;
	}

	/* add arguements */
	va_start(ap, string);
	while (i < num_args)
		var_array[i++] = va_arg(ap, unsigned int);
	va_end(ap);

	pm_logger_app_add_temp_trace_array(num_args, timeStamp,
			string, var_array);
	kfree(var_array);
}
EXPORT_SYMBOL(pm_logger_app_add_temp_trace);

subsys_initcall(pm_logger_app_init);
