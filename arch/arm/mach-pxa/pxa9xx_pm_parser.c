/*
 *  PM Parser
 *
 *  Support for Power management related event parser over PXAxxx
 *
 *  Author:	Moran Raviv
 *  Created:	Dec 24, 2010
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
#include <mach/pmu.h>
#include <mach/dvfm.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/pxa95x_pm.h>

#include <linux/semaphore.h>
#include <mach/pxa9xx_pm_parser.h>
#include <mach/pxa9xx_pm_logger.h>

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "MARVELL"
#define DRIVER_DESC "Power management event Parser"

/* module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

struct pm_logger_descriptor pm_logger;
static u8 ***database;

static u32 d1_exits_on_timer;
static u32 d2_exits_on_timer;
static u32 cgm_exits_on_timer;

/* this is how the pm logger looks like:
sync(24)+event num(8) - time stamp - arguments ...
sync(24)+event num(8) - time stamp - arguments ... */

/*
 * Checks for the end of buffer
 */
static unsigned int is_pm_parser_end_of_buffer(int entry)
{
	int count = PM_PARSER_EOB_PATTERN_COUNT;

	while (count > 0) {
		if (pm_logger.buffer[(entry++) & (pm_logger.buffSize-1)] !=
				PM_PARSER_EOB_PATTERN)
			return 0;
		count--;
	}

	return 1;
}

/*
* counts specific events
*/
static void pm_parser_count_timer_wakeups(u32 event, int args_num, int entry)
{
	/* lpm exits */
	if (((event == PM_D1_EXIT) || (event == PM_D2_EXIT) ||
	    (event == PM_CGM_EXIT)) && (args_num > 0)) {

		u32 wakeup_src = pm_logger.buffer[entry];
		entry = (entry+1) & (pm_logger.buffSize-1);

		/* wakeup reason is timer */
		if (wakeup_src & PXA95x_PM_WE_OST) {
			if (event == PM_D1_EXIT)
				d1_exits_on_timer++;
			else if (event == PM_D2_EXIT)
				d2_exits_on_timer++;
			else if (event == PM_CGM_EXIT)
				cgm_exits_on_timer++;
		}
	}
}

/*
 * Calculates the number of event arguments by counting the number
 * of entries between ts and next sync pattern.
 */
int get_pm_parser_args_num(int tsIndex)
{

	int args_num = 0;
	int entry = tsIndex;
	int size = pm_logger.buffSize;

	while (size != 0) {
		entry = (entry+1) & (pm_logger.buffSize-1); /* like modulo */

		/* this is sync pattern. stop */
		if ((pm_logger.buffer[entry] & PM_LOG_SYNC_MASK) ==
			PM_LOG_SYNC_PATTERN)
			break;

		/* check for end of buffer */
		if (is_pm_parser_end_of_buffer(entry))
			break;

		args_num++; /* still arguments. increment count and continue  */
		size--;
	}

	return args_num;
}

/*
 * Find start entry using current entry.
 */
static int get_pm_parser_start_entry(void)
{
	int entry = pm_logger.current_entry;
	int size = pm_logger.buffSize;

	/* check if buffer is allocated */
	if (pm_logger.buffer == NULL) {
		pr_info("PM Parser: apps logger is disabled\n");
		return -1;
	}

	/* search first sync pattern starting from current entry */
	while (size != 0) {
		/* this is sync pattern. stop */
		if ((pm_logger.buffer[entry] & PM_LOG_SYNC_MASK) ==
			PM_LOG_SYNC_PATTERN)
			return entry;
		else {
			/* not sync pattern yet. continue searching */
			entry = (entry+1) & (pm_logger.buffSize-1);
			size--;
		}
	}

	/* loop only once on buffer.
	this is for cases the buffer empty and no sync pattern is found */
	pr_info("PM Parser: Buffer is empty\n");
	return -1;
}

/*
 * Checks if given string is equal to database name.
 */
static unsigned int pm_parser_check_db_name(const char *db_name, const char *name)
{
	return (strlen(db_name) == strlen(name) && strcmp(db_name, name) == 0);
}

/*
 * Check if parser print as unsigned int
 */
static unsigned int is_pm_parser_dec_print(const char *db_name)
{
	if (strlen(db_name) == 4 && strcmp(db_name, "FREQ") == 0)
		return 1;

	return 0;
}

/*
 * Returns device name on given index.
 */
static char *pm_parser_get_dev_name(unsigned int index)
{
	char *ret = NULL;
	struct dvfm_trace_info *dev_ptr = NULL;

	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(dev_ptr, &dvfm_trace_list.list,	list) {
		if (dev_ptr != NULL) {
			if ((unsigned int)dev_ptr->index == index) {
				ret = dev_ptr->name;
				break;
			}
		}
	}
	read_unlock(&dvfm_trace_list.lock);
	return ret;
}

/*
 * Returns OP name on given index.
 */
static char *pm_parser_get_op_name(unsigned int index)
{
	char *ret = NULL;
	struct op_info *opinfo_ptr = NULL;
	struct dvfm_md_opt *op_ptr = NULL;

	read_lock(&pxa95x_dvfm_op_list.lock);
	list_for_each_entry(opinfo_ptr, &pxa95x_dvfm_op_list.list, list) {
		if (opinfo_ptr != NULL) {
			if ((unsigned int)opinfo_ptr->index == index) {
				op_ptr = (struct dvfm_md_opt *) opinfo_ptr->op;
				if (op_ptr != NULL)	{
					ret = op_ptr->name;
					break;
				}
			}
		}
	}
	read_unlock(&pxa95x_dvfm_op_list.lock);
	return ret;
}

/*
 * Print string trace by characters, return size in cells
 */
static int print_string_trace(unsigned int entry, unsigned int args_num)
{
	int string_size = 0, first_part = 0;
	int broken_string = false;
	char *char_buffer = (char *)&pm_logger.buffer[entry];

	while (string_size < args_num) {
		string_size++;
		char_buffer += sizeof(unsigned int);

		/* this cell is the end of string, since it ends with 0 */
		if (*(char_buffer - 1) == 0)
			break;

		if ((entry + string_size) >= pm_logger.buffSize
						&& broken_string == false) {
			broken_string = true;
			first_part = string_size;
			char_buffer = (char *)&pm_logger.buffer[0];
		}
	}

	if (broken_string == true) {
		char *tmp_buf = kzalloc((first_part*sizeof(unsigned int))+1,
								GFP_KERNEL);
		memcpy(tmp_buf,
			&pm_logger.buffer[entry],
			first_part*sizeof(unsigned int));
		printk(":%s", tmp_buf);
		char_buffer = (char *)&pm_logger.buffer[0];
		printk("%s", char_buffer);
		kfree(tmp_buf);
	} else {
		char_buffer = (char *)&pm_logger.buffer[entry];
		printk(":%s", char_buffer);
	}

	return string_size;
}

/*
 * Display buffer content. use database to display event and
 * registers' names.
 */
void pm_parser_display_log(int subsystem)
{

	int first_entry, entry, i, args_num, start_status = PM_LOGGER_STOP;
	u32 ts = 0, event, first_ts, total_time;
	const char *dbstr;
	char *valstr;
	unsigned int val;

	d1_exits_on_timer = d2_exits_on_timer = cgm_exits_on_timer = 0;

	pr_info("\n*****START PM LOGGER TRACE*****\n");

	switch (subsystem) {
	case COMM_SS:
		pr_info("PM Parser: not supported yet\n");
		return;
	case APP_SS:
		/* stop logger tracing until praser prints all */
		start_status = get_pm_logger_app_status();
		pm_logger_app_stop();
		pm_logger.buffSize = get_pm_logger_app_buffSize();
		pm_logger.current_entry =
				get_pm_logger_app_current_entry();
		pm_logger.buffer = get_pm_logger_app_buffer();
		database = get_pm_logger_app_db();
		break;
	}

	/* get start entry to print from it */
	first_entry = get_pm_parser_start_entry();
	if (first_entry == -1) {
		if ((subsystem == APP_SS) && (start_status == PM_LOGGER_START))
			pm_logger_app_start(); /* restart */
		return;
	}

	/* get first ts for the header at the end */
	first_ts = pm_logger.buffer[(first_entry+1) & (pm_logger.buffSize-1)];

	entry = first_entry;

	/* start parsing logger */
	do {

		if ((pm_logger.buffer[entry] & PM_LOG_SYNC_PATTERN)
			!= PM_LOG_SYNC_PATTERN)
			break;

		/* get event num */
		event = pm_logger.buffer[entry] & PM_LOG_EVENT_MASK;
		entry = (entry+1) & (pm_logger.buffSize-1);

		/* get ts and print */
		if (pm_logger.buffer[entry] == 0)
			break; /* from this point there are no more events */
		ts = pm_logger.buffer[entry];
		printk("%u", ts);

		/* get args num */
		args_num = get_pm_parser_args_num(entry);
		entry = (entry+1) & (pm_logger.buffSize-1);

		/* count timer wakeup events */
		pm_parser_count_timer_wakeups(event, args_num, entry);

		/* print event */
		i = 0;
		if (GET_DB_STRING(database, event, i) != NULL) {
			/* event name */
			printk(",%s",
				GET_DB_STRING(database, event, i));
			i++;
		} else
			printk(",%x", event); /* event number */

		/* handle string trace */
		if (event == PM_STRING) {
			int string_size = print_string_trace(entry, args_num);
			args_num -= string_size;
			entry = (entry+string_size) & (pm_logger.buffSize-1);
		}

		/* print arguments */
		while (args_num) {
			dbstr = GET_DB_STRING(database, event, i);
			val = pm_logger.buffer[entry];
			valstr = NULL;
			if (dbstr != NULL) {
				if (pm_parser_check_db_name(dbstr, "DEV"))
					valstr = pm_parser_get_dev_name(val);
				else if (pm_parser_check_db_name(dbstr, "OP"))
					valstr = pm_parser_get_op_name(val);

				if (valstr != NULL)
					printk(",%s:%s", dbstr, valstr);
				else if (is_pm_parser_dec_print(dbstr))
					printk(",%s:%u", dbstr, val);
				else
					printk(",%s:0x%x", dbstr, val);
				i++;
			} else
				printk(",raw data:0x%x",
					pm_logger.buffer[entry]);

			entry = (entry+1) & (pm_logger.buffSize-1);
			args_num--;
		}

		printk("\n");
	} while (entry != first_entry);

	/* restart logger tracing */
	if ((subsystem == APP_SS) && (start_status == PM_LOGGER_START))
		pm_logger_app_start();

	/* calculate total log time for header */
	total_time = (ts-first_ts)/32;

	/* print header */
	pr_info("\nTotal Log Time: %u milisecs",
		total_time);
	pr_info("\nTotal Buffer Size: %u bytes\n",
		pm_logger.buffSize * sizeof(unsigned int));
	pr_info("\nLPM wakeup from timer:  CGM- %u , D1- %u , D2- %u.\n",
		cgm_exits_on_timer, d1_exits_on_timer, d2_exits_on_timer);
}
