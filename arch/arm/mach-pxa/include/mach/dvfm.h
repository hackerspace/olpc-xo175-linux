/*
 * Copyright (C) 2003-2004 Intel Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef DVFM_H
#define DVFM_H

#ifdef __KERNEL__
enum {
	FV_NOTIFIER_QUERY_SET = 1,
	FV_NOTIFIER_PRE_SET = 2,
	FV_NOTIFIER_POST_SET = 3,
};

enum {
	CPUFREQ_PROFILER = 0,
	MSPM_PROFILER,
};

extern unsigned int cur_profiler;


#define MAXTOKENS			80
#define CONSTRAINT_NAME_LEN		20

#define DVFM_MAX_NAME			32
#define DVFM_MAX_DEVICE			32

#define DVFM_FREQUENCY_NOTIFIER		0
#define	DVFM_LOWPOWER_NOTIFIER		1

#define DVFM_FREQ_PRECHANGE		0
#define DVFM_FREQ_POSTCHANGE		1

#define DVFM_LOWPOWER_PRECHANGE		0
#define DVFM_LOWPOWER_POSTCHANGE	1

#define APPS_COMM_D2_THRESHOLD 326

/* set the lowest operating point that is equal or higher than specified */
#define RELATION_LOW			0
/* set the highest operating point that is equal or lower than specified */
#define RELATION_HIGH			1
/* set the specified operating point */
#define RELATION_STICK			2

/* Both of these states are used in statistical calculation */
#define CPU_STATE_RUN			1
#define CPU_STATE_IDLE			2

/*
 * operating point definition
 */

struct op_info {
	void *op;
	struct list_head list;
	unsigned int index;
	unsigned int device;	/* store the device ID blocking OP */
};

struct dvfm_freqs {
	unsigned int old;	/* operating point index */
	unsigned int new;	/* operating point index */
	struct op_info old_info;
	struct op_info new_info;
	unsigned int flags;
};

struct info_head {
	struct list_head list;
	rwlock_t lock;
	unsigned int device;	/* store the registerred device ID */
};

struct head_notifier {
	spinlock_t lock;
	struct notifier_block *head;
};

struct dvfm_lock {
	spinlock_t lock;
	unsigned long flags;
	int dev_idx;
	int count;
};

/*
 * Store the dev_id and dev_name.
 * Registered device number can't be larger than 32.
 */
struct dvfm_trace_info {
	struct list_head list;
	int index;		/* index is [0,31] */
	unsigned int dev_id;	/* dev_id == 1 << index */
	char name[DVFM_MAX_NAME];
};


struct dvfm_driver {
	int (*get_opinfo) (void *driver_data, void *info);
	int (*count) (void *driver_data, struct info_head * op_table);
	int (*set) (void *driver_data, struct dvfm_freqs * freq,
		    unsigned int new, unsigned int relation);
	int (*dump) (void *driver_data, struct op_info * md, char *buf);
	char *(*name) (void *driver_data, struct op_info * md);
	int (*request_set) (void *driver_data, int index);
	int (*request_set_relation_high) (void *driver_data, int index);
	int (*enable_dvfm) (void *driver_data, int dev_id);
	int (*disable_dvfm) (void *driver_data, int dev_id);
	int (*enable_op) (void *driver_data, int index, int relation);
	int (*disable_op) (void *driver_data, int index, int relation);
	int (*volt_show) (void *driver_data, char *buf);
	unsigned int (*ticks_to_usec) (unsigned int);
	unsigned int (*ticks_to_sec) (unsigned int);
	unsigned int (*read_time) (void);
	int (*current_core_freq_get)(void);
	int (*core_freqs_table_get)(void *driver_data,
			int *freqs_table,
			int *size,
			int table_sz);
	void *priv;
};

extern struct dvfm_driver *dvfm_driver;
extern struct info_head *dvfm_op_list;
extern unsigned int op_nums;
extern int PowerDisabled;
extern int DvfmDisabled;

#endif
#endif
