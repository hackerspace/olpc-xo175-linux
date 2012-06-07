
#ifndef DEBUG_PM_H
#define DEBUG_PM_H

/* for pm logger debugfs */
#define PM_LOGGER_COMM_DISPLAY 0
#define PM_LOGGER_APPS_DISPLAY 1
#define PM_LOGGER_BUF_CLEAR 2
#define PM_LOGGER_START_LOG 3
#define PM_LOGGER_STOP_LOG 4
#define PM_LOGGER_CHANGE_BUF_SIZE 5
#define PM_LOGGER_CHANGE_ONESHOT_MODE 6
#define PM_LOGGER_CHANGE_REG_MODE 7
#define PM_LOGGER_SET_MAX_D2_ACTIVE_TIME 8
#define PM_LOGGER_ADD_STRING 9

void print_pm_logger_usage(void);

enum pxa9xx_force_lpm {
	PXA9xx_Force_None,
	PXA9xx_Force_D2,
	PXA9xx_Force_D1,
	PXA9xx_Force_CGM,
	PXA9xx_Force_count
};

enum stats_clk_event {
	GC_CLK_ON,
	GC_CLK_OFF,
	VMETA_CLK_ON,
	VMETA_CLK_OFF
};

/* for debugfs*/
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <mach/pxa95x_dvfm.h>
#include "../../clock.h"

/* for pm logger dump*/
struct pm_logger_header {
	unsigned int app_table_size;
	unsigned int op_table_size;
	unsigned int dvfm_table_size;
	unsigned int buffer_size;
};

struct pm_logger_buffer_descriptor {
	char *buffer; /* main copy buffer */
	unsigned int len;
	struct pm_logger_header header;
};
struct gc_vmeta_op_cycle_type {
	unsigned long op_idx;
	unsigned int runtime;
	unsigned int idletime;
	unsigned int count;
};

struct gc_vmeta_ticks {
	unsigned long gc_cur_freq;
	unsigned long vm_cur_freq;
	unsigned int gc_prev_timestamp;
	unsigned int vm_prev_timestamp;
	unsigned int gc_state;
	unsigned int vmeta_state;
	unsigned int gc_stats_start;
	unsigned int gc_stats_stop;
	unsigned int vm_stats_start;
	unsigned int vm_stats_stop;
	struct gc_vmeta_op_cycle_type GC_op_ticks_array[GC_VM_OP_NUM_MAX];
	struct gc_vmeta_op_cycle_type VM_op_ticks_array[GC_VM_OP_NUM_MAX];
	struct mutex gc_stats_table_lock;
};
extern unsigned int read_curtime(void);

extern struct proc_op_array *proc_op;

#define EVENT_DB_ROW_MAGIC			0x44424C4E
#define OP_TABLE_ROW_MAGIC			0x4F504C4E
#define DEV_TABLE_ROW_MAGIC			0x44564C4E
#define EVENT_DB_HEADER_MAGIC		0x45564442
#define OP_TABLE_HEADER_MAGIC		0x4F505442
#define DEV_TABLE_HEADER_MAGIC		0x44565442

#ifdef CONFIG_DEBUG_FS
#define LPM_NAMES_LEN 20
extern const char pxa9xx_force_lpm_names__[][LPM_NAMES_LEN];
#endif
extern enum pxa9xx_force_lpm ForceLPM;
extern enum pxa9xx_force_lpm LastForceLPM;
extern unsigned int ForceLPMWakeup;
extern int RepeatMode;
extern int ForceVCTCXO_EN;
extern int ForceC0;

extern uint32_t profilerRecommendationPP;
extern uint32_t profilerRecommendationEnable;
void pxa_9xx_power_init_debugfs(void);
void pxa_9xx_power_cleanup_debugfs(void);
void gc_vmeta_stats_clk_event(enum stats_clk_event event);

#endif
