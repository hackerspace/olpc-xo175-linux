#ifdef CONFIG_DEBUG_FS

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <mach/debug_pm.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <mach/hardware.h>
#include <mach/pxa9xx_pm_logger.h>
#include <mach/pxa9xx_pm_parser.h>
#include <mach/regs-ost.h>
#include <mach/dvfm.h>
#include <mach/pxa95x_dvfm.h>

#include <mach/mfp.h>
#include <mach/mfp-pxa3xx.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include "generic.h"

/*
 * Debug fs
 */
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

enum pxa9xx_force_lpm ForceLPM = PXA9xx_Force_None;
enum pxa9xx_force_lpm LastForceLPM = PXA9xx_Force_None;
unsigned int ForceLPMWakeup;
int RepeatMode;
int ForceOP, ForcedOPIndex, ForceC0, ForceVCTCXO_EN, EnableD2VoltageChange = 1;

#define PXA9XX__POWER_DEBUG_NAME "PM"
#define USER_BUF_SIZE 50

static ssize_t PXA9xx_pm_logger_read(struct file *file,
				     char __user *userbuf,
				     size_t count, loff_t *ppos);
static ssize_t PXA9xx_pm_logger_write(struct file *file,
				      const char __user *ubuf,
				      size_t count, loff_t *ppos);

static ssize_t PXA9xx_DVFM_ForceLPM_seq_read(struct file *file,
					     char __user *userbuf,
					     size_t count, loff_t *ppos);

static int PXA9xx_DVFM_ForceLPM_seq_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos);

static ssize_t PXA9xx_DVFM_profilerRecommendation_seq_read(struct file *file,
							   char __user *userbuf,
							   size_t count,
							   loff_t *ppos);
static int PXA9xx_DVFM_profilerRecommendation_write(struct file *file,
						    const char __user *ubuf,
						    size_t count,
						    loff_t *ppos);
static int pxa_9xx_power_MeasureCoreFreq_open(struct inode *inode,
					      struct file *file);

static int pxa_9xx_gc_ticks_write(struct file *file,
					const char __user *ubuf,
					size_t count, loff_t *ppos);

static int pxa_9xx_gc_ticks_read(struct file *file,
				 char __user *userbuf,
				 size_t count, loff_t *ppos);

static int pxa_9xx_vm_ticks_write(struct file *file,
				  const char __user *ubuf,
				  size_t count, loff_t *ppos);

static int pxa_9xx_vm_ticks_read(struct file *file,
				 char __user *userbuf,
				 size_t count, loff_t *ppos);


static int pxa_9xx_gc_vmeta_stats_write(struct file *file,
					const char __user *ubuf,
					size_t count, loff_t *ppos);

static int pxa_9xx_gc_vmeta_stats_read(struct file *file,
				       char __user *userbuf,
				       size_t count, loff_t *ppos);

static ssize_t PXA9xx_force_VCTCXO_EN_read(struct file *file,
					   char __user *userbuf,
					   size_t count, loff_t *ppos);
static ssize_t PXA9xx_force_VCTCXO_EN_write(struct file *file,
					    const char __user *ubuf,
					    size_t count, loff_t *ppos);

static ssize_t PXA9xx_PI2C_read(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos);
static ssize_t PXA9xx_PI2C_write(struct file *file,
				 const char __user *ubuf,
				 size_t count, loff_t *ppos);

static ssize_t PXA9xx_force_C0_read(struct file *file,
							char __user *userbuf,
							size_t count,
							loff_t *ppos);
static ssize_t PXA9xx_force_C0_write(struct file *file,
							const char __user *ubuf,
							size_t count,
							loff_t *ppos);

static ssize_t PXA9xx_pm_logger_buffer_read(struct file *file,
							char __user *userbuf,
							size_t count,
							loff_t *ppos);
static int PXA9xx_pm_logger_buffer_open(struct inode *inode,
							struct file *file);
static int PXA9xx_pm_logger_buffer_release(struct inode *inode,
							struct file *file);

/* The exec names of the enum defined in debug_pm.h */
const char pxa9xx_force_lpm_names__[][LPM_NAMES_LEN] = {
	"PXA9xx_Force_None",
	"PXA9xx_Force_D2",
	"PXA9xx_Force_D1",
	"PXA9xx_Force_CGM"
};

static struct dentry *dbgfs_root, *pmLogger_file, *forceVCTCXO_EN_file,
			*ForceLPM_file, *MeasureCoreFreq_file,
			*profilerRecommendation_file, *GcTicks_file,
			*VmTicks_file,
			*GcVmetaStats_file,
			*PI2C_file,
			*ForceC0_file,
			*pmLoggerBuffer_file;

uint32_t ForceLPMWakeups_tmp;
uint32_t ForceLPM_tmp;
uint32_t profilerRecommendationPP = 6;
uint32_t profilerRecommendationEnable;

static const struct file_operations PXA9xx_file_op_pmLogger = {
	.owner = THIS_MODULE,
	.read = PXA9xx_pm_logger_read,
	.write = PXA9xx_pm_logger_write,
};

static const struct file_operations PXA9xx_file_force_VCTCXO_EN = {
	.owner = THIS_MODULE,
	.read = PXA9xx_force_VCTCXO_EN_read,
	.write = PXA9xx_force_VCTCXO_EN_write,
};

static const struct file_operations PXA9xx_file_force_C0 = { \
	.owner		= THIS_MODULE, \
	.read		= PXA9xx_force_C0_read, \
	.write		 = PXA9xx_force_C0_write, \
};

static const struct file_operations PXA9xx_file_op_profilerRecommendation = {
	.owner = THIS_MODULE,
	.read = PXA9xx_DVFM_profilerRecommendation_seq_read,
	.write = PXA9xx_DVFM_profilerRecommendation_write,
};

static const struct file_operations PXA9xx_file_op_ForceLPM = {
	.owner = THIS_MODULE,
	.read = PXA9xx_DVFM_ForceLPM_seq_read,
	.write = PXA9xx_DVFM_ForceLPM_seq_write,
};

static const struct file_operations PXA9xx_file_op_gc_ticks = {
	.owner = THIS_MODULE,
	.read = pxa_9xx_gc_ticks_read,
	.write = pxa_9xx_gc_ticks_write,
};

static const struct file_operations PXA9xx_file_op_vm_ticks = {
	.owner = THIS_MODULE,
	.read = pxa_9xx_vm_ticks_read,
	.write = pxa_9xx_vm_ticks_write,
};

static const struct file_operations PXA9xx_file_op_gc_vmeta_stats = {
	.owner = THIS_MODULE,
	.read = pxa_9xx_gc_vmeta_stats_read,
	.write = pxa_9xx_gc_vmeta_stats_write,
};

static const struct file_operations PXA9xx_file_PI2C = {
	.owner = THIS_MODULE,
	.read = PXA9xx_PI2C_read,
	.write = PXA9xx_PI2C_write,
};

static const struct file_operations PXA9xx_file_op_MeasureCoreFreq = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.open = pxa_9xx_power_MeasureCoreFreq_open,
};

static const struct file_operations PXA9xx_file_buffer_pmLogger = {
	.owner = THIS_MODULE,
	.read = PXA9xx_pm_logger_buffer_read,
	.open = PXA9xx_pm_logger_buffer_open,
	.release = PXA9xx_pm_logger_buffer_release,
};

void print_pm_logger_usage()
{
	pr_info("PM Logger Usage:\n"
		"0- print comm log\n"
		"1- print apps log\n"
		"2- clear buffer\n"
		"3- enable apps logger\n"
		"4- disable apps logger\n"
		"5, size- change buffer size in KB\n"
		"6- change mode to one shot\n"
		"7- change mode to regular\n"
		"8, msec- print apps log when active time bigger than msec\n"
		"9, string{, val1, val2, ...} -add string message and parameters\n");
}

static ssize_t PXA9xx_pm_logger_read(struct file *file,
				     char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	print_pm_logger_usage();
	return 0;
}

static ssize_t PXA9xx_pm_logger_write(struct file *file,
				      const char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	unsigned int pmlogger_val, pmlogger_val2;
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0, min_size, num_args = 0;
	char *pch, *string, *pparams;
	char string_val[USER_BUF_SIZE] = {0};
	unsigned int *var_array;

	/* copy user's input to kernel space */
	min_size = min_t(char, sizeof(buf) - 1, count);
	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;
	pos += sscanf(buf, "%d", &pmlogger_val);

	switch (pmlogger_val) {

	case PM_LOGGER_COMM_DISPLAY:
		pm_parser_display_log(COMM_SS);
		break;

	case PM_LOGGER_APPS_DISPLAY:
		pm_parser_display_log(APP_SS);
		break;

	case PM_LOGGER_BUF_CLEAR:
		pr_info("\nClearing buffer\n");
		pm_logger_app_clear();
		break;

	case PM_LOGGER_START_LOG:
		pr_info("\nEnable logger\n");
		pm_logger_app_start();
		break;

	case PM_LOGGER_STOP_LOG:
		pr_info("\nDisable logger\n");
		pm_logger_app_stop();
		break;

	case PM_LOGGER_CHANGE_BUF_SIZE:
		sscanf(buf + pos + 1, "%d", &pmlogger_val2);
		pr_info("\nChanging buffer size to %d KB\n",
		       pmlogger_val2);
		/* send the function the size in cells */
		pm_logger_app_change_buffSize((pmlogger_val2 * 1024) /
					      sizeof(unsigned int));
		break;

	case PM_LOGGER_CHANGE_ONESHOT_MODE:
		pr_info("\nChanging to one shot mode\n");
		set_pm_logger_app_mode(PM_LOGGER_ONESHOT_MODE);
		break;

	case PM_LOGGER_CHANGE_REG_MODE:
		pr_info("\nChanging to regular mode\n");
		set_pm_logger_app_mode(PM_LOGGER_REG_MODE);
		break;

	case PM_LOGGER_SET_MAX_D2_ACTIVE_TIME:
		pos += sscanf(buf + pos + 1, "%u", &pmlogger_val2);
		if (pmlogger_val2)
			pr_info("\nSet debug_length_in_msec to %d\n",
			       pmlogger_val2);
		else
			pr_info("\nDisable debug_length_in_msec feature\n");
		pm_logger_app_set_debug_length_in_msec(pmlogger_val2);
		break;

	case PM_LOGGER_ADD_STRING:
		/* get pointer to string */
		string = strpbrk(buf, ",");
		if (string == NULL) {
			pr_info("\nNo string was found\n");
			break;
		}
		string++;

		/* get pointer to params */
		pparams = strpbrk(string, ",");

		/* get string */
		strncpy(&string_val[0], string, USER_BUF_SIZE);
		string_val[USER_BUF_SIZE - 1] = 0;

		/* if no params, add only string */
		if (pparams == NULL) {
			pr_info("\nAdding single string\n");
			pm_logger_app_add_temp_trace(0, OSCR4, &string_val[0]);
			break;
		} else
			*pparams = 0;

		/* get params num */
		pch = pparams;
		while ((NULL != pch) && (pch < &buf[USER_BUF_SIZE])) {
			num_args++;
			pch = strpbrk(pch+1, ",");
		}
		var_array = kzalloc(num_args*sizeof(unsigned int),
					GFP_KERNEL);
		if (var_array == ZERO_SIZE_PTR) {
			pr_info("\nCan't allocate buffer\n");
			break;
		}

		/* fill array with params */
		pch = pparams;
		num_args = 0;
		while ((NULL != pch) && (pch < &buf[USER_BUF_SIZE])) {
			sscanf(pch+1, "%x", &pmlogger_val2);
			var_array[num_args++] = pmlogger_val2;
			pch = strpbrk(pch+1, ",");
		}

		pr_info("\nAdding string with arguments\n");
		pm_logger_app_add_temp_trace_array(num_args,
					OSCR4, &string_val[0], var_array);
		kfree(var_array);
		break;

	default:
		print_pm_logger_usage();
		break;

	}

	return count;
}

static ssize_t PXA9xx_force_VCTCXO_EN_read(struct file *file,
					   char __user *userbuf,
					   size_t count, loff_t *ppos)
{
	char buf[30] = { 0 };
	int ret, sum = 0;

	if (ForceVCTCXO_EN)
		ret = snprintf(buf + sum, sizeof(buf) - 1,
			       "force_VCTCXO_EN is ON\n");
	else
		ret = snprintf(buf + sum, sizeof(buf) - 1,
			       "force_VCTCXO_EN is OFF\n");

	if (-1 == ret)
		return ret;
	sum = ret;

	return simple_read_from_buffer(userbuf, count, ppos, buf, sum);
}

static ssize_t PXA9xx_force_VCTCXO_EN_write(struct file *file,
					    const char __user *ubuf,
					    size_t count, loff_t *ppos)
{
	unsigned int val;
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0;
	int min_size;

	/* copy user's input to kernel space */
	min_size = min_t(char, sizeof(buf) - 1, count);
	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;
	pos += sscanf(buf, "%d", &val);

	if (val == 0)
		ForceVCTCXO_EN = 0;
	else
		ForceVCTCXO_EN = 1;

	return count;
}

static ssize_t PXA9xx_force_C0_read(struct file *file,
					char __user *userbuf,
					size_t count,
					loff_t *ppos)
{
	char buf[30] = { 0 };
	int ret = 0;

	if (ForceC0)
		ret = snprintf(buf, sizeof(buf) - 1,
				"Force_C0 is ON\n");
	else
		ret = snprintf(buf, sizeof(buf) - 1,
				"Force_C0 is OFF\n");

	if (-1 == ret)
		return ret;

	return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static ssize_t PXA9xx_force_C0_write(struct file *file,
					const char __user *ubuf,
					size_t count,
					loff_t *ppos)
{
	unsigned int val;
	char buf[USER_BUF_SIZE] = {0};
	int pos = 0;
	int min_size;

	/* copy user's input to kernel space */
	min_size = min_t(char, sizeof(buf)-1, count);
	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;
	pos += sscanf(buf, "%d", &val);

	if (val == 0)
		ForceC0 = 0;
	else
		ForceC0 = 1;

	return count;
}

static ssize_t PXA9xx_DVFM_ForceLPM_seq_read(struct file *file,
					     char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	/* Consider buff size when modifiy this function */
	char buf[512] = { 0 };
	int ret, sum;
	int nameIndex = 0;

	/* Safe version of sprintf of that doesn't suffer from buffer
	 * overruns */
	ret = snprintf(buf, sizeof(buf) - 1,
		       "type [LPM to Force],[0x peripherals for wakeup] <r>\n\n");
	if (-1 == ret)
		return ret;
	sum = ret;

	for (; nameIndex < PXA9xx_Force_count; nameIndex++) {
		ret = snprintf(buf + sum, sizeof(buf) - 1,
			       "Enter %d to select %s\n", nameIndex,
			       pxa9xx_force_lpm_names__[nameIndex]);

		if (-1 == ret)
			return ret;
		sum += ret;
	}

	ret = snprintf(buf + sum, sizeof(buf) - 1,
		       "\n<r> - Repeat mode\n\nForceLPM state = ");
	if (-1 == ret)
		return ret;
	sum += ret;

	if (ForceLPM == 1) {
		if (RepeatMode == 1)
			ret = snprintf(buf + sum, sizeof(buf) - 1,
				       "Repeat mode\n");
		else
			ret = snprintf(buf + sum, sizeof(buf) - 1,
				       "Singleshot mode\n");
		if (-1 == ret)
			return ret;
		sum += ret;
		ret = snprintf(buf + sum, sizeof(buf) - 1,
			       "ForceLPMWakeup = 0x%x\n\n", ForceLPMWakeup);
	} else
		ret = snprintf(buf + sum, sizeof(buf) - 1, "Not active\n\n");

	if (-1 == ret)
		return ret;
	sum += ret;

	return simple_read_from_buffer(userbuf, count, ppos, buf, sum);
}

static int PXA9xx_DVFM_ForceLPM_seq_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0;
	int min_size;
	char ch;

	min_size = min_t(char, sizeof(buf) - 1, count);

	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;

	pos += sscanf(buf, "%d, %x %c",
		      &ForceLPM_tmp, &ForceLPMWakeups_tmp, &ch);

	if (ch == 'r')
		RepeatMode = 1;
	else
		RepeatMode = 0;

	if (ForceLPM_tmp < PXA9xx_Force_count) {
		ForceLPM = (enum pxa9xx_force_lpm)ForceLPM_tmp;
		LastForceLPM = PXA9xx_Force_None;
		ForceLPMWakeup = ForceLPMWakeups_tmp;
	} else
		printk(KERN_WARNING "\n%d is not a valid state\n",
		       ForceLPM_tmp);
	return count;
}

struct gc_vmeta_statsS {
	unsigned int enable;
	unsigned int test_start_time;
	unsigned int test_end_time;
	unsigned int gc_start_time;
	unsigned int gc_stop_time;
	unsigned int gc_is_on;
	unsigned int vmeta_start_time;
	unsigned int vmeta_stop_time;
	unsigned int vmeta_is_on;
	unsigned int gc_total_time;
	unsigned int vmeta_total_time;
};

struct gc_vmeta_statsS gc_vmeta_stats;

#define OFF			0
#define ON			1
#define UNKNOWN		2

static int pxa_9xx_gc_vmeta_stats_read(struct file *file,
				       char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	char buf[512] = { 0 };
	unsigned int sum = 0, timeStamp, test_total_time = 0;

	if (gc_vmeta_stats.enable == 1) {

		timeStamp = OSCR4;
		gc_vmeta_stats.test_end_time = timeStamp;

		if (gc_vmeta_stats.gc_is_on == ON) {
			/* if GC clock is still on, save timestamp
			   and sum-up active time from last start */
			gc_vmeta_stats.gc_stop_time = timeStamp;
			gc_vmeta_stats.gc_total_time +=
			    gc_vmeta_stats.gc_stop_time -
			    gc_vmeta_stats.gc_start_time;

			/* save start timestamp for future results */
			gc_vmeta_stats.gc_start_time = timeStamp;
		}

		if (gc_vmeta_stats.vmeta_is_on == ON) {
			/* if VMETA clock is still on, save timestamp
			   and sum-up active time from last start */
			gc_vmeta_stats.vmeta_stop_time = timeStamp;
			gc_vmeta_stats.vmeta_total_time +=
			    gc_vmeta_stats.vmeta_stop_time -
			    gc_vmeta_stats.vmeta_start_time;

			/* save start timestamp for future results */
			gc_vmeta_stats.vmeta_start_time = timeStamp;
		}
	}

	test_total_time = gc_vmeta_stats.test_end_time -
	    gc_vmeta_stats.test_start_time;

	if (test_total_time) {
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
				"Total test time    = %d seconds\n",
				test_total_time / 32768);

		if (gc_vmeta_stats.gc_total_time == 0)
			sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
					"GC    active time  = No activity\n");
		else
			sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
					"GC    active time  = %d%%\n",
					(gc_vmeta_stats.gc_total_time * 100) /
					(test_total_time));

		if (gc_vmeta_stats.vmeta_total_time == 0)
			sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
					"VMETA active time  = No activity\n");
		else
			sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
					"VMETA active time  = %d%%\n",
					(gc_vmeta_stats.vmeta_total_time * 100) /
					(test_total_time));

	} else
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
				"Please start the GC & VMETA stats first\n");
	return simple_read_from_buffer(userbuf, count, ppos, buf, sum);

}

static int pxa_9xx_gc_vmeta_stats_write(struct file *file,
					const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	unsigned int value, timeStamp;
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0;
	int min_size;

	/* copy user's input to kernel space */

	min_size = min_t(char, sizeof(buf) - 1, count);
	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;
	pos += sscanf(buf, "%d", &value);

	timeStamp = OSCR4;

	if (value == 1) {

		printk(KERN_INFO "\nGC and VMETA stats STARTED\n\n");
		/* Reset and start test */
		gc_vmeta_stats.enable = 1;
		gc_vmeta_stats.test_start_time = timeStamp;
		gc_vmeta_stats.gc_start_time = timeStamp;
		gc_vmeta_stats.vmeta_start_time = timeStamp;
		gc_vmeta_stats.gc_total_time = 0;
		gc_vmeta_stats.vmeta_total_time = 0;

	} else if (value == 0) {

		printk(KERN_INFO "\nGC and VMETA stats STOPED\n\n");
		/* Stop test */
		gc_vmeta_stats.enable = 0;
		gc_vmeta_stats.test_end_time = timeStamp;

		if (gc_vmeta_stats.gc_is_on == ON) {
			/* if GC clock is still on, save timestamp
			   and sum-up active time from last start */
			gc_vmeta_stats.gc_stop_time = timeStamp;
			gc_vmeta_stats.gc_total_time +=
			    gc_vmeta_stats.gc_stop_time -
			    gc_vmeta_stats.gc_start_time;
		}
		if (gc_vmeta_stats.vmeta_is_on == ON) {
			/* if VMETA clock is still on, save timestamp
			   and sum-up active time from last start */
			gc_vmeta_stats.vmeta_stop_time = timeStamp;
			gc_vmeta_stats.vmeta_total_time +=
			    gc_vmeta_stats.vmeta_stop_time -
			    gc_vmeta_stats.vmeta_start_time;
		}

	} else {
		/* Display Usage */
		printk(KERN_INFO "Usage:\n");
		printk(KERN_INFO
		       "echo 1 > GcVmetaStats  -  Reset and start test\n");
		printk(KERN_INFO "echo 0 > GcVmetaStats  -  Stop test\n");
		printk(KERN_INFO "cat GcVmetaStats       -  Display results\n");
	}

	return count;
}

/*These function and Variables are used for GC&VMETA stats*/
#define GC_FC	1
#define VMETA_FC	0
extern int get_gcu_freqs_table(unsigned long *gcu_freqs_table, int *item_counts, int max_item_counts);
extern void update_GC_VMETA_op_cycle(int gvsel, unsigned long new_rate, unsigned int runtime,
				     unsigned int idletime);

struct gc_vmeta_ticks gc_vmeta_ticks_info;
u64 gc_total_ticks;
u64 vm_total_ticks;

extern unsigned int gc_freq_counts;
extern unsigned long gc_cur_freqs_table[GC_VM_OP_NUM_MAX];

unsigned int read_curtime(void)
{
	return OSCR4;
}

static unsigned int ticks_to_sec(unsigned int ticks)
{
	return ticks >> 15;
}

static int pxa_9xx_gc_ticks_read(struct file *file,
				       char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	char buf[1000] = { 0 };
	unsigned int sum = 0, timestamp, time;
	unsigned int result, fraction, shift;

	int i;

	if (gc_vmeta_ticks_info.gc_stats_start || gc_vmeta_ticks_info.gc_stats_stop) {
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "GCU ticks stats\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "OP#   |   OP name   | run ticks | idle ticks | run second | idle second | count\n");

		timestamp = read_curtime();
		time = (timestamp >= gc_vmeta_ticks_info.gc_prev_timestamp) ? timestamp - gc_vmeta_ticks_info.gc_prev_timestamp
			: 0xFFFFFFFF - gc_vmeta_ticks_info.gc_prev_timestamp + timestamp;

		gc_vmeta_ticks_info.gc_prev_timestamp = timestamp;

		if (!gc_vmeta_ticks_info.gc_stats_stop) {
			gc_total_ticks = 0;
			if (gc_vmeta_ticks_info.gc_state == GC_CLK_ON)
				update_GC_VMETA_op_cycle(GC_FC, gc_vmeta_ticks_info.gc_cur_freq, time, 0);
			else
				update_GC_VMETA_op_cycle(GC_FC, gc_vmeta_ticks_info.gc_cur_freq, 0, time);

			for (i = 0; i < gc_freq_counts; i++) {
				gc_total_ticks += (u64)gc_vmeta_ticks_info.GC_op_ticks_array[i].runtime
					+ (u64)gc_vmeta_ticks_info.GC_op_ticks_array[i].idletime;
			}
			if (gc_total_ticks == 0) {
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "No OP change, no duty cycle info\n");
			}
		}
		for (i = 0; i < gc_freq_counts; i++) {
			sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
			"OP %2d | %9luHz |  %8u | %10u | %10u |  %10u | %4u |\n",
			i, gc_vmeta_ticks_info.GC_op_ticks_array[i].op_idx,
			gc_vmeta_ticks_info.GC_op_ticks_array[i].runtime,
			gc_vmeta_ticks_info.GC_op_ticks_array[i].idletime,
			ticks_to_sec(gc_vmeta_ticks_info.GC_op_ticks_array[i].runtime),
			ticks_to_sec(gc_vmeta_ticks_info.GC_op_ticks_array[i].idletime),
			gc_vmeta_ticks_info.GC_op_ticks_array[i].count);
		}
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "GC Duty cycle of operating point list:\n");
		if (gc_total_ticks) {
			result = gc_total_ticks >> 32;
			if (result) {
				shift = fls(result);
				gc_total_ticks >>= shift;
			} else
				shift = 0;
			for (i = 0; i < gc_freq_counts; i++) {
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "OP %2d [%10lu]Hz  ", i, gc_vmeta_ticks_info.GC_op_ticks_array[i].op_idx);

				/* The following algorithm prints a float number
				   with an accuracy of 2 digits after the dot */
				result = (unsigned int)div_u64_rem((((u64)gc_vmeta_ticks_info.GC_op_ticks_array[i].
								runtime) * 100) >> shift,
								(unsigned int)gc_total_ticks,
								&fraction);
				fraction = fraction * 100 / (unsigned int)gc_total_ticks;
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "run:%2u.", result);
				if (fraction < 10)
					sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "0");
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "%u%%   ", fraction);

				result = (unsigned int)div_u64_rem((((u64)gc_vmeta_ticks_info.GC_op_ticks_array[i].
								idletime) * 100)
						>> shift,
						(unsigned int)gc_total_ticks,
						&fraction);
				fraction = fraction * 100 / (unsigned int)gc_total_ticks;
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "idle:%2u.", result);
				if (fraction < 10)
					sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "0");
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "%u%%\n", fraction);
			}
		}
	} else {
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "Please start the GC Ticks stats first\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "Help information :\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    Suppose you mount the debugfs to the /sys/kernel/debug/:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "1. Way to start :\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    echo 1 > /sys/kernel/debug/PM/gcTicks:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "2. Way to check the result:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    cat /sys/kernel/debug/PM/gcTicks:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "3. Way to stop the stats:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    echo 0 > /sys/kernel/debug/PM/gcTicks:\n");
	}

	return simple_read_from_buffer(userbuf, count, ppos, buf, sum);
}

static int pxa_9xx_gc_ticks_write(struct file *file,
					const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	unsigned int value;
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0;
	int min_size, i;

	min_size = min_t(char, sizeof(buf) - 1, count);

	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;

	pos += sscanf(buf, "%d", &value);

	if (1 ==  value) {
		memset(gc_vmeta_ticks_info.GC_op_ticks_array, 0, sizeof(struct gc_vmeta_op_cycle_type) * GC_VM_OP_NUM_MAX);
		if (!gc_freq_counts)
			get_gcu_freqs_table(gc_cur_freqs_table, &gc_freq_counts, ARRAY_SIZE(gc_cur_freqs_table));

		for (i = 0; i < gc_freq_counts; i++)
			gc_vmeta_ticks_info.GC_op_ticks_array[i].op_idx = gc_cur_freqs_table[i];

		gc_vmeta_ticks_info.gc_prev_timestamp = read_curtime();
		gc_vmeta_ticks_info.gc_stats_start = 1;
		gc_vmeta_ticks_info.gc_stats_stop = 0;
		gc_total_ticks = 0;
	} else {
		unsigned int timestamp, time;
		timestamp = read_curtime();
		time = (timestamp >= gc_vmeta_ticks_info.gc_prev_timestamp) ? timestamp - gc_vmeta_ticks_info.gc_prev_timestamp
			: 0xFFFFFFFF - gc_vmeta_ticks_info.gc_prev_timestamp + timestamp;

		gc_vmeta_ticks_info.gc_prev_timestamp = timestamp;

		gc_vmeta_ticks_info.gc_stats_start = 0;
		gc_vmeta_ticks_info.gc_stats_stop = 1;
		if (!gc_vmeta_ticks_info.gc_stats_stop) {
			if (gc_vmeta_ticks_info.gc_state == GC_CLK_ON)
				update_GC_VMETA_op_cycle(GC_FC, gc_vmeta_ticks_info.gc_cur_freq, time, 0);
			else
				update_GC_VMETA_op_cycle(GC_FC, gc_vmeta_ticks_info.gc_cur_freq, 0, time);
		}

		update_GC_VMETA_op_cycle(GC_FC, gc_vmeta_ticks_info.gc_cur_freq, time, 0);
	}
	return count;
}

static int pxa_9xx_vm_ticks_read(struct file *file,
				       char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	char buf[1000] = { 0 };
	unsigned int sum = 0, timestamp, time;
	unsigned int result, fraction, shift;

	int i;

	if (gc_vmeta_ticks_info.vm_stats_start || gc_vmeta_ticks_info.vm_stats_stop) {
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "Vmeta ticks stats\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "OP#   |   OP name   | run ticks | idle ticks | run second | idle second | count\n");

		timestamp = read_curtime();
		time = (timestamp >= gc_vmeta_ticks_info.vm_prev_timestamp) ? (timestamp - gc_vmeta_ticks_info.vm_prev_timestamp)
			: (0xFFFFFFFF - gc_vmeta_ticks_info.vm_prev_timestamp + timestamp);

		gc_vmeta_ticks_info.vm_prev_timestamp = timestamp;

		if (!gc_vmeta_ticks_info.vm_stats_stop) {
			vm_total_ticks = 0;
			if (gc_vmeta_ticks_info.vmeta_state == VMETA_CLK_ON)
				update_GC_VMETA_op_cycle(VMETA_FC, gc_vmeta_ticks_info.vm_cur_freq, time, 0);
			else
				update_GC_VMETA_op_cycle(VMETA_FC, gc_vmeta_ticks_info.vm_cur_freq, 0, time);
			for (i = 0; i < gc_freq_counts; i++) {
				vm_total_ticks += (u64)gc_vmeta_ticks_info.VM_op_ticks_array[i].runtime
					+ (u64)gc_vmeta_ticks_info.VM_op_ticks_array[i].idletime;
			}
			if (vm_total_ticks == 0) {
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "No OP change, no duty cycle info\n");
			}
		}

		for (i = 0; i < gc_freq_counts; i++) {
			sum += snprintf(buf + sum, sizeof(buf) - sum - 1,
			"OP %2d | %9luHz |  %8u | %10u | %10u |  %10u | %4u |\n",
			i, gc_vmeta_ticks_info.VM_op_ticks_array[i].op_idx,
			gc_vmeta_ticks_info.VM_op_ticks_array[i].runtime,
			gc_vmeta_ticks_info.VM_op_ticks_array[i].idletime,
			ticks_to_sec(gc_vmeta_ticks_info.VM_op_ticks_array[i].runtime),
			ticks_to_sec(gc_vmeta_ticks_info.VM_op_ticks_array[i].idletime),
			gc_vmeta_ticks_info.VM_op_ticks_array[i].count);
		}
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "VMeta Duty cycle of operating point list:\n");
		if (vm_total_ticks) {
			result = vm_total_ticks >> 32;
			if (result) {
				shift = fls(result);
				vm_total_ticks >>= shift;
			} else
				shift = 0;
			for (i = 0; i < gc_freq_counts; i++) {
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "OP %2d [%10lu]Hz  ", i, gc_vmeta_ticks_info.VM_op_ticks_array[i].op_idx);

				/* The following algorithm prints a float number
				   with an accuracy of 2 digits after the dot */
				result = (unsigned int)div_u64_rem((((u64)gc_vmeta_ticks_info.VM_op_ticks_array[i].
								runtime) * 100) >> shift,
								(unsigned int)vm_total_ticks,
								&fraction);
				fraction = fraction * 100 / (unsigned int)vm_total_ticks;
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "run:%2u.", result);
				if (fraction < 10)
					sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "0");
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "%u%%   ", fraction);

				result = (unsigned int)div_u64_rem((((u64)gc_vmeta_ticks_info.VM_op_ticks_array[i].
								idletime) * 100) >> shift,
								(unsigned int)vm_total_ticks,
								&fraction);
				fraction = fraction * 100 / (unsigned int)vm_total_ticks;
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "idle:%2u.", result);
				if (fraction < 10)
					sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "0");
				sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "%u%%\n", fraction);
			}
		}
	} else {
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "Please start the VMETA ticks stats first\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "Help information :\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    Suppose you mount the debugfs to the /sys/kernel/debug/:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "1. Way to start :\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    echo 1 > /sys/kernel/debug/PM/VmetaTicks:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "2. Way to check the result:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    cat /sys/kernel/debug/PM/VmetaTicks:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "3. Way to stop the stats:\n");
		sum += snprintf(buf + sum, sizeof(buf) - sum - 1, "    echo 0 > /sys/kernel/debug/PM/VmetaTicks:\n");
	}

	return simple_read_from_buffer(userbuf, count, ppos, buf, sum);
}

static int pxa_9xx_vm_ticks_write(struct file *file,
					const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	unsigned int value;
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0;
	int min_size, i;

	min_size = min_t(char, sizeof(buf) - 1, count);

	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;

	pos += sscanf(buf, "%d", &value);

	if (1 ==  value) {
		memset(gc_vmeta_ticks_info.VM_op_ticks_array, 0, sizeof(struct gc_vmeta_op_cycle_type) * GC_VM_OP_NUM_MAX);
		if (!gc_freq_counts)
			get_gcu_freqs_table(gc_cur_freqs_table, &gc_freq_counts, ARRAY_SIZE(gc_cur_freqs_table));

		for (i = 0; i < gc_freq_counts; i++)
			gc_vmeta_ticks_info.VM_op_ticks_array[i].op_idx = gc_cur_freqs_table[i];

		gc_vmeta_ticks_info.vm_prev_timestamp = read_curtime();
		gc_vmeta_ticks_info.vm_stats_start = 1;
		gc_vmeta_ticks_info.vm_stats_stop = 0;
		vm_total_ticks = 0;
	} else {
		unsigned int timestamp, time;
		timestamp = read_curtime();
		time = (timestamp >= gc_vmeta_ticks_info.vm_prev_timestamp) ? timestamp - gc_vmeta_ticks_info.vm_prev_timestamp
			: 0xFFFFFFFF - gc_vmeta_ticks_info.vm_prev_timestamp + timestamp;

		gc_vmeta_ticks_info.vm_prev_timestamp = timestamp;

		gc_vmeta_ticks_info.vm_stats_start = 0;
		gc_vmeta_ticks_info.vm_stats_stop = 1;
		update_GC_VMETA_op_cycle(VMETA_FC, gc_vmeta_ticks_info.vm_cur_freq, time, 0);
	}
	return count;
}

void gc_vmeta_stats_clk_event(enum stats_clk_event event)
{
	switch (event) {

		case GC_CLK_ON:
			/* if GC clock was not already on,
			   save start timestamp */
			if (gc_vmeta_stats.gc_is_on != ON) {
				gc_vmeta_stats.gc_is_on = ON;
				gc_vmeta_stats.gc_start_time = OSCR4;
			}
			break;

		case GC_CLK_OFF:
			/* if GC clock was not already off,
			   save stop timestamp and sum-up active time */
			if (gc_vmeta_stats.gc_is_on != OFF) {
				gc_vmeta_stats.gc_is_on = OFF;
				gc_vmeta_stats.gc_stop_time = OSCR4;
				gc_vmeta_stats.gc_total_time +=
					gc_vmeta_stats.gc_stop_time -
					gc_vmeta_stats.gc_start_time;
			}
			break;

		case VMETA_CLK_ON:
			/* if VMETA clock was not already on,
			   save start timestamp */
			if (gc_vmeta_stats.vmeta_is_on != ON) {
				gc_vmeta_stats.vmeta_is_on = ON;
				gc_vmeta_stats.vmeta_start_time = OSCR4;
			}
			break;

		case VMETA_CLK_OFF:
			/* if VMETA clock was not already off,
			   save stop timestamp and sum-up active time */
			if (gc_vmeta_stats.vmeta_is_on != OFF) {
				gc_vmeta_stats.vmeta_is_on = OFF;
				gc_vmeta_stats.vmeta_stop_time = OSCR4;
				gc_vmeta_stats.vmeta_total_time +=
					gc_vmeta_stats.vmeta_stop_time -
					gc_vmeta_stats.vmeta_start_time;
			}
			break;
	}
}

static ssize_t PXA9xx_DVFM_profilerRecommendation_seq_read(struct file *file,
							   char __user *userbuf,
							   size_t count,
							   loff_t *ppos)
{
	/*Consider buff size when modifiy this function */
	char buf[512] = { 0 };

	int ret, sum;

	/*Safe version of sprintf of that doesn't suffer from buffer overruns */
	ret = snprintf(buf, sizeof(buf) - 1,
		       "type [Active 0/1],[ppIndex to recommend]\n");
	if (-1 == ret)
		return ret;
	sum = ret;

	ret = snprintf(buf + sum, sizeof(buf) - sum - 1,
		       "profilerRecommendationEnable = %d\n"
		       "profilerRecommendationPP = %d\n\n",
		       profilerRecommendationEnable, profilerRecommendationPP);
	if (-1 == ret)
		return ret;
	sum += ret;
	return simple_read_from_buffer(userbuf, count, ppos, buf, sum);
}

static int PXA9xx_DVFM_profilerRecommendation_write(struct file *file,
						    const char __user *ubuf,
						    size_t count, loff_t *ppos)
{
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0;
	int min_size;

	min_size = min_t(char, sizeof(buf) - 1, count);

	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;

	pos += sscanf(buf, "%d,%d", &profilerRecommendationEnable,
		      &profilerRecommendationPP);
	return count;
}

static int MeasureCoreFreq(struct seq_file *seq, void *unused)
{
	unsigned long start32kHz, startCore = 0, endCore = 0xffffffff;
	unsigned long flags, cycleEnable;
	unsigned long end32Khz;

	local_irq_save(flags);

	/*
	   The correct form to use single_open with printouts is with seq_printf
	   function, and not printk, however,   in this case all the printouts
	   will appear togther on the returning to user-space, which not
	   satisfies MeasureCoreFreq printouts.
	 */
	printk(KERN_WARNING "\nMake sure the core is running on single PP!\n"
	       "Starting...\n");

	/* The folowing commands are to make sure the CCNT counter is enable */

	/* Read count enable set */
	__asm__ __volatile__("mrc p15, 0, %0, c9, c12, 1;\n" :
			     "=r"(cycleEnable) : );
	cycleEnable |= 0x80000000;	/* Enable cycle counter */
	/* write to Count enable set */
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 1;\n" : :
			     "r"(cycleEnable));
	/* Read Performance Monitor Control */
	__asm__ __volatile__("mrc p15, 0, %0, c9, c12, 0;\n" :
			     "=r"(cycleEnable) : );
	cycleEnable |= 0x9;	/* Enable all counters and 64 prescaler */
	/* write Performance Monitor Control */
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 0;\n" : :
			     "r"(cycleEnable));

	/* Read oscr4 (32.768KHz) */
	start32kHz = __raw_readl((void *)&(__REG(0x40A00040)));

	/* Read cycle counter */
	__asm__ __volatile__("mrc p15,0,%0,c9,c13,0;\n" : "=r"(startCore) : );

	do {
		end32Khz = __raw_readl((void *)&(__REG(0x40A00040)));
		/* 1Sec = 32768 = 0x8000 counts for this counter */
	} while ((end32Khz - start32kHz) < 0x8000);
	/* Read cycle counter again */
	__asm__ __volatile__("mrc p15,0,%0,c9,c13,0;\n" : "=r"(endCore) : );

	printk(KERN_WARNING "Done.\n\nCore frequency = %lu MHz\n",
	       ((endCore - startCore) * 64 / 1000000) + 1);

	local_irq_restore(flags);
	return 0;
}

static int pxa_9xx_power_MeasureCoreFreq_open(struct inode *inode,
					      struct file *file)
{
	return single_open(file, MeasureCoreFreq, NULL);
}

static int start_status = PM_LOGGER_STOP;
static struct pm_logger_buffer_descriptor  pm_logger_buffer;

/*
 * Open - copy all data to be dumped to buffer.
 */
static int PXA9xx_pm_logger_buffer_open(struct inode *inode,
							struct file *file)
{
	size_t size = 0, tmp_size;
	char *dst_buff, *logger_buff, *tmp_buff;
	struct dvfm_trace_info *dev_ptr;
	unsigned char ***database;
	int event, i;
	unsigned int magic;

	/* stop logger before dumping data */
	start_status = get_pm_logger_app_status();
	pm_logger_app_stop();

	/* calculate buffer size */
	/* logger size */
	tmp_size =  get_pm_logger_app_buffSize()*sizeof(unsigned int);
	size += tmp_size;
	pm_logger_buffer.header.buffer_size = tmp_size;

	/* event database size */
	database = get_pm_logger_app_db();
	tmp_size = sizeof(unsigned int);
	for (event = 0; event < PM_EVENTS_NUM; event++) {
		tmp_size += sizeof(unsigned int);
		for (i = 0; i < MAX_DATA_NUM; i++) {
			char *dbstr = GET_DB_STRING(database, event, i);
			if (NULL != dbstr)
				tmp_size += strlen(dbstr);
			tmp_size += sizeof(unsigned char);
		}
	}
	size += tmp_size;
	pm_logger_buffer.header.app_table_size = tmp_size - 4;

	/* op table size */
	tmp_size = sizeof(unsigned int);
	tmp_size += ((OP_NAME_LEN +
			sizeof(unsigned int)) * proc_op->nr_op);
	size += tmp_size;
	pm_logger_buffer.header.op_table_size = tmp_size - 4;

	/* dvfm trace list */
	tmp_size = sizeof(unsigned int);
	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(dev_ptr, &dvfm_trace_list.list,	list) {
		if (dev_ptr != NULL) {
			tmp_size += sizeof(unsigned int); /* row magic */
			tmp_size += sizeof(unsigned char) + DVFM_MAX_NAME;
					/* index + DVFM_MAX_NAME */
		}
	}
	read_unlock(&dvfm_trace_list.lock);
	size += tmp_size;
	pm_logger_buffer.header.dvfm_table_size = tmp_size - 4;

	/* allocate buffer and copy data to it */
	dst_buff = kzalloc(size, GFP_KERNEL);
	if (dst_buff == ZERO_SIZE_PTR)
		return -1;

	tmp_buff = dst_buff;

	/* logger data */
	logger_buff = (char *)get_pm_logger_app_buffer();
	memcpy(tmp_buff, logger_buff, pm_logger_buffer.header.buffer_size);
	tmp_buff += pm_logger_buffer.header.buffer_size;

	/* event database data */
	magic = htonl(EVENT_DB_HEADER_MAGIC);
	memcpy(tmp_buff, &magic, sizeof(unsigned int));
	tmp_buff += sizeof(unsigned int);
	for (event = 0; event < PM_EVENTS_NUM; event++) {
		magic = htonl(EVENT_DB_ROW_MAGIC);
		memcpy(tmp_buff, &magic, sizeof(unsigned int));
		tmp_buff += sizeof(unsigned int);
		for (i = 0; i < MAX_DATA_NUM; i++) {
			char *dbstr = GET_DB_STRING(database, event, i);
			if (NULL != dbstr) {
				size_t len = strlen(dbstr);
				memcpy(tmp_buff, dbstr, len);
				tmp_buff += len;
			}
			*tmp_buff++ = 0;
		}
	}

	/* op table data */
	magic = htonl(OP_TABLE_HEADER_MAGIC);
	memcpy(tmp_buff, &magic, sizeof(unsigned int));
	tmp_buff += sizeof(unsigned int);
	for (i = 0; i < proc_op->nr_op; i++) {
		magic = htonl(OP_TABLE_ROW_MAGIC);
		memcpy(tmp_buff, &magic, sizeof(unsigned int));
		tmp_buff += sizeof(unsigned int);
		memcpy(tmp_buff, &proc_op->op_array[i].name,
				OP_NAME_LEN);
		tmp_buff += OP_NAME_LEN;
	}

	/* dvfm trace list data */
	magic = htonl(DEV_TABLE_HEADER_MAGIC);
	memcpy(tmp_buff, &magic, sizeof(unsigned int));
	tmp_buff += sizeof(unsigned int);
	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(dev_ptr, &dvfm_trace_list.list, list) {
		if (dev_ptr != NULL) {
			magic = htonl(DEV_TABLE_ROW_MAGIC);
			memcpy(tmp_buff, &magic, sizeof(unsigned int));
			tmp_buff += sizeof(unsigned int);
			*tmp_buff++ = (unsigned char)dev_ptr->index;
			memcpy(tmp_buff, dev_ptr->name, DVFM_MAX_NAME);
			tmp_buff += DVFM_MAX_NAME;
		}
	}
	read_unlock(&dvfm_trace_list.lock);

	pm_logger_buffer.buffer = dst_buff;
	pm_logger_buffer.len = size;
	file->private_data = &pm_logger_buffer;
	return 0;
}

/*
 * Read - hex dump buffer.
 * Format is: "PM01234 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF\n"
 */
static ssize_t PXA9xx_pm_logger_buffer_read(struct file *file,
							char __user *userbuf,
							size_t count,
							loff_t *ppos)
{
	ssize_t ret;
	int pos = 0, ofs = 0, buf_size = 0;
	struct pm_logger_buffer_descriptor *desc = file->private_data;
	const u8 *ptr;
	char *buf;
	size_t pm_buff_len = desc->len;

	/* allocate size according to following calc:
	 * 16 bytes per line in dump
	 * 3 bytes per each + foreach line PM+offset+\n +
	 * 16 bytes to protect allocation from misaligned copy */
	buf_size = (3 * pm_buff_len) + ((pm_buff_len/16) * (2+5+1)) + 16;
	ptr = desc->buffer;
	if (!ptr) {
		printk(KERN_WARNING "Invalid Buffer\n");
		return -ENOMEM;
	}
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (!buf) {
		printk(KERN_WARNING "Can not allocate Buffer\n");
		return -ENOMEM;
	}
	pos += scnprintf(buf + pos, buf_size - pos, "\nPM Logger dump\n");

	/* dump each line */
	for (ofs = 0 ; ofs < pm_buff_len ; ofs += 16) {
		pos += scnprintf(buf + pos, buf_size - pos, "PM%05x ", ofs);
		hex_dump_to_buffer(ptr + ofs, 16 , 16, 1, buf + pos,
			buf_size - pos, 0);
		pos += strlen(buf + pos);
		if (buf_size - pos > 0)
			buf[pos++] = '\n';
	}

	ret = simple_read_from_buffer(userbuf, count, ppos, buf, pos);
	kfree(buf);
	return ret;
}

/*
 * Release - restart logger if needed, release memory
 */
static int PXA9xx_pm_logger_buffer_release(struct inode *inode,
							struct file *file)
{
	struct pm_logger_buffer_descriptor *desc;

	if (PM_LOGGER_START == start_status) {
		pm_logger_app_start();
		start_status = PM_LOGGER_STOP;
	}

	desc  = file->private_data;
	kfree(desc->buffer);

	return 0;
}

/********************************************************/

#define UINT8_T char

struct I2C_BUS {
	UINT8_T scl;		/* i2c_clk  */
	UINT8_T sda;		/* i2c_data */
};

static struct I2C_BUS I2cBus;

/*************************************
	Start bit sequence is as followed:
			____
	SDA	:	    |_______
			_______
	SCL	:	       |____

************************************/

static void I2CEmulatorSendStartBit(void)
{
	__gpio_set_value(mfp_to_gpio(I2cBus.sda), 1);
	udelay(200);
	__gpio_set_value(mfp_to_gpio(I2cBus.scl), 1);
	udelay(200);
	__gpio_set_value(mfp_to_gpio(I2cBus.sda), 0);
	udelay(300);
	__gpio_set_value(mfp_to_gpio(I2cBus.scl), 0);
}

/************************************
	Stop bit sequence is as followed:
				___
	SDA	:	___|
			  _____
	SCL	: ___|

************************************/

static void I2CEmulatorSendStopBit(void)
{
	__gpio_set_value(mfp_to_gpio(I2cBus.sda), 0);
	udelay(200);
	__gpio_set_value(mfp_to_gpio(I2cBus.scl), 1);
	udelay(200);
	__gpio_set_value(mfp_to_gpio(I2cBus.sda), 1);
	udelay(300);
}

/* Emulate single byte send via i2c bus */

static void I2CEmulatorSendByte(UINT8_T Data)
{
	UINT8_T bit;

	for (bit = 0; bit < 8; bit++) {

		/* Raise or fall accrording to the bit */
		if (Data & 0x80)
			__gpio_set_value(mfp_to_gpio(I2cBus.sda), 1);
		else
			__gpio_set_value(mfp_to_gpio(I2cBus.sda), 0);

		Data = (Data << 1);
		udelay(100);

		/* Single clock tick */
		__gpio_set_value(mfp_to_gpio(I2cBus.scl), 1);
		udelay(200);
		__gpio_set_value(mfp_to_gpio(I2cBus.scl), 0);
	}

}

static int I2CEmulatorRecieveByte(void)
{
	UINT8_T bit;
	int RetVal = 0, val = 0, ret;

	/* Switch gpio to input in order to recieve byte */
	gpio_direction_input(I2cBus.sda);

	for (bit = 0; bit < 8; bit++) {

		udelay(100);
		__gpio_set_value(mfp_to_gpio(I2cBus.scl), 1);
		udelay(100);

		/* Get gpio level (bit value) */
		val = __gpio_get_value(I2cBus.sda);
		RetVal = RetVal << 1;
		if (val)
			RetVal |= 1;
		udelay(100);
		__gpio_set_value(mfp_to_gpio(I2cBus.scl), 0);
	}

	/* Switch gpio back to output */
	ret = gpio_direction_output(mfp_to_gpio(I2cBus.sda), 1);

	return RetVal;
}

static void I2CEmulatorGetAck(void)
{
	int ret;

	/* Switch gpio to input in order to recieve ack */
	gpio_direction_input(I2cBus.sda);
	udelay(200);

	/* Single clock tick */
	__gpio_set_value(mfp_to_gpio(I2cBus.scl), 1);
	udelay(200);
	__gpio_set_value(mfp_to_gpio(I2cBus.scl), 0);
	udelay(200);

	/* Switch gpio back to output */
	ret = gpio_direction_output(mfp_to_gpio(I2cBus.sda), 1);
}

static void I2CEmulatorSendAck(int ack)
{
	/* Raise Data-line */
	__gpio_set_value(mfp_to_gpio(I2cBus.sda), ack);
	udelay(200);

	/* Single clock tick */
	__gpio_set_value(mfp_to_gpio(I2cBus.scl), 1);
	udelay(200);
	__gpio_set_value(mfp_to_gpio(I2cBus.scl), 0);
}

static void I2CEmulatorRead(int slaveAddress, int regAddress)
{
	int val;
	unsigned int ret;

	ret = gpio_direction_output(mfp_to_gpio(I2cBus.scl), 1);
	ret = gpio_direction_output(mfp_to_gpio(I2cBus.sda), 1);

	/* Start bit */
	I2CEmulatorSendStartBit();
	udelay(400);
	/* Send slave address (WRITE) */
	I2CEmulatorSendByte(slaveAddress & 0xFE);
	udelay(400);
	/* Receive ack */
	I2CEmulatorGetAck();
	udelay(400);
	/* Send register address */
	I2CEmulatorSendByte(regAddress & 0xFF);
	udelay(400);
	/* Receive ack */
	I2CEmulatorGetAck();
	udelay(400);
	/* Re-Start bit */
	I2CEmulatorSendStartBit();
	udelay(400);
	/* Send slave address (READ) */
	I2CEmulatorSendByte(slaveAddress | 0x1);
	udelay(400);
	/* Receive ack */
	I2CEmulatorGetAck();
	udelay(400);
	/* Receive data byte */
	val = I2CEmulatorRecieveByte();
	udelay(400);
	/* Send ack */
	I2CEmulatorSendAck(1);
	udelay(400);
	/* Stop bit */
	I2CEmulatorSendStopBit();

	printk(KERN_INFO "\nValue read from "
	       "[slaveAddress 0x%x, register 0x%x] = 0x%x\n",
	       slaveAddress, regAddress, val);
}

static void I2CEmulatorWrite(int slaveAddress, int regAddress, int value)
{
	unsigned int ret;

	ret = gpio_direction_output(mfp_to_gpio(I2cBus.scl), 1);
	ret = gpio_direction_output(mfp_to_gpio(I2cBus.sda), 1);

	/* Start bit */
	I2CEmulatorSendStartBit();
	udelay(400);
	/* Send slave address (WRITE) */
	I2CEmulatorSendByte(slaveAddress & 0xFE);
	udelay(400);
	/* Receive ack */
	I2CEmulatorGetAck();
	udelay(400);
	/* Send register address */
	I2CEmulatorSendByte(regAddress & 0xFF);
	udelay(400);
	/* Receive ack */
	I2CEmulatorGetAck();
	udelay(400);
	/* Send data value */
	I2CEmulatorSendByte(value);
	udelay(400);
	/* Receive ack */
	I2CEmulatorGetAck();
	/* Stop bit */
	I2CEmulatorSendStopBit();

	printk(KERN_INFO "\nValue 0x%x written to"
	       "[slaveAddress 0x%x, register 0x%x]\n",
	       value, slaveAddress, regAddress);
}

static ssize_t PXA9xx_PI2C_read(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[30] = { 0 };
	int sum = 0;

	printk(KERN_INFO
	       "PI2C USAGE:\n"
	       "[r/w] [slaveAddress-hex], [regAddres-hex] <data-hex>\n"
	       "[s] [scl-GPIO-dec], [sda-GPIO-dec]\n");

	if (I2cBus.scl == 0 && I2cBus.sda == 0)
		printk(KERN_INFO
		       "Current setup: N/A, "
		       "please configure GPIOs for scl & sda\n");
	else
		printk(KERN_INFO
		       "Current setup: scl->GPIO%d, sda->GPIO%d\n",
		       I2cBus.scl, I2cBus.sda);

	return simple_read_from_buffer(userbuf, count, ppos, buf, sum);
}

static ssize_t PXA9xx_PI2C_write(struct file *file,
				 const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	char buf[USER_BUF_SIZE] = { 0 };
	int pos = 0;
	int min_size;
	char ch;
	unsigned long flags;
	int slaveAddress, regAddress, value, scl, sda;
	unsigned int ret;
	mfp_cfg_t mfp_c;

	/* copy user's input to kernel space */
	min_size = min_t(char, sizeof(buf) - 1, count);
	if (copy_from_user(buf, ubuf, min_size))
		return -EFAULT;

	local_irq_save(flags);
	pos += sscanf(buf, "%c", &ch);

	/* Setup mode, choose GPIOs */
	if (ch == 's') {
		pos += sscanf(buf + pos, "%d, %d", &scl, &sda);

		I2cBus.scl = scl & 0xFF;
		I2cBus.sda = sda & 0xFF;

		mfp_c = I2cBus.scl | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret = gpio_request(mfp_to_gpio(I2cBus.scl), "PI2C-scl");

		mfp_c = I2cBus.sda | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret = gpio_request(mfp_to_gpio(I2cBus.sda), "PI2C-sda");

		printk(KERN_INFO
		       "\nConfigure scl->GPIO%d, sda->GPIO%d\n",
		       I2cBus.scl, I2cBus.sda);

	}

	/* Read mode */
	if (ch == 'r') {
		if (I2cBus.scl == 0 && I2cBus.sda == 0)
			printk(KERN_INFO
			       "\nERROR: Please define GPIOs for scl & sda\n");
		else {
			pos += sscanf(buf + pos, "%x, %x",
				      &slaveAddress, &regAddress);
			I2CEmulatorRead(slaveAddress, regAddress);
		}
	}

	/* Write mode */
	if (ch == 'w') {
		if (I2cBus.scl == 0 && I2cBus.sda == 0)
			printk(KERN_INFO
			       "\nERROR: Please define GPIOs for scl & sda\n");
		else {
			pos += sscanf(buf + pos, "%x, %x, %x",
				      &slaveAddress, &regAddress, &value);
			I2CEmulatorWrite(slaveAddress, regAddress, value);
		}
	}

	local_irq_restore(flags);

	return count;
}

/*************************************************************************/

void pxa_9xx_power_init_debugfs(void)
{
	int errRet = 0;
	dbgfs_root = debugfs_create_dir(PXA9XX__POWER_DEBUG_NAME, NULL);
	if (!(IS_ERR(dbgfs_root) || !dbgfs_root)) {

		pmLogger_file = debugfs_create_file("pmLogger", 0600,
						    dbgfs_root, NULL,
						    &PXA9xx_file_op_pmLogger);
		if (!pmLogger_file)
			errRet = -EINVAL;

		forceVCTCXO_EN_file = debugfs_create_file("forceVCTCXO_EN",
							  0600, dbgfs_root,
							  NULL,
							  &PXA9xx_file_force_VCTCXO_EN);
		if (!forceVCTCXO_EN_file)
			errRet = -EINVAL;

		ForceLPM_file = debugfs_create_file("ForceLPM", 0600,
						    dbgfs_root, NULL,
						    &PXA9xx_file_op_ForceLPM);

		if (!ForceLPM_file)
			errRet = -EINVAL;

		MeasureCoreFreq_file = debugfs_create_file("MeasureCoreFreq",
							   0400, dbgfs_root,
							   NULL,
							   &PXA9xx_file_op_MeasureCoreFreq);
		if (!MeasureCoreFreq_file)
			errRet = -EINVAL;

		PI2C_file = debugfs_create_file("PI2C", 0600,
						dbgfs_root, NULL,
						&PXA9xx_file_PI2C);
		if (!PI2C_file)
			errRet = -EINVAL;

		ForceC0_file = debugfs_create_file("ForceC0", 0600,
						dbgfs_root, NULL,
						&PXA9xx_file_force_C0);
		if (!ForceC0_file)
			errRet = -EINVAL;

		profilerRecommendation_file =
		    debugfs_create_file("profilerRecommendation", 0600,
					dbgfs_root, NULL,
					&PXA9xx_file_op_profilerRecommendation);

		if (!profilerRecommendation_file)
			errRet = -EINVAL;

		GcTicks_file = debugfs_create_file("gcTicks", 0600,
						   dbgfs_root, NULL,
						   &PXA9xx_file_op_gc_ticks);
		if (!GcTicks_file)
			errRet = -EINVAL;

		VmTicks_file = debugfs_create_file("VmetaTicks", 0600,
						   dbgfs_root, NULL,
						   &PXA9xx_file_op_vm_ticks);
		if (!VmTicks_file)
			errRet = -EINVAL;

		GcVmetaStats_file = debugfs_create_file("GcVmetaStats", 0600,
							dbgfs_root, NULL,
							&PXA9xx_file_op_gc_vmeta_stats);
		if (!GcVmetaStats_file)
			errRet = -EINVAL;

		pmLoggerBuffer_file = debugfs_create_file("pmLoggerData",
						0400, dbgfs_root,
						&pm_logger_buffer,
						&PXA9xx_file_buffer_pmLogger);
		if (!pmLoggerBuffer_file)
			errRet = -EINVAL;

		if (errRet) {
			debugfs_remove_recursive(dbgfs_root);
			printk(KERN_ERR "%s Failed\n", __func__);
		}
	} else
		pr_err("pxa9xx_power: debugfs is not available\n");
	return;
}

void __exit pxa_9xx_power_cleanup_debugfs(void)
{
	debugfs_remove_recursive(dbgfs_root);
}
#else
inline void pxa_9xx_power_init_debugfs(void)
{
}

inline void pxa_9xx_power_cleanup_debugfs(void)
{
}

inline void gc_vmeta_stats_clk_event(enum stats_clk_event event)
{
}

#endif
