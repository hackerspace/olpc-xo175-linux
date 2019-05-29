#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <asm/cputime.h>

#ifdef CONFIG_HOTPLUG_CPU

struct cpu_hotplug_tuners;
struct hotplug_attr {
	struct attribute attr;
	ssize_t (*show)(struct cpu_hotplug_tuners *, char *);
	ssize_t (*store)(struct cpu_hotplug_tuners *, const char *, size_t count);
};

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct cpu_hotplug_tuners *tuners, char *buf)				\
{									\
	return sprintf(buf, "%u\n", tuners->object);			\
}

#define store_one(file_name, object)					\
static ssize_t store_##file_name					\
(struct cpu_hotplug_tuners *tuners, const char *buf, size_t count)	\
{									\
	unsigned int input;						\
	int ret;							\
									\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	tuners->object = input;						\
	return count;							\
}

#define define_hotplug_governor_attr_rw(_name)				\
store_one(_name, _name)							\
show_one(_name, _name)							\
static struct hotplug_attr _name =					\
__ATTR(_name, 0644, show_##_name, store_##_name)			\


#define DEFAULT_SAMPLING_PERIOD				(100000)

#define DEFAULT_SYSTEM_UP_THRESHOLD			(80)
#define DEFAULT_SYSTEM_DOWN_THRESHOLD			(30)
#define DEFAULT_IDLE_THRESHOLD				(5)
#define DEFAULT_SYSTEM_DOWN_PERIODS_THRESHOLD		(50)
#define DEFAULT_IDLE_PERIODS_THRESHOLD			(50)

#define DEFAULT_LOAD_RATIO				(50)

#define CPU_HOTPLUG_RUN_MASK		(1 << 3)

enum {
	CPU_HOTPLUG_STOP_HIGH	= 1,
	CPU_HOTPLUG_STOP_LOW	= 2,
	CPU_HOTPLUG_RUN_LV1	= 8,
	CPU_HOTPLUG_RUN_LV2	= 9,
	CPU_HOTPLUG_RUN_LV3	= 10,
};

struct cpu_hotplug_tuners {
	struct kobject kobj;
	struct delayed_work work;
	struct workqueue_struct *workq;
	unsigned int sampling_rate;
	/* threshold for while system*/
	unsigned int system_up_threshold;
	unsigned int system_down_threshold;
	/* threshold for single core, no up threshold, because cpufreq will take it*/
	unsigned int system_down_periods;
	unsigned int system_down_periods_threshold;
	/* for one core long idle, we can hotplug it*/
	unsigned int idle_threshold;
	unsigned int idle_periods_threshold;

	unsigned int load_ratio;
	unsigned int avg_load;

	unsigned int barrier_freq[NR_CPUS];
	/* struct mutex tuners_mutex; */
	int state;
	int state_limit;
	int boot_cores;
/*#define LOCK_OWNER_NONE		0
#define LOCK_OWNER_CPUFREQ	1
#define LOCK_OWNER_HOTPLUG	2
	atomic_t lock;*/
	spinlock_t lock;
	u64 periods;
};

struct cpu_hotplug_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	unsigned int idle_periods;
	struct cpu_hotplug_tuners *tuners;
};

static DEFINE_PER_CPU(struct cpu_hotplug_info_s, cpu_hotplug_info);

#define OP_NONE		0
#define OP_OFFLINE_CPU	1
#define OP_ONLINE_CPU	2

static int cpu_hotplug_check_cpu(struct cpu_hotplug_tuners *tuners)
{
	unsigned int total_load = 0;
	unsigned int avg_load = 0;
	int j, op_cpu = 0, op = OP_NONE;
	int idle_cpu = -1;
	int least_load_cpu = 0;
	int least_load = 100;
	struct cpu_hotplug_info_s *info;

	/*if (atomic_cmpxchg(&tuners->lock, LOCK_OWNER_NONE, LOCK_OWNER_HOTPLUG) != LOCK_OWNER_NONE)
		return -EAGAIN;*/
	spin_lock(&tuners->lock);
	if ((!(tuners->state & CPU_HOTPLUG_RUN_MASK))) {
		spin_unlock(&tuners->lock);
		/* atomic_set(&tuners->lock, LOCK_OWNER_NONE); */
		return -EPERM;
	}
	spin_unlock(&tuners->lock);

	for_each_online_cpu(j) {
		unsigned int load;
		unsigned int idle_time, wall_time;
		cputime64_t cur_wall_time, cur_idle_time;

		info = &per_cpu(cpu_hotplug_info, j);

		/* update both cur_idle_time and cur_wall_time */
		cur_idle_time = get_cpu_idle_time_us(j, &cur_wall_time);

		/* how much wall time has passed since last iteration? */
		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				info->prev_cpu_wall);
		info->prev_cpu_wall = cur_wall_time;

		/* how much idle time has passed since last iteration? */
		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				info->prev_cpu_idle);
		info->prev_cpu_idle = cur_idle_time;

		if (unlikely(!wall_time))
			continue;

		if (wall_time < idle_time)
			wall_time = idle_time;

		/* load is the percentage of time not spent in idle */
		load = 100 * (wall_time - idle_time) / wall_time;
		/* info->cpu_load[tuners->hotplug_load_index] = load; */
		if (load < tuners->idle_threshold)
			info->idle_periods++;
		else
			info->idle_periods = 0;
		if (info->idle_periods > tuners->idle_periods_threshold)
			idle_cpu = j;

		if (j != 0 && load < least_load) {
			least_load_cpu = j;
			least_load = load;
		}

		/* printk("cpu%d, load %d, idle_p %d\n", j, load, info->idle_periods); */
		/* keep track of combined load across all CPUs */
		total_load += load;
		/* printk("sample date for cpu %d, load %d\n", j, load); */
	}

	/* calculate the average load across all related CPUs */
	avg_load = total_load / num_online_cpus();

	if (tuners->periods == 0)
		tuners->avg_load = avg_load;
	else
		tuners->avg_load = (avg_load * tuners->load_ratio + tuners->avg_load * (100 - tuners->load_ratio)) / 100;

	tuners->periods++;
	/* printk("periods %lld, avg_laod %d, taotal %d, periods %d\n", tuners->periods, avg_load, tuners->avg_load, tuners->system_down_periods); */
	if (avg_load > tuners->system_up_threshold) {
		if (num_online_cpus() < tuners->boot_cores) {
			for_each_present_cpu(j) {
				if (!cpu_online(j)) {
					op_cpu = j;
					printk("cpu%d up\n", op_cpu);
					op = OP_ONLINE_CPU;
					goto do_op;
				}
			}
		}
	}

	if (tuners->avg_load < tuners->system_down_threshold)
		tuners->system_down_periods++;
	else
		tuners->system_down_periods = 0;

	if (tuners->system_down_periods > tuners->system_down_periods_threshold) {
		op_cpu = least_load_cpu;
		if (op_cpu != 0) {
			printk("cpu%d down\n", op_cpu);
			op = OP_OFFLINE_CPU;
			goto do_op;
		}
	}

	if (idle_cpu >= 0
		&& (avg_load * num_online_cpus() < tuners->system_up_threshold * (num_online_cpus() - 1))) {
		if (idle_cpu != 0) {
			op_cpu = idle_cpu;
			printk("idle cpu%d down\n", op_cpu);
			op = OP_OFFLINE_CPU;
			goto do_op;
		}
		else if (num_online_cpus() > 1) {
			op_cpu = least_load_cpu;
			if (op_cpu != 0) {
				printk("idle cpu%d down\n", op_cpu);
				op = OP_OFFLINE_CPU;
				goto do_op;
			}
		}
	}

do_op:
	if (op == OP_ONLINE_CPU || op == OP_OFFLINE_CPU) {
		spin_lock(&tuners->lock);
		if ((!(tuners->state & CPU_HOTPLUG_RUN_MASK))) {
			spin_unlock(&tuners->lock);
			return -EPERM;
		}
		else if (tuners->state == tuners->state_limit && op == OP_OFFLINE_CPU) {
			spin_unlock(&tuners->lock);
			goto out;
		}
		spin_unlock(&tuners->lock);

		if (op == OP_ONLINE_CPU)
			cpu_up(op_cpu);
		else
			cpu_down(op_cpu);
		tuners->system_down_periods = 0;
		tuners->periods = 0;
		for_each_online_cpu(j) {
			info = &per_cpu(cpu_hotplug_info, j);
			info->prev_cpu_idle = get_cpu_idle_time_us(j, &info->prev_cpu_wall);
			info->idle_periods = 0;
		}

		spin_lock(&tuners->lock);
		if ((!(tuners->state & CPU_HOTPLUG_RUN_MASK))) {
			spin_unlock(&tuners->lock);
			return -EPERM;
		}
		if (num_online_cpus() == 1)
			tuners->state = CPU_HOTPLUG_RUN_LV3;
		else if (num_online_cpus() == 2)
			tuners->state = CPU_HOTPLUG_RUN_LV2;
		else if (num_online_cpus() == 3)
			tuners->state = CPU_HOTPLUG_RUN_LV1;
		spin_unlock(&tuners->lock);
	}
	/* atomic_cmpxchg(&tuners->lock, LOCK_OWNER_HOTPLUG, LOCK_OWNER_NONE); */
out:
	return 0;
}


static void do_cpu_hotplug_timer(struct work_struct *work)
{
	struct cpu_hotplug_tuners *tuners =
		container_of(work, struct cpu_hotplug_tuners, work.work);
	int delay = usecs_to_jiffies(tuners->sampling_rate);
	int ret;

	delay -= jiffies % delay;

	ret = cpu_hotplug_check_cpu(tuners);
	if (ret == 0 || ret == -EAGAIN)
		queue_delayed_work_on(0, tuners->workq, &tuners->work, delay);
	/*
	else
		printk("stop profiling-----------------------------\n");
	*/
}

void hotplug_governor_start_profiling(struct cpu_hotplug_tuners *tuners)
{
	struct cpu_hotplug_info_s *info;
	unsigned int delay;
	int cpu;

	spin_lock(&tuners->lock);
	if (num_online_cpus() == 1)
		tuners->state = CPU_HOTPLUG_RUN_LV3;
	else if (num_online_cpus() == 2)
		tuners->state = CPU_HOTPLUG_RUN_LV2;
	else if (num_online_cpus() == 3)
		tuners->state = CPU_HOTPLUG_RUN_LV1;
	spin_unlock(&tuners->lock);

	delay = usecs_to_jiffies(tuners->sampling_rate);

	delay -= jiffies % delay;

	tuners->system_down_periods = 0;
	tuners->periods = 0;

	for_each_online_cpu(cpu) {
		info = &per_cpu(cpu_hotplug_info, cpu);
		info->prev_cpu_idle = get_cpu_idle_time_us(cpu, &info->prev_cpu_wall);
		info->idle_periods = 0;
	}
	queue_delayed_work_on(0, tuners->workq, &tuners->work, delay);
}

int hotplug_governor_cpufreq_action(unsigned long event, struct cpufreq_freqs *freqs)
{
	struct cpu_hotplug_info_s *info;
	struct cpu_hotplug_tuners *tuners;
	int cpu, ret = 0, old_state;

	info = &per_cpu(cpu_hotplug_info, smp_processor_id());
	tuners = info->tuners;

	old_state = tuners->state;
	/* printk("%s: event %ld, state %d\n", __func__, event, tuners->state); */
	if (event == CPUFREQ_PRECHANGE) {
		/*if (atomic_cmpxchg(&tuners->lock, LOCK_OWNER_NONE, LOCK_OWNER_CPUFREQ) != LOCK_OWNER_NONE) {
			printk("hotplug profiling is on\n");
			schedule();
			ret = -EAGAIN;
			goto out;
		}*/
		spin_lock(&tuners->lock);
		if (tuners->state & CPU_HOTPLUG_RUN_MASK) {
			if (num_online_cpus() == 1)
				tuners->state = CPU_HOTPLUG_RUN_LV3;
			else if (num_online_cpus() == 2)
				tuners->state = CPU_HOTPLUG_RUN_LV2;
			else if (num_online_cpus() == 3)
				tuners->state = CPU_HOTPLUG_RUN_LV1;

			if (tuners->state == CPU_HOTPLUG_RUN_LV1 || tuners->state == CPU_HOTPLUG_RUN_LV2) {
				int dump_state = tuners->state;

				for_each_online_cpu(cpu) {
					if (freqs[cpu].old > freqs[cpu].new) {
						spin_unlock(&tuners->lock);
						ret = -EPERM;
						WARN(1, "change freq from %d to be %d when state %d\n",
							freqs[cpu].old, freqs[cpu].new, dump_state);
						goto out;
					}
				}
			}
		}
		spin_unlock(&tuners->lock);
		/* printk("cpu %d: old %d, new %d, num_present_cpus %d\n",  freqs->cpu, freqs->old, freqs->new,  num_online_cpus()); */
	}
	else if (event == CPUFREQ_POSTCHANGE) {
		int change = 1;

		if (tuners->state == CPU_HOTPLUG_STOP_HIGH) {
			for_each_online_cpu(cpu) {
				if (freqs[cpu].new != tuners->barrier_freq[cpu]) {
					change = 0;
					break;
				}
			}

			if (change)
				hotplug_governor_start_profiling(tuners);
		}
		else {
			int up = 1;

			for_each_online_cpu(cpu) {
				if (freqs[cpu].new < freqs[cpu].old) {
					up = 0;
					break;
				}
			}
			if (up) {
				spin_lock(&tuners->lock);
				if (tuners->state == CPU_HOTPLUG_STOP_LOW && freqs[0].new <= tuners->barrier_freq[0]) {
					if (freqs[0].new == tuners->barrier_freq[0]) {
						tuners->state = CPU_HOTPLUG_RUN_LV3;
						spin_unlock(&tuners->lock);
						hotplug_governor_start_profiling(tuners);
					}
					else
						spin_unlock(&tuners->lock);
				}
				else {
					tuners->state = CPU_HOTPLUG_STOP_HIGH;
					spin_unlock(&tuners->lock);
					flush_delayed_work(&tuners->work);
					for_each_present_cpu(cpu)
						if (!cpu_online(cpu) && cpu < tuners->boot_cores)
							cpu_up(cpu);
				}
			}
			else {
				spin_lock(&tuners->lock);
				if (tuners->state == CPU_HOTPLUG_RUN_LV3) {
					tuners->state = CPU_HOTPLUG_STOP_LOW;
					spin_unlock(&tuners->lock);
					flush_delayed_work(&tuners->work);
				}
				else if (tuners->state == CPU_HOTPLUG_RUN_LV1 || tuners->state == CPU_HOTPLUG_RUN_LV2) {
					int dump_state = tuners->state;
					spin_unlock(&tuners->lock);
					ret = -EPERM;
					WARN(1, "change freq from %d to be %d when state %d\n",
						freqs[cpu].old, freqs[cpu].new, dump_state);
				}
				else
					spin_unlock(&tuners->lock);
			}
		}
		/* atomic_cmpxchg(&tuners->lock, LOCK_OWNER_CPUFREQ, LOCK_OWNER_NONE); */
	}
out:
	if (old_state != tuners->state)
		printk("return ret %d, state %d\n", ret, tuners->state);
	return ret;
}

static ssize_t cpu_hotplug_show(struct kobject * kobj, struct attribute * attr ,char * buf)
{
	struct hotplug_attr *cattr = container_of(attr, struct hotplug_attr, attr);
	struct cpu_hotplug_tuners *tuners = container_of(kobj, struct cpu_hotplug_tuners, kobj);

	if (cattr->show)
		return cattr->show(tuners, buf);

	return 0;
}

static ssize_t cpu_hotplug_store(struct kobject * kobj, struct attribute * attr,
		     const char * buf, size_t count)
{
	struct hotplug_attr *cattr = container_of(attr, struct hotplug_attr, attr);
	struct cpu_hotplug_tuners *tuners = container_of(kobj, struct cpu_hotplug_tuners, kobj);

	if (cattr->store)
		return cattr->store(tuners, buf, count);

	return count;
}

static const struct sysfs_ops cpu_hotplug_sysfs_ops = {
	.show = cpu_hotplug_show,
	.store = cpu_hotplug_store,
};

define_hotplug_governor_attr_rw(system_down_threshold);
define_hotplug_governor_attr_rw(system_up_threshold);
define_hotplug_governor_attr_rw(system_down_periods_threshold);
define_hotplug_governor_attr_rw(idle_threshold);
define_hotplug_governor_attr_rw(idle_periods_threshold);
define_hotplug_governor_attr_rw(sampling_rate);
define_hotplug_governor_attr_rw(load_ratio);

static struct attribute *tuners_attrs[] = {
	&system_down_threshold.attr,
	&system_up_threshold.attr,
	&system_down_periods_threshold.attr,
	&idle_threshold.attr,
	&idle_periods_threshold.attr,
	&sampling_rate.attr,
	&load_ratio.attr,
	NULL
};

static struct kobj_type cpu_hotplug_ktype = {
	.sysfs_ops = &cpu_hotplug_sysfs_ops,
	.default_attrs = tuners_attrs,
};

static int __init hotplug_governor_init(void)
{
	struct cpu_hotplug_tuners *tuners;
	struct cpu_hotplug_info_s *info;
	int cpu, ret;

	tuners = (struct cpu_hotplug_tuners *)kzalloc(sizeof(*tuners), GFP_KERNEL);
	if (!tuners) {
		ret = -ENOMEM;
		goto out;
	}

	for (cpu = 0; cpu <= NR_CPUS -1 ; cpu++) {
		tuners->barrier_freq[cpu] = 400000;
	}
	tuners->system_down_threshold = DEFAULT_SYSTEM_DOWN_THRESHOLD;
	tuners->system_up_threshold = DEFAULT_SYSTEM_UP_THRESHOLD;
	tuners->system_down_periods_threshold = DEFAULT_SYSTEM_DOWN_PERIODS_THRESHOLD;
	tuners->idle_threshold = DEFAULT_IDLE_THRESHOLD;
	tuners->idle_periods_threshold = DEFAULT_IDLE_PERIODS_THRESHOLD;
	tuners->sampling_rate = DEFAULT_SAMPLING_PERIOD;
	tuners->load_ratio = DEFAULT_LOAD_RATIO;
	tuners->state = CPU_HOTPLUG_STOP_HIGH;
	tuners->boot_cores = num_online_cpus();
	/* mutex_init(&tuners->tuners_mutex);
	atomic_set(&tuners->lock, 0); */
	spin_lock_init(&tuners->lock);
	tuners->workq = create_workqueue("khotplug");
	if (!tuners->workq) {
		printk("Creation of khotplug failed\n");
		ret =  -EFAULT;
		goto free_tuners;
	}

	INIT_DELAYED_WORK(&tuners->work, do_cpu_hotplug_timer);
	for_each_present_cpu(cpu) {
		info = &per_cpu(cpu_hotplug_info, cpu);
		info->tuners = tuners;
	}

	ret = kobject_init_and_add(&tuners->kobj, &cpu_hotplug_ktype,
				&cpu_sysdev_class.kset.kobj, "cpuhotplug");
	if (ret)
		goto free_tuners;
/*	ret = cpufreq_register_notifier(&hotplug_governor_cpufreq_notifier_block,
			CPUFREQ_TRANSITION_NOTIFIER);
        if (ret < 0) {
		printk(KERN_ERR "Unable to register CPU frequency change "
			"notifier for CPU hotplug (%d)\n", ret);
		goto error;
	}*/

	return 0;

/*error:
	for_each_present_cpu(cpu) {
		info = &per_cpu(cpu_hotplug_info, smp_processor_id());
		info->tuners = NULL;
	}*/
free_tuners:
	kfree(tuners);
out:
	return ret;
}

static void __exit hotplug_governor_exit(void)
{
	struct cpu_hotplug_tuners *tuners;
	struct cpu_hotplug_info_s *info;
	int cpu;

	/*cpufreq_unregister_notifier(&hotplug_governor_cpufreq_notifier_block,
			CPUFREQ_TRANSITION_NOTIFIER);*/
	info = &per_cpu(cpu_hotplug_info, smp_processor_id());
	tuners = info->tuners;
	for_each_present_cpu(cpu) {
		info = &per_cpu(cpu_hotplug_info, cpu);
		info->tuners = NULL;
	}
	kfree(tuners);
}
module_init(hotplug_governor_init);
module_exit(hotplug_governor_exit);

#endif /* CONFIG_HOTPLUG_CPU */
