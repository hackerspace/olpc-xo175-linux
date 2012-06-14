#include <linux/irq.h>

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

#define TIMER_RATE_32K	32768

struct sys_timer;

#ifdef CONFIG_SOC_LOCAL_TIMERS
extern void timer_init(int irq0, int irq1, int irq2);
#else
extern void timer_init(int irq0, int irq1);
#endif

extern void __init stimer_event_config(int n, int cpu, int irq, int r);
extern void __init stimer_source_select(int n, int r);
extern void __init stimer_device_init(unsigned int base);

extern int pxa910_set_wake(struct irq_data *data, unsigned int on);
extern int mmp3_set_wake(struct irq_data *data, unsigned int on);
extern int pxa988_set_wake(struct irq_data *data, unsigned int on);

extern void __init icu_init_irq(void);
extern void __init mmp_map_io(void);
extern void __init mmp_wakeupgen_init(void);

#ifdef CONFIG_SMP
extern void __iomem *pxa_scu_base_addr(void);
#else
static inline void __iomem *pxa_scu_base_addr(void)
{
	return NULL;
}
#endif
