#include <linux/irq.h>

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

struct sys_timer;

#ifdef CONFIG_SOC_LOCAL_TIMERS
extern void timer_init(int irq0, int irq1, int irq2);
#else
extern void timer_init(int irq0, int irq1);
#endif

extern int pxa910_set_wake(struct irq_data *data, unsigned int on);
extern int mmp3_set_wake(struct irq_data *data, unsigned int on);

extern void __init icu_init_irq(void);
extern void __init mmp_map_io(void);
