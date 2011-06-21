#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

struct sys_timer;

extern void timer_init(int irq0, int irq1);

extern void __init icu_init_irq(void);
extern void __init mmp_map_io(void);
