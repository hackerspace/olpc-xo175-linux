#ifndef __MACH_BARRIERS_H
#define __MACH_BARRIERS_H

#ifdef CONFIG_PJ4B_ERRATA_6075
#define mb()		dsb()
#define rmb()		dsb()
#define wmb()		mb()
#endif

#endif /* __MACH_BARRIERS_H */
