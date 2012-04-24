unsigned VirtualToPhysical(unsigned);
/*-----------------------------------------------------------------*/
#define PMU_DATA_SIZE               128
void save_performance_monitors(unsigned int *pointer);
void restore_performance_monitors(unsigned int *pointer);
/*-----------------------------------------------------------------*/
#define DEBUG_DATA_SIZE               128
void save_pxa978_debug(unsigned int *pointer);
void restore_pxa978_debug(unsigned int *pointer);
/*-----------------------------------------------------------------*/
/* Critical registers saved/restored inside one assembly function.
sramaddr[13..0]=0 is enforced.
These registers are saved in SRAM at the given address,
right after the reset vector code. Aproximae size: 0xc0 bytes.
*/
void pxa978_cpu_suspend(unsigned int pwrmode);
void pxa978_save_reset_handler(unsigned int sramaddr);
