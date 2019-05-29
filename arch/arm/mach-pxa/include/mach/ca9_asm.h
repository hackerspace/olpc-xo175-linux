
unsigned VirtualToPhysical(unsigned);
/*-----------------------------------------------------------------*/
#define PMU_DATA_SIZE               128
void save_performance_monitors(unsigned int *pointer);
void restore_performance_monitors(unsigned int *pointer);
/*-----------------------------------------------------------------*/
#define VFP_DATA_SIZE               288
void save_vfp(unsigned int *pointer);
void restore_vfp(unsigned int *pointer);
/*-----------------------------------------------------------------*/
#define DEBUG_DATA_SIZE               128
void save_ca9_debug(unsigned int *pointer);
void restore_ca9_debug(unsigned int *pointer);
/*-----------------------------------------------------------------*/
/* Critical registers saved/restored inside one assembly function.
sramaddr[13..0]=0 is enforced.
These registers are saved in SRAM at the given address,
right after the reset vector code. Aproximae size: 0xc0 bytes.
*/
void ca9_enter_c2_wrapper(struct pl310_context *pl310, unsigned l2c_base_address, unsigned pwrmode, unsigned sramaddr);
