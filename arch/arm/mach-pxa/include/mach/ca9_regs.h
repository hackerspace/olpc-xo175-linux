
#include <linux/kernel.h>

#define L2310_ADDR_START 0x58120000
#define L2310_ADDR_END	 0x58120FFF

struct lockdown_regs {
	unsigned int d, i;
};

struct pl310_registers {
	const unsigned cache_id;
	const unsigned cache_type;
	char padding1[0x0F8];
	volatile unsigned control;
	volatile unsigned aux_control;
	volatile unsigned tag_ram_control;
	volatile unsigned data_ram_control;
	char padding2[0x0F0];
	volatile unsigned ev_counter_ctrl;
	volatile unsigned ev_counter1_cfg;
	volatile unsigned ev_counter0_cfg;
	volatile unsigned ev_counter1;
	volatile unsigned ev_counter0;
	volatile unsigned int_mask;
	const volatile unsigned int_mask_status;
	const volatile unsigned int_raw_status;
	volatile unsigned int_clear;
	char padding3[0x50C];
	volatile unsigned cache_sync;
	char padding4[0x03C];
	volatile unsigned inv_pa;
	char padding5[0x008];
	volatile unsigned inv_way;
	char padding6[0x030];
	volatile unsigned clean_pa;
	char padding7[0x004];
	volatile unsigned clean_index;
	volatile unsigned clean_way;
	char padding8[0x030];
	volatile unsigned clean_inv_pa;
	char padding9[0x004];
	volatile unsigned clean_inv_index;
	volatile unsigned clean_inv_way;
	char paddinga[0x100];
	volatile struct lockdown_regs lockdown[8];
	char paddingb[0x010];
	volatile unsigned lock_line_en;
	volatile unsigned unlock_way;
	char paddingc[0x2A8];
	volatile unsigned addr_filtering_start;
	volatile unsigned addr_filtering_end;
	char paddingd[0x338];
	volatile unsigned debug_ctrl;
	char paddinge[0x01C];
	volatile unsigned prefetch_ctrl;
	char paddingf[0x01C];
	volatile unsigned power_ctrl;
	unsigned int *memset;
};


struct pl310_context {
	unsigned int control;
	unsigned int aux_control;
	unsigned int tag_ram_control;
	unsigned int data_ram_control;
	unsigned int ev_counter_ctrl;
	unsigned int ev_counter1_cfg;
	unsigned int ev_counter0_cfg;
	unsigned int ev_counter1;
	unsigned int ev_counter0;
	unsigned int int_mask;
	unsigned int lock_line_en;
	struct lockdown_regs lockdown[8];
	unsigned int unlock_way;
	unsigned int addr_filtering_start;
	unsigned int addr_filtering_end;
	unsigned int debug_ctrl;
	unsigned int prefetch_ctrl;
	unsigned int power_ctrl;
};
