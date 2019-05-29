/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */
#ifndef BOARD_DVFM_OPS_H
#define BOARD_DVFM_OPS_H

#ifndef __ASSEMBLY__
#include <mach/mmp2_pm.h>
/*
 * public data
 */
struct board_fc_info {
	void	*va_pmua;
	void	*va_dmcu;
	int	cc_ap;
	int	fccr;
	void	*va_pmum;
	int	v_flag;
	int	mem_config;

	void	*priv;			/* priv data for specific board */
};

struct board_fc_ops {
	int	(*setup_fc_seq)(void *vaddr);
	int	(*execute_fc_seq)(void *vaddr, void *vstack, int vol,  struct board_fc_info *info);

	int	(*get_profile)(void);
	int	(*get_max_freq)(void);
	int	(*get_voltage)(unsigned int product_point);
};

/*
 * public functions
 */
extern int register_board_fc_ops(struct board_fc_ops *ops);
extern int unregister_board_fc_ops(struct board_fc_ops *ops);

extern int board_get_profile(void);
extern int board_get_max_freq(void);
extern int board_get_voltage(unsigned int product_point);

extern int setup_fc_seq(void *vaddr);
extern int run_fc_seq(struct mmp2_fc_param *param, struct mmp2_pm_info *info);

extern void *copy_fcs_to_sram(void *vaddr, void *start, void *end);
extern unsigned int jump_to_fcs_sram(void *addr, void *stack, struct board_fc_info *info);

extern void *bs_do_fcs;
extern void *bs_do_fcs_end;
extern void *g50_do_fcs;
extern void *g50_do_fcs_end;
#endif
#endif
