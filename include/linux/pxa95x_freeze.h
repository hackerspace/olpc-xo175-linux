/* pxa95x_freeze.h
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef PXA95X_FREEZE_H
#define PXA95X_FREEZE_H


#define PROCESS_NON_FREEZE 1
#define MAX_LINE_LENGTH 256
#define FRZ_DEBUG_DISPLAY_TASK 1
#define FRZ_DEBUG_CHECK_FREEZE_TASKS (1 << 1)

typedef struct {
    unsigned long data_arg1;
    unsigned long data_arg2;
} data_struct;

#endif
