/* SSD2531_touch.h - header file for SSD2531 touch panel.
 *
 * Author: fwu <fwu@marvell.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __SSD2531_H__
#define __SSD2531_H__

struct ssd2531_platform_data {
	int (*set_power) (int);
	int *pin_data;
};

#endif
