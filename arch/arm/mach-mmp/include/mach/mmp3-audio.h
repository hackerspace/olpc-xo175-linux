/*
*
* Marvell MMP2 Audio Driver
*
* This file is licensed under the terms of the GNU General Public
* License version 2.  This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

#ifndef _MMP3_AUDIO_H_
#define _MMP3_AUDIO_H_

struct audio_dsp_pwr_status {
	u32 zsp_cnt;
	u32 main_pwr_cnt;
	u32 aud_dev_cnt;
	u32 aud_pll_cnt;
};

#endif
