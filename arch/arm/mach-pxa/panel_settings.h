/* panel setting defines */
#ifndef _PANEL_SETTINGS_H_
#define _PANEL_SETTINGS_H_
#define MV_IHDMI_MODES 4

extern struct fb_videomode video_modes_tc3587[1];
extern struct fb_videomode video_modes_trulywvga[1];
extern struct fb_videomode video_modes_si9226[1];
extern struct fb_videomode video_modes_adv7533[1];
extern struct fb_videomode video_modes_ihdmi[MV_IHDMI_MODES];

void panel_power_tc3587(int ssp_port, int on);
void panel_power_trulywvga(int ssp_port, int on);
void panel_set_trulywvga(int ssp_port);

#endif
