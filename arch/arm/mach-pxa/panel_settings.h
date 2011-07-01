/* panel setting defines */
#ifndef _PANEL_SETTINGS_H_
#define _PANEL_SETTINGS_H_
extern struct fb_videomode video_modes_tc3587[1];
extern struct fb_videomode video_modes_trulywvga[1];
extern struct fb_videomode video_modes_si9226[1];
extern struct fb_videomode video_modes_adv7533[1];

void panel_power_tc3587(int ssp_port, int on);
void panel_power_trulywvga(int ssp_port, int on);
void panel_set_trulywvga(int ssp_port);

#endif
