#ifndef __MACH_PXA_DISPLAY_H
#define __MACH_PXA_DISPLAY_H

#ifdef CONFIG_LCD_BYD_9009AD
void __init pxa95x_add_lcd_byd(void);
#endif
#ifdef CONFIG_LCD_TRULY_TFT480800
void __init pxa95x_add_lcd_truly(void);
#endif
#ifdef CONFIG_LCD_SHARP_LS043
void __init pxa95x_add_lcd_sharp(void);
#endif

#endif	/* __MACH_PXA_DISPLAY_H */
