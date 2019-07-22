/* panel setting defines */
#include <linux/delay.h>
#include <linux/pxa2xx_ssp.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include "generic.h"

#define WMLCD_COM_MSB (0x1 << 13)
#define WMLCD_COM_LSB (0x0 << 13)
#define WMLCD_DATA (0x1 << 14)
#define WMLCD_DELAY(y) (y << 24)
#define WMLCDDATA(x) (WMLCD_DATA | x)
#define WMLCDCOM(x) (WMLCD_COM_MSB | ((x&0xFF00)>>8)), (WMLCD_COM_LSB | (x&0xFF))

/* There should be a 1ms delay after each data entry */
static u16 lcd_panel_init[] = {
/* Delayms(200), */
/* ENABLE PAGE 1 */
	WMLCDCOM(0xF000), WMLCDDATA(0x55),
	WMLCDCOM(0xF001), WMLCDDATA(0xAA),
	WMLCDCOM(0xF002), WMLCDDATA(0x52),
	WMLCDCOM(0xF003), WMLCDDATA(0x08),
	WMLCDCOM(0xF004), WMLCDDATA(0x01),
/* VGMP/VGMN/VCOM SETING */
	WMLCDCOM(0xBC01), WMLCDDATA(0x90),
	WMLCDCOM(0xBC02), WMLCDDATA(0x51),
	WMLCDCOM(0xBD01), WMLCDDATA(0x90),
	WMLCDCOM(0xBD02), WMLCDDATA(0x51),
	WMLCDCOM(0xBE01), WMLCDDATA(0x67),
/* GAMMA SETING  RED */
	WMLCDCOM(0xD100), WMLCDDATA(0x00),
	WMLCDCOM(0xD101), WMLCDDATA(0x57),
	WMLCDCOM(0xD102), WMLCDDATA(0x00),
	WMLCDCOM(0xD103), WMLCDDATA(0x63),
	WMLCDCOM(0xD104), WMLCDDATA(0x00),
	WMLCDCOM(0xD105), WMLCDDATA(0x79),
	WMLCDCOM(0xD106), WMLCDDATA(0x00),
	WMLCDCOM(0xD107), WMLCDDATA(0x8D),
	WMLCDCOM(0xD108), WMLCDDATA(0x00),
	WMLCDCOM(0xD109), WMLCDDATA(0xA0),
	WMLCDCOM(0xD10A), WMLCDDATA(0x00),
	WMLCDCOM(0xD10B), WMLCDDATA(0xC2),
	WMLCDCOM(0xD10C), WMLCDDATA(0x00),
	WMLCDCOM(0xD10D), WMLCDDATA(0xE0),
	WMLCDCOM(0xD10E), WMLCDDATA(0x01),
	WMLCDCOM(0xD10F), WMLCDDATA(0x14),
	WMLCDCOM(0xD110), WMLCDDATA(0x01),
	WMLCDCOM(0xD111), WMLCDDATA(0x3E),
	WMLCDCOM(0xD112), WMLCDDATA(0x01),
	WMLCDCOM(0xD113), WMLCDDATA(0x81),
	WMLCDCOM(0xD114), WMLCDDATA(0x01),
	WMLCDCOM(0xD115), WMLCDDATA(0xB8),
	WMLCDCOM(0xD116), WMLCDDATA(0x02),
	WMLCDCOM(0xD117), WMLCDDATA(0x11),
	WMLCDCOM(0xD118), WMLCDDATA(0x02),
	WMLCDCOM(0xD119), WMLCDDATA(0x57),
	WMLCDCOM(0xD11A), WMLCDDATA(0x02),
	WMLCDCOM(0xD11B), WMLCDDATA(0x5A),
	WMLCDCOM(0xD11C), WMLCDDATA(0x02),
	WMLCDCOM(0xD11D), WMLCDDATA(0x97),
	WMLCDCOM(0xD11E), WMLCDDATA(0x02),
	WMLCDCOM(0xD11F), WMLCDDATA(0xD6),
	WMLCDCOM(0xD120), WMLCDDATA(0x02),
	WMLCDCOM(0xD121), WMLCDDATA(0xFB),
	WMLCDCOM(0xD122), WMLCDDATA(0x03),
	WMLCDCOM(0xD123), WMLCDDATA(0x2E),
	WMLCDCOM(0xD124), WMLCDDATA(0x03),
	WMLCDCOM(0xD125), WMLCDDATA(0x52),
	WMLCDCOM(0xD126), WMLCDDATA(0x03),
	WMLCDCOM(0xD127), WMLCDDATA(0x87),
	WMLCDCOM(0xD128), WMLCDDATA(0x03),
	WMLCDCOM(0xD129), WMLCDDATA(0xA9),
	WMLCDCOM(0xD12A), WMLCDDATA(0x03),
	WMLCDCOM(0xD12B), WMLCDDATA(0xCB),
	WMLCDCOM(0xD12C), WMLCDDATA(0x03),
	WMLCDCOM(0xD12D), WMLCDDATA(0xDC),
	WMLCDCOM(0xD12E), WMLCDDATA(0x03),
	WMLCDCOM(0xD12F), WMLCDDATA(0xE5),
	WMLCDCOM(0xD130), WMLCDDATA(0x03),
	WMLCDCOM(0xD131), WMLCDDATA(0xED),
	WMLCDCOM(0xD132), WMLCDDATA(0x03),
	WMLCDCOM(0xD133), WMLCDDATA(0xF0),
/* GAMMA SETING GREEN */
	WMLCDCOM(0xD200), WMLCDDATA(0x00),
	WMLCDCOM(0xD201), WMLCDDATA(0x57),
	WMLCDCOM(0xD202), WMLCDDATA(0x00),
	WMLCDCOM(0xD203), WMLCDDATA(0x63),
	WMLCDCOM(0xD204), WMLCDDATA(0x00),
	WMLCDCOM(0xD205), WMLCDDATA(0x79),
	WMLCDCOM(0xD206), WMLCDDATA(0x00),
	WMLCDCOM(0xD207), WMLCDDATA(0x8D),
	WMLCDCOM(0xD208), WMLCDDATA(0x00),
	WMLCDCOM(0xD209), WMLCDDATA(0xA0),
	WMLCDCOM(0xD20A), WMLCDDATA(0x00),
	WMLCDCOM(0xD20B), WMLCDDATA(0xC2),
	WMLCDCOM(0xD20C), WMLCDDATA(0x00),
	WMLCDCOM(0xD20D), WMLCDDATA(0xE0),
	WMLCDCOM(0xD20E), WMLCDDATA(0x01),
	WMLCDCOM(0xD20F), WMLCDDATA(0x14),
	WMLCDCOM(0xD210), WMLCDDATA(0x01),
	WMLCDCOM(0xD211), WMLCDDATA(0x3E),
	WMLCDCOM(0xD212), WMLCDDATA(0x01),
	WMLCDCOM(0xD213), WMLCDDATA(0x81),
	WMLCDCOM(0xD214), WMLCDDATA(0x01),
	WMLCDCOM(0xD215), WMLCDDATA(0xB8),
	WMLCDCOM(0xD216), WMLCDDATA(0x02),
	WMLCDCOM(0xD217), WMLCDDATA(0x11),
	WMLCDCOM(0xD218), WMLCDDATA(0x02),
	WMLCDCOM(0xD219), WMLCDDATA(0x57),
	WMLCDCOM(0xD21A), WMLCDDATA(0x02),
	WMLCDCOM(0xD21B), WMLCDDATA(0x5A),
	WMLCDCOM(0xD21C), WMLCDDATA(0x02),
	WMLCDCOM(0xD21D), WMLCDDATA(0x97),
	WMLCDCOM(0xD21E), WMLCDDATA(0x02),
	WMLCDCOM(0xD21F), WMLCDDATA(0xD6),
	WMLCDCOM(0xD220), WMLCDDATA(0x02),
	WMLCDCOM(0xD221), WMLCDDATA(0xFB),
	WMLCDCOM(0xD222), WMLCDDATA(0x03),
	WMLCDCOM(0xD223), WMLCDDATA(0x2E),
	WMLCDCOM(0xD224), WMLCDDATA(0x03),
	WMLCDCOM(0xD225), WMLCDDATA(0x52),
	WMLCDCOM(0xD226), WMLCDDATA(0x03),
	WMLCDCOM(0xD227), WMLCDDATA(0x87),
	WMLCDCOM(0xD228), WMLCDDATA(0x03),
	WMLCDCOM(0xD229), WMLCDDATA(0xA9),
	WMLCDCOM(0xD22A), WMLCDDATA(0x03),
	WMLCDCOM(0xD22B), WMLCDDATA(0xCB),
	WMLCDCOM(0xD22C), WMLCDDATA(0x03),
	WMLCDCOM(0xD22D), WMLCDDATA(0xDC),
	WMLCDCOM(0xD22E), WMLCDDATA(0x03),
	WMLCDCOM(0xD22F), WMLCDDATA(0xE5),
	WMLCDCOM(0xD230), WMLCDDATA(0x03),
	WMLCDCOM(0xD231), WMLCDDATA(0xED),
	WMLCDCOM(0xD232), WMLCDDATA(0x03),
	WMLCDCOM(0xD233), WMLCDDATA(0xF0),
/* GAMMA SETING BLUE */
	WMLCDCOM(0xD300), WMLCDDATA(0x00),
	WMLCDCOM(0xD301), WMLCDDATA(0x57),
	WMLCDCOM(0xD302), WMLCDDATA(0x00),
	WMLCDCOM(0xD303), WMLCDDATA(0x63),
	WMLCDCOM(0xD304), WMLCDDATA(0x00),
	WMLCDCOM(0xD305), WMLCDDATA(0x79),
	WMLCDCOM(0xD306), WMLCDDATA(0x00),
	WMLCDCOM(0xD307), WMLCDDATA(0x8D),
	WMLCDCOM(0xD308), WMLCDDATA(0x00),
	WMLCDCOM(0xD309), WMLCDDATA(0xA0),
	WMLCDCOM(0xD30A), WMLCDDATA(0x00),
	WMLCDCOM(0xD30B), WMLCDDATA(0xC2),
	WMLCDCOM(0xD30C), WMLCDDATA(0x00),
	WMLCDCOM(0xD30D), WMLCDDATA(0xE0),
	WMLCDCOM(0xD30E), WMLCDDATA(0x01),
	WMLCDCOM(0xD30F), WMLCDDATA(0x14),
	WMLCDCOM(0xD310), WMLCDDATA(0x01),
	WMLCDCOM(0xD311), WMLCDDATA(0x3E),
	WMLCDCOM(0xD312), WMLCDDATA(0x01),
	WMLCDCOM(0xD313), WMLCDDATA(0x81),
	WMLCDCOM(0xD314), WMLCDDATA(0x01),
	WMLCDCOM(0xD315), WMLCDDATA(0xB8),
	WMLCDCOM(0xD316), WMLCDDATA(0x02),
	WMLCDCOM(0xD317), WMLCDDATA(0x11),
	WMLCDCOM(0xD318), WMLCDDATA(0x02),
	WMLCDCOM(0xD319), WMLCDDATA(0x57),
	WMLCDCOM(0xD31A), WMLCDDATA(0x02),
	WMLCDCOM(0xD31B), WMLCDDATA(0x5A),
	WMLCDCOM(0xD31C), WMLCDDATA(0x02),
	WMLCDCOM(0xD31D), WMLCDDATA(0x97),
	WMLCDCOM(0xD31E), WMLCDDATA(0x02),
	WMLCDCOM(0xD31F), WMLCDDATA(0xD6),
	WMLCDCOM(0xD320), WMLCDDATA(0x02),
	WMLCDCOM(0xD321), WMLCDDATA(0xFB),
	WMLCDCOM(0xD322), WMLCDDATA(0x03),
	WMLCDCOM(0xD323), WMLCDDATA(0x2E),
	WMLCDCOM(0xD324), WMLCDDATA(0x03),
	WMLCDCOM(0xD325), WMLCDDATA(0x52),
	WMLCDCOM(0xD326), WMLCDDATA(0x03),
	WMLCDCOM(0xD327), WMLCDDATA(0x87),
	WMLCDCOM(0xD328), WMLCDDATA(0x03),
	WMLCDCOM(0xD329), WMLCDDATA(0xA9),
	WMLCDCOM(0xD32A), WMLCDDATA(0x03),
	WMLCDCOM(0xD32B), WMLCDDATA(0xCB),
	WMLCDCOM(0xD32C), WMLCDDATA(0x03),
	WMLCDCOM(0xD32D), WMLCDDATA(0xDC),
	WMLCDCOM(0xD32E), WMLCDDATA(0x03),
	WMLCDCOM(0xD32F), WMLCDDATA(0xE5),
	WMLCDCOM(0xD330), WMLCDDATA(0x03),
	WMLCDCOM(0xD331), WMLCDDATA(0xED),
	WMLCDCOM(0xD332), WMLCDDATA(0x03),
	WMLCDCOM(0xD333), WMLCDDATA(0xF0),

/* GAMMA SETING RED */
	WMLCDCOM(0xD400), WMLCDDATA(0x00),
	WMLCDCOM(0xD401), WMLCDDATA(0x57),
	WMLCDCOM(0xD402), WMLCDDATA(0x00),
	WMLCDCOM(0xD403), WMLCDDATA(0x63),
	WMLCDCOM(0xD404), WMLCDDATA(0x00),
	WMLCDCOM(0xD405), WMLCDDATA(0x79),
	WMLCDCOM(0xD406), WMLCDDATA(0x00),
	WMLCDCOM(0xD407), WMLCDDATA(0x8D),
	WMLCDCOM(0xD408), WMLCDDATA(0x00),
	WMLCDCOM(0xD409), WMLCDDATA(0xA0),
	WMLCDCOM(0xD40A), WMLCDDATA(0x00),
	WMLCDCOM(0xD40B), WMLCDDATA(0xC2),
	WMLCDCOM(0xD40C), WMLCDDATA(0x00),
	WMLCDCOM(0xD40D), WMLCDDATA(0xE0),
	WMLCDCOM(0xD40E), WMLCDDATA(0x01),
	WMLCDCOM(0xD40F), WMLCDDATA(0x14),
	WMLCDCOM(0xD410), WMLCDDATA(0x01),
	WMLCDCOM(0xD411), WMLCDDATA(0x3E),
	WMLCDCOM(0xD412), WMLCDDATA(0x01),
	WMLCDCOM(0xD413), WMLCDDATA(0x81),
	WMLCDCOM(0xD414), WMLCDDATA(0x01),
	WMLCDCOM(0xD415), WMLCDDATA(0xB8),
	WMLCDCOM(0xD416), WMLCDDATA(0x02),
	WMLCDCOM(0xD417), WMLCDDATA(0x11),
	WMLCDCOM(0xD418), WMLCDDATA(0x02),
	WMLCDCOM(0xD419), WMLCDDATA(0x57),
	WMLCDCOM(0xD41A), WMLCDDATA(0x02),
	WMLCDCOM(0xD41B), WMLCDDATA(0x5A),
	WMLCDCOM(0xD41C), WMLCDDATA(0x02),
	WMLCDCOM(0xD41D), WMLCDDATA(0x97),
	WMLCDCOM(0xD41E), WMLCDDATA(0x02),
	WMLCDCOM(0xD41F), WMLCDDATA(0xD6),
	WMLCDCOM(0xD420), WMLCDDATA(0x02),
	WMLCDCOM(0xD421), WMLCDDATA(0xFB),
	WMLCDCOM(0xD422), WMLCDDATA(0x03),
	WMLCDCOM(0xD423), WMLCDDATA(0x2E),
	WMLCDCOM(0xD424), WMLCDDATA(0x03),
	WMLCDCOM(0xD425), WMLCDDATA(0x52),
	WMLCDCOM(0xD426), WMLCDDATA(0x03),
	WMLCDCOM(0xD427), WMLCDDATA(0x87),
	WMLCDCOM(0xD428), WMLCDDATA(0x03),
	WMLCDCOM(0xD429), WMLCDDATA(0xA9),
	WMLCDCOM(0xD42A), WMLCDDATA(0x03),
	WMLCDCOM(0xD42B), WMLCDDATA(0xCB),
	WMLCDCOM(0xD42C), WMLCDDATA(0x03),
	WMLCDCOM(0xD42D), WMLCDDATA(0xDC),
	WMLCDCOM(0xD42E), WMLCDDATA(0x03),
	WMLCDCOM(0xD42F), WMLCDDATA(0xE5),
	WMLCDCOM(0xD430), WMLCDDATA(0x03),
	WMLCDCOM(0xD431), WMLCDDATA(0xED),
	WMLCDCOM(0xD432), WMLCDDATA(0x03),
	WMLCDCOM(0xD433), WMLCDDATA(0xF0),
/* GAMMASETING GERREN */
	WMLCDCOM(0xD500), WMLCDDATA(0x00),
	WMLCDCOM(0xD501), WMLCDDATA(0x57),
	WMLCDCOM(0xD502), WMLCDDATA(0x00),
	WMLCDCOM(0xD503), WMLCDDATA(0x63),
	WMLCDCOM(0xD504), WMLCDDATA(0x00),
	WMLCDCOM(0xD505), WMLCDDATA(0x79),
	WMLCDCOM(0xD506), WMLCDDATA(0x00),
	WMLCDCOM(0xD507), WMLCDDATA(0x8D),
	WMLCDCOM(0xD508), WMLCDDATA(0x00),
	WMLCDCOM(0xD509), WMLCDDATA(0xA0),
	WMLCDCOM(0xD50A), WMLCDDATA(0x00),
	WMLCDCOM(0xD50B), WMLCDDATA(0xC2),
	WMLCDCOM(0xD50C), WMLCDDATA(0x00),
	WMLCDCOM(0xD50D), WMLCDDATA(0xE0),
	WMLCDCOM(0xD50E), WMLCDDATA(0x01),
	WMLCDCOM(0xD50F), WMLCDDATA(0x14),
	WMLCDCOM(0xD510), WMLCDDATA(0x01),
	WMLCDCOM(0xD511), WMLCDDATA(0x3E),
	WMLCDCOM(0xD512), WMLCDDATA(0x01),
	WMLCDCOM(0xD513), WMLCDDATA(0x81),
	WMLCDCOM(0xD514), WMLCDDATA(0x01),
	WMLCDCOM(0xD515), WMLCDDATA(0xB8),
	WMLCDCOM(0xD516), WMLCDDATA(0x02),
	WMLCDCOM(0xD517), WMLCDDATA(0x11),
	WMLCDCOM(0xD518), WMLCDDATA(0x02),
	WMLCDCOM(0xD519), WMLCDDATA(0x57),
	WMLCDCOM(0xD51A), WMLCDDATA(0x02),
	WMLCDCOM(0xD51B), WMLCDDATA(0x5A),
	WMLCDCOM(0xD51C), WMLCDDATA(0x02),
	WMLCDCOM(0xD51D), WMLCDDATA(0x97),
	WMLCDCOM(0xD51E), WMLCDDATA(0x02),
	WMLCDCOM(0xD51F), WMLCDDATA(0xD6),
	WMLCDCOM(0xD520), WMLCDDATA(0x02),
	WMLCDCOM(0xD521), WMLCDDATA(0xFB),
	WMLCDCOM(0xD522), WMLCDDATA(0x03),
	WMLCDCOM(0xD523), WMLCDDATA(0x2E),
	WMLCDCOM(0xD524), WMLCDDATA(0x03),
	WMLCDCOM(0xD525), WMLCDDATA(0x52),
	WMLCDCOM(0xD526), WMLCDDATA(0x03),
	WMLCDCOM(0xD527), WMLCDDATA(0x87),
	WMLCDCOM(0xD528), WMLCDDATA(0x03),
	WMLCDCOM(0xD529), WMLCDDATA(0xA9),
	WMLCDCOM(0xD52A), WMLCDDATA(0x03),
	WMLCDCOM(0xD52B), WMLCDDATA(0xCB),
	WMLCDCOM(0xD52C), WMLCDDATA(0x03),
	WMLCDCOM(0xD52D), WMLCDDATA(0xDC),
	WMLCDCOM(0xD52E), WMLCDDATA(0x03),
	WMLCDCOM(0xD52F), WMLCDDATA(0xE5),
	WMLCDCOM(0xD530), WMLCDDATA(0x03),
	WMLCDCOM(0xD531), WMLCDDATA(0xED),
	WMLCDCOM(0xD532), WMLCDDATA(0x03),
	WMLCDCOM(0xD533), WMLCDDATA(0xF0),
/* GAMMA SETING BLUE */
	WMLCDCOM(0xD600), WMLCDDATA(0x00),
	WMLCDCOM(0xD601), WMLCDDATA(0x57),
	WMLCDCOM(0xD602), WMLCDDATA(0x00),
	WMLCDCOM(0xD603), WMLCDDATA(0x63),
	WMLCDCOM(0xD604), WMLCDDATA(0x00),
	WMLCDCOM(0xD605), WMLCDDATA(0x79),
	WMLCDCOM(0xD606), WMLCDDATA(0x00),
	WMLCDCOM(0xD607), WMLCDDATA(0x8D),
	WMLCDCOM(0xD608), WMLCDDATA(0x00),
	WMLCDCOM(0xD609), WMLCDDATA(0xA0),
	WMLCDCOM(0xD60A), WMLCDDATA(0x00),
	WMLCDCOM(0xD60B), WMLCDDATA(0xC2),
	WMLCDCOM(0xD60C), WMLCDDATA(0x00),
	WMLCDCOM(0xD60D), WMLCDDATA(0xE0),
	WMLCDCOM(0xD60E), WMLCDDATA(0x01),
	WMLCDCOM(0xD60F), WMLCDDATA(0x14),
	WMLCDCOM(0xD610), WMLCDDATA(0x01),
	WMLCDCOM(0xD611), WMLCDDATA(0x3E),
	WMLCDCOM(0xD612), WMLCDDATA(0x01),
	WMLCDCOM(0xD613), WMLCDDATA(0x81),
	WMLCDCOM(0xD614), WMLCDDATA(0x01),
	WMLCDCOM(0xD615), WMLCDDATA(0xB8),
	WMLCDCOM(0xD616), WMLCDDATA(0x02),
	WMLCDCOM(0xD617), WMLCDDATA(0x11),
	WMLCDCOM(0xD618), WMLCDDATA(0x02),
	WMLCDCOM(0xD619), WMLCDDATA(0x57),
	WMLCDCOM(0xD61A), WMLCDDATA(0x02),
	WMLCDCOM(0xD61B), WMLCDDATA(0x5A),
	WMLCDCOM(0xD61C), WMLCDDATA(0x02),
	WMLCDCOM(0xD61D), WMLCDDATA(0x97),
	WMLCDCOM(0xD61E), WMLCDDATA(0x02),
	WMLCDCOM(0xD61F), WMLCDDATA(0xD6),
	WMLCDCOM(0xD620), WMLCDDATA(0x02),
	WMLCDCOM(0xD621), WMLCDDATA(0xFB),
	WMLCDCOM(0xD622), WMLCDDATA(0x03),
	WMLCDCOM(0xD623), WMLCDDATA(0x2E),
	WMLCDCOM(0xD624), WMLCDDATA(0x03),
	WMLCDCOM(0xD625), WMLCDDATA(0x52),
	WMLCDCOM(0xD626), WMLCDDATA(0x03),
	WMLCDCOM(0xD627), WMLCDDATA(0x87),
	WMLCDCOM(0xD628), WMLCDDATA(0x03),
	WMLCDCOM(0xD629), WMLCDDATA(0xA9),
	WMLCDCOM(0xD62A), WMLCDDATA(0x03),
	WMLCDCOM(0xD62B), WMLCDDATA(0xCB),
	WMLCDCOM(0xD62C), WMLCDDATA(0x03),
	WMLCDCOM(0xD62D), WMLCDDATA(0xDC),
	WMLCDCOM(0xD62E), WMLCDDATA(0x03),
	WMLCDCOM(0xD62F), WMLCDDATA(0xE5),
	WMLCDCOM(0xD630), WMLCDDATA(0x03),
	WMLCDCOM(0xD631), WMLCDDATA(0xED),
	WMLCDCOM(0xD632), WMLCDDATA(0x03),
	WMLCDCOM(0xD633), WMLCDDATA(0xF0),
/* AVDD VOLTAGE SETTING */
	WMLCDCOM(0xB000), WMLCDDATA(0x09),
	WMLCDCOM(0xB001), WMLCDDATA(0x09),
	WMLCDCOM(0xB002), WMLCDDATA(0x09),
/* AVEE VOLTAGE SETTING */
	WMLCDCOM(0xB100), WMLCDDATA(0x12),
	WMLCDCOM(0xB101), WMLCDDATA(0x12),
	WMLCDCOM(0xB102), WMLCDDATA(0x12),
/* VGLX VOLTAGE SETTING */
	WMLCDCOM(0xBA00), WMLCDDATA(0x24),
	WMLCDCOM(0xBA01), WMLCDDATA(0x24),
	WMLCDCOM(0xBA02), WMLCDDATA(0x24),
/* BGH VOLTAGE SETTING */
	WMLCDCOM(0xB900), WMLCDDATA(0x34),
	WMLCDCOM(0xB901), WMLCDDATA(0x34),
	WMLCDCOM(0xB902), WMLCDDATA(0x34),
/* ENABLE PAGE 0 */
	WMLCDCOM(0xF000), WMLCDDATA(0x55),
	WMLCDCOM(0xF001), WMLCDDATA(0xAA),
	WMLCDCOM(0xF002), WMLCDDATA(0x52),
	WMLCDCOM(0xF003), WMLCDDATA(0x08),
	WMLCDCOM(0xF004), WMLCDDATA(0x00),
/* RAM KEEP */
	WMLCDCOM(0xB100), WMLCDDATA(0xCC),
/* Z-INVERSION */
	WMLCDCOM(0xBC00), WMLCDDATA(0x05),
	WMLCDCOM(0xBC01), WMLCDDATA(0x05),
	WMLCDCOM(0xBC02), WMLCDDATA(0x05),
/* SOURCE EQ */
	WMLCDCOM(0xB800), WMLCDDATA(0x01),
/* PORCH LINES */
	WMLCDCOM(0xBD02), WMLCDDATA(0x07),
	WMLCDCOM(0xBD03), WMLCDDATA(0x31),
	WMLCDCOM(0xBE02), WMLCDDATA(0x07),
	WMLCDCOM(0xBE03), WMLCDDATA(0x31),
	WMLCDCOM(0xBF02), WMLCDDATA(0x07),
	WMLCDCOM(0xBF03), WMLCDDATA(0x31),

	WMLCDCOM(0xFF00), WMLCDDATA(0xAA),
	WMLCDCOM(0xFF01), WMLCDDATA(0x55),
	WMLCDCOM(0xFF02), WMLCDDATA(0x25),
	WMLCDCOM(0xFF03), WMLCDDATA(0x01),
	WMLCDCOM(0xF304), WMLCDDATA(0x11),
	WMLCDCOM(0xF306), WMLCDDATA(0x10),
	WMLCDCOM(0xF408), WMLCDDATA(0x00),
/* TE ON */
	WMLCDCOM(0x3500), WMLCDDATA(0x00),
/* OTHER SET */
	WMLCDCOM(0x3600), WMLCDDATA(0x00),
	WMLCDCOM(0x3A00), WMLCDDATA(0x77),	/* 0x77 for 24 bit, 0x55 for 16 bit */
};

static u16 lcd_panel_slpout[] = {
	WMLCDCOM(0x1100),
/* Delay 120 */
};

static u16 lcd_panel_dison[] = {
	WMLCDCOM(0x2900),
/* Delay 100 */
};

/* Todo - Fixme */
static u16 lcd_panel_disoff[] = {
	WMLCDCOM(0x2800),
};

/* Todo - Fixme */
static u16 lcd_panel_slpin[] = {
	WMLCDCOM(0x1000),
};
static u16 lcd_panel_rotate_0[] = {
	WMLCDCOM(0x3600), WMLCDDATA(0x00),
};

static u16 lcd_panel_rotate_90[] __attribute__((unused)) = {
	WMLCDCOM(0x3600), WMLCDDATA(0x60),
};

static u16 lcd_panel_rotate_180[] __attribute__((unused)) = {
	WMLCDCOM(0x3600), WMLCDDATA(0x80),
};

static u16 lcd_panel_rotate_270[] __attribute__((unused)) = {
	WMLCDCOM(0x3600), WMLCDDATA(0xA0),
};


/******************** Begin of Truly TFT1P3624 settings**********************/

/* Truly TFT1P3624-E WVGA LCD power on cmd */
static u16 truly_1p3624_spi_cmdon[] = {
/* Set password */
	0xB9,
	0x1FF,
	0x183,
	0x169,
/*Set power*/
	0xB1,
	0x185,
	0x100,
	0x134,
	0x107,
	0x100,
	0x10F,
	0x10F,
	0x12A,
	0x132,
	0x13F,
	0x13F,
/* Update VBIAS */
	0x101,
	0x13A,
	0x101,
	0x1E6,
	0x1E6,
	0x1E6,
	0x1E6,
	0x1E6,
/* Set Display 480x800 */
	0xB2,
	0x100,
	0x123,/* 28 */
	0x103,/* 05 */
	0x103,/* 05 */
	0x170,
	0x100,
	0x1FF,
	0x100,
	0x100,
	0x100,
	0x100,
	0x103,
	0x103,
	0x100,
	0x101,
/* Set Display 480x800 */
	0xB4,
	0x100,
	0x118,
	0x180,
	0x106,
	0x102,
/* Set VCOM */
	0xB6,
	0x142,/* Update VCOM */
	0x142,

	0xD5,
	0x100,
	0x104,
	0x103,
	0x100,
	0x101,
	0x104,
	0x11A,
	0x1FF,
	0x101,
	0x113,
	0x100,
	0x100,
	0x140,
	0x106,
	0x151,
	0x107,
	0x100,
	0x100,
	0x141,
	0x106,
	0x150,
	0x107,
	0x107,
	0x10F,
	0x104,
	0x100,
};
/* Truly TFT1P3624-E WVGA LCD Gamma2.2 */
static u16 truly_1p3624_spi_cmd_gamma[] = {
/* Gamma2.2 */
	0xE0,
	0x100,
	0x113,
	0x119,
	0x138,
	0x13D,
	0x13F,
	0x128,
	0x146,
	0x107,
	0x10D,
	0x10E,
	0x112,
	0x115,
	0x112,
	0x114,
	0x10F,
	0x117,
	0x100,
	0x113,
	0x119,
	0x138,
	0x13D,
	0x13F,
	0x128,
	0x146,
	0x107,
	0x10D,
	0x10E,
	0x112,
	0x115,
	0x112,
	0x114,
	0x10F,
	0x117,
};

/* Truly TFT1P3624-E WVGA LCD color */
static u16 truly_1p3624_spi_cmd_color[] = {
	0xC1,
	0x101,
/* R */
	0x104,
	0x113,
	0x11A,
	0x120,
	0x127,
	0x12C,
	0x132,
	0x136,
	0x13F,
	0x147,
	0x150,
	0x159,
	0x160,
	0x168,
	0x171,
	0x17B,
	0x182,
	0x189,
	0x191,
	0x198,
	0x1A0,
	0x1A8,
	0x1B0,
	0x1B8,
	0x1C1,
	0x1C9,
	0x1D0,
	0x1D7,
	0x1E0,
	0x1E7,
	0x1EF,
	0x1F7,
	0x1FE,
	0x1CF,
	0x152,
	0x134,
	0x1F8,
	0x151,
	0x1F5,
	0x19D,
	0x175,
	0x100,
/* G */
	0x104,
	0x113,
	0x11A,
	0x120,
	0x127,
	0x12C,
	0x132,
	0x136,
	0x13F,
	0x147,
	0x150,
	0x159,
	0x160,
	0x168,
	0x171,
	0x17B,
	0x182,
	0x189,
	0x191,
	0x198,
	0x1A0,
	0x1A8,
	0x1B0,
	0x1B8,
	0x1C1,
	0x1C9,
	0x1D0,
	0x1D7,
	0x1E0,
	0x1E7,
	0x1EF,
	0x1F7,
	0x1FE,
	0x1CF,
	0x152,
	0x134,
	0x1F8,
	0x151,
	0x1F5,
	0x19D,
	0x175,
	0x100,
/* B */
	0x104,
	0x113,
	0x11A,
	0x120,
	0x127,
	0x12C,
	0x132,
	0x136,
	0x13F,
	0x147,
	0x150,
	0x159,
	0x160,
	0x168,
	0x171,
	0x17B,
	0x182,
	0x189,
	0x191,
	0x198,
	0x1A0,
	0x1A8,
	0x1B0,
	0x1B8,
	0x1C1,
	0x1C9,
	0x1D0,
	0x1D7,
	0x1E0,
	0x1E7,
	0x1EF,
	0x1F7,
	0x1FE,
	0x1CF,
	0x152,
	0x134,
	0x1F8,
	0x151,
	0x1F5,
	0x19D,
	0x175,
	0x100,
};

/* Truly TFT1P3624-E WVGA LCD cmd on */
static u16 truly_1p3624_spi_cmd_on1[] = {
/*Adjust parameter in 0x36H can turn over Gate and source.
*      0x36,
*      0x80,
*/
	0x3A,
	0x177,
	0X11,
};
/* Truly TFT1P3624-E WVGA LCD cmd on */
static u16 truly_1p3624_spi_cmd_on2[] = {
	0x29,
	0x2C,
};

/* Truly TFT1P3624-E WVGA LCD cmd off */
static u16 truly_1p3624_spi_cmd_off[] = {
	0x10,
};
/******************** End of Truly TFT1P3624 settings**********************/

struct fb_videomode video_modes_tc3587[] = {
	[0] = {
		.pixclock       = 41701,
		.refresh        = 60,
		.xres           = 480,
		.yres           = 640,
		.hsync_len      = 19,
		.left_margin    = 40,
		.right_margin   = 59,
		.vsync_len      = 9,
		.upper_margin   = 4,
		.lower_margin   = 9,
		.sync           = 0,
	},
};

struct fb_videomode video_modes_trulywvga[] = {
	[0] = {
		.pixclock = 41701,
		.refresh = 60,
		.xres = 480,
		.yres = 800,
		.hsync_len = 19,
		.left_margin = 40,
		.right_margin = 59,
		.vsync_len = 9,
		.upper_margin = 4,
		.lower_margin = 9,
		.sync = 0,
	},
};

struct fb_videomode video_modes_truly1p3624[] = {
	[0] = {
		.pixclock = 41028,
		.refresh = 60,
		.xres = 480,
		.yres = 800,
		.hsync_len = 14,
		.left_margin = 5,
		.right_margin = 5,
		.vsync_len = 2,
		.upper_margin = 2,
		.lower_margin = 2,
		.sync = 0,
	},
};

struct fb_videomode video_modes_si9226[] = {
	[0] = {
		.pixclock       = 74170,
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 720,
		.hsync_len      = 0x45, /*HSW*/
		.left_margin    = 0xf9, /*BLW*/
		.right_margin   = 0xfa, /*ELW*/
		.vsync_len      = 0x1, /*VSW*/
		.upper_margin   = 0x12, /*BFW*/
		.lower_margin   = 0x9, /*EFW*/
		.sync           = 0,
	},
};

struct fb_videomode video_modes_adv7533[] = {
	[0] = {
		.pixclock       = 41701,
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.hsync_len      = 96, /*HSW*/
		.left_margin    = 48, /*BLW*/
		.right_margin   = 16, /*ELW*/
		.vsync_len      = 2, /*VSW*/
		.upper_margin   = 33, /*BFW*/
		.lower_margin   = 10, /*EFW*/
		.sync           = 0,
	},
};

/* refer to EIA-CEA-861-D format list*/
struct fb_videomode video_modes_ihdmi[] = {
	[0] = { /* 640x480p */
		.pixclock		= 25170,
		.refresh		= 60,
		.xres			= 640,
		.yres			= 480,
		.hsync_len		= 96, /*HSW*/
		.left_margin	= 48, /*BLW*/
		.right_margin	= 16, /*ELW*/
		.vsync_len		= 2, /*VSW*/
		.upper_margin	= 33, /*BFW*/
		.lower_margin	= 10, /*EFW*/
		/*(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),*/
		.sync			= 0,
	},
	[1] = { /* 720x480p */
		.pixclock		= 27000,
		.refresh		= 60,
		.xres			= 720,
		.yres			= 480,
		.hsync_len		= 62, /*HSW*/
		.left_margin	= 60, /*BLW*/
		.right_margin	= 16, /*ELW*/
		.vsync_len		= 6, /*VSW*/
		.upper_margin	= 30, /*BFW*/
		.lower_margin	= 9, /*EFW*/
		/*(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),*/
		.sync			= 0,
	},
	[2] = { /* 720x480p */
		.pixclock		= 27000,
		.refresh		= 60,
		.xres			= 720,
		.yres			= 480,
		.hsync_len		= 62, /*HSW*/
		.left_margin	= 60, /*BLW*/
		.right_margin	= 16, /*ELW*/
		.vsync_len		= 6, /*VSW*/
		.upper_margin	= 30, /*BFW*/
		.lower_margin	= 9, /*EFW*/
		/*(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),*/
		.sync			= 0,
	},
	[3] = { /* 1280x720p */
		.pixclock		= 74170,
		.refresh		= 60,
		.xres			= 1280,
		.yres			= 720,
		.hsync_len		= 40, /*HSW*/
		.left_margin	= 220, /*BLW*/
		.right_margin	= 110, /*ELW*/
		.vsync_len		= 5, /*VSW*/
		.upper_margin	= 20, /*BFW*/
		.lower_margin	= 5, /*EFW*/
		/*(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),*/
		.sync			= 0,
	},
	[15] = { /* 1080p */
		.pixclock		= 148350,
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1080,
		.hsync_len		= 44, /*HSW*/
		.left_margin	= 148, /*BLW*/
		.right_margin	= 88, /*ELW*/
		.vsync_len		= 5, /*VSW*/
		.upper_margin	= 36, /*BFW*/
		.lower_margin	= 4, /*EFW*/
		/*(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT),*/
		.sync			= 0,
	},
};

static struct ssp_device *ssp_lcd_init(int port, u32 flag)
{
	struct ssp_device *ssp = pxa_ssp_request(port, "SSP");
	if (ssp != NULL){
		clk_enable(ssp->clk);
		/*disable SSP*/
		pxa_ssp_disable(ssp);

		/* set up port type, speed, port settings */
		pxa_ssp_write_reg(ssp, SSCR1, 0x18);
		pxa_ssp_write_reg(ssp, SSPSP, 0);
		pxa_ssp_write_reg(ssp, SSCR0, flag & (~SSCR0_SSE));
		pxa_ssp_enable(ssp);
	}

	return ssp;
}

static void ssp_lcd_deinit(struct ssp_device *ssp)
{
	/*disable SSP*/
	pxa_ssp_disable(ssp);

	clk_disable(ssp->clk);
	pxa_ssp_free(ssp);
}

static void ssp_lcd_send_cmd_para(
	struct ssp_device *ssp, u16 *cmd, int num, u32 mask)
{
	int i;
	for (i = 0; i < num; i++, cmd++) {
		pxa_ssp_write_word(ssp, *cmd & mask);
		pxa_ssp_flush(ssp);
	}
}

void panel_power_tc3587(int ssp_port, int on)
{
	struct ssp_device *ssp = ssp_lcd_init(ssp_port, 0x00000588);

	if (on) {
		/* SLPOUT*/
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_slpout), 0x1ff);
		msleep(120);
		/* DISON */
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_dison), 0x1ff);
		msleep(100);
	} else {
		/* DISOFF  */
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_disoff), 0x1ff);
		msleep(60);
		/* SLPIN */
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_slpin), 0x1ff);
	}

	ssp_lcd_deinit(ssp);
}

void panel_power_trulywvga(int ssp_port, int on)
{
	struct ssp_device *ssp = ssp_lcd_init(ssp_port, 0x0000058F);

	if (on) {
		/* INIT */
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_init), 0xffff);
		msleep(200);
		/* SLPOUT*/
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_slpout), 0xffff);
		msleep(120);
		/* DISON */
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_dison), 0xffff);
		msleep(100);
	} else {
		/* DISOFF  */
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_disoff), 0xffff);
		msleep(60);
		/* SLPIN */
		ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_slpin), 0xffff);
	}

	ssp_lcd_deinit(ssp);
}


/* TODO: extention panel set might be added */
void panel_set_trulywvga(int ssp_port)
{
	struct ssp_device *ssp = ssp_lcd_init(ssp_port, 0x0000058F);
	ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(lcd_panel_rotate_0), 0xffff);
	pr_info("panel_set setting orientation\n");
	ssp_lcd_deinit(ssp);
}

void panel_power_truly1p3624(int ssp_port, int on)
{
	struct ssp_device *ssp = ssp_lcd_init(ssp_port, 0x00000588);
	if (on) {
		 /* Init */
		 ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(truly_1p3624_spi_cmdon), 0x1ff);

		 /* Set gamma */
		 ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(truly_1p3624_spi_cmd_gamma), 0x1ff);
		 msleep(10);

		 /* Set color */
		 ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(truly_1p3624_spi_cmd_color), 0x1ff);
		 msleep(10);

		 /* command on 1*/
		 ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(truly_1p3624_spi_cmd_on1), 0x1ff);
		 msleep(120);

		 /* command on 2*/
		 ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(truly_1p3624_spi_cmd_on2), 0x1ff);
	} else
		 /* power off */
		 ssp_lcd_send_cmd_para(ssp, ARRAY_AND_SIZE(truly_1p3624_spi_cmd_off), 0x1ff);

	ssp_lcd_deinit(ssp);
	return;
}
