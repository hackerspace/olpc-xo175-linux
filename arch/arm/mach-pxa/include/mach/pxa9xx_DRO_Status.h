#ifndef PM_DRO_H
#define PM_DRO_H

#include <linux/io.h>

#define PXA9XX__POWER_DEBUG_NAME	"PM"
#define USER_BUF_SIZE	50

#define DRO_CNT_WIDTH_NEW	32
#define DRO_MAX_NUM	32

typedef struct {
	char chDroGroup[5];
	char chDroType[5];
	unsigned int wDrosNum;
	unsigned int wBitPosition;
	unsigned int wCnt;
	unsigned int fFreq;
} sDroChainInfo;

unsigned int wDroCntWidth;
unsigned int iwMaxDroChain;
sDroChainInfo sDrosChainInfo[DRO_MAX_NUM];
char wTmpData[DRO_MAX_NUM][DRO_CNT_WIDTH_NEW + 1];	/*allow max cnt width of 32 although it is actually 16. allow max of 32 counters */
void __iomem *DRO_base;
void __iomem *Vmeta_base;
void __iomem *BPB_PRID;
void __iomem *MPMU_CSER_ADDR;
#define DRO_JCON_REG_ADDR	(DRO_base + 0x0)
#define DRO_SW_ACT_ADDR	(DRO_base + 0x4)
#define DRO_RES_RD_ADDR	(DRO_base + 0x8)
#define DRO_START_CNT_ADDR	(DRO_base + 0xC)
#define BPMU_VMPWR_ADDR	(Vmeta_base + 0x0)

unsigned int wDroInfo[] = {	/*Excpet first two, ACISII table of name */
	0x10,			/*cnt width */
	0x10,			/*max dro chain */
	0x00005242, /*BR*/
	0x00545648, /*HVT*/
	0x00504F54, /*TOP*/
	0x00545648, /*HVT*/
	0x00004247, /*GB*/
	0x00545648, /*HVT*/
	0x0057454E, /*NEW*/
	0x00545648, /*HVT*/
	0x0057454E, /*NEW*/
	0x00545648, /*HVT*/
	0x0057454E, /*NEW*/
	0x00545648, /*HVT*/
	0x0057454E, /*NEW*/
	0x00545648, /*HVT*/
	0x0057454E, /*NEW*/
	0x00545648, /*HVT*/
	0x0,
	0x0,
	0x0,
	0x0,
	0x00005242, /*BR*/
	0x0054564C, /*LVT*/
	0x00504F54, /*TOP*/
	0x0054564C, /*LVT*/
	0x00004247, /*GB*/
	0x0054564C, /*LVT*/
	0x0057454E, /*NEW*/
	0x0054564C, /*LVT*/
	0x0057454E, /*NEW*/
	0x0054564C, /*LVT*/
	0x0057454E, /*NEW*/
	0x0054564C, /*LVT*/
	0x0057454E, /*NEW*/
	0x0054564C, /*LVT*/
	0x0057454E, /*NEW*/
	0x0054564C, /*LVT*/
	0x0,
	0x0,
	0x0,
	0x0,
	0x00005242, /*BR*/
	0x00545653, /*SVT*/
	0x00504F54, /*TOP*/
	0x00545653, /*SVT*/
	0x00004247, /*GB*/
	0x00545653, /*SVT*/
	0x4F525056, /*VPRO*/
	0x54564C4E, /*NLVT*/
	0x45524F43, /*CORE*/
	0x54564C4E, /*NLVT*/
	0x0057454E, /*NEW*/
	0x00545653, /*SVT*/
	0x0057454E, /*NEW*/
	0x00545653, /*SVT*/
	0x0057454E, /*NEW*/
	0x00545653, /*SVT*/
	0x0057454E, /*NEW*/
	0x00545653, /*VT*/
	0x0057454E, /*NEW*/
	0x00545653, /*SVT*/
	0x0,
	0x0,
	0x0,
	0x0
};

unsigned int droBin2Hex(char *chData)
{
	unsigned int PowOfTwo = 32768;
	int i;
	unsigned int wHexValue = 0;
	for (i = 0; i < wDroCntWidth; i++) {
		if (chData[i] == '1')
			wHexValue += PowOfTwo;
		PowOfTwo /= 2;
	}
	return wHexValue;
}

#endif
