#ifndef _AGS02MA_h_
#define _AGS02MA_h_

#define TVOC_PPB		0
#define TVOC_ug			1

void Delay1ms(unsigned int u32CNT);
void VOC_Init(void);
unsigned int Get_VOC_Data(unsigned char type);

#endif 
