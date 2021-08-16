#ifndef __FLASH_H
#define __FLASH_H 			   
#include "sys.h"
 

void FLASH_W(u32 add,u16 dat);
u16 FLASH_R(u32 add);
void FLASH_W_Buff(u32 add,u16 *dat,u16 WriteNum); //FLASH写入多个数据 参数1：32位FLASH地址。参数2：16位数据 参数3: 需要写入的数据数量

#endif





























