#ifndef __FLASH_H
#define __FLASH_H 			   
#include "sys.h"
 

void FLASH_W(u32 add,u16 dat);
u16 FLASH_R(u32 add);
void FLASH_W_Buff(u32 add,u16 *dat,u16 WriteNum); //FLASHд�������� ����1��32λFLASH��ַ������2��16λ���� ����3: ��Ҫд�����������

#endif





























