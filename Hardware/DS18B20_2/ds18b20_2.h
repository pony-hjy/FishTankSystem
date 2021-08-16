#ifndef __DS18B20_2_H
#define __DS18B20_2_H 
  
#include "stm32f10x.h"
 
u8 DS18B20_Init_2(void);
u8 DS18B20_Read_Byte_2(void);
u8 DS18B20_Read_Bit_2(void);
u8 DS18B20_Answer_Check_2(void);
void  DS18B20_GPIO_Config_2(void);
void  DS18B20_Mode_IPU_2(void);
void  DS18B20_Mode_Out_2(void);
void  DS18B20_Rst_2(void);
void  DS18B20_Write_Byte_2(u8 dat);
float DS18B20_Get_Temp_2(void);
 
#endif
