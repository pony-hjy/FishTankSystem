#ifndef __DS18B20_H
#define __DS18B20_H 
  
#include "stm32f10x.h"
 
u8 DS18B20_Init(void);
u8 DS18B20_Read_Byte(void);
u8 DS18B20_Read_Bit(void);
u8 DS18B20_Answer_Check(void);
void  DS18B20_GPIO_Config(void);
void  DS18B20_Mode_IPU(void);
void  DS18B20_Mode_Out(void);
void  DS18B20_Rst(void);
void  DS18B20_Search_Rom(void);
void  DS18B20_Write_Byte(u8 dat);
float DS18B20_Get_Temp(void);
 
#endif
