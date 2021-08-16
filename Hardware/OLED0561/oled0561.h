#ifndef __OLED_H
#define __OLED_H	 
#include "sys.h"
#include "i2c.h"

#define OLED0561_ADD	0x78  // OLED的I2C地址（禁止修改）
#define COM				0x00  // OLED 指令（禁止修改）
#define DAT 			0x40  // OLED 数据（禁止修改）

void OLED0561_Init(void);//初始化
void OLED_DISPLAY_ON (void);//OLED屏开显示
void OLED_DISPLAY_OFF (void);//OLED屏关显示
void OLED_DISPLAY_LIT (u8 x);//OLED屏亮度设置（0~255）
void OLED_DISPLAY_CLEAR(void);//清屏操作
void OLED_DISPLAY_8x16(u8 x,u8 y,u16 w);//显示8x16的单个字符 
void ANTI_OLED_DISPLAY_8x16(u8 x,u8 y,u16 w); //取反ANTI显示单个英文数字 
void OLED_DISPLAY_8x16_BUFFER(u8 row,u8 *str);//显示8x16的字符串
void ANTI_OLED_DISPLAY_8x16_BUFFER(u8 row,u8 *str);	//反码显示8x16的字符串

void OLED_DISPLAY_16x16(u8 x,u8 y,uc8 *china); //汉字显示
void ANTI_OLED_DISPLAY_16x16(u8 x,u8 y,uc8 *china);//ANTI取反显示 ,显示汉字的页坐标（从0xB0到0xB7） 


void OLED_DISPLAY_Buff_16x16(u8 x,u8 y,uc8 *china,u8 num); //显示一串汉字,num定义显示的汉字个数
void ANTI_OLED_DISPLAY_Buff_16x16(u8 x,u8 y,uc8 *china,u8 num); //反白背景显示一串汉字,num定义显示的汉字个数

void OLED_DISPLAY_PIC1(void);//图片显示

void OLED_DISPLAY_CLEAR_OneLine(u8 l);//只清屏一行 行数0,2,4,6		 				    
#endif
