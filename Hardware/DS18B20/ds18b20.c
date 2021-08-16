#include "DS18B20.h"
#include "Delay.h"
#include "stdio.h" // printf用
 
#define DS18B20_GPIO_NUM				GPIO_Pin_2
#define DS18B20_GPIO_X					GPIOB
#define RCC_APB2Periph_DS18B20_GPIO_X	RCC_APB2Periph_GPIOB

#define DS18B20_DQ_OUT_Low			GPIO_ResetBits(DS18B20_GPIO_X,DS18B20_GPIO_NUM) 
#define DS18B20_DQ_OUT_High			GPIO_SetBits(DS18B20_GPIO_X,DS18B20_GPIO_NUM) 
#define DS18B20_DQ_IN				GPIO_ReadInputDataBit(DS18B20_GPIO_X,DS18B20_GPIO_NUM) 

// 配置DS18B20用到的I/O口
void DS18B20_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_DS18B20_GPIO_X, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO_NUM;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DS18B20_GPIO_X, &GPIO_InitStructure);
	GPIO_SetBits(DS18B20_GPIO_X, DS18B20_GPIO_NUM);
}
 
// 引脚输入
void DS18B20_Mode_IPU(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO_NUM;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DS18B20_GPIO_X, &GPIO_InitStructure);
}
 
// 引脚输出
void DS18B20_Mode_Out(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO_NUM;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DS18B20_GPIO_X, &GPIO_InitStructure);
 
}
 
// 复位，主机给从机发送复位脉冲
void DS18B20_Rst(void)
{
	DS18B20_Mode_Out();
	DS18B20_DQ_OUT_Low;		// 产生至少480us的低电平复位信号
	delay_us(480);
	DS18B20_DQ_OUT_High;	// 在产生复位信号后，需将总线拉高
	delay_us(15);
}
 
// 检测从机给主机返回的应答脉冲。从机接收到主机的复位信号后，会在15~60us后给主机发一个应答脉冲
u8 DS18B20_Answer_Check(void)
{
	u8 delay = 0;
	DS18B20_Mode_IPU(); // 主机设置为上拉输入
	// 等待应答脉冲（一个60~240us的低电平信号 ）的到来
	// 如果100us内，没有应答脉冲，退出函数，注意：从机接收到主机的复位信号后，会在15~60us后给主机发一个存在脉冲
	while (DS18B20_DQ_IN&&delay < 100)
	{
		delay++;
		delay_us(1);
	}
	// 经过100us后，如果没有应答脉冲，退出函数
	if (delay >= 100)//Hu200
		return 1;
	else
		delay = 0;
	// 有应答脉冲，且存在时间不超过240us
	while (!DS18B20_DQ_IN&&delay < 240)
	{
		delay++;
		delay_us(1);
	}
	if (delay >= 240)
		return 1;
	return 0;
}

 
// 从DS18B20读取1个位
u8 DS18B20_Read_Bit(void)
{
	u8 data;
	DS18B20_Mode_Out();
	DS18B20_DQ_OUT_Low; // 读时间的起始：必须由主机产生 >1us <15us 的低电平信号
	delay_us(2);
	DS18B20_DQ_OUT_High;
	delay_us(12);
	DS18B20_Mode_IPU();// 设置成输入，释放总线，由外部上拉电阻将总线拉高
	if (DS18B20_DQ_IN)
		data = 1;
	else
		data = 0;
	delay_us(50);
	return data;
}
 
// 从DS18B20读取2个位
u8 DS18B20_Read_2Bit(void)//读二位 子程序
{
	u8 i;
	u8 dat = 0;
	for (i = 2; i > 0; i--)
	{
		dat = dat << 1;
		DS18B20_Mode_Out();
		DS18B20_DQ_OUT_Low;
		delay_us(2);
		DS18B20_DQ_OUT_High;
		DS18B20_Mode_IPU();
		delay_us(12);
		if (DS18B20_DQ_IN)	dat |= 0x01;
		delay_us(50);
	}
	return dat;
}
 
// 从DS18B20读取1个字节
u8 DS18B20_Read_Byte(void)	// read one byte
{
	u8 i, j, dat;
	dat = 0;
	for (i = 0; i < 8; i++)
	{
		j = DS18B20_Read_Bit();
		dat = (dat) | (j << i);
	}
	return dat;
}
 
// 写1位到DS18B20
void DS18B20_Write_Bit(u8 dat)
{
	DS18B20_Mode_Out();
	if (dat)
	{
		DS18B20_DQ_OUT_Low;// Write 1
		delay_us(2);
		DS18B20_DQ_OUT_High;
		delay_us(60);
	}
	else
	{
		DS18B20_DQ_OUT_Low;// Write 0
		delay_us(60);
		DS18B20_DQ_OUT_High;
		delay_us(2);
	}
}
 
// 写1字节到DS18B20
void DS18B20_Write_Byte(u8 dat)
{
	u8 j;
	u8 testb;
	DS18B20_Mode_Out();
	for (j = 1; j <= 8; j++)
	{
		testb = dat & 0x01;
		dat = dat >> 1;
		if (testb)
		{
			DS18B20_DQ_OUT_Low;// 写1
			delay_us(10);
			DS18B20_DQ_OUT_High;
			delay_us(50);
		}
		else
		{
			DS18B20_DQ_OUT_Low;// 写0
			delay_us(60);
			DS18B20_DQ_OUT_High;// 释放总线
			delay_us(2);
		}
	}
}
 
//初始化DS18B20的IO口，同时检测DS的存在
u8 DS18B20_Init(void)
{
	DS18B20_GPIO_Config();
	DS18B20_Rst();
	return DS18B20_Answer_Check();
}
 
// 从ds18b20得到温度值，精度：0.1C，返回温度值（-55~125），Temperature1返回浮点实际温度 如: 33.36
// 参数i 表示之前读到的放在DS18B20_ID[MaxSensorNum][8];里面的第?位ID
float DS18B20_Get_Temp()
{
	//u8 flag;
	u8 TL, TH;
	short Temperature;
	float Temperature1;
	DS18B20_Rst();
	DS18B20_Answer_Check();
	DS18B20_Write_Byte(0xcc);// skip rom
	DS18B20_Write_Byte(0x44);// convert
	DS18B20_Rst();
	DS18B20_Answer_Check();

	DS18B20_Write_Byte(0xcc); // skip rom 
	DS18B20_Write_Byte(0xbe);// convert
	TL = DS18B20_Read_Byte(); // LSB   
	TH = DS18B20_Read_Byte(); // MSB  
	if (TH & 0xfc)
	{
		//flag=1;
		Temperature = (TH << 8) | TL;
		Temperature1 = (~Temperature) + 1;
		Temperature1 = (Temperature1 * 0.0625) * 10;
	}
	else
	{
		//flag=0;
		Temperature1 = (((TH << 8) | TL)*0.0625) * 10;
	}
	return Temperature1;
}
 
