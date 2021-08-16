#include "DS18B20_2.h"
#include "Delay.h"
#include "stdio.h" // printf��
 
#define DS18B20_GPIO_NUM_2					GPIO_Pin_8
#define DS18B20_GPIO_X_2					GPIOB
#define RCC_APB2Periph_DS18B20_GPIO_X_2		RCC_APB2Periph_GPIOB

#define DS18B20_DQ_OUT_Low_2			GPIO_ResetBits(DS18B20_GPIO_X_2,DS18B20_GPIO_NUM_2) 
#define DS18B20_DQ_OUT_High_2			GPIO_SetBits(DS18B20_GPIO_X_2,DS18B20_GPIO_NUM_2) 
#define DS18B20_DQ_IN_2					GPIO_ReadInputDataBit(DS18B20_GPIO_X_2,DS18B20_GPIO_NUM_2) 

// ����DS18B20�õ���I/O��
void DS18B20_GPIO_Config_2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_DS18B20_GPIO_X_2, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO_NUM_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DS18B20_GPIO_X_2, &GPIO_InitStructure);
	GPIO_SetBits(DS18B20_GPIO_X_2, DS18B20_GPIO_NUM_2);
}
 
// ��������
void DS18B20_Mode_IPU_2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO_NUM_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DS18B20_GPIO_X_2, &GPIO_InitStructure);
}
 
// �������
void DS18B20_Mode_Out_2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO_NUM_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DS18B20_GPIO_X_2, &GPIO_InitStructure);
 
}
 
// ��λ���������ӻ����͸�λ����
void DS18B20_Rst_2(void)
{
	DS18B20_Mode_Out_2();
	DS18B20_DQ_OUT_Low_2;		// ��������480us�ĵ͵�ƽ��λ�ź�
	delay_us(480);
	DS18B20_DQ_OUT_High_2;	// �ڲ�����λ�źź��轫��������
	delay_us(15);
}
 
// ���ӻ����������ص�Ӧ�����塣�ӻ����յ������ĸ�λ�źź󣬻���15~60us���������һ��Ӧ������
u8 DS18B20_Answer_Check_2(void)
{
	u8 delay = 0;
	DS18B20_Mode_IPU_2(); // ��������Ϊ��������
	// �ȴ�Ӧ�����壨һ��60~240us�ĵ͵�ƽ�ź� ���ĵ���
	// ���100us�ڣ�û��Ӧ�����壬�˳�������ע�⣺�ӻ����յ������ĸ�λ�źź󣬻���15~60us���������һ����������
	while (DS18B20_DQ_IN_2&&delay < 100)
	{
		delay++;
		delay_us(1);
	}
	// ����100us�����û��Ӧ�����壬�˳�����
	if (delay >= 100)//Hu200
		return 1;
	else
		delay = 0;
	// ��Ӧ�����壬�Ҵ���ʱ�䲻����240us
	while (!DS18B20_DQ_IN_2&&delay < 240)
	{
		delay++;
		delay_us(1);
	}
	if (delay >= 240)
		return 1;
	return 0;
}

 
// ��DS18B20��ȡ1��λ
u8 DS18B20_Read_Bit_2(void)
{
	u8 data;
	DS18B20_Mode_Out_2();
	DS18B20_DQ_OUT_Low_2; // ��ʱ�����ʼ���������������� >1us <15us �ĵ͵�ƽ�ź�
	delay_us(2);
	DS18B20_DQ_OUT_High_2;
	delay_us(12);
	DS18B20_Mode_IPU_2();// ���ó����룬�ͷ����ߣ����ⲿ�������轫��������
	if (DS18B20_DQ_IN_2)
		data = 1;
	else
		data = 0;
	delay_us(50);
	return data;
}
 
// ��DS18B20��ȡ2��λ
u8 DS18B20_Read_2Bit_2(void)//����λ �ӳ���
{
	u8 i;
	u8 dat = 0;
	for (i = 2; i > 0; i--)
	{
		dat = dat << 1;
		DS18B20_Mode_Out_2();
		DS18B20_DQ_OUT_Low_2;
		delay_us(2);
		DS18B20_DQ_OUT_High_2;
		DS18B20_Mode_IPU_2();
		delay_us(12);
		if (DS18B20_DQ_IN_2)	dat |= 0x01;
		delay_us(50);
	}
	return dat;
}
 
// ��DS18B20��ȡ1���ֽ�
u8 DS18B20_Read_Byte_2(void)	// read one byte
{
	u8 i, j, dat;
	dat = 0;
	for (i = 0; i < 8; i++)
	{
		j = DS18B20_Read_Bit_2();
		dat = (dat) | (j << i);
	}
	return dat;
}
 
// д1λ��DS18B20
void DS18B20_Write_Bit_2(u8 dat)
{
	DS18B20_Mode_Out_2();
	if (dat)
	{
		DS18B20_DQ_OUT_Low_2;// Write 1
		delay_us(2);
		DS18B20_DQ_OUT_High_2;
		delay_us(60);
	}
	else
	{
		DS18B20_DQ_OUT_Low_2;// Write 0
		delay_us(60);
		DS18B20_DQ_OUT_High_2;
		delay_us(2);
	}
}
 
// д1�ֽڵ�DS18B20
void DS18B20_Write_Byte_2(u8 dat)
{
	u8 j;
	u8 testb;
	DS18B20_Mode_Out_2();
	for (j = 1; j <= 8; j++)
	{
		testb = dat & 0x01;
		dat = dat >> 1;
		if (testb)
		{
			DS18B20_DQ_OUT_Low_2;// д1
			delay_us(10);
			DS18B20_DQ_OUT_High_2;
			delay_us(50);
		}
		else
		{
			DS18B20_DQ_OUT_Low_2;// д0
			delay_us(60);
			DS18B20_DQ_OUT_High_2;// �ͷ�����
			delay_us(2);
		}
	}
}
 
//��ʼ��DS18B20��IO�ڣ�ͬʱ���DS�Ĵ���
u8 DS18B20_Init_2(void)
{
	DS18B20_GPIO_Config_2();
	DS18B20_Rst_2();
	return DS18B20_Answer_Check_2();
}
 
// ��ds18b20�õ��¶�ֵ�����ȣ�0.1C�������¶�ֵ��-55~125����Temperature1���ظ���ʵ���¶� ��: 33.36
// ����i ��ʾ֮ǰ�����ķ���DS18B20_ID[MaxSensorNum][8];����ĵ�?λID
float DS18B20_Get_Temp_2()
{
	//u8 flag;
	u8 TL, TH;
	short Temperature;
	float Temperature1;
	DS18B20_Rst_2();
	DS18B20_Answer_Check_2();
	DS18B20_Write_Byte_2(0xcc);// skip rom
	DS18B20_Write_Byte_2(0x44);// convert
	DS18B20_Rst_2();
	DS18B20_Answer_Check_2();

	DS18B20_Write_Byte_2(0xcc); // skip rom 
	DS18B20_Write_Byte_2(0xbe);// convert
	TL = DS18B20_Read_Byte_2(); // LSB   
	TH = DS18B20_Read_Byte_2(); // MSB  
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
 
