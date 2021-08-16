/*********************************************************************************************
ģ��������  Damon
��������	�������ϵͳ
��д�ˣ�	������	
��дʱ�䣺	2021��08��03��
Ӳ��֧�֣�	STM32F103C8  �ⲿ����8MHz RCC����������Ƶ72MHz ���ⲿ���پ���32.768KHz  

�޸���־������
1-202108030916 ������������͸�����,�����жϴ���,ʵ����/������İ����л�	
2-202108031452 �������ò˵�����,�Ӽ��������жϴ�����	
3-202108031818 �������¶ȼ��ľ�������,�¶�ֵ�����ں󱸼Ĵ���,ˮ�������޿��Ƽ̵�������
4-202108041242 �������ò˵���ѡ��ʱ�����ù���,
5-202108051036 ���뵥���߶������¶ȴ���������,
6-202108051832 �������ܵƹ��Ӳ˵�,ѡ�����ò˵��ȴ���һ�μ���
7-202108052205 ���������ܵƹ�˵��´��������
8-202108061144 �Ż��˰����жϺ���,���ڸ�����Ч��
9-202108062044 ���뿪�������ٽ�ֵ���ý���
10-202108071655 �����˹���ˮ�����ý��沢����ж������Կ��Ƽ̵���2ʵ��ˮ�õĿ����͹ر�
11-202108091451 �����Զ�Ͷι�˵����жϴ�������
12-202108101703 �ѵ����߶�����¶ȴ������ĳɶ��io�ڶ�ȡ��ʽ,flash��ַ�ĵ���ȷ��ַ��63ҳ,֮ǰ��127ҳ.
							
˵����	A�����л�����, 
		B�������ݼ�����,�������ټ�.
		C�������ݼӼ�,�������ټ�.
		D���ǽ���/�˳����ð���


*********************************************************************************************/
#include "stm32f10x.h" //STM32ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "CHS_16x16.h" //�����ֿ� 

#include "oled0561.h"
#include "ds18b20.h"
#include "ds18b20_2.h"
#include "rtc.h"
#include "touch_key.h"
#include "NVIC.h"
#include "relay.h"
#include "adc.h"
#include "flash.h"
#include "pwm.h"
#include "ChangeWatIoConfig.h"

#define FLASH_START_ADDR  0X0800FC00	  //д�����ʼ��ַ ��63ҳ  ��64k
//#define FLASH_START_ADDR  0x0801f000	  //д�����ʼ��ַ
#define WriteNum  43 //��Ҫд�����������, ÿ��д��һ������,��Ӧ������ֵҲҪ�޸�
#define WriteFlashBuffNum  43 //��Ҫд������黺����������, ÿ��д��һ������,��Ӧ������ֵҲҪ�޸�

extern unsigned char DS18B20_ID[2][8];//��ż�⵽�Ĵ�����ID������
extern unsigned char DS18B20_SensorNum; //��⵽��ds18b20����

int main (void){//������
	u8 MENU; //���˵�ֵ
	short Temp;//��һ���¶ȴ�����
	short Temp2; // �ڶ����¶ȴ�����
	u8 TemFreFlag; //�¶�����ˢ�±�־λ
	u8 TemCoun;	//�¶�����ˢ����ʱ����
	u8 MENUSET;	//���ò˵�ֵ
	u8 SUBMENU;	//�Ӳ˵�ֵ
	u8 WatTemMod; //ˮ��ģʽ��־λ
	u8 MaxWatTemp;//ˮ������ֵ
	u8 MinWatTemp;//ˮ������ֵ
	u8 R1Flag; //�̵���������־λ
	u8 TimeShift; //ʱ�������л�ѡ���޸ı�־λ
	u8 WriteTime; //д��ʱ���ú��ʱ�������־λ
	u16	ADC1buf[10]; //���ڴ�Ŷ�10�ε�ADCֵ
	u16	MinADC;//�����ҳ�10��ADCֵ����Сֵ
	u16	MaxADC;//�����ҳ�10��ADCֵ�����ֵ
	u16 EquaADC;//���ڰ�10��ADCֵȡƽ����
	u16 i; //ͨ�ñ���
	u8 LightMod; //���ܵƹ�ģʽ��־λ
	u8 LightShift; //���ܵƹ�˵���ʱ������ʱ�������л���־λ
	u8 DayTimeStartHour; //���ܵƹ�˵��°������俪ʼСʱʱ��ֵ
	u8 DayTimeStartMin;	//���ܵƹ�˵��°������俪ʼ����ʱ��ֵ
	u8 DayTimeEndHour;//���ܵƹ�˵��°����������Сʱʱ��ֵ  ������Ϊ�������俪ʼСʱֵ
	u8 DayTimeEndMin; //���ܵƹ�˵��°��������������ʱ��ֵ  ������Ϊ�������俪ʼ����ֵ
	u8 DayTimeGear;	//���ܵƹ�˵��°�������ƹ⵵λֵ
	u16 WriteFlashBuff[WriteFlashBuffNum] = {0}; //д��flash�����ݻ���,��Ҫдflash�����ݷ���������,д��flash��������һ����д������,��������ͬ��һ�����������ݲ��ᶪʧ		  
	u8 DuskTimeEndHour;//���ܵƹ�˵��°������俪ʼСʱʱ��ֵ
	u8 DuskTimeEndMin; //���ܵƹ�˵��°������俪ʼ����ʱ��ֵ
	u8 DuskTimeGear;	//���ܵƹ�˵��°�������ƹ⵵λֵ
	u16 LuxSwitch; //���ܵƹ�˵��´���׵Ƶ������ٽ�ֵ
	u8 Mos1Flag; //���ܵƹ�PWM���Ƶ�mos��1״̬��־λ
	u8 FiltPumpMod; //����ˮ��ģʽ��־λ
	u8 MENUShift; //ͨ�õĲ˵�����ֵѡ�������л���־λ 
	u8 FiltPumpStartHour; //���ܵƹ�˵��°������俪ʼСʱʱ��ֵ
	u8 FiltPumpStartMin;	//���ܵƹ�˵��°������俪ʼ����ʱ��ֵ
	u8 FiltPumpEndHour;//���ܵƹ�˵��°����������Сʱʱ��ֵ  ������Ϊ�������俪ʼСʱֵ
	u8 FiltPumpEndMin; //���ܵƹ�˵��°��������������ʱ��ֵ  ������Ϊ�������俪ʼ����ֵ
	u8 R2Flag; //���ƹ���ˮ�õļ̵���״̬��־λ
	u8 AutoFeedMod; //�Զ�Ͷιģʽֵ
	u8 FeedTimeNum; //����ѡ��1--5��Ͷιʱ����
	u8 FeedTimeHour[6] = {0}; //Ͷιʱ���5�����Сʱ������
	u8 FeedTimeMin[6] = {0}; //Ͷιʱ���5����ŷ��ӵ�����
	u8 FeedTimeLast[6] = {0};//Ͷιʱ���5�����Ͷι����ʱ��������
	u8 FeedTriggerNum;//Ͷιʱ�䴥�����������
	u8 Steer1Flag;//Ͷιʱ�䴥���Ķ��״̬��־λ
	u8 FeedLastCount; //Ͷι����ʱ
	u8 FeedLastCountFlag; //Ͷι����ʱ1��任״̬��־λ
	u8 ChangeWaterMod; //��ˮģʽ��־λ
	u8 ChangWatTimNum; //�����л�ѡ����Զ���ˮʱ���� 1--5�� 
	u8 ChangWatWeek[6]; //������õ��Զ���ˮ������ֵ	    
	u8 ChangWatTimHour[6]; //���1--5�Զ���ˮʱ�����Сʱֵ
	u8 ChangWatTimMin[6];  //���1--5�Զ���ˮʱ����ķ���ֵ
	u8 ChangWatFlag;//��ˮ״̬��־λ
	u8 ChangWatTrigNum;//��Ŵ����Ļ�ˮʱ����
	u8 ChangWatProcess;	//��ˮ���̱�־λ
	u8 SetInstructFlag; //��������ʱ��ʾ����˵����ʾ��־λ

	delay_ms(500); //�ϵ�ʱ�ȴ�������������
	RCC_Configuration(); //ϵͳʱ�ӳ�ʼ�� 
	I2C_Configuration();//I2C��ʼ��
	OLED0561_Init(); //OLED��ʼ��				   
	OLED_DISPLAY_LIT(150);//��������
	DS18B20_Init(); //�¶ȴ�������ʼ��
	RTC_Config(); //ʵʱʱ�ӳ�ʼ��
	RTC_WritBKP_Config(); //дRTC�ı��üĴ���BKP����
	EXti_Config(); //�жϳ�ʼ��
	RELAY_Init();//�̵�����ʼ��
	ADCx_Init();//dma��ȡadc��ʼ��
//	TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,�������ܵƹ�ĵƹ����,����Ƶ��Ϊ11KHz����ʽΪ�����ʱ��Tout����λ�룩=(arr+1)(psc+1)/Tclk Ԥ��Ƶϵ��	= 11KHZ (90us)/101(PWM)*72000000
//                          //TclkΪͨ�ö�ʱ����ʱ�ӣ����APB1û�з�Ƶ�����Ϊϵͳʱ�ӣ�72MHZ			                                       64.15	= 0.00009/101(�Զ�װ��ֵ)*72000000
//                          //PWMʱ��Ƶ��=72000000/(100+1)*(63+1) = 11138.6  11KHZ (90us),�����Զ�װ��ֵ101,Ԥ��Ƶϵ��64
//	TIM3_PWM_CH3_Init(59999,23); //Time3ͨ��3��ʼ��,����Ƶ��Ϊ50Hz�������Զ�Ͷι�Ķ������
	ChangeWatIoConfig();//�Զ���ˮ�������IO�ڳ�ʼ��

	MENU = 1;  //���˵���ʼֵΪ1
	TemFreFlag = 1;	//ˢ���¶ȱ�־λ��ʼֵΪ1
	MENUSET = 1; //���ò˵���ʼ��ֵΪ1
	SUBMENU = 1; //�Ӳ˵�ֵ��ʼֵΪ1
	R1Flag = 0; //�̵���1����״̬��־λ��ʼֵΪ0
	TimeShift = 1; //ʱ�������л�ѡ���޸ı�־λ��ʼֵΪ1
	WriteTime = 0; //д�����ú��ʱ�������־λ��ʼֵΪ0
	LightShift = 1;//���ܵƹ�˵���ʱ������ʱ�������л���־λ��ʼֵΪ1
	Mos1Flag = 0;//���ܵƹ�PWM���Ƶ�mos��1״̬��־λ��ʼֵΪ0
	MENUShift = 1; //ͨ�õĲ˵�����ֵѡ�������л���־λ
	R2Flag = 0; //���ƹ���ˮ�õļ̵���״̬��־λ
	FeedTimeNum = 1; //����ѡ��1--5��Ͷιʱ���� ��ʼֵΪ1
	FeedTriggerNum = 0;//Ͷιʱ�䴥�����������	��ʼֵΪ0
	Steer1Flag = 0;//Ͷιʱ�䴥���Ķ��״̬��־λ ��ʼֵΪ0
	FeedLastCount = 0; //Ͷι����ʱ	��ʼֵΪ0
	FeedLastCountFlag = 0; //Ͷι����ʱ1��任״̬��־λ ��ʼֵΪ0
	ChangWatTimNum = 1; //�����л�ѡ����Զ���ˮʱ���� 1--5�� ��ʼֵΪ1
	ChangWatFlag = 0;//��ˮ״̬��־λ  ��ʼֵΪ0
	ChangWatTrigNum = 0;//��Ŵ����Ļ�ˮʱ���� ��ʼֵΪ0
	ChangWatProcess	= 1; //��ˮ���̱�־λ ��ʼֵΪ1
	SetInstructFlag = 0; //��������ʱ��ʾ����˵����ʾ��־λ	��ʼֵΪ0

	if(FLASH_R(FLASH_START_ADDR+(0*2)) != 0xfefe){ //��flash��־λ,��������ڱ�־λ0xfefe,��������ݶ�ʧ
	   	WriteFlashBuff[0] = 0xfefe;//д���־λ�Ž�д�뻺������,�ȴ�д��
		DayTimeGear = 85; //��������ƹ⵵λֵ��ʼֵΪ85
		DuskTimeEndHour = 21; //�������ʱ���ʼֵΪ 21:00
		DuskTimeEndMin = 0;	
		DuskTimeGear = 35; //��������ƹ⵵λֵ��ʼֵΪ35
		LuxSwitch = 1300;////�򿪵ƹ������ٽ�ֵ��ʼֵΪ1300
		FiltPumpMod = 0; //����ˮ��ģʽ��־λ��ʼֵΪ0
		FiltPumpStartHour = 8;//����ˮ�õĿ���ʱ�������ʼֵ��: 08:00 - 21:00
		FiltPumpStartMin = 0;
		FiltPumpEndHour = 21;
		FiltPumpEndMin = 0;
		AutoFeedMod = 0; //�Զ�Ͷιģʽֵ��ʼֵΪ0
		for(i=1; i<6; i++){	//�Զ�Ͷι5��ʱ�����ʼֵ��Ϊ  ����: (off)24ʱ:24��  Ͷι����ʱ��10�� 
 		   FeedTimeHour[i] =  24; 
 		   FeedTimeMin[i] =  25; 
 		   FeedTimeLast[i] =  10; 
		}
		ChangeWaterMod = 0; //��ʼ�Զ���ˮģʽֵΪ0
		for(i=1; i<6; i++){	//�Զ���ˮ5��ʱ�����ʼֵ  ���ڶ�Ϊoff��־λ7  ʱ�䶼Ϊ 12:25 
			ChangWatWeek[i] = 7; //
			ChangWatTimHour[i] = 12; //
			ChangWatTimMin[i] = 25; 
		}
		WriteFlashBuff[1] = DayTimeGear; //��������ƹ⵵λֵ��ʼֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[2] = DuskTimeEndHour; //��ʼ�����������Сʱֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[3] = DuskTimeEndMin; //��ʼ���������������ֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[4] = DuskTimeGear; //��ʼ��������ƹ⵵λֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[5] = LuxSwitch; //��ʼ�����ٽ�ֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[6] = FiltPumpMod; //��ʼ����ˮ��ģʽֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[7] = FiltPumpStartHour; //��ʼ����ˮ�ÿ��������俪ʼСʱֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[8] = FiltPumpStartMin; //��ʼ����ˮ�ÿ��������俪ʼ����ֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[9] = FiltPumpEndHour; //��ʼ����ˮ�ÿ������������Сʱֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[10] = FiltPumpEndMin; //��ʼ����ˮ�ÿ����������������ֵ�Ž�д�뻺������,�ȴ�д��
		WriteFlashBuff[11] = AutoFeedMod; //��ʼ�Զ�Ͷιģʽֵ�Ž�д�뻺������,�ȴ�д��
		for(i=1; i<6; i++){	//�Զ�Ͷι5��ʱ�����ʼֵ�ȴ�д��
 		   WriteFlashBuff[11+i] = FeedTimeHour[i]; //�Զ�Ͷι��5��ʱ�����ʼֵ�Ž�д�뻺������,�ȴ�д��
 		   WriteFlashBuff[16+i] = FeedTimeMin[i];
 		   WriteFlashBuff[21+i] = FeedTimeLast[i]; 
		}
		WriteFlashBuff[27] = ChangeWaterMod; //��ʼ�Զ���ˮģʽֵ�Ž�д�뻺������,�ȴ�д��
		for(i=1; i<6; i++){	//�Զ���ˮ5��ʱ�����ʼֵ�ȴ�д�� 
			WriteFlashBuff[27+i] = ChangWatWeek[i]; //��ʼ�Զ���ˮСʱֵ�Ž�д�뻺������,�ȴ�д��
			WriteFlashBuff[32+i] = ChangWatTimHour[i]; //��ʼ����Զ���ˮ����ֵ�Ž�д�뻺������,�ȴ�д��
			WriteFlashBuff[37+i] = ChangWatTimMin[i]; //��ʼ�Զ���ˮ����ʱ��ֵ�Ž�д�뻺������,�ȴ�д��
		}
		FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,������WriteNum����
	}else{ // ��־λ����,��ʾ����û�ж�ʧ
		DayTimeGear = FLASH_R(FLASH_START_ADDR+(1*2));//����flash������
		DuskTimeEndHour = FLASH_R(FLASH_START_ADDR+(2*2));//����flash������
		DuskTimeEndMin = FLASH_R(FLASH_START_ADDR+(3*2));//����flash������
		DuskTimeGear = FLASH_R(FLASH_START_ADDR+(4*2));//����flash������
		LuxSwitch = FLASH_R(FLASH_START_ADDR+(5*2)); //����flash������
		FiltPumpMod = FLASH_R(FLASH_START_ADDR+(6*2)); //����flash������;
		FiltPumpStartHour = FLASH_R(FLASH_START_ADDR+(7*2)); //����flash������; 
		FiltPumpStartMin = FLASH_R(FLASH_START_ADDR+(8*2)); //����flash������
		FiltPumpEndHour = FLASH_R(FLASH_START_ADDR+(9*2)); //����flash������;
		FiltPumpEndMin = FLASH_R(FLASH_START_ADDR+(10*2)); //����flash������;
		AutoFeedMod = FLASH_R(FLASH_START_ADDR+(11*2)); //����flash������;
		for(i=1; i<6; i++){	//���Զ�Ͷι��5��ʱ�����flash����
 		   FeedTimeHour[i] =  FLASH_R(FLASH_START_ADDR+((11+i)*2)); //����flash������;
 		   FeedTimeMin[i] =  FLASH_R(FLASH_START_ADDR+((16+i)*2)); //����flash������;
 		   FeedTimeLast[i] =  FLASH_R(FLASH_START_ADDR+((21+i)*2)); //����flash������;
		}
		ChangeWaterMod = FLASH_R(FLASH_START_ADDR+(27*2)); //����flash������;
		for(i=1; i<6; i++){	//�Զ���ˮ5��ʱ�����flash���� 
			ChangWatWeek[i] = FLASH_R(FLASH_START_ADDR+((27+i)*2)); //����flash������;
			ChangWatTimHour[i] = FLASH_R(FLASH_START_ADDR+((32+i)*2)); //����flash������;
			ChangWatTimMin[i] = FLASH_R(FLASH_START_ADDR+((37+i)*2)); //����flash������;
		}
	}
	 	  
	if(BKP_ReadBackupRegister(BKP_DR2) != 0xfefe){//��BKP�󱸼Ĵ�����־λ,��������ڱ�־λ0xfefe,��������ݶ�ʧ
		BKP_WriteBackupRegister(BKP_DR2, 0xfefe);//��󱸼Ĵ���д���־λ	
		WatTemMod = 0;
		MaxWatTemp = 30;
		MinWatTemp = 15;
		LightMod = 0;//��ʼ���ܵƹ�ģʽֵ		
		DayTimeStartHour = 8;//��������Ŀ�ʼ��������ʼʱ��ֵΪ : 08:00 -- 17: 00 
		DayTimeStartMin = 0;
		DayTimeEndHour = 17;
		DayTimeEndMin = 0;
		BKP_WriteBackupRegister(BKP_DR3, WatTemMod);//��󱸼Ĵ���д���ʼ�¶ȼ��ģʽֵ	
		BKP_WriteBackupRegister(BKP_DR4, MaxWatTemp);//��󱸼Ĵ���д���ʼˮ������ֵ	
		BKP_WriteBackupRegister(BKP_DR5, MinWatTemp);//��󱸼Ĵ���д���ʼˮ������ֵ
		BKP_WriteBackupRegister(BKP_DR6, LightMod);//��󱸼Ĵ���д���ʼ���ܵƹ�ģʽֵ
		BKP_WriteBackupRegister(BKP_DR7, DayTimeStartHour);//��󱸼Ĵ���д���ʼ�������俪ʼСʱֵ	
		BKP_WriteBackupRegister(BKP_DR8, DayTimeStartMin);//��󱸼Ĵ���д���ʼ�������俪ʼ����ֵ	
		BKP_WriteBackupRegister(BKP_DR9, DayTimeEndHour);//��󱸼Ĵ���д���ʼ�����������Сʱֵ	
		BKP_WriteBackupRegister(BKP_DR10, DayTimeEndMin);//��󱸼Ĵ���д����������������ֵ	
	}else{ // ��־λ����,��ʾ����û�ж�ʧ
		WatTemMod = BKP_ReadBackupRegister(BKP_DR3);//�����󱸼Ĵ���
		MaxWatTemp = BKP_ReadBackupRegister(BKP_DR4);//�����󱸼Ĵ���
		MinWatTemp = BKP_ReadBackupRegister(BKP_DR5);//�����󱸼Ĵ���
		LightMod = BKP_ReadBackupRegister(BKP_DR6);//�����󱸼Ĵ���
		DayTimeStartHour = BKP_ReadBackupRegister(BKP_DR7);//�����󱸼Ĵ���
		DayTimeStartMin = BKP_ReadBackupRegister(BKP_DR8);//�����󱸼Ĵ���
		DayTimeEndHour = BKP_ReadBackupRegister(BKP_DR9);//�����󱸼Ĵ���
		DayTimeEndMin = BKP_ReadBackupRegister(BKP_DR10);//�����󱸼Ĵ��� 
	}
	//��������
	OLED_DISPLAY_Buff_16x16(0,2*16,&HuanYingShiYon_16[0],4); //��ʾ: ��ӭʹ��
	OLED_DISPLAY_Buff_16x16(2,1*16,&ZhiNengDengGuang_16[0],2);//��ʾ: �������ϵͳ
	OLED_DISPLAY_Buff_16x16(2,3*16,&YuGangXiTon_16[0],4); 
	OLED_DISPLAY_Buff_16x16(4,1*16,&KaiFaRen_16[0],3);//��ʾ: ������:Damon
	OLED_DISPLAY_8x16(4,8*8,':');// 
	OLED_DISPLAY_8x16(4,9*8,'D');// 
	OLED_DISPLAY_8x16(4,10*8,'a');// 
	OLED_DISPLAY_8x16(4,11*8,'m');// 
	OLED_DISPLAY_8x16(4,12*8,'o');// 
	OLED_DISPLAY_8x16(4,13*8,'n');// 
	delay_ms(1000);
	OLED_DISPLAY_CLEAR();//��������
	OLED_DISPLAY_16x16(0,1*16,&an4_16[0]); //��A���л�
	OLED_DISPLAY_8x16(0,4*8,'A');// 
	OLED_DISPLAY_16x16(0,3*16,&jian4_16[0]); //
	OLED_DISPLAY_Buff_16x16(0,4*16,&QieHuan_16[0],2); 
	OLED_DISPLAY_16x16(6,1*16,&an4_16[0]); //��D������
	OLED_DISPLAY_8x16(6,4*8,'D');// 
	OLED_DISPLAY_16x16(6,3*16,&jian4_16[0]); //
	OLED_DISPLAY_16x16(6,4*16,&she_16[0]); //
	OLED_DISPLAY_16x16(6,5*16,&zhi4_16[0]); //
 	delay_ms(2000);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...ˢ������
	while(1){ //��ѭ��
		if(MENU < 70 || MENU == 160){ //�����������Ӳ˵�ʱ ���ڻ�ˮ���������˵�
 			Temp = DS18B20_Get_Temp();//���ڶ����¶ȴ�����
			Temp2 = DS18B20_Get_Temp_2();//���ڶ����¶ȴ�����
			if(TemFreFlag == 1){  //�ж�ˢ���¶ȱ�־λ�Ƿ�Ϊ1				
				Temp = DS18B20_Get_Temp();//����һ���¶ȴ�����
			}else{ //ˢ���¶ȱ�־λΪ0������ʱ
				TemCoun++;	//ˢ���¶���ʱ����ֵ+1
				if(TemCoun >= 30){ //��ʱһ��ʱ��
					TemFreFlag = 1;	//ˢ���¶ȱ�־λ��1
					TemCoun = 0;
				}
			}
		}
		if(WriteTime == 1){	//���д��ʱ���־λΪ1
			WriteTime = 0;//д��ʱ���־λ��0
			RTC_Set(ryear,rmon,rday,rhour,rmin,rsec); //д�뵱ǰʱ�䣨1970~2099����Ч����		
		}
		if(MENU < 70 || MENU == 129){ //�����������Ӳ˵����������ô���׵ƹ������ٽ�ֵ�˵�ʱˢ�¹��ADCֵ			
			for(i=0; i<10; i++){
				ADC1buf[i] = ADC_ConvertedValue;	//ÿ��2ms��һ��ADC,��10��
				delay_ms(2);//��ʱ
			}
			MinADC = ADC1buf[0];
			for(i=0; i<10; i++){ //�ҳ�10�����е���Сֵ
				if(MinADC > ADC1buf[i]){			
					MinADC = ADC1buf[i];
				} 
			}
			MaxADC = ADC1buf[0];
			for(i=0; i<10; i++){ //�ҳ�10�����е����ֵ
				if(MaxADC < ADC1buf[i]){			
					MaxADC = ADC1buf[i];
				} 
			}
			for(i=0; i<10; i++){ //10�������
					EquaADC += ADC1buf[i];
			}
			EquaADC	= EquaADC - (MaxADC + MinADC);//��ȥ10�������е����ֵ����Сֵ,���ٸ���
			EquaADC = (EquaADC /8) + (EquaADC %8); //ʣ��8�����ݳ���8�ó�ƽ��ֵ
			EquaADC	= 4096 - EquaADC; //�ѹ��ADC��ֵ������,��֮ǰ�Ĺ���Խǿ��ֵԽС�����ڵĹ���Խǿ��ֵԽ��Ա�,���߸��������
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,ˢ������
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...������
		if(MENU == 1){	//��̬��ʾ����
			OLED_DISPLAY_CLEAR();//oled��������
			OLED_DISPLAY_16x16(0,0*16,&shui_16[0]); //��ʾ:ˮ��:   . �� 
			OLED_DISPLAY_16x16(0,1*16,&wen_16[0]);
			OLED_DISPLAY_8x16(0,4*8,':');
			OLED_DISPLAY_8x16(0,8*8,'.');//
			OLED_DISPLAY_16x16(0,10*8,&C_16[0]); //��ʾ�¶ȷ��š�//
			OLED_DISPLAY_16x16(2,0*16,&ri_16[0]);//��ʾ:����:  -  -
			OLED_DISPLAY_16x16(2,1*16,&qi_16[0]);
 			OLED_DISPLAY_8x16(2,4*8,':');//
			OLED_DISPLAY_8x16(2,9*8,'-');//				
			OLED_DISPLAY_8x16(2,12*8,'-');//				
			OLED_DISPLAY_16x16(4,0*16,&shij_16[0]);//��ʾ:ʱ��   :  : 
			OLED_DISPLAY_16x16(4,1*16,&jian_16[0]);
			OLED_DISPLAY_8x16(4,4*8,':');//
			OLED_DISPLAY_8x16(4,8*8,':');//				
			OLED_DISPLAY_8x16(4,11*8,':');//
			OLED_DISPLAY_16x16(6,3*16,&xing_16[0]);//��ʾ:   ����
			OLED_DISPLAY_16x16(6,4*16,&qi1_16[0]);
			MENU = 31; //�л�����̬ˢ�����ݲ˵�
		}
		if(MENU == 31){	//��̬ˢ����ʾ����
			RTC_Get(); //��ȡ��ǰʱ��
			OLED_DISPLAY_8x16(2,5*8,ryear/1000+0x30);//ˢ����ʾ��
			OLED_DISPLAY_8x16(2,6*8,ryear%1000/100+0x30);//
			OLED_DISPLAY_8x16(2,7*8,ryear%100/10+0x30);//
			OLED_DISPLAY_8x16(2,8*8,ryear%10+0x30);//
			OLED_DISPLAY_8x16(2,10*8,rmon/10+0x30);//ˢ����ʾ��
			OLED_DISPLAY_8x16(2,11*8,rmon%10+0x30);//
			OLED_DISPLAY_8x16(2,13*8,rday/10+0x30);//ˢ����ʾ��
			OLED_DISPLAY_8x16(2,14*8,rday%10+0x30);//
			OLED_DISPLAY_8x16(4,6*8,rhour/10+0x30);//ˢ����ʾСʱ
			OLED_DISPLAY_8x16(4,7*8,rhour%10+0x30);//
			OLED_DISPLAY_8x16(4,9*8,rmin/10+0x30);//ˢ����ʾ����
			OLED_DISPLAY_8x16(4,10*8,rmin%10+0x30);//
			OLED_DISPLAY_8x16(4,12*8,rsec/10+0x30);//ˢ����ʾ��
			OLED_DISPLAY_8x16(4,13*8,rsec%10+0x30);//
			OLED_DISPLAY_16x16(6,5*16,&xingqi1_7_16[rweek*32]);	//��ʾ����һ~~��

			//��ʾ�¶ȴ�����1
			if(Temp > 0){//�ж�Ϊ���¶�
				OLED_DISPLAY_8x16(0,5*8,' ');//��ʾ�¶�ֵ
				OLED_DISPLAY_8x16(0,6*8,Temp%1000/100+0x30);//��ʾ�¶�ֵ,�¶ȷ�Χ-55~~99
				OLED_DISPLAY_8x16(0,7*8,Temp%100/10+0x30);//
				OLED_DISPLAY_8x16(0,9*8,Temp%10+0x30);//
			}
//			else{
//				Temp *= -1;
//				OLED_DISPLAY_8x16(0,5*8,'-');//��ʾ�¶�ֵ
//				OLED_DISPLAY_8x16(0,6*8,Temp%1000/100+0x30);//��ʾ�¶�ֵ,�¶ȷ�Χ-55~~99
//				OLED_DISPLAY_8x16(0,7*8,Temp%100/10+0x30);//
//				OLED_DISPLAY_8x16(0,9*8,Temp%10+0x30);//
//			}

			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 2; //��ת��������
				}
				if(KEY == 4){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 61; //��ת�����ý���
					SetInstructFlag = 1; //��ʾ����˵����־λ��1
					OLED_DISPLAY_CLEAR();//oled��������
				}
			}
		}
		if(MENU == 2){
			OLED_DISPLAY_CLEAR();//oled��������
			OLED_DISPLAY_16x16(0,0*16,&guang_16[0]);//��ʾ:����ֵ:
			OLED_DISPLAY_16x16(0,1*16,&zhao_16[0]);
			OLED_DISPLAY_16x16(0,2*16,&zhi_16[0]);
			OLED_DISPLAY_8x16(0,6*8,':');//	
			OLED_DISPLAY_16x16(2,0*16,&xin1_16[0]);//��ʾ:��ˮˮ��:  . ��
			OLED_DISPLAY_16x16(2,1*16,&shui_16[0]);
 			OLED_DISPLAY_16x16(2,2*16,&shui_16[0]);
			OLED_DISPLAY_16x16(2,3*16,&wen_16[0]);
			OLED_DISPLAY_8x16(2,8*8,':');//	
			OLED_DISPLAY_8x16(2,12*8,'.');//
			OLED_DISPLAY_16x16(2,14*8,&C_16[0]); //��ʾ�¶ȷ��š�//
			MENU = 32;//�л�����̬ˢ�����ݲ˵�	
		}
		if(MENU == 32){	 //��̬ˢ�����ݲ˵�			
			OLED_DISPLAY_8x16(0,9*8,EquaADC/1000+0x30);//��ʾADC��ֵ,�����ǹ��ADC
			OLED_DISPLAY_8x16(0,10*8,EquaADC%1000/100+0x30);//
			OLED_DISPLAY_8x16(0,11*8,EquaADC%100/10+0x30);//
			OLED_DISPLAY_8x16(0,12*8,EquaADC%10+0x30);//
	
			Temp2 = DS18B20_Get_Temp_2();//���ڶ����¶ȴ�����
			if(Temp2 > 0){//�ж�Ϊ���¶�
				OLED_DISPLAY_8x16(2,9*8,' ');//��ʾ�¶�ֵ
				OLED_DISPLAY_8x16(2,10*8,Temp2%1000/100+0x30);//��ʾ�¶�ֵ,�¶ȷ�Χ-55~~99
				OLED_DISPLAY_8x16(2,11*8,Temp2%100/10+0x30);//
				OLED_DISPLAY_8x16(2,13*8,Temp2%10+0x30);//
			}
//			else{
//				Temp2 *= -1;
//				OLED_DISPLAY_8x16(2,9*8,'-');//��ʾ�¶�ֵ
//				OLED_DISPLAY_8x16(2,10*8,Temp2%1000/100+0x30);//��ʾ�¶�ֵ,�¶ȷ�Χ-55~~99
//				OLED_DISPLAY_8x16(2,11*8,Temp2%100/10+0x30);//
//				OLED_DISPLAY_8x16(2,13*8,Temp2%10+0x30);//
//			}

			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
				if(KEY == 4){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 61; //��ת�����ý���
					SetInstructFlag = 1; //��ʾ����˵����־λ��1
					OLED_DISPLAY_CLEAR();//oled��������
				}
			}
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...������
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...���ý���
		if(MENU == 61){
			if(SetInstructFlag == 1){
				SetInstructFlag = 0;
				OLED_DISPLAY_16x16(0,0*16,&an4_16[0]); //��A������ѡ��
				OLED_DISPLAY_8x16(0,2*8,'A');// 
				OLED_DISPLAY_16x16(0,2*16,&jian4_16[0]); //
				OLED_DISPLAY_16x16(0,3*16,&jin4_16[0]); //
				OLED_DISPLAY_16x16(0,4*16,&ru4_16[0]); //
				OLED_DISPLAY_Buff_16x16(0,5*16,&XuanXiang_16[0],2); 

				OLED_DISPLAY_16x16(2,0*16,&an4_16[0]); //��B��ѡ���1
				OLED_DISPLAY_8x16(2,2*8,'B');// 
				OLED_DISPLAY_16x16(2,2*16,&jian4_16[0]); //
				OLED_DISPLAY_Buff_16x16(2,3*16,&XuanXiang_16[0],2); 
				OLED_DISPLAY_16x16(2,5*16,&jian3_16[0]); //
				OLED_DISPLAY_8x16(2,12*8,'1');// 

				OLED_DISPLAY_16x16(4,0*16,&an4_16[0]); //��C��ѡ���1
				OLED_DISPLAY_8x16(4,2*8,'C');// 
				OLED_DISPLAY_16x16(4,2*16,&jian4_16[0]); //
				OLED_DISPLAY_Buff_16x16(4,3*16,&XuanXiang_16[0],2); 
				OLED_DISPLAY_16x16(4,5*16,&jia1_16[0]); //
				OLED_DISPLAY_8x16(4,12*8,'1');// 

				OLED_DISPLAY_16x16(6,0*16,&an4_16[0]); //��D���˳�����
				OLED_DISPLAY_8x16(6,2*8,'D');// 
				OLED_DISPLAY_16x16(6,2*16,&jian4_16[0]); //
				OLED_DISPLAY_16x16(6,3*16,&tui4_16[0]); //
				OLED_DISPLAY_16x16(6,4*16,&chu1_16[0]); //
				OLED_DISPLAY_16x16(6,5*16,&she_16[0]); //
				OLED_DISPLAY_16x16(6,6*16,&zhi4_16[0]); //
			 	delay_ms(1500);
				OLED_DISPLAY_CLEAR();//oled��������
			}

			OLED_DISPLAY_16x16(0,3*16,&she_16[0]);//��ʾ:   ����
			OLED_DISPLAY_16x16(0,4*16,&zhi4_16[0]);
			if(MENUSET < 4){ //ѡ��ֵС��4
				if(MENUSET == 1){ //ѡ��һ��ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ˮ�¼��				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&shui_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&wen_16[0]);
					ANTI_OLED_DISPLAY_16x16(2,3*16,&jian1_16[0]);
					ANTI_OLED_DISPLAY_16x16(2,4*16,&ce_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.ˮ�¼��
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&shui_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&wen_16[0]);				
					OLED_DISPLAY_16x16(2,3*16,&jian1_16[0]);
					OLED_DISPLAY_16x16(2,4*16,&ce_16[0]);
				}	
				if(MENUSET == 2){ //ѡ�ж���ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.ʱ������				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,3*16,&she_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&zhi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.ʱ������
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&jian_16[0]);				
					OLED_DISPLAY_16x16(4,3*16,&she_16[0]);
					OLED_DISPLAY_16x16(4,4*16,&zhi4_16[0]);
				}	
				if(MENUSET == 3){ //ѡ������ѡ��
					ANTI_OLED_DISPLAY_8x16(6,0*8,'3');//���ױ�����ʾ:3.���ܵƹ�				
					ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(6,1*16,&ZhiNengDengGuang_16[0],4);
				}else{
					OLED_DISPLAY_8x16(6,0*8,'3');//������ʾ:3.���ܵƹ�
					OLED_DISPLAY_8x16(6,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(6,1*16,&ZhiNengDengGuang_16[0],4);
				}	
			}
			if(MENUSET > 3 && MENUSET < 7){ //ѡ��ֵ����3 ���� ѡ��ֵС��7
				if(MENUSET == 4){ //ѡ��һ��ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'4');//���ױ�����ʾ:4.����ˮ��			GuoLvShuiBeng_16[]	
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(2,1*16,&GuoLvShuiBeng_16[0],4);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'4');//������ʾ:4.����ˮ��
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(2,1*16,&GuoLvShuiBeng_16[0],4);
				}	
				if(MENUSET == 5){ //ѡ�ж���ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'5');//���ױ�����ʾ:5.�Զ�Ͷι		ZiDongTouWei_16[]		
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'5');//������ʾ:5.�Զ�Ͷι
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);
				}	
				if(MENUSET == 6){ //ѡ������ѡ��
					ANTI_OLED_DISPLAY_8x16(6,0*8,'6');//���ױ�����ʾ:6.�Զ���ˮ	ZiDongHuanShui_16[]			
					ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(6,1*16,&ZiDongHuanShui_16[0],4);
				}else{
					OLED_DISPLAY_8x16(6,0*8,'6');//������ʾ:6.�Զ���ˮ
					OLED_DISPLAY_8x16(6,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(6,1*16,&ZiDongHuanShui_16[0],4);
				}	
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //���뵽ѡ������
					SUBMENU = 1; //��ѡ��ֵ��1
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(MENUSET <= 1) MENUSET = 7; //ѡ����Сֵ
					MENUSET--; //����ѡ���־ֵ��1
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					MENUSET++; //����ѡ���־ֵ��1
					if(MENUSET > 6) MENUSET = 1; //ѡ�����ֵ
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}
		}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....���ˮ��
		if(MENU == 71){	 //ˮ�¼������
			OLED_DISPLAY_16x16(0,1*16,&shui_16[0]); //��ʾ: ˮ�¼������ 
			OLED_DISPLAY_16x16(0,2*16,&wen_16[0]);				
			OLED_DISPLAY_16x16(0,3*16,&jian1_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&ce_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			if(WatTemMod == 1){ //��ģʽֵΪ1,������ˮ�¼��״̬,��ʾ����������ֵѡ��
				if(SUBMENU < 4){ //��ѡ��ֵС��4
					if(SUBMENU == 1){ //ѡ��һ����ѡ��
						ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
						ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
						ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
						ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
					}else{
						OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.ģʽ
						OLED_DISPLAY_8x16(2,1*8,'.');//
						OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
						OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
					}	
					if(SUBMENU == 2){ //ѡ�ж�����ѡ��
						ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.ˮ������				
						ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
						ANTI_OLED_DISPLAY_16x16(4,1*16,&shui_16[0]);  
						ANTI_OLED_DISPLAY_16x16(4,2*16,&wen_16[0]);				
						ANTI_OLED_DISPLAY_16x16(4,3*16,&shang4_16[0]);
						ANTI_OLED_DISPLAY_16x16(4,4*16,&xian4_16[0]);
					}else{
						OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.ˮ������
						OLED_DISPLAY_8x16(4,1*8,'.');//
						OLED_DISPLAY_16x16(4,1*16,&shui_16[0]);  
						OLED_DISPLAY_16x16(4,2*16,&wen_16[0]);				
						OLED_DISPLAY_16x16(4,3*16,&shang4_16[0]);
						OLED_DISPLAY_16x16(4,4*16,&xian4_16[0]);
					}	
					if(SUBMENU == 3){ //ѡ��������ѡ��
						ANTI_OLED_DISPLAY_8x16(6,0*8,'3');//���ױ�����ʾ:3.ˮ������				
						ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
						ANTI_OLED_DISPLAY_16x16(6,1*16,&shui_16[0]);  
						ANTI_OLED_DISPLAY_16x16(6,2*16,&wen_16[0]);				
						ANTI_OLED_DISPLAY_16x16(6,3*16,&xia4_16[0]);
						ANTI_OLED_DISPLAY_16x16(6,4*16,&xian4_16[0]);
					}else{
						OLED_DISPLAY_8x16(6,0*8,'3');//������ʾ:3.ˮ������
						OLED_DISPLAY_8x16(6,1*8,'.');//
						OLED_DISPLAY_16x16(6,1*16,&shui_16[0]);  
						OLED_DISPLAY_16x16(6,2*16,&wen_16[0]);				
						OLED_DISPLAY_16x16(6,3*16,&xia4_16[0]);
						OLED_DISPLAY_16x16(6,4*16,&xian4_16[0]);
					}	
				}
			}else{//ģʽֵΪ0,��ʾ�ر��¶ȼ��״̬,ֻ��ʾ1.ģʽ����
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);			
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(WatTemMod == 1){ //��ˮ�¼��ģʽֵΪ1,������ˮ�¼��״̬,���Խ�������
						MENU = 100 + SUBMENU; //���뵽��ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
					}else{ //��ˮ�¼��ģʽֵ��Ϊ1,���ر�ˮ�¼��״̬,ֻ�ܽ���ģʽ����
						MENU = 101; //ֻ�ܽ��뵽ģʽ��ѡ������
						OLED_DISPLAY_CLEAR();//oled��������					
					}
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(SUBMENU  <= 1) SUBMENU = 4; //��ѡ����Сֵ
					SUBMENU --; //��ѡ���־ֵ��1
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					SUBMENU++; //��ѡ���־ֵ��1
					if(SUBMENU > 3) SUBMENU = 1; //��ѡ�����ֵ
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}				
		}
		if(MENU == 101){//ˮ�¼��������ѡ�� //ģʽ����
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //��ʾ:  ģʽ���� 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(WatTemMod == 1){	//ˮ��ģʽ��־λΪ1,���������ˮ��
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//��ʾ:����
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //ˮ��ģʽ��־λ��Ϊ1,����رռ��ˮ��
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//��ʾ: �ر�
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //���뵽��ѡ������
					BKP_WriteBackupRegister(BKP_DR3, WatTemMod);//������ɺ�,��󱸼Ĵ�����д��ˮ�¼��ģʽֵ
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(WatTemMod  == 0) WatTemMod = 2; //��ѡ����Сֵ
					WatTemMod --; //��ѡ���־ֵ��1
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					WatTemMod++; //��ѡ���־ֵ��1
					if(WatTemMod > 1) WatTemMod = 0; //��ѡ�����ֵ
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					BKP_WriteBackupRegister(BKP_DR3, WatTemMod);//������ɺ�,��󱸼Ĵ�����д��ˮ�¼��ģʽֵ
					MENU = 1; //��ת��������
				}
			}								
		}
		if(MENU == 102){//ˮ�¼��������ѡ��  //�¶�����	  			
			OLED_DISPLAY_16x16(0,1*16,&shui_16[0]);  //��ʾ: �¶��������� 
			OLED_DISPLAY_16x16(0,2*16,&wen_16[0]);				
			OLED_DISPLAY_16x16(0,3*16,&shang4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&xian4_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			ANTI_OLED_DISPLAY_8x16(4,7*8,MaxWatTemp/10+0x30);//���ױ�����ʾ:ˮ���������ݵ�ֵ ��: 22��				
			ANTI_OLED_DISPLAY_8x16(4,8*8,MaxWatTemp%10+0x30);//
			ANTI_OLED_DISPLAY_16x16(4,9*8,&C_16[0]); //��ʾ�¶ȷ��š�//
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //�˻ص���ѡ������
					BKP_WriteBackupRegister(BKP_DR4, MaxWatTemp);//������ɺ�,��󱸼Ĵ�����д��ˮ������ֵ
					OLED_DISPLAY_CLEAR();//oled��������
				}

				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						if(MaxWatTemp  == 0) MaxWatTemp = 36; //ˮ��������Сֵ
						MaxWatTemp--; //ˮ������ֵ��1
						ANTI_OLED_DISPLAY_8x16(4,7*8,MaxWatTemp/10+0x30);//ˢ����ʾ���ױ�����ʾ:ˮ���������ݵ�ֵ ��: 22��				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MaxWatTemp%10+0x30);//
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					if(MaxWatTemp  == 0) MaxWatTemp = 36; //ˮ��������Сֵ
					MaxWatTemp--; //ˮ������ֵ��1								
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						MaxWatTemp++; //ˮ������ֵ��1
						if(MaxWatTemp > 35) MaxWatTemp = 0; //ˮ���������ֵ
						ANTI_OLED_DISPLAY_8x16(4,7*8,MaxWatTemp/10+0x30);//ˢ����ʾ���ױ�����ʾ:ˮ���������ݵ�ֵ ��: 22��				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MaxWatTemp%10+0x30);//
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					MaxWatTemp++; //ˮ������ֵ��1
					if(MaxWatTemp > 35) MaxWatTemp = 0; //ˮ���������ֵ
				} //�̰�����	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					BKP_WriteBackupRegister(BKP_DR4, MaxWatTemp);//������ɺ�,��󱸼Ĵ�����д��ˮ������ֵ
					MENU = 1; //��ת��������
				}
			}								
		}
		if(MENU == 103){//ˮ�¼��������ѡ��  //�¶�����	  			
			OLED_DISPLAY_16x16(0,1*16,&shui_16[0]);  //��ʾ: �¶��������� 
			OLED_DISPLAY_16x16(0,2*16,&wen_16[0]);				
			OLED_DISPLAY_16x16(0,3*16,&xia4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&xian4_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			ANTI_OLED_DISPLAY_8x16(4,7*8,MinWatTemp/10+0x30);//���ױ�����ʾ:ˮ���������ݵ�ֵ ��: 22��				
			ANTI_OLED_DISPLAY_8x16(4,8*8,MinWatTemp%10+0x30);//
			ANTI_OLED_DISPLAY_16x16(4,9*8,&C_16[0]); //��ʾ�¶ȷ��š�//
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //�˻ص���ѡ������
					OLED_DISPLAY_CLEAR();//oled��������
					BKP_WriteBackupRegister(BKP_DR5, MinWatTemp);//������ɺ�,��󱸼Ĵ�����д��ˮ������ֵ
				}
				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						if(MinWatTemp  == 0) MinWatTemp = 36; //ˮ��������Сֵ
						MinWatTemp--; //ˮ������ֵ��
						ANTI_OLED_DISPLAY_8x16(4,7*8,MinWatTemp/10+0x30);//ˢ����ʾ���ױ�����ʾ:ˮ���������ݵ�ֵ ��: 22��				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MinWatTemp%10+0x30);//
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					if(MinWatTemp  == 0) MinWatTemp = 36; //ˮ��������Сֵ
					MinWatTemp--; //ˮ������ֵ��1									
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						MinWatTemp++; //ˮ������ֵ��1
						if(MinWatTemp > 35) MinWatTemp = 0; //ˮ���������ֵ
						ANTI_OLED_DISPLAY_8x16(4,7*8,MinWatTemp/10+0x30);//ˢ����ʾ���ױ�����ʾ:ˮ���������ݵ�ֵ ��: 22��				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MinWatTemp%10+0x30);//
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					MinWatTemp++; //ˮ������ֵ��1
					if(MinWatTemp > 35) MinWatTemp = 0; //ˮ���������ֵ
				} //�̰�����
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					BKP_WriteBackupRegister(BKP_DR5, MinWatTemp);//������ɺ�,��󱸼Ĵ�����д��ˮ������ֵ
				}
			}								
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...���ˮ��
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...ʱ������
		if(MENU == 72){	 //ʱ������
			OLED_DISPLAY_16x16(0,2*16,&shij_16[0]); //��ʾ:  ʱ������ 
			OLED_DISPLAY_16x16(0,3*16,&jian_16[0]);				
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(SUBMENU < 4){ //��ѡ��ֵС��4
				if(SUBMENU == 1){ //ѡ��һ����ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.����			
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&ri_16[0]);//
					ANTI_OLED_DISPLAY_16x16(2,2*16,&qi_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.����
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&ri_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&qi_16[0]);
				}	
				if(SUBMENU == 2){ //ѡ�ж�����ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.ʱ��				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&fen1_16[0]);				
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.ʱ��
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&fen1_16[0]);				
				}	
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 110 + SUBMENU; //���뵽ʱ��������ѡ�����ò˵�
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(SUBMENU  <= 1) SUBMENU = 3; //��ѡ����Сֵ
					SUBMENU --; //��ѡ���־ֵ��1
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					SUBMENU++; //��ѡ���־ֵ��1
					if(SUBMENU > 2) SUBMENU = 1; //��ѡ�����ֵ
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}				
		}
		if(MENU == 111){//ʱ��������ѡ��  //��������	  			
			OLED_DISPLAY_16x16(0,2*16,&ri_16[0]); //��ʾ: �������� 
			OLED_DISPLAY_16x16(0,3*16,&qi_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);

			if(TimeShift == 1){	//���ױ�����ʾ��,��ʾ��ʾ�û���ǰѡ������
				ANTI_OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//��ʾ:������  ��:2021-08-04
				ANTI_OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
				ANTI_OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
				ANTI_OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
				OLED_DISPLAY_8x16(4,7*8,'-');//				
				OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//��
				OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
				OLED_DISPLAY_8x16(4,10*8,'-');//				
				OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//��
				OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
			}
			if(TimeShift == 2){	 //���ױ�����ʾ��,��ʾ��ʾ�û���ǰѡ������
				OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//��ʾ:������  ��:2021-08-04
				OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
				OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
				OLED_DISPLAY_8x16(4,7*8,'-');//				
				ANTI_OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//��
				ANTI_OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
				OLED_DISPLAY_8x16(4,10*8,'-');//				
				OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//��
				OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
			}
			if(TimeShift == 3){//���ױ�����ʾ��,��ʾ��ʾ�û���ǰѡ������
				OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//��ʾ:������  ��:2021-08-04
				OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
				OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
				OLED_DISPLAY_8x16(4,7*8,'-');//				
				OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//��
				OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
				OLED_DISPLAY_8x16(4,10*8,'-');//				
				ANTI_OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//��
				ANTI_OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(TimeShift < 3){
						TimeShift++;//ʱ������л�ѡ���־��1	
					}else{
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						TimeShift = 1; //ʱ������л�ѡ���־λ��1
						WriteTime = 1; //д��ʱ���־λ��1,д��ʱ�亯��ִ��д�����ú��ʱ��ֵ
					}
				}
				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (TimeShift){
							case 1: if(ryear <= 1970) ryear = 2100;	//����Сֵ
									ryear--;			 
									ANTI_OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//ˢ����ʾ:��
									ANTI_OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
									break;
							case 2: if(rmon <= 1) rmon = 13; //����Сֵ
									rmon--;
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//ˢ����ʾ��
									ANTI_OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
									break;
							case 3: if(rday <= 1) rday = 32; //��ֵ��Сֵ
									rday--;
									ANTI_OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//ˢ����ʾ��
									ANTI_OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
									break;
							default:TimeShift = 1;
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					switch (TimeShift){
						case 1: if(ryear <= 1970) ryear = 2100;	//����Сֵ
								ryear--;
								break;
						case 2: if(rmon <= 1) rmon = 13;  //����Сֵ
								rmon--;
								break;
						case 3: if(rday <= 1) rday = 32; //��ֵ��Сֵ
								rday--;
								break;
						default:TimeShift = 1;	//����
								break;											
					}
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (TimeShift){
							case 1: ryear++;
									if(ryear > 2099) ryear = 1970;	//�����ֵ										
									ANTI_OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//ˢ����ʾ:��
									ANTI_OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
									break;
							case 2: rmon++;
									if(rmon > 12) rmon = 1;		//�����ֵ										
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//ˢ����ʾ��
									ANTI_OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
									break;
							case 3: rday++;
									if(rday > 31) rday = 1;	 	//��ֵ���ֵ
									ANTI_OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//ˢ����ʾ��
									ANTI_OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
									break;
							default:TimeShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					switch (TimeShift){
						case 1: ryear++;
								if(ryear > 2099) ryear = 1970;//�����ֵ											
								break;
						case 2: rmon++;
								if(rmon > 12) rmon = 1;	//�����ֵ										
								break;
						case 3: rday++;
								if(rday > 31) rday = 1;	//��ֵ���ֵ
								break;
						default:TimeShift = 1;	//����
								break;											
					}
				} //�̰�����
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					WriteTime = 1; //д��ʱ���־λ��1,д��ʱ�亯��ִ��д�����ú��ʱ��ֵ
				}
			}								
		}
		if(MENU == 112){//ʱ��������ѡ��  //ʱ������	  			
			OLED_DISPLAY_16x16(0,2*16,&shij_16[0]); //��ʾ: ʱ������ 
			OLED_DISPLAY_16x16(0,3*16,&fen1_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(TimeShift == 1){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ����Сʱ
				ANTI_OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//Сʱ
				ANTI_OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,':');//				
				OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//����
				OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
			}
			if(TimeShift == 2){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���Ƿ���
				OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//Сʱ
				OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,':');//				
				ANTI_OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//����
				ANTI_OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
			}
			OLED_DISPLAY_8x16(4,9*8,':');//				
			OLED_DISPLAY_8x16(4,10*8,rsec/10+0x30);//��
			OLED_DISPLAY_8x16(4,11*8,rsec%10+0x30);//
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(TimeShift < 2){
						TimeShift++;//ʱ�����ѡ���־��1	
					}else{
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						TimeShift = 1; //ʱ�����ѡ���־λ��1
						WriteTime = 1; //д��ʱ���־λ��1,д��ʱ�亯��ִ��д�����ú��ʱ��ֵ
					}
				}
				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (TimeShift){
							case 1: if(rhour <= 0) rhour = 24; //Сʱ��Сֵ
									rhour--;
									ANTI_OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//Сʱ
									ANTI_OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
									break;
							case 2: if(rmin <= 0) rmin = 60;  //������Сֵ
									rmin--;
									rsec = 0;	  //����0
									ANTI_OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//����
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
									OLED_DISPLAY_8x16(4,10*8,rsec/10+0x30);//��
									OLED_DISPLAY_8x16(4,11*8,rsec%10+0x30);//
									break;
							default:TimeShift = 1;
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					switch (TimeShift){
						case 1: if(rhour <= 0) rhour = 24;//Сʱ��Сֵ
								rhour--;
								break;
						case 2: if(rmin <= 0) rmin = 60; //������Сֵ
								rmin--;
								rsec = 0;	//����0
								break;
						default:TimeShift = 1; //����
								break;											
					}
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (TimeShift){
							case 1: rhour++;
									if(rhour > 23) rhour = 0;//Сʱ���ֵ											
									ANTI_OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//Сʱ
									ANTI_OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
									break;
							case 2: rmin++;
									if(rmin > 59) rmin = 0;	//�������ֵ										
									rsec = 0;	 //����0
									ANTI_OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//ˢ����ʾ����
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
									OLED_DISPLAY_8x16(4,10*8,rsec/10+0x30);//��
									OLED_DISPLAY_8x16(4,11*8,rsec%10+0x30);//
									break;
							default:TimeShift = 1;
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					switch (TimeShift){
						case 1: rhour++;
								if(rhour > 23) rhour = 0;	//Сʱ���ֵ											
								break;
						case 2: rmin++;
								if(rmin > 59) rmin = 0;	//�������ֵ										
								rsec = 0;			   //����0
								break;
						default:TimeShift = 1;  //����
								break;											
					}
				} //�̰�����
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					WriteTime = 1; //д��ʱ���־λ��1,д��ʱ�亯��ִ��д�����ú��ʱ��ֵ
				}
			}								
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...ʱ������
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...���ܵƹ�����
		if(MENU == 73){	 //���ܵƹ�����			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZhiNengDengGuang_16[0],4);//��ʾ: ���ܵƹ�����
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			
			if(LightMod == 0){ //ģʽֵΪ0,��ʾ�ر����ܵƹ�״̬,ֻ��ʾ1.ģʽ
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(LightMod == 1){ //��ģʽֵΪ1,����������ʱ��ο������ܵƹ�״̬,��ʾ����ʱ���ѡ��
				if(SUBMENU == 1){ //ѡ��һ����ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.ģʽ
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //ѡ�ж�����ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.��������͵�λ				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&bai1_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&tian1_16[0]);				
					ANTI_OLED_DISPLAY_16x16(4,3*16,&qu1_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,5*16,&he2_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&dang3_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,7*16,&wei4_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.��������͵�λ
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&bai1_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&tian1_16[0]);				
					OLED_DISPLAY_16x16(4,3*16,&qu1_16[0]);
					OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);
					OLED_DISPLAY_16x16(4,5*16,&he2_16[0]);
					OLED_DISPLAY_16x16(4,6*16,&dang3_16[0]);
					OLED_DISPLAY_16x16(4,7*16,&wei4_16[0]);

				}	
				if(SUBMENU == 3){ //ѡ��������ѡ��
					ANTI_OLED_DISPLAY_8x16(6,0*8,'3');//���ױ�����ʾ:3.��������͵�λ				
					ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(6,1*16,&bang4_16[0]);  
					ANTI_OLED_DISPLAY_16x16(6,2*16,&wan3_16[0]);				
					ANTI_OLED_DISPLAY_16x16(6,3*16,&qu1_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,4*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,5*16,&he2_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,6*16,&dang3_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,7*16,&wei4_16[0]);
				}else{
					OLED_DISPLAY_8x16(6,0*8,'3');//������ʾ:3.��������͵�λ
					OLED_DISPLAY_8x16(6,1*8,'.');//
					OLED_DISPLAY_16x16(6,1*16,&bang4_16[0]);  
					OLED_DISPLAY_16x16(6,2*16,&wan3_16[0]);				
					OLED_DISPLAY_16x16(6,3*16,&qu1_16[0]);
					OLED_DISPLAY_16x16(6,4*16,&jian_16[0]);
					OLED_DISPLAY_16x16(6,5*16,&he2_16[0]);
					OLED_DISPLAY_16x16(6,6*16,&dang3_16[0]);
					OLED_DISPLAY_16x16(6,7*16,&wei4_16[0]);
				}	
			}
			if(LightMod == 2){ //��ģʽֵΪ2,���������չ������ȿ������ܵƹ�״̬,��ʾ������������ֵѡ��
				if(SUBMENU == 1){ //ѡ��һ����ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.ģʽ
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //ѡ�ж�����ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.��������ֵ				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&kai1_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&deng1_16[0]);				
					ANTI_OLED_DISPLAY_16x16(4,3*16,&liang4_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&du4_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,5*16,&zhi_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.��������ֵ
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&kai1_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&deng1_16[0]);				
					OLED_DISPLAY_16x16(4,3*16,&liang4_16[0]);
					OLED_DISPLAY_16x16(4,4*16,&du4_16[0]);
					OLED_DISPLAY_16x16(4,5*16,&zhi_16[0]);
				}	
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(LightMod == 0){ //��ˮ�¼��ģʽֵΪ0,���ر�ˮ�¼��״̬,ֻ�ܽ���ģʽ����
						MENU = 121; //ֻ�ܽ��뵽ģʽ��ѡ������
					}
					if(LightMod == 1){ //��ˮ�¼��ģʽֵΪ1,������ˮ�¼��״̬,���Խ�������
						MENU = 120 + SUBMENU; //���뵽��ѡ������
					}
					if(LightMod == 2){ //��ˮ�¼��ģʽֵΪ2,�����չ������ȴ򿪵ƹ�״̬,ֻ�ܽ��������������
						if(SUBMENU == 1) MENU = 121; //��ģʽ���ò˵�
						if(SUBMENU == 2) MENU = 129; //129�˵�,ר�����ڴ򿪵ƹ�Ĺ�����������
					}
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(LightMod == 1){
						if(SUBMENU  == 1) SUBMENU = 4; //��ѡ����Сֵ
						SUBMENU --; //��ѡ���־ֵ��1					
					}
					if(LightMod == 2){
						if(SUBMENU  == 1) SUBMENU = 3; //��ѡ����Сֵ
						SUBMENU --; //��ѡ���־ֵ��1					
					}
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					if(LightMod == 1){
						SUBMENU++; //��ѡ���־ֵ��1
						if(SUBMENU > 3) SUBMENU = 1; //��ѡ�����ֵ
					}
					if(LightMod == 2){
						SUBMENU++; //��ѡ���־ֵ��1
						if(SUBMENU > 2) SUBMENU = 1; //��ѡ�����ֵ
					}
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}								
		}
		if(MENU == 121){//���ܵƹ�������ѡ�� //ģʽ����
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //��ʾ:  ģʽ���� 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(LightMod == 0){ //���ܵƹ�ģʽ��־λΪ0,����ر����ܵƹ�
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//��ʾ: �ر�
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(LightMod == 1){	//���ܵƹ�ģʽ��־λΪ1,������ʱ��ο������ܵƹ�	
				ANTI_OLED_DISPLAY_16x16(4,1*16,&an4_16[0]); //��ʾ:����ʱ���
				ANTI_OLED_DISPLAY_16x16(4,2*16,&zhao4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,3*16,&shij_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,5*16,&duan4_16[0]);
			}
			if(LightMod == 2){ //���ܵƹ�ģʽ��־λΪ2,�����չ������ȿ������ܵƹ�
				ANTI_OLED_DISPLAY_16x16(4,1*16,&an4_16[0]); //��ʾ:���չ�������
				ANTI_OLED_DISPLAY_16x16(4,2*16,&zhao4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guang_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,4*16,&Gxian4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,5*16,&liang4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,6*16,&du4_16[0]);
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0������־λ
					MENU = 70 + MENUSET; //���뵽��ѡ������
					BKP_WriteBackupRegister(BKP_DR6, LightMod);//������ɺ�,���6���󱸼Ĵ�����д�����ܵƹ�ģʽֵ
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(LightMod  == 0) LightMod = 3; //��ѡ����Сֵ
					LightMod --; //��ѡ���־ֵ��1
					OLED_DISPLAY_CLEAR_OneLine(4);//ֻ����һ�� ����0,2,4,6
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					LightMod++; //��ѡ���־ֵ��1
					if(LightMod > 2) LightMod = 0; //��ѡ�����ֵ
					OLED_DISPLAY_CLEAR_OneLine(4);//ֻ����һ�� ����0,2,4,6
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					BKP_WriteBackupRegister(BKP_DR6, LightMod);//������ɺ�,���6���󱸼Ĵ�����д�����ܵƹ�ģʽֵ
					MENU = 1; //��ת��������
				}
			}								
		}
		if(MENU == 122){//���ܵƹ�������ѡ��  //��������	  			
			OLED_DISPLAY_16x16(0,2*16,&bai1_16[0]);  //��ʾ:  ��������
			OLED_DISPLAY_16x16(0,3*16,&tian1_16[0]);				
			OLED_DISPLAY_16x16(0,4*16,&qu1_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&jian_16[0]);
			OLED_DISPLAY_16x16(6,0*16,&deng1_16[0]);//��ʾ: �ƹ⵵λ:				
			OLED_DISPLAY_16x16(6,1*16,&guang_16[0]);
			OLED_DISPLAY_16x16(6,2*16,&dang3_16[0]);
			OLED_DISPLAY_16x16(6,3*16,&wei4_16[0]);
			OLED_DISPLAY_8x16(6,8*8,':');//				
			if(LightShift < 3){ //����ˢ����ʾ��������Ľ���Сʱֵ
				if(LightShift == 1){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ���ǰ������俪ʼ��Сʱֵ
					ANTI_OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//Сʱ
					ANTI_OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//����
					OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
				}
				if(LightShift == 2){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���ǰ������俪ʼ�ķ���ֵ
					OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//����
					ANTI_OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//Сʱ
				OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
				OLED_DISPLAY_8x16(4,11*8,':');//				
				OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//����
				OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//��λֵ 0~100
				OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
				OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
			}
			if(LightShift > 2){ //����ˢ����ʾ��������Ľ���Сʱֵ
				if(LightShift == 3){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ���ǰ������������Сʱֵ
					ANTI_OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//Сʱ
					ANTI_OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//����
					OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				}
				if(LightShift == 4){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���ǰ�����������ķ���ֵ
					OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//����
					ANTI_OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				}
				if(LightShift == 5){	 //���ױ�����ʾ��λ,��ʾ��ʾ�û���ǰѡ���ǰ�������ƹ⵵λ
					ANTI_OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//��λֵ 0~100
					ANTI_OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
					ANTI_OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//����
					OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//Сʱ
				OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
				OLED_DISPLAY_8x16(4,5*8,':');//				
				OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//����
				OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
			}
			OLED_DISPLAY_8x16(4,8*8,'-');// �������俪ʼ�ͽ���ʱ���м�ı�ʶ�� ~				
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(LightShift < 5){
						LightShift++;//ʱ������л�ѡ���־��1	
					}else{
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						LightShift = 1; //ʱ�����ѡ���־λ��1
						BKP_WriteBackupRegister(BKP_DR7, DayTimeStartHour);//��󱸼Ĵ���д�����ú�İ������俪ʼСʱֵ	
						BKP_WriteBackupRegister(BKP_DR8, DayTimeStartMin);//��󱸼Ĵ���д�����ú�İ������俪ʼ����ֵ	
						BKP_WriteBackupRegister(BKP_DR9, DayTimeEndHour);//��󱸼Ĵ���д�����ú�İ����������Сʱֵ	
						BKP_WriteBackupRegister(BKP_DR10, DayTimeEndMin);//��󱸼Ĵ���д�����ú�İ��������������ֵ	
						WriteFlashBuff[1] = DayTimeGear; //��������ƹ⵵λֵ��ʼֵ�Ž�д�뻺������,�ȴ�д��
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					}
				}

				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (LightShift){
							case 1: if(DayTimeStartHour <= 0) DayTimeStartHour = 24; //�������俪ʼ��Сʱ��Сֵ
									DayTimeStartHour--;
									ANTI_OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
									break;
							case 2: if(DayTimeStartMin <= 0) DayTimeStartMin = 60;  //�������俪ʼ�ķ�����Сֵ
									DayTimeStartMin--;
									ANTI_OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
									break;
							case 3: if(DayTimeEndHour <= 0) DayTimeEndHour = 24; //�������������Сʱ��Сֵ
									DayTimeEndHour--;
									ANTI_OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
									break;
							case 4: if(DayTimeEndMin <= 0) DayTimeEndMin = 60;  //������������ķ�����Сֵ
									DayTimeEndMin--;
									ANTI_OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
									break;
							case 5:	if(DayTimeGear <= 0) DayTimeGear = 101;  //��������ƹ⵵λ��Сֵ
									DayTimeGear--;	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//���ټӼ�ʱˢ����ʾ��λֵ 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					switch (LightShift){
						case 1: if(DayTimeStartHour <= 0) DayTimeStartHour = 24; //�������俪ʼ��Сʱ��Сֵ
								DayTimeStartHour--;
								break;
						case 2: if(DayTimeStartMin <= 0) DayTimeStartMin = 60;  //�������俪ʼ�ķ�����Сֵ
								DayTimeStartMin--;
								break;
						case 3: if(DayTimeEndHour <= 0) DayTimeEndHour = 24; //�������������Сʱ��Сֵ
								DayTimeEndHour--;
								break;
						case 4: if(DayTimeEndMin <= 0) DayTimeEndMin = 60;  //������������ķ�����Сֵ
								DayTimeEndMin--;
								break;
						case 5:	if(DayTimeGear <= 0) DayTimeGear = 101;  //��������ƹ⵵λ��Сֵ
								DayTimeGear--;	 
								break;
						default:LightShift = 1; //����
								break;											
					}
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (LightShift){
							case 1: DayTimeStartHour++;
									if(DayTimeStartHour >23) DayTimeStartHour = 0; //�������俪ʼ��Сʱ���ֵ											
									ANTI_OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
									break;
							case 2: DayTimeStartMin++;
									if(DayTimeStartMin > 59) DayTimeStartMin = 0;  //�������俪ʼ�ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
									break;
							case 3: DayTimeEndHour++;
									if(DayTimeEndHour > 23) DayTimeEndHour = 0; //�������������Сʱ���ֵ											
									ANTI_OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
									break;
							case 4: DayTimeEndMin++;
									if(DayTimeEndMin > 59) DayTimeEndMin = 0;  //������������ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
									break;
							case 5:	DayTimeGear++;
									if(DayTimeGear > 100) DayTimeGear = 0;  //��������ƹ⵵λ���ֵ	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//���ټӼ�ʱˢ����ʾ��λֵ 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					switch (LightShift){
						case 1: DayTimeStartHour++;
								if(DayTimeStartHour >23) DayTimeStartHour = 0; //�������俪ʼ��Сʱ���ֵ											
								break;
						case 2: DayTimeStartMin++;
								if(DayTimeStartMin > 59) DayTimeStartMin = 0;  //�������俪ʼ�ķ������ֵ											
								break;
						case 3: DayTimeEndHour++;
								if(DayTimeEndHour > 23) DayTimeEndHour = 0; //�������������Сʱ���ֵ											
								break;
						case 4: DayTimeEndMin++;
								if(DayTimeEndMin > 59) DayTimeEndMin = 0;  //������������ķ������ֵ																					
						case 5:	DayTimeGear++;
								if(DayTimeGear > 100) DayTimeGear = 0;  //��������ƹ⵵λ���ֵ
								break;	 
						default:LightShift = 1; //����
								break;																
					}
				} //�̰�����	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					BKP_WriteBackupRegister(BKP_DR7, DayTimeStartHour);//��󱸼Ĵ���д�����ú�İ������俪ʼСʱֵ	
					BKP_WriteBackupRegister(BKP_DR8, DayTimeStartMin);//��󱸼Ĵ���д�����ú�İ������俪ʼ����ֵ	
					BKP_WriteBackupRegister(BKP_DR9, DayTimeEndHour);//��󱸼Ĵ���д�����ú�İ����������Сʱֵ	
					BKP_WriteBackupRegister(BKP_DR10, DayTimeEndMin);//��󱸼Ĵ���д�����ú�İ��������������ֵ	
					WriteFlashBuff[1] = DayTimeGear; //��������ƹ⵵λֵ��ʼֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
				}
			}
		}
		if(MENU == 123){//���ܵƹ�������ѡ��---��������	  			
			OLED_DISPLAY_16x16(0,2*16,&bang4_16[0]);  //��ʾ:  �������� 
			OLED_DISPLAY_16x16(0,3*16,&wan3_16[0]);				
			OLED_DISPLAY_16x16(0,4*16,&qu1_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&jian_16[0]);
			OLED_DISPLAY_16x16(6,0*16,&deng1_16[0]);//��ʾ: �ƹ⵵λ:				
			OLED_DISPLAY_16x16(6,1*16,&guang_16[0]);
			OLED_DISPLAY_16x16(6,2*16,&dang3_16[0]);
			OLED_DISPLAY_16x16(6,3*16,&wei4_16[0]);
			OLED_DISPLAY_8x16(6,8*8,':');//				
			OLED_DISPLAY_8x16(4,8*8,'-');// �������俪ʼ�ͽ���ʱ���м�ı�ʶ�� ~				
			if(LightShift < 3){ //����ˢ����ʾ��������Ľ���Сʱ�ͷ���ֵ
				if(LightShift == 1){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ���ǰ������������Сʱֵ
					ANTI_OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//Сʱ
					ANTI_OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//����
					OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
				}
				if(LightShift == 2){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���ǰ�����������ķ���ֵ
					OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//����
					ANTI_OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,3*8,DayTimeEndHour/10+0x30);//��ʾ�������������Сʱ�ͷ���,ͬʱ��Ϊ�������俪ʼ��Сʱ�ͷ���
				OLED_DISPLAY_8x16(4,4*8,DayTimeEndHour%10+0x30);//
				OLED_DISPLAY_8x16(4,5*8,':');//				
				OLED_DISPLAY_8x16(4,6*8,DayTimeEndMin/10+0x30);//����
				OLED_DISPLAY_8x16(4,7*8,DayTimeEndMin%10+0x30);//

				OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//��λֵ 0~100
				OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
				OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
			}
			if(LightShift > 2){ //����ˢ����ʾ��������ĵ�λֵ
				if(LightShift == 3){	 //���ױ�����ʾ��λ,��ʾ��ʾ�û���ǰѡ���ǰ�������ƹ⵵λ
					ANTI_OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//��λֵ 0~100
					ANTI_OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
					ANTI_OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//����
					OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
				}
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(LightShift < 3){
						LightShift++;//ʱ������л�ѡ���־��1	
					}else{
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						LightShift = 1; //ʱ�����ѡ���־λ��1
						WriteFlashBuff[2] = DuskTimeEndHour; //���ú�İ����������Сʱֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[3] = DuskTimeEndMin; //���ú�İ��������������ֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[4] = DuskTimeGear; //���ú�İ�������ƹ⵵λֵ�Ž�д�뻺������,�ȴ�д��
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					}
				}
				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (LightShift){
							case 1: if(DuskTimeEndHour <= 0) DuskTimeEndHour = 24; //�������������Сʱ��Сֵ
									DuskTimeEndHour--;
									ANTI_OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
									break;
							case 2: if(DuskTimeEndMin <= 0) DuskTimeEndMin = 60;  //������������ķ�����Сֵ
									DuskTimeEndMin--;
									ANTI_OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
									break;
							case 3:	if(DuskTimeGear <= 0) DuskTimeGear = 101;  //��������ƹ⵵λ��Сֵ
									DuskTimeGear--;	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//���ټӼ�ʱˢ����ʾ��λֵ 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					switch (LightShift){
						case 1: if(DuskTimeEndHour <= 0) DuskTimeEndHour = 24; //�������������Сʱ��Сֵ
								DuskTimeEndHour--;
								break;
						case 2: if(DuskTimeEndMin <= 0) DuskTimeEndMin = 60;  //������������ķ�����Сֵ
								DuskTimeEndMin--;
								break;
						case 3:	if(DuskTimeGear <= 0) DuskTimeGear = 101;  //��������ƹ⵵λ��Сֵ
								DuskTimeGear--;	 
								break;
						default:LightShift = 1; //����
								break;											
					}
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (LightShift){
							case 1: DuskTimeEndHour++;
									if(DuskTimeEndHour > 23) DuskTimeEndHour = 0; //�������������Сʱ���ֵ											
									ANTI_OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
									break;
							case 2: DuskTimeEndMin++;
									if(DuskTimeEndMin > 59) DuskTimeEndMin = 0;  //������������ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
									break;
							case 3:	DuskTimeGear++;
									if(DuskTimeGear > 100) DuskTimeGear = 0;  //��������ƹ⵵λ���ֵ	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//���ټӼ�ʱˢ����ʾ��λֵ 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					switch (LightShift){
						case 1: DuskTimeEndHour++;
								if(DuskTimeEndHour > 23) DuskTimeEndHour = 0; //�������������Сʱ���ֵ											
								break;
						case 2: DuskTimeEndMin++;
								if(DuskTimeEndMin > 59) DuskTimeEndMin = 0;  //������������ķ������ֵ	
								break;																				
						case 3:	DuskTimeGear++;
								if(DuskTimeGear > 100) DuskTimeGear = 0;  //��������ƹ⵵λ���ֵ
								break;	 
						default:LightShift = 1; //����
								break;																
					}
				} //�̰�����	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					WriteFlashBuff[2] = DuskTimeEndHour; //���ú�İ����������Сʱֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[3] = DuskTimeEndMin; //���ú�İ��������������ֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[4] = DuskTimeGear; //���ú�İ�������ƹ⵵λֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
				}
			}
		}
		if(MENU == 129){//���ܵƹ�������ѡ��  //��������ֵ	  			
			OLED_DISPLAY_16x16(0,0*16,&kai1_16[0]);  //��ʾ:���ƹ�����������
			OLED_DISPLAY_16x16(0,1*16,&deng1_16[0]);		
			OLED_DISPLAY_16x16(0,2*16,&guang_16[0]);					
			OLED_DISPLAY_16x16(0,3*16,&Gxian4_16[0]);									
			OLED_DISPLAY_16x16(0,4*16,&liang4_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&du4_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,7*16,&zhi4_16[0]);
			OLED_DISPLAY_8x16(4,4*8,'L');//��ʾ: Lux:
			OLED_DISPLAY_8x16(4,5*8,'u');//
			OLED_DISPLAY_8x16(4,6*8,'x');//
			OLED_DISPLAY_8x16(4,7*8,':');//
			ANTI_OLED_DISPLAY_8x16(4,8*8,LuxSwitch/1000+0x30);//��ʾ�򿪵Ƶ������ٽ�ֵ
			ANTI_OLED_DISPLAY_8x16(4,9*8,LuxSwitch%1000/100+0x30);//
			ANTI_OLED_DISPLAY_8x16(4,10*8,LuxSwitch%100/10+0x30);//
			ANTI_OLED_DISPLAY_8x16(4,11*8,LuxSwitch%10+0x30);//
			OLED_DISPLAY_16x16(6,0*16,&dang1_16[0]);//��ʾ: ��ǰ������:					
 			OLED_DISPLAY_16x16(6,1*16,&qian3_16[0]);								
			OLED_DISPLAY_16x16(6,2*16,&guang_16[0]);					
			OLED_DISPLAY_16x16(6,3*16,&liang4_16[0]);
			OLED_DISPLAY_16x16(6,4*16,&du4_16[0]);
			OLED_DISPLAY_8x16(6,10*8,':');//
			OLED_DISPLAY_8x16(6,11*8,EquaADC/1000+0x30);//��ʾADC��ֵ,�����ǹ��ADC
			OLED_DISPLAY_8x16(6,12*8,EquaADC%1000/100+0x30);//
			OLED_DISPLAY_8x16(6,13*8,EquaADC%100/10+0x30);//
			OLED_DISPLAY_8x16(6,14*8,EquaADC%10+0x30);//
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //�˻ص���ѡ������
					WriteFlashBuff[5] = LuxSwitch; //���ú�������ٽ�ֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					OLED_DISPLAY_CLEAR();//oled��������
				}

				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						LuxSwitch -= 100; //�����ٽ�ֵֵ��1
						if(LuxSwitch > 4096) LuxSwitch = 4196; //�����ٽ�ֵ��Сֵ						
						ANTI_OLED_DISPLAY_8x16(4,8*8,LuxSwitch/1000+0x30);//ˢ����ʾ���ױ�����ʾ:�����ٽ�ֵ
						ANTI_OLED_DISPLAY_8x16(4,9*8,LuxSwitch%1000/100+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,10*8,LuxSwitch%100/10+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,11*8,LuxSwitch%10+0x30);//
						delay_ms(100); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					if(LuxSwitch  == 0) LuxSwitch = 4097; //ˮ��������Сֵ
					LuxSwitch--; //ˮ������ֵ��1								
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						LuxSwitch += 100; //�����ٽ�ֵֵ��100
						if(LuxSwitch > 4096) LuxSwitch = 0; //�����ٽ�ֵ��Сֵ						
						ANTI_OLED_DISPLAY_8x16(4,8*8,LuxSwitch/1000+0x30);//ˢ����ʾ���ױ�����ʾ:�����ٽ�ֵ
						ANTI_OLED_DISPLAY_8x16(4,9*8,LuxSwitch%1000/100+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,10*8,LuxSwitch%100/10+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,11*8,LuxSwitch%10+0x30);//
						delay_ms(100); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					LuxSwitch++; //ˮ������ֵ��1
					if(LuxSwitch > 4096) LuxSwitch = 0; //ˮ���������ֵ
				} //�̰�����	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					WriteFlashBuff[5] = LuxSwitch; //���ú�������ٽ�ֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					MENU = 1; //��ת��������
				}
			}								
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,���ܵƹ�����
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...����ˮ������
		if(MENU == 74){	 //����ˮ������			
			OLED_DISPLAY_Buff_16x16(0,1*16,&GuoLvShuiBeng_16[0],4);//��ʾ: ����ˮ������
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			
			if(FiltPumpMod == 0){ //ģʽֵΪ0,��ʾ�رչ���ˮ��ģʽ,ֻ��ʾ 1.ģʽ
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(FiltPumpMod == 1){ //��ģʽֵΪ1,����������ˮ��ģʽ,��ʾ����ʱ������ѡ��
				if(SUBMENU == 1){ //ѡ��һ����ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.ģʽ
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //ѡ�ж�����ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.������ʱ������				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&kai1_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,2*16,&qi3_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,3*16,&de_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&shij_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,5*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&qu1_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,7*16,&jian_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.������ʱ������
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&kai1_16[0]);
					OLED_DISPLAY_16x16(4,2*16,&qi3_16[0]);
					OLED_DISPLAY_16x16(4,3*16,&de_16[0]);
					OLED_DISPLAY_16x16(4,4*16,&shij_16[0]);
					OLED_DISPLAY_16x16(4,5*16,&jian_16[0]);
					OLED_DISPLAY_16x16(4,6*16,&qu1_16[0]);
					OLED_DISPLAY_16x16(4,7*16,&jian_16[0]);
				}	
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(FiltPumpMod == 0){ //������ˮ��ģʽֵΪ0,���رչ���ˮ��״̬,ֻ�ܽ���ģʽ����
						MENU = 131; //ֻ�ܽ��뵽ģʽ��ѡ������
					}
					if(FiltPumpMod == 1){ //������ˮ��ģʽֵΪ1,����������ˮ��״̬,���Խ�����ѡ������
						MENU = 130 + SUBMENU; //���뵽��ѡ������
					}
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(FiltPumpMod == 1){
						if(SUBMENU  == 1) SUBMENU = 3; //��ѡ����Сֵ
						SUBMENU --; //��ѡ���־ֵ��1					
					}
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					if(FiltPumpMod == 1){
						SUBMENU++; //��ѡ���־ֵ��1
						if(SUBMENU > 2) SUBMENU = 1; //��ѡ�����ֵ
					}
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}								
		}
		if(MENU == 131){//����ˮ��������ѡ�� //ģʽ����
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //��ʾ:  ģʽ���� 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(FiltPumpMod == 1){	//����ˮ��ģʽ��־λΪ1,����������ˮ��
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//��ʾ:����
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //����ˮ��ģʽ��־λ��Ϊ1,����رչ���ˮ��
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//��ʾ: �ر�
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //���뵽��ѡ������
					WriteFlashBuff[6] = FiltPumpMod; //���ú�Ĺ���ˮ��ģʽֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(FiltPumpMod  == 0) FiltPumpMod = 2; //��ѡ����Сֵ
					FiltPumpMod --; //��ѡ���־ֵ��1
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					FiltPumpMod++; //��ѡ���־ֵ��1
					if(FiltPumpMod > 1) FiltPumpMod = 0; //��ѡ�����ֵ
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					WriteFlashBuff[6] = FiltPumpMod; //���ú�Ĺ���ˮ��ģʽֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					MENU = 1; //��ת��������
				}
			}								
		}
		if(MENU == 132){//����ˮ��������ѡ��  //����ʱ����������	  			
			OLED_DISPLAY_16x16(0,0*16,&kai1_16[0]);//��ʾ:2.����ʱ����������
			OLED_DISPLAY_16x16(0,1*16,&qi3_16[0]);
			OLED_DISPLAY_16x16(0,2*16,&shij_16[0]);
			OLED_DISPLAY_16x16(0,3*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&qu1_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,7*16,&zhi4_16[0]);

			if(MENUShift < 3){ //����ˢ����ʾ��������Ľ���Сʱֵ
				if(MENUShift == 1){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ�����俪ʼ��Сʱֵ
					ANTI_OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//Сʱ
					ANTI_OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//����
					OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
				}
				if(MENUShift == 2){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ�����俪ʼ�ķ���ֵ
					OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//����
					ANTI_OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//������Сʱ
				OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
				OLED_DISPLAY_8x16(4,11*8,':');//				
				OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//�����ķ���
				OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
			}
			if(MENUShift > 2){ //����ˢ����ʾ��������Ľ���Сʱֵ
				if(MENUShift == 3){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ�����������Сʱֵ
					ANTI_OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//Сʱ
					ANTI_OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//����
					OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
				}
				if(MENUShift == 4){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ����������ķ���ֵ
					OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//����
					ANTI_OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//��ʼ��Сʱ
				OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
				OLED_DISPLAY_8x16(4,5*8,':');//				
				OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//��ʼ�ķ���
				OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
			}
			OLED_DISPLAY_8x16(4,8*8,'-');// ����ˮ�����俪ʼ�ͽ���ʱ���м�ı�ʶ�� ~				
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(MENUShift < 4){
						MENUShift++;//ʱ������л�ѡ���־��1	
					}else{
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						MENUShift = 1; //ʱ�����ѡ���־λ��1
						WriteFlashBuff[7] = FiltPumpStartHour; //���ú�Ĺ���ˮ�ÿ��������俪ʼСʱֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[8] = FiltPumpStartMin; //���ú�Ĺ���ˮ�ÿ��������俪ʼ����ֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[9] = FiltPumpEndHour; //���ú�Ĺ���ˮ�ÿ������������Сʱֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[10] = FiltPumpEndMin; //���ú�Ĺ���ˮ�ÿ����������������ֵ�Ž�д�뻺������,�ȴ�д��
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					}
				}

				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (MENUShift){
							case 1: if(FiltPumpStartHour <= 0) FiltPumpStartHour = 24; //����ˮ�ÿ������俪ʼ��Сʱ��Сֵ
									FiltPumpStartHour--;
									ANTI_OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
									break;
							case 2: if(FiltPumpStartMin <= 0) FiltPumpStartMin = 60;  //����ˮ�ÿ������俪ʼ�ķ�����Сֵ
									FiltPumpStartMin--;
									ANTI_OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
									break;
							case 3: if(FiltPumpEndHour <= 0) FiltPumpEndHour = 24; //����ˮ�ÿ������������Сʱ��Сֵ
									FiltPumpEndHour--;
									ANTI_OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
									break;
							case 4: if(FiltPumpEndMin <= 0) FiltPumpEndMin = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
									FiltPumpEndMin--;
									ANTI_OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
									break;
							default:MENUShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					switch (MENUShift){
						case 1: if(FiltPumpStartHour <= 0) FiltPumpStartHour = 24; //����ˮ�ÿ������俪ʼ��Сʱ��Сֵ
								FiltPumpStartHour--;
								break;
						case 2: if(FiltPumpStartMin <= 0) FiltPumpStartMin = 60;  //����ˮ�ÿ������俪ʼ�ķ�����Сֵ
								FiltPumpStartMin--;
								break;
						case 3: if(FiltPumpEndHour <= 0) FiltPumpEndHour = 24; //����ˮ�ÿ������������Сʱ��Сֵ
								FiltPumpEndHour--;
								break;
						case 4: if(FiltPumpEndMin <= 0) FiltPumpEndMin = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
								FiltPumpEndMin--;
								break;
						default:MENUShift = 1; //����
								break;											
					}
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (MENUShift){
							case 1: FiltPumpStartHour++;
									if(FiltPumpStartHour >23) FiltPumpStartHour = 0; //����ˮ�ÿ������俪ʼ��Сʱ���ֵ											
									ANTI_OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
									break;
							case 2: FiltPumpStartMin++;
									if(FiltPumpStartMin > 59) FiltPumpStartMin = 0;  //����ˮ�ÿ������俪ʼ�ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
									break;
							case 3: FiltPumpEndHour++;
									if(FiltPumpEndHour > 23) FiltPumpEndHour = 0; //����ˮ�ÿ������������Сʱ���ֵ											
									ANTI_OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//���ټӼ�ʱˢ����ʾСʱֵ
									ANTI_OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
									break;
							case 4: FiltPumpEndMin++;
									if(FiltPumpEndMin > 59) FiltPumpEndMin = 0;  //����ˮ�ÿ�����������ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//���ټӼ�ʱˢ����ʾ����ֵ
									ANTI_OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
									break;
							default:MENUShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					switch (MENUShift){
						case 1: FiltPumpStartHour++;
								if(FiltPumpStartHour >23) FiltPumpStartHour = 0; //����ˮ�ÿ������俪ʼ��Сʱ���ֵ											
								break;
						case 2: FiltPumpStartMin++;
								if(FiltPumpStartMin > 59) FiltPumpStartMin = 0;  //����ˮ�ÿ������俪ʼ�ķ������ֵ											
								break;
						case 3: FiltPumpEndHour++;
								if(FiltPumpEndHour > 23) FiltPumpEndHour = 0; //����ˮ�ÿ������������Сʱ���ֵ											
								break;
						case 4: FiltPumpEndMin++;
								if(FiltPumpEndMin > 59) FiltPumpEndMin = 0;  //����ˮ�ÿ�����������ķ������ֵ
								break;																					
						default:MENUShift = 1; //����
								break;																
					}
				} //�̰�����	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					WriteFlashBuff[7] = FiltPumpStartHour; //���ú�Ĺ���ˮ�ÿ��������俪ʼСʱֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[8] = FiltPumpStartMin; //���ú�Ĺ���ˮ�ÿ��������俪ʼ����ֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[9] = FiltPumpEndHour; //���ú�Ĺ���ˮ�ÿ������������Сʱֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[10] = FiltPumpEndMin; //���ú�Ĺ���ˮ�ÿ����������������ֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
				}
			}
		}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,����ˮ������
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...�Զ�Ͷι����
		if(MENU == 75){	 //�Զ�Ͷι����			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongTouWei_16[0],4);//��ʾ: �Զ�Ͷι����
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
		
			if(AutoFeedMod == 0){ //ģʽֵΪ0,��ʾ�رչ���ˮ��ģʽ,ֻ��ʾ 1.ģʽ
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(AutoFeedMod == 1){ //��ģʽֵΪ1,����������ˮ��ģʽ,��ʾ����ʱ������ѡ��
				if(SUBMENU == 1){ //ѡ��һ����ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.ģʽ
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //ѡ�ж�����ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.�Զ�Ͷιʱ��				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);//
					ANTI_OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.�Զ�Ͷιʱ��
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);//
					OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}	
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(AutoFeedMod == 0){ //������ˮ��ģʽֵΪ0,���رչ���ˮ��״̬,ֻ�ܽ���ģʽ����
						MENU = 141; //ֻ�ܽ��뵽ģʽ��ѡ������
					}
					if(AutoFeedMod == 1){ //������ˮ��ģʽֵΪ1,����������ˮ��״̬,���Խ�����ѡ������
						MENU = 140 + SUBMENU; //���뵽��ѡ������
					}
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(AutoFeedMod == 1){
						if(SUBMENU  == 1) SUBMENU = 3; //��ѡ����Сֵ
						SUBMENU --; //��ѡ���־ֵ��1					
					}
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					if(AutoFeedMod == 1){
						SUBMENU++; //��ѡ���־ֵ��1
						if(SUBMENU > 2) SUBMENU = 1; //��ѡ�����ֵ
					}
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}								
		}

		if(MENU == 141){//�Զ�Ͷι������ѡ�� //ģʽ����
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //��ʾ:  ģʽ���� 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(AutoFeedMod == 1){	//����ˮ��ģʽ��־λΪ1,�������Զ�Ͷι
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//��ʾ:����
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //����ˮ��ģʽ��־λ��Ϊ1,����ر��Զ�Ͷι
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//��ʾ: �ر�
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //���뵽��ѡ������
					WriteFlashBuff[11] = AutoFeedMod; //���ú���Զ�Ͷιģʽֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(AutoFeedMod  == 0) AutoFeedMod = 2; //��ѡ����Сֵ
					AutoFeedMod --; //��ѡ���־ֵ��1
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					AutoFeedMod++; //��ѡ���־ֵ��1
					if(AutoFeedMod > 1) AutoFeedMod = 0; //��ѡ�����ֵ
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					WriteFlashBuff[11] = AutoFeedMod; //���ú���Զ�Ͷιģʽֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					MENU = 1; //��ת��������
				}
			}								
		}
		if(MENU == 142){//�Զ�Ͷι������ѡ��  //�Զ�Ͷιʱ������	  			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongTouWei_16[2*32],2);//��ʾ:Ͷιʱ������
			OLED_DISPLAY_16x16(0,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);

			OLED_DISPLAY_Buff_16x16(4,0*16,&ZiDongTouWei_16[2*32],2);//��ʾ:Ͷιʱ��
			OLED_DISPLAY_16x16(4,2*16,&shij_16[0]);
			OLED_DISPLAY_16x16(4,3*16,&jian_16[0]);

			OLED_DISPLAY_8x16(4,9*8,'-');// ����ˮ�����俪ʼ�ͽ���ʱ���м�ı�ʶ�� -
			
			OLED_DISPLAY_16x16(6,1*16,&chi2_16[0]);	//��ʾ:����ʱ��:  S
			OLED_DISPLAY_16x16(6,2*16,&xu4_16[0]);			
			OLED_DISPLAY_16x16(6,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(6,4*16,&jian_16[0]);
			OLED_DISPLAY_8x16(6,10*8,':');//
			OLED_DISPLAY_8x16(6,13*8,'S');//


			if(MENUShift == 1){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ�����俪ʼ��Сʱֵ
				ANTI_OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//��ʾѡ�е�Ͷιʱ���� һ��5��
				if(FeedTimeHour[FeedTimeNum] == 24){ //��ǰΪ�رո�ʱ�����־λ ��ʾOFF  
					OLED_DISPLAY_8x16(4,10*8,'O');//
					OLED_DISPLAY_8x16(4,11*8,'F');//
					OLED_DISPLAY_8x16(4,12*8,'F');//
					OLED_DISPLAY_8x16(4,13*8,' ');//
					OLED_DISPLAY_8x16(4,14*8,' ');//
					OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//ˢ����ʾ�Զ�ιʳ�ĳ���ʱ�� 0--59s
					OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//				
				}else{ //��ǰ��������ʾ
					OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//ˢ����ʾͶιʱ��Сʱ
					OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,':');//				
					OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//ˢ����ʾͶιʱ�����
					OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//ˢ����ʾ�Զ�ιʳ�ĳ���ʱ�� 0--59s
					OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//				
				}
			}
			if(MENUShift > 1){ //����ˢ����ʾ��������Ľ���Сʱֵ
				if(MENUShift == 2){	//���ױ�����ʾСʱ,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ�����������Сʱֵ
					if(FeedTimeHour[FeedTimeNum] == 24){ //��ǰΪ�رո�ʱ�����־λ ��ʾOFF  
						ANTI_OLED_DISPLAY_8x16(4,10*8,'O');//
						ANTI_OLED_DISPLAY_8x16(4,11*8,'F');//
						ANTI_OLED_DISPLAY_8x16(4,12*8,'F');//
						OLED_DISPLAY_8x16(4,13*8,' ');//
						OLED_DISPLAY_8x16(4,14*8,' ');//
					}else{ //��ǰ��������ʾ
						ANTI_OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//Сʱ
						ANTI_OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
						OLED_DISPLAY_8x16(4,12*8,':');//				
						OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//����
						OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
					}
					OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//ˢ����ʾ�Զ�ιʳ�ĳ���ʱ�� 0--59s
					OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//				
				}
				if(MENUShift == 3){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ����������ķ���ֵ
					OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//����
					ANTI_OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
				}																						   
				if(MENUShift == 4){	 //���ױ�����ʾ����,��ʾ��ʾ�û���ǰѡ���ǹ���ˮ����������ķ���ֵ
					ANTI_OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//��ʾ�Զ�ιʳ�ĳ���ʱ�� 0--59s
					ANTI_OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//
				
					OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//����
					OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
				}

				OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//��ʾѡ�е�Ͷιʱ���� һ��5��
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(MENUShift == 2 && FeedTimeHour[FeedTimeNum] == 24){
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						MENUShift = 1; //����ѡ���־λ��1
						WriteFlashBuff[11+FeedTimeNum] = FeedTimeHour[FeedTimeNum]; //���ú���Զ�ͶιСʱֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[16+FeedTimeNum] = FeedTimeMin[FeedTimeNum]; //���ú���Զ�Ͷι����ֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[21+FeedTimeNum] = FeedTimeLast[FeedTimeNum]; //���ú��Զ�Ͷι����ʱ��ֵ�Ž�д�뻺������,�ȴ�д��
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����					
					}else if(MENUShift < 4){
						MENUShift++;//ʱ������л�ѡ���־��1	
					}else{
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						MENUShift = 1; //ʱ�����ѡ���־λ��1
						WriteFlashBuff[11+FeedTimeNum] = FeedTimeHour[FeedTimeNum]; //���ú���Զ�ͶιСʱֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[16+FeedTimeNum] = FeedTimeMin[FeedTimeNum]; //���ú���Զ�Ͷι����ֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[21+FeedTimeNum] = FeedTimeLast[FeedTimeNum]; //���ú��Զ�Ͷι����ʱ��ֵ�Ž�д�뻺������,�ȴ�д��
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					}
				}

				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (MENUShift){
//							case 1: if(FeedTimeNum == 1) FeedTimeNum = 6; //����ˮ�ÿ������俪ʼ��Сʱ��Сֵ
//									FeedTimeNum--;
//									ANTI_OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//��ʾѡ�е�Ͷιʱ���� һ��5��
//									break;
							case 2: if(FeedTimeHour[FeedTimeNum] <= 0) FeedTimeHour[FeedTimeNum] = 25;  //����ˮ�ÿ������俪ʼ�ķ�����Сֵ
									FeedTimeHour[FeedTimeNum]--;
									ANTI_OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//���ټӼ�ʱˢ����ʾСʱ
									ANTI_OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
									break;
							case 3: if(FeedTimeMin[FeedTimeNum] <= 0) FeedTimeMin[FeedTimeNum] = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
									FeedTimeMin[FeedTimeNum]--;
									ANTI_OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//����
									ANTI_OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
									break;
							case 4: if(FeedTimeLast[FeedTimeNum] <= 0) FeedTimeLast[FeedTimeNum] = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
									FeedTimeLast[FeedTimeNum]--;
									ANTI_OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//��ʾ�Զ�ιʳ�ĳ���ʱ�� 0--59s
									ANTI_OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					switch (MENUShift){
						case 1: if(FeedTimeNum == 1) FeedTimeNum = 6; //����ˮ�ÿ������俪ʼ��Сʱ��Сֵ
								FeedTimeNum--;
								break;
						case 2: if(FeedTimeHour[FeedTimeNum] <= 0) FeedTimeHour[FeedTimeNum] = 25;  //����ˮ�ÿ������俪ʼ�ķ�����Сֵ
								FeedTimeHour[FeedTimeNum]--;
								break;
						case 3: if(FeedTimeMin[FeedTimeNum] <= 0) FeedTimeMin[FeedTimeNum] = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
								FeedTimeMin[FeedTimeNum]--;
								break;
						case 4: if(FeedTimeLast[FeedTimeNum] <= 0) FeedTimeLast[FeedTimeNum] = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
								FeedTimeLast[FeedTimeNum]--;
								break;
						default:MENUShift = 1; //����
								break;											
					}
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (MENUShift){
//							case 1: FeedTimeNum++;
//									if(FeedTimeNum >5) FeedTimeNum = 1; //����ˮ�ÿ������俪ʼ��Сʱ���ֵ											
//									ANTI_OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//��ʾѡ�е�Ͷιʱ���� һ��5��
//									break;
							case 2: FeedTimeHour[FeedTimeNum]++;
									if(FeedTimeHour[FeedTimeNum] > 24) FeedTimeHour[FeedTimeNum] = 0;  //����ˮ�ÿ������俪ʼ�ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//���ټӼ�ʱˢ����ʾСʱ
									ANTI_OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
									break;
							case 3: FeedTimeMin[FeedTimeNum]++;
									if(FeedTimeMin[FeedTimeNum] > 59) FeedTimeMin[FeedTimeNum] = 0;  //����ˮ�ÿ�����������ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//����
									ANTI_OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
									break;
							case 4: FeedTimeLast[FeedTimeNum]++;
									if(FeedTimeLast[FeedTimeNum] > 59) FeedTimeLast[FeedTimeNum] = 0;  //����ˮ�ÿ�����������ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//��ʾ�Զ�ιʳ�ĳ���ʱ�� 0--59s
									ANTI_OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					switch (MENUShift){
						case 1: FeedTimeNum++;
								if(FeedTimeNum >5) FeedTimeNum = 1; //����ˮ�ÿ������俪ʼ��Сʱ���ֵ											
								break;
						case 2: FeedTimeHour[FeedTimeNum]++;
								if(FeedTimeHour[FeedTimeNum] > 24) FeedTimeHour[FeedTimeNum] = 0;  //����ˮ�ÿ������俪ʼ�ķ������ֵ											
								break;
						case 3: FeedTimeMin[FeedTimeNum]++;
								if(FeedTimeMin[FeedTimeNum] > 59) FeedTimeMin[FeedTimeNum] = 0;  //����ˮ�ÿ�����������ķ������ֵ											
								break;
						case 4: FeedTimeLast[FeedTimeNum]++;
								if(FeedTimeLast[FeedTimeNum] > 59) FeedTimeLast[FeedTimeNum] = 0;  //����ˮ�ÿ�����������ķ������ֵ											
								break;																					
						default:MENUShift = 1; //����
								break;																
					}
				} //�̰�����	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					WriteFlashBuff[11+FeedTimeNum] = FeedTimeHour[FeedTimeNum]; //���ú���Զ�ͶιСʱֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[16+FeedTimeNum] = FeedTimeMin[FeedTimeNum]; //���ú���Զ�Ͷι����ֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[21+FeedTimeNum] = FeedTimeLast[FeedTimeNum]; //���ú��Զ�Ͷι����ʱ��ֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
				}
			}
		}
		if(MENU == 149){//��̬��ʾ����Ͷι�˵�  	  			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongTouWei_16[2*32],2);//��ʾ:Ͷι������
			OLED_DISPLAY_16x16(0,3*16,&jin4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&xing2_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhong1_16[0]);

			OLED_DISPLAY_Buff_16x16(4,0*16,&ZiDongTouWei_16[2*32],2);//��ʾ:Ͷιʱ��
			OLED_DISPLAY_16x16(4,2*16,&shij_16[0]);
			OLED_DISPLAY_16x16(4,3*16,&jian_16[0]);

			OLED_DISPLAY_8x16(4,9*8,'-');// ����ˮ�����俪ʼ�ͽ���ʱ���м�ı�ʶ�� -
			
			OLED_DISPLAY_16x16(6,1*16,&chi2_16[0]);	//��ʾ:����ʱ��:  S
			OLED_DISPLAY_16x16(6,2*16,&xu4_16[0]);			
			OLED_DISPLAY_16x16(6,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(6,4*16,&jian_16[0]);
			OLED_DISPLAY_8x16(6,10*8,':');//
			OLED_DISPLAY_8x16(6,13*8,'S');//

			OLED_DISPLAY_8x16(4,8*8,FeedTriggerNum%10+0x30);//��ʾ������Ͷιʱ���� һ��5��
			OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTriggerNum]/10+0x30);//ˢ����ʾ������Ͷιʱ��Сʱ
			OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTriggerNum]%10+0x30);//
			OLED_DISPLAY_8x16(4,12*8,':');//				
			OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTriggerNum]/10+0x30);//ˢ����ʾ������Ͷιʱ�����
			OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTriggerNum]%10+0x30);//
			MENU = 150; //��ת����̬ˢ����ʾ����Ͷι����ʱ�˵�
		}
		if(MENU == 150){ //��̬ˢ����ʾ����Ͷι����ʱ�˵�
			RTC_Get();//������ǰʱ��ֵ
			if(rsec%2 == 1 && FeedLastCountFlag != 1){
				FeedLastCount -= 1; //ÿ����һ�ξ���1��
				FeedLastCountFlag = 1;
				TIM3_PWM_CH3_Init(59999,23); //Time3ͨ��3��ʼ��,
				TIM_SetCompare3(TIM3,1500);  //���0�Ƚ�
				OLED_DISPLAY_8x16(6,11*8,FeedLastCount/10+0x30);//����ʱˢ����ʾ������ιʳ����ʱ�� 0--59s
				OLED_DISPLAY_8x16(6,12*8,FeedLastCount%10+0x30);//				
			}else if(rsec%2 == 0 && FeedLastCountFlag != 0){
				FeedLastCount -= 1; //ÿ����һ�ξ���1��
				FeedLastCountFlag = 0;
				TIM3_PWM_CH3_Init(59999,23); //Time3ͨ��3��ʼ��,
				TIM_SetCompare3(TIM3,7500);  //���180�Ƚ�
				OLED_DISPLAY_8x16(6,11*8,FeedLastCount/10+0x30);//����ʱˢ����ʾ������ιʳ����ʱ�� 0--59s
				OLED_DISPLAY_8x16(6,12*8,FeedLastCount%10+0x30);//				
			}
			if(FeedLastCount == 0){	//�������ʱ����
				MENU = 1; //��ת�����˵�
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}		
		}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,�Զ�Ͷι����
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....�Զ���ˮ����
		if(MENU == 76){	 //	�Զ���ˮ����		
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongHuanShui_16[0],4);//��ʾ:	�Զ���ˮ����
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
		
			if(ChangeWaterMod == 0){ //ģʽֵΪ0,��ʾ�ر��Զ���ˮģʽ,ֻ��ʾ 1.ģʽ
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(ChangeWaterMod == 1){ //��ģʽֵΪ1,�������Զ���ˮģʽ,��ʾ����ʱ������ѡ��
				if(SUBMENU == 1){ //ѡ��һ����ѡ��
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//���ױ�����ʾ:1.ģʽ				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//������ʾ:1.ģʽ
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //ѡ�ж�����ѡ��
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//���ױ�����ʾ:2.�Զ���ˮʱ��				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongHuanShui_16[0],4);//
					ANTI_OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//������ʾ:2.�Զ���ˮʱ��
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongHuanShui_16[0],4);//
					OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}	
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(ChangeWaterMod == 0){ //���Զ���ˮģʽֵΪ0,���ر��Զ���ˮ״̬,ֻ�ܽ���ģʽ����
						MENU = 151; //ֻ�ܽ��뵽ģʽ��ѡ������
					}
					if(ChangeWaterMod == 1){ //���Զ���ˮģʽֵΪ1,�������Զ���ˮ״̬,���Խ�����ѡ������
						MENU = 150 + SUBMENU; //���뵽��ѡ������
					}
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(ChangeWaterMod == 1){
						if(SUBMENU  == 1) SUBMENU = 3; //��ѡ����Сֵ
						SUBMENU --; //��ѡ���־ֵ��1					
					}
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					if(ChangeWaterMod == 1){
						SUBMENU++; //��ѡ���־ֵ��1
						if(SUBMENU > 2) SUBMENU = 1; //��ѡ�����ֵ
					}
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}								
		}

		if(MENU == 151){//�Զ���ˮģʽ����
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //��ʾ:  ģʽ���� 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(ChangeWaterMod == 1){	//ģʽ��־λΪ1,�������Զ���ˮ
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//��ʾ:����
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //ģʽ��־λ��Ϊ1,����ر��Զ���ˮ
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//��ʾ: �ر�
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					MENU = 70 + MENUSET; //���뵽��ѡ������
					WriteFlashBuff[27] = ChangeWaterMod; //���ú���Զ���ˮģʽֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					OLED_DISPLAY_CLEAR();//oled��������
				}
				if(KEY == 2){ //��������
					KEY = 0; //��0��־λ
					if(ChangeWaterMod  == 0) ChangeWaterMod = 2; //��ѡ����Сֵ
					ChangeWaterMod --; //��ѡ���־ֵ��1
				}
				if(KEY == 3){ //��������
					KEY = 0; //��0��־λ
					ChangeWaterMod++; //��ѡ���־ֵ��1
					if(ChangeWaterMod > 1) ChangeWaterMod = 0; //��ѡ�����ֵ
				}
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					WriteFlashBuff[27] = ChangeWaterMod; //���ú���Զ���ˮģʽֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//һ����д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					MENU = 1; //��ת��������
				}
			}								
		}

		if(MENU == 152){//�Զ���ˮʱ������	  			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongHuanShui_16[2*32],2);//��ʾ:��ˮʱ������
			OLED_DISPLAY_16x16(0,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);

			OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongHuanShui_16[2*32],2);//��ʾ:��ˮʱ��
			OLED_DISPLAY_16x16(4,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);

			OLED_DISPLAY_16x16(6,1*16,&xing_16[0]);//��ʾ: ����
			OLED_DISPLAY_16x16(6,2*16,&qi1_16[0]);
			
			if(MENUShift == 1){	//���ױ�����ʾ,��ʾ��ʾ�û���ǰѡ���ǻ�ˮʱ����
				ANTI_OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//��ʾѡ�еĻ�ˮʱ���� һ��5��
				if(ChangWatWeek[ChangWatTimNum] == 7){ //��ǰΪ�رո�ʱ�����־λ ��ʾOFF  
					OLED_DISPLAY_8x16(6,6*8,'O');//
					OLED_DISPLAY_8x16(6,7*8,'F');//
					OLED_DISPLAY_8x16(6,8*8,'F');//
					OLED_DISPLAY_8x16(6,9*8,' ');//
					OLED_DISPLAY_8x16(6,10*8,' ');//
					OLED_DISPLAY_8x16(6,11*8,' ');//
					OLED_DISPLAY_8x16(6,12*8,' ');//
					OLED_DISPLAY_8x16(6,13*8,' ');//
				}else{ //��ǰ��������ʾ
					OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//��ʾ����һ~~��
					OLED_DISPLAY_8x16(6,8*8,'-');// ����?��ʱ���м�ı�ʶ�� -
					OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,':');//				
					OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//����
					OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
				}
			}
			if(MENUShift > 1){ //����ˢ����ʾ��������Ľ���Сʱֵ
				if(MENUShift == 2){	//���ױ�����ʾ,��ʾ��ʾ�û���ǰѡ��������ֵ
					if(ChangWatWeek[ChangWatTimNum] == 7){ //��ǰΪ�رո�ʱ�����־λ ��ʾOFF  
						ANTI_OLED_DISPLAY_8x16(6,6*8,'O');//
						ANTI_OLED_DISPLAY_8x16(6,7*8,'F');//
						ANTI_OLED_DISPLAY_8x16(6,8*8,'F');//
						OLED_DISPLAY_8x16(6,9*8,' ');//
						OLED_DISPLAY_8x16(6,10*8,' ');//
						OLED_DISPLAY_8x16(6,11*8,' ');//
						OLED_DISPLAY_8x16(6,12*8,' ');//
						OLED_DISPLAY_8x16(6,13*8,' ');//
					}else{ //��ǰ��������ʾ
						ANTI_OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//��ʾ����һ~~��
						OLED_DISPLAY_8x16(6,8*8,'-');// ����?��ʱ���м�ı�ʶ�� -
						OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//Сʱ
						OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
						OLED_DISPLAY_8x16(6,11*8,':');//				
						OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//����
						OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
					}
				}
				if(MENUShift == 3){	 //���ױ�����ʾ,��ʾ��ʾ�û���ǰѡ����Сʱֵ
					ANTI_OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//Сʱ
					ANTI_OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,':');//				
					OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//����
					OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//

					OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//��ǰ����ѡ������  ������ʾ����һ~~��
				}																						   
				if(MENUShift == 4){	 //���ױ�����ʾ,��ʾ�û���ǰѡ���Ƿ���ֵ				
					OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//Сʱ
					OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//����
					ANTI_OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//��ʾѡ�еĻ�ˮʱ���� һ��5��
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������
				if(KEY == 1){ //����1������A����
					KEY = 0; //��0��־λ
					if(MENUShift == 2 && ChangWatWeek[ChangWatTimNum] == 7){
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						MENUShift = 1; //����ѡ���־λ��1
						WriteFlashBuff[27+ChangWatTimNum] = ChangWatWeek[ChangWatTimNum]; //���ú���Զ���ˮСʱֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[32+ChangWatTimNum] = ChangWatTimHour[ChangWatTimNum]; //���ú���Զ���ˮ����ֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[37+ChangWatTimNum] = ChangWatTimMin[ChangWatTimNum]; //���ú��Զ���ˮ����ʱ��ֵ�Ž�д�뻺������,�ȴ�д��
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					}else if(MENUShift < 4){
						MENUShift++;//ʱ������л�ѡ���־��1	
					}else{
						MENU = 70 + MENUSET; //�˻ص���ѡ������
						OLED_DISPLAY_CLEAR();//oled��������
						MENUShift = 1; //ʱ�����ѡ���־λ��1
						WriteFlashBuff[27+ChangWatTimNum] = ChangWatWeek[ChangWatTimNum]; //���ú���Զ���ˮСʱֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[32+ChangWatTimNum] = ChangWatTimHour[ChangWatTimNum]; //���ú���Զ���ˮ����ֵ�Ž�д�뻺������,�ȴ�д��
						WriteFlashBuff[37+ChangWatTimNum] = ChangWatTimMin[ChangWatTimNum]; //���ú��Զ���ˮ����ʱ��ֵ�Ž�д�뻺������,�ȴ�д��
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
					}
				}
				if(KEY == 12){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (MENUShift){
//							case 1: if(ChangWatTimNum == 1) ChangWatTimNum = 6; //����ˮ�ÿ������俪ʼ��Сʱ��Сֵ
//									ChangWatTimNum--;
//									ANTI_OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//��ʾѡ�еĻ�ˮʱ���� һ��5��
//									break;
//							case 2: ChangWatWeek[ChangWatTimNum]--;
//									if(ChangWatWeek[ChangWatTimNum] > 6) ChangWatWeek[ChangWatTimNum] = 7;  //������Сֵ									
//									ANTI_OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//���ټӼ�ʱˢ����ʾ����һ~~��
//									break;
							case 3: if(ChangWatTimHour[ChangWatTimNum] <= 0) ChangWatTimHour[ChangWatTimNum] = 24;  //����ˮ�ÿ������俪ʼ�ķ�����Сֵ
									ChangWatTimHour[ChangWatTimNum]--;
									ANTI_OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//���ټӼ�ʱˢ����ʾСʱ
									ANTI_OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
									break;
							case 4: if(ChangWatTimMin[ChangWatTimNum] <= 0) ChangWatTimMin[ChangWatTimNum] = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
									ChangWatTimMin[ChangWatTimNum]--;
									ANTI_OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//���ټӼ�ʱˢ����ʾ����
									ANTI_OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 2){ //�����̰�
					KEY = 0; //��0��־λ
					switch (MENUShift){
						case 1: if(ChangWatTimNum == 1) ChangWatTimNum = 6; //����ˮ�ÿ������俪ʼ��Сʱ��Сֵ
								ChangWatTimNum--;
								break;
						case 2: ChangWatWeek[ChangWatTimNum]--;
								if(ChangWatWeek[ChangWatTimNum] > 6) ChangWatWeek[ChangWatTimNum] = 7;  //������Сֵ									
								break;
						case 3: if(ChangWatTimHour[ChangWatTimNum] <= 0) ChangWatTimHour[ChangWatTimNum] = 24;  //����ˮ�ÿ������俪ʼ�ķ�����Сֵ
								ChangWatTimHour[ChangWatTimNum]--;
								break;
						case 4: if(ChangWatTimMin[ChangWatTimNum] <= 0) ChangWatTimMin[ChangWatTimNum] = 60;  //����ˮ�ÿ�����������ķ�����Сֵ
								ChangWatTimMin[ChangWatTimNum]--;
								break;
						default:MENUShift = 1; //����
								break;											
					}
				} //�̰�����
				if(KEY == 13){ //��������
					KEY = 0; //��0��־λ
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (MENUShift){
//							case 1: ChangWatTimNum++;
//									if(ChangWatTimNum >5) ChangWatTimNum = 1; //����ˮ�ÿ������俪ʼ��Сʱ���ֵ											
//									ANTI_OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//��ʾѡ�еĻ�ˮʱ���� һ��5��
//									break;
//							case 2: ChangWatWeek[ChangWatTimNum]++;
//									if(ChangWatWeek[ChangWatTimNum] > 7) ChangWatWeek[ChangWatTimNum] = 0;  //�������ֵ									
//									ANTI_OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//���ټӼ�ʱˢ����ʾ����һ~~��
//									break;
							case 3: ChangWatTimHour[ChangWatTimNum]++;
									if(ChangWatTimHour[ChangWatTimNum] > 23) ChangWatTimHour[ChangWatTimNum] = 0;  //����ˮ�ÿ������俪ʼ�ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//���ټӼ�ʱˢ����ʾСʱ
									ANTI_OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
									break;
							case 4: ChangWatTimMin[ChangWatTimNum]++;
									if(ChangWatTimMin[ChangWatTimNum] > 59) ChangWatTimMin[ChangWatTimNum] = 0;  //����ˮ�ÿ�����������ķ������ֵ											
									ANTI_OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//���ټӼ�ʱˢ����ʾ����
									ANTI_OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//����
									break;											
						}
						delay_ms(60); //����һ�����ʵ���ֵ�仯�ٶ�																
					}
				}//��������				
				if(KEY == 3){ //�����̰�
					KEY = 0; //��0��־λ
					switch (MENUShift){
						case 1: ChangWatTimNum++;
								if(ChangWatTimNum >5) ChangWatTimNum = 1; //����ˮ�ÿ������俪ʼ��Сʱ���ֵ											
								break;
						case 2: ChangWatWeek[ChangWatTimNum]++;
								if(ChangWatWeek[ChangWatTimNum] > 7) ChangWatWeek[ChangWatTimNum] = 0;  //�������ֵ									
								break;
						case 3: ChangWatTimHour[ChangWatTimNum]++;
								if(ChangWatTimHour[ChangWatTimNum] > 23) ChangWatTimHour[ChangWatTimNum] = 0;  //����ˮ�ÿ������俪ʼ�ķ������ֵ											
								break;
						case 4: ChangWatTimMin[ChangWatTimNum]++;
								if(ChangWatTimMin[ChangWatTimNum] > 59) ChangWatTimMin[ChangWatTimNum] = 0;  //����ˮ�ÿ�����������ķ������ֵ											
								break;
						default:MENUShift = 1; //����
								break;																
					}
				} //�̰�����	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
					WriteFlashBuff[27+ChangWatTimNum] = ChangWatWeek[ChangWatTimNum]; //���ú���Զ���ˮСʱֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[32+ChangWatTimNum] = ChangWatTimHour[ChangWatTimNum]; //���ú���Զ���ˮ����ֵ�Ž�д�뻺������,�ȴ�д��
					WriteFlashBuff[37+ChangWatTimNum] = ChangWatTimMin[ChangWatTimNum]; //���ú��Զ���ˮ����ʱ��ֵ�Ž�д�뻺������,�ȴ�д��
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//д��������, ������WriteFlashBuff[]����,�����ɺ궨��WriteNum����
				}
			}
		}

		if(MENU == 159){//��̬��ʾ������ˮ�˵�  	  			
			ANTI_OLED_DISPLAY_8x16_BUFFER(0,"                ");	//��յ�һ��
			ANTI_OLED_DISPLAY_16x16(0,0*16,&zheng4_16[0]);//��ʾ:���ڳ������ˮ
			ANTI_OLED_DISPLAY_16x16(0,1*16,&zai4_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,2*16,&chou1_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,3*16,&chu1_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,4*16,&gang1_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,5*16,&nei4_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,6*16,&shui_16[0]);

			OLED_DISPLAY_16x16(2,0*16,&gang1_16[0]);//��ʾ:����ˮ��:   . C
			OLED_DISPLAY_16x16(2,1*16,&nei4_16[0]);
			OLED_DISPLAY_16x16(2,2*16,&shui_16[0]);	 
			OLED_DISPLAY_16x16(2,3*16,&wen_16[0]);
			OLED_DISPLAY_8x16(2,8*8,':');// 
			OLED_DISPLAY_8x16(2,12*8,'.');// 
			OLED_DISPLAY_16x16(2,14*8,&C_16[0]); //��ʾ�¶ȷ��š�//

			OLED_DISPLAY_16x16(4,0*16,&xin1_16[0]);//��ʾ:��ˮˮ��:   . C
			OLED_DISPLAY_16x16(4,1*16,&shui_16[0]);
			OLED_DISPLAY_16x16(4,2*16,&shui_16[0]);	 
			OLED_DISPLAY_16x16(4,3*16,&wen_16[0]);
			OLED_DISPLAY_8x16(4,8*8,':');// 
			OLED_DISPLAY_8x16(4,12*8,'.');// 
			OLED_DISPLAY_16x16(4,14*8,&C_16[0]); //��ʾ�¶ȷ��š�//

			OLED_DISPLAY_16x16(6,0*16,&shang4_16[0]);//��ʾ:��ˮλ ��ˮλ
			OLED_DISPLAY_16x16(6,1*16,&shui_16[0]);//
			OLED_DISPLAY_16x16(6,2*16,&wei4_16[0]);//

			OLED_DISPLAY_16x16(6,4*16,&xia4_16[0]);//
			OLED_DISPLAY_16x16(6,5*16,&shui_16[0]);//
			OLED_DISPLAY_16x16(6,6*16,&wei4_16[0]);//

			MENU = 160; //��ת����̬ˢ����ʾ������ˮ״̬ˢ�²˵�
		}

		if(MENU == 160){ //��̬ˢ����ʾ������ˮ״̬ˢ�²˵�
			//��ʾ�¶ȴ�����1
			if(Temp > 0){//�ж�Ϊ���¶�
				OLED_DISPLAY_8x16(2,9*8,' ');//��ʾ�¶�ֵ
				OLED_DISPLAY_8x16(2,10*8,Temp%1000/100+0x30);//��ʾ�¶�ֵ,�¶ȷ�Χ-55~~99
				OLED_DISPLAY_8x16(2,11*8,Temp%100/10+0x30);//
				OLED_DISPLAY_8x16(2,13*8,Temp%10+0x30);//
			}
			//��ʾ�¶ȴ�����2
			if(Temp2 > 0){//�ж�Ϊ���¶�
				OLED_DISPLAY_8x16(4,9*8,' ');//��ʾ�¶�ֵ
				OLED_DISPLAY_8x16(4,10*8,Temp2%1000/100+0x30);//��ʾ�¶�ֵ,-550~~1250
				OLED_DISPLAY_8x16(4,11*8,Temp2%100/10+0x30);//
				OLED_DISPLAY_8x16(4,13*8,Temp2%10+0x30);//
			}

			if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_HIGH) == 1){ //�����ˮλ��⵽��ˮ
				OLED_DISPLAY_16x16(6,3*16,&gou1_16[0]);//��ʾ�� ��
			}else{ //��ˮλ��⵽û��ˮ
				OLED_DISPLAY_16x16(6,3*16,&cha1_16[0]);//��ʾ�� �� 
			}
			if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_LOW) == 1){ //�����ˮλ��⵽��ˮ
				OLED_DISPLAY_16x16(6,7*16,&gou1_16[0]);//��ʾ�� ��
			}else{// ��ˮλ��⵽û��ˮ
				OLED_DISPLAY_16x16(6,7*16,&cha1_16[0]);//��ʾ�� �� 
			}

			if(ChangWatProcess == 1){ //��ˮ�ĵ�һ�� ���ˮ�²� ���ܳ�����3��
				if((Temp - Temp2) > 30 || (Temp2 - Temp) > 30 ){ //�����ˮˮ�±ȸ���ˮ�µͻ��3��, ��ˮ���쳣
					ANTI_OLED_DISPLAY_8x16_BUFFER(0,"                ");	//��յ�һ��
					ANTI_OLED_DISPLAY_16x16(0,0*16,&shui_16[0]);//��ʾ:ˮ���쳣
					ANTI_OLED_DISPLAY_16x16(0,1*16,&wen_16[0]);
					ANTI_OLED_DISPLAY_16x16(0,2*16,&yi4_16[0]);
					ANTI_OLED_DISPLAY_16x16(0,3*16,&chang2_16[0]);
					ChangWatProcess = 1; //ˮ���쳣,������ˮ����					
				}else{					 
					 ChangWatProcess = 2; //ˮ������,����ˮ����
				}
 			}
			if(ChangWatProcess == 2){ //��ˮ�ĵڶ���
				ANTI_OLED_DISPLAY_16x16(0,0*16,&zheng4_16[0]);//��ʾ:���ڳ������ˮ
				ANTI_OLED_DISPLAY_16x16(0,1*16,&zai4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,2*16,&chou1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,3*16,&chu1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,4*16,&gang1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,5*16,&nei4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,6*16,&shui_16[0]);
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS1,(BitAction)(1));//�������ڵĳ�ˮ��
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS2,(BitAction)(0));//�ر���ˮ�ĳ�ˮ��
				if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_LOW) == 0 && GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_HIGH) == 0){ //�����ˮλ��⵽û��ˮ
					ChangWatProcess = 3; //��ת����ˮ�ĵ�����
				}
			} 
			if(ChangWatProcess == 3){	//��ˮ�ĵ�����
				ANTI_OLED_DISPLAY_16x16(0,0*16,&zheng4_16[0]);//��ʾ:���ڳ���ˮ����
				ANTI_OLED_DISPLAY_16x16(0,1*16,&zai4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,2*16,&chou1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,3*16,&xin1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,4*16,&shui_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,5*16,&jin4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,6*16,&gang1_16[0]);
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS1,(BitAction)(0));//�رո��ڵĳ�ˮ��
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS2,(BitAction)(1));//������ˮ�ĳ�ˮ��
				if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_HIGH) == 1 && GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_LOW) == 1){ //�����ˮλ��⵽��ˮ ��ˮ���̽���
					ChangWatProcess = 1; //��ˮ���̱�־λ��1
					MENU = 1; //��ת��������
					GPIO_WriteBit(MOS_SENSOR_PORT,MOS1,(BitAction)(0));//�رո��ڵĳ�ˮ��
					GPIO_WriteBit(MOS_SENSOR_PORT,MOS2,(BitAction)(0));//�ر���ˮ�ĳ�ˮ��
				}
			}
			if(KEY != 0){ //�����жϱ�־λ��Ϊ0,���ʾ�а�������	
				if(KEY == 4){ //��������
					KEY = 0; //��0��־λ
					MENU = 1; //��ת��������
				}
			}		
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,�Զ���ˮ����
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,���ý���
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...�жϴ�������
		if(MENU < 70){//�˵�С��70,���������ý���ʱ�Żᴥ��
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...ˮ�¼���жϲ��� ͨ���̵���������رռ��Ȱ�
			if(WatTemMod == 1){	//������¶�ģʽֵΪ0,��ʾ�������ˮ��ʱ
				if(Temp /10 <= MinWatTemp && R1Flag != 1){//��ǰ�¶ȵ����¶�����ʱ
					RELAY_1(1); //�򿪼��Ȱ�
					R1Flag = 1;
				}
				if(Temp /10 >= MaxWatTemp && R1Flag != 2){//��ǰ�¶ȸ����¶�����ʱ
					RELAY_1(0); //�رռ��Ȱ�
					R1Flag = 2;
				}			
			}
			if(WatTemMod == 0){//������¶�ģʽֵΪ0,��ʾ�رռ��ˮ��ʱ
				if(R1Flag != 0){
					RELAY_1(0); //�رռ��Ȱ�
					R1Flag = 0; 
				}
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,ˮ�¼���жϲ��� ͨ���̵���������رռ��Ȱ�
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...���ܵƹ��жϲ��� ͨ��mos��1������رջ���ڵƹ�
			if(LightMod == 1){//���ܵƹ�ģʽ��1����ʱ�俪��ģʽʱ  ���˼·: 00:00--�ر�--08:00--����--17:00--����--21:00--�ر�--23:59
				if(rhour == DayTimeStartHour && rmin < DayTimeStartMin && Mos1Flag != 0){//��ǰʱ��С�ڰ������俪ʼʱ��ʱ 
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,0); //PWM��0��ʾ�ص�
					Mos1Flag = 0;
				}else if(rhour < DayTimeStartHour && Mos1Flag != 0){//�����ǰСʱС�ڰ������俪ʼСʱֱ�ӹر���׵ƹ�
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,0); //PWM��0��ʾ�ص�
					Mos1Flag = 0; 
				}							
				if(rhour == DayTimeStartHour && rmin >= DayTimeStartMin && Mos1Flag != 1){//��ǰʱ����ڰ������俪ʼʱ��ʱ
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,DayTimeGear); //���հ�������ƹ⵵λֵ����PWM������׵�����
					Mos1Flag = 1;
				}else if(rhour > DayTimeStartHour && rhour <= DayTimeEndHour && Mos1Flag != 1){//�����ǰСʱ���ڰ������俪ʼСʱ�����жϷ���ֱֵ�ӿ����ƹ�
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,DayTimeGear); //���հ�������ƹ⵵λֵ����PWM������׵�����
					Mos1Flag = 1;
				}	
				if(rhour == DayTimeEndHour && rmin >= DayTimeEndMin && Mos1Flag != 2){//��ǰʱ����ڰ����������ʱ��ʱ
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,DuskTimeGear); //���հ�������ƹ⵵λֵ����PWM������׵�����
					Mos1Flag = 2;
				}else if(rhour > DayTimeEndHour && rhour <= DuskTimeEndHour && Mos1Flag != 2){//�����ǰСʱ���ڰ����������Сʱ����С�ڰ����������Сʱ�����жϷ���ֱֵ�ӿ����ƹ�
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,DuskTimeGear); //���հ�������ƹ⵵λֵ����PWM������׵�����
					Mos1Flag = 2;
				}	
				if(rhour == DuskTimeEndHour && rmin >= DuskTimeEndMin && Mos1Flag != 3){//��ǰʱ����ڰ����������ʱ��ʱ
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,0); //PWM��0��ʾ�ص�
					Mos1Flag = 3;
				}else if(rhour > DuskTimeEndHour && Mos1Flag != 3){//��ǰСʱ���ڰ����������Сʱ�����жϷ���ֱֵ�ӹرյƹ�
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,0); //���հ�������ƹ⵵λֵ����PWM������׵�����
					Mos1Flag = 3;
				}
			}
			if(LightMod == 2){//��ģʽֵΪ2,��ʾ���ջ��������ٽ�ֵ����׵ƹ�ģʽʱ
				if(EquaADC <= LuxSwitch && Mos1Flag != 1){//��ǰ��������ֵ<=�趨������ֵʱ
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,100); //����
					Mos1Flag = 1;
				}
				if(EquaADC > (LuxSwitch+300) && Mos1Flag != 2){//��ǰ��������ֵ>�趨������ֵ��ƫ��300ʱ(Ŀ���ǲ�������ֵ���ٽ������ʱmos��Ҳ���ſ��ٿ���)
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,0); //�ص�
					Mos1Flag = 2;
				}									
			}
			if(LightMod == 0){//��ģʽֵΪ0,��ʾ�ر����ܵƹ�ģʽʱ
				if(Mos1Flag != 0){
					TIM3_PWM_CH4_Init(100,63); //Time3ͨ��4��ʼ��,
					TIM_SetCompare4(TIM3,0); //PWM��0��ʾ������
					Mos1Flag = 0; 
				}			
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...���ܵƹ��жϲ��� ͨ��mos��1������رջ���ڵƹ�
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...����ˮ���жϲ��� ͨ��R2�̵���2����ˮ�ÿ�����ر�
			if(FiltPumpMod == 1){	//������ˮ��ģʽֵΪ1,��ʾ��������ˮ��ʱ �������: 00:00--�ر�--08:00--����--21:00--�ر�--23:59
				if(rhour == FiltPumpStartHour && rmin < FiltPumpStartMin && R2Flag != 0){//��ǰʱ��С�ڹ���ˮ�ÿ������俪ʼʱ��ʱ 
					RELAY_2(0); //�رչ���ˮ��
					R2Flag = 0;
				}else if(rhour < FiltPumpStartHour && R2Flag != 0){//�����ǰСʱС�ڿ�ʼСʱֱ�ӹرչ���ˮ��
					RELAY_2(0); //�رչ���ˮ��
					R2Flag = 0; 
				}							
				if(rhour == FiltPumpStartHour && rmin >= FiltPumpStartMin && R2Flag != 1){//��ǰʱ����ڹ���ˮ�ÿ������俪ʼʱ��ʱ
					RELAY_2(1); //��������ˮ��
					R2Flag = 1;
				}else if(rhour > FiltPumpStartHour && rhour < FiltPumpEndHour && R2Flag != 1){//�����ǰСʱ���ڿ�ʼСʱ�����жϷ���ֱֵ�ӿ����ƹ�
					RELAY_2(1); //��������ˮ��
					R2Flag = 1;
				}	
				if(rhour == FiltPumpEndHour && rmin >= FiltPumpEndMin && R2Flag != 2){//��ǰʱ����ڹ���ˮ�ÿ����������ʱ��ʱ
					RELAY_2(0); //�رչ���ˮ��
					R2Flag = 2;
				}else if(rhour > FiltPumpEndHour && R2Flag != 2){//��ǰСʱ���ڹ���ˮ�ÿ����������Сʱ�����жϷ���ֱֵ�ӹرչ���ˮ��
					RELAY_2(0); //�رչ���ˮ��
					R2Flag = 2;
				}
			}
			if(FiltPumpMod == 0){//������ˮ��ģʽֵΪ0,��ʾ�رչ���ˮ��ʱ
				if(R2Flag != 0){
					RELAY_2(0); //�رչ���ˮ��
					R2Flag = 0; 
				}
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,����ˮ���жϲ��� ͨ��R2�̵���2����ˮ�ÿ�����ر�
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....�Զ�Ͷι�жϲ���, ͨ�����ģ��
			if(AutoFeedMod == 1){//���Զ�ͶιģʽֵΪ1,��ʾ�����Զ�Ͷιʱ
				for(i=1; i<6; i++){ //�ж�5��Ͷιʱ���Ƿ���ϴ�������
					if(FeedTimeHour[i] != 24 && Steer1Flag == 0){ //Ͷιʱ�����Сʱֵ������off�رո����־λ24
						if(rhour == FeedTimeHour[i] && rmin == FeedTimeMin[i]){//��ǰʱ�����5��Ͷιʱ������һʱ��ʱ
							MENU = 149; //��ת����ʾͶι����ʱ�˵�
							FeedTriggerNum = i;	//����������ֵ���
							FeedLastCount = FeedTimeLast[FeedTriggerNum]; //������ʾ����ʱ
							Steer1Flag = 1; //���״̬��־λ��1
							OLED_DISPLAY_CLEAR();//��������
						}
					}
				}				
				if(rmin != FeedTimeMin[FeedTriggerNum] && Steer1Flag == 1){ //��ǰ���Ӳ����ڴ����ķ���ֵ���Ҷ��������״̬1
					Steer1Flag = 0; //���״̬��־λ����	
				}
			}			
			if(AutoFeedMod == 0 && Steer1Flag != 0){//���Զ�ͶιģʽֵΪ0,��ʾ�ر��Զ�Ͷιʱ
				Steer1Flag = 0;
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,�Զ�Ͷι�жϲ���, ͨ�����ģ��
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....�Զ���ˮ�жϲ���,
			if(ChangeWaterMod == 1){//��ģʽֵΪ1,��ʾ�����Զ���ˮʱ
				for(i=1; i<6; i++){ //�ж�5�黻ˮʱ���Ƿ���ϴ�������
					if(ChangWatWeek[i] != 7 && ChangWatFlag != 1){ //��ˮʱ���������ֵ������off�رո����־λ 7
						if(rweek == ChangWatWeek[i] && rhour == ChangWatTimHour[i] && rmin == ChangWatTimMin[i]){//��ǰʱ�����5����ˮʱ������һʱ��ʱ
							MENU = 159; //��ת����ʾͶι����ʱ�˵�
							ChangWatTrigNum = i; //���´�����ʱ����ֵ
							ChangWatFlag = 1; //��ˮ״̬��־λ��1
							ChangWatProcess	= 1; //��ˮ���̱�־λ��1
							OLED_DISPLAY_CLEAR();//��������
						}
					}
				}				
				if(rmin != ChangWatTimMin[ChangWatTrigNum] && ChangWatFlag != 0){ //��ǰ���Ӳ����ڴ����Ļ�ˮ����ֵ
					ChangWatFlag = 0; //��ˮ״̬��־λ����	
				}
			}			
			if(ChangeWaterMod == 0 && ChangWatFlag != 0){//��ģʽֵΪ0,��ʾ�ر��Զ���ˮʱ
				ChangWatFlag = 0;
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,�Զ���ˮ�жϲ���,
		}//if(MENU < 70)
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...�жϴ�������
	}//��ѭ��
}//������

/*********************************************************************************************

���������塿
u32     a; //����32λ�޷��ű���a
u16     a; //����16λ�޷��ű���a
u8     a; //����8λ�޷��ű���a
vu32     a; //�����ױ��32λ�޷��ű���a
vu16     a; //�����ױ�� 16λ�޷��ű���a
vu8     a; //�����ױ�� 8λ�޷��ű���a
uc32     a; //����ֻ����32λ�޷��ű���a
uc16     a; //����ֻ�� ��16λ�޷��ű���a
uc8     a; //����ֻ�� ��8λ�޷��ű���a

delay_us(1); //��ʱ1΢��
delay_ms(1); //��ʱ1����
delay_s(1); //��ʱ1��

*/



