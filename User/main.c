/*********************************************************************************************
模板制作：  Damon
程序名：	智能鱼缸系统
编写人：	黄威铭	
编写时间：	2021年08月03日
硬件支持：	STM32F103C8  外部晶振8MHz RCC函数设置主频72MHz 　外部低速晶振32.768KHz  

修改日志：　　
1-202108030916 加入了主界面和副界面,按键中断处理,实现主/副界面的按键切换	
2-202108031452 加入设置菜单界面,加减按键的中断处理函数	
3-202108031818 加入了温度监测的具体设置,温度值保存在后备寄存器,水温上下限控制继电器程序
4-202108041242 加入设置菜单子选项时间设置功能,
5-202108051036 加入单总线读两个温度传感器驱动,
6-202108051832 加入智能灯光子菜单,选项设置菜单等待下一次加入
7-202108052205 加入了智能灯光菜单下大白天区间
8-202108061144 优化了按键中断函数,现在更简洁高效了
9-202108062044 加入开灯亮度临界值设置界面
10-202108071655 加入了过滤水泵设置界面并添加判断条件以控制继电器2实现水泵的开启和关闭
11-202108091451 加入自动投喂菜单和判断触发条件
12-202108101703 把单总线读多个温度传感器改成多个io口读取方式,flash地址改到正确地址第63页,之前是127页.
							
说明：	A键是切换按键, 
		B键是数据减按键,长按快速减.
		C键是数据加键,长按快速加.
		D键是进入/退出设置按键


*********************************************************************************************/
#include "stm32f10x.h" //STM32头文件
#include "sys.h"
#include "delay.h"
#include "CHS_16x16.h" //汉字字库 

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

#define FLASH_START_ADDR  0X0800FC00	  //写入的起始地址 第63页  第64k
//#define FLASH_START_ADDR  0x0801f000	  //写入的起始地址
#define WriteNum  43 //需要写入的数据数量, 每多写入一个数据,对应的数量值也要修改
#define WriteFlashBuffNum  43 //需要写入的数组缓存数据数量, 每多写入一个数据,对应的数量值也要修改

extern unsigned char DS18B20_ID[2][8];//存放检测到的传感器ID存数组
extern unsigned char DS18B20_SensorNum; //检测到的ds18b20个数

int main (void){//主程序
	u8 MENU; //主菜单值
	short Temp;//第一个温度传感器
	short Temp2; // 第二个温度传感器
	u8 TemFreFlag; //温度数据刷新标志位
	u8 TemCoun;	//温度数据刷新延时计数
	u8 MENUSET;	//设置菜单值
	u8 SUBMENU;	//子菜单值
	u8 WatTemMod; //水温模式标志位
	u8 MaxWatTemp;//水温上限值
	u8 MinWatTemp;//水温下限值
	u8 R1Flag; //继电器工作标志位
	u8 TimeShift; //时间设置切换选择修改标志位
	u8 WriteTime; //写入时设置后的时间参数标志位
	u16	ADC1buf[10]; //用于存放读10次的ADC值
	u16	MinADC;//用于找出10个ADC值的最小值
	u16	MaxADC;//用于找出10个ADC值的最大值
	u16 EquaADC;//用于把10个ADC值取平均数
	u16 i; //通用变量
	u8 LightMod; //智能灯光模式标志位
	u8 LightShift; //智能灯光菜单下时间区间时间设置切换标志位
	u8 DayTimeStartHour; //智能灯光菜单下白天区间开始小时时间值
	u8 DayTimeStartMin;	//智能灯光菜单下白天区间开始分钟时间值
	u8 DayTimeEndHour;//智能灯光菜单下白天区间结束小时时间值  并且作为傍晚区间开始小时值
	u8 DayTimeEndMin; //智能灯光菜单下白天区间结束分钟时间值  并且作为傍晚区间开始分钟值
	u8 DayTimeGear;	//智能灯光菜单下白天区间灯光档位值
	u16 WriteFlashBuff[WriteFlashBuffNum] = {0}; //写入flash的数据缓存,需要写flash的数据放在数组里,写入flash函数调用一次性写入数据,这样其他同在一个扇区的数据不会丢失		  
	u8 DuskTimeEndHour;//智能灯光菜单下傍晚区间开始小时时间值
	u8 DuskTimeEndMin; //智能灯光菜单下傍晚区间开始分钟时间值
	u8 DuskTimeGear;	//智能灯光菜单下傍晚区间灯光档位值
	u16 LuxSwitch; //智能灯光菜单下打开鱼缸灯的亮度临界值
	u8 Mos1Flag; //智能灯光PWM控制的mos管1状态标志位
	u8 FiltPumpMod; //过滤水泵模式标志位
	u8 MENUShift; //通用的菜单下数值选项设置切换标志位 
	u8 FiltPumpStartHour; //智能灯光菜单下白天区间开始小时时间值
	u8 FiltPumpStartMin;	//智能灯光菜单下白天区间开始分钟时间值
	u8 FiltPumpEndHour;//智能灯光菜单下白天区间结束小时时间值  并且作为傍晚区间开始小时值
	u8 FiltPumpEndMin; //智能灯光菜单下白天区间结束分钟时间值  并且作为傍晚区间开始分钟值
	u8 R2Flag; //控制过滤水泵的继电器状态标志位
	u8 AutoFeedMod; //自动投喂模式值
	u8 FeedTimeNum; //用于选择1--5个投喂时间组
	u8 FeedTimeHour[6] = {0}; //投喂时间的5个存放小时的数组
	u8 FeedTimeMin[6] = {0}; //投喂时间的5个存放分钟的数组
	u8 FeedTimeLast[6] = {0};//投喂时间的5个存放投喂持续时长的数组
	u8 FeedTriggerNum;//投喂时间触发的数组号数
	u8 Steer1Flag;//投喂时间触发的舵机状态标志位
	u8 FeedLastCount; //投喂倒计时
	u8 FeedLastCountFlag; //投喂倒计时1秒变换状态标志位
	u8 ChangeWaterMod; //换水模式标志位
	u8 ChangWatTimNum; //用于切换选择的自动换水时间组 1--5组 
	u8 ChangWatWeek[6]; //存放设置的自动换水的星期值	    
	u8 ChangWatTimHour[6]; //存放1--5自动换水时间组的小时值
	u8 ChangWatTimMin[6];  //存放1--5自动换水时间组的分钟值
	u8 ChangWatFlag;//换水状态标志位
	u8 ChangWatTrigNum;//存放触发的换水时间组
	u8 ChangWatProcess;	//换水流程标志位
	u8 SetInstructFlag; //进入设置时显示按键说明显示标志位

	delay_ms(500); //上电时等待其他器件就绪
	RCC_Configuration(); //系统时钟初始化 
	I2C_Configuration();//I2C初始化
	OLED0561_Init(); //OLED初始化				   
	OLED_DISPLAY_LIT(150);//亮度设置
	DS18B20_Init(); //温度传感器初始化
	RTC_Config(); //实时时钟初始化
	RTC_WritBKP_Config(); //写RTC的备用寄存器BKP配置
	EXti_Config(); //中断初始化
	RELAY_Init();//继电器初始化
	ADCx_Init();//dma读取adc初始化
//	TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,用于智能灯光的灯光控制,设置频率为11KHz，公式为：溢出时间Tout（单位秒）=(arr+1)(psc+1)/Tclk 预分频系数	= 11KHZ (90us)/101(PWM)*72000000
//                          //Tclk为通用定时器的时钟，如果APB1没有分频，则就为系统时钟，72MHZ			                                       64.15	= 0.00009/101(自动装载值)*72000000
//                          //PWM时钟频率=72000000/(100+1)*(63+1) = 11138.6  11KHZ (90us),设置自动装载值101,预分频系数64
//	TIM3_PWM_CH3_Init(59999,23); //Time3通道3初始化,设置频率为50Hz，用于自动投喂的舵机控制
	ChangeWatIoConfig();//自动换水部件相关IO口初始化

	MENU = 1;  //主菜单初始值为1
	TemFreFlag = 1;	//刷新温度标志位初始值为1
	MENUSET = 1; //设置菜单初始化值为1
	SUBMENU = 1; //子菜单值初始值为1
	R1Flag = 0; //继电器1工作状态标志位初始值为0
	TimeShift = 1; //时间设置切换选择修改标志位初始值为1
	WriteTime = 0; //写入设置后的时间参数标志位初始值为0
	LightShift = 1;//智能灯光菜单下时间区间时间设置切换标志位初始值为1
	Mos1Flag = 0;//智能灯光PWM控制的mos管1状态标志位初始值为0
	MENUShift = 1; //通用的菜单下数值选项设置切换标志位
	R2Flag = 0; //控制过滤水泵的继电器状态标志位
	FeedTimeNum = 1; //用于选择1--5个投喂时间组 初始值为1
	FeedTriggerNum = 0;//投喂时间触发的数组号数	初始值为0
	Steer1Flag = 0;//投喂时间触发的舵机状态标志位 初始值为0
	FeedLastCount = 0; //投喂倒计时	初始值为0
	FeedLastCountFlag = 0; //投喂倒计时1秒变换状态标志位 初始值为0
	ChangWatTimNum = 1; //用于切换选择的自动换水时间组 1--5组 初始值为1
	ChangWatFlag = 0;//换水状态标志位  初始值为0
	ChangWatTrigNum = 0;//存放触发的换水时间组 初始值为0
	ChangWatProcess	= 1; //换水流程标志位 初始值为1
	SetInstructFlag = 0; //进入设置时显示按键说明显示标志位	初始值为0

	if(FLASH_R(FLASH_START_ADDR+(0*2)) != 0xfefe){ //读flash标志位,如果不等于标志位0xfefe,则代表数据丢失
	   	WriteFlashBuff[0] = 0xfefe;//写入标志位放进写入缓存数组,等待写入
		DayTimeGear = 85; //白天区间灯光档位值初始值为85
		DuskTimeEndHour = 21; //傍晚结束时间初始值为 21:00
		DuskTimeEndMin = 0;	
		DuskTimeGear = 35; //傍晚区间灯光档位值初始值为35
		LuxSwitch = 1300;////打开灯光亮度临界值初始值为1300
		FiltPumpMod = 0; //过滤水泵模式标志位初始值为0
		FiltPumpStartHour = 8;//过滤水泵的开启时间区间初始值是: 08:00 - 21:00
		FiltPumpStartMin = 0;
		FiltPumpEndHour = 21;
		FiltPumpEndMin = 0;
		AutoFeedMod = 0; //自动投喂模式值初始值为0
		for(i=1; i<6; i++){	//自动投喂5个时间组初始值都为  比如: (off)24时:24分  投喂持续时长10秒 
 		   FeedTimeHour[i] =  24; 
 		   FeedTimeMin[i] =  25; 
 		   FeedTimeLast[i] =  10; 
		}
		ChangeWaterMod = 0; //初始自动换水模式值为0
		for(i=1; i<6; i++){	//自动换水5个时间组初始值  星期都为off标志位7  时间都为 12:25 
			ChangWatWeek[i] = 7; //
			ChangWatTimHour[i] = 12; //
			ChangWatTimMin[i] = 25; 
		}
		WriteFlashBuff[1] = DayTimeGear; //白天区间灯光档位值初始值放进写入缓存数组,等待写入
		WriteFlashBuff[2] = DuskTimeEndHour; //初始傍晚区间结束小时值放进写入缓存数组,等待写入
		WriteFlashBuff[3] = DuskTimeEndMin; //初始傍晚区间结束分钟值放进写入缓存数组,等待写入
		WriteFlashBuff[4] = DuskTimeGear; //初始傍晚区间灯光档位值放进写入缓存数组,等待写入
		WriteFlashBuff[5] = LuxSwitch; //初始亮度临界值放进写入缓存数组,等待写入
		WriteFlashBuff[6] = FiltPumpMod; //初始过滤水泵模式值放进写入缓存数组,等待写入
		WriteFlashBuff[7] = FiltPumpStartHour; //初始过滤水泵开启的区间开始小时值放进写入缓存数组,等待写入
		WriteFlashBuff[8] = FiltPumpStartMin; //初始过滤水泵开启的区间开始分钟值放进写入缓存数组,等待写入
		WriteFlashBuff[9] = FiltPumpEndHour; //初始过滤水泵开启的区间结束小时值放进写入缓存数组,等待写入
		WriteFlashBuff[10] = FiltPumpEndMin; //初始过滤水泵开启的区间结束分钟值放进写入缓存数组,等待写入
		WriteFlashBuff[11] = AutoFeedMod; //初始自动投喂模式值放进写入缓存数组,等待写入
		for(i=1; i<6; i++){	//自动投喂5个时间组初始值等待写入
 		   WriteFlashBuff[11+i] = FeedTimeHour[i]; //自动投喂的5个时间组初始值放进写入缓存数组,等待写入
 		   WriteFlashBuff[16+i] = FeedTimeMin[i];
 		   WriteFlashBuff[21+i] = FeedTimeLast[i]; 
		}
		WriteFlashBuff[27] = ChangeWaterMod; //初始自动换水模式值放进写入缓存数组,等待写入
		for(i=1; i<6; i++){	//自动换水5个时间组初始值等待写入 
			WriteFlashBuff[27+i] = ChangWatWeek[i]; //初始自动换水小时值放进写入缓存数组,等待写入
			WriteFlashBuff[32+i] = ChangWatTimHour[i]; //初始后的自动换水分钟值放进写入缓存数组,等待写入
			WriteFlashBuff[37+i] = ChangWatTimMin[i]; //初始自动换水持续时长值放进写入缓存数组,等待写入
		}
		FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由WriteNum决定
	}else{ // 标志位符合,表示数据没有丢失
		DayTimeGear = FLASH_R(FLASH_START_ADDR+(1*2));//读出flash中数据
		DuskTimeEndHour = FLASH_R(FLASH_START_ADDR+(2*2));//读出flash中数据
		DuskTimeEndMin = FLASH_R(FLASH_START_ADDR+(3*2));//读出flash中数据
		DuskTimeGear = FLASH_R(FLASH_START_ADDR+(4*2));//读出flash中数据
		LuxSwitch = FLASH_R(FLASH_START_ADDR+(5*2)); //读出flash中数据
		FiltPumpMod = FLASH_R(FLASH_START_ADDR+(6*2)); //读出flash中数据;
		FiltPumpStartHour = FLASH_R(FLASH_START_ADDR+(7*2)); //读出flash中数据; 
		FiltPumpStartMin = FLASH_R(FLASH_START_ADDR+(8*2)); //读出flash中数据
		FiltPumpEndHour = FLASH_R(FLASH_START_ADDR+(9*2)); //读出flash中数据;
		FiltPumpEndMin = FLASH_R(FLASH_START_ADDR+(10*2)); //读出flash中数据;
		AutoFeedMod = FLASH_R(FLASH_START_ADDR+(11*2)); //读出flash中数据;
		for(i=1; i<6; i++){	//把自动投喂的5个时间组从flash读出
 		   FeedTimeHour[i] =  FLASH_R(FLASH_START_ADDR+((11+i)*2)); //读出flash中数据;
 		   FeedTimeMin[i] =  FLASH_R(FLASH_START_ADDR+((16+i)*2)); //读出flash中数据;
 		   FeedTimeLast[i] =  FLASH_R(FLASH_START_ADDR+((21+i)*2)); //读出flash中数据;
		}
		ChangeWaterMod = FLASH_R(FLASH_START_ADDR+(27*2)); //读出flash中数据;
		for(i=1; i<6; i++){	//自动换水5个时间组从flash读出 
			ChangWatWeek[i] = FLASH_R(FLASH_START_ADDR+((27+i)*2)); //读出flash中数据;
			ChangWatTimHour[i] = FLASH_R(FLASH_START_ADDR+((32+i)*2)); //读出flash中数据;
			ChangWatTimMin[i] = FLASH_R(FLASH_START_ADDR+((37+i)*2)); //读出flash中数据;
		}
	}
	 	  
	if(BKP_ReadBackupRegister(BKP_DR2) != 0xfefe){//读BKP后备寄存器标志位,如果不等于标志位0xfefe,则代表数据丢失
		BKP_WriteBackupRegister(BKP_DR2, 0xfefe);//向后备寄存器写入标志位	
		WatTemMod = 0;
		MaxWatTemp = 30;
		MinWatTemp = 15;
		LightMod = 0;//初始智能灯光模式值		
		DayTimeStartHour = 8;//白天区间的开始到结束初始时间值为 : 08:00 -- 17: 00 
		DayTimeStartMin = 0;
		DayTimeEndHour = 17;
		DayTimeEndMin = 0;
		BKP_WriteBackupRegister(BKP_DR3, WatTemMod);//向后备寄存器写入初始温度监控模式值	
		BKP_WriteBackupRegister(BKP_DR4, MaxWatTemp);//向后备寄存器写入初始水温上限值	
		BKP_WriteBackupRegister(BKP_DR5, MinWatTemp);//向后备寄存器写入初始水温下限值
		BKP_WriteBackupRegister(BKP_DR6, LightMod);//向后备寄存器写入初始智能灯光模式值
		BKP_WriteBackupRegister(BKP_DR7, DayTimeStartHour);//向后备寄存器写入初始白天区间开始小时值	
		BKP_WriteBackupRegister(BKP_DR8, DayTimeStartMin);//向后备寄存器写入初始白天区间开始分钟值	
		BKP_WriteBackupRegister(BKP_DR9, DayTimeEndHour);//向后备寄存器写入初始白天区间结束小时值	
		BKP_WriteBackupRegister(BKP_DR10, DayTimeEndMin);//向后备寄存器写入白天区间结束分钟值	
	}else{ // 标志位符合,表示数据没有丢失
		WatTemMod = BKP_ReadBackupRegister(BKP_DR3);//读出后备寄存器
		MaxWatTemp = BKP_ReadBackupRegister(BKP_DR4);//读出后备寄存器
		MinWatTemp = BKP_ReadBackupRegister(BKP_DR5);//读出后备寄存器
		LightMod = BKP_ReadBackupRegister(BKP_DR6);//读出后备寄存器
		DayTimeStartHour = BKP_ReadBackupRegister(BKP_DR7);//读出后备寄存器
		DayTimeStartMin = BKP_ReadBackupRegister(BKP_DR8);//读出后备寄存器
		DayTimeEndHour = BKP_ReadBackupRegister(BKP_DR9);//读出后备寄存器
		DayTimeEndMin = BKP_ReadBackupRegister(BKP_DR10);//读出后备寄存器 
	}
	//开机界面
	OLED_DISPLAY_Buff_16x16(0,2*16,&HuanYingShiYon_16[0],4); //显示: 欢迎使用
	OLED_DISPLAY_Buff_16x16(2,1*16,&ZhiNengDengGuang_16[0],2);//显示: 智能鱼缸系统
	OLED_DISPLAY_Buff_16x16(2,3*16,&YuGangXiTon_16[0],4); 
	OLED_DISPLAY_Buff_16x16(4,1*16,&KaiFaRen_16[0],3);//显示: 开发人:Damon
	OLED_DISPLAY_8x16(4,8*8,':');// 
	OLED_DISPLAY_8x16(4,9*8,'D');// 
	OLED_DISPLAY_8x16(4,10*8,'a');// 
	OLED_DISPLAY_8x16(4,11*8,'m');// 
	OLED_DISPLAY_8x16(4,12*8,'o');// 
	OLED_DISPLAY_8x16(4,13*8,'n');// 
	delay_ms(1000);
	OLED_DISPLAY_CLEAR();//清屏操作
	OLED_DISPLAY_16x16(0,1*16,&an4_16[0]); //按A键切换
	OLED_DISPLAY_8x16(0,4*8,'A');// 
	OLED_DISPLAY_16x16(0,3*16,&jian4_16[0]); //
	OLED_DISPLAY_Buff_16x16(0,4*16,&QieHuan_16[0],2); 
	OLED_DISPLAY_16x16(6,1*16,&an4_16[0]); //按D键设置
	OLED_DISPLAY_8x16(6,4*8,'D');// 
	OLED_DISPLAY_16x16(6,3*16,&jian4_16[0]); //
	OLED_DISPLAY_16x16(6,4*16,&she_16[0]); //
	OLED_DISPLAY_16x16(6,5*16,&zhi4_16[0]); //
 	delay_ms(2000);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...刷新数据
	while(1){ //主循环
		if(MENU < 70 || MENU == 160){ //当不在设置子菜单时 或在换水条件触发菜单
 			Temp = DS18B20_Get_Temp();//读第二个温度传感器
			Temp2 = DS18B20_Get_Temp_2();//读第二个温度传感器
			if(TemFreFlag == 1){  //判断刷新温度标志位是否为1				
				Temp = DS18B20_Get_Temp();//读第一个温度传感器
			}else{ //刷新温度标志位为0进入延时
				TemCoun++;	//刷新温度延时计数值+1
				if(TemCoun >= 30){ //延时一段时间
					TemFreFlag = 1;	//刷新温度标志位置1
					TemCoun = 0;
				}
			}
		}
		if(WriteTime == 1){	//如果写入时间标志位为1
			WriteTime = 0;//写入时间标志位清0
			RTC_Set(ryear,rmon,rday,rhour,rmin,rsec); //写入当前时间（1970~2099年有效），		
		}
		if(MENU < 70 || MENU == 129){ //当不在设置子菜单或者在设置打开鱼缸灯光亮度临界值菜单时刷新光感ADC值			
			for(i=0; i<10; i++){
				ADC1buf[i] = ADC_ConvertedValue;	//每隔2ms读一次ADC,读10次
				delay_ms(2);//延时
			}
			MinADC = ADC1buf[0];
			for(i=0; i<10; i++){ //找出10个当中的最小值
				if(MinADC > ADC1buf[i]){			
					MinADC = ADC1buf[i];
				} 
			}
			MaxADC = ADC1buf[0];
			for(i=0; i<10; i++){ //找出10个当中的最大值
				if(MaxADC < ADC1buf[i]){			
					MaxADC = ADC1buf[i];
				} 
			}
			for(i=0; i<10; i++){ //10个数相加
					EquaADC += ADC1buf[i];
			}
			EquaADC	= EquaADC - (MaxADC + MinADC);//减去10个数当中的最大值和最小值,减少干扰
			EquaADC = (EquaADC /8) + (EquaADC %8); //剩下8个数据除以8得出平均值
			EquaADC	= 4096 - EquaADC; //把光感ADC的值倒过来,从之前的光线越强数值越小到现在的光线越强数值越大对比,后者更容易理解
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,刷新数据
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...主界面
		if(MENU == 1){	//静态显示字样
			OLED_DISPLAY_CLEAR();//oled清屏操作
			OLED_DISPLAY_16x16(0,0*16,&shui_16[0]); //显示:水温:   . ℃ 
			OLED_DISPLAY_16x16(0,1*16,&wen_16[0]);
			OLED_DISPLAY_8x16(0,4*8,':');
			OLED_DISPLAY_8x16(0,8*8,'.');//
			OLED_DISPLAY_16x16(0,10*8,&C_16[0]); //显示温度符号℃//
			OLED_DISPLAY_16x16(2,0*16,&ri_16[0]);//显示:日期:  -  -
			OLED_DISPLAY_16x16(2,1*16,&qi_16[0]);
 			OLED_DISPLAY_8x16(2,4*8,':');//
			OLED_DISPLAY_8x16(2,9*8,'-');//				
			OLED_DISPLAY_8x16(2,12*8,'-');//				
			OLED_DISPLAY_16x16(4,0*16,&shij_16[0]);//显示:时间   :  : 
			OLED_DISPLAY_16x16(4,1*16,&jian_16[0]);
			OLED_DISPLAY_8x16(4,4*8,':');//
			OLED_DISPLAY_8x16(4,8*8,':');//				
			OLED_DISPLAY_8x16(4,11*8,':');//
			OLED_DISPLAY_16x16(6,3*16,&xing_16[0]);//显示:   星期
			OLED_DISPLAY_16x16(6,4*16,&qi1_16[0]);
			MENU = 31; //切换到动态刷新数据菜单
		}
		if(MENU == 31){	//动态刷新显示数据
			RTC_Get(); //获取当前时间
			OLED_DISPLAY_8x16(2,5*8,ryear/1000+0x30);//刷新显示年
			OLED_DISPLAY_8x16(2,6*8,ryear%1000/100+0x30);//
			OLED_DISPLAY_8x16(2,7*8,ryear%100/10+0x30);//
			OLED_DISPLAY_8x16(2,8*8,ryear%10+0x30);//
			OLED_DISPLAY_8x16(2,10*8,rmon/10+0x30);//刷新显示月
			OLED_DISPLAY_8x16(2,11*8,rmon%10+0x30);//
			OLED_DISPLAY_8x16(2,13*8,rday/10+0x30);//刷新显示日
			OLED_DISPLAY_8x16(2,14*8,rday%10+0x30);//
			OLED_DISPLAY_8x16(4,6*8,rhour/10+0x30);//刷新显示小时
			OLED_DISPLAY_8x16(4,7*8,rhour%10+0x30);//
			OLED_DISPLAY_8x16(4,9*8,rmin/10+0x30);//刷新显示分钟
			OLED_DISPLAY_8x16(4,10*8,rmin%10+0x30);//
			OLED_DISPLAY_8x16(4,12*8,rsec/10+0x30);//刷新显示秒
			OLED_DISPLAY_8x16(4,13*8,rsec%10+0x30);//
			OLED_DISPLAY_16x16(6,5*16,&xingqi1_7_16[rweek*32]);	//显示星期一~~日

			//显示温度传感器1
			if(Temp > 0){//判断为正温度
				OLED_DISPLAY_8x16(0,5*8,' ');//显示温度值
				OLED_DISPLAY_8x16(0,6*8,Temp%1000/100+0x30);//显示温度值,温度范围-55~~99
				OLED_DISPLAY_8x16(0,7*8,Temp%100/10+0x30);//
				OLED_DISPLAY_8x16(0,9*8,Temp%10+0x30);//
			}
//			else{
//				Temp *= -1;
//				OLED_DISPLAY_8x16(0,5*8,'-');//显示温度值
//				OLED_DISPLAY_8x16(0,6*8,Temp%1000/100+0x30);//显示温度值,温度范围-55~~99
//				OLED_DISPLAY_8x16(0,7*8,Temp%100/10+0x30);//
//				OLED_DISPLAY_8x16(0,9*8,Temp%10+0x30);//
//			}

			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 2; //跳转到副界面
				}
				if(KEY == 4){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 61; //跳转到设置界面
					SetInstructFlag = 1; //显示设置说明标志位置1
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
			}
		}
		if(MENU == 2){
			OLED_DISPLAY_CLEAR();//oled清屏操作
			OLED_DISPLAY_16x16(0,0*16,&guang_16[0]);//显示:光照值:
			OLED_DISPLAY_16x16(0,1*16,&zhao_16[0]);
			OLED_DISPLAY_16x16(0,2*16,&zhi_16[0]);
			OLED_DISPLAY_8x16(0,6*8,':');//	
			OLED_DISPLAY_16x16(2,0*16,&xin1_16[0]);//显示:新水水温:  . ℃
			OLED_DISPLAY_16x16(2,1*16,&shui_16[0]);
 			OLED_DISPLAY_16x16(2,2*16,&shui_16[0]);
			OLED_DISPLAY_16x16(2,3*16,&wen_16[0]);
			OLED_DISPLAY_8x16(2,8*8,':');//	
			OLED_DISPLAY_8x16(2,12*8,'.');//
			OLED_DISPLAY_16x16(2,14*8,&C_16[0]); //显示温度符号℃//
			MENU = 32;//切换到动态刷新数据菜单	
		}
		if(MENU == 32){	 //动态刷新数据菜单			
			OLED_DISPLAY_8x16(0,9*8,EquaADC/1000+0x30);//显示ADC的值,这里是光感ADC
			OLED_DISPLAY_8x16(0,10*8,EquaADC%1000/100+0x30);//
			OLED_DISPLAY_8x16(0,11*8,EquaADC%100/10+0x30);//
			OLED_DISPLAY_8x16(0,12*8,EquaADC%10+0x30);//
	
			Temp2 = DS18B20_Get_Temp_2();//读第二个温度传感器
			if(Temp2 > 0){//判断为正温度
				OLED_DISPLAY_8x16(2,9*8,' ');//显示温度值
				OLED_DISPLAY_8x16(2,10*8,Temp2%1000/100+0x30);//显示温度值,温度范围-55~~99
				OLED_DISPLAY_8x16(2,11*8,Temp2%100/10+0x30);//
				OLED_DISPLAY_8x16(2,13*8,Temp2%10+0x30);//
			}
//			else{
//				Temp2 *= -1;
//				OLED_DISPLAY_8x16(2,9*8,'-');//显示温度值
//				OLED_DISPLAY_8x16(2,10*8,Temp2%1000/100+0x30);//显示温度值,温度范围-55~~99
//				OLED_DISPLAY_8x16(2,11*8,Temp2%100/10+0x30);//
//				OLED_DISPLAY_8x16(2,13*8,Temp2%10+0x30);//
//			}

			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
				if(KEY == 4){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 61; //跳转到设置界面
					SetInstructFlag = 1; //显示设置说明标志位置1
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
			}
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...主界面
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...设置界面
		if(MENU == 61){
			if(SetInstructFlag == 1){
				SetInstructFlag = 0;
				OLED_DISPLAY_16x16(0,0*16,&an4_16[0]); //按A键进入选项
				OLED_DISPLAY_8x16(0,2*8,'A');// 
				OLED_DISPLAY_16x16(0,2*16,&jian4_16[0]); //
				OLED_DISPLAY_16x16(0,3*16,&jin4_16[0]); //
				OLED_DISPLAY_16x16(0,4*16,&ru4_16[0]); //
				OLED_DISPLAY_Buff_16x16(0,5*16,&XuanXiang_16[0],2); 

				OLED_DISPLAY_16x16(2,0*16,&an4_16[0]); //按B键选项减1
				OLED_DISPLAY_8x16(2,2*8,'B');// 
				OLED_DISPLAY_16x16(2,2*16,&jian4_16[0]); //
				OLED_DISPLAY_Buff_16x16(2,3*16,&XuanXiang_16[0],2); 
				OLED_DISPLAY_16x16(2,5*16,&jian3_16[0]); //
				OLED_DISPLAY_8x16(2,12*8,'1');// 

				OLED_DISPLAY_16x16(4,0*16,&an4_16[0]); //按C键选项加1
				OLED_DISPLAY_8x16(4,2*8,'C');// 
				OLED_DISPLAY_16x16(4,2*16,&jian4_16[0]); //
				OLED_DISPLAY_Buff_16x16(4,3*16,&XuanXiang_16[0],2); 
				OLED_DISPLAY_16x16(4,5*16,&jia1_16[0]); //
				OLED_DISPLAY_8x16(4,12*8,'1');// 

				OLED_DISPLAY_16x16(6,0*16,&an4_16[0]); //按D键退出设置
				OLED_DISPLAY_8x16(6,2*8,'D');// 
				OLED_DISPLAY_16x16(6,2*16,&jian4_16[0]); //
				OLED_DISPLAY_16x16(6,3*16,&tui4_16[0]); //
				OLED_DISPLAY_16x16(6,4*16,&chu1_16[0]); //
				OLED_DISPLAY_16x16(6,5*16,&she_16[0]); //
				OLED_DISPLAY_16x16(6,6*16,&zhi4_16[0]); //
			 	delay_ms(1500);
				OLED_DISPLAY_CLEAR();//oled清屏操作
			}

			OLED_DISPLAY_16x16(0,3*16,&she_16[0]);//显示:   设置
			OLED_DISPLAY_16x16(0,4*16,&zhi4_16[0]);
			if(MENUSET < 4){ //选项值小于4
				if(MENUSET == 1){ //选中一个选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.水温监测				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&shui_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&wen_16[0]);
					ANTI_OLED_DISPLAY_16x16(2,3*16,&jian1_16[0]);
					ANTI_OLED_DISPLAY_16x16(2,4*16,&ce_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.水温监测
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&shui_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&wen_16[0]);				
					OLED_DISPLAY_16x16(2,3*16,&jian1_16[0]);
					OLED_DISPLAY_16x16(2,4*16,&ce_16[0]);
				}	
				if(MENUSET == 2){ //选中二个选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.时间设置				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,3*16,&she_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&zhi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.时间设置
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&jian_16[0]);				
					OLED_DISPLAY_16x16(4,3*16,&she_16[0]);
					OLED_DISPLAY_16x16(4,4*16,&zhi4_16[0]);
				}	
				if(MENUSET == 3){ //选中三个选项
					ANTI_OLED_DISPLAY_8x16(6,0*8,'3');//反白背景显示:3.智能灯光				
					ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(6,1*16,&ZhiNengDengGuang_16[0],4);
				}else{
					OLED_DISPLAY_8x16(6,0*8,'3');//正常显示:3.智能灯光
					OLED_DISPLAY_8x16(6,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(6,1*16,&ZhiNengDengGuang_16[0],4);
				}	
			}
			if(MENUSET > 3 && MENUSET < 7){ //选项值大于3 并且 选项值小于7
				if(MENUSET == 4){ //选中一个选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'4');//反白背景显示:4.过滤水泵			GuoLvShuiBeng_16[]	
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(2,1*16,&GuoLvShuiBeng_16[0],4);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'4');//正常显示:4.过滤水泵
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(2,1*16,&GuoLvShuiBeng_16[0],4);
				}	
				if(MENUSET == 5){ //选中二个选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'5');//反白背景显示:5.自动投喂		ZiDongTouWei_16[]		
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'5');//正常显示:5.自动投喂
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);
				}	
				if(MENUSET == 6){ //选中三个选项
					ANTI_OLED_DISPLAY_8x16(6,0*8,'6');//反白背景显示:6.自动换水	ZiDongHuanShui_16[]			
					ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(6,1*16,&ZiDongHuanShui_16[0],4);
				}else{
					OLED_DISPLAY_8x16(6,0*8,'6');//正常显示:6.自动换水
					OLED_DISPLAY_8x16(6,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(6,1*16,&ZiDongHuanShui_16[0],4);
				}	
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //进入到选项设置
					SUBMENU = 1; //子选项值置1
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(MENUSET <= 1) MENUSET = 7; //选项最小值
					MENUSET--; //设置选项标志值减1
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					MENUSET++; //设置选项标志值加1
					if(MENUSET > 6) MENUSET = 1; //选项最大值
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}
		}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....监测水温
		if(MENU == 71){	 //水温监测设置
			OLED_DISPLAY_16x16(0,1*16,&shui_16[0]); //显示: 水温监测设置 
			OLED_DISPLAY_16x16(0,2*16,&wen_16[0]);				
			OLED_DISPLAY_16x16(0,3*16,&jian1_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&ce_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			if(WatTemMod == 1){ //当模式值为1,即开启水温监测状态,显示调整上下限值选项
				if(SUBMENU < 4){ //子选项值小于4
					if(SUBMENU == 1){ //选中一个子选项
						ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
						ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
						ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
						ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
					}else{
						OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.模式
						OLED_DISPLAY_8x16(2,1*8,'.');//
						OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
						OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
					}	
					if(SUBMENU == 2){ //选中二个子选项
						ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.水温上限				
						ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
						ANTI_OLED_DISPLAY_16x16(4,1*16,&shui_16[0]);  
						ANTI_OLED_DISPLAY_16x16(4,2*16,&wen_16[0]);				
						ANTI_OLED_DISPLAY_16x16(4,3*16,&shang4_16[0]);
						ANTI_OLED_DISPLAY_16x16(4,4*16,&xian4_16[0]);
					}else{
						OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.水温上限
						OLED_DISPLAY_8x16(4,1*8,'.');//
						OLED_DISPLAY_16x16(4,1*16,&shui_16[0]);  
						OLED_DISPLAY_16x16(4,2*16,&wen_16[0]);				
						OLED_DISPLAY_16x16(4,3*16,&shang4_16[0]);
						OLED_DISPLAY_16x16(4,4*16,&xian4_16[0]);
					}	
					if(SUBMENU == 3){ //选中三个子选项
						ANTI_OLED_DISPLAY_8x16(6,0*8,'3');//反白背景显示:3.水温下限				
						ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
						ANTI_OLED_DISPLAY_16x16(6,1*16,&shui_16[0]);  
						ANTI_OLED_DISPLAY_16x16(6,2*16,&wen_16[0]);				
						ANTI_OLED_DISPLAY_16x16(6,3*16,&xia4_16[0]);
						ANTI_OLED_DISPLAY_16x16(6,4*16,&xian4_16[0]);
					}else{
						OLED_DISPLAY_8x16(6,0*8,'3');//正常显示:3.水温下限
						OLED_DISPLAY_8x16(6,1*8,'.');//
						OLED_DISPLAY_16x16(6,1*16,&shui_16[0]);  
						OLED_DISPLAY_16x16(6,2*16,&wen_16[0]);				
						OLED_DISPLAY_16x16(6,3*16,&xia4_16[0]);
						OLED_DISPLAY_16x16(6,4*16,&xian4_16[0]);
					}	
				}
			}else{//模式值为0,表示关闭温度监测状态,只显示1.模式设置
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);			
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(WatTemMod == 1){ //当水温监测模式值为1,即开启水温监测状态,可以进入设置
						MENU = 100 + SUBMENU; //进入到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
					}else{ //当水温监测模式值不为1,即关闭水温监测状态,只能进入模式设置
						MENU = 101; //只能进入到模式子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作					
					}
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(SUBMENU  <= 1) SUBMENU = 4; //子选项最小值
					SUBMENU --; //子选项标志值减1
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					SUBMENU++; //子选项标志值加1
					if(SUBMENU > 3) SUBMENU = 1; //子选项最大值
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}				
		}
		if(MENU == 101){//水温检测设置子选项 //模式设置
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //显示:  模式设置 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(WatTemMod == 1){	//水温模式标志位为1,代表开启监测水温
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//显示:开启
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //水温模式标志位不为1,代表关闭监测水温
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//显示: 关闭
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //进入到子选项设置
					BKP_WriteBackupRegister(BKP_DR3, WatTemMod);//设置完成后,向后备寄存器中写入水温监测模式值
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(WatTemMod  == 0) WatTemMod = 2; //子选项最小值
					WatTemMod --; //子选项标志值减1
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					WatTemMod++; //子选项标志值加1
					if(WatTemMod > 1) WatTemMod = 0; //子选项最大值
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					BKP_WriteBackupRegister(BKP_DR3, WatTemMod);//设置完成后,向后备寄存器中写入水温监测模式值
					MENU = 1; //跳转到主界面
				}
			}								
		}
		if(MENU == 102){//水温检测设置子选项  //温度上限	  			
			OLED_DISPLAY_16x16(0,1*16,&shui_16[0]);  //显示: 温度上限设置 
			OLED_DISPLAY_16x16(0,2*16,&wen_16[0]);				
			OLED_DISPLAY_16x16(0,3*16,&shang4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&xian4_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			ANTI_OLED_DISPLAY_8x16(4,7*8,MaxWatTemp/10+0x30);//反白背景显示:水温上限数据的值 如: 22℃				
			ANTI_OLED_DISPLAY_8x16(4,8*8,MaxWatTemp%10+0x30);//
			ANTI_OLED_DISPLAY_16x16(4,9*8,&C_16[0]); //显示温度符号℃//
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //退回到子选项设置
					BKP_WriteBackupRegister(BKP_DR4, MaxWatTemp);//设置完成后,向后备寄存器中写入水温上限值
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}

				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						if(MaxWatTemp  == 0) MaxWatTemp = 36; //水温上限最小值
						MaxWatTemp--; //水温上限值减1
						ANTI_OLED_DISPLAY_8x16(4,7*8,MaxWatTemp/10+0x30);//刷新显示反白背景显示:水温上限数据的值 如: 22℃				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MaxWatTemp%10+0x30);//
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					if(MaxWatTemp  == 0) MaxWatTemp = 36; //水温上限最小值
					MaxWatTemp--; //水温上限值减1								
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						MaxWatTemp++; //水温上限值加1
						if(MaxWatTemp > 35) MaxWatTemp = 0; //水温上限最大值
						ANTI_OLED_DISPLAY_8x16(4,7*8,MaxWatTemp/10+0x30);//刷新显示反白背景显示:水温上限数据的值 如: 22℃				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MaxWatTemp%10+0x30);//
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					MaxWatTemp++; //水温上限值加1
					if(MaxWatTemp > 35) MaxWatTemp = 0; //水温上限最大值
				} //短按结束	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					BKP_WriteBackupRegister(BKP_DR4, MaxWatTemp);//设置完成后,向后备寄存器中写入水温上限值
					MENU = 1; //跳转到主界面
				}
			}								
		}
		if(MENU == 103){//水温检测设置子选项  //温度下限	  			
			OLED_DISPLAY_16x16(0,1*16,&shui_16[0]);  //显示: 温度下限设置 
			OLED_DISPLAY_16x16(0,2*16,&wen_16[0]);				
			OLED_DISPLAY_16x16(0,3*16,&xia4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&xian4_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			ANTI_OLED_DISPLAY_8x16(4,7*8,MinWatTemp/10+0x30);//反白背景显示:水温下限数据的值 如: 22℃				
			ANTI_OLED_DISPLAY_8x16(4,8*8,MinWatTemp%10+0x30);//
			ANTI_OLED_DISPLAY_16x16(4,9*8,&C_16[0]); //显示温度符号℃//
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //退回到子选项设置
					OLED_DISPLAY_CLEAR();//oled清屏操作
					BKP_WriteBackupRegister(BKP_DR5, MinWatTemp);//设置完成后,向后备寄存器中写入水温下限值
				}
				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						if(MinWatTemp  == 0) MinWatTemp = 36; //水温下限最小值
						MinWatTemp--; //水温下限值减
						ANTI_OLED_DISPLAY_8x16(4,7*8,MinWatTemp/10+0x30);//刷新显示反白背景显示:水温下限数据的值 如: 22℃				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MinWatTemp%10+0x30);//
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					if(MinWatTemp  == 0) MinWatTemp = 36; //水温下限最小值
					MinWatTemp--; //水温下限值减1									
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						MinWatTemp++; //水温下限值加1
						if(MinWatTemp > 35) MinWatTemp = 0; //水温下限最大值
						ANTI_OLED_DISPLAY_8x16(4,7*8,MinWatTemp/10+0x30);//刷新显示反白背景显示:水温下限数据的值 如: 22℃				
						ANTI_OLED_DISPLAY_8x16(4,8*8,MinWatTemp%10+0x30);//
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					MinWatTemp++; //水温下限值加1
					if(MinWatTemp > 35) MinWatTemp = 0; //水温下限最大值
				} //短按结束
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					BKP_WriteBackupRegister(BKP_DR5, MinWatTemp);//设置完成后,向后备寄存器中写入水温下限值
				}
			}								
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...监测水温
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...时间设置
		if(MENU == 72){	 //时间设置
			OLED_DISPLAY_16x16(0,2*16,&shij_16[0]); //显示:  时间设置 
			OLED_DISPLAY_16x16(0,3*16,&jian_16[0]);				
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(SUBMENU < 4){ //子选项值小于4
				if(SUBMENU == 1){ //选中一个子选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.日期			
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&ri_16[0]);//
					ANTI_OLED_DISPLAY_16x16(2,2*16,&qi_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.日期
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&ri_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&qi_16[0]);
				}	
				if(SUBMENU == 2){ //选中二个子选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.时间				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&fen1_16[0]);				
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.时间
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&shij_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&fen1_16[0]);				
				}	
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 110 + SUBMENU; //进入到时间设置子选项设置菜单
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(SUBMENU  <= 1) SUBMENU = 3; //子选项最小值
					SUBMENU --; //子选项标志值减1
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					SUBMENU++; //子选项标志值加1
					if(SUBMENU > 2) SUBMENU = 1; //子选项最大值
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}				
		}
		if(MENU == 111){//时间设置子选项  //日期设置	  			
			OLED_DISPLAY_16x16(0,2*16,&ri_16[0]); //显示: 日期设置 
			OLED_DISPLAY_16x16(0,3*16,&qi_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);

			if(TimeShift == 1){	//反白背景显示年,表示提示用户当前选择是年
				ANTI_OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//显示:年月日  如:2021-08-04
				ANTI_OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
				ANTI_OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
				ANTI_OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
				OLED_DISPLAY_8x16(4,7*8,'-');//				
				OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//月
				OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
				OLED_DISPLAY_8x16(4,10*8,'-');//				
				OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//日
				OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
			}
			if(TimeShift == 2){	 //反白背景显示月,表示提示用户当前选择是月
				OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//显示:年月日  如:2021-08-04
				OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
				OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
				OLED_DISPLAY_8x16(4,7*8,'-');//				
				ANTI_OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//月
				ANTI_OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
				OLED_DISPLAY_8x16(4,10*8,'-');//				
				OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//日
				OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
			}
			if(TimeShift == 3){//反白背景显示日,表示提示用户当前选择是日
				OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//显示:年月日  如:2021-08-04
				OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
				OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
				OLED_DISPLAY_8x16(4,7*8,'-');//				
				OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//月
				OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
				OLED_DISPLAY_8x16(4,10*8,'-');//				
				ANTI_OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//日
				ANTI_OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(TimeShift < 3){
						TimeShift++;//时间参数切换选择标志加1	
					}else{
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						TimeShift = 1; //时间参数切换选择标志位置1
						WriteTime = 1; //写入时间标志位置1,写入时间函数执行写入设置后的时间值
					}
				}
				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (TimeShift){
							case 1: if(ryear <= 1970) ryear = 2100;	//年最小值
									ryear--;			 
									ANTI_OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//刷新显示:年
									ANTI_OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
									break;
							case 2: if(rmon <= 1) rmon = 13; //月最小值
									rmon--;
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//刷新显示月
									ANTI_OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
									break;
							case 3: if(rday <= 1) rday = 32; //日值最小值
									rday--;
									ANTI_OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//刷新显示日
									ANTI_OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
									break;
							default:TimeShift = 1;
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					switch (TimeShift){
						case 1: if(ryear <= 1970) ryear = 2100;	//年最小值
								ryear--;
								break;
						case 2: if(rmon <= 1) rmon = 13;  //月最小值
								rmon--;
								break;
						case 3: if(rday <= 1) rday = 32; //日值最小值
								rday--;
								break;
						default:TimeShift = 1;	//冗余
								break;											
					}
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (TimeShift){
							case 1: ryear++;
									if(ryear > 2099) ryear = 1970;	//年最大值										
									ANTI_OLED_DISPLAY_8x16(4,3*8,ryear/1000+0x30);//刷新显示:年
									ANTI_OLED_DISPLAY_8x16(4,4*8,ryear%1000/100+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,5*8,ryear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(4,6*8,ryear%10+0x30);//
									break;
							case 2: rmon++;
									if(rmon > 12) rmon = 1;		//月最大值										
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmon/10+0x30);//刷新显示月
									ANTI_OLED_DISPLAY_8x16(4,9*8,rmon%10+0x30);//
									break;
							case 3: rday++;
									if(rday > 31) rday = 1;	 	//日值最大值
									ANTI_OLED_DISPLAY_8x16(4,11*8,rday/10+0x30);//刷新显示日
									ANTI_OLED_DISPLAY_8x16(4,12*8,rday%10+0x30);//
									break;
							default:TimeShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					switch (TimeShift){
						case 1: ryear++;
								if(ryear > 2099) ryear = 1970;//年最大值											
								break;
						case 2: rmon++;
								if(rmon > 12) rmon = 1;	//月最大值										
								break;
						case 3: rday++;
								if(rday > 31) rday = 1;	//日值最大值
								break;
						default:TimeShift = 1;	//冗余
								break;											
					}
				} //短按结束
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					WriteTime = 1; //写入时间标志位置1,写入时间函数执行写入设置后的时间值
				}
			}								
		}
		if(MENU == 112){//时间设置子选项  //时分设置	  			
			OLED_DISPLAY_16x16(0,2*16,&shij_16[0]); //显示: 时分设置 
			OLED_DISPLAY_16x16(0,3*16,&fen1_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(TimeShift == 1){	//反白背景显示小时,表示提示用户当前选择是小时
				ANTI_OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//小时
				ANTI_OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,':');//				
				OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//分钟
				OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
			}
			if(TimeShift == 2){	 //反白背景显示分钟,表示提示用户当前选择是分钟
				OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//小时
				OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
				OLED_DISPLAY_8x16(4,6*8,':');//				
				ANTI_OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//分钟
				ANTI_OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
			}
			OLED_DISPLAY_8x16(4,9*8,':');//				
			OLED_DISPLAY_8x16(4,10*8,rsec/10+0x30);//秒
			OLED_DISPLAY_8x16(4,11*8,rsec%10+0x30);//
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(TimeShift < 2){
						TimeShift++;//时间参数选择标志加1	
					}else{
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						TimeShift = 1; //时间参数选择标志位置1
						WriteTime = 1; //写入时间标志位置1,写入时间函数执行写入设置后的时间值
					}
				}
				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (TimeShift){
							case 1: if(rhour <= 0) rhour = 24; //小时最小值
									rhour--;
									ANTI_OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//小时
									ANTI_OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
									break;
							case 2: if(rmin <= 0) rmin = 60;  //分钟最小值
									rmin--;
									rsec = 0;	  //秒清0
									ANTI_OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//分钟
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
									OLED_DISPLAY_8x16(4,10*8,rsec/10+0x30);//秒
									OLED_DISPLAY_8x16(4,11*8,rsec%10+0x30);//
									break;
							default:TimeShift = 1;
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					switch (TimeShift){
						case 1: if(rhour <= 0) rhour = 24;//小时最小值
								rhour--;
								break;
						case 2: if(rmin <= 0) rmin = 60; //分钟最小值
								rmin--;
								rsec = 0;	//秒清0
								break;
						default:TimeShift = 1; //冗余
								break;											
					}
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (TimeShift){
							case 1: rhour++;
									if(rhour > 23) rhour = 0;//小时最大值											
									ANTI_OLED_DISPLAY_8x16(4,4*8,rhour/10+0x30);//小时
									ANTI_OLED_DISPLAY_8x16(4,5*8,rhour%10+0x30);//
									break;
							case 2: rmin++;
									if(rmin > 59) rmin = 0;	//分钟最大值										
									rsec = 0;	 //秒清0
									ANTI_OLED_DISPLAY_8x16(4,7*8,rmin/10+0x30);//刷新显示分钟
									ANTI_OLED_DISPLAY_8x16(4,8*8,rmin%10+0x30);//
									OLED_DISPLAY_8x16(4,10*8,rsec/10+0x30);//秒
									OLED_DISPLAY_8x16(4,11*8,rsec%10+0x30);//
									break;
							default:TimeShift = 1;
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					switch (TimeShift){
						case 1: rhour++;
								if(rhour > 23) rhour = 0;	//小时最大值											
								break;
						case 2: rmin++;
								if(rmin > 59) rmin = 0;	//分钟最大值										
								rsec = 0;			   //秒清0
								break;
						default:TimeShift = 1;  //冗余
								break;											
					}
				} //短按结束
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					WriteTime = 1; //写入时间标志位置1,写入时间函数执行写入设置后的时间值
				}
			}								
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...时间设置
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...智能灯光设置
		if(MENU == 73){	 //智能灯光设置			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZhiNengDengGuang_16[0],4);//显示: 智能灯光设置
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			
			if(LightMod == 0){ //模式值为0,表示关闭智能灯光状态,只显示1.模式
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(LightMod == 1){ //当模式值为1,即开启按照时间段开启智能灯光状态,显示调整时间段选项
				if(SUBMENU == 1){ //选中一个子选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.模式
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //选中二个子选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.白天区间和档位				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&bai1_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&tian1_16[0]);				
					ANTI_OLED_DISPLAY_16x16(4,3*16,&qu1_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,5*16,&he2_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&dang3_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,7*16,&wei4_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.白天区间和档位
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&bai1_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&tian1_16[0]);				
					OLED_DISPLAY_16x16(4,3*16,&qu1_16[0]);
					OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);
					OLED_DISPLAY_16x16(4,5*16,&he2_16[0]);
					OLED_DISPLAY_16x16(4,6*16,&dang3_16[0]);
					OLED_DISPLAY_16x16(4,7*16,&wei4_16[0]);

				}	
				if(SUBMENU == 3){ //选中三个子选项
					ANTI_OLED_DISPLAY_8x16(6,0*8,'3');//反白背景显示:3.傍晚区间和档位				
					ANTI_OLED_DISPLAY_8x16(6,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(6,1*16,&bang4_16[0]);  
					ANTI_OLED_DISPLAY_16x16(6,2*16,&wan3_16[0]);				
					ANTI_OLED_DISPLAY_16x16(6,3*16,&qu1_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,4*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,5*16,&he2_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,6*16,&dang3_16[0]);
					ANTI_OLED_DISPLAY_16x16(6,7*16,&wei4_16[0]);
				}else{
					OLED_DISPLAY_8x16(6,0*8,'3');//正常显示:3.傍晚区间和档位
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
			if(LightMod == 2){ //当模式值为2,即开启按照光线亮度开启智能灯光状态,显示调整开灯亮度值选项
				if(SUBMENU == 1){ //选中一个子选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.模式
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //选中二个子选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.开灯亮度值				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&kai1_16[0]);  
					ANTI_OLED_DISPLAY_16x16(4,2*16,&deng1_16[0]);				
					ANTI_OLED_DISPLAY_16x16(4,3*16,&liang4_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&du4_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,5*16,&zhi_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.开灯亮度值
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_16x16(4,1*16,&kai1_16[0]);  
					OLED_DISPLAY_16x16(4,2*16,&deng1_16[0]);				
					OLED_DISPLAY_16x16(4,3*16,&liang4_16[0]);
					OLED_DISPLAY_16x16(4,4*16,&du4_16[0]);
					OLED_DISPLAY_16x16(4,5*16,&zhi_16[0]);
				}	
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(LightMod == 0){ //当水温监测模式值为0,即关闭水温监测状态,只能进入模式设置
						MENU = 121; //只能进入到模式子选项设置
					}
					if(LightMod == 1){ //当水温监测模式值为1,即开启水温监测状态,可以进入设置
						MENU = 120 + SUBMENU; //进入到子选项设置
					}
					if(LightMod == 2){ //当水温监测模式值为2,即按照光线亮度打开灯光状态,只能进入光线亮度设置
						if(SUBMENU == 1) MENU = 121; //到模式设置菜单
						if(SUBMENU == 2) MENU = 129; //129菜单,专门用于打开灯光的光线亮度设置
					}
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(LightMod == 1){
						if(SUBMENU  == 1) SUBMENU = 4; //子选项最小值
						SUBMENU --; //子选项标志值减1					
					}
					if(LightMod == 2){
						if(SUBMENU  == 1) SUBMENU = 3; //子选项最小值
						SUBMENU --; //子选项标志值减1					
					}
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					if(LightMod == 1){
						SUBMENU++; //子选项标志值加1
						if(SUBMENU > 3) SUBMENU = 1; //子选项最大值
					}
					if(LightMod == 2){
						SUBMENU++; //子选项标志值加1
						if(SUBMENU > 2) SUBMENU = 1; //子选项最大值
					}
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}								
		}
		if(MENU == 121){//智能灯光设置子选项 //模式设置
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //显示:  模式设置 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(LightMod == 0){ //智能灯光模式标志位为0,代表关闭智能灯光
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//显示: 关闭
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(LightMod == 1){	//智能灯光模式标志位为1,代表按照时间段开启智能灯光	
				ANTI_OLED_DISPLAY_16x16(4,1*16,&an4_16[0]); //显示:按照时间段
				ANTI_OLED_DISPLAY_16x16(4,2*16,&zhao4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,3*16,&shij_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,5*16,&duan4_16[0]);
			}
			if(LightMod == 2){ //智能灯光模式标志位为2,代表按照光线亮度开启智能灯光
				ANTI_OLED_DISPLAY_16x16(4,1*16,&an4_16[0]); //显示:按照光线亮度
				ANTI_OLED_DISPLAY_16x16(4,2*16,&zhao4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guang_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,4*16,&Gxian4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,5*16,&liang4_16[0]);
				ANTI_OLED_DISPLAY_16x16(4,6*16,&du4_16[0]);
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0按键标志位
					MENU = 70 + MENUSET; //进入到子选项设置
					BKP_WriteBackupRegister(BKP_DR6, LightMod);//设置完成后,向第6个后备寄存器中写入智能灯光模式值
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(LightMod  == 0) LightMod = 3; //子选项最小值
					LightMod --; //子选项标志值减1
					OLED_DISPLAY_CLEAR_OneLine(4);//只清屏一行 行数0,2,4,6
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					LightMod++; //子选项标志值加1
					if(LightMod > 2) LightMod = 0; //子选项最大值
					OLED_DISPLAY_CLEAR_OneLine(4);//只清屏一行 行数0,2,4,6
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					BKP_WriteBackupRegister(BKP_DR6, LightMod);//设置完成后,向第6个后备寄存器中写入智能灯光模式值
					MENU = 1; //跳转到主界面
				}
			}								
		}
		if(MENU == 122){//智能灯光设置子选项  //白天区间	  			
			OLED_DISPLAY_16x16(0,2*16,&bai1_16[0]);  //显示:  白天区间
			OLED_DISPLAY_16x16(0,3*16,&tian1_16[0]);				
			OLED_DISPLAY_16x16(0,4*16,&qu1_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&jian_16[0]);
			OLED_DISPLAY_16x16(6,0*16,&deng1_16[0]);//显示: 灯光档位:				
			OLED_DISPLAY_16x16(6,1*16,&guang_16[0]);
			OLED_DISPLAY_16x16(6,2*16,&dang3_16[0]);
			OLED_DISPLAY_16x16(6,3*16,&wei4_16[0]);
			OLED_DISPLAY_8x16(6,8*8,':');//				
			if(LightShift < 3){ //用于刷新显示白天区间的结束小时值
				if(LightShift == 1){	//反白背景显示小时,表示提示用户当前选择是白天区间开始的小时值
					ANTI_OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//小时
					ANTI_OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
				}
				if(LightShift == 2){	 //反白背景显示分钟,表示提示用户当前选择是白天区间开始的分钟值
					OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//小时
					OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//分钟
					ANTI_OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//小时
				OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
				OLED_DISPLAY_8x16(4,11*8,':');//				
				OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//分钟
				OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//档位值 0~100
				OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
				OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
			}
			if(LightShift > 2){ //用于刷新显示白天区间的结束小时值
				if(LightShift == 3){	//反白背景显示小时,表示提示用户当前选择是白天区间结束的小时值
					ANTI_OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//小时
					ANTI_OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				}
				if(LightShift == 4){	 //反白背景显示分钟,表示提示用户当前选择是白天区间结束的分钟值
					OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//小时
					OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//分钟
					ANTI_OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				}
				if(LightShift == 5){	 //反白背景显示档位,表示提示用户当前选择是白天区间灯光档位
					ANTI_OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//档位值 0~100
					ANTI_OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
					ANTI_OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//小时
				OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
				OLED_DISPLAY_8x16(4,5*8,':');//				
				OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//分钟
				OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
			}
			OLED_DISPLAY_8x16(4,8*8,'-');// 白天区间开始和结束时间中间的标识符 ~				
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(LightShift < 5){
						LightShift++;//时间参数切换选择标志加1	
					}else{
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						LightShift = 1; //时间参数选择标志位置1
						BKP_WriteBackupRegister(BKP_DR7, DayTimeStartHour);//向后备寄存器写入设置后的白天区间开始小时值	
						BKP_WriteBackupRegister(BKP_DR8, DayTimeStartMin);//向后备寄存器写入设置后的白天区间开始分钟值	
						BKP_WriteBackupRegister(BKP_DR9, DayTimeEndHour);//向后备寄存器写入设置后的白天区间结束小时值	
						BKP_WriteBackupRegister(BKP_DR10, DayTimeEndMin);//向后备寄存器写入设置后的白天区间结束分钟值	
						WriteFlashBuff[1] = DayTimeGear; //白天区间灯光档位值初始值放进写入缓存数组,等待写入
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					}
				}

				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (LightShift){
							case 1: if(DayTimeStartHour <= 0) DayTimeStartHour = 24; //白天区间开始的小时最小值
									DayTimeStartHour--;
									ANTI_OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
									break;
							case 2: if(DayTimeStartMin <= 0) DayTimeStartMin = 60;  //白天区间开始的分钟最小值
									DayTimeStartMin--;
									ANTI_OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
									break;
							case 3: if(DayTimeEndHour <= 0) DayTimeEndHour = 24; //白天区间结束的小时最小值
									DayTimeEndHour--;
									ANTI_OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
									break;
							case 4: if(DayTimeEndMin <= 0) DayTimeEndMin = 60;  //白天区间结束的分钟最小值
									DayTimeEndMin--;
									ANTI_OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
									break;
							case 5:	if(DayTimeGear <= 0) DayTimeGear = 101;  //白天区间灯光档位最小值
									DayTimeGear--;	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//快速加减时刷新显示档位值 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					switch (LightShift){
						case 1: if(DayTimeStartHour <= 0) DayTimeStartHour = 24; //白天区间开始的小时最小值
								DayTimeStartHour--;
								break;
						case 2: if(DayTimeStartMin <= 0) DayTimeStartMin = 60;  //白天区间开始的分钟最小值
								DayTimeStartMin--;
								break;
						case 3: if(DayTimeEndHour <= 0) DayTimeEndHour = 24; //白天区间结束的小时最小值
								DayTimeEndHour--;
								break;
						case 4: if(DayTimeEndMin <= 0) DayTimeEndMin = 60;  //白天区间结束的分钟最小值
								DayTimeEndMin--;
								break;
						case 5:	if(DayTimeGear <= 0) DayTimeGear = 101;  //白天区间灯光档位最小值
								DayTimeGear--;	 
								break;
						default:LightShift = 1; //冗余
								break;											
					}
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (LightShift){
							case 1: DayTimeStartHour++;
									if(DayTimeStartHour >23) DayTimeStartHour = 0; //白天区间开始的小时最大值											
									ANTI_OLED_DISPLAY_8x16(4,3*8,DayTimeStartHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,4*8,DayTimeStartHour%10+0x30);//
									break;
							case 2: DayTimeStartMin++;
									if(DayTimeStartMin > 59) DayTimeStartMin = 0;  //白天区间开始的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(4,6*8,DayTimeStartMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,7*8,DayTimeStartMin%10+0x30);//
									break;
							case 3: DayTimeEndHour++;
									if(DayTimeEndHour > 23) DayTimeEndHour = 0; //白天区间结束的小时最大值											
									ANTI_OLED_DISPLAY_8x16(4,9*8,DayTimeEndHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,10*8,DayTimeEndHour%10+0x30);//
									break;
							case 4: DayTimeEndMin++;
									if(DayTimeEndMin > 59) DayTimeEndMin = 0;  //白天区间结束的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(4,12*8,DayTimeEndMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,13*8,DayTimeEndMin%10+0x30);//
									break;
							case 5:	DayTimeGear++;
									if(DayTimeGear > 100) DayTimeGear = 0;  //白天区间灯光档位最大值	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DayTimeGear/100+0x30);//快速加减时刷新显示档位值 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DayTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DayTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					switch (LightShift){
						case 1: DayTimeStartHour++;
								if(DayTimeStartHour >23) DayTimeStartHour = 0; //白天区间开始的小时最大值											
								break;
						case 2: DayTimeStartMin++;
								if(DayTimeStartMin > 59) DayTimeStartMin = 0;  //白天区间开始的分钟最大值											
								break;
						case 3: DayTimeEndHour++;
								if(DayTimeEndHour > 23) DayTimeEndHour = 0; //白天区间结束的小时最大值											
								break;
						case 4: DayTimeEndMin++;
								if(DayTimeEndMin > 59) DayTimeEndMin = 0;  //白天区间结束的分钟最大值																					
						case 5:	DayTimeGear++;
								if(DayTimeGear > 100) DayTimeGear = 0;  //白天区间灯光档位最大值
								break;	 
						default:LightShift = 1; //冗余
								break;																
					}
				} //短按结束	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					BKP_WriteBackupRegister(BKP_DR7, DayTimeStartHour);//向后备寄存器写入设置后的白天区间开始小时值	
					BKP_WriteBackupRegister(BKP_DR8, DayTimeStartMin);//向后备寄存器写入设置后的白天区间开始分钟值	
					BKP_WriteBackupRegister(BKP_DR9, DayTimeEndHour);//向后备寄存器写入设置后的白天区间结束小时值	
					BKP_WriteBackupRegister(BKP_DR10, DayTimeEndMin);//向后备寄存器写入设置后的白天区间结束分钟值	
					WriteFlashBuff[1] = DayTimeGear; //白天区间灯光档位值初始值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
				}
			}
		}
		if(MENU == 123){//智能灯光设置子选项---傍晚区间	  			
			OLED_DISPLAY_16x16(0,2*16,&bang4_16[0]);  //显示:  傍晚区间 
			OLED_DISPLAY_16x16(0,3*16,&wan3_16[0]);				
			OLED_DISPLAY_16x16(0,4*16,&qu1_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&jian_16[0]);
			OLED_DISPLAY_16x16(6,0*16,&deng1_16[0]);//显示: 灯光档位:				
			OLED_DISPLAY_16x16(6,1*16,&guang_16[0]);
			OLED_DISPLAY_16x16(6,2*16,&dang3_16[0]);
			OLED_DISPLAY_16x16(6,3*16,&wei4_16[0]);
			OLED_DISPLAY_8x16(6,8*8,':');//				
			OLED_DISPLAY_8x16(4,8*8,'-');// 傍晚区间开始和结束时间中间的标识符 ~				
			if(LightShift < 3){ //用于刷新显示傍晚区间的结束小时和分钟值
				if(LightShift == 1){	//反白背景显示小时,表示提示用户当前选择是傍晚区间结束的小时值
					ANTI_OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//小时
					ANTI_OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
				}
				if(LightShift == 2){	 //反白背景显示分钟,表示提示用户当前选择是傍晚区间结束的分钟值
					OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//小时
					OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//分钟
					ANTI_OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,3*8,DayTimeEndHour/10+0x30);//显示白天区间结束的小时和分钟,同时作为傍晚区间开始的小时和分钟
				OLED_DISPLAY_8x16(4,4*8,DayTimeEndHour%10+0x30);//
				OLED_DISPLAY_8x16(4,5*8,':');//				
				OLED_DISPLAY_8x16(4,6*8,DayTimeEndMin/10+0x30);//分钟
				OLED_DISPLAY_8x16(4,7*8,DayTimeEndMin%10+0x30);//

				OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//档位值 0~100
				OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
				OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
			}
			if(LightShift > 2){ //用于刷新显示傍晚区间的档位值
				if(LightShift == 3){	 //反白背景显示档位,表示提示用户当前选择是傍晚区间灯光档位
					ANTI_OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//档位值 0~100
					ANTI_OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
					ANTI_OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
				}
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(LightShift < 3){
						LightShift++;//时间参数切换选择标志加1	
					}else{
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						LightShift = 1; //时间参数选择标志位置1
						WriteFlashBuff[2] = DuskTimeEndHour; //设置后的傍晚区间结束小时值放进写入缓存数组,等待写入
						WriteFlashBuff[3] = DuskTimeEndMin; //设置后的傍晚区间结束分钟值放进写入缓存数组,等待写入
						WriteFlashBuff[4] = DuskTimeGear; //设置后的傍晚区间灯光档位值放进写入缓存数组,等待写入
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					}
				}
				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (LightShift){
							case 1: if(DuskTimeEndHour <= 0) DuskTimeEndHour = 24; //白天区间结束的小时最小值
									DuskTimeEndHour--;
									ANTI_OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
									break;
							case 2: if(DuskTimeEndMin <= 0) DuskTimeEndMin = 60;  //白天区间结束的分钟最小值
									DuskTimeEndMin--;
									ANTI_OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
									break;
							case 3:	if(DuskTimeGear <= 0) DuskTimeGear = 101;  //白天区间灯光档位最小值
									DuskTimeGear--;	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//快速加减时刷新显示档位值 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					switch (LightShift){
						case 1: if(DuskTimeEndHour <= 0) DuskTimeEndHour = 24; //白天区间结束的小时最小值
								DuskTimeEndHour--;
								break;
						case 2: if(DuskTimeEndMin <= 0) DuskTimeEndMin = 60;  //白天区间结束的分钟最小值
								DuskTimeEndMin--;
								break;
						case 3:	if(DuskTimeGear <= 0) DuskTimeGear = 101;  //白天区间灯光档位最小值
								DuskTimeGear--;	 
								break;
						default:LightShift = 1; //冗余
								break;											
					}
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (LightShift){
							case 1: DuskTimeEndHour++;
									if(DuskTimeEndHour > 23) DuskTimeEndHour = 0; //白天区间结束的小时最大值											
									ANTI_OLED_DISPLAY_8x16(4,9*8,DuskTimeEndHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,10*8,DuskTimeEndHour%10+0x30);//
									break;
							case 2: DuskTimeEndMin++;
									if(DuskTimeEndMin > 59) DuskTimeEndMin = 0;  //白天区间结束的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(4,12*8,DuskTimeEndMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,13*8,DuskTimeEndMin%10+0x30);//
									break;
							case 3:	DuskTimeGear++;
									if(DuskTimeGear > 100) DuskTimeGear = 0;  //白天区间灯光档位最大值	 
									ANTI_OLED_DISPLAY_8x16(6,9*8,DuskTimeGear/100+0x30);//快速加减时刷新显示档位值 0~100
									ANTI_OLED_DISPLAY_8x16(6,10*8,DuskTimeGear%100/10+0x30);//
									ANTI_OLED_DISPLAY_8x16(6,11*8,DuskTimeGear%10+0x30);//
									break;
							default:LightShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					switch (LightShift){
						case 1: DuskTimeEndHour++;
								if(DuskTimeEndHour > 23) DuskTimeEndHour = 0; //白天区间结束的小时最大值											
								break;
						case 2: DuskTimeEndMin++;
								if(DuskTimeEndMin > 59) DuskTimeEndMin = 0;  //白天区间结束的分钟最大值	
								break;																				
						case 3:	DuskTimeGear++;
								if(DuskTimeGear > 100) DuskTimeGear = 0;  //白天区间灯光档位最大值
								break;	 
						default:LightShift = 1; //冗余
								break;																
					}
				} //短按结束	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					WriteFlashBuff[2] = DuskTimeEndHour; //设置后的傍晚区间结束小时值放进写入缓存数组,等待写入
					WriteFlashBuff[3] = DuskTimeEndMin; //设置后的傍晚区间结束分钟值放进写入缓存数组,等待写入
					WriteFlashBuff[4] = DuskTimeGear; //设置后的傍晚区间灯光档位值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
				}
			}
		}
		if(MENU == 129){//智能灯光设置子选项  //开灯亮度值	  			
			OLED_DISPLAY_16x16(0,0*16,&kai1_16[0]);  //显示:开灯光线亮度设置
			OLED_DISPLAY_16x16(0,1*16,&deng1_16[0]);		
			OLED_DISPLAY_16x16(0,2*16,&guang_16[0]);					
			OLED_DISPLAY_16x16(0,3*16,&Gxian4_16[0]);									
			OLED_DISPLAY_16x16(0,4*16,&liang4_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&du4_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,7*16,&zhi4_16[0]);
			OLED_DISPLAY_8x16(4,4*8,'L');//显示: Lux:
			OLED_DISPLAY_8x16(4,5*8,'u');//
			OLED_DISPLAY_8x16(4,6*8,'x');//
			OLED_DISPLAY_8x16(4,7*8,':');//
			ANTI_OLED_DISPLAY_8x16(4,8*8,LuxSwitch/1000+0x30);//显示打开灯的亮度临界值
			ANTI_OLED_DISPLAY_8x16(4,9*8,LuxSwitch%1000/100+0x30);//
			ANTI_OLED_DISPLAY_8x16(4,10*8,LuxSwitch%100/10+0x30);//
			ANTI_OLED_DISPLAY_8x16(4,11*8,LuxSwitch%10+0x30);//
			OLED_DISPLAY_16x16(6,0*16,&dang1_16[0]);//显示: 当前光亮度:					
 			OLED_DISPLAY_16x16(6,1*16,&qian3_16[0]);								
			OLED_DISPLAY_16x16(6,2*16,&guang_16[0]);					
			OLED_DISPLAY_16x16(6,3*16,&liang4_16[0]);
			OLED_DISPLAY_16x16(6,4*16,&du4_16[0]);
			OLED_DISPLAY_8x16(6,10*8,':');//
			OLED_DISPLAY_8x16(6,11*8,EquaADC/1000+0x30);//显示ADC的值,这里是光感ADC
			OLED_DISPLAY_8x16(6,12*8,EquaADC%1000/100+0x30);//
			OLED_DISPLAY_8x16(6,13*8,EquaADC%100/10+0x30);//
			OLED_DISPLAY_8x16(6,14*8,EquaADC%10+0x30);//
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //退回到子选项设置
					WriteFlashBuff[5] = LuxSwitch; //设置后的亮度临界值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}

				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						LuxSwitch -= 100; //亮度临界值值减1
						if(LuxSwitch > 4096) LuxSwitch = 4196; //亮度临界值最小值						
						ANTI_OLED_DISPLAY_8x16(4,8*8,LuxSwitch/1000+0x30);//刷新显示反白背景显示:亮度临界值
						ANTI_OLED_DISPLAY_8x16(4,9*8,LuxSwitch%1000/100+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,10*8,LuxSwitch%100/10+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,11*8,LuxSwitch%10+0x30);//
						delay_ms(100); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					if(LuxSwitch  == 0) LuxSwitch = 4097; //水温上限最小值
					LuxSwitch--; //水温上限值减1								
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						LuxSwitch += 100; //亮度临界值值加100
						if(LuxSwitch > 4096) LuxSwitch = 0; //亮度临界值最小值						
						ANTI_OLED_DISPLAY_8x16(4,8*8,LuxSwitch/1000+0x30);//刷新显示反白背景显示:亮度临界值
						ANTI_OLED_DISPLAY_8x16(4,9*8,LuxSwitch%1000/100+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,10*8,LuxSwitch%100/10+0x30);//
						ANTI_OLED_DISPLAY_8x16(4,11*8,LuxSwitch%10+0x30);//
						delay_ms(100); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					LuxSwitch++; //水温上限值加1
					if(LuxSwitch > 4096) LuxSwitch = 0; //水温上限最大值
				} //短按结束	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					WriteFlashBuff[5] = LuxSwitch; //设置后的亮度临界值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					MENU = 1; //跳转到主界面
				}
			}								
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,智能灯光设置
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...过滤水泵设置
		if(MENU == 74){	 //过滤水泵设置			
			OLED_DISPLAY_Buff_16x16(0,1*16,&GuoLvShuiBeng_16[0],4);//显示: 过滤水泵设置
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
			
			if(FiltPumpMod == 0){ //模式值为0,表示关闭过滤水泵模式,只显示 1.模式
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(FiltPumpMod == 1){ //当模式值为1,即开启过滤水泵模式,显示调整时间区间选项
				if(SUBMENU == 1){ //选中一个子选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.模式
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //选中二个子选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.开启的时间区间				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(4,1*16,&kai1_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,2*16,&qi3_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,3*16,&de_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,4*16,&shij_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,5*16,&jian_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&qu1_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,7*16,&jian_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.开启的时间区间
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
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(FiltPumpMod == 0){ //当过滤水泵模式值为0,即关闭过滤水泵状态,只能进入模式设置
						MENU = 131; //只能进入到模式子选项设置
					}
					if(FiltPumpMod == 1){ //当过滤水泵模式值为1,即开启过滤水泵状态,可以进入子选项设置
						MENU = 130 + SUBMENU; //进入到子选项设置
					}
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(FiltPumpMod == 1){
						if(SUBMENU  == 1) SUBMENU = 3; //子选项最小值
						SUBMENU --; //子选项标志值减1					
					}
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					if(FiltPumpMod == 1){
						SUBMENU++; //子选项标志值加1
						if(SUBMENU > 2) SUBMENU = 1; //子选项最大值
					}
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}								
		}
		if(MENU == 131){//过滤水泵设置子选项 //模式设置
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //显示:  模式设置 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(FiltPumpMod == 1){	//过滤水泵模式标志位为1,代表开启过滤水泵
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//显示:开启
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //过滤水泵模式标志位不为1,代表关闭过滤水泵
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//显示: 关闭
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //进入到子选项设置
					WriteFlashBuff[6] = FiltPumpMod; //设置后的过滤水泵模式值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(FiltPumpMod  == 0) FiltPumpMod = 2; //子选项最小值
					FiltPumpMod --; //子选项标志值减1
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					FiltPumpMod++; //子选项标志值加1
					if(FiltPumpMod > 1) FiltPumpMod = 0; //子选项最大值
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					WriteFlashBuff[6] = FiltPumpMod; //设置后的过滤水泵模式值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					MENU = 1; //跳转到主界面
				}
			}								
		}
		if(MENU == 132){//过滤水泵设置子选项  //开启时间区间设置	  			
			OLED_DISPLAY_16x16(0,0*16,&kai1_16[0]);//显示:2.开启时间区间设置
			OLED_DISPLAY_16x16(0,1*16,&qi3_16[0]);
			OLED_DISPLAY_16x16(0,2*16,&shij_16[0]);
			OLED_DISPLAY_16x16(0,3*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&qu1_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,7*16,&zhi4_16[0]);

			if(MENUShift < 3){ //用于刷新显示白天区间的结束小时值
				if(MENUShift == 1){	//反白背景显示小时,表示提示用户当前选择是过滤水泵区间开始的小时值
					ANTI_OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//小时
					ANTI_OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
				}
				if(MENUShift == 2){	 //反白背景显示分钟,表示提示用户当前选择是过滤水泵区间开始的分钟值
					OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//小时
					OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
					OLED_DISPLAY_8x16(4,5*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//分钟
					ANTI_OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//结束的小时
				OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
				OLED_DISPLAY_8x16(4,11*8,':');//				
				OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//结束的分钟
				OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
			}
			if(MENUShift > 2){ //用于刷新显示白天区间的结束小时值
				if(MENUShift == 3){	//反白背景显示小时,表示提示用户当前选择是过滤水泵区间结束的小时值
					ANTI_OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//小时
					ANTI_OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
				}
				if(MENUShift == 4){	 //反白背景显示分钟,表示提示用户当前选择是过滤水泵区间结束的分钟值
					OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//小时
					OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
					OLED_DISPLAY_8x16(4,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//分钟
					ANTI_OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//开始的小时
				OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
				OLED_DISPLAY_8x16(4,5*8,':');//				
				OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//开始的分钟
				OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
			}
			OLED_DISPLAY_8x16(4,8*8,'-');// 过滤水泵区间开始和结束时间中间的标识符 ~				
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(MENUShift < 4){
						MENUShift++;//时间参数切换选择标志加1	
					}else{
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						MENUShift = 1; //时间参数选择标志位置1
						WriteFlashBuff[7] = FiltPumpStartHour; //设置后的过滤水泵开启的区间开始小时值放进写入缓存数组,等待写入
						WriteFlashBuff[8] = FiltPumpStartMin; //设置后的过滤水泵开启的区间开始分钟值放进写入缓存数组,等待写入
						WriteFlashBuff[9] = FiltPumpEndHour; //设置后的过滤水泵开启的区间结束小时值放进写入缓存数组,等待写入
						WriteFlashBuff[10] = FiltPumpEndMin; //设置后的过滤水泵开启的区间结束分钟值放进写入缓存数组,等待写入
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					}
				}

				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (MENUShift){
							case 1: if(FiltPumpStartHour <= 0) FiltPumpStartHour = 24; //过滤水泵开启区间开始的小时最小值
									FiltPumpStartHour--;
									ANTI_OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
									break;
							case 2: if(FiltPumpStartMin <= 0) FiltPumpStartMin = 60;  //过滤水泵开启区间开始的分钟最小值
									FiltPumpStartMin--;
									ANTI_OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
									break;
							case 3: if(FiltPumpEndHour <= 0) FiltPumpEndHour = 24; //过滤水泵开启区间结束的小时最小值
									FiltPumpEndHour--;
									ANTI_OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
									break;
							case 4: if(FiltPumpEndMin <= 0) FiltPumpEndMin = 60;  //过滤水泵开启区间结束的分钟最小值
									FiltPumpEndMin--;
									ANTI_OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
									break;
							default:MENUShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					switch (MENUShift){
						case 1: if(FiltPumpStartHour <= 0) FiltPumpStartHour = 24; //过滤水泵开启区间开始的小时最小值
								FiltPumpStartHour--;
								break;
						case 2: if(FiltPumpStartMin <= 0) FiltPumpStartMin = 60;  //过滤水泵开启区间开始的分钟最小值
								FiltPumpStartMin--;
								break;
						case 3: if(FiltPumpEndHour <= 0) FiltPumpEndHour = 24; //过滤水泵开启区间结束的小时最小值
								FiltPumpEndHour--;
								break;
						case 4: if(FiltPumpEndMin <= 0) FiltPumpEndMin = 60;  //过滤水泵开启区间结束的分钟最小值
								FiltPumpEndMin--;
								break;
						default:MENUShift = 1; //冗余
								break;											
					}
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (MENUShift){
							case 1: FiltPumpStartHour++;
									if(FiltPumpStartHour >23) FiltPumpStartHour = 0; //过滤水泵开启区间开始的小时最大值											
									ANTI_OLED_DISPLAY_8x16(4,3*8,FiltPumpStartHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,4*8,FiltPumpStartHour%10+0x30);//
									break;
							case 2: FiltPumpStartMin++;
									if(FiltPumpStartMin > 59) FiltPumpStartMin = 0;  //过滤水泵开启区间开始的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(4,6*8,FiltPumpStartMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,7*8,FiltPumpStartMin%10+0x30);//
									break;
							case 3: FiltPumpEndHour++;
									if(FiltPumpEndHour > 23) FiltPumpEndHour = 0; //过滤水泵开启区间结束的小时最大值											
									ANTI_OLED_DISPLAY_8x16(4,9*8,FiltPumpEndHour/10+0x30);//快速加减时刷新显示小时值
									ANTI_OLED_DISPLAY_8x16(4,10*8,FiltPumpEndHour%10+0x30);//
									break;
							case 4: FiltPumpEndMin++;
									if(FiltPumpEndMin > 59) FiltPumpEndMin = 0;  //过滤水泵开启区间结束的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(4,12*8,FiltPumpEndMin/10+0x30);//快速加减时刷新显示分钟值
									ANTI_OLED_DISPLAY_8x16(4,13*8,FiltPumpEndMin%10+0x30);//
									break;
							default:MENUShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					switch (MENUShift){
						case 1: FiltPumpStartHour++;
								if(FiltPumpStartHour >23) FiltPumpStartHour = 0; //过滤水泵开启区间开始的小时最大值											
								break;
						case 2: FiltPumpStartMin++;
								if(FiltPumpStartMin > 59) FiltPumpStartMin = 0;  //过滤水泵开启区间开始的分钟最大值											
								break;
						case 3: FiltPumpEndHour++;
								if(FiltPumpEndHour > 23) FiltPumpEndHour = 0; //过滤水泵开启区间结束的小时最大值											
								break;
						case 4: FiltPumpEndMin++;
								if(FiltPumpEndMin > 59) FiltPumpEndMin = 0;  //过滤水泵开启区间结束的分钟最大值
								break;																					
						default:MENUShift = 1; //冗余
								break;																
					}
				} //短按结束	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					WriteFlashBuff[7] = FiltPumpStartHour; //设置后的过滤水泵开启的区间开始小时值放进写入缓存数组,等待写入
					WriteFlashBuff[8] = FiltPumpStartMin; //设置后的过滤水泵开启的区间开始分钟值放进写入缓存数组,等待写入
					WriteFlashBuff[9] = FiltPumpEndHour; //设置后的过滤水泵开启的区间结束小时值放进写入缓存数组,等待写入
					WriteFlashBuff[10] = FiltPumpEndMin; //设置后的过滤水泵开启的区间结束分钟值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
				}
			}
		}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,过滤水泵设置
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...自动投喂设置
		if(MENU == 75){	 //自动投喂设置			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongTouWei_16[0],4);//显示: 自动投喂设置
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
		
			if(AutoFeedMod == 0){ //模式值为0,表示关闭过滤水泵模式,只显示 1.模式
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(AutoFeedMod == 1){ //当模式值为1,即开启过滤水泵模式,显示调整时间区间选项
				if(SUBMENU == 1){ //选中一个子选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.模式
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //选中二个子选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.自动投喂时间				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);//
					ANTI_OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.自动投喂时间
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongTouWei_16[0],4);//
					OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}	
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(AutoFeedMod == 0){ //当过滤水泵模式值为0,即关闭过滤水泵状态,只能进入模式设置
						MENU = 141; //只能进入到模式子选项设置
					}
					if(AutoFeedMod == 1){ //当过滤水泵模式值为1,即开启过滤水泵状态,可以进入子选项设置
						MENU = 140 + SUBMENU; //进入到子选项设置
					}
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(AutoFeedMod == 1){
						if(SUBMENU  == 1) SUBMENU = 3; //子选项最小值
						SUBMENU --; //子选项标志值减1					
					}
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					if(AutoFeedMod == 1){
						SUBMENU++; //子选项标志值加1
						if(SUBMENU > 2) SUBMENU = 1; //子选项最大值
					}
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}								
		}

		if(MENU == 141){//自动投喂设置子选项 //模式设置
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //显示:  模式设置 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(AutoFeedMod == 1){	//过滤水泵模式标志位为1,代表开启自动投喂
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//显示:开启
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //过滤水泵模式标志位不为1,代表关闭自动投喂
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//显示: 关闭
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //进入到子选项设置
					WriteFlashBuff[11] = AutoFeedMod; //设置后的自动投喂模式值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(AutoFeedMod  == 0) AutoFeedMod = 2; //子选项最小值
					AutoFeedMod --; //子选项标志值减1
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					AutoFeedMod++; //子选项标志值加1
					if(AutoFeedMod > 1) AutoFeedMod = 0; //子选项最大值
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					WriteFlashBuff[11] = AutoFeedMod; //设置后的自动投喂模式值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					MENU = 1; //跳转到主界面
				}
			}								
		}
		if(MENU == 142){//自动投喂设置子选项  //自动投喂时间设置	  			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongTouWei_16[2*32],2);//显示:投喂时间设置
			OLED_DISPLAY_16x16(0,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);

			OLED_DISPLAY_Buff_16x16(4,0*16,&ZiDongTouWei_16[2*32],2);//显示:投喂时间
			OLED_DISPLAY_16x16(4,2*16,&shij_16[0]);
			OLED_DISPLAY_16x16(4,3*16,&jian_16[0]);

			OLED_DISPLAY_8x16(4,9*8,'-');// 过滤水泵区间开始和结束时间中间的标识符 -
			
			OLED_DISPLAY_16x16(6,1*16,&chi2_16[0]);	//显示:持续时间:  S
			OLED_DISPLAY_16x16(6,2*16,&xu4_16[0]);			
			OLED_DISPLAY_16x16(6,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(6,4*16,&jian_16[0]);
			OLED_DISPLAY_8x16(6,10*8,':');//
			OLED_DISPLAY_8x16(6,13*8,'S');//


			if(MENUShift == 1){	//反白背景显示小时,表示提示用户当前选择是过滤水泵区间开始的小时值
				ANTI_OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//显示选中的投喂时间组 一共5组
				if(FeedTimeHour[FeedTimeNum] == 24){ //当前为关闭该时间组标志位 显示OFF  
					OLED_DISPLAY_8x16(4,10*8,'O');//
					OLED_DISPLAY_8x16(4,11*8,'F');//
					OLED_DISPLAY_8x16(4,12*8,'F');//
					OLED_DISPLAY_8x16(4,13*8,' ');//
					OLED_DISPLAY_8x16(4,14*8,' ');//
					OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//刷新显示自动喂食的持续时长 0--59s
					OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//				
				}else{ //当前是正常显示
					OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//刷新显示投喂时间小时
					OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,':');//				
					OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//刷新显示投喂时间分钟
					OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//刷新显示自动喂食的持续时长 0--59s
					OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//				
				}
			}
			if(MENUShift > 1){ //用于刷新显示白天区间的结束小时值
				if(MENUShift == 2){	//反白背景显示小时,表示提示用户当前选择是过滤水泵区间结束的小时值
					if(FeedTimeHour[FeedTimeNum] == 24){ //当前为关闭该时间组标志位 显示OFF  
						ANTI_OLED_DISPLAY_8x16(4,10*8,'O');//
						ANTI_OLED_DISPLAY_8x16(4,11*8,'F');//
						ANTI_OLED_DISPLAY_8x16(4,12*8,'F');//
						OLED_DISPLAY_8x16(4,13*8,' ');//
						OLED_DISPLAY_8x16(4,14*8,' ');//
					}else{ //当前是正常显示
						ANTI_OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//小时
						ANTI_OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
						OLED_DISPLAY_8x16(4,12*8,':');//				
						OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//分钟
						OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
					}
					OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//刷新显示自动喂食的持续时长 0--59s
					OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//				
				}
				if(MENUShift == 3){	 //反白背景显示分钟,表示提示用户当前选择是过滤水泵区间结束的分钟值
					OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//小时
					OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
					OLED_DISPLAY_8x16(4,12*8,':');//				
					ANTI_OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//分钟
					ANTI_OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
				}																						   
				if(MENUShift == 4){	 //反白背景显示分钟,表示提示用户当前选择是过滤水泵区间结束的分钟值
					ANTI_OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//显示自动喂食的持续时长 0--59s
					ANTI_OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//
				
					OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//分钟
					OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
				}

				OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//显示选中的投喂时间组 一共5组
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(MENUShift == 2 && FeedTimeHour[FeedTimeNum] == 24){
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						MENUShift = 1; //参数选择标志位置1
						WriteFlashBuff[11+FeedTimeNum] = FeedTimeHour[FeedTimeNum]; //设置后的自动投喂小时值放进写入缓存数组,等待写入
						WriteFlashBuff[16+FeedTimeNum] = FeedTimeMin[FeedTimeNum]; //设置后的自动投喂分钟值放进写入缓存数组,等待写入
						WriteFlashBuff[21+FeedTimeNum] = FeedTimeLast[FeedTimeNum]; //设置后自动投喂持续时长值放进写入缓存数组,等待写入
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定					
					}else if(MENUShift < 4){
						MENUShift++;//时间参数切换选择标志加1	
					}else{
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						MENUShift = 1; //时间参数选择标志位置1
						WriteFlashBuff[11+FeedTimeNum] = FeedTimeHour[FeedTimeNum]; //设置后的自动投喂小时值放进写入缓存数组,等待写入
						WriteFlashBuff[16+FeedTimeNum] = FeedTimeMin[FeedTimeNum]; //设置后的自动投喂分钟值放进写入缓存数组,等待写入
						WriteFlashBuff[21+FeedTimeNum] = FeedTimeLast[FeedTimeNum]; //设置后自动投喂持续时长值放进写入缓存数组,等待写入
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					}
				}

				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (MENUShift){
//							case 1: if(FeedTimeNum == 1) FeedTimeNum = 6; //过滤水泵开启区间开始的小时最小值
//									FeedTimeNum--;
//									ANTI_OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//显示选中的投喂时间组 一共5组
//									break;
							case 2: if(FeedTimeHour[FeedTimeNum] <= 0) FeedTimeHour[FeedTimeNum] = 25;  //过滤水泵开启区间开始的分钟最小值
									FeedTimeHour[FeedTimeNum]--;
									ANTI_OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//快速加减时刷新显示小时
									ANTI_OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
									break;
							case 3: if(FeedTimeMin[FeedTimeNum] <= 0) FeedTimeMin[FeedTimeNum] = 60;  //过滤水泵开启区间结束的分钟最小值
									FeedTimeMin[FeedTimeNum]--;
									ANTI_OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//分钟
									ANTI_OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
									break;
							case 4: if(FeedTimeLast[FeedTimeNum] <= 0) FeedTimeLast[FeedTimeNum] = 60;  //过滤水泵开启区间结束的分钟最小值
									FeedTimeLast[FeedTimeNum]--;
									ANTI_OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//显示自动喂食的持续时长 0--59s
									ANTI_OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					switch (MENUShift){
						case 1: if(FeedTimeNum == 1) FeedTimeNum = 6; //过滤水泵开启区间开始的小时最小值
								FeedTimeNum--;
								break;
						case 2: if(FeedTimeHour[FeedTimeNum] <= 0) FeedTimeHour[FeedTimeNum] = 25;  //过滤水泵开启区间开始的分钟最小值
								FeedTimeHour[FeedTimeNum]--;
								break;
						case 3: if(FeedTimeMin[FeedTimeNum] <= 0) FeedTimeMin[FeedTimeNum] = 60;  //过滤水泵开启区间结束的分钟最小值
								FeedTimeMin[FeedTimeNum]--;
								break;
						case 4: if(FeedTimeLast[FeedTimeNum] <= 0) FeedTimeLast[FeedTimeNum] = 60;  //过滤水泵开启区间结束的分钟最小值
								FeedTimeLast[FeedTimeNum]--;
								break;
						default:MENUShift = 1; //冗余
								break;											
					}
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (MENUShift){
//							case 1: FeedTimeNum++;
//									if(FeedTimeNum >5) FeedTimeNum = 1; //过滤水泵开启区间开始的小时最大值											
//									ANTI_OLED_DISPLAY_8x16(4,8*8,FeedTimeNum%10+0x30);//显示选中的投喂时间组 一共5组
//									break;
							case 2: FeedTimeHour[FeedTimeNum]++;
									if(FeedTimeHour[FeedTimeNum] > 24) FeedTimeHour[FeedTimeNum] = 0;  //过滤水泵开启区间开始的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTimeNum]/10+0x30);//快速加减时刷新显示小时
									ANTI_OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTimeNum]%10+0x30);//
									break;
							case 3: FeedTimeMin[FeedTimeNum]++;
									if(FeedTimeMin[FeedTimeNum] > 59) FeedTimeMin[FeedTimeNum] = 0;  //过滤水泵开启区间结束的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTimeNum]/10+0x30);//分钟
									ANTI_OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTimeNum]%10+0x30);//
									break;
							case 4: FeedTimeLast[FeedTimeNum]++;
									if(FeedTimeLast[FeedTimeNum] > 59) FeedTimeLast[FeedTimeNum] = 0;  //过滤水泵开启区间结束的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(6,11*8,FeedTimeLast[FeedTimeNum]/10+0x30);//显示自动喂食的持续时长 0--59s
									ANTI_OLED_DISPLAY_8x16(6,12*8,FeedTimeLast[FeedTimeNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					switch (MENUShift){
						case 1: FeedTimeNum++;
								if(FeedTimeNum >5) FeedTimeNum = 1; //过滤水泵开启区间开始的小时最大值											
								break;
						case 2: FeedTimeHour[FeedTimeNum]++;
								if(FeedTimeHour[FeedTimeNum] > 24) FeedTimeHour[FeedTimeNum] = 0;  //过滤水泵开启区间开始的分钟最大值											
								break;
						case 3: FeedTimeMin[FeedTimeNum]++;
								if(FeedTimeMin[FeedTimeNum] > 59) FeedTimeMin[FeedTimeNum] = 0;  //过滤水泵开启区间结束的分钟最大值											
								break;
						case 4: FeedTimeLast[FeedTimeNum]++;
								if(FeedTimeLast[FeedTimeNum] > 59) FeedTimeLast[FeedTimeNum] = 0;  //过滤水泵开启区间结束的分钟最大值											
								break;																					
						default:MENUShift = 1; //冗余
								break;																
					}
				} //短按结束	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					WriteFlashBuff[11+FeedTimeNum] = FeedTimeHour[FeedTimeNum]; //设置后的自动投喂小时值放进写入缓存数组,等待写入
					WriteFlashBuff[16+FeedTimeNum] = FeedTimeMin[FeedTimeNum]; //设置后的自动投喂分钟值放进写入缓存数组,等待写入
					WriteFlashBuff[21+FeedTimeNum] = FeedTimeLast[FeedTimeNum]; //设置后自动投喂持续时长值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
				}
			}
		}
		if(MENU == 149){//静态显示触发投喂菜单  	  			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongTouWei_16[2*32],2);//显示:投喂进行中
			OLED_DISPLAY_16x16(0,3*16,&jin4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&xing2_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhong1_16[0]);

			OLED_DISPLAY_Buff_16x16(4,0*16,&ZiDongTouWei_16[2*32],2);//显示:投喂时间
			OLED_DISPLAY_16x16(4,2*16,&shij_16[0]);
			OLED_DISPLAY_16x16(4,3*16,&jian_16[0]);

			OLED_DISPLAY_8x16(4,9*8,'-');// 过滤水泵区间开始和结束时间中间的标识符 -
			
			OLED_DISPLAY_16x16(6,1*16,&chi2_16[0]);	//显示:持续时间:  S
			OLED_DISPLAY_16x16(6,2*16,&xu4_16[0]);			
			OLED_DISPLAY_16x16(6,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(6,4*16,&jian_16[0]);
			OLED_DISPLAY_8x16(6,10*8,':');//
			OLED_DISPLAY_8x16(6,13*8,'S');//

			OLED_DISPLAY_8x16(4,8*8,FeedTriggerNum%10+0x30);//显示触发的投喂时间组 一共5组
			OLED_DISPLAY_8x16(4,10*8,FeedTimeHour[FeedTriggerNum]/10+0x30);//刷新显示触发的投喂时间小时
			OLED_DISPLAY_8x16(4,11*8,FeedTimeHour[FeedTriggerNum]%10+0x30);//
			OLED_DISPLAY_8x16(4,12*8,':');//				
			OLED_DISPLAY_8x16(4,13*8,FeedTimeMin[FeedTriggerNum]/10+0x30);//刷新显示触发的投喂时间分钟
			OLED_DISPLAY_8x16(4,14*8,FeedTimeMin[FeedTriggerNum]%10+0x30);//
			MENU = 150; //跳转到动态刷新显示触发投喂倒计时菜单
		}
		if(MENU == 150){ //动态刷新显示触发投喂倒计时菜单
			RTC_Get();//读出当前时间值
			if(rsec%2 == 1 && FeedLastCountFlag != 1){
				FeedLastCount -= 1; //每触发一次就是1秒
				FeedLastCountFlag = 1;
				TIM3_PWM_CH3_Init(59999,23); //Time3通道3初始化,
				TIM_SetCompare3(TIM3,1500);  //舵机0度角
				OLED_DISPLAY_8x16(6,11*8,FeedLastCount/10+0x30);//倒计时刷新显示触发的喂食持续时长 0--59s
				OLED_DISPLAY_8x16(6,12*8,FeedLastCount%10+0x30);//				
			}else if(rsec%2 == 0 && FeedLastCountFlag != 0){
				FeedLastCount -= 1; //每触发一次就是1秒
				FeedLastCountFlag = 0;
				TIM3_PWM_CH3_Init(59999,23); //Time3通道3初始化,
				TIM_SetCompare3(TIM3,7500);  //舵机180度角
				OLED_DISPLAY_8x16(6,11*8,FeedLastCount/10+0x30);//倒计时刷新显示触发的喂食持续时长 0--59s
				OLED_DISPLAY_8x16(6,12*8,FeedLastCount%10+0x30);//				
			}
			if(FeedLastCount == 0){	//如果倒计时结束
				MENU = 1; //跳转到主菜单
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}		
		}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,自动投喂设置
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....自动换水设置
		if(MENU == 76){	 //	自动换水设置		
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongHuanShui_16[0],4);//显示:	自动换水设置
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);
		
			if(ChangeWaterMod == 0){ //模式值为0,表示关闭自动换水模式,只显示 1.模式
				ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
				ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
				ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
				ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);						
			}
			if(ChangeWaterMod == 1){ //当模式值为1,即开启自动换水模式,显示调整时间区间选项
				if(SUBMENU == 1){ //选中一个子选项
					ANTI_OLED_DISPLAY_8x16(2,0*8,'1');//反白背景显示:1.模式				
					ANTI_OLED_DISPLAY_8x16(2,1*8,'.');//
					ANTI_OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					ANTI_OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}else{
					OLED_DISPLAY_8x16(2,0*8,'1');//正常显示:1.模式
					OLED_DISPLAY_8x16(2,1*8,'.');//
					OLED_DISPLAY_16x16(2,1*16,&mo2_16[0]);  
					OLED_DISPLAY_16x16(2,2*16,&shi4_16[0]);
				}	
				if(SUBMENU == 2){ //选中二个子选项
					ANTI_OLED_DISPLAY_8x16(4,0*8,'2');//反白背景显示:2.自动换水时间				
					ANTI_OLED_DISPLAY_8x16(4,1*8,'.');//
					ANTI_OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongHuanShui_16[0],4);//
					ANTI_OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					ANTI_OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}else{
					OLED_DISPLAY_8x16(4,0*8,'2');//正常显示:2.自动换水时间
					OLED_DISPLAY_8x16(4,1*8,'.');//
					OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongHuanShui_16[0],4);//
					OLED_DISPLAY_16x16(4,5*16,&shij_16[0]);
					OLED_DISPLAY_16x16(4,6*16,&jian_16[0]);
				}	
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(ChangeWaterMod == 0){ //当自动换水模式值为0,即关闭自动换水状态,只能进入模式设置
						MENU = 151; //只能进入到模式子选项设置
					}
					if(ChangeWaterMod == 1){ //当自动换水模式值为1,即开启自动换水状态,可以进入子选项设置
						MENU = 150 + SUBMENU; //进入到子选项设置
					}
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(ChangeWaterMod == 1){
						if(SUBMENU  == 1) SUBMENU = 3; //子选项最小值
						SUBMENU --; //子选项标志值减1					
					}
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					if(ChangeWaterMod == 1){
						SUBMENU++; //子选项标志值加1
						if(SUBMENU > 2) SUBMENU = 1; //子选项最大值
					}
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}								
		}

		if(MENU == 151){//自动换水模式设置
			OLED_DISPLAY_16x16(0,2*16,&mo2_16[0]); //显示:  模式设置 
			OLED_DISPLAY_16x16(0,3*16,&shi4_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&zhi4_16[0]);
			if(ChangeWaterMod == 1){	//模式标志位为1,代表开启自动换水
				ANTI_OLED_DISPLAY_16x16(4,3*16,&kai1_16[0]);//显示:开启
				ANTI_OLED_DISPLAY_16x16(4,4*16,&qi3_16[0]);
			}else{ //模式标志位不为1,代表关闭自动换水
				ANTI_OLED_DISPLAY_16x16(4,3*16,&guan1_16[0]);//显示: 关闭
				ANTI_OLED_DISPLAY_16x16(4,4*16,&bi4_16[0]);			
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					MENU = 70 + MENUSET; //进入到子选项设置
					WriteFlashBuff[27] = ChangeWaterMod; //设置后的自动换水模式值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					OLED_DISPLAY_CLEAR();//oled清屏操作
				}
				if(KEY == 2){ //按键按下
					KEY = 0; //清0标志位
					if(ChangeWaterMod  == 0) ChangeWaterMod = 2; //子选项最小值
					ChangeWaterMod --; //子选项标志值减1
				}
				if(KEY == 3){ //按键按下
					KEY = 0; //清0标志位
					ChangeWaterMod++; //子选项标志值加1
					if(ChangeWaterMod > 1) ChangeWaterMod = 0; //子选项最大值
				}
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					WriteFlashBuff[27] = ChangeWaterMod; //设置后的自动换水模式值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//一次性写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					MENU = 1; //跳转到主界面
				}
			}								
		}

		if(MENU == 152){//自动换水时间设置	  			
			OLED_DISPLAY_Buff_16x16(0,1*16,&ZiDongHuanShui_16[2*32],2);//显示:换水时间设置
			OLED_DISPLAY_16x16(0,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(0,4*16,&jian_16[0]);
			OLED_DISPLAY_16x16(0,5*16,&she_16[0]);
			OLED_DISPLAY_16x16(0,6*16,&zhi4_16[0]);

			OLED_DISPLAY_Buff_16x16(4,1*16,&ZiDongHuanShui_16[2*32],2);//显示:换水时间
			OLED_DISPLAY_16x16(4,3*16,&shij_16[0]);
			OLED_DISPLAY_16x16(4,4*16,&jian_16[0]);

			OLED_DISPLAY_16x16(6,1*16,&xing_16[0]);//显示: 星期
			OLED_DISPLAY_16x16(6,2*16,&qi1_16[0]);
			
			if(MENUShift == 1){	//反白背景显示,表示提示用户当前选择是换水时间组
				ANTI_OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//显示选中的换水时间组 一共5组
				if(ChangWatWeek[ChangWatTimNum] == 7){ //当前为关闭该时间组标志位 显示OFF  
					OLED_DISPLAY_8x16(6,6*8,'O');//
					OLED_DISPLAY_8x16(6,7*8,'F');//
					OLED_DISPLAY_8x16(6,8*8,'F');//
					OLED_DISPLAY_8x16(6,9*8,' ');//
					OLED_DISPLAY_8x16(6,10*8,' ');//
					OLED_DISPLAY_8x16(6,11*8,' ');//
					OLED_DISPLAY_8x16(6,12*8,' ');//
					OLED_DISPLAY_8x16(6,13*8,' ');//
				}else{ //当前是正常显示
					OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//显示星期一~~日
					OLED_DISPLAY_8x16(6,8*8,'-');// 星期?和时间中间的标识符 -
					OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//小时
					OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,':');//				
					OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//分钟
					OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
				}
			}
			if(MENUShift > 1){ //用于刷新显示白天区间的结束小时值
				if(MENUShift == 2){	//反白背景显示,表示提示用户当前选择是星期值
					if(ChangWatWeek[ChangWatTimNum] == 7){ //当前为关闭该时间组标志位 显示OFF  
						ANTI_OLED_DISPLAY_8x16(6,6*8,'O');//
						ANTI_OLED_DISPLAY_8x16(6,7*8,'F');//
						ANTI_OLED_DISPLAY_8x16(6,8*8,'F');//
						OLED_DISPLAY_8x16(6,9*8,' ');//
						OLED_DISPLAY_8x16(6,10*8,' ');//
						OLED_DISPLAY_8x16(6,11*8,' ');//
						OLED_DISPLAY_8x16(6,12*8,' ');//
						OLED_DISPLAY_8x16(6,13*8,' ');//
					}else{ //当前是正常显示
						ANTI_OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//显示星期一~~日
						OLED_DISPLAY_8x16(6,8*8,'-');// 星期?和时间中间的标识符 -
						OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//小时
						OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
						OLED_DISPLAY_8x16(6,11*8,':');//				
						OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//分钟
						OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
					}
				}
				if(MENUShift == 3){	 //反白背景显示,表示提示用户当前选择是小时值
					ANTI_OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//小时
					ANTI_OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,':');//				
					OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//分钟
					OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//

					OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//当前不是选择星期  正常显示星期一~~日
				}																						   
				if(MENUShift == 4){	 //反白背景显示,提示用户当前选择是分钟值				
					OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//小时
					OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
					OLED_DISPLAY_8x16(6,11*8,':');//				
					ANTI_OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//分钟
					ANTI_OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
				}
				OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//显示选中的换水时间组 一共5组
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下
				if(KEY == 1){ //按键1即按键A按下
					KEY = 0; //清0标志位
					if(MENUShift == 2 && ChangWatWeek[ChangWatTimNum] == 7){
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						MENUShift = 1; //参数选择标志位置1
						WriteFlashBuff[27+ChangWatTimNum] = ChangWatWeek[ChangWatTimNum]; //设置后的自动换水小时值放进写入缓存数组,等待写入
						WriteFlashBuff[32+ChangWatTimNum] = ChangWatTimHour[ChangWatTimNum]; //设置后的自动换水分钟值放进写入缓存数组,等待写入
						WriteFlashBuff[37+ChangWatTimNum] = ChangWatTimMin[ChangWatTimNum]; //设置后自动换水持续时长值放进写入缓存数组,等待写入
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					}else if(MENUShift < 4){
						MENUShift++;//时间参数切换选择标志加1	
					}else{
						MENU = 70 + MENUSET; //退回到子选项设置
						OLED_DISPLAY_CLEAR();//oled清屏操作
						MENUShift = 1; //时间参数选择标志位置1
						WriteFlashBuff[27+ChangWatTimNum] = ChangWatWeek[ChangWatTimNum]; //设置后的自动换水小时值放进写入缓存数组,等待写入
						WriteFlashBuff[32+ChangWatTimNum] = ChangWatTimHour[ChangWatTimNum]; //设置后的自动换水分钟值放进写入缓存数组,等待写入
						WriteFlashBuff[37+ChangWatTimNum] = ChangWatTimMin[ChangWatTimNum]; //设置后自动换水持续时长值放进写入缓存数组,等待写入
						FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
					}
				}
				if(KEY == 12){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){
						switch (MENUShift){
//							case 1: if(ChangWatTimNum == 1) ChangWatTimNum = 6; //过滤水泵开启区间开始的小时最小值
//									ChangWatTimNum--;
//									ANTI_OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//显示选中的换水时间组 一共5组
//									break;
//							case 2: ChangWatWeek[ChangWatTimNum]--;
//									if(ChangWatWeek[ChangWatTimNum] > 6) ChangWatWeek[ChangWatTimNum] = 7;  //星期最小值									
//									ANTI_OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//快速加减时刷新显示星期一~~日
//									break;
							case 3: if(ChangWatTimHour[ChangWatTimNum] <= 0) ChangWatTimHour[ChangWatTimNum] = 24;  //过滤水泵开启区间开始的分钟最小值
									ChangWatTimHour[ChangWatTimNum]--;
									ANTI_OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//快速加减时刷新显示小时
									ANTI_OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
									break;
							case 4: if(ChangWatTimMin[ChangWatTimNum] <= 0) ChangWatTimMin[ChangWatTimNum] = 60;  //过滤水泵开启区间结束的分钟最小值
									ChangWatTimMin[ChangWatTimNum]--;
									ANTI_OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//快速加减时刷新显示分钟
									ANTI_OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 2){ //按键短按
					KEY = 0; //清0标志位
					switch (MENUShift){
						case 1: if(ChangWatTimNum == 1) ChangWatTimNum = 6; //过滤水泵开启区间开始的小时最小值
								ChangWatTimNum--;
								break;
						case 2: ChangWatWeek[ChangWatTimNum]--;
								if(ChangWatWeek[ChangWatTimNum] > 6) ChangWatWeek[ChangWatTimNum] = 7;  //星期最小值									
								break;
						case 3: if(ChangWatTimHour[ChangWatTimNum] <= 0) ChangWatTimHour[ChangWatTimNum] = 24;  //过滤水泵开启区间开始的分钟最小值
								ChangWatTimHour[ChangWatTimNum]--;
								break;
						case 4: if(ChangWatTimMin[ChangWatTimNum] <= 0) ChangWatTimMin[ChangWatTimNum] = 60;  //过滤水泵开启区间结束的分钟最小值
								ChangWatTimMin[ChangWatTimNum]--;
								break;
						default:MENUShift = 1; //冗余
								break;											
					}
				} //短按结束
				if(KEY == 13){ //按键长按
					KEY = 0; //清0标志位
					while(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){
						switch (MENUShift){
//							case 1: ChangWatTimNum++;
//									if(ChangWatTimNum >5) ChangWatTimNum = 1; //过滤水泵开启区间开始的小时最大值											
//									ANTI_OLED_DISPLAY_8x16(4,10*8,ChangWatTimNum%10+0x30);//显示选中的换水时间组 一共5组
//									break;
//							case 2: ChangWatWeek[ChangWatTimNum]++;
//									if(ChangWatWeek[ChangWatTimNum] > 7) ChangWatWeek[ChangWatTimNum] = 0;  //星期最大值									
//									ANTI_OLED_DISPLAY_16x16(6,3*16,&xingqi1_7_16[ChangWatWeek[ChangWatTimNum]*32]);	//快速加减时刷新显示星期一~~日
//									break;
							case 3: ChangWatTimHour[ChangWatTimNum]++;
									if(ChangWatTimHour[ChangWatTimNum] > 23) ChangWatTimHour[ChangWatTimNum] = 0;  //过滤水泵开启区间开始的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(6,9*8,ChangWatTimHour[ChangWatTimNum]/10+0x30);//快速加减时刷新显示小时
									ANTI_OLED_DISPLAY_8x16(6,10*8,ChangWatTimHour[ChangWatTimNum]%10+0x30);//
									break;
							case 4: ChangWatTimMin[ChangWatTimNum]++;
									if(ChangWatTimMin[ChangWatTimNum] > 59) ChangWatTimMin[ChangWatTimNum] = 0;  //过滤水泵开启区间结束的分钟最大值											
									ANTI_OLED_DISPLAY_8x16(6,12*8,ChangWatTimMin[ChangWatTimNum]/10+0x30);//快速加减时刷新显示分钟
									ANTI_OLED_DISPLAY_8x16(6,13*8,ChangWatTimMin[ChangWatTimNum]%10+0x30);//
									break;
							default:MENUShift = 1;	//冗余
									break;											
						}
						delay_ms(60); //调整一个合适的数值变化速度																
					}
				}//长按结束				
				if(KEY == 3){ //按键短按
					KEY = 0; //清0标志位
					switch (MENUShift){
						case 1: ChangWatTimNum++;
								if(ChangWatTimNum >5) ChangWatTimNum = 1; //过滤水泵开启区间开始的小时最大值											
								break;
						case 2: ChangWatWeek[ChangWatTimNum]++;
								if(ChangWatWeek[ChangWatTimNum] > 7) ChangWatWeek[ChangWatTimNum] = 0;  //星期最大值									
								break;
						case 3: ChangWatTimHour[ChangWatTimNum]++;
								if(ChangWatTimHour[ChangWatTimNum] > 23) ChangWatTimHour[ChangWatTimNum] = 0;  //过滤水泵开启区间开始的分钟最大值											
								break;
						case 4: ChangWatTimMin[ChangWatTimNum]++;
								if(ChangWatTimMin[ChangWatTimNum] > 59) ChangWatTimMin[ChangWatTimNum] = 0;  //过滤水泵开启区间结束的分钟最大值											
								break;
						default:MENUShift = 1; //冗余
								break;																
					}
				} //短按结束	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
					WriteFlashBuff[27+ChangWatTimNum] = ChangWatWeek[ChangWatTimNum]; //设置后的自动换水小时值放进写入缓存数组,等待写入
					WriteFlashBuff[32+ChangWatTimNum] = ChangWatTimHour[ChangWatTimNum]; //设置后的自动换水分钟值放进写入缓存数组,等待写入
					WriteFlashBuff[37+ChangWatTimNum] = ChangWatTimMin[ChangWatTimNum]; //设置后自动换水持续时长值放进写入缓存数组,等待写入
					FLASH_W_Buff(FLASH_START_ADDR,&WriteFlashBuff[0],WriteNum);//写入多个数据, 数据在WriteFlashBuff[]加入,数量由宏定义WriteNum决定
				}
			}
		}

		if(MENU == 159){//静态显示触发换水菜单  	  			
			ANTI_OLED_DISPLAY_8x16_BUFFER(0,"                ");	//清空第一行
			ANTI_OLED_DISPLAY_16x16(0,0*16,&zheng4_16[0]);//显示:正在抽出缸内水
			ANTI_OLED_DISPLAY_16x16(0,1*16,&zai4_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,2*16,&chou1_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,3*16,&chu1_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,4*16,&gang1_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,5*16,&nei4_16[0]);
			ANTI_OLED_DISPLAY_16x16(0,6*16,&shui_16[0]);

			OLED_DISPLAY_16x16(2,0*16,&gang1_16[0]);//显示:缸内水温:   . C
			OLED_DISPLAY_16x16(2,1*16,&nei4_16[0]);
			OLED_DISPLAY_16x16(2,2*16,&shui_16[0]);	 
			OLED_DISPLAY_16x16(2,3*16,&wen_16[0]);
			OLED_DISPLAY_8x16(2,8*8,':');// 
			OLED_DISPLAY_8x16(2,12*8,'.');// 
			OLED_DISPLAY_16x16(2,14*8,&C_16[0]); //显示温度符号℃//

			OLED_DISPLAY_16x16(4,0*16,&xin1_16[0]);//显示:新水水温:   . C
			OLED_DISPLAY_16x16(4,1*16,&shui_16[0]);
			OLED_DISPLAY_16x16(4,2*16,&shui_16[0]);	 
			OLED_DISPLAY_16x16(4,3*16,&wen_16[0]);
			OLED_DISPLAY_8x16(4,8*8,':');// 
			OLED_DISPLAY_8x16(4,12*8,'.');// 
			OLED_DISPLAY_16x16(4,14*8,&C_16[0]); //显示温度符号℃//

			OLED_DISPLAY_16x16(6,0*16,&shang4_16[0]);//显示:上水位 下水位
			OLED_DISPLAY_16x16(6,1*16,&shui_16[0]);//
			OLED_DISPLAY_16x16(6,2*16,&wei4_16[0]);//

			OLED_DISPLAY_16x16(6,4*16,&xia4_16[0]);//
			OLED_DISPLAY_16x16(6,5*16,&shui_16[0]);//
			OLED_DISPLAY_16x16(6,6*16,&wei4_16[0]);//

			MENU = 160; //跳转到动态刷新显示触发换水状态刷新菜单
		}

		if(MENU == 160){ //动态刷新显示触发换水状态刷新菜单
			//显示温度传感器1
			if(Temp > 0){//判断为正温度
				OLED_DISPLAY_8x16(2,9*8,' ');//显示温度值
				OLED_DISPLAY_8x16(2,10*8,Temp%1000/100+0x30);//显示温度值,温度范围-55~~99
				OLED_DISPLAY_8x16(2,11*8,Temp%100/10+0x30);//
				OLED_DISPLAY_8x16(2,13*8,Temp%10+0x30);//
			}
			//显示温度传感器2
			if(Temp2 > 0){//判断为正温度
				OLED_DISPLAY_8x16(4,9*8,' ');//显示温度值
				OLED_DISPLAY_8x16(4,10*8,Temp2%1000/100+0x30);//显示温度值,-550~~1250
				OLED_DISPLAY_8x16(4,11*8,Temp2%100/10+0x30);//
				OLED_DISPLAY_8x16(4,13*8,Temp2%10+0x30);//
			}

			if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_HIGH) == 1){ //如果高水位监测到有水
				OLED_DISPLAY_16x16(6,3*16,&gou1_16[0]);//显示√ 勾
			}else{ //高水位监测到没有水
				OLED_DISPLAY_16x16(6,3*16,&cha1_16[0]);//显示× 叉 
			}
			if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_LOW) == 1){ //如果低水位监测到有水
				OLED_DISPLAY_16x16(6,7*16,&gou1_16[0]);//显示√ 勾
			}else{// 低水位监测到没有水
				OLED_DISPLAY_16x16(6,7*16,&cha1_16[0]);//显示× 叉 
			}

			if(ChangWatProcess == 1){ //换水的第一步 检测水温差 不能超过±3度
				if((Temp - Temp2) > 30 || (Temp2 - Temp) > 30 ){ //如果新水水温比缸内水温低或高3度, 则水温异常
					ANTI_OLED_DISPLAY_8x16_BUFFER(0,"                ");	//清空第一行
					ANTI_OLED_DISPLAY_16x16(0,0*16,&shui_16[0]);//显示:水温异常
					ANTI_OLED_DISPLAY_16x16(0,1*16,&wen_16[0]);
					ANTI_OLED_DISPLAY_16x16(0,2*16,&yi4_16[0]);
					ANTI_OLED_DISPLAY_16x16(0,3*16,&chang2_16[0]);
					ChangWatProcess = 1; //水温异常,不允许换水操作					
				}else{					 
					 ChangWatProcess = 2; //水温正常,允许换水操作
				}
 			}
			if(ChangWatProcess == 2){ //换水的第二步
				ANTI_OLED_DISPLAY_16x16(0,0*16,&zheng4_16[0]);//显示:正在抽出缸内水
				ANTI_OLED_DISPLAY_16x16(0,1*16,&zai4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,2*16,&chou1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,3*16,&chu1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,4*16,&gang1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,5*16,&nei4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,6*16,&shui_16[0]);
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS1,(BitAction)(1));//开启缸内的抽水机
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS2,(BitAction)(0));//关闭新水的抽水机
				if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_LOW) == 0 && GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_HIGH) == 0){ //如果低水位监测到没有水
					ChangWatProcess = 3; //跳转到换水的第三步
				}
			} 
			if(ChangWatProcess == 3){	//换水的第三步
				ANTI_OLED_DISPLAY_16x16(0,0*16,&zheng4_16[0]);//显示:正在抽新水进缸
				ANTI_OLED_DISPLAY_16x16(0,1*16,&zai4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,2*16,&chou1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,3*16,&xin1_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,4*16,&shui_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,5*16,&jin4_16[0]);
				ANTI_OLED_DISPLAY_16x16(0,6*16,&gang1_16[0]);
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS1,(BitAction)(0));//关闭缸内的抽水机
				GPIO_WriteBit(MOS_SENSOR_PORT,MOS2,(BitAction)(1));//开启新水的抽水机
				if(GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_HIGH) == 1 && GPIO_ReadInputDataBit(MOS_SENSOR_PORT, SENSOR_LOW) == 1){ //如果高水位监测到有水 换水流程结束
					ChangWatProcess = 1; //换水流程标志位置1
					MENU = 1; //跳转到主界面
					GPIO_WriteBit(MOS_SENSOR_PORT,MOS1,(BitAction)(0));//关闭缸内的抽水机
					GPIO_WriteBit(MOS_SENSOR_PORT,MOS2,(BitAction)(0));//关闭新水的抽水机
				}
			}
			if(KEY != 0){ //按键中断标志位不为0,则表示有按键按下	
				if(KEY == 4){ //按键按下
					KEY = 0; //清0标志位
					MENU = 1; //跳转到主界面
				}
			}		
		}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,自动换水设置
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,设置界面
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...判断触发条件
		if(MENU < 70){//菜单小于70,即不在设置界面时才会触发
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...水温监测判断部分 通过继电器开启或关闭加热棒
			if(WatTemMod == 1){	//当监测温度模式值为0,表示开启监测水温时
				if(Temp /10 <= MinWatTemp && R1Flag != 1){//当前温度低于温度下限时
					RELAY_1(1); //打开加热棒
					R1Flag = 1;
				}
				if(Temp /10 >= MaxWatTemp && R1Flag != 2){//当前温度高于温度上限时
					RELAY_1(0); //关闭加热棒
					R1Flag = 2;
				}			
			}
			if(WatTemMod == 0){//当监测温度模式值为0,表示关闭监测水温时
				if(R1Flag != 0){
					RELAY_1(0); //关闭加热棒
					R1Flag = 0; 
				}
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,水温监测判断部分 通过继电器开启或关闭加热棒
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...智能灯光判断部分 通过mos管1开启或关闭或调节灯光
			if(LightMod == 1){//智能灯光模式是1即按时间开启模式时  大概思路: 00:00--关闭--08:00--白天--17:00--傍晚--21:00--关闭--23:59
				if(rhour == DayTimeStartHour && rmin < DayTimeStartMin && Mos1Flag != 0){//当前时间小于白天区间开始时间时 
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,0); //PWM清0表示关灯
					Mos1Flag = 0;
				}else if(rhour < DayTimeStartHour && Mos1Flag != 0){//如果当前小时小于白天区间开始小时直接关闭鱼缸灯光
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,0); //PWM清0表示关灯
					Mos1Flag = 0; 
				}							
				if(rhour == DayTimeStartHour && rmin >= DayTimeStartMin && Mos1Flag != 1){//当前时间等于白天区间开始时间时
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,DayTimeGear); //按照白天区间灯光档位值开启PWM调节鱼缸灯亮度
					Mos1Flag = 1;
				}else if(rhour > DayTimeStartHour && rhour <= DayTimeEndHour && Mos1Flag != 1){//如果当前小时大于白天区间开始小时则不用判断分钟值直接开启灯光
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,DayTimeGear); //按照白天区间灯光档位值开启PWM调节鱼缸灯亮度
					Mos1Flag = 1;
				}	
				if(rhour == DayTimeEndHour && rmin >= DayTimeEndMin && Mos1Flag != 2){//当前时间等于白天区间结束时间时
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,DuskTimeGear); //按照傍晚区间灯光档位值开启PWM调节鱼缸灯亮度
					Mos1Flag = 2;
				}else if(rhour > DayTimeEndHour && rhour <= DuskTimeEndHour && Mos1Flag != 2){//如果当前小时大于白天区间结束小时并且小于傍晚区间结束小时则不用判断分钟值直接开启灯光
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,DuskTimeGear); //按照傍晚区间灯光档位值开启PWM调节鱼缸灯亮度
					Mos1Flag = 2;
				}	
				if(rhour == DuskTimeEndHour && rmin >= DuskTimeEndMin && Mos1Flag != 3){//当前时间等于傍晚区间结束时间时
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,0); //PWM清0表示关灯
					Mos1Flag = 3;
				}else if(rhour > DuskTimeEndHour && Mos1Flag != 3){//当前小时大于傍晚区间结束小时则不用判断分钟值直接关闭灯光
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,0); //按照傍晚区间灯光档位值开启PWM调节鱼缸灯亮度
					Mos1Flag = 3;
				}
			}
			if(LightMod == 2){//当模式值为2,表示按照环境亮度临界值打开鱼缸灯光模式时
				if(EquaADC <= LuxSwitch && Mos1Flag != 1){//当前环境亮度值<=设定的亮度值时
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,100); //开灯
					Mos1Flag = 1;
				}
				if(EquaADC > (LuxSwitch+300) && Mos1Flag != 2){//当前环境亮度值>设定的亮度值再偏移300时(目的是不让亮度值在临界点跳动时mos管也跟着快速开关)
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,0); //关灯
					Mos1Flag = 2;
				}									
			}
			if(LightMod == 0){//当模式值为0,表示关闭智能灯光模式时
				if(Mos1Flag != 0){
					TIM3_PWM_CH4_Init(100,63); //Time3通道4初始化,
					TIM_SetCompare4(TIM3,0); //PWM清0表示不开灯
					Mos1Flag = 0; 
				}			
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...智能灯光判断部分 通过mos管1开启或关闭或调节灯光
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...过滤水泵判断部分 通过R2继电器2控制水泵开启或关闭
			if(FiltPumpMod == 1){	//当过滤水泵模式值为1,表示开启过滤水泵时 大概流程: 00:00--关闭--08:00--开启--21:00--关闭--23:59
				if(rhour == FiltPumpStartHour && rmin < FiltPumpStartMin && R2Flag != 0){//当前时间小于过滤水泵开启区间开始时间时 
					RELAY_2(0); //关闭过滤水泵
					R2Flag = 0;
				}else if(rhour < FiltPumpStartHour && R2Flag != 0){//如果当前小时小于开始小时直接关闭过滤水泵
					RELAY_2(0); //关闭过滤水泵
					R2Flag = 0; 
				}							
				if(rhour == FiltPumpStartHour && rmin >= FiltPumpStartMin && R2Flag != 1){//当前时间等于过滤水泵开启区间开始时间时
					RELAY_2(1); //开启过滤水泵
					R2Flag = 1;
				}else if(rhour > FiltPumpStartHour && rhour < FiltPumpEndHour && R2Flag != 1){//如果当前小时大于开始小时则不用判断分钟值直接开启灯光
					RELAY_2(1); //开启过滤水泵
					R2Flag = 1;
				}	
				if(rhour == FiltPumpEndHour && rmin >= FiltPumpEndMin && R2Flag != 2){//当前时间等于过滤水泵开启区间结束时间时
					RELAY_2(0); //关闭过滤水泵
					R2Flag = 2;
				}else if(rhour > FiltPumpEndHour && R2Flag != 2){//当前小时大于过滤水泵开启区间结束小时则不用判断分钟值直接关闭过滤水泵
					RELAY_2(0); //关闭过滤水泵
					R2Flag = 2;
				}
			}
			if(FiltPumpMod == 0){//当过滤水泵模式值为0,表示关闭过滤水泵时
				if(R2Flag != 0){
					RELAY_2(0); //关闭过滤水泵
					R2Flag = 0; 
				}
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,过滤水泵判断部分 通过R2继电器2控制水泵开启或关闭
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....自动投喂判断部分, 通过舵机模拟
			if(AutoFeedMod == 1){//当自动投喂模式值为1,表示开启自动投喂时
				for(i=1; i<6; i++){ //判断5组投喂时间是否符合触发条件
					if(FeedTimeHour[i] != 24 && Steer1Flag == 0){ //投喂时间组的小时值不等于off关闭该组标志位24
						if(rhour == FeedTimeHour[i] && rmin == FeedTimeMin[i]){//当前时间等于5个投喂时间组任一时间时
							MENU = 149; //跳转到显示投喂进行时菜单
							FeedTriggerNum = i;	//触发的数组值存放
							FeedLastCount = FeedTimeLast[FeedTriggerNum]; //用于显示倒计时
							Steer1Flag = 1; //舵机状态标志位置1
							OLED_DISPLAY_CLEAR();//清屏操作
						}
					}
				}				
				if(rmin != FeedTimeMin[FeedTriggerNum] && Steer1Flag == 1){ //当前分钟不等于触发的分钟值并且舵机是启动状态1
					Steer1Flag = 0; //舵机状态标志位清零	
				}
			}			
			if(AutoFeedMod == 0 && Steer1Flag != 0){//当自动投喂模式值为0,表示关闭自动投喂时
				Steer1Flag = 0;
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,自动投喂判断部分, 通过舵机模拟
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.....自动换水判断部分,
			if(ChangeWaterMod == 1){//当模式值为1,表示开启自动换水时
				for(i=1; i<6; i++){ //判断5组换水时间是否符合触发条件
					if(ChangWatWeek[i] != 7 && ChangWatFlag != 1){ //换水时间组的星期值不等于off关闭该组标志位 7
						if(rweek == ChangWatWeek[i] && rhour == ChangWatTimHour[i] && rmin == ChangWatTimMin[i]){//当前时间等于5个换水时间组任一时间时
							MENU = 159; //跳转到显示投喂进行时菜单
							ChangWatTrigNum = i; //记下触发的时间组值
							ChangWatFlag = 1; //换水状态标志位置1
							ChangWatProcess	= 1; //换水流程标志位置1
							OLED_DISPLAY_CLEAR();//清屏操作
						}
					}
				}				
				if(rmin != ChangWatTimMin[ChangWatTrigNum] && ChangWatFlag != 0){ //当前分钟不等于触发的换水分钟值
					ChangWatFlag = 0; //换水状态标志位清零	
				}
			}			
			if(ChangeWaterMod == 0 && ChangWatFlag != 0){//当模式值为0,表示关闭自动换水时
				ChangWatFlag = 0;
			}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,,,,自动换水判断部分,
		}//if(MENU < 70)
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...判断触发条件
	}//主循环
}//主函数

/*********************************************************************************************

【变量定义】
u32     a; //定义32位无符号变量a
u16     a; //定义16位无符号变量a
u8     a; //定义8位无符号变量a
vu32     a; //定义易变的32位无符号变量a
vu16     a; //定义易变的 16位无符号变量a
vu8     a; //定义易变的 8位无符号变量a
uc32     a; //定义只读的32位无符号变量a
uc16     a; //定义只读 的16位无符号变量a
uc8     a; //定义只读 的8位无符号变量a

delay_us(1); //延时1微秒
delay_ms(1); //延时1毫秒
delay_s(1); //延时1秒

*/



