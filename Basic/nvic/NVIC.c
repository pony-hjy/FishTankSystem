
/*
//杜洋工作室出品
//洋桃系列开发板应用程序
//关注微信公众号：洋桃电子
//洋桃开发板资料下载 www.DoYoung.net/YT 
//即可免费看所有教学视频，下载技术资料，技术疑难提问
//更多内容尽在 杜洋工作室主页 www.doyoung.net
*/

/*
《修改日志》
1-201708202312 创建。


*/
#include "NVIC.h"
#include "delay.h"
#include "touch_key.h"

u8 KEY;//中断标志位
#define KEYA_SPEED1 100 //判定长按时间长度

void EXti_Config(void)
{
	//定义结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	//把NVIC中断优先级分组设置为第0组,即不会有嵌套中断发生
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);//开启PA组功能时钟,开启APB2总线复用功能时钟
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...按键A的中断初始化
	//io口模式初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ; //给io口赋初始化模式值，
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//调用库函数进行初始化
	//中断触发模式设置	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); //选择中断线 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;	//给中断向量线赋初始化模式值，	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//设置中断触发模式，通过这个来不断查询相应的寄存器
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//设置触发模式为下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//使能	
	EXTI_Init(&EXTI_InitStructure);	//调用库函数
	//优先级设置	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //中断线对应的中断向量
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//设置抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//设置响应优先级为0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能	
	NVIC_Init(&NVIC_InitStructure);	//调用库函数
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...按键A的中断初始化
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...按键B的中断初始化
	//io口模式初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; //给io口赋初始化模式值，
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//调用库函数进行初始化
	//中断触发模式设置	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); //选择中断线 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;	//给中断向量线赋初始化模式值，	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//设置中断触发模式，通过这个来不断查询相应的寄存器
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//设置触发模式为下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//使能	
	EXTI_Init(&EXTI_InitStructure);	//调用库函数
	//优先级设置	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; //中断线对应的中断向量
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//设置抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//设置响应优先级为0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能	
	NVIC_Init(&NVIC_InitStructure);	//调用库函数
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...按键B的中断初始化
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...按键C的中断初始化
	//io口模式初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //给io口赋初始化模式值，
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//调用库函数进行初始化
	//中断触发模式设置	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2); //选择中断线 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;	//给中断向量线赋初始化模式值，	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//设置中断触发模式，通过这个来不断查询相应的寄存器
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//设置触发模式为下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//使能	
	EXTI_Init(&EXTI_InitStructure);	//调用库函数
	//优先级设置	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; //中断线对应的中断向量
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//设置抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//设置响应优先级为0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能	
	NVIC_Init(&NVIC_InitStructure);	//调用库函数
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...按键C的中断初始化
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...按键D的中断初始化
	//io口模式初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; //给io口赋初始化模式值，
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//调用库函数进行初始化
	//中断触发模式设置	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3); //选择中断线 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;	//给中断向量线赋初始化模式值，	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//设置中断触发模式，通过这个来不断查询相应的寄存器
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//设置触发模式为下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//使能	
	EXTI_Init(&EXTI_InitStructure);	//调用库函数
	//优先级设置	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; //中断线对应的中断向量
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//设置抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//设置响应优先级为0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能	
	NVIC_Init(&NVIC_InitStructure);	//调用库函数
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...按键D的中断初始化

}

void EXTI0_IRQHandler(void){  //中断0处理函数
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){ //确保是否是产生了中断
		KEY = 1;//按键值
		EXTI_ClearITPendingBit(EXTI_Line0); //清除中断标志位
	}	
}
void EXTI1_IRQHandler(void){  //中断1处理函数
	u8 c=0;
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){ //确保是否是产生了中断
		if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){//判断长短按
			while((!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)) && c<KEYA_SPEED1){ //c一直加1到条件不成立,即大于KEYA_SPEED1
				c++; Counter_delay_ms(10); //长按判断的计时
			}
			if(c >= KEYA_SPEED1){ //c的计数值可以判断出是否长按
				//执行长按程序
				KEY = 12; //按键值		
			}else{
			 //执行短按		
				KEY = 2; //按键值	
			}
			c=0; //c的计数值清0
		}else{ //实际测试中,有时会出现手离开得很快导致if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B))没有读到
		//执行短按
			KEY = 2; //按键值		
		}//按键判断在此结束
		EXTI_ClearITPendingBit(EXTI_Line1); //清除中断标志位
	}	
}
void EXTI2_IRQHandler(void){  //中断2处理函数
	u8 c=0;
	if(EXTI_GetITStatus(EXTI_Line2) != RESET){ //确保是否是产生了中断
		if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){//判断长短按
			while((!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)) && c<KEYA_SPEED1){ //c一直加1到条件不成立,即大于KEYA_SPEED1
				c++; Counter_delay_ms(10); //长按判断的计时
			}
			if(c >= KEYA_SPEED1){ //c的计数值可以判断出是否长按
				//执行长按程序
				KEY = 13; //按键值		
			}else{
			 //执行短按		
				KEY = 3; //按键值	
			}
			c=0; //c的计数值清0
		}else{ //实际测试中,有时会出现手离开得很快导致if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C))没有读到
		//执行短按
			KEY = 3; //按键值		
		}//按键判断在此结束
		EXTI_ClearITPendingBit(EXTI_Line2); //清除中断标志位
	}	
}
void EXTI3_IRQHandler(void){  //中断3处理函数
	if(EXTI_GetITStatus(EXTI_Line3) != RESET){ //确保是否是产生了中断
		KEY = 4; //按键值
		EXTI_ClearITPendingBit(EXTI_Line3); //清除中断标志位
	}	
}


