


#include "ChangeWatIoConfig.h"

void ChangeWatIoConfig(void){ //微动开关的接口初始化
	GPIO_InitTypeDef  GPIO_InitStructure; //定义GPIO的初始化枚举结构	
    GPIO_InitStructure.GPIO_Pin = MOS1 | MOS2; //选择端口                       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //选择IO接口工作方式 //上拉电阻       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //设置IO接口速度（2/10/50MHz）    
	GPIO_Init(MOS_SENSOR_PORT,&GPIO_InitStructure);
 
    GPIO_InitStructure.GPIO_Pin = SENSOR_HIGH | SENSOR_LOW; //选择端口                       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //选择IO接口工作方式 //上拉电阻       
	GPIO_Init(MOS_SENSOR_PORT,&GPIO_InitStructure);
				
}
 

/*********************************************************************************************
 * 杜洋工作室 www.DoYoung.net
 * 洋桃电子 www.DoYoung.net/YT 
*********************************************************************************************/



/*
选择IO接口工作方式：
GPIO_Mode_AIN 模拟输入
GPIO_Mode_IN_FLOATING 浮空输入
GPIO_Mode_IPD 下拉输入
GPIO_Mode_IPU 上拉输入
GPIO_Mode_Out_PP 推挽输出
GPIO_Mode_Out_OD 开漏输出
GPIO_Mode_AF_PP 复用推挽输出
GPIO_Mode_AF_OD 复用开漏输出
*/
