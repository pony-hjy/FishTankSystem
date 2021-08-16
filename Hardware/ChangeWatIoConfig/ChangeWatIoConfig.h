#ifndef __ChangeWatIoConfig_H
#define __ChangeWatIoConfig_H	 
#include "sys.h"

#define MOS_SENSOR_PORT	GPIOB	//定义IO接口组
#define MOS1			GPIO_Pin_9	//定义IO接口
#define MOS2			GPIO_Pin_10	//定义IO接口
#define SENSOR_HIGH		GPIO_Pin_11	//定义IO接口
#define SENSOR_LOW		GPIO_Pin_12	//定义IO接口


void ChangeWatIoConfig(void);//自动换水部件相关IO口初始化

		 				    
#endif
