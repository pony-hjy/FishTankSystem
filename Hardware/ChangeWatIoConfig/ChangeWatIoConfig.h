#ifndef __ChangeWatIoConfig_H
#define __ChangeWatIoConfig_H	 
#include "sys.h"

#define MOS_SENSOR_PORT	GPIOB	//����IO�ӿ���
#define MOS1			GPIO_Pin_9	//����IO�ӿ�
#define MOS2			GPIO_Pin_10	//����IO�ӿ�
#define SENSOR_HIGH		GPIO_Pin_11	//����IO�ӿ�
#define SENSOR_LOW		GPIO_Pin_12	//����IO�ӿ�


void ChangeWatIoConfig(void);//�Զ���ˮ�������IO�ڳ�ʼ��

		 				    
#endif
