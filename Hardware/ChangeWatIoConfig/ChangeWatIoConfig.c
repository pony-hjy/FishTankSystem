


#include "ChangeWatIoConfig.h"

void ChangeWatIoConfig(void){ //΢�����صĽӿڳ�ʼ��
	GPIO_InitTypeDef  GPIO_InitStructure; //����GPIO�ĳ�ʼ��ö�ٽṹ	
    GPIO_InitStructure.GPIO_Pin = MOS1 | MOS2; //ѡ��˿�                       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //ѡ��IO�ӿڹ�����ʽ //��������       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //����IO�ӿ��ٶȣ�2/10/50MHz��    
	GPIO_Init(MOS_SENSOR_PORT,&GPIO_InitStructure);
 
    GPIO_InitStructure.GPIO_Pin = SENSOR_HIGH | SENSOR_LOW; //ѡ��˿�                       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //ѡ��IO�ӿڹ�����ʽ //��������       
	GPIO_Init(MOS_SENSOR_PORT,&GPIO_InitStructure);
				
}
 

/*********************************************************************************************
 * �������� www.DoYoung.net
 * ���ҵ��� www.DoYoung.net/YT 
*********************************************************************************************/



/*
ѡ��IO�ӿڹ�����ʽ��
GPIO_Mode_AIN ģ������
GPIO_Mode_IN_FLOATING ��������
GPIO_Mode_IPD ��������
GPIO_Mode_IPU ��������
GPIO_Mode_Out_PP �������
GPIO_Mode_Out_OD ��©���
GPIO_Mode_AF_PP �����������
GPIO_Mode_AF_OD ���ÿ�©���
*/
