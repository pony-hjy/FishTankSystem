
/*
//�������ҳ�Ʒ
//����ϵ�п�����Ӧ�ó���
//��ע΢�Ź��ںţ����ҵ���
//���ҿ������������� www.DoYoung.net/YT 
//������ѿ����н�ѧ��Ƶ�����ؼ������ϣ�������������
//�������ݾ��� ����������ҳ www.doyoung.net
*/

/*
���޸���־��
1-201708202312 ������


*/
#include "NVIC.h"
#include "delay.h"
#include "touch_key.h"

u8 KEY;//�жϱ�־λ
#define KEYA_SPEED1 100 //�ж�����ʱ�䳤��

void EXti_Config(void)
{
	//����ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	//��NVIC�ж����ȼ���������Ϊ��0��,��������Ƕ���жϷ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);//����PA�鹦��ʱ��,����APB2���߸��ù���ʱ��
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...����A���жϳ�ʼ��
	//io��ģʽ��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ; //��io�ڸ���ʼ��ģʽֵ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//���ÿ⺯�����г�ʼ��
	//�жϴ���ģʽ����	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); //ѡ���ж��� 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;	//���ж������߸���ʼ��ģʽֵ��	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//�����жϴ���ģʽ��ͨ����������ϲ�ѯ��Ӧ�ļĴ���
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//���ô���ģʽΪ�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//ʹ��	
	EXTI_Init(&EXTI_InitStructure);	//���ÿ⺯��
	//���ȼ�����	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //�ж��߶�Ӧ���ж�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//������ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//������Ӧ���ȼ�Ϊ0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ��	
	NVIC_Init(&NVIC_InitStructure);	//���ÿ⺯��
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...����A���жϳ�ʼ��
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...����B���жϳ�ʼ��
	//io��ģʽ��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; //��io�ڸ���ʼ��ģʽֵ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//���ÿ⺯�����г�ʼ��
	//�жϴ���ģʽ����	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); //ѡ���ж��� 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;	//���ж������߸���ʼ��ģʽֵ��	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//�����жϴ���ģʽ��ͨ����������ϲ�ѯ��Ӧ�ļĴ���
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//���ô���ģʽΪ�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//ʹ��	
	EXTI_Init(&EXTI_InitStructure);	//���ÿ⺯��
	//���ȼ�����	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; //�ж��߶�Ӧ���ж�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//������ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//������Ӧ���ȼ�Ϊ0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ��	
	NVIC_Init(&NVIC_InitStructure);	//���ÿ⺯��
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...����B���жϳ�ʼ��
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...����C���жϳ�ʼ��
	//io��ģʽ��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //��io�ڸ���ʼ��ģʽֵ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//���ÿ⺯�����г�ʼ��
	//�жϴ���ģʽ����	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2); //ѡ���ж��� 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;	//���ж������߸���ʼ��ģʽֵ��	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//�����жϴ���ģʽ��ͨ����������ϲ�ѯ��Ӧ�ļĴ���
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//���ô���ģʽΪ�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//ʹ��	
	EXTI_Init(&EXTI_InitStructure);	//���ÿ⺯��
	//���ȼ�����	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; //�ж��߶�Ӧ���ж�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//������ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//������Ӧ���ȼ�Ϊ0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ��	
	NVIC_Init(&NVIC_InitStructure);	//���ÿ⺯��
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...����C���жϳ�ʼ��
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...����D���жϳ�ʼ��
	//io��ģʽ��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; //��io�ڸ���ʼ��ģʽֵ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//���ÿ⺯�����г�ʼ��
	//�жϴ���ģʽ����	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3); //ѡ���ж��� 	
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;	//���ж������߸���ʼ��ģʽֵ��	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//�����жϴ���ģʽ��ͨ����������ϲ�ѯ��Ӧ�ļĴ���
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 	//���ô���ģʽΪ�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 	//ʹ��	
	EXTI_Init(&EXTI_InitStructure);	//���ÿ⺯��
	//���ȼ�����	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; //�ж��߶�Ӧ���ж�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	//������ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 	//������Ӧ���ȼ�Ϊ0	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ��	
	NVIC_Init(&NVIC_InitStructure);	//���ÿ⺯��
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<...����D���жϳ�ʼ��

}

void EXTI0_IRQHandler(void){  //�ж�0������
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){ //ȷ���Ƿ��ǲ������ж�
		KEY = 1;//����ֵ
		EXTI_ClearITPendingBit(EXTI_Line0); //����жϱ�־λ
	}	
}
void EXTI1_IRQHandler(void){  //�ж�1������
	u8 c=0;
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){ //ȷ���Ƿ��ǲ������ж�
		if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)){//�жϳ��̰�
			while((!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B)) && c<KEYA_SPEED1){ //cһֱ��1������������,������KEYA_SPEED1
				c++; Counter_delay_ms(10); //�����жϵļ�ʱ
			}
			if(c >= KEYA_SPEED1){ //c�ļ���ֵ�����жϳ��Ƿ񳤰�
				//ִ�г�������
				KEY = 12; //����ֵ		
			}else{
			 //ִ�ж̰�		
				KEY = 2; //����ֵ	
			}
			c=0; //c�ļ���ֵ��0
		}else{ //ʵ�ʲ�����,��ʱ��������뿪�úܿ쵼��if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_B))û�ж���
		//ִ�ж̰�
			KEY = 2; //����ֵ		
		}//�����ж��ڴ˽���
		EXTI_ClearITPendingBit(EXTI_Line1); //����жϱ�־λ
	}	
}
void EXTI2_IRQHandler(void){  //�ж�2������
	u8 c=0;
	if(EXTI_GetITStatus(EXTI_Line2) != RESET){ //ȷ���Ƿ��ǲ������ж�
		if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)){//�жϳ��̰�
			while((!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C)) && c<KEYA_SPEED1){ //cһֱ��1������������,������KEYA_SPEED1
				c++; Counter_delay_ms(10); //�����жϵļ�ʱ
			}
			if(c >= KEYA_SPEED1){ //c�ļ���ֵ�����жϳ��Ƿ񳤰�
				//ִ�г�������
				KEY = 13; //����ֵ		
			}else{
			 //ִ�ж̰�		
				KEY = 3; //����ֵ	
			}
			c=0; //c�ļ���ֵ��0
		}else{ //ʵ�ʲ�����,��ʱ��������뿪�úܿ쵼��if(!GPIO_ReadInputDataBit(TOUCH_KEYPORT,TOUCH_KEY_C))û�ж���
		//ִ�ж̰�
			KEY = 3; //����ֵ		
		}//�����ж��ڴ˽���
		EXTI_ClearITPendingBit(EXTI_Line2); //����жϱ�־λ
	}	
}
void EXTI3_IRQHandler(void){  //�ж�3������
	if(EXTI_GetITStatus(EXTI_Line3) != RESET){ //ȷ���Ƿ��ǲ������ж�
		KEY = 4; //����ֵ
		EXTI_ClearITPendingBit(EXTI_Line3); //����жϱ�־λ
	}	
}


