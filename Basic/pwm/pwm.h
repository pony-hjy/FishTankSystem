#ifndef  __PWM_H
#define  __PWM_H
#include "sys.h"

void TIM3_PWM_CH4_Init(u16 arr,u16 psc);  //TIM3通道4 PWM初始化 arr重装载值 psc预分频系数
void TIM3_PWM_CH3_Init(u16 arr,u16 psc);  //TIM3通道3 PWM初始化 arr重装载值 psc预分频系数

#endif
