#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern TIM_HandleTypeDef TIM2_Handler;  //��ʱ��2��� 
extern TIM_HandleTypeDef TIM7_Handler;  //��ʱ��7���
extern TIM_HandleTypeDef TIM9_Handler;  //��ʱ��9���
extern TIM_HandleTypeDef TIM10_Handler; //��ʱ��10��� 


void TIM2_Init(u16 arr,u16 psc);
void TIM7_Init(u16 arr,u16 psc);        //��ʱ��7��ʼ������
void TIM9_Init(u16 arr,u16 psc);        //��ʱ��7��ʼ������
void TIM10_Init(u16 arr,u16 psc);       //��ʱ��10��ʼ������

void TIM10_Stop(void);                  //�رն�ʱ��10
void TIM9_Stop(void);                   //�رն�ʱ��9
void TIM2_Stop(void);                   //�رն�ʱ��2
#endif




