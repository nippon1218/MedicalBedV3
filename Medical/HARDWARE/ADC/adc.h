#ifndef __ADC_H
#define __ADC_H
#include "sys.h"

void MY_ADC_Init(void);                 //��ʼ��ADC
u16 Get_Adc(u32 ch);                    //���ADCֵ
u16 Get_Adc_Average(u32 ch,u8 times);   //��ȡָ��ͨ����ת��ֵ

#endif