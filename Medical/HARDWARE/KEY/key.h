#ifndef _KEY_H
#define _KEY_H
#include "sys.h"

#define WK_UP       PAin(0) //WKUP����PA0
#define WKUP_PRES   2       //WK_UP ����ʱ�ķ���ֵ

//���ð���ɨ��ķ�����¼���������ʵ����������ֵ
#define Motor3_Tim      PGin(14)    //3�ŵ��
#define Motor3_Alm      PGin(13)

#define Motor4_Tim      PIin(4)     //5�ŵ��
#define Motor4_Alm      PGin(11)
#define Motor5_Tim      PIin(1)     //4�ŵ��
#define Motor5_Alm      PCin(5)

#define Motor6_Tim      PHin(14)    //6�ŵ��
#define Motor6_Alm      PHin(15)
#define Motor7_Tim      PGin(12)    //7�ŵ��


//#define Motor7_Alm      PAin(12)

//���ð���ɨ��ķ�����¼���������ʵ����������ֵ
#define Motor3_Tim_PRES 3
#define Motor3_Alm_PRES 4
#define Motor4_Tim_PRES 5
#define Motor4_Alm_PRES 6
#define Motor5_Tim_PRES 7
#define Motor5_Alm_PRES 8
#define Motor6_Tim_PRES 9
#define Motor6_Alm_PRES 10
#define Motor7_Tim_PRES 11
#define Motor7_Alm_PRES 12

#define M3TIM 1
#define M3ALM 2
#define M4TIM 3
#define M4ALM 4
#define M5TIM 5
#define M5ALM 6
#define M6TIM 7
#define M6ALM 8
#define M7TIM 9
#define M7ALM 10


void KEY_Init(void);    //����IO�ڳ�ʼ������
u8 KEY_Scan(u8 mode);   //����������

u8 KeyCheck(u8 num);
void KeyCheckAll(void);

#endif
