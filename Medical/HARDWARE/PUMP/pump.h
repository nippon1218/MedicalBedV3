#ifndef __PUMP
#define __PUMP
#include "sys.h"

/****************ˮ��***********************/

#define DIR_SB      PBout(11)          //ˮ��PB12

//#define DIR_SB     PEout(3)           //ˮ��PB12,ԭ������

#define DIR_QB     PHout(2)           //����
#define DIR_HG     PBout(10)          //���
#define DIR_XZFPQ   PEout(3)          //��ת������
//#define DIR_JRAM   PBout(11)          //��Ħ
#define DIR_JR     PHout(3)           //����-�̵���
#define DIR_WD     PHout(8)           //�¶ȴ�����

void Pump_Init(void);                 //ˮ��IO�ڳ�ʼ��

/************��ϴ����Ƹ�********************/

#define DIR_CXHG      PCout(1)          //��ϴ����Ƹ˷����
#define PWM_CXHG      PBout(13)         //��ϴ������巽���

void Push_Rod_Swash_Dry_Init(void);     //��ϴ��ɵ綯��
void Push_Rod_Swash_Dry(u8 dir,u32 n);  //��ϴ��ɵ綯��
void Push_Rod_Swash_Dry1(u8 dir,u32 n); //��ϴ��ɵ綯��

void Push_Rod_Swash(u8 dir,u32 n);      //��ϴ�綯��
void Push_Rod_Dry(u8 dir,u32 n);        //��ɵ綯��

//���ں���
void Uart_Push_Rod_Swash_Dry(u8 dir,u32 n);  //��ϴ��ɵ綯��
void Uart_Push_Rod_Swash(u8 dir,u32 n);      //��ϴ�綯��
void Uart_Push_Rod_Dry(u8 dir,u32 n);        //��ɵ綯��

/***************��翪��********************/

#define GD3_Start       PEin(6)       //3�ŵ����翪��
#define GD3_Left_End    PBin(15)      //����ֹ
#define GD3_Right_End   PBin(14)      //�ҷ���ֹ

#define GD4_Start       PEin(5)       //4�ŵ����翪��
#define GD4_Left_End    PEin(4)       //����ֹ
#define GD4_Right_End   PHin(6)       //�ҷ���ֹ

#define GD5_Start       PEin(2)       //5�ŵ����翪��
#define GD5_Left_End    PCin(13)      //����ֹ
#define GD5_Right_End   PHin(9)   	  //�ҷ���ֹ

#define GD6_Start       PCin(7)       //6�ŵ����翪��
#define GD6_End   		PCin(6) 
#define GD7_Start       PCin(8)       //7�ŵ����翪��
#define GD7_End       	PCin(9) 

#define Liq_Sensor      PHin(7)       //Һλ������

#define GD3S 1
#define GD3LE 2
#define GD3RE 3

#define GD4S	4
#define GD4LE 5
#define GD4RE 6

#define GD5S 7
#define GD5LE 8
#define GD5RE	9

#define GD6S 10
#define GD6E 11

#define GD7S 12
#define GD7E 13

#define GD34S	14
#define GD34LE	15
#define GD34RE	16

void Sensor_Init(void);               //��缰��������ʼ��

#endif


