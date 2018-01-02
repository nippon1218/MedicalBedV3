#ifndef __HX711_H__
#define __HX711_H__
#include "sys.h"

//����ͨѶ���ɹܽ�PD_SCK��DOUT��ɣ�����������ݣ�ѡ������ͨ��������

/****************������ʱ�ӿ����ú���***********************/

#define HX711_SCK_INPUT()   {GPIOB->MODER &=~(3<<(5*2));GPIOB->MODER |=0<<(5*2);}	  //PB5����ģʽ
#define HX711_SCK_OUTPUT()  {GPIOB->MODER &=~(3<<(5*2));GPIOB->MODER |=1<<(5*2);}     //PB5���ģʽ

/****************���������ݿ����ú���***********************/

#define HX711_DOUT_INPUT()  {GPIOH->MODER &=~(3<<(10*2));GPIOH->MODER |=0<<(10*2);}	  //PH10����ģʽ
#define HX711_DOUT_OUTPUT() {GPIOH->MODER &=~(3<<(10*2));GPIOH->MODER |=1<<(10*2);}   //PH10���ģʽ
 
/*********************IO��������****************************/

#define	HX711_SCK_OUT PBout(5)         //SCK�˿����    PB5
#define	HX711_SCK_IN  PBin(5)          //SCK�˿�����    PB5 

#define	HX711_DOUT_OUT PHout(10)       //DOUT�˿����	PH10
#define	HX711_DOUT_IN  PHin(10)        //DOUT�˿�����	PH10

unsigned long HX711_Read(void);        //�������ɼ����ݺ���
unsigned long Get_Weight(void);        //���غ���
unsigned long filter(void);            //���ݴ�����
void HX711_Init(void);                 //��ʼ��HX711��IO�ں���

#endif















