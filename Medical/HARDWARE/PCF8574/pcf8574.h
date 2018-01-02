#ifndef __PCF8574_H
#define __PCF8574_H
#include "sys.h"
#include "myiic.h"

#define PCF8574_INT  PBin(12)   //PCF8574 INT��
#define PCF8574_ADDR 	0X40	//PCF8574��ַ(������һλ)

/*******************PCF8574����IO�Ĺ���******************/

#define BEEP_IO           0		//��������������  	    P0
//#define AP_INT_IO       1   	//AP3216C�ж�����	    P1
//#define DCMI_PWDN_IO    2    	//DCMI�ĵ�Դ��������	    P2
#define USB_PWR_IO        3    	//USB��Դ��������	    P3
//#define EX_IO      	  4    	//��չIO,�Զ���ʹ��   	P4
//#define MPU_INT_IO      5   	//MPU9250�ж�����	    P5
#define RS485_RE_IO       6    	//RS485_RE����		    P6
#define ETH_RESET_IO      7    	//��̫����λ����		    P7

#define EXIO1       1   	//AP3216C�ж�����	    P1
#define EXIO2       2    	//DCMI�ĵ�Դ��������	    P2
#define EXIO3      	4    	//��չIO,�Զ���ʹ��   	P4
#define EXIO4       5   	//MPU9250�ж�����	    P5


u8 PCF8574_Init(void);                       //PCF8574��ʼ������
u8 PCF8574_ReadOneByte(void);                //��ȡPCF8574��8λIOֵ
void PCF8574_WriteOneByte(u8 DataToWrite);   //��PCF8574д��8λIOֵ 
void PCF8574_WriteBit(u8 bit,u8 sta);        //����PCF8574ĳ��IO�ĸߵ͵�ƽ
u8 PCF8574_ReadBit(u8 bit);                  //��ȡPCF8574��ĳ��IO��ֵ 

void BeepRun(u8 num,u16 time);


#endif

