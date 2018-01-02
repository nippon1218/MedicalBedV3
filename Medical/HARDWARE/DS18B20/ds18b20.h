#ifndef __DS18B20_H
#define __DS18B20_H
#include "sys.h"

//IO��������
#define DS18B20_IO_IN()  {GPIOH->MODER&=~(3<<(8*2));GPIOH->MODER|=0<<8*2;} //PH8����ģʽ
#define DS18B20_IO_OUT() {GPIOH->MODER&=~(3<<(8*2));GPIOH->MODER|=1<<8*2;} //PH8���ģʽ
 
////IO��������											   
#define	DS18B20_DQ_OUT PHout(8)//���ݶ˿�	PH8
#define	DS18B20_DQ_IN  PHin(8) //���ݶ˿�	PH8 
   	
u8 DS18B20_Init(void);			//��ʼ��DS18B20
short DS18B20_Get_Temp(void);	//��ȡ�¶�
void DS18B20_Start(void);		//��ʼ�¶�ת��
void DS18B20_Write_Byte(u8 dat);//д��һ���ֽ�
u8 DS18B20_Read_Byte(void);		//����һ���ֽ�
u8 DS18B20_Read_Bit(void);		//����һ��λ
u8 DS18B20_Check(void);			//����Ƿ����DS18B20
void DS18B20_Rst(void);			//��λDS18B20 
#endif
