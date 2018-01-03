#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

/***************���ܺ�����������*********************/

#define DIR3 PDout(7)       //3�ŵ�������ź���     
#define DIR4 PDout(3)       //4�ŵ�������ź���
#define DIR5 PDout(2)       //5�ŵ�������ź��� 

#define DIR6 PCout(12)      //6�ŵ�������ź��� 
#define DIR6_1 PAout(5)     //6������ս���������ź��� 
#define DIR6_2 PHout(13)    //6������ս�����Ƹ������ź��� 
#define PWM6_2 PCout(4)     //6������ս�����Ƹ�����������  

#define RELAY6 PHout(12)    //����������ս����ͨ�ϵļ̵���
#define DIR7 PAout(6)       //7�ŵ�������ź��� 
                            //2�ŵ�������ź���
#define DIR2_DOWN PIout(8)  //�������ź���
#define DIR2_UP PGout(10)   //�������ź���

#define DIR1_DOWN PIout(3)  //֧�������ź���
#define DIR1_UP   PIout(7)  //֧�������ź���


/***************���ҵ�������*********************/

#define HANG_DIR1 PBout(6)   //1�ŵ��ҵ�������ź��� 
#define HANG_DIR2 PBout(7)   //2�ŵ��ҵ�������ź��� 
#define HANG_DIR3 PBout(8)   //3�ŵ��ҵ�������ź��� 
#define HANG_DIR4 PBout(9)   //4�ŵ��ҵ�������ź��� 

/***************���ҵ�������*********************/

#define HANG_PWM1 PHout(11)  //1�ŵ��ҵ�������ź��� 
#define HANG_PWM2 PGout(3)   //2�ŵ��ҵ�������ź���  
#define HANG_PWM3 PDout(13)  //3�ŵ��ҵ�������ź��� 
#define HANG_PWM4 PGout(6)   //4�ŵ��ҵ�������ź��� 

/***************���ҵ�������*********************/
#define DG_Relay				PAout(12)		//���Ҽ̵���
/**************************************************************
                   ���ܺ��������������
                   arr���Զ���װֵ
				   psc��ʱ��Ԥ��Ƶ��
***************************************************************/

void Motor_Dir_Init(void);                     //�������ڳ�ʼ������

void Motor_3_START(u16 arr,u16 psc);           //����������к���
void Motor_4_START(u16 arr,u16 psc);           //����������к���
void Motor_4_Compensate(u8 dir,u16 time_arr,u16 arr,u16 psc);	//��ʱ����������
void Motor_5_START(u16 arr,u16 psc);           //�෭������к���
void Motor_3_4_5_START_left(u16 arr,u16 psc);  //��/�ҷ�������к���
void Motor_3_4_5_START_right(u16 arr,u16 psc); //��/�ҷ�������к���
void Motor_6_START(u16 arr,u16 psc);           //������������к���
void Motor_6_1_START(u16 arr,u16 psc);         //���������������к���
void Motor_6_2_START(u8 dir,u32 pulse);        //����������Ƹ���������
void Motor_7_START(u16 arr,u16 psc);           //С���ӵ�����к���

void Push_Rod_Init(void);                      //�綯�Ƹ˳�ʼ������
void Push_Rod_Start(u8 dir);                   //���ȵ�����к���
void Push_Rod_Stop(void);                      //���ȵ��ֹͣ����

void Motor_1_START(u8 dir);                    //֧��������к���
void Motor_1_STOP(void);                       //֧�����ֹͣ����


void MotorStart(u8 MotorID,u8 dir,u16 arr);

void MotorStop(u8 MotorID);

void Motor345Start(u8 dir,u16 M3Arr,u16 M4Arr,u16 M5Arr);
void Motor345Stop();

void WashletRun(u8 dir,u16 TimArr,u16 TimPsc);
void BodyLeftRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 TimArr);
void BodyRightRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 TimArr);

void DeskRun(u8 dir,u16 HalfDist);	//С���ӳ���
void DeskRun1(u8 dir,u16 Dist);	//С���ӳ���



/***************���ܺ������ֹͣ����*********************/

void Motor_3_STOP(void);                //�������ֹͣ����
void Motor_4_STOP(void);                //����֧�����ֹͣ����
void Motor_5_STOP(void);                //�෭���ֹͣ����
void Motor_6_STOP(void);                //���������ֹͣ����
void Motor_6_1_STOP(void);              //������������ֹͣ����
void Motor_7_STOP(void);                //С���ӵ��ֹͣ����
void Motor_3_4_5_STOP(void);            //��/�ҷ�����ֹͣ����
void Motor_All_Stop(void);              //���е��ֹͣ


/**************************************************************
                   ���ҵ����������
                   dir�����Ƶ��ҵ���ٶ�
				   pulse�����Ƶ��ҵ������ʱ��
***************************************************************/

void Hang_Init(void);                         //���ҵ������ڡ�����ڳ�ʼ��

void Auto_Hang_1(u16 dir,u32 pulse);          //���ҵ��1�ŵ������
void Auto_Hang_3(u16 dir,u32 pulse);          //���ҵ��3�ŵ������
void Auto_Hang_1_2(u16 dir,u32 pulse);        //����1/2�ŵ��ͬʱ����
void Auto_Hang_1_3(u16 dir,u32 pulse);        //����1/3�ŵ��ͬʱ����
void Auto_Hang_3_4(u16 dir,u32 pulse);        //����3/4�ŵ��ͬʱ����
void Auto_Hang_1_2_3_4(u16 dir,u32 pulse);    //����1/2/3/4�ŵ��ͬʱ����

void Hand_Hang_1(u16 dir,u32 pulse);          //���ҵ��1�ŵ������
void Hand_Hang_1_2(u16 dir,u32 pulse);        //����1/2�ŵ��ͬʱ����      

void Hand_Hang_3(u16 dir,u32 pulse);          //���ҵ��3�ŵ������
void Hand_Hang_3_4(u16 dir,u32 pulse);        //����3/4�ŵ��ͬʱ����

void Hand_Hang_1_3(u16 dir,u32 pulse);        //����1/3�ŵ��ͬʱ����
void Hand_Hang_1_2_3_4(u16 dir,u32 pulse);    //����1/2/3/4�ŵ��ͬʱ����


/**************************************************************
                   ���ҵ����������--����
                   dir�����Ƶ��ҵ���ٶ�
				   pulse�����Ƶ��ҵ������ʱ��
***************************************************************/

void Uart_Auto_Hang_1(u16 dir,u32 pulse);          //���ҵ��1�ŵ������
void Uart_Auto_Hang_3(u16 dir,u32 pulse);          //���ҵ��3�ŵ������
void Uart_Auto_Hang_1_2(u16 dir,u32 pulse);        //����1/2�ŵ��ͬʱ����
void Uart_Auto_Hang_1_3(u16 dir,u32 pulse);        //����1/3�ŵ��ͬʱ����
void Uart_Auto_Hang_3_4(u16 dir,u32 pulse);        //����3/4�ŵ��ͬʱ����
void Uart_Auto_Hang_1_2_3_4(u16 dir,u32 pulse);    //����1/2/3/4�ŵ��ͬʱ����

void Uart_Hand_Hang_1(u16 dir,u32 pulse);          //���ҵ��1�ŵ������
void Uart_Hand_Hang_1_2(u16 dir,u32 pulse);        //����1/2�ŵ��ͬʱ����      

void Uart_Hand_Hang_3(u16 dir,u32 pulse);          //���ҵ��3�ŵ������
void Uart_Hand_Hang_3_4(u16 dir,u32 pulse);        //����3/4�ŵ��ͬʱ����

void Uart_Hand_Hang_1_3(u16 dir,u32 pulse);        //����1/3�ŵ��ͬʱ����
void Uart_Hand_Hang_1_2_3_4(u16 dir,u32 pulse);    //����1/2/3/4�ŵ��ͬʱ����

void Uart_Motor_6_2_START(u8 dir,u32 pulse);       //����������Ƹ���������

#endif

