#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define RXBUFFERSIZE            1       //�����С


extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern UART_HandleTypeDef UART1_Handler; //UART1���
extern UART_HandleTypeDef UART2_Handler; //UART2���
extern u8 aRxBuffer[RXBUFFERSIZE];
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);

//����2��غ궨��
#define USART2_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define USART2_MAX_SEND_LEN		800					//����ͻ����ֽ���
#define USART2_RX_EN 			1					//0,������;1,����.


//����3��غ궨��
#define USART3_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		800					//����ͻ����ֽ���
#define USART3_RX_EN 			1					//0,������;1,����.

//����4��غ궨��
#define UART4_MAX_RECV_LEN		2048				//�����ջ����ֽ���
#define UART4_MAX_SEND_LEN		2048				//����ͻ����ֽ���
#define UART4_RX_EN 			1					//0,������;1,����.

//����6��غ궨��
#define USART6_MAX_RECV_LEN		600					//�����ջ����ֽ���
#define USART6_MAX_SEND_LEN		600					//����ͻ����ֽ���
#define USART6_RX_EN 			1					//0,������;1,����.


extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern u16 USART3_RX_LEN;   						//��������״̬

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		//���ջ���,���USART2_MAX_RECV_LEN�ֽ�
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
extern u16 USART2_RX_LEN;   						//��������״̬

extern u8  UART4_RX_BUF[UART4_MAX_RECV_LEN]; 		//���ջ���,���UART4_MAX_RECV_LEN�ֽ�
extern u8  UART4_TX_BUF[UART4_MAX_SEND_LEN]; 		//���ͻ���,���UART4_MAX_SEND_LEN�ֽ�
extern u16 UART4_RX_LEN;   							//��������״̬

extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8  USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern u16 USART6_RX_LEN;   						//��������״̬


void usart6_init(u32 bound);				//����6��ʼ��
void uart4_init(u32 bound);					//����4��ʼ��
void usart3_init(u32 bound);				//����3��ʼ�� 
void usart2_init(u32 bound);				//����2��ʼ�� 
void TIM7_Init(u16 arr,u16 psc);            //��ʱ��7��ʼ������

void u6_printf(char* fmt, ...);             //����6���ͺ���
void u4_printf(char* fmt, ...);             //����4���ͺ���
void u3_printf(char* fmt, ...);             //����3���ͺ���
void u2_printf(char* fmt, ...);             //����2���ͺ���


int long usmart_pow1(u8 m,u8 n);             //��m^n�η�����
int long usmart_strnum(u8*str);              //���ַ���תΪ���ֺ���

u32 usmart_strnum2(u8*str);

u16 abs(u16 data1,u16 data2);                //�����ֵ����
#endif

