#include "usart.h"
#include "modbus_master.h"
#include "led.h"
#include "vk3214.h"
 	 
//���ʹ��os,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os ʹ��	  
#endif
  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

//#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

u8 aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���

//���ڷ��ͻ�����
__align(8) u8 USART6_TX_BUF[USART6_MAX_SEND_LEN]; 	//���ͻ���,���USART4_MAX_SEND_LEN�ֽ� 
__align(8) u8 UART4_TX_BUF[UART4_MAX_SEND_LEN]; 	//���ͻ���,���USART4_MAX_SEND_LEN�ֽ� 
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ� 
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ� 

//���ڽ��ջ�����
u8 USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		   	    //���ջ���,���USART4_MAX_RECV_LEN���ֽ�.
u8 UART4_RX_BUF[UART4_MAX_RECV_LEN]; 		   	    //���ջ���,���USART4_MAX_RECV_LEN���ֽ�.
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.


u8 modbus_ready;

//ͨ���жϽ�������2���ַ�֮���ʱ������100ms�������ǲ���һ������������.
//���2���ַ����ռ������100ms,����Ϊ����1����������.Ҳ���ǳ���100msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
u16 USART6_RX_LEN=0; 
u16 UART4_RX_LEN=0; 
u16 USART3_RX_LEN=0; 
u16 USART2_RX_LEN=0;
UART_HandleTypeDef UART1_Handler; //UART���
UART_HandleTypeDef UART2_Handler; //UART���
UART_HandleTypeDef UART3_Handler; //UART���
UART_HandleTypeDef UART4_Handler; //UART���
UART_HandleTypeDef UART6_Handler; //UART���

/***********************************************************************
 ������      ��uart_init()  
 ��������    ����ʼ��IO ����1 
 ����        ��bound:������
 ���        ����
                           
************************************************************************/
void uart_init(u32 bound)
{	
	//UART ��ʼ������
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //������
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()��ʹ��UART1
}

/***********************************************************************
 ������      ��usart2_init()  
 ��������    ����ʼ��IO ����2 
 ����        ��bound:������
 ���        ����
                           
************************************************************************/
void usart2_init(u32 bound)
{	
	//UART ��ʼ������
	UART2_Handler.Instance=USART2;					    //USART2
	UART2_Handler.Init.BaudRate=bound;				    //������
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()��ʹ��UART1
}

/***********************************************************************
 ������      ��usart3_init()  
 ��������    ����ʼ��IO ����3 
 ����        ��bound:������
 ���        ����
                           
************************************************************************/
void usart3_init(u32 bound)
{	
	//UART ��ʼ������
	UART3_Handler.Instance=USART3;					    //USART3
	UART3_Handler.Init.BaudRate=bound;				    //������
	UART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART3_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART3_Handler);					    //HAL_UART_Init()��ʹ��UART1
}

/***********************************************************************
 ������      ��usart4_init()  
 ��������    ����ʼ��IO ����4
 ����        ��bound:������
 ���        ����
                           
************************************************************************/
void uart4_init(u32 bound)
{	
	//UART ��ʼ������
	UART4_Handler.Instance=UART4;					    //USART4
	UART4_Handler.Init.BaudRate=bound;				    //������
	UART4_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART4_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART4_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART4_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART4_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART4_Handler);					    //HAL_UART_Init()��ʹ��UART1
}

/***********************************************************************
 ������      ��usart6_init()  
 ��������    ����ʼ��IO ����6 
 ����        ��bound:������
 ���        ����
                           
************************************************************************/
void usart6_init(u32 bound)
{	
	//UART ��ʼ������
	UART6_Handler.Instance=USART6;					    //USART3
	UART6_Handler.Init.BaudRate=bound;				    //������
	UART6_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART6_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART6_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART6_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART6_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART6_Handler);					    //HAL_UART_Init()��ʹ��UART1
}

/***********************************************************************
 ������      ��HAL_UART_MspInit()  
 ��������    ��UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж����� 
 ����        ��huart:���ھ��
 ���        ����
 ˵��        ���˺����ᱻHAL_UART_Init()���� 
                           
************************************************************************/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if(huart==(&UART1_Handler))
	{
    //GPIO�˿�����
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9	
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//����ΪUSART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
	
    __HAL_UART_DISABLE_IT(huart,UART_IT_TC);
#if EN_USART1_RX
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//���������ж�
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}
	
	if(huart==(&UART2_Handler))
	{
    //GPIO�˿�����
		GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART2_CLK_ENABLE();			//ʹ��USART2ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_2;			//PA2	
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART2;	//����ΪUSART2
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

		GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA3
	
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//���������ж�
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//ʹ��USART2�ж�
		HAL_NVIC_SetPriority(USART2_IRQn,1,3);			//��ռ���ȼ�1�������ȼ�3
	}
		
	if(huart==(&UART3_Handler))
	{
		  //GPIO�˿�����
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOB_CLK_ENABLE();			//ʹ��GPIOBʱ��
		__HAL_RCC_USART3_CLK_ENABLE();			//ʹ��USART3ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART3;	//����ΪUSART3
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB11
	
//		__HAL_UART_DISABLE_IT(huart,UART_IT_TC);
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//���������ж�
		HAL_NVIC_EnableIRQ(USART3_IRQn);				//ʹ��USART3�ж�
		HAL_NVIC_SetPriority(USART3_IRQn,2,3);			//��ռ���ȼ�2�������ȼ�3	
	}
	
		if(huart==(&UART4_Handler))
		{
		  //GPIO�˿�����
		GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_GPIOC_CLK_ENABLE();			//ʹ��GPIOCʱ��
		__HAL_RCC_UART4_CLK_ENABLE();			//ʹ��UART4ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PC10,U4 TX
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF8_UART4;	//����ΪUART4
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//��ʼ��PC10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11,U4 RX
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//��ʼ��PB11		
		
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//���������ж�
		HAL_NVIC_EnableIRQ(UART4_IRQn);				    //ʹ��UART4�ж�
		HAL_NVIC_SetPriority(UART4_IRQn,0,3);			//��ռ���ȼ�2�������ȼ�3	
		TIM7_Init(1000-1,9000-1);		                //100ms�ж�
		UART4_RX_LEN=0;		                            //����
		TIM7->CR1&=~(1<<0);                             //�رն�ʱ��7			
		}	
	
	if(huart==(&UART6_Handler))
	{
		  //GPIO�˿�����
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOC_CLK_ENABLE();			//ʹ��GPIOBʱ��
		__HAL_RCC_USART6_CLK_ENABLE();			//ʹ��USART6ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_6;			//PC6,U6 TX
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF8_USART6;	//����ΪUSART6
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//��ʼ��PB10

		GPIO_Initure.Pin=GPIO_PIN_7;			//PC7,U6 RX
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//��ʼ��PB11
	
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//���������ж�
		HAL_NVIC_EnableIRQ(USART6_IRQn);				//ʹ��USART3�ж�
		HAL_NVIC_SetPriority(USART3_IRQn,4,3);			//��ռ���ȼ�4�������ȼ�3	
	}	
}

/***********************************************************************
 ������      ��u2_printf()  
 ��������    ������2,printf ����
 ����        ����
 ���        ����
 ˵��        ��ȷ��һ�η������ݲ�����USART2_MAX_SEND_LEN�ֽ�
                           
************************************************************************/
void u2_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART2_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((USART2->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART2->DR=USART2_TX_BUF[j];  
	} 
}

/***********************************************************************
 ������      ��u3_printf()  
 ��������    ������3,printf ����
 ����        ����
 ���        ����
 ˵��        ��ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
                           
************************************************************************/
void u3_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((USART3->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART3->DR=USART3_TX_BUF[j];  
	} 
}

/***********************************************************************
 ������      ��u4_printf()  
 ��������    ������4,printf ����
 ����        ����
 ���        ����
 ˵��        ��ȷ��һ�η������ݲ�����USART4_MAX_SEND_LEN�ֽ�
                           
************************************************************************/
void u4_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)UART4_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART4_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((UART4->SR&0X40)==0);			//ѭ������,ֱ���������   
		UART4->DR=UART4_TX_BUF[j];  
	} 
}

/***********************************************************************
 ������      ��u6_printf()  
 ��������    ������6,printf ����
 ����        ����
 ���        ����
 ˵��        ��ȷ��һ�η������ݲ�����USART6_MAX_SEND_LEN�ֽ�
                           
************************************************************************/
void u6_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART6_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART6_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((USART6->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART6->DR=USART6_TX_BUF[j];  
	} 
}

/***********************************************************************
 ������      ��USART1_IRQHandler()  
 ��������    ������1�жϷ������
 ����        ����
 ���        ����
                           
************************************************************************/
void USART1_IRQHandler(void)                	
{ 
		u32 timeout=0;
	u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&UART1_Handler);	//����HAL���жϴ����ú���
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY)//�ȴ�����
	{
	 timeout++;////��ʱ����
   if(timeout>maxDelay) break;			
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntExit();  											 
#endif
}
 
/***********************************************************************
 ������      ��USART2_IRQHandler()  
 ��������    ������2�жϷ������
 ����        ����
 ���        ����
                           
************************************************************************/
void USART2_IRQHandler(void)                	
{ 
	u8 Res;
	if((__HAL_UART_GET_FLAG(&UART2_Handler,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
        HAL_UART_Receive(&UART2_Handler,&Res,1,1000); 
		if((USART2_RX_LEN&0x8000)==0)//����δ���
		{
			if(USART2_RX_LEN&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART2_RX_LEN=0;//���մ���,���¿�ʼ
				else USART2_RX_LEN|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART2_RX_LEN|=0x4000;
				else
				{
					USART2_RX_BUF[USART2_RX_LEN&0X3FFF]=Res ;
					USART2_RX_LEN++;
					if(USART2_RX_LEN>(USART2_MAX_RECV_LEN-1))USART2_RX_LEN=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART2_Handler);	
} 

/***********************************************************************
 ������      ��USART3_IRQHandler()  
 ��������    ������3�жϷ������
 ����        ����
 ���        ����
                           
************************************************************************/
void USART3_IRQHandler(void)
{
	u8 Res;
	if((__HAL_UART_GET_FLAG(&UART3_Handler,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
        HAL_UART_Receive(&UART3_Handler,&Res,1,1000); 
		if((USART3_RX_LEN&0x8000)==0)//����δ���
		{
			if(USART3_RX_LEN&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART3_RX_LEN=0;//���մ���,���¿�ʼ
				else USART3_RX_LEN|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART3_RX_LEN|=0x4000;
				else
				{
					USART3_RX_BUF[USART3_RX_LEN&0X3FFF]=Res ;
					USART3_RX_LEN++;
					if(USART3_RX_LEN>(USART3_MAX_RECV_LEN-1))USART3_RX_LEN=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART3_Handler);	
}  

/***********************************************************************
 ������      ��USART4_IRQHandler()  
 ��������    ������4�жϷ������
 ����        ����
 ���        ����
                           
************************************************************************/
void UART4_IRQHandler(void)                	
{ 
	u8 res;	      
	if(__HAL_UART_GET_FLAG(&UART4_Handler,UART_FLAG_RXNE)!=RESET)//���յ�����
	{	 
		res=UART4->DR; 			 
		if((UART4_RX_LEN&(1<<15))==0)               //�������һ������,��û�б�����,���ٽ�����������
		{ 
			if(UART4_RX_LEN<UART4_MAX_RECV_LEN)	    //�����Խ�������
			{
				TIM7->CNT=0;         				//���������	
				if(UART4_RX_LEN==0) 				//ʹ�ܶ�ʱ��7���ж� 
				{
					TIM7->CR1|=1<<0;     			//ʹ�ܶ�ʱ��7
				}
				UART4_RX_BUF[UART4_RX_LEN++]=res;	//��¼���յ���ֵ
				
			}
			else 
			{
				UART4_RX_LEN|=1<<15;			    //ǿ�Ʊ�ǽ������
			} 
		}
	}  				 											 
}  
 
/***********************************************************************
 ������      ��USART6_IRQHandler()  
 ��������    ������6�жϷ������
 ����        ����
 ���        ����
                           
************************************************************************/
void USART6_IRQHandler(void)
{
	u32 timeout=0;
	u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&UART6_Handler);	//����HAL���жϴ����ú���
	
	timeout=0;
    while (HAL_UART_GetState(&UART6_Handler) != HAL_UART_STATE_READY)//�ȴ�����
	{
	 timeout++;	//��ʱ����
   if(timeout>maxDelay) break;			
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART6_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntExit();  											 
#endif
}  

/***********************************************************************
 ������      ��usmart_pow1()  
 ��������    ��m^n����
 ����        ��
 ���        ��m^n�η�
                           
************************************************************************/
int long usmart_pow1(u8 m,u8 n)
{
	int long result=1;	 
	while(n--)result*=m;    
	return result;
}

/***********************************************************************
 ������      ��Fabs()  
 ��������    ���������ֵ
 ����        ��
 ���        ���������ľ���ֵ
                           
************************************************************************/
u16 abs(u16 data1,u16 data2)
{
  if((data1-data2)>0)
	{
	  return (data1-data2);
	}
	
	else
	{
	  return (data2-data1);
	}
}

/***********************************************************************
 ������      ��usmart_strnum()  
 ��������    �����ַ���תΪ����
 ����        ��*str:�����ַ���ָ��
 ���        ��0���������.
                           
************************************************************************/
int long   usmart_strnum(u8*str)
{
	u32 t;
	u8  bnum=0;	//���ֵ�λ��
	u8 *p;		  
	u8 hexdec=10;//Ĭ��Ϊʮ��������
	p=str;
	int long  res=0;//����.
	while(1)    //�����ַ��������ֵĸ���
	{		
		if(*p<='9'&&*p>='0') 	//�ַ����д�������
		 {
			 bnum++;					//λ������.
		 }
		else if(*p=='\0')
		{
			break;	//����������,�˳�.
		}
		p++; 
	} 
	p=str;			    //���¶�λ���ַ�����ʼ�ĵ�ַ.
	if(bnum==0) return 0;//λ��Ϊ0��ֱ���˳�.	  
	while(1)
	{
		if((*p<='9'&&*p>='0') && (bnum>0))
		{
			bnum--;
			t=*p-'0';	//�õ����ֵ�ֵ
			res+=t*usmart_pow1(hexdec,bnum);		 //���õ�������ת��Ϊʮ����  
		}   	
		p++;
		if(*p=='\0')
		{
			break;//���ݶ�������.	
		}
	}
	return res;//�ɹ�ת��,����ת�����ʮ����������
}



/***********************************************************************
 ������      ��usmart_strnum()  
 ��������    �����ַ���תΪ����
 ����        ��*str:�����ַ���ָ��
 ���        ��0���������.
                           
************************************************************************/
u32 usmart_strnum2(u8*str)
{
	u32 t;
	u8  bnum=0;	//���ֵ�λ��
	u8 *p;		  
	u8 hexdec=10;//Ĭ��Ϊʮ��������
	p=str;
	u32  res=0;//����.
	while(1)    //�����ַ��������ֵĸ���
	{		
		if(*p<='9'&&*p>='0') 	//�ַ����д�������
		 {
			 bnum++;					//λ������.
		 }
		else if(*p=='\0')
		{
			break;	//����������,�˳�.
		}
		p++; 
	} 
	p=str;			    //���¶�λ���ַ�����ʼ�ĵ�ַ.
	if(bnum==0) return 0;//λ��Ϊ0��ֱ���˳�.	  
	while(1)
	{
		if((*p<='9'&&*p>='0') && (bnum>0))
		{
			bnum--;
			t=*p-'0';	//�õ����ֵ�ֵ
			res+=t*usmart_pow1(hexdec,bnum);		 //���õ�������ת��Ϊʮ����  
		}   	
		p++;
		if(*p=='\0')
		{
			break;//���ݶ�������.	
		}
	}
	return res;//�ɹ�ת��,����ת�����ʮ����������
}
