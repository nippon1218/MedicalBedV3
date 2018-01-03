#include "pump.h"
#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"
#include "function.h"
#include "led.h"
#include "motor.h"

/*********************************************************************
*������       ��Pump_Init()
*��������     ��ˮ��IO�ڳ�ʼ��
*����         ����
*���         ����

***********************************************************************/
void Pump_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();                         //����GPIOBʱ��
	__HAL_RCC_GPIOH_CLK_ENABLE();                         //����GPIOHʱ��
	
	
    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3;               //PH2/3/7
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //�������
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	  
	GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11;             //PB10/11
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //�������
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);		
	
	GPIO_Initure.Pin=GPIO_PIN_3;                          //PE3
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //�������
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //����
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);	
	
	GPIO_Initure.Pin=GPIO_PIN_1;             
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;                //�������
    GPIO_Initure.Pull=GPIO_PULLDOWN;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                   //����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);		
}

/*********************************************************************
*������       ��Sensor_Init()
*��������     ��������IO�ڳ�ʼ��
*����         ����
*���         ����

***********************************************************************/
void Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();                       //����GPIOBʱ��
	__HAL_RCC_GPIOC_CLK_ENABLE();                       //����GPIOBʱ��
	__HAL_RCC_GPIOE_CLK_ENABLE();                       //����GPIOEʱ�� 
	__HAL_RCC_GPIOH_CLK_ENABLE();                       //����GPIOHʱ�� 

    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;           //PB14/15
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //���븡��
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13;  //PC6/7/8/9/13
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //���븡��
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	
	  
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;  //PE2/4/5/6
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //���븡��
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
 	
	GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_9;             //PH679
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //���븡��
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_7;  						//PH7
    GPIO_Initure.Mode=GPIO_MODE_INPUT;                  //���븡��
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	
}

/*********************************************************************
*������       ��Push_Rod_Swash_Dry_Init()
*��������     ����ϴ����Ƹ˷���ں�����ڳ�ʼ��
*����         ����
*���         ����

***********************************************************************/
void Push_Rod_Swash_Dry_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //����GPIOBʱ��
	__HAL_RCC_GPIOC_CLK_ENABLE();           //����GPIOCʱ��

	GPIO_Initure.Pin=GPIO_PIN_13;           //PB13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //���
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_1;            //PC1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //���
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	
	
}

/*********************************************************************
*������       ��Push_Rod_Swash_Dry()
*��������     ����ϴ����Ƹ�������
*����         ��dir(�����źţ�1�������0������)
*���         ����

***********************************************************************/
void Push_Rod_Swash_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //�Ƹ˳�ʼ��	
	u8 len;
	int j,flag=1;                   //flagΪ����ߵͱ�־λ	
	DIR_CXHG=dir;                   //�жϳ�ϴ����Ƹ����������
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	delay_ms(100);
	TIM2_Init(1800-1,motor_timer_freq); //3600                                    //�򿪶�ʱ��
	if(((1==dir)&&(push_rod_runed_pulse<push_rod_pulse_lim))||((0==dir)&&(push_rod_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //�����ʱ���жϱ�־λ					
			PWM_CXHG=flag;                                              //��ϴ��ɵ����������ڸ�/�͵�ƽ
			if(UART4_RX_LEN&0x8000)
			{			
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				push_rod_runed_pulse++;
			}
			else
			{
				push_rod_runed_pulse--;					
			}
			//���ͳ�ϴ����Ƹ˶���ָ��
			if(push_rod_runed_pulse==1)	
			{
				u2_printf("Cartoon_Push_Rod_1");
			}
			else if(push_rod_runed_pulse==(u16)(push_rod_pulse_lim/3))	
			{
				u2_printf("Cartoon_Push_Rod_2");				
			}
			else if(push_rod_runed_pulse==(u16)(2*(push_rod_pulse_lim/3)))
			{
				u2_printf("Cartoon_Push_Rod_3");
			}
			else if(((push_rod_runed_pulse==push_rod_pulse_lim)&&(1==dir))||((push_rod_runed_pulse==push_rod_pulse_lim-1)&&(0==dir)))	
			{
				u2_printf("Cartoon_Push_Rod_4");
			}
		}
	}	
	PWM_CXHG=0;                                        //��ϴ��ɵ����Ӧ�������������  
	TIM2_Stop();                                       //�رն�ʱ��
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //�������4������
	UART4_RX_LEN=0;
}	
//���Ժ���
void Push_Rod_Swash_Dry1(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //�Ƹ˳�ʼ��	
	u8 len;
	int j,flag=1;                   //flagΪ����ߵͱ�־λ	
	DIR_CXHG=dir;                   //�жϳ�ϴ����Ƹ����������
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;		
	for(j=0;j<n;j++)
	{ 	
		flag = !flag;
		PWM_CXHG=flag;                                 //��ϴ��ɵ����������ڸ�/�͵�ƽ
		delay_ms(1);      //�����������
	}	
	PWM_CXHG=0;                                        //��ϴ��ɵ����Ӧ�������������  
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //�������4������
	UART4_RX_LEN=0;
}	


/*********************************************************************
*������       ��Push_Rod_Swash()
*��������     ����ϴ�Ƹ�������
*����         ��dir(�����źţ�1�������0������)
*���         ����

***********************************************************************/
void Push_Rod_Swash(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //�Ƹ˳�ʼ��
	static u16 k;                   //����k�Ŷ���
	static u8 kj;
	u8 break_flag;
	u8 len;                         //���յ��ַ�������	
	int j,flag=1;                   //flagΪ����ߵͱ�־λ	
	DIR_CXHG=dir;                   //�жϳ�ϴ����Ƹ����������
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //�򿪶�ʱ��
	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //�����ʱ���жϱ�־λ					
			PWM_CXHG=flag;                                              //��ϴ��ɵ����������ڸ�/�͵�ƽ
			if(UART4_RX_LEN&0x8000)
			{			
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;
				//�����յ�Stop,������ѭ��	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==swash_hand_flag))
				{
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
					break;
				}
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//���ͳ�ϴ�Ƹ˶���ָ��						
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Swash_2");
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_5");
			}
		}
	}	
	PWM_CXHG=0;                                        //��ϴ��ɵ����Ӧ�������������  
	TIM2_Stop();                                       //�رն�ʱ��
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //�������4������
	UART4_RX_LEN=0;
}	


/*********************************************************************
*������       ��Push_Rod_Dry()
*��������     ������Ƹ�������
*����         ��dir(�����źţ�1�������0������)
*���         ����

***********************************************************************/
void Push_Rod_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //�Ƹ˳�ʼ��	
	u8 len;
	u8 break_flag;
	int j,flag=1;                   //flagΪ����ߵͱ�־λ	
	DIR_CXHG=dir;                   //�жϳ�ϴ����Ƹ����������
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //�򿪶�ʱ��
	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //�����ʱ���жϱ�־λ					
			PWM_CXHG=flag;                                              //��ϴ��ɵ����������ڸ�/�͵�ƽ
			if(UART4_RX_LEN&0x8000)
			{			
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;				
				//�����յ�Stop,������ѭ��	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==dry_hand_flag))
				{
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
					break;
				}
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
					UART4_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//���ͺ���Ƹ˶���ָ��
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Dry_2");				
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))				
			{
				u2_printf("Cartoon_Push_Rod_Dry_5");
			}
		}
	}	
	PWM_CXHG=0;                                        //��ϴ��ɵ����Ӧ�������������  
	TIM2_Stop();                                       //�رն�ʱ��
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);         //�������4������
	UART4_RX_LEN=0;
}	

/*********************************************************************
*������       ��Uart_Push_Rod_Swash_Dry()
*��������     ����ϴ����Ƹ�������
*����         ��dir(�����źţ�1�������0������)
*���         ����

***********************************************************************/
void Uart_Push_Rod_Swash_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //�Ƹ˳�ʼ��	
	u8 len;
	int j,flag=1;                   //flagΪ����ߵͱ�־λ	
	RELAY6=1;                       //�̵����õ磬��������պϣ��������������õ�

	DIR_CXHG=dir;                   //�жϳ�ϴ����Ƹ����������
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //�򿪶�ʱ��
	if(((1==dir)&&(push_rod_runed_pulse<push_rod_pulse_lim))||((0==dir)&&(push_rod_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //�����ʱ���жϱ�־λ					
			PWM_CXHG=flag;                                              //��ϴ��ɵ����������ڸ�/�͵�ƽ
			if(USART2_RX_LEN&0x8000)
			{			
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				push_rod_runed_pulse++;
			}
			else
			{
				push_rod_runed_pulse--;					
			}
			//���ͳ�ϴ����Ƹ˶���ָ��
//			if(push_rod_runed_pulse==1)	
//			{
//				u2_printf("Cartoon_Push_Rod_1");				
//			}
//			else if(push_rod_runed_pulse==(u16)(push_rod_pulse_lim/3))	
//			{
//				u2_printf("Cartoon_Push_Rod_2");				
//			}
//			else if(push_rod_runed_pulse==(u16)(2*(push_rod_pulse_lim/3)))
//			{
//				u2_printf("Cartoon_Push_Rod_3");
//			}			
//			else if(((push_rod_runed_pulse==push_rod_pulse_lim)&&(1==dir))||((push_rod_runed_pulse==push_rod_pulse_lim-1)&&(0==dir)))	
//			{
//				u2_printf("Cartoon_Push_Rod_4");
//			}
		}
	}	
	PWM_CXHG=0;                                        //��ϴ��ɵ����Ӧ�������������  
	TIM2_Stop();                                       //�رն�ʱ��
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //�������4������
	USART2_RX_LEN=0;
}	

/*********************************************************************
*������       ��Uart_Push_Rod_Swash()
*��������     ����ϴ�Ƹ�������
*����         ��dir(�����źţ�1�������0������)
*���         ����

***********************************************************************/
void Uart_Push_Rod_Swash(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //�Ƹ˳�ʼ��
	static u16 k;                   //����k�Ŷ���
	static u8 kj;
	u8 break_flag;
	u8 len;                         //���յ��ַ�������	
	int j,flag=1;                   //flagΪ����ߵͱ�־λ	
	DIR_CXHG=dir;                   //�жϳ�ϴ����Ƹ����������
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //�򿪶�ʱ��
//	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
//	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //�����ʱ���жϱ�־λ					
			PWM_CXHG=flag;                                              //��ϴ��ɵ����������ڸ�/�͵�ƽ
			if(USART2_RX_LEN&0x8000)
			{			
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;
				 //�����յ�Stop,������ѭ��	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==swash_hand_flag))
				{
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
					break;
				}
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //�������4������
					USART2_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//���ͳ�ϴ�Ƹ˶���ָ��						
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Swash_2");
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Swash_5");
			}
		}
//	}	
	PWM_CXHG=0;                                        //��ϴ��ɵ����Ӧ�������������  
	TIM2_Stop();                                       //�رն�ʱ��
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //�������2������
	USART2_RX_LEN=0;
}	


/*********************************************************************
*������       ��Uart_Push_Rod_Dry()
*��������     ������Ƹ�������
*����         ��dir(�����źţ�1�������0������)
*���         ����

***********************************************************************/
void Uart_Push_Rod_Dry(u8 dir,u32 n)
{
	Push_Rod_Swash_Dry_Init();      //�Ƹ˳�ʼ��	
	u8 len;
	u8 break_flag;
	int j,flag=1;                   //flagΪ����ߵͱ�־λ	
	DIR_CXHG=dir;                   //�жϳ�ϴ����Ƹ����������
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;			
	TIM2_Init(3600-1,motor_timer_freq);                                     //�򿪶�ʱ��
	if(((1==dir)&&(swash_dry_runed_pulse<swash_dry_pulse_lim))||((0==dir)&&(swash_dry_runed_pulse>0)))	
	{	
		for(j=0;j<n;j++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler,TIM_SR_CC3IF)));   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag;
			__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC3IF);          //�����ʱ���жϱ�־λ					
			PWM_CXHG=flag;                                              //��ϴ��ɵ����������ڸ�/�͵�ƽ
			if(USART2_RX_LEN&0x8000)
			{			
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;			
				//�����յ�Stop,������ѭ��	
				if(((strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))|| 
					(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))||
							(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone")))&&(1==dry_hand_flag))
				{
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
					break;
				}
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
				{	}
				else 
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //�������4������
					USART2_RX_LEN=0;
				}				
			}
			if(1==dir)
			{
				swash_dry_runed_pulse++;
			}
			else
			{
				swash_dry_runed_pulse--;					
			}
			//���ͺ���Ƹ˶���ָ��
			if(swash_dry_runed_pulse==(u16)(swash_dry_pulse_lim/4))	
			{
				u2_printf("Cartoon_Push_Rod_Dry_2");				
			}
			else if(swash_dry_runed_pulse==(u16)(2*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_3");
			}
			else if(swash_dry_runed_pulse==(u16)(3*(swash_dry_pulse_lim/4)))
			{
				u2_printf("Cartoon_Push_Rod_Dry_4");
			}			
			else if(((swash_dry_runed_pulse==swash_dry_pulse_lim)&&(1==dir))||((swash_dry_runed_pulse==swash_dry_pulse_lim-1)&&(0==dir)))				
			{
				u2_printf("Cartoon_Push_Rod_Dry_5");
			}
		}
	}	
	PWM_CXHG=0;                                        //��ϴ��ɵ����Ӧ�������������  
	TIM2_Stop();                                       //�رն�ʱ��
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);         //�������4������
	USART2_RX_LEN=0;
}	




