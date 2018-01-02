#include "motor.h"
#include "sys.h"
#include "pwm.h"
#include "delay.h"
#include "usart.h"
#include "function.h"
#include "pump.h"
#include "usart.h"
#include "LED.h"
#include "pcf8574.h"
#include "key.h"
#include "check.h"


u8 Desk1st2st=1;
u8 DeskSwitch=0;
u8 DeskLastJudgeFlg=0;
u8 DeskFrontFlg=0;


extern unsigned int motor_hang_freq;    //���ҵ���Զ���װ��ֵ�����Ƶ�������ٶ� 
extern unsigned int motor_timer_freq;   //��ʱ����Ƶֵpsc   ��ʱ��Ƶ��=90M/50

/*********************************************************************
*������       ��Motor_Dir_Init()
*��������     ��6̨�������ڳ�ʼ��
*����         ����
*���         ����
***********************************************************************/
void Motor_Dir_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOA_CLK_ENABLE();           //����GPIOAʱ��
	__HAL_RCC_GPIOC_CLK_ENABLE();           //����GPIOCʱ��
	__HAL_RCC_GPIOD_CLK_ENABLE();           //����GPIODʱ��
	__HAL_RCC_GPIOG_CLK_ENABLE();           //����GPIOGʱ�� 
	__HAL_RCC_GPIOH_CLK_ENABLE();           //����GPIOHʱ��	
	
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6;            //PA5/6  
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;             //�������
    GPIO_Initure.Pull=GPIO_PULLUP;                     //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                //����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	GPIO_Initure.Pin= GPIO_PIN_4|GPIO_PIN_12;           //PC4/12
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //�������
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	
	
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7;  //PD2/3/7
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //�������
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);		
			
	GPIO_Initure.Pin=GPIO_PIN_13;                       //PH13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //�������
    GPIO_Initure.Pull=GPIO_PULLUP;                      //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	
	
//�������������̵��������Ƶ����ͨ�ϣ��������㣬�ߵ�ƽ��Ч
	
	GPIO_Initure.Pin=GPIO_PIN_12;                       //PH12
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //�������
    GPIO_Initure.Pull=GPIO_PULLDOWN;                    //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;                 //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
}

/*********************************************************************
*������       ��Push_Rod_Init()
*��������     ���綯�Ƹ˷���ں�����ڳ�ʼ��-֧��������
*����         ����
*���         ����
***********************************************************************/
void Push_Rod_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOI_CLK_ENABLE();           //����GPIOIʱ��
	__HAL_RCC_GPIOG_CLK_ENABLE(); 	        //����GPIOGʱ��
	
	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_3|GPIO_PIN_7;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_10;           //PG10
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);		
}

/*********************************************************************
*������       ��Hang_Init()
*��������     �����ҵ������ڡ�����ڳ�ʼ��
*����         ����
*���         ����
***********************************************************************/
void Hang_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //����GPIOBʱ��
	__HAL_RCC_GPIOD_CLK_ENABLE();           //����GPIODʱ��  
    __HAL_RCC_GPIOG_CLK_ENABLE();           //����GPIOGʱ��
	__HAL_RCC_GPIOH_CLK_ENABLE();           //����GPIOHʱ�� 

	  GPIO_Initure.Pin=GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //�������
    GPIO_Initure.Pull=GPIO_PULLUP;              //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	
    GPIO_Initure.Pin=GPIO_PIN_6 | GPIO_PIN_7|  GPIO_PIN_8|GPIO_PIN_9;  //PB6/7/8//9
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //�������
    GPIO_Initure.Pull=GPIO_PULLUP;              //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����	
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
  	GPIO_Initure.Pin=GPIO_PIN_13;               //PD13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //�������
    GPIO_Initure.Pull=GPIO_PULLUP;              //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����	
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_11;              //PH11
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;     //�������
    GPIO_Initure.Pull=GPIO_PULLUP;             //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;        //����	
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_6;    //PG3/6
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;     //�������
    GPIO_Initure.Pull=GPIO_PULLUP;             //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;        //����
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
}

/*********************************************************************
*������       ��Push_Rod_Start()
*��������     ���綯�Ƹ���������
*����         ��dir��1�������ȣ�0��������
*���         ����
***********************************************************************/
void Push_Rod_Start(u8 dir)
{
	Push_Rod_Init();
	if(dir==1)
	{
		DIR2_DOWN=0 ;   //������
		DIR2_UP=1 ; 		
	}
	else
	{
		DIR2_DOWN=1 ;   //������
		DIR2_UP=0 ; 	
	}
}
/*********************************************************************
*������       ��Push_Rod_Stop()
*��������     ���綯�Ƹ�ֹͣ����
*����         ����
*���         ����
***********************************************************************/
void Push_Rod_Stop(void)
{
	DIR2_DOWN=0 ;     //��Ӧ���������
	DIR2_UP=0 ; 	
}

/*********************************************************************
*������       ��Motor_1_START()
*��������     ��֧���綯�Ƹ���������
*����         ��dir��1�������ȣ�0��������
*���         ����
***********************************************************************/
void Motor_1_START(u8 dir)
{
	Push_Rod_Init();
	if(dir==1)
	{
		DIR1_DOWN=0 ;   //֧������
		DIR1_UP=1 ;  		
	}
	else
	{
		DIR1_DOWN=1 ;   //֧������
		DIR1_UP=0 ; 	
	}
}
/*********************************************************************
*������       ��Motor_1_STOP()
*��������     ��֧���綯�Ƹ�ֹͣ����
*����         ����
*���         ����
***********************************************************************/
void Motor_1_STOP(void)
{
	DIR1_DOWN=0 ;     //��Ӧ���������
	DIR1_UP=0 ; 	
}

/*********************************************************************
*������       ��Motor_3_START()
*��������     ��3�ŵ������
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_3_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8_PWM_CHANNEL_2_START(arr,psc);     //TIM8ͨ�� 2 PWM������� 	
}
/*********************************************************************
*������       ��Motor_5_START()
*��������     ��5�ŵ������
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_5_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8_PWM_CHANNEL_1_START(arr,psc);     //TIM8ͨ�� 1 PWM������� 
	
}
/*********************************************************************
*������       ��Motor_4_START()
*��������     ��4�ŵ������
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_4_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM5_PWM_CHANNEL_4_START(arr,psc);     //TIM5ͨ��4 PWM������� 
}

/*********************************************************************
*������       ��Motor_4_Compensate()
*��������     ��4�ŵ������
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_4_Compensate(u8 dir,u16 time_arr,u16 arr,u16 psc)
{
	DIR4=dir;
	Motor_4_START(arr,psc);
	TIM10_Init(time_arr,timer10_freq);
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); 
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
	{

	}
	Motor_4_STOP();
}


/*********************************************************************
*������       ��Motor_6_START()
*��������     ��6�ŵ������
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_6_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM3_PWM_CHANNEL_2_START(arr,psc);     //TIM8ͨ�� 2 PWM������� 
}

/*********************************************************************
*������       ��Motor_6_1_START()
*��������     �������������ս��������
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_6_1_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8_PWM_CHANNEL_4_START(arr,psc);     //TIM8ͨ�� 4 PWM������� 
}

/*********************************************************************
*������       ��Motor_6_2_START()
*��������     �������������ս�ǰ�Ƹ�������
*����         ��dir(���з���)��pulse(����������)
*���         ����
***********************************************************************/
void Motor_6_2_START(u8 dir,u32 pulse)
{	
	Motor_Dir_Init();
	int i,flag=1;          //flagΪ����ߵͱ�־λ
    DIR6_2=dir;            //������ս��������ת��������ƣ�1�ߵ�ƽ��ת��0�͵�ƽ��ת
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
    TIM10_Init(3600-1,motor_timer_freq);         //�򿪶�ʱ��
	if(((1==dir)&&(push_rod_tig_runed_pulse<push_rod_tig_pulse_lim))||((0==dir)&&(push_rod_tig_runed_pulse>0)))
    {
		for(i=0;i<pulse;i++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag ;
			PWM6_2=flag;          //������ս����������������ڸ�/�͵�ƽ
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
			//�����������ж������ۼ�
			if(1==dir)           //���У�����+
			{
				push_rod_tig_runed_pulse++;
			}
			else                 //���У�����-
			{
				push_rod_tig_runed_pulse--;
			}
			//���Ͷ���ָ��
			if(push_rod_tig_runed_pulse==1)
			{
				u2_printf("Cartoon_Push_Rod_Tig_1");
			}
			if(push_rod_tig_runed_pulse==(u16)(pulse/2))
			{
				u2_printf("Cartoon_Push_Rod_Tig_2");
			}
			if(((push_rod_tig_runed_pulse==pulse)&&(1==dir))||((push_rod_tig_runed_pulse==pulse-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Tig_3");
			} 
		}
	}
	PWM6_2=0;                      //������ս����������Ӧ�������������  
	TIM10_Stop();                  //�رն�ʱ��
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);   //����жϱ�־λ
}

/*********************************************************************
*������       ��Motor_7_START()
*��������     ��7�ŵ������
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_7_START(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM1_PWM_CHANNEL_4_START(arr,psc);     //TIM8ͨ�� 2 PWM������� 
}

/*********************************************************************
*������       ��Motor_3_4_5_START()
*��������     ��345�ŵ��ͬʱ��������ɷ���
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_3_4_5_START_left(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8_PWM_CHANNEL_2_START(arr,psc);     //TIM8ͨ�� 2 PWM������� ��3�ŵ��
	TIM8_PWM_CHANNEL_1_START(arr,psc);     //TIM8ͨ�� 1 PWM������� ,5�ŵ��
	TIM5_PWM_CHANNEL_4_START((u16)(arr*1.2),psc);     //TIM5ͨ�� 4 PWM������� 	��4�ŵ��
}

/*********************************************************************
*������       ��Motor_3_4_5_START()
*��������     ��345�ŵ��ͬʱ��������ɷ���
*����         ��arr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor_3_4_5_START_right(u16 arr,u16 psc)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8_PWM_CHANNEL_1_START(arr,psc);     //TIM8ͨ�� 2 PWM�������,5�ŵ�� 
	TIM8_PWM_CHANNEL_2_START((u16)(arr*1.4),psc);     //TIM8ͨ�� 2 PWM������� ��3�ŵ��
	
	TIM5_PWM_CHANNEL_4_START(arr,psc);     //TIM8ͨ�� 2 PWM������� 	��4�ŵ��
}

/*********************************************************************
*������       ��Motor_3_STOP()
*��������     ��3�ŵ��ֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_3_STOP(void)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8->CCER&=~(1<<4);                   //��ʱ��8ͨ��2
}
/*********************************************************************
*������       ��Motor_5_STOP()
*��������     ��5�ŵ��ֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_5_STOP(void)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8->CCER&=~(1<<0);                   //��ʱ��8ͨ��1
}
/*********************************************************************
*������       ��Motor_5_STOP()
*��������     ��5�ŵ��ֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_4_STOP(void)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM5->CCER&=~(1<<12);                  //��ʱ��5ͨ��4
}
/*********************************************************************
*������       ��Motor_6_STOP()
*��������     ��6�ŵ��ֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_6_STOP(void)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM3->CCER&=~(1<<4);                   //��ʱ��3ͨ��2
}

/*********************************************************************
*������       ��Motor_6_1_STOP()
*��������     �������������ս����ֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_6_1_STOP(void)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8->CCER&=~(1<<12);                  //��ʱ��8ͨ��4
}
/*********************************************************************
*������       ��Motor_7_STOP()
*��������     ��7�ŵ��ֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_7_STOP(void)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM1->CCER&=~(1<<12);                  //��ʱ��1ͨ��4
}
/*********************************************************************
*������       ��Motor_3_4_5_STOP()
*��������     ��345�ŵ��ͬʱֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_3_4_5_STOP(void)
{
	Motor_Dir_Init();                      //�������ڳ�ʼ������
	TIM8->CCER&=~(1<<4);                   //��ʱ��8ͨ��2
	TIM8->CCER&=~(1<<0);                   //��ʱ��8ͨ��1
	TIM5->CCER&=~(1<<12);	               //��ʱ��5ͨ��4
}

/*********************************************************************
*������       ��Motor_All_Stop()
*��������     �����е��ֹͣ
*����         ����
*���         ����
***********************************************************************/
void Motor_All_Stop(void)
{      
	Motor_1_STOP();                //֧�����ֹͣ����
	Push_Rod_Stop();               //���ȵ��ֹͣ����
	Motor_3_STOP();                //�������ֹͣ����
	Motor_4_STOP();                //����֧�����ֹͣ����
	Motor_5_STOP();                //�෭���ֹͣ����
	Motor_3_4_5_STOP();            //��/�ҷ�����ֹͣ����	
	Motor_6_STOP();                //���������ֹͣ����
	Motor_6_1_STOP();              //������������ֹͣ����
	Motor_7_STOP();                //С���ӵ��ֹͣ����	
}

/*********************************************************************
*������       ��Auto_Hang_1()
*��������     ��1�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Auto_Hang_1(u16 dir,u32 pulse)
{
    Hang_Init();                      //�������ڳ�ʼ������
	int i,flag=1;                     //flagΪ����ߵͱ�־λ
	int j=0,k=0;
    HANG_DIR1=dir;                    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת
    memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ			
		HANG_PWM1=flag;              //1�ŵ��ҵ����������ڸߵ�ƽ 
		if(arm_fore_left_flag==1)    //��첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_fore_left_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_runed--;
			}
			//������첲���Ҷ���ָ��
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}									
		}
		else if( leg_fore_left_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_fore_left_runed++; 
			}
			else           //���У�����-
			{
				leg_fore_left_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}		
			}
		}
	}
	HANG_PWM1=0;                      //1�ŵ��ҵ����Ӧ�������������  
	TIM10_Stop();                     //�رն�ʱ��
}
/*********************************************************************
*������       ��Auto_Hang_3()
*��������     ��3�ŵ��ҵ������
*����         ��dir(����:1��0��)��n(������)
*���         ����
***********************************************************************/
void Auto_Hang_3(u16 dir,u32 pulse)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	if(1==dir)
	{
		HANG_DIR3=0;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת   
	}
	else
	{
		HANG_DIR3=1;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת   
	}
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;   
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM3=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ
		if(arm_fore_right_flag==1)    //�Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(0==dir)     //���У�����+
			{
				arm_fore_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_right_runed--;
			}			
			//������첲���Ҷ���ָ��
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}									
		}
		if( leg_fore_right_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_fore_right_runed++; 
			}
			else           //���У�����-
			{
				leg_fore_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}		
			}
		}		 
	}
	HANG_PWM3=0;              //3�ŵ��ҵ����Ӧ�������������  
	TIM10_Stop();             //�رն�ʱ��
}
/*********************************************************************
*������       ��Auto_Hang_1_2()
*��������     ��1/2�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Auto_Hang_1_2(u16 dir,u32 pulse)
{
	Hang_Init();        //�������ڳ�ʼ������
	int i,flag=1;       //flagΪ����ߵͱ�־λ
	u8 j=0,k=0;
    if(dir==1)
     {
		HANG_DIR1=1;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR2=0;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    else
     {
		HANG_DIR1=0;   //1�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR2=1;   //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;    
    TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)));    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag;
		if(arm_fore_left_flag==1)    //��첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_left_runed++;
			}
			else           //���У�����-
			{
				arm_left_runed--;
			}
			//������첲���Ҷ���ָ��	
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}			
			}	
		}
		if( leg_fore_left_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_left_runed++; 
			}
			else           //���У�����-
			{
				leg_left_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}			
			}			
		}		
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM2=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Auto_Hang_1_3()
*��������     ��1/3�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Auto_Hang_1_3(u16 dir,u32 pulse)
{
	Hang_Init();        //�������ڳ�ʼ������
	int i,flag=1;       //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	if(dir==1)
     {
		HANG_DIR1=1;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR3=0;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    else
     {
		HANG_DIR1=0;   //1�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR3=1;   //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;    
	TIM10_Init(motor_hang_freq,motor_timer_freq);//�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM1=flag;   //1�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM3=flag;   //2�ŵ��ҵ����������ڸ�/�͵�ƽ	
 		if(arm_fore_left_right_flag==1)    //���Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_fore_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_right_runed--;
			}
			//������첲���Ҷ���ָ��
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}									
		}
		if( leg_fore_left_right_flag==1)    //������
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_fore_left_right_runed++; 
			}
			else           //���У�����-
			{
				leg_fore_left_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}		
			}
		}  
	}
	HANG_PWM1=0;          //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM3=0;          //3�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();         //�رն�ʱ��
}


/*********************************************************************
*������       ��Auto_Hang_3_4()
*��������     ��3/4�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Auto_Hang_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	u8 j=0,k=0;	
	if(dir==1)
     {
		HANG_DIR3=0;    //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR4=1;    //4�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    else
     {
		HANG_DIR3=1;   //3�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR4=0;   //4�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);//�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
			
		HANG_PWM3=flag;          //3�ŵ��ҵ����������ڸߵ�ƽ
		HANG_PWM4=flag;          //4�ŵ��ҵ����������ڸߵ�ƽ
		if(arm_fore_right_flag==1)    //�Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_right_runed++;
			}
			else           //���У�����-
			{
				arm_right_runed--;
			}
			//�����Ҹ첲���Ҷ���ָ��	
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}			
			}	
		}
		if( leg_fore_right_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_right_runed++; 
			}
			else           //���У�����-
			{
				leg_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}			
			}			
		}		
	}
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Auto_Hang_1_2_3_4()
*��������     ��1/2/3/4�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Auto_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	u8 j=0,k=0;
	if(dir==1)
    {
		HANG_DIR1=1;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR2=0;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR3=0;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR4=1;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
    }
    else
    {
		HANG_DIR1=0;   //1�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR2=1;   //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR3=1;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR4=0;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
    }
     memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM1=flag;   //1�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM2=flag;   //2�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM3=flag;   //3�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM4=flag;   //4�ŵ��ҵ����������ڸ�/�͵�ƽ	
		if(arm_fore_left_right_flag==1)    //���Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_left_right_runed--;
			}
			//������첲���Ҷ���ָ��	
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}			
			}	
		}
		if( leg_fore_left_right_flag==1)    //������
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_left_right_runed++; 
			}
			else           //���У�����-
			{
				leg_left_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}			
			}			
		}	  		 
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM2=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Hand_Hang_1()
*��������     ��1�ŵ��ҵ������֫С��
*����         ��dir:����ڣ�n�������
*���         ����
***********************************************************************/
void Hand_Hang_1(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;            //flagΪ����ߵͱ�־λ	
	static int k=0;	
    Hang_Init();               //�������ڳ�ʼ������
	HANG_DIR1=dir;  		   //1��ת��0��ת  
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM1=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;		
			 //�����յ�ָֹͣ��,������ѭ��
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))||    
			   (strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftDownHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftDownHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
			   (strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}			
		}
		//������С�۶���ָ��
		if((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag))        //��С�ۡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_fore_left_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_runed--;
			}
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}
		}
		else if((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))   //��С�ȡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_fore_left_runed++;
			}
			else           //���У�����-
			{
				leg_fore_left_runed--;
			}
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}				
			}			
		}
		
		
		if(((arm_fore_left_runed==0)&&((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag)) )||
			((leg_fore_left_runed==0)&&((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}		
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Hand_Hang_1_2()
*��������     ��1/2�ŵ��ҵ����������֫
*����         ��m:����ڣ�pulse�������
*���         ����
***********************************************************************/
void Hand_Hang_1_2(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;     //flagΪ����ߵͱ�־λ
	static int k=0;
    Hang_Init();        //�������ڳ�ʼ������

	if(1==dir)
	{
		HANG_DIR1=1;  	//1��ת��2��ת
		HANG_DIR2=0;  
	}
	else
	{
		HANG_DIR1=0;  	//1��ת��2��ת
		HANG_DIR2=1; 
	}	

	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag; 
	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand")))    
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//������첲����ָ��
		if(1==arm_fore_left_flag)        //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_left_runed++;
			}
			else           //���У�����-
			{
				arm_left_runed--;
			}
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}
		else if((1==arm_post_left_flag)||(1==arm_fore_post_left_flag))   //���ۡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_post_left_runed++;
			}
			else           //���У�����-
			{
				arm_post_left_runed--;
			}
			j=arm_post_left_runed/(arm_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}		
		//�������ȶ���ָ��
		else if(1==leg_fore_left_flag)   //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_left_runed++;
			}
			else           //���У�����-
			{
				leg_left_runed--;
			}
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		else if((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))   //����ȡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_post_left_runed++;
			}
			else           //���У�����-
			{
				leg_post_left_runed--;
			}
			j=leg_post_left_runed/(leg_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		if(((arm_left_runed==0)&&(1==arm_fore_left_flag))||
			((arm_post_left_runed==0)&&((1==arm_post_left_flag)||(1==arm_fore_post_left_flag)))||
			((leg_left_runed==0)&&(1==leg_fore_left_flag))||
			((0==leg_post_left_runed)&&((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM2=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Hand_Hang_3()
*��������     ��3�ŵ��ҵ������֫С��
*����         ��m:����ڣ�n�������
*���         ����
***********************************************************************/

void Hand_Hang_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;          //flagΪ����ߵͱ�־λ	
	static int k=0;		
    Hang_Init();             //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR3=0;  		 //1��ת��2��ת  
	}
	else
	{
		HANG_DIR3=1;  		 //1��ת��2��ת  
	}
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM3=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//������С�۶���ָ��
		if((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag))         //��С�ۡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_fore_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_right_runed--;
			}
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}
		}
		else if((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))    //��С�ȡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_fore_right_runed++;
			}
			else           //���У�����-
			{
				leg_fore_right_runed--;
			}
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}				
			}			
		}
		if(((arm_fore_right_runed==0)&&((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag)))||
			((leg_fore_right_runed==0)&&((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
		
  }
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	TIM10_Stop();          //�رն�ʱ��
}


/*********************************************************************
*������       ��Hand_Hang_3_4()
*��������     ��3/4�ŵ��ҵ����������֫
*����         ��dir:����ڣ�pulse��������
*���         ����
***********************************************************************/
void Hand_Hang_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flagΪ����ߵͱ�־λ
	static int k=0;	
    Hang_Init();          //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR3=0;  	  //1��ת��2��ת
		HANG_DIR4=1;
	}
	else
	{
		HANG_DIR3=1;  	  //1��ת��2��ת
		HANG_DIR4=0;
	}  
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM3=flag; 
		HANG_PWM4=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}			
		}
		//�����Ҹ첲����ָ��
		if(1==arm_fore_right_flag)            //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_right_runed++;
			}
			else           //���У�����-
			{
				arm_right_runed--;
			}
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if(1==leg_fore_right_flag)       //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_right_runed++;
			}
			else           //���У�����-
			{
				leg_right_runed--;
			}
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}
		
		else if((1==arm_post_right_flag)||(1==arm_fore_post_right_flag))    //�Ҵ�ۡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_post_right_runed++;
			}
			else           //���У�����-
			{
				arm_post_right_runed--;
			}
			j=arm_post_right_runed/(arm_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))    //�Ҵ��ȡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_post_right_runed++;
			}
			else           //���У�����-
			{
				leg_post_right_runed--;
			}
			j=leg_post_right_runed/(leg_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}


		if(((arm_right_runed==0)&&(1==arm_fore_right_flag))||
			((0==leg_right_runed)&&(1==leg_fore_right_flag))||
			((arm_post_right_runed==0)&&((1==arm_post_right_flag)||(1==arm_fore_post_right_flag)))||
			((0==leg_post_right_runed)&&((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
   }
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Hand_Hang_1_3()
*��������     ��1/3�ŵ��ҵ������������֫С��/С��
*����         ��m:����ڣ�pulse�������
*���         ����
***********************************************************************/
void Hand_Hang_1_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flagΪ����ߵͱ�־λ
	static int k=0;		
    Hang_Init();          //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR1=1;  	  //1��ת��2��ת
		HANG_DIR3=0; 
	}
	else
	{
		HANG_DIR1=0;  	  //1��ת��2��ת
		HANG_DIR3=1; 
	}

	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
		
		HANG_PWM1=flag; 
		HANG_PWM3=flag; 
	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			} 
			if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//�������Ҹ첲����ָ��
		if((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag))          //����С�ۡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_fore_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_right_runed--;
			}
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}
		}
		else if((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))     //����С�ȡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_fore_left_right_runed++;
			}
			else           //���У�����-
			{
				leg_fore_left_right_runed--;
			}
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}				
			}			
		}
		if(((arm_fore_left_right_runed==0)&&((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_fore_left_right_runed)&&((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}			
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM3=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Hand_Hang_1_2_3_4()
*��������     ��1/2/3/4�ŵ��ҵ������������֫
*����         ��dir:����ڣ�pulse��������
*���         ����
***********************************************************************/
void Hand_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flagΪ����ߵͱ�־λ
	static int k=0;	
    Hang_Init();          //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR1=1;
		HANG_DIR2=0;  	  //1��ת��2��ת
		HANG_DIR3=0;
		HANG_DIR4=1; 
	}
	else
	{
		HANG_DIR1=0;
		HANG_DIR2=1;  	  //1��ת��2��ת
		HANG_DIR3=1;
		HANG_DIR4=0; 
	}	  
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM1=flag;
	    HANG_PWM2=flag;
	    HANG_PWM3=flag;
		HANG_PWM4=flag; 	  
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;			
			if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightDownHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))
			{			
				u2_printf("break_flag==1");
				break;
			} 
			else if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");       //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
			}
		}
		//�������Ҹ첲����ָ��
		if(1==arm_fore_left_right_flag)          //����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_left_right_runed--;
			}
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if(1==leg_fore_left_right_flag)     //����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_left_right_runed++;
			}
			else           //���У�����-
			{
				leg_left_right_runed--;
			}
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}
		else if((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag))  //���Ҵ�ۡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_post_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_post_left_right_runed--;
			}
			j=arm_post_left_right_runed/(arm_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))  //���Ҵ��ȡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_post_left_right_runed++;
			}
			else           //���У�����-
			{
				leg_post_left_right_runed--;
			}
			j=leg_post_left_right_runed/(leg_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}		
		if(((arm_left_right_runed==0)&&(1==arm_fore_left_right_flag))||
			((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))||
			((arm_post_left_right_runed==0)&&((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_post_left_right_runed)&&((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}		
  }
	HANG_PWM1=0; 
	HANG_PWM2=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM3=0; 
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}


/*********************************************************************

						���Ҵ��ں���

***********************************************************************/

/*********************************************************************
*������       ��Uart_Auto_Hang_1()
*��������     ��1�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Uart_Auto_Hang_1(u16 dir,u32 pulse)
{
  Hang_Init();                      //�������ڳ�ʼ������
	int i,flag=1;                     //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	if(1==dir)
	{
		HANG_DIR1=1;                    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת		
	}
	else
	{
		HANG_DIR1=0;                    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת		
	}
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
    TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ			
		HANG_PWM1=flag;              //1�ŵ��ҵ����������ڸߵ�ƽ 
		if(arm_fore_left_flag==1)    //��첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_fore_left_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_runed--;
			}
			//������첲���Ҷ���ָ��
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}									
		}
		else if( leg_fore_left_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_fore_left_runed++; 
			}
			else           //���У�����-
			{
				leg_fore_left_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}		
			}
		}
	}
	HANG_PWM1=0;                      //1�ŵ��ҵ����Ӧ�������������  
	TIM10_Stop();                     //�رն�ʱ��
}
/*********************************************************************
*������       ��Uart_Auto_Hang_3()
*��������     ��3�ŵ��ҵ������
*����         ��dir(����:1��0��)��n(������)
*���         ����
***********************************************************************/
void Uart_Auto_Hang_3(u16 dir,u32 pulse)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	if(1==dir)
	{
		HANG_DIR3=0;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת   		
	}
 	else
	{
		HANG_DIR3=1;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת   		
	}	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);				
	USART2_RX_LEN=0;  
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM3=flag;          //3�ŵ��ҵ����������ڸߵ�ƽ
		if(arm_fore_right_flag==1)    //�Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_fore_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_right_runed--;
			}			
			//������첲���Ҷ���ָ��
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}									
		}
		if( leg_fore_right_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_fore_right_runed++; 
			}
			else           //���У�����-
			{
				leg_fore_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}		
			}
		}		 
	}
	HANG_PWM3=0;              //3�ŵ��ҵ����Ӧ�������������  
	TIM10_Stop();             //�رն�ʱ��
}
/*********************************************************************
*������       ��Uart_Auto_Hang_1_2()
*��������     ��1/2�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Uart_Auto_Hang_1_2(u16 dir,u32 pulse)
{
	Hang_Init();        //�������ڳ�ʼ������
	int i,flag=1;       //flagΪ����ߵͱ�־λ
	u8 j=0,k=0;
    if(dir==1)
     {
		HANG_DIR1=1;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR2=0;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    else
     {
		HANG_DIR1=0;   //1�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR2=1;   //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
    TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��3
    for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)));    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag;
		if(arm_fore_left_flag==1)    //��첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_left_runed++;
			}
			else           //���У�����-
			{
				arm_left_runed--;
			}
			//������첲���Ҷ���ָ��	
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}			
			}	
		}
		if( leg_fore_left_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_left_runed++; 
			}
			else           //���У�����-
			{
				leg_left_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}			
			}			
		}		
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM2=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Auto_Hang_1_3()
*��������     ��1/3�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Uart_Auto_Hang_1_3(u16 dir,u32 pulse)
{
	Hang_Init();        //�������ڳ�ʼ������
	int i,flag=1;       //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	if(dir==1)
     {
		HANG_DIR1=1;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR3=0;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    else
     {
		HANG_DIR1=0;   //1�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR3=1;   //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	 
	TIM10_Init(motor_hang_freq,motor_timer_freq);//�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM1=flag;   //1�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM3=flag;   //2�ŵ��ҵ����������ڸ�/�͵�ƽ	
 		if(arm_fore_left_right_flag==1)    //���Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_fore_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_right_runed--;
			}
			//������첲���Ҷ���ָ��
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}									
		}
		if( leg_fore_left_right_flag==1)    //������
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_fore_left_right_runed++; 
			}
			else           //���У�����-
			{
				leg_fore_left_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
						
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)	
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}		
			}
		}  
	}
	HANG_PWM1=0;          //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM3=0;          //3�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();         //�رն�ʱ��
}


/*********************************************************************
*������       ��Uart_Auto_Hang_3_4()
*��������     ��3/4�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Uart_Auto_Hang_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	u8 j=0,k=0;	
	if(dir==1)
     {
		HANG_DIR3=0;    //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR4=1;    //4�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
    else
     {
		HANG_DIR3=1;   //3�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR4=0;   //4�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
     }
   
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	 
	TIM10_Init(motor_hang_freq,motor_timer_freq);//�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
			
		HANG_PWM3=flag;          //3�ŵ��ҵ����������ڸߵ�ƽ
		HANG_PWM4=flag;          //4�ŵ��ҵ����������ڸߵ�ƽ
		if(arm_fore_right_flag==1)    //�Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_right_runed++;
			}
			else           //���У�����-
			{
				arm_right_runed--;
			}
			//�����Ҹ첲���Ҷ���ָ��	
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}			
			}	
		}
		if( leg_fore_right_flag==1)    //����
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_right_runed++; 
			}
			else           //���У�����-
			{
				leg_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}			
			}			
		}		
	}
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Auto_Hang_1_2_3_4()
*��������     ��1/2/3/4�ŵ��ҵ������
*����         ��dir(����:1��0��)��pulse(������)
*���         ����
***********************************************************************/
void Uart_Auto_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	u8 j=0,k=0;
	if(dir==1)
    {
		HANG_DIR1=1;    //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR2=0;    //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR3=0;    //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR4=1;    //4�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
    }
    else
    {
		HANG_DIR1=0;   //1�ŵ��ҵ��ת��������ƣ��͵�ƽ��ת
		HANG_DIR2=1;   //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR3=1;   //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
		HANG_DIR4=0;   //4�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת
    }
  
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
     { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM1=flag;   //1�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM2=flag;   //2�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM3=flag;   //3�ŵ��ҵ����������ڸ�/�͵�ƽ
		HANG_PWM4=flag;   //4�ŵ��ҵ����������ڸ�/�͵�ƽ	
		if(arm_fore_left_right_flag==1)    //���Ҹ첲
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{
				arm_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_left_right_runed--;
			}
			//������첲���Ҷ���ָ��	
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}			
			}	
		}
		if( leg_fore_left_right_flag==1)    //������
		{
			//ͨ���������ж������ۼ�
			if(1==dir)     //���У�����+
			{				
				leg_left_right_runed++; 
			}
			else           //���У�����-
			{
				leg_left_right_runed--;
			}
			//�������ȵ��Ҷ���ָ��						
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			
			if(k!=j)
			{
				k=j;
				if(k==1)	
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)	
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}			
			}			
		}	  		 
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM2=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Hand_Hang_1()
*��������     ��1�ŵ��ҵ������֫С��
*����         ��dir:����ڣ�n�������
*���         ����
***********************************************************************/
void Uart_Hand_Hang_1(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;            //flagΪ����ߵͱ�־λ	
	static int k=0;	
    Hang_Init();               //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR1=1;  		   //1��ת��0��ת 
	}
	else
	{
		HANG_DIR1=0;  		   //1��ת��0��ת 
	}	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM1=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;				
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))||    
			   (strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftDownHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftDownHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
			   (strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("USART2_RX_BUF=%s",USART2_RX_BUF);
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(USART2_RX_BUF,0,len);   //�������
				USART2_RX_LEN=0; 
			}			
		}
		//������С�۶���ָ��
		if((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag))        //��С�ۡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_fore_left_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_runed--;
			}
			j=arm_fore_left_runed/(arm_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_15");
				}				
			}
		}
		else if((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))   //��С�ȡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_fore_left_runed++;
			}
			else           //���У�����-
			{
				leg_fore_left_runed--;
			}
			j=leg_fore_left_runed/(leg_fore_left_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_15");
				}				
			}			
		}
		
		
		if(((arm_fore_left_runed==0)&&((1==arm_fore_left_flag)||(1==arm_fore_post_left_flag)) )||
			((leg_fore_left_runed==0)&&((1==leg_fore_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}		
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Hand_Hang_1_2()
*��������     ��1/2�ŵ��ҵ����������֫
*����         ��m:����ڣ�pulse�������
*���         ����
***********************************************************************/
void Uart_Hand_Hang_1_2(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;     //flagΪ����ߵͱ�־λ
	static int k=0;
    Hang_Init();        //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR1=1;  	//1��ת��2��ת
		HANG_DIR2=0;
	}
	else
	{
		HANG_DIR1=0;  	//1��ת��2��ת
		HANG_DIR2=1;
	}	   
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
		
		HANG_PWM1=flag; 
		HANG_PWM2=flag; 
	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand"))) 	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(USART2_RX_BUF,0,len);   //�������
				USART2_RX_LEN=0; 
			}
		}
		//������첲����ָ��
		if(1==arm_fore_left_flag)        //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_left_runed++;
			}
			else           //���У�����-
			{
				arm_left_runed--;
			}
			j=arm_left_runed/(arm_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}
		else if((1==arm_post_left_flag)||(1==arm_fore_post_left_flag))   //���ۡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_post_left_runed++;
			}
			else           //���У�����-
			{
				arm_post_left_runed--;
			}
			j=arm_post_left_runed/(arm_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_5");
				}
			}
		}		
		//�������ȶ���ָ��
		else if(1==leg_fore_left_flag)   //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_left_runed++;
			}
			else           //���У�����-
			{
				leg_left_runed--;
			}
			j=leg_left_runed/(leg_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		else if((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))   //����ȡ����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_post_left_runed++;
			}
			else           //���У�����-
			{
				leg_post_left_runed--;
			}
			j=leg_post_left_runed/(leg_post_left_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_5");
				}
			}			
		}
		if(((arm_left_runed==0)&&(1==arm_fore_left_flag))||
			((arm_post_left_runed==0)&&((1==arm_post_left_flag)||(1==arm_fore_post_left_flag)))||
			((leg_left_runed==0)&&(1==leg_fore_left_flag))||
			((0==leg_post_left_runed)&&((1==leg_post_left_flag)||(1==leg_fore_post_left_flag))))
		{
			k=0;     
		}
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM2=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Hand_Hang_3()
*��������     ��3�ŵ��ҵ������֫С��
*����         ��m:����ڣ�n�������
*���         ����
***********************************************************************/
void Uart_Hand_Hang_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;          //flagΪ����ߵͱ�־λ	
	static int k=0;		
    Hang_Init();             //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR3=0;  		 //1��ת��2��ת 
	}
	else
	{
		HANG_DIR3=1;  		 //1��ת��2��ת 
	}
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM3=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}
		}
		//������С�۶���ָ��
		if((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag))         //��С�ۡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_fore_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_right_runed--;
			}
			j=arm_fore_right_runed/(arm_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Right_12");
				}				
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Right_13");
				}
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Right_15");
				}				
			}
		}
		else if((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))    //��С�ȡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_fore_right_runed++;
			}
			else           //���У�����-
			{
				leg_fore_right_runed--;
			}
			j=leg_fore_right_runed/(leg_fore_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Right_15");
				}				
			}			
		}
		if(((arm_fore_right_runed==0)&&((1==arm_fore_right_flag)||(1==arm_fore_post_right_flag)))||
			((leg_fore_right_runed==0)&&((1==leg_fore_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
		
  }
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	TIM10_Stop();          //�رն�ʱ��
}


/*********************************************************************
*������       ��Uart_Hand_Hang_3_4()
*��������     ��3/4�ŵ��ҵ����������֫
*����         ��dir:����ڣ�pulse��������
*���         ����
***********************************************************************/
void Uart_Hand_Hang_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flagΪ����ߵͱ�־λ
	static int k=0;	
    Hang_Init();          //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR3=0;  	  //1��ת��2��ת
		HANG_DIR4=1;
	}
	else
	{
		HANG_DIR3=1;  	  //1��ת��2��ת
		HANG_DIR4=0;
	}	 
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM3=flag; 
		HANG_PWM4=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}			
		}
		//�����Ҹ첲����ָ��
		if(1==arm_fore_right_flag)            //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_right_runed++;
			}
			else           //���У�����-
			{
				arm_right_runed--;
			}
			j=arm_right_runed/(arm_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if(1==leg_fore_right_flag)       //��С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_right_runed++;
			}
			else           //���У�����-
			{
				leg_right_runed--;
			}
			j=leg_right_runed/(leg_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}
		
		else if((1==arm_post_right_flag)||(1==arm_fore_post_right_flag))    //�Ҵ�ۡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_post_right_runed++;
			}
			else           //���У�����-
			{
				arm_post_right_runed--;
			}
			j=arm_post_right_runed/(arm_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Right_5");
				}
			}
		}
		else if((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))    //�Ҵ��ȡ��Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_post_right_runed++;
			}
			else           //���У�����-
			{
				leg_post_right_runed--;
			}
			j=leg_post_right_runed/(leg_post_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Right_5");
				}
			}			
		}


		if(((arm_right_runed==0)&&(1==arm_fore_right_flag))||
			((0==leg_right_runed)&&(1==leg_fore_right_flag))||
			((arm_post_right_runed==0)&&((1==arm_post_right_flag)||(1==arm_fore_post_right_flag)))||
			((0==leg_post_right_runed)&&((1==leg_post_right_flag)||(1==leg_fore_post_right_flag))))
		{
			k=0;     
		}		
   }
	HANG_PWM3=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Hand_Hang_1_3()
*��������     ��1/3�ŵ��ҵ������������֫С��/С��
*����         ��m:����ڣ�pulse�������
*���         ����
***********************************************************************/
void Uart_Hand_Hang_1_3(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flagΪ����ߵͱ�־λ
	static int k=0;		
    Hang_Init();          //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR1=1;  	  //1��ת��2��ת
		HANG_DIR3=0;
	}
	else
	{
		HANG_DIR1=0;  	  //1��ת��2��ת
		HANG_DIR3=1;
	}	   
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��3
	for(i=0;i<pulse;i++)
	{ 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
		
		HANG_PWM1=flag; 
		HANG_PWM3=flag; 
	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))	
			{			
				u2_printf("break_flag==1");
				break;
			} 
			if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}
		}
		//�������Ҹ첲����ָ��
		if((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag))          //����С�ۡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_fore_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_fore_left_right_runed--;
			}
			j=arm_fore_left_right_runed/(arm_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Arm_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Arm_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Arm_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Arm_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Arm_Left_Right_15");
				}				
			}
		}
		else if((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))     //����С�ȡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_fore_left_right_runed++;
			}
			else           //���У�����-
			{
				leg_fore_left_right_runed--;
			}
			j=leg_fore_left_right_runed/(leg_fore_left_right_lim/9);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_6");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_7");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_8");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_9");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_10");
				}
				else if(k==6)
				{
					u2_printf("Cartoon_Leg_Left_Right_11");
				}
				else if(k==7)
				{
					u2_printf("Cartoon_Leg_Left_Right_12");
				}			
				else if(k==8)
				{
					u2_printf("Cartoon_Leg_Left_Right_13");
				}			
				else if(k==9)
				{
					u2_printf("Cartoon_Leg_Left_Right_14");
				}
				else if(k==10)
				{
					u2_printf("Cartoon_Leg_Left_Right_15");
				}				
			}			
		}
		if(((arm_fore_left_right_runed==0)&&((1==arm_fore_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_fore_left_right_runed)&&((1==leg_fore_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}			
	}
	HANG_PWM1=0;           //1�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM3=0;           //2�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Hand_Hang_1_2_3_4()
*��������     ��1/2/3/4�ŵ��ҵ������������֫
*����         ��dir:����ڣ�pulse��������
*���         ����
***********************************************************************/
void Uart_Hand_Hang_1_2_3_4(u16 dir,u32 pulse)
{
	u8 len;
	int i,j,flag=1;       //flagΪ����ߵͱ�־λ
	static int k=0;	
    Hang_Init();          //�������ڳ�ʼ������
	if(1==dir)
	{
		HANG_DIR1=1;
		HANG_DIR2=0;  	  //1��ת��2��ת
		HANG_DIR3=0;
		HANG_DIR4=1; 
	}
	else
	{
		HANG_DIR1=0;
		HANG_DIR2=1;  	  //1��ת��2��ת
		HANG_DIR3=1;
		HANG_DIR4=0; 
	}	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	TIM10_Init(motor_hang_freq,motor_timer_freq);    //�򿪶�ʱ��10
	for(i=0;i<pulse;i++)
  { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
		HANG_PWM1=flag;
	    HANG_PWM2=flag;
	    HANG_PWM3=flag;
		HANG_PWM4=flag; 	  
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;			
			if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightDownHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))||
				(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand")))		
			{			
				u2_printf("break_flag==1");
				break;
			} 
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))    //�����յ�Stop,������ѭ��	
			{	}
			else
			{
				u2_printf("NotRun");       //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
				memset(USART2_RX_BUF,0,len);
				USART2_RX_LEN=0; 
			}
		}
		//�������Ҹ첲����ָ��
		if(1==arm_fore_left_right_flag)          //����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_left_right_runed--;
			}
			j=arm_left_right_runed/(arm_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if(1==leg_fore_left_right_flag)     //����С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_left_right_runed++;
			}
			else           //���У�����-
			{
				leg_left_right_runed--;
			}
			j=leg_left_right_runed/(leg_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}
		else if((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag))  //���Ҵ�ۡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				arm_post_left_right_runed++;
			}
			else           //���У�����-
			{
				arm_post_left_right_runed--;
			}
			j=arm_post_left_right_runed/(arm_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;    
				if(k==1)
				{
					u2_printf("Cartoon_Arm_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Arm_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Arm_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Arm_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Arm_Left_Right_5");
				}
			}
		}
		else if((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))  //���Ҵ��ȡ����Ҵ�С��
		{
			//ͨ���������ж������ۼ�
			if(1==dir)      //���У�����+
			{
				leg_post_left_right_runed++;
			}
			else           //���У�����-
			{
				leg_post_left_right_runed--;
			}
			j=leg_post_left_right_runed/(leg_post_left_right_lim/4);
			j=j+1;
			if(k!=j)
			{
				k=j;			
				if(k==1)
				{
					u2_printf("Cartoon_Leg_Left_Right_1");
				}
				else if(k==2)
				{
					u2_printf("Cartoon_Leg_Left_Right_2");
				}			
				else if(k==3)
				{
					u2_printf("Cartoon_Leg_Left_Right_3");
				}			
				else if(k==4)
				{
					u2_printf("Cartoon_Leg_Left_Right_4");
				}
				else if(k==5)
				{
					u2_printf("Cartoon_Leg_Left_Right_5");
				}
			}			
		}		
		if(((arm_left_right_runed==0)&&(1==arm_fore_left_right_flag))||
			((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))||
			((arm_post_left_right_runed==0)&&((1==arm_post_left_right_flag)||(1==arm_fore_post_left_right_flag)))||
			((0==leg_post_left_right_runed)&&((1==leg_post_left_right_flag)||(1==leg_fore_post_left_right_flag))))
		{
			k=0;     
		}		
  }
	HANG_PWM1=0; 
	HANG_PWM2=0;           //3�ŵ��ҵ����������ڵ͵�ƽ
	HANG_PWM3=0; 
	HANG_PWM4=0;           //4�ŵ��ҵ����������ڸߵ�ƽ 
	TIM10_Stop();          //�رն�ʱ��
}

/*********************************************************************
*������       ��Uart_Motor_6_2_START()
*��������     �������������ս�ǰ�Ƹ�������
*����         ��dir(���з���)��pulse(����������)
*���         ����
***********************************************************************/
void Uart_Motor_6_2_START(u8 dir,u32 pulse)
{
	
	Motor_Dir_Init();
	int i,flag=1;          //flagΪ����ߵͱ�־λ	
    DIR6_2=dir;            //������ս��������ת��������ƣ�1�ߵ�ƽ��ת��0�͵�ƽ��ת 

	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
    TIM10_Init(3600-1,motor_timer_freq);                       //�򿪶�ʱ��
//	if(((1==dir)&&(push_rod_tig_runed_pulse<push_rod_tig_pulse_lim))||((0==dir)&&(push_rod_tig_runed_pulse>0)))
//    {
		for(i=0;i<pulse;i++)
		{ 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag ;
			PWM6_2=flag;          //������ս����������������ڸ�/�͵�ƽ
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ		
			//�����������ж������ۼ�
			if(1==dir)           //���У�����+
			{
				push_rod_tig_runed_pulse++;
			}
			else                 //���У�����-
			{
				push_rod_tig_runed_pulse--;
			}
			//���Ͷ���ָ��
			if(push_rod_tig_runed_pulse==1)
			{
				u2_printf("Cartoon_Push_Rod_Tig_1");
			}
			if(push_rod_tig_runed_pulse==(u16)(pulse/2))
			{
				u2_printf("Cartoon_Push_Rod_Tig_2");
			}
			if(((push_rod_tig_runed_pulse==pulse)&&(1==dir))||((push_rod_tig_runed_pulse==pulse-1)&&(0==dir)))
			{
				u2_printf("Cartoon_Push_Rod_Tig_3");
			} 
		}
//	}
	PWM6_2=0;                      //������ս����������Ӧ�������������  
	TIM10_Stop();                  //�رն�ʱ��
}



/*********************************************************************
*������       ��MotorStart()
*��������     �������������ս�ǰ�Ƹ�������
*����         ��MotorID-���ID��dir-���򣬶�ʱ����װ��ֵarr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void MotorStart(u8 MotorID,u8 dir,u16 arr)
{
	switch(MotorID)
	{
		case 1:
			Motor_1_START(dir);
			break;
		case 2:
			Push_Rod_Start(dir);
			break;
		case 3:
			DIR3=dir;
			Motor_3_START(arr,motor_timer_freq);
			break;
		case 4:
			DIR4=dir;
			Motor_4_START(arr,motor_timer_freq);
			break;
		case 5:
			DIR5=dir;
			Motor_5_START(arr,motor_timer_freq);
			break;
		case 6:
			DIR6=dir;
			Motor_6_START(arr,motor_timer_freq);		
			break;
		case 7:
			DIR7=dir;
			Motor_7_START(arr,motor_timer_freq);
			break;
		case 8:
			RELAY6=1; 
			Uart_Push_Rod_Swash(dir,arr);   //1:��ϴ����Ƹ����,0:��ϴ����Ƹ��ջ�
			RELAY6=0; 
			break;
		case 9:
			RELAY6=1;             //�̵����õ�
			Uart_Motor_6_2_START(dir,arr);      //1:�����Ƹ����	,0:�����Ƹ�����
			RELAY6=0;  
			break;
		case 10:
			UartWashletTig(dir,13000,arr);		//1:��������ߵ������,0����
			break;
	}
}

/*********************************************************************
*������       ��Motor345Start(u8 dir,u16 M3Arr,u16 M4Arr,u16 M5Arr,u16 psc)
*��������     �������������ս�ǰ�Ƹ�������
*����         ��MotorID-���ID��dir-���򣬶�ʱ����װ��ֵarr(�Զ���װ��ֵ)��psc(��Ƶֵ)
*���         ����
***********************************************************************/
void Motor345Start(u8 dir,u16 M3Arr,u16 M4Arr,u16 M5Arr)
{
	Motor_Dir_Init();                     	 //�������ڳ�ʼ������
	TIM8_PWM_CHANNEL_2_START(M3Arr,motor_timer_freq);     //TIM8ͨ�� 2 PWM������� ��3�ŵ��
	TIM5_PWM_CHANNEL_4_START(M4Arr,motor_timer_freq);		 //TIM5ͨ�� 4 PWM������� ��4�ŵ��
	TIM8_PWM_CHANNEL_1_START(M5Arr,motor_timer_freq);     //TIM8ͨ�� 1 PWM������� ,5�ŵ��

	DIR3=dir;
	DIR4=dir;
	DIR5=dir;
}


/*********************************************************************
*������       ��MotorStop()
*��������     �������������ս�ǰ�Ƹ�������
*����         ��MotorID-���ID
*���         ����
***********************************************************************/ 
void MotorStop(u8 MotorID)
{
	switch(MotorID)
	{
		case 1:
			Motor_1_STOP();
		case 2:
			Push_Rod_Stop();
		case 3:
			Motor_3_STOP();
			break;
		case 4:
			Motor_4_STOP();
			break;
		case 5:
			Motor_5_STOP();
			break;
		case 6:
			Motor_6_STOP();	
			break;
		case 7:
			Motor_7_STOP();	
			break;
		case 8:
			Motor_3_STOP();
			Motor_4_STOP();
			Motor_5_STOP();
			break;
	}
}

/*********************************************************************
*������       ��WashletRun()
*��������     �����������
*����         ��
*���         ����
***********************************************************************/ 
void WashletRun(u8 dir,u16 TimArr,u16 TimPsc)
{
	u8 err=0;
	u8 i=0;
	u8 breakflg=0;
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))		//������������ִ��һ��Ҫ�ڷ���������ȸ�λʱ����
	{
//		if((washlet_flag==0)&&((1==GD6_Start)||(1==Motor6_Alm)))	//�жϳ�ʼλ�ù���Ƿ���λ
//		{
//			delay_us(100);
//			if(((1==GD6_Start)||(1==Motor6_Alm)))
//			{
//				err=1;
//				u2_printf("GD6SErr");
//				BeepRun(2,300);
//				//�˴�����д�¸�λ����....						
//			}
//		}
//		else
//		{err=0;}
		
		//��������µ�ִ��
		if(err==0)
		{
			MotorStart(6,!dir,motor_washlet_freq);
			TIM10_Init(TimArr,TimPsc); 
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				//�����λ
				if((0==GD6_End)&&(1==dir))
				{
					delay_us(100);
					if(0==GD6_End)
					{
						breakflg=1;
						u2_printf("GD6End"); 
						break ;
					}						
				}
				if((0==GD6_Start)&&(0==dir))
				{
					delay_us(100);
					if(0==GD6_Start)
					{
						breakflg=1;
						u2_printf("GD6Start"); 
						break ; 
					}
				}
			}		
			//����whileѭ�����жϹ����û�е�λ
			if(dir==0)
			{
				GDCheckDealy(GD6S,300);	//���GD6Start,�����û��������ʱ300ms	
				if(GDCheckAlm(GD6S,2))
				{
					//�˴�дδ��λ�ĺ���		
				}
			}		
			else if(dir==1)
			{
				GDCheckDealy(GD6E,300);
				if(GDCheckAlm(GD6E,2))
				{
					//�˴�дδ��λ�ĺ���		
				}
			}	
			MotorStop(6);	//6�ŵ��ֹͣ
			TIM10_Stop();		
			breakflg=0;
			//����ж���û�е�λ
			//if((GD6_Start==0)&&(dir==0))		//��λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))			
			{washlet_flag=0;}	 		//��־λ����
			//else if((GD6_End==0)&&(dir==1))	//����λ��
			else if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==dir))	//����λ��
			{washlet_flag=1;	}	
			else
			{
				washlet_flag=1;
				if(dir==0)
					{u2_printf("GD6_SFalse\r\n");}
				else
					{u2_printf("GD6_EFalse\r\n");}
			}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);			
		}
	}
	else{LedAlm(300,"Uart_Washlet_Auto_Interfere");}//��������������LED0/LED1��һ��
}

/*********************************************************************
*������       ��BodyLeftRun()
*��������     ������
*����         ��
*���         ����
***********************************************************************/ 
void BodyLeftRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 Angle)
{
	u8 err=0;
	u16 arr_now;
	static u8 motor5_run_flag; 
  u8 breakflag;
	
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{
//		if((body_left_flag==0)&&(body_right_flag==0))
//		{
//			if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
//			{
//				delay_us(100);
//				if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
//				{
//					err=1;
//					if(	GD3_Start==1)
//					{u2_printf("GD3SErr");BeepRun(2,300);}	
//					else if(GD4_Start==1)
//					{u2_printf("GD4SErr");BeepRun(2,300);}
//					else if(GD5_Start==1)
//					{u2_printf("GD4SErr");BeepRun(2,300);}
//				}
//			}
//			else
//			{err=0;}
//		}
//		else{err=0;}		
		if(err==0)
		{
			if((body_left_flag==0)&&(dir==1))  //�����λ����ʼ״̬����ִ������
			{
				body_left_flag=1;
				motor5_run_flag=1;
				u2_printf("BodyLeftStart");		delay_ms(200);		
				MotorStart(5,1,motor_body_freq);
				TIM10_Init(body_angle_to_arr(Angle),timer10_freq); 
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
				{
						//�����λ
//						if((0==GD5_Left_End)&&(1==dir))
//						{
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								//breakflag=1;
//								u2_printf("GD5LE"); 
//								break ;
//							}						
//						}
						//������ϡ��������
//						if(1==Motor5_Alm)       
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("BodyLeftOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;							
//							}	             
//						}													
				}	
				GDCheckDealy(GD5LE,300);
				if(GDCheckAlm(GD5LE,2))
				{
					//�˴�дδ��λ�ĺ���		
				}
				Motor_5_STOP();            //���5ֹͣ
				TIM10_Stop();              //��ʱ���ر�
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//���ò�������
			}	
			//����345�ŵ������	
			if((dir==1)||(motor5_run_flag==1))
			{
				if(body_angle_to_arr(Angle)>body_left_runed_arr)
				{
					u2_printf("345Up");
					motor5_run_flag=0;					
					body_left_dir_flag=1;
					Motor345Start(0,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_angle_to_arr(Angle)-body_left_runed_arr,timer10_freq);	
				}
			}
			else if(dir==0)
			{
				u2_printf("345Down");
				if(body_left_runed_arr>0)
				{
					Motor345Start(1,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_left_runed_arr,timer10_freq);	
				}
			}
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
			if(((body_left_runed_arr!=body_angle_to_arr(Angle))&&(1==dir))||((0!=body_left_runed_arr)&&(0==dir)))
			{
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  
				{
					//�����λ
						if(((0==GD3_Start)||(0==GD4_Start))&&(0==dir))
						{
							delay_us(100);
							if(0==GD3_Start)
							{
								u2_printf("GD3S"); 
								break ;
							}
							else if(0==GD4_Start)
							{
								u2_printf("GD4S"); 
								break ;
							}
						}
						else if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==dir))
						{
							delay_us(100);
							if(0==GD3_Left_End)
							{
								u2_printf("GD3LE"); 
								break ;
							}
							else if(0==GD4_Left_End)
							{
								u2_printf("GD4LE"); 
								break ;
							}
						}									
						if(UsartCheck2("BodyLeftUpNew","BodyLeftDownNew"))
						{
							u2_printf("Stop\r\n");
							breakflag=1;
							break;
						}
				}				
				if((breakflag==0)||(dir==0))
				{
					GDCheckDealy(GD34S,300);
					if(GDCheckAlm(GD34S,2))
					{
						//�˴�дδ��λ�ĺ���		
					}					
				}				
				MotorStop(8);	//3,4,5���ֹͣ	
				breakflag=0;				
			//�жϸ�λ			
			//if(((GD3_Start==0)||(GD4_Start)||(GD5_Start))&&(dir=0))
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir=0))
				{
					arr_now=0;
					body_left_flag=0;
				}	
				else
				{					
					arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now
					body_left_flag=1;
	//				W25QXX_Write((u8*)&body_left_flag,33,1);														
				}
				//ͨ���������ж������ۼ�
				if(dir==1)           //���У�����++
				{
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
						u2_printf("LeftEnd\r\n");
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);
					}
					else
					{
						body_left_runed_arr=body_left_runed_arr+arr_now;
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);
					}		
				}	
				else     //���У�����-
				{			
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_left_runed_arr=0;
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);
					}
					else
					{
						body_left_runed_arr=body_left_runed_arr-arr_now;	
						u2_printf("body_left_runed_arr=%d\r\n",body_left_runed_arr);						
					}
				}					
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //����жϱ�־λ				

				if(body_left_flag==0) 
				{
					Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
					u2_printf("5�ŵ����ʼ��λ");
					MotorStart(5,0,motor_body_freq);
					
					body_left_runed_arr=0;
					TIM10_Init(body_angle_to_arr(Angle),timer10_freq);                     //�򿪶�ʱ��				
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
					{
						if((0==GD5_Start)&&(0==dir))
						{
							delay_us(100);
							if(0==GD5_Start)
							{
								//breakflag=1;
								u2_printf("GD5S"); 
								break ;
							}						
						}
					}
					GDCheckDealy(GD5S,300);
					if(GDCheckAlm(GD5S,2))
					{
						//�˴�дδ��λ�ĺ���		
					}							
					Motor_5_STOP();       //���ֹͣ
					TIM10_Stop();         //�رն�ʱ��
					breakflag=0;         //�����־λ
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					delay_ms(200);
					u2_printf("body_left_flag==0");
					delay_ms(200);
					u2_printf("BodyLeftRes");
					delay_ms(200);
				}
			}	
		}
	}
	else
	{		LedAlm(300,"BodyLeftInterfere");	}
}

/*********************************************************************
*������       ��BodyRightRun()
*��������     ������
*����         ��
*���         ����
***********************************************************************/ 
void BodyRightRun(u8 dir,u16 M3Arr,u16 M4Arr,u16 Angle)
{
	u8 err=0;
	u16 arr_now;
	static u8 motor5_run_flag; 
  u8 breakflag;
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{
		if((body_left_flag==0)&&(body_right_flag==0))
		{
			if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
			{
				delay_us(100);
				if((GD3_Start==1)||(GD4_Start==1)||(GD5_Start==1)||(1==Motor5_Alm))
				{
					err=1;
					if(	GD3_Start==1)
					{u2_printf("GD3SErr");BeepRun(2,300);}	
					else if(GD4_Start==1)
					{u2_printf("GD4SErr");BeepRun(2,300);}
					else if(GD5_Start==1)
					{u2_printf("GD4SErr");BeepRun(2,300);}
				}
			}
			else
			{err=0;}
		}
		else{err=0;}		
		if(err==0)
		{
			if((body_left_flag==0)&&(dir==1))  //�����λ����ʼ״̬����ִ������
			{
				body_left_flag=1;
				motor5_run_flag=1;
				u2_printf("BodyRightStart");		delay_ms(200);		
				MotorStart(5,0,motor_body_freq);
				TIM10_Init(body_angle_to_arr(Angle),timer10_freq); 
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
				{
						//�����λ
						if((0==GD5_Right_End)&&(1==dir))
						{
							delay_us(100);
							if(0==GD5_Right_End)
							{
								breakflag=1;
								u2_printf("GD5RE"); 
								break ;
							}						
						}
						//������ϡ��������
//						if(1==Motor5_Alm)       
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("BodyLeftOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;							
//							}	             
//						}													
				}	
				GDCheckDealy(GD5RE,300);
				Motor_5_STOP();            //���5ֹͣ
				TIM10_Stop();              //��ʱ���ر�
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);	//���ò�������
			}	
			//����345�ŵ������	
			if((dir==1)||(motor5_run_flag==1))
			{
				if(body_angle_to_arr(Angle)>body_right_runed_arr)
				{
					u2_printf("345Up");
					motor5_run_flag=0;					
					body_right_dir_flag=1;
					Motor345Start(1,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_angle_to_arr(Angle)-body_right_runed_arr,timer10_freq);	
				}
			}
			else if(dir==0)
			{
				u2_printf("345Down");
				if(body_left_runed_arr>0)
				{
					Motor345Start(1,M3Arr,M4Arr,motor_body_freq);
					TIM10_Init(body_right_runed_arr,timer10_freq);	
				}
			}
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
			if(((body_right_runed_arr!=body_angle_to_arr(Angle))&&(1==dir))||((0!=body_right_runed_arr)&&(0==dir)))
			{
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  
				{
					//�����λ
						if(((0==GD3_Start)||(0==GD4_Start))&&(0==dir))
						{
							delay_us(100);
							if(0==GD3_Start)
							{
								breakflag=1;
								u2_printf("GD3S"); 
								break ;
							}
							else if(0==GD4_Start)
							{
								breakflag=1;
								u2_printf("GD4S"); 
								break ;
							}
						}
						else if(((0==GD3_Right_End)||(0==GD4_Right_End))&&(1==dir))
						{
							delay_us(100);
							if(0==GD3_Right_End)
							{
								u2_printf("GD3RE"); 
								break ;
							}
							else if(0==GD4_Right_End)
							{
								u2_printf("GD4RE"); 
								break ;
							}
						}									
						if(UsartCheck2("BodyRightUpNew","BodyRightDownNew"))
						{
							u2_printf("Stop\r\n");
							breakflag=1;
							break;
						}
				}				
				if((breakflag==0)||(dir==0))
				{
					GDCheckDealy(GD34S,300);
				}				
				MotorStop(8);	//3,4,5���ֹͣ
				breakflag=0;				
			//�жϸ�λ			
			//if(((GD3_Start==0)||(GD4_Start)||(GD5_Start))&&(dir=0))
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir=0))
				{
					arr_now=0;
					body_right_flag=0;
				}	
				else
				{					
					arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now
					body_right_flag=1;
	//				W25QXX_Write((u8*)&body_left_flag,33,1);														
				}
				//ͨ���������ж������ۼ�
				if(dir==1)           //���У�����+
				{
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_right_runed_arr=body_angle_to_arr(Angle);
						u2_printf("RightEnd\r\n");
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);
					}
					else
					{
						body_right_runed_arr=body_right_runed_arr+arr_now;
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);
					}		
				}	
				else     //���У�����-
				{			
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{
						body_right_runed_arr=0;
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);
					}
					else
					{
						body_right_runed_arr=body_right_runed_arr-arr_now;	
						u2_printf("body_right_runed_arr=%d\r\n",body_right_runed_arr);						
					}
				}					
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //����жϱ�־λ				

				if(body_right_flag==0) 
				{
					Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);
					u2_printf("5�ŵ����ʼ��λ");
					MotorStart(5,1,motor_body_freq);
					
					body_left_runed_arr=0;
					TIM10_Init(body_angle_to_arr(Angle),timer10_freq);                     //�򿪶�ʱ��				
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
					{
						if((0==GD5_Start)&&(0==dir))
						{
							delay_us(100);
							if(0==GD5_Start)
							{
								//breakflag=1;
								u2_printf("GD5S"); 
								break ;
							}						
						}
					}
					GDCheckDealy(GD5S,300);
					Motor_5_STOP();       //���ֹͣ
					TIM10_Stop();         //�رն�ʱ��
					breakflag=0;         //�����־λ
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					delay_ms(200);
					u2_printf("body_right_flag==0");
					delay_ms(200);
					u2_printf("BodyRightRes");
					delay_ms(200);
				}
			}	
		}
	}
	else
	{		LedAlm(300,"BodyRightInterfere");	}
}



/*********************************************************************
*������       ��DeskRun()
*��������     ��С����
*����         ��
*���         ����
***********************************************************************/ 
void DeskRun(u8 dir,u16 HalfDist)
{
	u8 err=0;
	u8 breakflg=0;
	u16 arr_now; 
//	DeskLastJudgeFlg=0;
	
//	DeskSwitch=0;
	
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{
//		if((desk_flag==0)&&((1==GD7_Start)||(1==Motor7_Alm)))
//		{
//			delay_us(100);
//			if((1==GD7_Start)||(1==Motor7_Alm))
//			{	err=1;
//				u2_printf("GD67SErr");
//				BeepRun(2,300);
//				//�˴�����д�¸�λ����....		
//			}
//			else
//			{err=0;}
//		}
//		else
//		{err=0;}
		
		if(err==0)
		{
			if(dir==1)
			{
				u2_printf("DeskUp\r\n");
				if(desk_distance_to_arr(HalfDist)>desk_runed_arr)
				{
					MotorStart(7,dir,motor_desk_freq);
					TIM10_Init(desk_distance_to_arr(HalfDist)-desk_runed_arr,timer10_freq); 				
				}
			}
			else if(dir==0)
			{
				u2_printf("DeskDown\r\n");
				if(desk_runed_arr>0)
				{
					MotorStart(7,dir,motor_desk_freq);
					TIM10_Init(desk_runed_arr,timer10_freq); 				
				}
			}
			
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;	
			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);	
		 if(((desk_runed_arr!=desk_distance_to_arr(HalfDist))&&(1==dir))||((0!=desk_runed_arr)&&(0==dir)))	
		 { 
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					//�����λ
//					if((0==GD7_End)&&(1==dir))
//					{
//						delay_us(100);
//						if(0==GD6_End)
//						{
//							breakflg=1;
//							u2_printf("GD7End"); 
//							break ;
//						}						
//					}
//					if((0==GD7_Start)&&(0==dir))
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							breakflg=1;
//							u2_printf("GD7Start"); 
//							break ; 
//						}
//					}
						if(UsartCheck2("DeskRunUpNew","DeskRunDownNew"))
						{
							u2_printf("Stop");
							breakflg=2;
							break;
						}
				}

				if((breakflg==0)&&(Desk1st2st==1)&&(dir==1))
				{
					DeskSwitch=1;
					Desk1st2st=2;
					arr_now=0;
					desk_runed_arr=0;
					u2_printf("1�л�2\r\n");
					DeskRun(dir,HalfDist);
				}
				else if((breakflg==0)&&(Desk1st2st==2)&&(dir==0))
				{
					DeskSwitch=1;
					Desk1st2st=1;
					desk_runed_arr=desk_distance_to_arr(HalfDist);
					u2_printf("2�л�1\r\n");
					//arr_now=desk_distance_to_arr(HalfDist);
					DeskRun(dir,HalfDist);
				}
				else
				{u2_printf("���л�\r\n");}
				
				if(breakflg!=2)
				{
					if(dir==0)
					{GDCheckDealy(GD7S,300);}
					else if(dir==1)
					{GDCheckDealy(GD7E,300);}
				}	
				
				
				Motor_7_STOP();     //���ֹͣ
				TIM10_Stop();       //��ʱ���ر�
				breakflg=0;
				
				
				if((DeskSwitch==0)||(DeskLastJudgeFlg==0))
				{	
					DeskLastJudgeFlg=1;
					DeskSwitch=0;
					//����ж���û�е�λ
					//if((GD7_Start==0)&&(dir==0))		//��λ
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))			
					{
						desk_flag=0;	 			//��־λ����
						arr_now=0;  
					}
					else
					{
						arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now		
						desk_flag=1;
					}
					
					if(dir==1)
					{
						//if(0==GD7_End)
						if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
						{
							desk_runed_arr=desk_distance_to_arr(HalfDist);
							u2_printf("DeskLim");
							delay_ms(200);				
						}
						else
						{  desk_runed_arr=desk_runed_arr+arr_now;	}
					}
					else if(dir==0)
					{	//if(0==GD7_Start)
						if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
						{
							desk_runed_arr=0;			
							u2_printf("DeskRes");
							delay_ms(200);
						}
						else
						{	desk_runed_arr=desk_runed_arr-arr_now;	}

					}
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); 
					u2_printf("Desk1st2st=%d",Desk1st2st);
				}
				else
				{DeskLastJudgeFlg=1;
					
				}
			}
		 else
		 {u2_printf("ERR\r\n");}
		}
	}
	else
	{		LedAlm(300,"DeskInterfere");	}
}



void DeskRun1(u8 dir,u16 Dist)
{
	u8 err=0;
	u8 breakflg=0;
	u16 arr_now; 

	
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{
//		if((desk_flag==0)&&((1==GD7_Start)||(1==Motor7_Alm)))
//		{
//			delay_us(100);
//			if((1==GD7_Start)||(1==Motor7_Alm))
//			{	err=1;
//				u2_printf("GD7SErr");
//				BeepRun(2,300);
//				//�˴�����д�¸�λ����....		
//			}
//			else
//			{err=0;}
//		}
//		else
//		{err=0;}
		
		if(err==0)
		{	
			if(desk_flag==0)
			{
				if(dir==1)
				{
					u2_printf("��һ�׶ο�ʼ\r\n");
					MotorStart(7,!dir,motor_desk_freq);
					TIM10_Init((u16)(desk_distance_to_arr(Dist)/2.6),timer10_freq);
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
					{
						
					}
					Motor_7_STOP();     //���ֹͣ
					TIM10_Stop();       //��ʱ���ر�
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					u2_printf("���7��һ�׶��������\r\n");
				}
			}
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
			
			if((dir==1))
			{
				u2_printf("DeskUp\r\n");
				if(desk_distance_to_arr(Dist)>desk_runed_arr)
				{
					MotorStart(7,!dir,motor_desk_freq);
					TIM10_Init(desk_distance_to_arr(Dist)-desk_runed_arr,timer10_freq); 				
				}
			}
			else if(dir==0)
			{
				if(desk_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
				{
					MotorStart(7,!dir,motor_desk_freq);
					TIM10_Init(desk_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
				}
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
			if(((desk_runed_arr!=desk_distance_to_arr(Dist))&&(1==dir))||((0!=desk_runed_arr)&&(0==dir)))
			{
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					//�����λ											   						
//					if((0==GD7_End)&&(1==dir))
//					{
//						delay_us(100);
//						if(0==GD7_End)
//						{
//							u2_printf("GD7End");					
//							break;					
//						}								
//					}

					
					if(UsartCheck2("DeskRunUpNew","DeskRunDownNew"))
					{
						u2_printf("Stop");
						breakflg=1;
						break;
					}			
				}
				
				if(breakflg==0)
				{
					if(dir==1)
					{
						GDCheckDealy(GD7E,300);
						if(GDCheckAlm(GD7E,2))
						{
							//�˴�дδ��λ�ĺ���		
						}	
					}
				}
				Motor_7_STOP();     //���ֹͣ
				TIM10_Stop();       //��ʱ���ر�
				breakflg=0;
	
				//�жϸ�λ
				//if((GD7_Start==0)&&(dir==0))
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
				{
					arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
					desk_flag=0;
				}
				else
				{
					arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
					desk_flag=1;
				}
				
				//ͨ���������ж������ۼ�
				if(dir==1)        //�����С����ǰ��������+
				{
					//if(0==GD7_End)
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{ 					
						desk_runed_arr=desk_distance_to_arr(Dist);
						u2_printf("DeskLim");
						delay_ms(200);
					}
					else
					{  desk_runed_arr=desk_runed_arr+arr_now;	}				
				}
				else if(dir==0)                 //�����С���Ӻ��ˣ�����-
				{
					//if(0==GD7_Start)
					if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
					{						
						desk_runed_arr=0; 
						u2_printf("DeskRes");
						delay_ms(200);
					}
					else
					{	 
						desk_runed_arr=desk_runed_arr-arr_now;
					}						
				}				
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				u2_printf("desk_runed_arr=%d",desk_runed_arr);
				
				if(desk_flag==0)
				{
					MotorStart(7,1,motor_desk_freq);
					desk_runed_arr=0;
					Motor_7_START(motor_desk_freq,motor_timer_freq);
					TIM10_Init((u16)(desk_distance_to_arr(Dist)/2.6),timer10_freq);
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
					{
						if((0==GD7_Start)&&(0==dir))
						{
//							delay_us(100);
//							if(0==GD7_Start)
//							{
//								u2_printf("GD7Start");					
//								break;						
//							}					
						}						
					}				
					if(dir==0)
					{
						GDCheckDealy(GD7S,300);
						if(GDCheckAlm(GD7S,2))
						{
							//�˴�дδ��λ�ĺ���		
						}	
					}
					Motor_7_STOP();     //���ֹͣ
					TIM10_Stop();       //��ʱ���ر�
					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
					
					u2_printf("���7��һ�׶η������");					
					}
			}
		}
	}
	else
	{		LedAlm(300,"DeskInterfere");	}
}















