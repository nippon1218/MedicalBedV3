#include "Hang.h"
#include "sys.h"
#include "usart.h"
#include "led.h"
#include "motor.h"
#include "function.h"
#include "pcf8574.h"
#include "delay.h"

void HangRun(u8 dir,u8 height,u8 hang1,u8 hang2,u8 hang3,u8 hang4)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	
	HANG_DIR1=dir;       //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽUp���͵�ƽDown
	HANG_DIR2=!dir;			 //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽDown���͵�ƽUp
	HANG_DIR3=dir;			 //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽUp���͵�ƽDown
	HANG_DIR4=!dir;		  	//4�ŵ��ҵ��ת��������ƣ��ߵ�ƽDown���͵�ƽUp
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(70-1,motor_timer_freq);                       //�򿪶�ʱ��3
	
    for(i=0;i<100000*height;i++)   //����ȷ��Ϊ3000000
    { 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag ;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
			if(hang1==1)
			{
				HANG_PWM1=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ
			}
			if(hang2==1)
			{
				HANG_PWM2=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ
			}
			if(hang3==1)
			{
				HANG_PWM3=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ
			}
			if(hang4==1)
			{
				HANG_PWM4=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ
			}
	  }
	HANG_PWM1=0;              //3�ŵ��ҵ����Ӧ������������� 
	HANG_PWM2=0; 
	HANG_PWM3=0; 
	HANG_PWM4=0; 
	TIM10_Stop();             //�رն�ʱ��
		u2_printf("%d\r\n",height);
			if(hang1==1)
			{
				u2_printf("hang1\r\n");
			}
			if(hang2==1)
			{
				u2_printf("hang2\r\n");
			}
			if(hang3==1)
			{
				u2_printf("hang3\r\n"); 
			}
			if(hang4==1)
			{
				u2_printf("hang4\r\n");
			}
}




void HangRunAuto(u8 dir,u8 turn,u8 ArmLeg,u8 times,u16 height)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;					
	
	HANG_DIR1=dir;       //1�ŵ��ҵ��ת��������ƣ��ߵ�ƽUp���͵�ƽDown
	HANG_DIR2=!dir;			 //2�ŵ��ҵ��ת��������ƣ��ߵ�ƽDown���͵�ƽUp
	HANG_DIR3=dir;			 //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽUp���͵�ƽDown
	HANG_DIR4=!dir;		  	//4�ŵ��ҵ��ת��������ƣ��ߵ�ƽDown���͵�ƽUp
	
	TIM10_Init(70-1,motor_timer_freq);                       //�򿪶�ʱ��3

	for(i=0;i<100000*height;i++)   //����ȷ��Ϊ3000000
	{
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );
		//flag=!flag;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); 
		if(turn==0)
		{
			HANG_PWM2=!HANG_PWM2;
			if(i%times==0)
			{HANG_PWM1=!HANG_PWM1;}
			
			if(ArmLeg==0)	//�ۣ�0
			{
				if(1==dir)
				{arm_left_runed++;}
				else
				{arm_left_runed--;}
				
				j=arm_left_runed/(100000*height/4);
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
			else		//�ȣ�1
			{
				if(1==dir)
				{leg_left_runed++;}
				else
				{leg_left_runed--;}
				
				j=leg_left_runed/(100000*height/4);
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
		else
		{
			HANG_PWM3=!HANG_PWM3;
			if(i%times==0)
			{HANG_PWM4=!HANG_PWM4  ;}
			
			if(ArmLeg==0)	//�ۣ�0
			{
				if(1==dir)
				{arm_right_runed++;}
				else
				{arm_right_runed--;}
				
				j=arm_right_runed/(100000*height/4);
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
			else		//�ȣ�1
			{
				if(1==dir)
				{leg_right_runed++;}
				else
				{leg_right_runed--;}
				
				j=leg_right_runed/(100000*height/4);
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

	}
	HANG_PWM1=0;              //3�ŵ��ҵ����Ӧ������������� 
	HANG_PWM2=0; 
	HANG_PWM3=0; 
	HANG_PWM4=0; 		
	TIM10_Stop();             //�رն�ʱ��
	u2_printf("\r\ndir=%d,turn=%d,ArmLeg=%d,times=%d,height=%d\r\n",dir,turn,ArmLeg,times,height);
}

void Uart_Auto_Arm_Leg(u8 dir,u8 turn,u8 ArmLeg,u8 times,u16 height)
{
	int j;
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   { 
		 PCF8574_WriteBit(EXIO1,0);	
		 if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftAuto"))
		 {
			 if(leg_fore_left_flag==0)       //��ֹ�����󴥷������±�־λ��λ
				{	
					arm_fore_left_flag=1;
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmLeftAutoStart");				
				}
		 }
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftAuto"))    //����
		{
			if(arm_fore_left_flag==0)      //��ֹ�����󴥷������±�־λ��λ
			{	
				leg_fore_left_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegLeftAutoStart");
				delay_ms(200);
			}			
		}		 
		HangRunAuto(1,0,0,2,15);
		delay_ms(1000);	 
		for(j=0;j<1;j++)                   //����t����֫����ѵ��
		{
			Uart_Auto_Hang_1(1,75000);     //�����˶�
			delay_ms(1000);
			Uart_Auto_Hang_1(0,75000);     //�����˶�
			delay_ms(1000);	
		}	
		 
		 
	 }


}



