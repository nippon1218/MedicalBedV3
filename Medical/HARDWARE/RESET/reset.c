#include "sys.h"
#include "reset.h"
#include "function.h"
#include "motor.h"
#include "delay.h"
#include "pump.h"



void ResetAll(void)
{
	lock_flag=1;
	
	if(muscle_massager_flag==1)
	{
		Muscle_Massager();		
	}
	
	
	if(body_left_flag==1)
	{
		if(back_nursing_left_flag==1)
		{
			WriteInUART2("BackNursingLeftPhone");
			Uart_Back_Nursing_Left();           //����λ
			delay_ms(100);			
			WriteInUART2("BodyLeftDownPhone");
			Uart_Body_Left();           //����λ
			delay_ms(100);				
		
//			MotorStart(4,1,(u16)(motor_body_freq*1.2));
//			TIM10_Init(body_left_runed_arr,timer10_freq);          //�򿪶�ʱ��
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);   //����жϱ�־λ	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
//			{
//				if((0==GD4_Start))
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");
//						break;				
//					}	
//				}
//			}	
//			Motor_4_STOP();     //���ֹͣ
//			TIM10_Stop();       //��ʱ���ر�
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
		}
		else if(waist_nursing_left_flag==1)
		{
			WriteInUART2("WaistNursingLeftPhone");
			Uart_Waist_Nursing_Left();           //����λ
			delay_ms(100);			
			WriteInUART2("BodyLeftDownPhone");
			Uart_Body_Left();           //����λ
			delay_ms(100);	
	
//			MotorStart(3,1,motor_body_freq);
//			TIM10_Init(body_left_runed_arr,timer10_freq);          //�򿪶�ʱ��
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);   //����жϱ�־λ	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
//			{
//				if((0==GD3_Start))
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("GD3Start");
//						break;				
//					}	
//				}
//			}	
//			Motor_3_STOP();     //���ֹͣ
//			TIM10_Stop();       //��ʱ���ر�
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
		}
		else
		{
			WriteInUART2("BodyLeftDownPhone");
			Uart_Body_Left();           //����λ
			delay_ms(100);
		}
	}
	else if(body_right_flag==1)
	{
		if(back_nursing_right_flag==1)
		{
			WriteInUART2("BackNursingRightPhone");
			Uart_Back_Nursing_Right();           //����λ
			delay_ms(100);			
			WriteInUART2("BodyRightDownPhone");
			Uart_Body_Right();           //����λ
			delay_ms(100);			
				
//			MotorStart(4,0,motor_body_freq);
//			TIM10_Init(body_right_runed_arr,timer10_freq);          //�򿪶�ʱ��
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);   //����жϱ�־λ	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
//			{
//				if((0==GD4_Start))
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");
//						break;				
//					}	
//				}
//			}	
//			Motor_4_STOP();     //���ֹͣ
//			TIM10_Stop();       //��ʱ���ر�
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
		}
		else if(waist_nursing_right_flag==1)
		{
			WriteInUART2("WaistNursingRightPhone");
			Uart_Waist_Nursing_Right();           //����λ
			delay_ms(100);			
			WriteInUART2("BodyRightDownPhone");
			Uart_Body_Right();           //����λ
			delay_ms(100);
				
//			MotorStart(3,0,motor_body_freq);
//			TIM10_Init(body_right_runed_arr,timer10_freq);          //�򿪶�ʱ��
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);   //����жϱ�־λ	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
//			{
//				if((0==GD3_Start))
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("GD3Start");
//						break;				
//					}	
//				}
//			}	
//			Motor_3_STOP();     //���ֹͣ
//			TIM10_Stop();       //��ʱ���ر�
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
		}
		else
		{
			WriteInUART2("BodyRightDownPhone");
			Uart_Body_Right();           //����λ
			delay_ms(100);
		}
	}
	else if(back_flag==1)
	{
		WriteInUART2("BackDownPhone");
		Uart_Back();                //֧��
		delay_ms(100);
	}
	
	if(leg_up_flag==1)
	{
		WriteInUART2("LegUpDownPhone");
		Uart_Leg_Up();            //������	
		delay_ms(100);
	
	}
	else if(leg_down_flag==1)
	{
		WriteInUART2("LegDownUpPhone");
		Fun_Leg_Down();            //������	
		delay_ms(100);
	}
		
	if(washlet_flag==1)
	{
			u2_printf("�������ر�");
			washlet_picture_k=24;
			Uart_Washlet(1);		
	}
		
	if(desk_flag==1)
	{ 
		DeskRun1(0,100);
		delay_ms(100);	
	}
	

	
	FlagClear();
	
}



