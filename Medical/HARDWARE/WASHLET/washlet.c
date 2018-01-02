#include "sys.h"
#include "usart.h"
#include "washlet.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "delay.h"
#include "pump.h"

void WashLetAuto_V1(u8 dir,u16 arr)
{



}


/***********************************************************************
 ������      ��Uart_Washlet(void)  
 ��������    ������ִ������������
 ����        ��dir: 1(������)��0���ر����㣩
 ���        ����
                           
************************************************************************/
void WashLet_V1(u8 dir,u16 arr)
{
	u16 arr_send;
	u16 j=0;	
	static u16 k=0; 
	static u8 kj;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;

	washlet_flag=1;
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{
		DIR6=!dir;		//DIR6=0ʱ�򿪣�DIR6=1ʱ�ر�
		u2_printf("��������ʼ����");
		Motor_6_START(motor_washlet_freq,motor_timer_freq);           //�������
		TIM10_Init(arr,timer10_freq);                     //�򿪶�ʱ��35000
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ
		if(1==dir)
		{
			washlet_flag=1;	
			u2_printf("Cartoon_Washlet_1");
			delay_ms(100);  
		}
		else
		{
			washlet_picture_k=24;
			u2_printf("Cartoon_Washlet_25");	
			delay_ms(100);
		}
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		{	
			//�����λ
//				if((0==GD6_End)&&(0==dir))
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						break_flag=1;
//						u2_printf("GD6End"); 
//						break ;
//					}						
//				}
//				if((0==GD6_Start)&&(1==dir))
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						break_flag=1;
//						u2_printf("GD6Start"); 
//						break ; 
//					}
//				}
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
				}
			
				//������ϡ��������
//				if(1==Motor6_Alm)     
//				{
//					delay_us(100);
//					if(1==Motor6_Alm)  
//					{
//						washlet_auto_overload=1;
//						u2_printf("WashletAutoOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;		
//					}						
//				}	
				
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(arr/24);
			if(0==dir)
			{
				j=24-j;
			}
			k=washlet_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);				
				if(kj<2)
				{
					k=j;   washlet_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Washlet_2");
								break;
						case 2:	u2_printf("Cartoon_Washlet_3");
								break;					
						case 3:	u2_printf("Cartoon_Washlet_4");
								break;					
						case 4:	u2_printf("Cartoon_Washlet_5");
								break;	
						case 5:	u2_printf("Cartoon_Washlet_6");
								break;	
						case 6:	u2_printf("Cartoon_Washlet_7");
								break;
						case 7:	u2_printf("Cartoon_Washlet_8");
								break;	
						case 8:	u2_printf("Cartoon_Washlet_9");
								break;	
						case 9:	u2_printf("Cartoon_Washlet_10");
								break;
						case 10:u2_printf("Cartoon_Washlet_11");
								break;	
						case 11:u2_printf("Cartoon_Washlet_12");
								break;	
						case 12:u2_printf("Cartoon_Washlet_13");
								break;		
						case 13:u2_printf("Cartoon_Washlet_14");
								break;	
						case 14:u2_printf("Cartoon_Washlet_15");
								break;	
						case 15:u2_printf("Cartoon_Washlet_16");
								break;	
						case 16:u2_printf("Cartoon_Washlet_17");
								break;		
						case 17:u2_printf("Cartoon_Washlet_18");
								break;	
						case 18:u2_printf("Cartoon_Washlet_19");
								break;
						case 19:u2_printf("Cartoon_Washlet_20");
								break;	
						case 20:u2_printf("Cartoon_Washlet_21");
								break;	
						case 21:u2_printf("Cartoon_Washlet_22");
								break;		
						case 22:u2_printf("Cartoon_Washlet_23");
								break;	
						case 23:u2_printf("Cartoon_Washlet_24");
								break;								
					}
				}
			}							
		}
			Motor_6_STOP();    //6�ŵ��ֹͣ
			TIM10_Stop();      //�رն�ʱ��	
		//�жϸ�λ
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(dir==0))//�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
		{		
			washlet_flag=0;	 
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");		
		}
		 else if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==dir))
		 {
			washlet_flag=1;    
			delay_ms(200);
			u2_printf("Cartoon_Washlet_25");
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
		u2_printf("��������������");
	}
	else
	 {
			LED0=0;          //��������������LED0/LED1��һ��
			LED1=0;
			delay_ms(100);
			LED0=1;
			LED1=1;    	
	 }	
}




void WashLet_V2(u8 dir,u16 arr)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//�������ܣ�ֻ���������ȡ����ҷ���λ�󣬲���ִ������������	
	if((lock_flag==1)&&(0==body_left_flag)&&(0==body_right_flag)) 
	{
		if(0==washlet_auto_flag)
		{			
			u2_printf("Uart_Washlet_Auto_Start");
			delay_ms(200);
		}          
	
		if((dir==1)&&(0==washlet_auto_flag))     //�Զ��������г�
		{
			u2_printf("washlet_auto_flag==1");			 
			washlet_auto_flag=1;
			delay_ms(200);
			if((1==back_flag)&&(0==leg_down_flag))                 //��ʱ�Ѵ���֧��֧��״̬
			{
				Push_Rod_Start(1);                                 //��������	
				leg_down_state_flag=1;                             //��������Ҫ������֧������Ҫ 
				back_state_flag=0;
				u2_printf("Cartoon_Washlet_Leg_Down_1");
				delay_ms(200);
			}
			else if((0==back_flag)&&(1==leg_down_flag))            //��ʱ�Ѵ���������״̬
			{
				Motor_1_START(1);                                   //֧������
				back_dir_flag=1;
				back_state_flag=1;                                  //֧����Ҫ�����������Ȳ���Ҫ 
				leg_down_state_flag=0;
				u2_printf("Cartoon_Washlet_Back_1");
				delay_ms(200);
			}
			else if((0==back_flag)&&(0==leg_down_flag))            //��ʱ����֧���������ȸ�λ״̬
			{
				Motor_1_START(1);                                  //֧������   
				Push_Rod_Start(1);                                 //��������
				back_dir_flag=1;
				leg_down_state_flag=1;                             //֧����Ҫ��������������Ҫ����       
				back_state_flag=1;
				u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				delay_ms(200);
			}
			else if((1==back_flag)&&(1==leg_down_flag))            //��ʱ�Ѵ���֧��֧��������״̬
			{
				leg_down_state_flag=0;            
				back_state_flag=0; 
			}
		}		
		else if((dir==0)&&(1==washlet_auto_flag))         //�Զ����㸴λ-֧���������ȸ�λ
		{
			u2_printf("�ָ��˻ָ���,��ʼ�ָ�\r\n");
			back_dir_flag=0; 
			leg_down_state_flag=1;            
			back_state_flag=1;			
			Motor_1_START(0);                                      //֧������       
			Push_Rod_Start(0);                                     //��������			
			u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
			delay_ms(200);
		}
		
		if((1==leg_down_state_flag)||(1==back_state_flag))
		{	
			if(1==leg_down_state_flag){ leg_down_flag=1; }
			if(1==back_state_flag)    { back_flag=1;     }
			Uart_Back_Leg();
		}
		
		if(dir==1)
		{
			//Uart_Washlet(0);	            //�����		
			u2_printf("�����\r\n");			
			WashLet_V1(1,arr);
			delay_ms(500);
			u2_printf("��ʼ������\r\n");		
			Uart_Washlet_Weight();          //������
			u2_printf("����������\r\n");	
			//delay_ms(1000);

			Uart_Swash_Dry();             //��ϴ���			
			
			
			WashLet_V1(0,arr);	            //����ر�
			u2_printf("����ر�\r\n");	
			
			if(washlet_flag==0)             //�ж������Ƿ��ڸ�λ״̬���ٽ���������ս�
			{									

				washlet_flag=1;
				RELAY6=1;
				u2_printf("�̵����õ�\r\n");
				//�̵����õ�
				Uart_Motor_6_2_START(1,21000);   //�����Ƹ����
				u2_printf("�����Ƹ����\r\n");
				RELAY6=0; 
				u2_printf("�̵���ʧ��\r\n");
				washlet_flag=0;
							
				delay_ms(100);
				//Uart_Washlet_Tig(1);        //������ս�				
				//Uart_Washlet_Auto();        //�ٴε��øú�����ʹ��־λȡ������λ
				washlet_flag=1;
				RELAY6=1;
				Uart_Motor_6_2_START(0,21000);   //�����Ƹ�����
				RELAY6=0;
				washlet_flag=0;
				u2_printf("�����Ƹ�����\r\n");
				
				
				//С����
				u2_printf("\r\n\r\n*****С����******\r\n\r\n");
				DeskRun1(1,100);
				delay_ms(6000);  	delay_ms(4000);  
				DeskRun1(0,100);
				delay_ms(1000);				
							
				WashLet_V2(0,arr);
				
				leg_down_flag=0;            //�����־λ
				back_flag=0;
				washlet_auto_flag=0;
				leg_down_state_flag=0;
				back_state_flag=0;
				back_dir_flag=0;
				delay_ms(200);
				u2_printf("washlet_auto_flag==0");
				delay_ms(200);
				u2_printf("WashletAutoRes");				
			}	
		}				
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Uart_Washlet_Auto_Interfere");
		LED0=1;
		LED1=1;  
	}	
}


