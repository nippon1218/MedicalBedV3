#include "function.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "motor.h"
#include "pump.h"
#include "ds18b20.h"
#include "hx711.h"
#include "timer.h"
#include "pwm.h"
#include "key.h"
#include "modbus_master.h"
#include "bsp_user_lib.h"
#include "nand.h"
#include "pcf8574.h"
#include "common.h"
#include "w25qxx.h"
#include "check.h"
#include "hang.h"
#include "washlet.h"

//����
unsigned long weight1[10]; 
unsigned long weight2[10];
unsigned long weight3[10];
unsigned long u1=0,u2=0,u3=0;

unsigned int i;
unsigned int k,w,num=1;

u8 device_num;                      //���ڼ��㵱ǰWiFi�����豸��
u8 _step=0;

/********************���������������Ƕ�********************************/

u8  back_angle_max=85;               //��λ���趨֧�����нǶ�
u8  leg_up_angle_max=30;             //��λ���趨���������нǶ�
u8  leg_down_angle_max=80;           //��λ���趨���������нǶ�
u8  body_left_angle_max=45;          //��λ���趨�����нǶ�
u8  body_right_angle_max=40;         //��λ���趨�ҷ����нǶ�
u8  desk_distance_max=100;           //��λ���趨����С�����˶�����
u8  swash_dry_time_max=1;            //��λ���趨���������ϴ���ʱ��/����
u16 washlet_arr_lim=32950;           //�����������£���ʱ����arr����װ��ֵ��12000

/********************����������ֵ********************************/

u16 back_runed_arr=0;                //֧������������Զ���װ��ֵ
u16 leg_up_runed_arr=0;              //�����ȣ���������Զ���װ��ֵ
u16 leg_down_runed_arr=0;            //�����ȣ���������Զ���װ��ֵ
u16 body_left_runed_arr=0;           //�󷭣���������Զ���װ��ֵ
u16 body_right_runed_arr=0;          //�ҷ�����������Զ���װ��ֵ
u16 washlet_runed_arr=0;             //����������������Զ���װ��ֵ
u16 desk_runed_arr=0;                //���ӣ���������Զ���װ��ֵ
u16 back_nursing_left_runed_arr=0;   //�󱳻�����������Զ���װ��ֵ
u16 back_nursing_right_runed_arr=0;  //�ұ�������������Զ���װ��ֵ
u16 waist_nursing_left_runed_arr=0;  //����������������Զ���װ��ֵ
u16 waist_nursing_right_runed_arr=0; //����������������Զ���װ��ֵ

u16 add_arr=5000;                    //Ϊ��֤��λ����ʼλ�ã���Ӷ�������ֵ

/********************���ҿ�������������********************************/

//��֫
u32 arm_left_runed=0;                  //�����ʼλ��������������             
u32 arm_left_lim=2000000;              //��������ֵ 
u32 leg_left_runed=0;                  //�����ʼλ��������������             
u32 leg_left_lim=3000000;              //��������ֵ 

//��֫
u32 arm_right_runed=0;                 //�����ʼλ��������������            
u32 arm_right_lim=2000000;             //��������ֵ 
u32 leg_right_runed=0;                 //�����ʼλ��������������            
u32 leg_right_lim=3000000;             //��������ֵ 

//����֫
u32 arm_left_right_runed=0;            //�����ʼλ��������������        
u32 arm_left_right_lim=2000000;        //��������ֵ 
u32 leg_left_right_runed=0;            //�����ʼλ��������������        
u32 leg_left_right_lim=3000000;        //��������ֵ 

//��С��/С��
u32 arm_fore_left_runed=0;             //�����ʼλ��������������        
u32 arm_fore_left_lim=2000000;         //��������ֵ 
u32 leg_fore_left_runed=0;             //�����ʼλ��������������        
u32 leg_fore_left_lim=3000000;         //��������ֵ 

//����/����
u32 arm_post_left_lim=2000000;         //��������ֵ       
u32 arm_post_left_runed=0;             //�����ʼλ��������������
u32 leg_post_left_lim=3000000;         //��������ֵ       
u32 leg_post_left_runed=0;             //�����ʼλ��������������


//��С��/С��
u32 arm_fore_right_lim=2000000;        //��������ֵ      
u32 arm_fore_right_runed=0;            //�����ʼλ��������������
u32 leg_fore_right_lim=3000000;        //��������ֵ      
u32 leg_fore_right_runed=0;            //�����ʼλ��������������

//�Ҵ��/����
u32 arm_post_right_lim=2000000;        //��������ֵ      
u32 arm_post_right_runed=0;            //�����ʼλ��������������
u32 leg_post_right_lim=3000000;        //��������ֵ      
u32 leg_post_right_runed=0;            //�����ʼλ��������������


//����С��/��
u32 arm_fore_left_right_lim=2000000;   //��������ֵ 
u32 arm_fore_left_right_runed=0;       //�����ʼλ��������������
u32 leg_fore_left_right_lim=3000000;   //��������ֵ 
u32 leg_fore_left_right_runed=0;       //�����ʼλ��������������

//���Ҵ��/����
u32 arm_post_left_right_lim=2000000;   //��������ֵ 
u32 arm_post_left_right_runed=0;       //�����ʼλ��������������
u32 leg_post_left_right_lim=3000000;   //��������ֵ 
u32 leg_post_left_right_runed=0;       //�����ʼλ��������������

/********************��ϴ����Ƹ�***************************/

u32 push_rod_runed_pulse=0;                   //�����ʼλ��������������  
u32 push_rod_pulse_lim=30000;                  //��������

u32 swash_dry_runed_pulse=0;                  //�����ʼλ��������������  
u32 swash_dry_pulse_lim=25000;                 //��������

/********************������ս�ǰ�ƶ����Ƹ�***************************/

u32 push_rod_tig_runed_pulse=0;              //�����ʼλ��������������  
u32 push_rod_tig_pulse_now=0;                //��ǰһ������������
u32 push_rod_tig_pulse_lim=5000;             //��������

/*****************��λ״̬��־λ��1��δ��λ �� 0����λ����ʼ״̬***********/

u8 back_flag=0;               //֧��
u8 leg_up_flag=0;             //������
u8 leg_down_flag=0;           //������
u8 body_left_flag=0;          //��
u8 body_right_flag=0;         //�ҷ�
u8 back_nursing_left_flag=0;  //�󱳲�����
u8 back_nursing_right_flag=0; //�ұ�������
u8 waist_nursing_left_flag=0; //����������
u8 waist_nursing_right_flag=0;//����������
u8 washlet_flag=0;            //������
u8 washlet_auto_flag=0;       //�Զ�������
u8 desk_flag=0;               //�Ͳ�����һ����
u8 jram_flag=0;               //���ⰴĦ
u8 swash_dry_flag=0;          //��ϴ���
u8 lock_flag=1;               //һ����������
u8 fault_flag=0;              //������ϱ�־λ

u8 swash_hand_flag=0;         //�ֶ���ϴ��־λ
u8 dry_hand_flag=0;           //�ֶ���ɱ�־λ

//����
u8 armleg_left_flag=0;        //�ֶ���֫
u8 armleg_right_flag=0;       //�ֶ���֫
u8 armleg_left_right_flag=0;  //�ֶ�����֫

u8 arm_fore_left_flag=0;      //��С��
u8 leg_fore_left_flag=0;      //��С��

u8 arm_fore_right_flag=0;     //��С��
u8 leg_fore_right_flag=0;     //��С��

u8 arm_post_left_flag=0;      //����
u8 leg_post_left_flag=0;      //�����

u8 arm_post_right_flag=0;     //�Ҵ��
u8 leg_post_right_flag=0;     //�Ҵ���

u8 arm_fore_post_left_flag=0;       //���С��
u8 leg_fore_post_left_flag=0;       //���С��

u8 arm_fore_post_right_flag=0;      //�Ҵ�С��
u8 leg_fore_post_right_flag=0;      //�Ҵ�С��

u8 arm_fore_left_right_flag=0;      //����С��
u8 leg_fore_left_right_flag=0;      //����С��

u8 arm_post_left_right_flag=0;      //���Ҵ��
u8 leg_post_left_right_flag=0;      //���Ҵ���

u8 arm_fore_post_left_right_flag=0; //���Ҵ�С��
u8 leg_fore_post_left_right_flag=0; //���Ҵ�С��

/****************����ͼƬ******************************/
u8 back_picture_k=0;                //֧��
u8 leg_up_picture_k=0;              //������
u8 leg_down_picture_k=0;            //������
u8 desk_picture_k=0;                //С����
u8 washlet_picture_k=0;             //������
u8 body_left_picture_k=0;           //����
u8 left_motor5_picture_m=0;         //��С�෭
u8 body_right_picture_k=0;          //�ҷ���
u8 right_motor5_picture_m=0;        //��С�෭

u8 back_nursing_left_picture_k=0;   //�󱳲�����
u8 waist_nursing_left_picture_k=0;  //����������
u8 back_nursing_right_picture_k=0;  //�ұ�������
u8 waist_nursing_right_picture_k=0; //����������

/****************����״̬��־λ******************************/

//����������-�������
u8 body_left_overload_3=0;          //����
u8 body_left_overload_4=0;          
u8 body_left_overload_5=0;         
u8 body_right_overload_3=0;         //�ҷ���
u8 body_right_overload_4=0;         
u8 body_right_overload_5=0;        
u8 washlet_auto_overload=0;         //�Զ�����
u8 desk_overload=0;                 //�Ͳ�����һ����
u8 back_nursing_left_overload=0;    //�󱳲�����
u8 back_nursing_right_overload=0;   //�ұ�������
u8 waist_nursing_left_overload=0;   //����������
u8 waist_nursing_right_overload=0;  //����������

//����������-���ʧ��
u8 body_left_losepulse=0;            //����         
u8 body_right_losepulse=0;           //�ҷ���         
u8 washlet_auto_losepulse=0;         //�Զ�����
u8 desk_losepulse=0;                 //�Ͳ�����һ����
u8 back_nursing_left_losepulse=0;    //�󱳲�����
u8 back_nursing_right_losepulse=0;   //�ұ�������
u8 waist_nursing_left_losepulse=0;   //����������
u8 waist_nursing_right_losepulse=0;  //����������

//�������г��ֶ��������־λ
u8 back_interfere=0;                 //֧��
u8 leg_up_interfere=0;               //������
u8 leg_down_interfere=0;             //������    
u8 leg_interfere=0;                  //����
u8 body_left_interfere=0;            //����
u8 body_right_interfere=0;           //�ҷ���
u8 washlet_auto_interfere=0;         //������
u8 desk_interfere=0;                 //С����
u8 back_nursing_left_interfere=0;    //�󱳲�����
u8 back_nursing_right_interfere=0;   //�ұ�������
u8 waist_nursing_left_interfere=0;   //����������
u8 waist_nursing_right_interfere=0;  //����������

/****************�����־λ******************************/

u8 back_dir_flag=0;                    //֧�������־λ
u8 leg_up_dir_flag=0;                  //�����ȷ����־λ
u8 leg_down_dir_flag=0;                //�����ȷ����־λ
u8 body_left_dir_flag=0;               //�󷭷����־λ
u8 body_right_dir_flag=0;              //�ҷ������־λ
u8 back_nursing_left_dir_flag=0;       //�󱳲��������־λ
u8 back_nursing_right_dir_flag=0;      //�ұ����������־λ
u8 waist_nursing_left_dir_flag=0;      //�������������־λ
u8 waist_nursing_right_dir_flag=0;     //�������������־λ
u8 washlet_dir_flag=0;                 //�����������־λ
u8 washlet_auto_dir_flag=0;            //�Զ������������־λ
u8 desk_dir_flag=0;                    //С���ӷ����־
u8 muscle_massager_flag=0;             //���ⰴĦ�����־
u8 lock_dir_flag=0;                    //������־

u8 leg_down_state_flag=0;              //�Զ�����ʱ��¼�����Ƿ��Ѵ��ڶ���״̬
u8 back_state_flag=0;                  //�Զ�����ʱ��¼֧���Ƿ��Ѵ��ڶ���״̬

unsigned int temp_flag_1=1;            //�¶ȱ�־
unsigned int temp_flag_2=1;            //�¶ȱ�־

/*************��������Զ���װ��ֵ�����Ƶ�������ٶ�**********************/

unsigned int motor_back_freq=1000-1;     //֧��1000
unsigned int motor_body_freq=1400-1;     //����1250
unsigned int motor_washlet_freq=1400-1;  //���� 700
unsigned int motor_desk_freq=700-1;      //С����700
unsigned int motor_hang_freq=70-1;      //���� 800

/****************************��ʱ����Ƶֵ********************************/

unsigned int motor_timer_freq=25-1;      //��ʱ����Ƶֵpsc�����Ƶ�������ٶ�
u16 timer10_freq=65000;                  //��ʱ����Ƶֵpsc�����Ƶ�������ٶ�
unsigned int timer10_freq_1=45000-1;     //��ʱ��3��Ƶֵpsc
unsigned int timer10_arr_1s=2000-1;      //��ʱ��3��Ԥװ��ֵarr:psc*arr/90M=1s

u16 bodyleft_compleate=1400;
u16 bodyright_compleate=2200;

/***********************************************************************
 ������      ��Fun_Back(void)   
 ��������    ��ִ��֧������-�綯�Ƹ�
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Back(void)
{
	u8 direct,len;
	u16 arr_now;               //��ǰһ������������
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;           //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	static u8 dir_fore;        //��¼��һ�ε��˶�����
	static u8 dir_change;      //�ж��˶������Ƿ����仯
	static u16 k;              //����k�Ŷ���
	static u8 kj;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;              //��ǰһ����������ֵ
	static u8 back_limit_flag; //֧�����е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧��
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackUpPhone"))
		{			
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=1;
				back_dir_flag=1;      
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(200);
					u2_printf("back_flag==1");
					delay_ms(200);
					u2_printf("BackStart");
					delay_ms(200);
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
				}
				Motor_1_START(1);                                                          //֧������
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq); //�򿪶�ʱ��
			}				
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackDownPhone"))
		{
			if(back_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				direct=0;
				back_dir_flag=0; 
				if(1==back_limit_flag)
				{
					back_limit_flag=0;					
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);                           //֧������
				TIM10_Init(back_runed_arr,timer10_freq);    //�򿪶�ʱ��
			}
		}	
		
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;									
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
		
	 	if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{					
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
			{							
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{			
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"BackUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"BackDownPhone")))    //�����յ�Stop,������ѭ��	
						{					
							break_flag=1;
							memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
							UART4_RX_LEN=0;	
							break;						
						}
						else if(strstr((const char *)UART4_RX_BUF,(const char *)"SEND OK"))
						{	}
						else 
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���							
							memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
							UART4_RX_LEN=0;
						}
					 }
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //��ǰһ������ֵ
				//���䶯��ָ��
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				k=back_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);					
					if(kj<2)
					{
						k=j;   back_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Back_2");									
									break;
							case 2:	u2_printf("Cartoon_Back_3");
									break;					
							case 3:	u2_printf("Cartoon_Back_4");
									break;					
							case 4:	u2_printf("Cartoon_Back_5");
									break;	
							case 5:	u2_printf("Cartoon_Back_6");
									break;	
							case 6:	u2_printf("Cartoon_Back_7");
									break;
							case 7:	u2_printf("Cartoon_Back_8");
									break;
							case 8:	u2_printf("Cartoon_Back_9");
									break;						
							case 9:	u2_printf("Cartoon_Back_10");
									break;												
							case 10:u2_printf("Cartoon_Back_11");
									break;
							case 11:u2_printf("Cartoon_Back_12");									
									break;
							case 12:u2_printf("Cartoon_Back_13");
									break;					
							case 13:u2_printf("Cartoon_Back_14");
									break;					
							case 14:u2_printf("Cartoon_Back_15");
									break;	
							case 15:u2_printf("Cartoon_Back_16");
									break;	
							case 16:u2_printf("Cartoon_Back_17");
									break;
							case 17:u2_printf("Cartoon_Back_18");
									break;
							case 18:u2_printf("Cartoon_Back_19");
									break;		
						}
					}				
				}	
			}		
			Motor_1_STOP();    //���ֹͣ
			TIM10_Stop();      //�رն�ʱ��
			break_flag=0;      //��־λ����
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
			{				
				arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				back_flag=0;
				delay_ms(200);
				u2_printf("back_flag==0");
				delay_ms(200);
			}		
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
				back_flag=1;
			}
		  //ͨ���������ж������ۼ�		
			if(	direct==1)        //�����֧�����У�����+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{  
					back_runed_arr=back_angle_to_arr(back_angle_lim);
					back_limit_flag=1; 
					delay_ms(200);
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("BackLim");
				}
				else
				{  
					back_runed_arr+=arr_now;	
				}				
			}
			else                //�����֧�����У�����-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{	
					back_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("BackRes");
				}
				else
				{
					back_runed_arr-=arr_now;
				}						
			}		
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
		}
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Fun_Leg_Up(void)   
 ��������    ��ִ�������Ȳ���
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Leg_Up(void)
{
	u16 arr_now;              //��ǰһ������������
	u8 len;                   //WiFi���ڽ����ַ�������
	u8 direct;	              //���з����־λ
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�жϳ����Ƿ�ͨ��break���� 
	static u8 dir_fore;       //��¼��һ�ε��˶�����
	static u8 dir_change=0;   //�ж��˶������Ƿ����仯
	static u16 k=0;           //�����k�Ŷ���ָ��
	static u8 kj;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;               //��ǰһ����������ֵ
	static u8 leg_up_limit_flag;//���������е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�������ȸ�λ����ܽ���������
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_down_flag==0)&&(lock_flag==1))
	{		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpUpPhone"))  //����������	
		{				   
			if(leg_angle_to_arr(leg_up_angle_lim)>leg_up_runed_arr)            //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=1; 
				if(leg_up_flag==0)
				{
					leg_up_flag=1;
					delay_ms(200);					
					u2_printf("leg_up_flag==1");					
					delay_ms(200);
					u2_printf("LegUpStart");
					delay_ms(200);
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_1");
					delay_ms(200);
				}											
				Push_Rod_Start(0);
				TIM10_Init(leg_angle_to_arr(leg_up_angle_lim)-leg_up_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz					
			} 
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpDownPhone"))  //����������
		{			
			if(leg_up_runed_arr>0)                          //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				direct=0;
				if(leg_up_limit_flag==1)
				{
					leg_up_limit_flag=0;
					u2_printf("Cartoon_Leg_Up_8");
					delay_ms(200);
				}										
				Push_Rod_Start(1);
				TIM10_Init(leg_up_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}
		}		
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;		
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
	 	if(((leg_up_runed_arr!=leg_angle_to_arr(leg_up_angle_lim))&&(1==direct))||((0!=leg_up_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)	
				{	
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpDownPhone")))    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;	
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
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//�жϴ��䶯��ָ��
				if(direct==1)
				{
					j=(leg_up_runed_arr+arr_send)/(leg_angle_to_arr(leg_up_angle_max)/7);
				}
				else
				{
					j=abs(leg_up_runed_arr,arr_send)/(leg_angle_to_arr(leg_up_angle_max)/7);
				}
				k=leg_up_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);					
					if(kj<2)
					{
						k=j;    leg_up_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Leg_Up_2");									
									break;
							case 2:	u2_printf("Cartoon_Leg_Up_3");
									break;					
							case 3:	u2_printf("Cartoon_Leg_Up_4");
									break;					
							case 4:	u2_printf("Cartoon_Leg_Up_5");
									break;	
							case 5:	u2_printf("Cartoon_Leg_Up_6");
									break;	
							case 6:	u2_printf("Cartoon_Leg_Up_7");
									break;
						}
					}					
				}				
			}				//�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
	
			Push_Rod_Stop();    //�Ƹ�ֹͣ
			TIM10_Stop();       //�رն�ʱ��
			break_flag=0;       //��־λ����
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0)) //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
			{
				arr_now=0;                  //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				leg_up_flag=0;
				delay_ms(200);
				u2_printf("leg_up_flag==0");
				delay_ms(200);
			}
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);       
				leg_up_flag=1;
			}			
			 //ͨ���������ж������ۼ�	
			if(direct==1)    //��������������У�����+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_up_runed_arr=leg_angle_to_arr(leg_up_angle_lim);
					leg_up_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("LegUpLim");
				}
				 else
				{
					leg_up_runed_arr=leg_up_runed_arr+arr_now;
				}	
			}
			else     //��������������У�����-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_up_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_1");
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("LegUpRes");
				}
				 else
				{
					leg_up_runed_arr=leg_up_runed_arr-arr_now;
				}			
			}	
			 __HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 		
		}
	}	
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("LegUpInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 ������      ��Phone_Leg_Down(void)   
 ��������    ��ִ�������Ȳ���
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Leg_Down(void)
{
	u16 arr_now;         //��ǰһ������������   
	u8 len;              //���յ��ַ�������
	u8 direct;           //����ĳ���������еķ����־��1-�������У�0-��������
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�ж��Ƿ�ͨ��break����ѭ�� 
	static u8 dir_fore;       //��¼��һ�ε��˶�����
	static u8 dir_change=0;   //�ж��˶������Ƿ����仯
	static u16 k=0;           //���͵�K�Ŷ���ָ��
	static u16 kj=0;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //��ǰһ������������
	static u8 leg_down_limit_flag;//���������е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�������ȸ�λ����ܽ���������
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(lock_flag==1))
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownDownPhone"))   //����������
		{				
			if(leg_angle_to_arr(leg_down_angle_lim)>leg_down_runed_arr)             //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=1;
				if(leg_down_flag==0)
				{
					leg_down_flag=1;
					delay_ms(200);
					u2_printf("leg_down_flag==1");					
					delay_ms(200);
					u2_printf("LegDownStart");
					delay_ms(200);
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_1");
					delay_ms(200);
				}			
				Push_Rod_Start(1);
				TIM10_Init(leg_angle_to_arr(leg_down_angle_lim)-leg_down_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
				leg_down_flag=1;
			}	
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownUpPhone"))  //����������
		{
			if(leg_down_runed_arr>0)     //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=0;
				if(leg_down_limit_flag==1)
				{
					leg_down_limit_flag=0;
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
				}			
				Push_Rod_Start(0);
				TIM10_Init(leg_down_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //����жϱ�־λ	

	   if(((leg_down_runed_arr!=leg_angle_to_arr(leg_down_angle_lim))&&(1==direct))||((0!=leg_down_runed_arr)&&(0==direct)))
	   {			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{						
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownDownPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownUpPhone")))    //�����յ�Stop,������ѭ��	
						{				
							break_flag=1;
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
				 }
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//���Ͷ���ָ��
				if(direct==1)
				{
					j=(leg_down_runed_arr+arr_send)/(leg_angle_to_arr(leg_down_angle_max)/19);
				}
				else
				{
					j=abs(leg_down_runed_arr,arr_send)/(leg_angle_to_arr(leg_down_angle_max)/19);
				}
				k=leg_down_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);  
					if(kj<2)
					{
						k=j;    leg_down_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Leg_Down_2");									
									break;
							case 2:	u2_printf("Cartoon_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Leg_Down_12");									
									break;
							case 12:u2_printf("Cartoon_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Leg_Down_15");
									break;	
							case 15:u2_printf("Cartoon_Leg_Down_16");
									break;	
							case 16:u2_printf("Cartoon_Leg_Down_17");
									break;
							case 17:u2_printf("Cartoon_Leg_Down_18");
									break;
							case 18:u2_printf("Cartoon_Leg_Down_19");
									break;																		
						}
					}				
				}				 
			}				   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��	      
			Push_Rod_Stop();   //�Ƹ�ֹͣ
			TIM10_Stop();      //�رն�ʱ��
			break_flag=0;	   //���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;                 //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				leg_down_flag=0;
				delay_ms(200);			
				u2_printf("leg_down_flag==0");
				delay_ms(200);			
			}
			else
			{			
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);     
				leg_down_flag=1;
			}			
			//ͨ���������ж������ۼ�
			if(direct==1)    //��������������У�����+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);
					leg_down_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("LegDownLim");
				}
				 else
				{
					leg_down_runed_arr=leg_down_runed_arr+arr_now;
				}	
			}
			else		//��������������У�����-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_1");
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("LegDownRes");
				}
				 else
				{
					leg_down_runed_arr=leg_down_runed_arr-arr_now;
				}			
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 					
		}
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("LegDownInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 ������      ��Fun_Body_Left(void)  
 ��������    ��ִ���������
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;                 //��ǰһ������������,���������ۼ�
	u8 len;                      //���յ��ַ�������
	u16 arr_feed;                //��������е�ǰһ�������������������жϵ��ʧ������
	u16 pulse_num=0;             //������۽��յ�������ֵ
	u16 num1=0,num2=0,num3=0;    //���ʵ�����е�����ֵ	
	static u8 motor5_run_flag;   //�ж�С�෭�Ƿ��Ѿ���������������λ��1 
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //�жϳ����Ƿ��break����
	static u16 k=0,m=0;
	u8 i=0,i1=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;   //��ǰһ������������
	static u8 kj;
	static u8 M345_Start;     //345�����һ������
	static u8 M345_End;       //345������е��ϼ���λ��
	static u8 mn;
		
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ��������
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		//С�෭����
		if(body_left_flag==0)   //�����λ����ʼ״̬����ִ������
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))
			{
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				//5�Ų෭��			
				DIR5=1; 
				body_left_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1); 
				u2_printf("body_left_flag==1");				
				delay_ms(200);
				u2_printf("BodyLeftStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	 //�������	
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);    //�򿪶�ʱ��20000
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);                 //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{					
						//�����λ
//						if((0==GD5_Left_End)&&(1==body_left_flag))        //������翪������ѭ�������ͣת 
//						{						
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;	
//							}
//						}
						  //�ж���û���յ���λ��ָ��		
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
				    arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);						
						if(mn<2)
						{
							m=n;   left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;												
							}
						}
					}
				}
				Motor_5_STOP();            //���5ֹͣ
				TIM10_Stop();              //��ʱ���ر�
				break_flag=0;              //�����־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);				
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//���ò�������
			}
		}	
		//����345�ŵ������	
		if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))||(1==motor5_run_flag))
		{				
			if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
			{
			     motor5_run_flag=0;
				 
				//345��������			
				 DIR3=0;DIR4=0;DIR5=0;direct=1;body_left_dir_flag=1;
				 if(M345_Start==0)
				 {
					 delay_ms(200);
					 M345_Start=1;
					 delay_ms(200);
					 u2_printf("Cartoon_Body_Left_1");
					 delay_ms(200);
				 }				 
				 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);				
				 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);			
			}
		}		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftDownPhone"))
		{					
			if(body_left_runed_arr>0)
			{			   
				//345��������	
			     DIR3=1;DIR4=1;DIR5=1;direct=0;body_left_dir_flag=0;
				 if(M345_End==1)
				 {
					 M345_End=0;
					 delay_ms(200);
					 u2_printf("Cartoon_Body_Left_8");	
					 delay_ms(200);
				 }					 
			     Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	//�������			
			     TIM10_Init(body_left_runed_arr,timer10_freq);			//�رն�ʱ��
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		 if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		 {
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//�����λ
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{					
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");		
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");		
//						}
//						break_flag=1;
//						break;
//					}					
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))    //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//						}
//						break_flag=1;					
//						break;
//					}
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftDownPhone")))//�����յ�Stop,������ѭ��	
						{
							break_flag=1;	
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
					//������ϡ��������
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))       
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_left_overload_3=1;
//							u2_printf("BodyLeftOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_left_overload_4=1;
//							u2_printf("BodyLeftOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_left_overload_5=1;
//							u2_printf("BodyLeftOverload5");
//						}					
//						Breakdown_Treatment();
//						break_flag=1;				
//						break;												
//					}						
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}	
				k=body_left_picture_k;
				if(k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{	
						k=j;  body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;
						}
					}				
				}				
			}							
			Motor_3_4_5_STOP();    //���ֹͣ
			TIM10_Stop();          //�رն�ʱ��
			break_flag=0;		   //�����־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_left_flag=0;
				W25QXX_Write((u8*)&body_left_flag,33,1);				
			}					
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now
				body_left_flag=1;									
			}
			//ͨ���������ж������ۼ�
			if(direct==1)     //���У�����+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Left_End)||(0==GD4_Left_End))
				{					
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
					M345_End=1; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("BodyLeftLim");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else     //���У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))
				{										
					body_left_runed_arr=0;
					M345_Start=0;   
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
		 //����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_left_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //���3û��λ
//				{
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD3Start");
//							break;																		
//						}
//					}			
//					Motor_3_STOP();
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //���4û��λ
//				{
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD4Start");
//							break;																		
//						}
//					}			
//					Motor_4_STOP();       //���ֹͣ
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //���3/4��û��λ
//				{
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							body_left_flag=0;
//							W25QXX_Write((u8*)&body_left_flag,33,1);
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							}
//							break;																		
//						}
//					}			
//					Motor_3_4_5_STOP();   //���ֹͣ
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
			//5�ŵ����λ
			if(body_left_flag==0)     //345������λ����ʼ״̬���Ÿ�λ5�ŵ��
			{			
				//5�Ų෭��λ
				Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;		
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);  //5�ŵ������
				body_left_runed_arr=0;			
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);                     //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//�����λ
//						if(((0==GD5_Start)&&(0==body_left_flag)))   //������翪������ѭ�������ͣת 
//						{												
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;  left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}
						}
					}
				}      
				Motor_5_STOP();       //���ֹͣ
				TIM10_Stop();         //�رն�ʱ��
				break_flag=0;         //�����־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ					
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				u2_printf("body_left_flag==0");				
				delay_ms(200);
				Wifi_Send("RunRes");
				delay_ms(200);
				u2_printf("BodyLeftRes");
				//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
			}				
		}
	}
	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Fun_Body_Right(void)  
 ��������    ��ִ���ҷ������
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;        //��ǰһ�����������������������ۼ�
	u8 len;             //���յ��ַ�������
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num1=0,num2=0,num3=0;
	
	static u8 motor5_run_flag;  //С�෭�����б�־λ
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //�жϳ����Ƿ��break���� 
	static u16 k=0,m=0;
	static u8 M345R_Start=0;  //345����ӳ�ʼλ������
	static u8 M345R_End=0;    //345��������ϼ���λ��
	u8 mn;
	u8 kj;

	u8 i=0,i1=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	  //��ǰһ������������
	
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ����������
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{
		if(body_right_flag==0)   //�����λ����ʼ״̬����ִ���ҷ���
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))
			{
			 //5�Ų෭��
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				DIR5=0; 
				body_right_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);
				delay_ms(200);
				u2_printf("body_right_flag==1");
				delay_ms(200);
				u2_printf("BodyRightStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	//�������	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);   //�򿪶�ʱ��  
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//�����λ
//						if((0==GD5_Right_End)&&(1==body_right_flag))                     //������翪������ѭ�������ͣת 
//						{		
//							delay_us(100);
//							if(0==GD5_Right_End)
//							{
//								break_flag=1;
//								u2_printf("GD5RightEnd"); 
//								break;		
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
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
						//������ϡ��������
//						if(1==Motor5_Alm)        
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;	
//							}								
//						}											
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//����ͼƬָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch(m)
							{								
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;
							}
						}
					}
				}					      
				Motor_5_STOP();       //���ֹͣ
				TIM10_Stop();         //�رն�ʱ��
				break_flag=0;         //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);//���ò�������
			}			
		}	
		//����345�ŵ������	
		if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))||(1==motor5_run_flag))  //�ҷ�����
		{		
			if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
			{
				motor5_run_flag=0;
				//345��������
				DIR3=1;DIR4=1;DIR5=1;direct=1;body_right_dir_flag=1;
				if(M345R_Start==0)
				{
					M345R_Start=1;
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");	
					delay_ms(200);
				}			
				Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //�������	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim)-body_right_runed_arr,timer10_freq);	//��ʱ����			
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightDownPhone"))  //�ҷ�����
		{			
			if(body_right_runed_arr>0)
			{
				//345��������
			   DIR3=0;DIR4=0;DIR5=0;direct=0;body_right_dir_flag=0;
			   if(M345R_End==1)
			   {
					M345R_End=0; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
			   }			
													
			   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //�������		
			   TIM10_Init(body_right_runed_arr,timer10_freq);		  //�򿪶�ʱ��
			}	
		}				
		  memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		  UART4_RX_LEN=0;
			
		  __HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ	 	

		 if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
		 {
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))                    //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;					
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");
//							break_flag=1;					
//							break;
//						}
//					}
//					if(((0==GD3_Right_End)||(0==GD4_Right_End))&&(1==direct))           //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if(0==GD3_Right_End)
//						{
//							u2_printf("GD3RightEnd");
//							break_flag=1;					
//							break;
//						}
//						if(0==GD4_Right_End)
//						{
//							u2_printf("GD4RightEnd");
//							break_flag=1;					
//							break;
//						}
//					}
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))|| 
								(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightDownPhone")))  //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
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
					//������ϡ��������
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{											
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_right_overload_3=1;
//							u2_printf("BodyRightOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_right_overload_4=1;
//							u2_printf("BodyRightOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("BodyRightOverload5");
//						}
//						Breakdown_Treatment();	
//						break_flag=1;				
//						break;
//					}										
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;		
						}
					}				
				}				
			}				
				 
			Motor_3_4_5_STOP();    //���ֹͣ
			TIM10_Stop();          //��ʱ���ر�
			break_flag=0;		   //�����־λ
			//�жϸ�λ	
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_right_flag=0;					
				W25QXX_Write((u8*)&body_right_flag,34,1);				
			}					
			else
			{							
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);					
			}
			//ͨ���������ж������ۼ�
			if(direct==1)    //�������У�����+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End))
				{					
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("BodyRightLim");
				}
				else
				{				
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}
			}
			else		//�������У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))
				{					
					body_right_runed_arr=0;	
					M345R_Start=0; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
					delay_ms(200);
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;
				}			
			}			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ�
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start)&&(0==direct))   //���3û��λ
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{	
//							u2_printf("GD3Start");			
//							break;																		
//						}
//					}			
//					Motor_3_STOP();   //���ֹͣ
//					TIM10_Stop();     //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start)&&(0==direct))         //���4û��λ
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD4Start");
//							break;																		
//						}
//					}			
//					Motor_4_STOP();    //���ֹͣ
//					TIM10_Stop();      //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start)&&(0==direct))   //���3/4��û��λ
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							}
//							break;																		
//						}
//					}			
//					Motor_3_4_5_STOP();   //���ֹͣ
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
			 //�෭��λ
			if(0==body_right_flag)      //ֻ�з���λ����ʼ״̬��С�෭�Ÿ�λ
			{			
				//5�Ų෭��λ
				Motor_4_Compensate(0,bodyright_compleate,motor_body_freq,motor_timer_freq);//���ò�������
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;		
				DIR5=1;
				delay_ms(200);			
				u2_printf("Cartoon_Body_Right_Motor5_10");	
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);    //�������
				body_right_runed_arr=0;
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                     //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//�����λ
//						if(((0==GD5_Start)&&(0==body_right_flag)) )           //������翪������ѭ�������ͣת 
//						{						
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break_flag=1;
//								break;
//							}
//						}
						  //�ж���û���յ���λ��ָ��		
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
						//������ϡ��������
//						if(1==Motor5_Alm)       
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;
//							}
//						}				
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					n=9-n;	
					m=right_motor5_picture_m;
					if(	m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}
						}
					}
				}	     
				Motor_5_STOP();      //���ֹͣ
				TIM10_Stop();        //��ʱ���ر�
				break_flag=0;    
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ		   
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);			
				u2_printf("body_right_flag==0");
				delay_ms(200);
				Wifi_Send("RunRes");
				delay_ms(200);
				u2_printf("BodyRightRes");
				//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_right_flag))
//				{
//					DIR5=1;
//					Motor_5_START(motor_body_freq,motor_timer_freq);    //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();    //���ֹͣ
//					TIM10_Stop();      //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}			
			 }	
		}
	}
	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Fun_Desk(void)  
 ��������    ��С����
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Desk(void)
{
	u8 direct,key;    //��ʾ������з���1��С����ǰ����0��С���Ӻ���
	u16 arr_now;      //������������ֵ
	u8 len;           //��ʾ���յ��ַ����ĳ���
	u16 arr_feed;     //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;  //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�жϳ����Ƿ��break���� 
	static u16 k=0;           //���͵�k��ͼƬ
	static u16 kj=0;
	u8 i=0;
	u16 j=0;	
	u16 arr_send;                 //��ǰһ������������
	static u8 desk_limit_flag;    //�ж�С�����Ƿ����е�����λ�ã����Ƿ��ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���С�����ƶ�
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskUpPhone"))
		{
			DIR7=0;
			direct=1;
			if(desk_flag==0)
			{
				desk_flag=1;
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_1");
				delay_ms(200);
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskDownPhone"))
		{
			DIR7=1;
			direct=0;
			if(1==desk_limit_flag)
			{
				desk_limit_flag=0;
				u2_printf("Cartoon_Desk_20");
				delay_ms(200);
			}
		}		
		if(direct==1)   //�����С������ǰ
		{		
			if(desk_distance_to_arr(desk_distance_lim)>desk_runed_arr)  //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)-desk_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}				
		}
		else       //�����С���Ӻ���
		{
			if(desk_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);//�������
				TIM10_Init(desk_runed_arr,timer10_freq);        //�򿪶�ʱ��			
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
	    if(((desk_runed_arr!=desk_distance_to_arr(desk_distance_lim))&&(1==direct))||((0!=desk_runed_arr)&&(0==direct)))
		 {	 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��  
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ											   						
//					if((0==GD7_End)&&(1==direct))
//					{
//						delay_us(100);
//						if(0==GD7_End)
//						{
//							u2_printf("GD7End");
//							break_flag=1;				
//							break;	
//						}
//					}
//					if((0==GD7_Start)&&(0==direct))
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");
//							break_flag=1;					
//							break;
//						}							
//					}
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"Stop"))||
							(strstr((const char *)UART4_RX_BUF,(const char *)"DeskUpPhone"))||
								(strstr((const char *)UART4_RX_BUF,(const char *)"DeskDownPhone")))    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
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
					//������ϡ��������
//					if(1==Motor7_Alm)        
//					{						
//						delay_us(100);
//						if(1==Motor7_Alm)
//						{
//							desk_overload=1;
//							u2_printf("DeskOverload");
//							Breakdown_Treatment();
//							break_flag=1;					
//							break;
//						}							
//					}								
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(desk_runed_arr+arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				else
				{
					j=abs(desk_runed_arr,arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				k=desk_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;   desk_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Desk_2");									
									break;
							case 2:	u2_printf("Cartoon_Desk_3");
									break;					
							case 3:	u2_printf("Cartoon_Desk_4");
									break;					
							case 4:	u2_printf("Cartoon_Desk_5");
									break;	
							case 5:	u2_printf("Cartoon_Desk_6");
									break;	
							case 6:	u2_printf("Cartoon_Desk_7");
									break;
							case 7:	u2_printf("Cartoon_Desk_8");
									break;
							case 8:	u2_printf("Cartoon_Desk_9");
									break;						
							case 9:	u2_printf("Cartoon_Desk_10");
									break;												
							case 10:u2_printf("Cartoon_Desk_11");
									break;	
							case 11:u2_printf("Cartoon_Desk_12");									
									break;
							case 12:u2_printf("Cartoon_Desk_13");
									break;					
							case 13:u2_printf("Cartoon_Desk_14");
									break;					
							case 14:u2_printf("Cartoon_Desk_15");
									break;	
							case 15:u2_printf("Cartoon_Desk_16");
									break;	
							case 16:u2_printf("Cartoon_Desk_17");
									break;
							case 17:u2_printf("Cartoon_Desk_18");
									break;
							case 18:u2_printf("Cartoon_Desk_19");
									break;						
						}
					}					
				}				
			}				    
			Motor_7_STOP();     //���ֹͣ
			TIM10_Stop();       //��ʱ���ر�
			break_flag=0;		//���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
			{
				arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				desk_flag=0;
				delay_ms(200);
				u2_printf("desk_flag==0");			
			}			
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
				desk_flag=1;
			}	
			//ͨ���������ж������ۼ�
			if(	direct==1)        //�����С����ǰ��������+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_End))
				{ 
					desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
					desk_limit_flag=1;	
					delay_ms(200); 
					u2_printf("Cartoon_Desk_20");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("DeskLim");
				}
				else
				{  
					desk_runed_arr=desk_runed_arr+arr_now;	
				}				
			}
			else                //�����С���Ӻ��ˣ�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_Start))
				{	
					desk_runed_arr=0; 
					delay_ms(200);
					u2_printf("Cartoon_Desk_1");
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("DeskRes");
				}
				else
				{	
					desk_runed_arr=desk_runed_arr-arr_now;
				}						
			}				
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	   			
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶�,��ɾ������ж���䣩
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD7_Start==1)&&(direct==0)&&(desk_flag==1))
//			{   
//				u2_printf("������������");
//				DIR7=1;
//				Motor_7_START(motor_desk_freq,motor_timer_freq);
//			    TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{			
//					if(0==GD7_Start)  //����ʱ������翪�أ�����ѭ�� 
//					{				
//						u2_printf("GD7Start");
//						break;																		
//					}
//				}			
//				Motor_7_STOP();   //���ֹͣ
//				TIM10_Stop();     //�رն�ʱ��
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//			}			
		}
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("DeskInterfere");
		LED0=1;
		LED1=1;	

	}		
}

/***********************************************************************
 ������      ��Fun_Back_Nursing_Left(void)  
 ��������    ���󱳲�����
 ����        ����
 ���        ���� 

************************************************************************/
void Fun_Back_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;    //�жϳ����Ƿ��break����
	static u16 k=0;     //���͵�k�Ŷ���ָ��
	u8 i=0;
	u16 j=0;	
	u16 arr_send;	    //��ǰһ������������
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	
	//�������ܣ�ֻ������ִ��������������������λ�󣬲��ܽ����󱳲�����
	if((lock_flag==1)&&(body_left_flag==1)&&(waist_nursing_left_flag==0))
	{
		back_nursing_left_flag=!back_nursing_left_flag;	
		back_nursing_left_dir_flag=!back_nursing_left_dir_flag;			
		if(back_nursing_left_dir_flag==1)
		{ 
			DIR3=1; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_left_flag==1");			
			delay_ms(200);
			u2_printf("BackNursingLeftStart");
			delay_ms(200);
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=0; direct=0;
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
		}

		Motor_3_START(motor_body_freq,motor_timer_freq);	       //�������
		TIM10_Init(body_left_runed_arr,timer10_freq);              //�򿪶�ʱ��body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//�����λ
//				if((0==GD3_Left_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD3_Left_End)
//					{
//						u2_printf("GD3LeftEnd");
//						break_flag=1;
//						break;	
//					}
//				}
//				if((0==GD3_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("GD3Start");
//						break_flag=1;					
//						break;	
//					}						
//				}
				  //�ж���û���յ���λ��ָ��		
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
				//������ϡ��������
//				if(1==Motor3_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor3_Alm)
//					{
//						back_nursing_left_overload=1;
//						u2_printf("BackNursingLeftOverload");
//						Breakdown_Treatment();
//						break_flag=1;					
//						break;
//					}						
//				}			
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_left_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;  back_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Back_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Left_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Left_19");
								break;						
					}
				}
			}
		}				  
		Motor_3_STOP();     //���ֹͣ
		TIM10_Stop();       //�رն�ʱ��    
		break_flag=0;       //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		if(back_nursing_left_dir_flag==1)
		{
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("BackNursingLeftLim");
		}
		else
		{			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("BackNursingLeftRes");
			delay_ms(200);
			u2_printf("back_nursing_left_flag==0");
		}	
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackNursingLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Fun_Back_Nursing_Right(void)
 ��������    ���ұ�������
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Back_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;     //�жϳ����break����
	static u16 k=0;      //���͵�k�Ŷ���ָ��
	u8 i=0;
	u16 j=0;	
	u16 arr_send;	    //��ǰһ������������
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;		
	
	//�������ܣ�ֻ������ִ���ҷ�����������������λ�󣬲��ܽ����ұ�������
	if((lock_flag==1)&&(1==body_right_flag)&&(waist_nursing_right_flag==0))
	{	
		back_nursing_right_flag=!back_nursing_right_flag;
		back_nursing_right_dir_flag=!back_nursing_right_dir_flag;	
		if(back_nursing_right_dir_flag==1)
		{ 
			DIR3=0; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("BackNursingRightStart");
			delay_ms(200);
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=1; direct=0;
			u2_printf("Cartoon_Back_Nursing_Right_20");
			delay_ms(200);
		}		
		Motor_3_START(motor_body_freq*1.4,motor_timer_freq);              //�������
		TIM10_Init(body_right_runed_arr,timer10_freq);                //�򿪶�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ�� 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//������
//				if((0==GD3_Right_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD3_Right_End)
//					{
//						u2_printf("GD3RightEnd");
//						break_flag=1;
//						break;
//					}						
//				}
//				if((0==GD3_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("GD3Start");
//						break_flag=1;
//						break;
//					}						
//				}
				  //�ж���û���յ���λ��ָ��		
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
				//������ϡ��������
//				if(1==Motor3_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor3_Alm)
//					{
//						back_nursing_right_overload=1;
//						u2_printf("BackNursingRightOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//						break;
//					}						
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_right_picture_k;
			if(	k!=j)
			{	
				kj=abs(k,j);				
				if(kj<2)
				{
					k=j;  back_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Right_7");
								break;						
						case 7:	u2_printf("Cartoon_Back_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Right_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Right_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Right_19");
								break;	
					}
				}
			}			
		}			
		Motor_3_STOP();      //���ֹͣ
		TIM10_Stop();        //�رն�ʱ��
		break_flag=0;        //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		
		if(back_nursing_right_dir_flag==1)
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_20");
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("BackNursingRightLim");
		}
		else
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_1"); 
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("BackNursingRightRes");
			delay_ms(200);
			u2_printf("back_nursing_right_flag==0");
		}				
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackNursingRightInterfere");
		LED0=1;	
		LED1=1;		
	}
}

/***********************************************************************
 ������      ��Fun_Waist_Nursing_Left(void)  
 ��������    ������������
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Waist_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;    //�жϳ����break����
	static u16 k=0;     //���͵�k�Ŷ���ָ��
	u8 i=0;
	u16 j=0;	 
	u16 arr_send;	    //��ǰһ������������
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;		
	
	//�������ܣ�ֻ������ִ�����������󱳲�����λ�󣬲��ܽ�������������
	if((lock_flag==1)&&(1==body_left_flag)&&(back_nursing_left_flag==0))
	{
		waist_nursing_left_flag=!waist_nursing_left_flag;
		waist_nursing_left_dir_flag=!waist_nursing_left_dir_flag;		
		if(waist_nursing_left_dir_flag==1)
		{ 
			DIR4=1; direct=1;
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==1");			
			delay_ms(200);
			u2_printf("WaistNursingLeftStart");
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1");
			delay_ms(200);
		}
		else
		{ 
			DIR4=0; direct=0;			
			u2_printf("Cartoon_Waist_Nursing_Left_20");
			delay_ms(200);
		}
			
		Motor_4_START((u16)(motor_body_freq*1.2),motor_timer_freq);             //�������
		TIM10_Init(body_left_runed_arr,timer10_freq);                //�򿪶�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ�� 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//�����λ
//				if((0==GD4_Left_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD4_Left_End)
//					{
//						u2_printf("GD4LeftEnd");
//						break_flag=1;
//						break;	
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))      //����
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");
//						break_flag=1;
//						break;
//					}						
//				}
				  //�ж���û���յ���λ��ָ��		
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
				//������ϡ��������
//				if(1==Motor4_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor4_Alm)
//					{
//						waist_nursing_left_overload=1;
//						u2_printf("WaistNursingLeftOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//						break;	
//					}						
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
            //���Ͷ���ָ��			
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_left_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);			
				if(kj<2)
				{
					k=j;   waist_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Waist_Nursing_Left_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Left_19");
								break;		
					}
				}
			}			
		}				     
		Motor_4_STOP();     //���ֹͣ
		TIM10_Stop();       //��ʱ���ر�
		break_flag=0;       //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		
		if(waist_nursing_left_dir_flag==1)
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_20");  
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("WaistNursingLeftLim");
		}
		else
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1");
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==0");			
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("WaistNursingLeftRes");
		}				
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("WaistNursingLeftInterfere");
		LED0=1;	
		LED1=1;
	}
}

/***********************************************************************
 ������      ��Fun_Waist_Nursing_Right(void)  
 ��������    ������������
 ����        ����
 ���        ����                           
************************************************************************/
void Fun_Waist_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;      //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;   //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;      //�жϳ����break����
	static u16 k=0;       //���͵�k�Ŷ���ָ��
	u8 i=0;
	u16 j=0;	
	u16 arr_send;	      //��ǰһ������������
	static u8 kj;
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	
	//�������ܣ�ֻ������ִ���ҷ��������ұ�������λ�󣬲��ܽ�������������
	if((lock_flag==1)&&(body_right_flag==1)&&(back_nursing_right_flag==0))
	{
		waist_nursing_right_flag=!waist_nursing_right_flag;	
		waist_nursing_right_dir_flag=!waist_nursing_right_dir_flag;		
		if(waist_nursing_right_dir_flag==1)
		{ 
			DIR4=0; direct=1;
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("WaistNursingRightStart");
			Wifi_Send("RunStart");
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");
			delay_ms(200);
		}
		else
		{ 
			DIR4=1; direct=0;
			u2_printf("Cartoon_Waist_Nursing_Right_20");
			delay_ms(200);
		}
			
		Motor_4_START(motor_body_freq,motor_timer_freq);             //�������
		TIM10_Init(body_right_runed_arr,timer10_freq);               //�򿪶�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//�����λ
//				if((0==GD4_Right_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD4_Right_End)
//					{
//						u2_printf("GD4RightEnd");
//						break_flag=1;
//						break;	
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");
//						break_flag=1;
//						break;	
//					}						
//				}
				  //�ж���û���յ���λ��ָ��		
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
				//������ϡ��������
//				if(1==Motor4_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor4_Alm)
//					{
//						waist_nursing_right_overload=1;
//						u2_printf("WaistNursingRightOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//						break;	
//					}						
//				}			
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}	
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_right_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);							
				if(kj<2)	
				{
					k=j;   waist_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Right_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Right_17");
								break;
						case 17:u2_printf("Cartoon_Waist_Nursing_Right_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Right_19");
								break;						
					}
				}
			}
		}				    
		Motor_4_STOP();    //���ֹͣ
		TIM10_Stop();      //�رն�ʱ�� 
		break_flag=0;      //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		
		if(waist_nursing_right_dir_flag==1)
		{ 			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_20");  
			delay_ms(200);
			Wifi_Send("RunLim");
			delay_ms(200);
			u2_printf("WaistNursingRightLim");
		}
		else
		{			
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");  
			delay_ms(200);
			Wifi_Send("RunRes");
			delay_ms(200);
			u2_printf("WaistNursingRightRes");
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==0");
		}		
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("WaistNursingRightInterfere");		
		LED0=1;	
		LED1=1;
	}
}

/***********************************************************************
 ������      ��Washlet_Auto(void)   
 ��������    ������ִ���Զ�����������
 ����        ����
 ���        ���� 
                          
************************************************************************/
void Washlet_Auto(void)
{
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;	
	
	//�������ܣ�ֻ���������ȡ����ҷ���λ�󣬲���ִ������������	
	if((lock_flag==1)&&(0==body_left_flag)&&(0==body_right_flag)) 
	{
		if(0==washlet_auto_flag)
		{
			//Wifi_Send("RunStart");
			//delay_ms(200);
			u2_printf("WashletAutoStart");
			delay_ms(200);
		}                         
		washlet_auto_dir_flag=!washlet_auto_dir_flag; 
		
		if((washlet_auto_dir_flag==1)&&(0==washlet_auto_flag))     //�Զ��������г�
		{
			u2_printf("washlet_auto_flag==1");
			delay_ms(200);
			back_dir_flag=1; 
			washlet_auto_flag=1;
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
				Motor_1_START(1);                                  //֧������
				back_state_flag=1;                                 //֧����Ҫ�����������Ȳ���Ҫ 
				leg_down_state_flag=0;
				u2_printf("Cartoon_Washlet_Back_1");
				delay_ms(200);
			}
			else if((0==back_flag)&&(0==leg_down_flag))            //��ʱ����֧���������ȸ�λ״̬
			{
				Motor_1_START(1);                                  //֧������  
				Push_Rod_Start(1);                                 //��������
				leg_down_state_flag=1;                             //֧����Ҫ��������������Ҫ����       
				back_state_flag=1;
				u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				delay_ms(200);
			}
			else if((1==back_flag)&&(1==leg_down_flag))                 //��ʱ�Ѵ���֧��֧��������״̬
			{
				leg_down_state_flag=0;            
				back_state_flag=0; 
			}
		}		
		else if((washlet_auto_dir_flag==0)&&(1==washlet_auto_flag))         //�Զ����㸴λ-֧���������ȸ�λ
		{
			back_dir_flag=0; 
			leg_down_state_flag=1;            
			back_state_flag=1;			
			Motor_1_START(0);             //֧������       
			Push_Rod_Start(0);            //��������			
			u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
			delay_ms(200);
		}
		if((1==leg_down_state_flag)||(1==back_state_flag))
		{	
			if(1==leg_down_state_flag){ leg_down_flag=1; }
			if(1==back_state_flag)    { back_flag=1; }
			Back_Leg();
		}
		if(washlet_auto_dir_flag==1)
		{
			Washlet(0);	            //�����			
			delay_ms(1000);
//			Washlet_Weight();       //������
//			delay_ms(1000);
			Swash_Dry();            //��ϴ���			
			Washlet(1);	            //����ر�
			if(washlet_flag==0)     //�ж������Ƿ��ڸ�λ״̬���ٽ���������ս�
			{									
				Washlet_Tig(1);             //������ս�
				delay_ms(100);
				Washlet_Auto();             //�ٴε��øú�����ʹ��־λȡ������λ
				leg_down_flag=0;
				back_flag=0;
				washlet_auto_flag=0;
				leg_down_state_flag=0;
				back_state_flag=0;
				delay_ms(200);
				Wifi_Send("RunRes");
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
		Wifi_Send("WashletAutoInterfere");
		LED0=1;
		LED1=1;  
	}	
}


/***********************************************************************
 ������      ��Washlet(void)  
 ��������    ������ִ������������
 ����        ��dir: 0(������)��1���ر����㣩
 ���        ����
                           
************************************************************************/
void Washlet(u8 dir)
{
	u8 direct;       //����ĳ���������еķ����־��1-�������У�0-��������
	u8 key;          //����ɨ�躯������ֵ,�����жϵ��ʧ������
	u16 num,len;
	u16 arr_feed;    //��������е�ǰһ������������
	u16 pulse_num;   //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�����break������־λ
	static u16 k=0;           //���͵�k��ͼƬָ��
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //��ǰһ������������
	static u8 kj;
	
	washlet_flag=1;
	//�������ܣ�ֻ���������ȡ����ҷ���λ�󣬲���ִ������������
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{
		DIR6=dir;	   
		direct=!dir;

		Motor_6_START(motor_washlet_freq,motor_timer_freq);           //�������
		TIM10_Init(washlet_arr_lim,timer10_freq);                     //�򿪶�ʱ��35000
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ
		if(1==direct)
		{
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");
			delay_ms(200);
		}
		else
		{
			u2_printf("Cartoon_Washlet_25");
			delay_ms(200);
		}
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		{						
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{	
				//�����λ
//				if((0==GD6_End)&&(1==direct))
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						u2_printf("GD6End");
//						break_flag=1;
//						break ; 
//					}
//				}
//				if((0==GD6_Start)&&(0==direct))
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						u2_printf("GD6Start");
//						break_flag=1;
//						break ;
//					}
//				}
				  //�ж���û���յ���λ��ָ��		
				if(USART2_RX_LEN&0x8000)
				{
					len=USART2_RX_LEN&0x3fff;				
					USART2_RX_BUF[len]=0;
					if(strstr((const char *)USART2_RX_BUF,(const char *)"SEND OK"))
					{	}
					else 
					{
						u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
						memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
						UART4_RX_LEN=0;
					}												
				}							
				//������ϡ��������
//				if(1==Motor6_Alm)     
//				{	
//					delay_us(100);
//					if(1==Motor6_Alm) 
//					{
//						washlet_auto_overload=1;
//						Wifi_Send("WashletAutoOverload");
//						Breakdown_Treatment();
//						break_flag=1;
//					}
//					break;		             
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(washlet_arr_lim/24);	
			if(0==direct)
			{
				j=24-j;
			}
			k=washlet_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;  washlet_picture_k=k;
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
		break_flag=0;      //���break��־λ
		//�жϸ�λ
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))//�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
		{		
			washlet_flag=0;	  
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");	
				
		}				
		 else
		{
			washlet_flag=1;   
			delay_ms(200);
			u2_printf("Cartoon_Washlet_25");
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//		//ʹ�����λ����ʼ״̬����簲װ��ֱ�Ӵ򿪴˶Σ�
//		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD6_Start)&&(direct==0)&&(washlet_flag==1))
//		{
//			DIR6=1;
//			Motor_6_START(motor_washlet_freq,motor_timer_freq);           //�������
//			TIM10_Init(add_arr,timer10_freq);                             //�򿪶�ʱ��35000
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ	 	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
//			{
//				if(0==GD6_Start)  //���������ػ�������翪�أ�������ѭ�������ֹͣת�� 
//				{  				
//					u2_printf("GD6Start");
//					break; 											 
//				}				
//			}				                                 			
//			Motor_6_STOP();      //���ֹͣ
//			TIM10_Stop();        //�رն�ʱ��
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
//		}
	}
	else
	 {
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("WashletInterfere");
		LED0=1;
		LED1=1;    	
	 }	
}


/***********************************************************************
 ������      ��Back_Leg()   
 ��������    ��֧����������ͬʱ���к���
 ����        ����
 ���        ���� 
                          
************************************************************************/
void Back_Leg(void)
{
	u8 len;
	u16 arr_now;               //��ǰһ������������
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;           //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	static u8 k;               //����k�Ŷ���
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //��ǰһ����������ֵ
		
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧����������
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
		
		//����֧���������Ȼ�֧����������ͬʱ����
		TIM10_Init(leg_angle_to_arr(leg_down_angle_lim),timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )    //��ʱʱ�䵽
		{
			 //�ж���û���յ����ָ��
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
			 //����֧������ָ��
			if((1==back_state_flag)&&(0==leg_down_state_flag))           //ִֻ��֧��
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
				j=arr_send/(back_angle_to_arr(back_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=back_picture_k;
				if(k!=j)
				{
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;  back_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_2");							
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_3");											
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_4");											
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_5");											
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_6");											
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_7");											
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_8");										
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_9");											
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_10");											
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_11");											
									break;
							case 11:u2_printf("Cartoon_Washlet_Back_12");																		
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_13");											
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_14");											
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_15");										
									break;	
							case 15:u2_printf("Cartoon_Washlet_Back_16");																		
									break;
							case 16:u2_printf("Cartoon_Washlet_Back_17");											
									break;					
							case 17:u2_printf("Cartoon_Washlet_Back_18");											
									break;					
							case 18:u2_printf("Cartoon_Washlet_Back_19");										
									break;							
						}
					}
				}
			}
			//�������ȶ���ָ��
			else if((0==back_state_flag)&&(1==leg_down_state_flag))      //ִֻ��������
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(leg_angle_to_arr(leg_down_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=leg_down_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;  leg_down_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Leg_Down_15");
									break;	
							case 15:u2_printf("Cartoon_Washlet_Leg_Down_16");
									break;	
							case 16:u2_printf("Cartoon_Washlet_Leg_Down_17");	
									break;
							case 17:u2_printf("Cartoon_Washlet_Leg_Down_18");
									break;					
							case 18:u2_printf("Cartoon_Washlet_Leg_Down_19");
									break;					
							case 19:u2_printf("Cartoon_Washlet_Leg_Down_20");
									break;	
						}
					}				
				}			
			}			
			
			//����֧����������ͬʱ���ж���ָ��
			else if((1==back_state_flag)&&(1==leg_down_state_flag))      //֧����������ͬʱ����
			{				
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(back_angle_to_arr(back_angle_max)/14);
				if(0==washlet_auto_dir_flag)
				{
					j=14-j;
				}
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
									break;																											
						}
					}				
				}								
			}										
		}
		Push_Rod_Stop();            //������ֹͣ
		TIM10_Stop();		        //�رն�ʱ��	
		//�ж������ȸ�λ
		if((washlet_auto_dir_flag==0)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=0; 
			leg_down_runed_arr=0; 
			if(0==back_state_flag)                   //ֻ�����������У�֧��������
			{
				k=0;   leg_down_picture_k=0;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_1");
			}
		}
		if((washlet_auto_dir_flag==1)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=1; 
			leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim); 
			if(0==back_state_flag)                   //ֻ�����������У�֧��������
			{
				k=19;   leg_down_picture_k=19;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_20");
			}
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		
		//��������֧����֧����������ͬʱ��������
		if(back_state_flag==1)              //��������֧����֧����������ͬʱ��������
		{				
			TIM10_Init(back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim),timer10_freq);
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) ) //֧����ʱʱ�䵽
			{	
				 //�ж���û���յ����ָ��
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
				 if(break_flag==1)
				 {
					u2_printf("break_flag==1");
					break;
				 }
				 //��������֧�����ж���ָ��
				 if(0==leg_down_state_flag)        //ֻ��֧������
				 {
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/19);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/19);
					 }
					 k=back_picture_k;
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;   back_picture_k=k;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_2");							
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_3");											
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_4");											
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_5");											
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_6");											
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_7");											
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_8");										
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_9");											
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_10");											
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_11");											
										break;
								case 11:u2_printf("Cartoon_Washlet_Back_12");																		
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_13");											
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_14");											
										break;					
								case 14:u2_printf("Cartoon_Washlet_Back_15");										
										break;	
								case 15:u2_printf("Cartoon_Washlet_Back_16");																		
										break;
								case 16:u2_printf("Cartoon_Washlet_Back_17");											
										break;					
								case 17:u2_printf("Cartoon_Washlet_Back_18");											
										break;					
								case 18:u2_printf("Cartoon_Washlet_Back_19");										
										break;																																			
							}
						}				
					 }						
				 }
				 //�������������ȡ�֧��ͬʱ���еĶ���ָ��
				 if(1==leg_down_state_flag)       //֧�������ȶ�����
				 { 
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/14);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/14);
					 }
					 k=leg_down_picture_k;
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;  leg_down_picture_k=k;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
										break;	
								case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
										break;																																									
							}
						}				
					}	
				}						 
			}
			Motor_1_STOP();                                             //֧��ֹͣ
			TIM10_Stop();                                               //�رն�ʱ��
			//�ж�֧����λ
			if((washlet_auto_dir_flag==0)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=0; 
				back_runed_arr=0;
				if(0==leg_down_state_flag)         //ֻ��֧������
				{
					k=0;   back_picture_k=0;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_1");
				}
				if(1==leg_down_state_flag)         //֧��������ͬʱ����
				{
					k=0;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				}
			}
			if((washlet_auto_dir_flag==1)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=1; 
				back_runed_arr=back_angle_to_arr(back_angle_lim);
				if(0==leg_down_state_flag)         //ֻ��֧������
				{
					k=19;   back_picture_k=19;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_20");
				}
				if(1==leg_down_state_flag)         //֧��������ͬʱ����
				{
					k=14;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
				}
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		}				
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

/***********************************************************************
 ������      ��Weight()   
 ��������    �����غ���
 ����        ����
 ���        ��1�������ڱ仯��0������δ�����仯 
                          
************************************************************************/
u8 Weight(void)  
{
    //��ʼ�������				
	if(num==1)
	{				         			
		num=2;
		u1=filter();
		u2_printf("\r\nu1=%d\r\n",u1);
		delay_ms(1000);
		if(num==2)
		{	
			num=3;
			u2=filter();
			u2_printf("\r\nu2=%d\r\n",u2);
			delay_ms(1000);
			if(num==3)
			{	 
				num=1;
				u3=filter();
				u2_printf("\r\nu3=%d\r\n",u3);
				delay_ms(1000);
			}
		}
	}
	u2_printf("\r\nabs(u1,u2)=%d\r\n",abs(u1,u2));
	u2_printf("\r\nabs(u1,u3)=%d\r\n",abs(u1,u3));
	u2_printf("\r\nabs(u2,u3)=%d\r\n",abs(u2,u3));
	//������������Ƿ����仯
	 if((abs(u1,u2)<0x30)&&(abs(u1,u3)<0x30)&&(abs(u2,u3)<0x30))//0x300=768
	 {
		 u2_printf("\r\n����û�з����仯\r\n");
		 u2_printf("\r\nu1=%d\r\n",u1);
		 u2_printf("\r\nu2=%d\r\n",u2);
		 u2_printf("\r\nu3=%d\r\n",u3);
		 return 0;
	 }
	 else
	 {
		 u2_printf("\r\n���������仯\r\n");
		 return 1;
	 }	 	
}	


/***********************************************************************
 ������      ��Washlet_Weight(void)   
 ��������    ������ִ���Զ�����������
 ����        ����
 ���        ���� 
                          
************************************************************************/
u8  Washlet_Weight(void)
{
	u8 m=0,i=0;
	while(1)
	{
		TIM10_Init(60000-1,timer10_freq);      //�򿪶�ʱ��30S
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
		{
			if(Weight())                //�������ڱ仯����ѭ�������¿�ʼ��ʱ
			{  
				m++;
				switch(m)
				{
					case 1:u2_printf("Cartoon_Washlet_Weight_1");									
							break;
					case 2:u2_printf("Cartoon_Washlet_Weight_2");									
							break;
					case 3:u2_printf("Cartoon_Washlet_Weight_3");									
							break;
					case 4:u2_printf("Cartoon_Washlet_Weight_4");									
							break;
					case 5:u2_printf("Cartoon_Washlet_Weight_5");									
							break;
					case 6:u2_printf("Cartoon_Washlet_Weight_6");									
							break;
					case 7:u2_printf("Cartoon_Washlet_Weight_7");									
							break;
					case 8:u2_printf("Cartoon_Washlet_Weight_8");									
							break;
					case 9:u2_printf("Cartoon_Washlet_Weight_9");									
							break;
					case 10:u2_printf("Cartoon_Washlet_Weight_10");									
							break;
					case 11:u2_printf("Cartoon_Washlet_Weight_11");									
							break;
					case 12:u2_printf("Cartoon_Washlet_Weight_12");									
							break;
					case 13:u2_printf("Cartoon_Washlet_Weight_13");									
							break;
					case 14:u2_printf("Cartoon_Washlet_Weight_14");									
							break;
					case 15:u2_printf("Cartoon_Washlet_Weight_15");									
							break;
				}				
			}
		}
		//���30S��ʱ�䵽������û�з����仯
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(0==Weight())) 
		{ 
			i++; 
		}
		else { i=0; }                    //�������¼�ʱ
		if(i==4)                         //����������û�з����仯����ʼ��ϴ���
		{
			Wifi_Send("Washlet_Over");   //�ű����
			u1=0;
			u2=0;
			u3=0;
			break; 
		}   
	}	   	 		
}

/***********************************************************************
 ������      ��Washlet_Tig()   
 ��������    ��������ս�
 ����        ��dir:������з����־��1-��ת��0-��ת
 ���        ����
                          
************************************************************************/	
void Washlet_Tig(u8 dir)
{	
	u2_printf("������ս���ʼ");
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;          //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	u16 k=0;                  //����k�Ŷ���
	u8 kj;
	u16 j=0;	
	u16 arr_send;             //��ǰһ����������ֵ	
	
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
	UART4_RX_LEN=0;
	u8 len;
	RELAY6=1;                  //�̵����õ磬��������պϣ��������������õ�
	delay_ms(1000);
	Motor_6_2_START(0,18000);  //�����Ƹ����
	u2_printf("Cartoon_Washlet_Tig_1");		
	DIR6_1=dir;
	Motor_6_1_START(360-1,250-1);                  //�����������
	TIM2_Init(10500,timer10_freq);                             //�򿪶�ʱ��35000
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//�ж���û���յ���λ��ָ��		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //��ǰһ������ֵ
		//���䶯��ָ��
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
	Motor_6_1_STOP();                                           //���ֹͣ
	TIM2_Stop();                                                //�رն�ʱ��
	delay_ms(100);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
	Motor_6_2_START(1,18000);                                    //�����Ƹ�����	
	delay_ms(1000);
	RELAY6=0;                                                   //�̵�����λ���������������ϵ�
	u2_printf("������ս�����");
}	


/***********************************************************************
 ������      ��Swash_Dry(void)   
 ��������    ������ִ�г�ϴ��ɹ���
 ����        ����
 ���        ����  
                          
************************************************************************/
void Swash_Dry()
{
	u8 num,len;    //�����ַ�������
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	UART4_RX_LEN=0;
	
    //�������ܣ�ֻ���������ʱ���ܽ����������
	washlet_flag=1;
	if((lock_flag==1)&&(1==washlet_flag))
	{
		Push_Rod_Swash_Dry(1,2000);                 //��ϴ����Ƹ����  
		delay_ms(200);
		
/********************��ʼ��ϴ**********************/
        Swash_Auto();                               //�Զ���ϴ
		
		//�Զ���ϴ�������ȴ�30S���ٴΰ��³�ϴ�����������ֶ����ڳ�ϴ
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);  //��ս��ռĴ���
		UART4_RX_LEN=0;
		TIM9_Init(60000,timer10_freq);              //�򿪶�ʱ������ʱ30S
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
		{					
			if(UART4_RX_LEN&0x8000)
			{
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;	
				if(strstr((const char *)UART4_RX_BUF,(const char *)"SwashPhone"))   //�����յ�Stop,������ѭ��	
				{					
					u2_printf("\r\nSwashPhone����\r\n");
					Swash_Hand();
				}				
			}
		}		
		TIM9_Stop();                                         //�رն�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);   //����жϱ�־λ	
		delay_ms(1000);
		
/**********************************��ʼ���***********************************/
		
		Dry_Auto();       //�Զ����2����
		delay_ms(1000);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
		UART4_RX_LEN=0;
		
		//�Զ���ɽ������ȴ�30S���ٴΰ��º�ɰ����������ֶ����ں��		
		TIM9_Init(60000,timer10_freq);                             //�򿪶�ʱ������ʱ30S
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
		{
			if(UART4_RX_LEN&0x8000)
			{
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;
				if(strstr((const char *)UART4_RX_BUF,(const char *)"DryPhone"))    //�����յ�Stop,������ѭ��	
				{
					u2_printf("\r\nDryPhone����\r\n");
					Dry_Hand();
				}				
			}
		}
		TIM9_Stop();                                        //�رն�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);  //����жϱ�־λ		
		delay_ms(1000);
		Push_Rod_Swash_Dry(0,2000+swash_dry_runed_pulse);   //��ϴ����Ƹ�����		
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

/***********************************************************************
 ������      ��Swash_Hand(void)   
 ��������    ������ִ�г�ϴ����
 ����        ����
 ���        ����  
                          
************************************************************************/
void Swash_Hand(void)
{
	u8 len;              //WiFi�����ַ�������
	u8 direct;           //�����־λ
	u8 i;
	Pump_Init();
    //�������ܣ�ֻ���������ʱ���ܽ�����ˮ��ϴ
	if((lock_flag==1)&&(1==washlet_flag))		
	{								
		//��ˮ��ϴ
		swash_hand_flag=1;
		RELAY6=1;             //�̵����õ�
		DIR_SB=1;             //ˮ�ÿ���PB12
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		delay_ms(200);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
		UART4_RX_LEN=0;		
		for(i=0;i<2*swash_dry_time;i++)      //��ϴswash_dry_time����
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //�򿪶�ʱ��,��ʱ������Ϊ30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{ 					
				if(0==Liq_Sensor)	                        //ˮλ�ڵ�ˮλ�£�����һ�γ�ϴ����ֱ������
				{ 
					delay_ms(100);
					Wifi_Send("LiquidLevellow");          //���͸���λ��ָ���źţ���ʾ��ʱˮλƫ��
					break;
				}
				if(UART4_RX_LEN&0x8000)
				{
					len=UART4_RX_LEN&0x3fff;				
					UART4_RX_BUF[len]=0;
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodUpPhone"))    //�Ƹ����	
					{
						direct=1;
						Push_Rod_Swash(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodDown_hone"))  //�Ƹ�����	
					{ 
						direct=0;
						Push_Rod_Swash(0,swash_dry_pulse_lim); 
					}
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
					UART4_RX_LEN=0;	
				}	
			}						 		
			TIM10_Stop();		                                //�رն�ʱ��
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ  
		}
		swash_hand_flag=0;
		DIR_SB=0;             //ˮ�ùر�PB12
		if(swash_dry_runed_pulse>0)
		{
			direct=0;
			Push_Rod_Swash(0,swash_dry_runed_pulse);
		}		
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		RELAY6=0;             //�̵����ϵ�
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("SwashHandInterfere");
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Dry_Hand(void)   
 ��������    ������ִ�к�ɹ���
 ����        ����
 ���        ����  
                          
************************************************************************/
void Dry_Hand(void)
{
	u8 len,i;
	u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������         
	
    //�������ܣ�ֻ���������ʱ���ܽ����������
	if((lock_flag==1)&&(1==washlet_flag))		
	{	
		//�������
		DIR_HG=1;             //����������Ŵ�PB10
		delay_ms(1000);       //�ȴ����Ŵ�1S
		dry_hand_flag=1;
		RELAY6=1;             //�̵����õ�
		DIR_QB=1;             //��������PH2
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		delay_ms(200);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
		UART4_RX_LEN=0;		
		for(i=0;i<2*swash_dry_time;i++)      //���swash_dry_time����
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //�򿪶�ʱ��,��ʱ������Ϊ30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{ 					
				if(UART4_RX_LEN&0x8000)
				{
					len=UART4_RX_LEN&0x3fff;				
					UART4_RX_BUF[len]=0;
					//�ж�������
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodUpPhone"))    //�Ƹ����	
					{
						direct=1;
						Push_Rod_Dry(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)UART4_RX_BUF,(const char *)"PushRodDown_hone"))  //�Ƹ�����	
					{ 
						direct=0;
						Push_Rod_Dry(0,swash_dry_pulse_lim);
					}
					memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
					UART4_RX_LEN=0;
				}				
			}              		
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ   		
			TIM10_Stop();   //�رն�ʱ��		 	
		}
		dry_hand_flag=0;
		if(0==swash_dry_runed_pulse)
		{
			direct=0;
			Push_Rod_Dry(0,swash_dry_runed_pulse);
		}		
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		RELAY6=0;       //�̵����ϵ�
		DIR_QB=0;       //���ùر�PH2
		DIR_HG=0;       //����������ر�PB10		
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("DryHandInterfere");
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Swash_Auto(void)   
 ��������    ������ִ�г�ϴ����
 ����        ����
 ���        ����  
                          
************************************************************************/
void Swash_Auto(void)
{
	u8 direct;   //����ûĳ���������еķ����־��1-�������У�0-��������
	u8 flag=0;   //�����Ƹ˷����л�
	u8 i;
	u8 num,len;
	Pump_Init();
    //�������ܣ�ֻ���������ʱ���ܽ�����ˮ��ϴ
	if((lock_flag==1)&&(1==washlet_flag))		
	{						
		delay_ms(100);
		//Wifi_Send("\r\n��ʼ��ˮ\r\n");
		//��ϴ֮ǰ���ˮλ��������ˮλ���£����������������ȴ�ˮ��עˮ
		if(0==Liq_Sensor)  
		{
			delay_ms(100);
			u2_printf("Liquid_Level_Low");
			//Wifi_Send("Liquid_Level_Low");  //���͸���λ��ָ���źţ���ʾ��ʱˮλƫ��
		}
		delay_ms(200);
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);  //��ս��ռĴ���
		UART4_RX_LEN=0;
		
		while(0==Liq_Sensor)                        //ˮ��עˮ�󣬲��ܼ�������ִ��		
		{
			PCF8574_WriteBit(BEEP_IO,0);	        //���Ʒ���������	
			if(UART4_RX_LEN&0x8000)
			{
				len=UART4_RX_LEN&0x3fff;				
				UART4_RX_BUF[len]=0;				
				if(strstr((const char *)UART4_RX_BUF,(const char *)"BeepOff"))
				{
					PCF8574_WriteBit(BEEP_IO,1);    //������ֹͣ����						
					break;
				}
			}			
		}	
		while(0==Liq_Sensor);                //��ˮ��δע�����ȴ�ע��
		PCF8574_WriteBit(BEEP_IO,1);         //������ֹͣ����
		delay_ms(100);
		//Wifi_Send("ˮ��ע������ʼ����");
		u2_printf("ˮ��ע������ʼ����");
		delay_ms(1000);
		RELAY6=1;                            //�̵����õ�
		//��ˮ��ϴ
		DIR_SB=1;                            //ˮ�ÿ���PB12	
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		for(i=0;i<10*swash_dry_time;i++)     //��ϴ����Ƹ��Զ�ѭ����ϴswash_dry_time����
		{
			flag=!flag;
			Push_Rod_Swash(flag,swash_dry_pulse_lim);      //ÿ���������5S��
			delay_ms(50);			
		}	
		if(0==swash_dry_runed_pulse)
		{                   
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		RELAY6=0;             //�̵����ϵ�
		DIR_SB=0;             //ˮ�ùر�PB12		
		delay_ms(100);
	}
	else
	{
		LED0=0;               //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("SwashAutoInterfere");
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Dry_Auto(void)   
 ��������    ������ִ�к�ɹ���
 ����        ����
 ���        ����  
                          
************************************************************************/
void Dry_Auto(void)
{
	u8 flag=0;        //�����Ƹ˷����л�
	u8 i;
	
    //�������ܣ�ֻ���������ʱ���ܽ����������
	if((lock_flag==1)&&(1==washlet_flag))		
	{
		delay_ms(100);
		Wifi_Send("\r\n��ʼ�������\r\n");		
		//�������
		DIR_HG=1;             //����������Ŵ�PB10
		delay_ms(1000);       //�ȴ����Ŵ�1S
		RELAY6=1;             //�̵����õ�
		DIR_QB=1;             //��������PH2
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}		
		for(i=0;i<10*swash_dry_time;i++)     //��ϴ����Ƹ��Զ�ѭ�����swash_dry_time ����
		{
			flag=!flag;
			Push_Rod_Dry(flag,swash_dry_pulse_lim);        //ÿ���������5S��
			delay_ms(50);
		} 
		if(0==swash_dry_runed_pulse)
		{
			delay_ms(100);
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		RELAY6=0;        //�̵����ϵ�
		DIR_QB=0;        //���ùر�PH2
		DIR_HG=0;        //����������ر�PB10
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("DryAutoInterfere");
		LED0=1;
		LED1=1;
	}	
}
			

/***********************************************************************
 ������      ��IO_TEST(void)   
 ��������    �����ư�IO�ڲ���
 ����        ����
 ���        ����  
                          
************************************************************************/
void IO_TEST(void)
{
	
	GPIO_InitTypeDef GPIO_Initure;          //����ṹ�����GPIO_Initure
    __HAL_RCC_GPIOA_CLK_ENABLE();           //ʹ��GPIOAʱ�� 
	__HAL_RCC_GPIOB_CLK_ENABLE();           //ʹ��GPIOAʱ�� 
    __HAL_RCC_GPIOC_CLK_ENABLE();           //����GPIOCʱ��
	__HAL_RCC_GPIOD_CLK_ENABLE();           //����GPIOCʱ��
    __HAL_RCC_GPIOE_CLK_ENABLE();           //����GPIOEʱ��
    __HAL_RCC_GPIOH_CLK_ENABLE();           //����GPIOHʱ��
    __HAL_RCC_GPIOG_CLK_ENABLE();           //����GPIOGʱ��
    __HAL_RCC_GPIOI_CLK_ENABLE();           //����GPIOIʱ��
		
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;        //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;            //PA12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;       //����
    GPIO_Initure.Pull=GPIO_PULLUP;           //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;      //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);		
	
    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_13; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8; //PI4
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);		
}

/***********************************************************************
 ������      ��Muscle_Massager(void)   
 ��������    ��ִ�м��ⰴĦ����
 ����        ��t:���ⰴĦ����
 ���        ����  
                          
***********************************************************************/
void Muscle_Massager(void)
{
	//�������ܣ�ֻ��������׹ر�ʱ�ſɽ��а�Ħ����
	if((lock_flag==1))
	{
		muscle_massager_flag=!muscle_massager_flag;  //���ⰴĦ��־λȡ��,1-��ʼ��0������
		if(muscle_massager_flag==1)
		{
			//DIR_JRAM=1;          //��Ħ�������Ŵ�
			DIR_XZFPQ=1;					//��ת���������Ŵ�
			delay_ms(500);      //�ȴ����Ŵ�		
			DIR_QB=1;            //��������
			//Wifi_Send("MuscleMassagerStart");
			u2_printf("MuscleMassagerStart");
		}		
		else
		{
			DIR_QB=0;           //����ֹͣ
			DIR_XZFPQ=0;				//��ת������ֹͣ����
			//DIR_JRAM=0;         //��Ħ�������Źر�
			delay_ms(1000);     //�ȴ����Źر�
			//Wifi_Send("MuscleMassagerStop");
			u2_printf("MuscleMassagerStop");
		}		
	}		
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		//Wifi_Send("MuscleMassagerInterfere");
		LED0=1;
		LED1=1;
	} 
}

/***********************************************************************
 ������      ��Heat(void)   
 ��������    ��ִ�м��ȹ���
 ����        ����
 ���        ����  
                          
***********************************************************************/
void Heat()
{
	short temp;
	if(1==washlet_flag)   	//�������ܣ�ֻ��������򿪵�ʱ����ܽ��м���
	{
		LED1=0;
		//����
		DIR_JR=1;           //������������ʼ����PH3
		TIM10_Init(timer10_arr_1s,timer10_freq_1); //�򿪶�ʱ��3,��ʱ������Ϊ(9000*10000)/90MHZ=1S
		for(i=0;i<20;i++)    //��ʱ20S
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)));  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ   
		}
		TIM10_Stop();       //�رն�ʱ��	 
		DS18B20_Start();    //����һ���¶�ת��
		delay_ms(750);
			
		DS18B20_Rst();	 
		DS18B20_Write_Byte(0xCC);
		DS18B20_Write_Byte(0xBE);
			
		while(1)
		{
			if(1==temp_flag_1)   //��ʼ��ʱ�ñ�־λΪ1
			{
				temp=DS18B20_Get_Temp();
				if(temp<=35)
				{
					u2_printf("\r\n%d\r\n",temp);	 
				}
				else
				{
					DIR_JR=0;        //�رռ�����
					u2_printf("\r\n%d\r\n",temp);
					temp_flag_1=0;   //ʹ�������else��������һ��whileѭ��
				}
			}
			else
			{
				temp_flag_1=1;
				break;
			}
		}			
		while(1)
		{
			if(1==temp_flag_2)              //��ʼ��ʱ�ñ�־λΪ1
			{
				temp=DS18B20_Get_Temp();    //��ds18b20�õ��¶�ֵ
				u2_printf("\r\n%d\r\n",temp);
				if(temp<=30)
				{
					DIR_JR=1;               //��������������				
				}
				else if(temp>=37)
				{
					DIR_JR=0;               //�رռ�����ֹͣ����
					temp_flag_2=0;          //ʹ�������else��������һ��whileѭ��
				}
			}
			else
			{
				temp_flag_1=1;
				temp_flag_2=1;
				break;
			}
		}		
		LED1=1;	
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("HeatInterfere");
		LED0=1;
		LED1=1;
	}	
}



/***********************************************************************
 ������      ��WriteInUART4(void)  
 ��������    ������4д�뺯��
 ����        ��Ҫд��UART4_RX_BUF���ַ���""
 ���        ����                           
************************************************************************/
void WriteInUART4(char *p)
{
	u8 len=strlen(p);
	u8 i;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	for(i=0;i<len;i++)
	{
	  UART4_RX_BUF[i]=p[i];
	}	
}

/***********************************************************************
 ������      ��LDUART4(void)  
 ��������    ���������Ժ���
 ����        ����
 ���        ����                           
************************************************************************/
void LDUART4(void)
{
	//��
	WriteInUART4("BodyLeftUpPhone");		
	Fun_Body_Left();           //����	
	delay_ms(1000);
	Fun_Back_Nursing_Left();   //�󱳲�����	
	delay_ms(1000);
	Fun_Back_Nursing_Left();   //�󱳲�����λ	
	delay_ms(1000);
	Fun_Waist_Nursing_Left();  //����������
	delay_ms(1000);
	Fun_Waist_Nursing_Left();  //����������λ	
	delay_ms(1000);
	WriteInUART4("BodyLeftDownPhone");
	Fun_Body_Left();           //����λ
	delay_ms(1000);

	//�ҷ�
	WriteInUART4("BodyRightUpPhone");
	Fun_Body_Right();          //�ҷ���
	delay_ms(1000);
	Fun_Back_Nursing_Right();  //�ұ�������
	delay_ms(1000);
	Fun_Back_Nursing_Right();  //�ұ�������λ
	delay_ms(1000);
	Fun_Waist_Nursing_Right(); //����������
	delay_ms(1000);
	Fun_Waist_Nursing_Right(); //����������λ
	delay_ms(1000);
	WriteInUART4("BodyRightDownPhone");
	Fun_Body_Right();          //�ҷ���λ
	delay_ms(1000);
	
	//������
	WriteInUART4("LegUpUpPhone");
	Fun_Leg_Up();              //������
	delay_ms(1000);
	WriteInUART4("LegUpDownPhone");
	Fun_Leg_Up();              //�����ȸ�λ
	delay_ms(1000);
	
	//�γ�����-֧���������ȡ���������С����
	
	//֧��
//	WriteInUART4("BackUpPhone");
//	Fun_Back();                //֧����
//	delay_ms(1000);
	
	//������
	WriteInUART4("LegDownDownPhone");
	Fun_Leg_Down();           //������	
	delay_ms(1000);
	
	//������
	Washlet(0);               //��������
	delay_ms(1000);
	Washlet(1);               //�������ر�
	delay_ms(1000);
	
	//С����
	WriteInUART4("DeskUpPhone");
	Fun_Desk();               //С���ӿ���
	delay_ms(1000);
	WriteInUART4("DeskDownPhone");
	Fun_Desk();               //С���Ӻ���
	delay_ms(1000);	
	
	WriteInUART4("LegDownUpPhone");
	Fun_Leg_Down();           //�����ȸ�λ
	delay_ms(1000);
	
//	WriteInUART4("BackDownPhone");
//	Fun_Back();               //֧����λ			
}

/***********************************************************************
 ������      ��GB_Back(void)  
 ��������    ������֧����������
 ����        ����
 ���        ����
                           
************************************************************************/
void GB_Back(void)
{
	static u8 back_limit_flag; //�ж�֧���Ƿ����е�����λ�ã����Ƿ��ͼ���λ��ͼƬ
	u8 direct,key;
	u16 arr_now;               //��ǰһ�����������������������ۼ� 
	u8 len;	                   //���ܵ��ַ�������
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�����break������־λ
	static u16 k=0;           //���͵�k��ͼƬָ��
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //��ǰһ������������
	static u8 kj;
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧��
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		back_dir_flag=!back_dir_flag;
		if(back_dir_flag==1)
		{
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=1;
				back_dir_flag=1;      
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(200);
					u2_printf("back_flag==1");
					delay_ms(200);
					u2_printf("BackStart");
					Wifi_Send("RunStart");
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
				}
				Motor_1_START(1);                                                          //֧������
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq); //�򿪶�ʱ��
			}			
		}
		if(back_dir_flag==0)
		{
			if(back_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				direct=0;
				back_dir_flag=0; 
				if(1==back_limit_flag)
				{
					back_limit_flag=0;
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);                        //֧������
				TIM10_Init(back_runed_arr,timer10_freq); //�򿪶�ʱ��
			}
		}				
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);            //����жϱ�־λ	 	
		
		if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))     //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ�� 
			{			
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"BackGB"))||(strstr((const char *)UART4_RX_BUF,(const char *)"Stop")))    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
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
				}	
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//����ͼƬָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}			
				k=back_picture_k;			
				if(	k!=j)
				{
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;	back_picture_k=k;
						switch (k)
						{					
							case 1:	u2_printf("Cartoon_Back_2");									
									break;
							case 2:	u2_printf("Cartoon_Back_3");
									break;					
							case 3:	u2_printf("Cartoon_Back_4");
									break;					
							case 4:	u2_printf("Cartoon_Back_5");
									break;	
							case 5:	u2_printf("Cartoon_Back_6");
									break;	
							case 6:	u2_printf("Cartoon_Back_7");
									break;
							case 7:	u2_printf("Cartoon_Back_8");
									break;
							case 8:	u2_printf("Cartoon_Back_9");
									break;						
							case 9:	u2_printf("Cartoon_Back_10");
									break;												
							case 10:u2_printf("Cartoon_Back_11");
									break;
							case 11:u2_printf("Cartoon_Back_12");									
									break;
							case 12:u2_printf("Cartoon_Back_13");
									break;					
							case 13:u2_printf("Cartoon_Back_14");
									break;					
							case 14:u2_printf("Cartoon_Back_15");
									break;	
							case 15:u2_printf("Cartoon_Back_16");
									break;	
							case 16:u2_printf("Cartoon_Back_17");
									break;
							case 17:u2_printf("Cartoon_Back_18");
									break;
							case 18:u2_printf("Cartoon_Back_19");
									break;						
						}	
					 }				
				}							
			}
			Motor_1_STOP();     //���ֹͣ
			TIM10_Stop();       //�رն�ʱ��
			break_flag=0;	    //���break��־λ	
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
			{				
				arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				back_flag=0;
				delay_ms(200);
				u2_printf("back_flag==0");
			}		
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
				back_flag=1;							 
			}	
			//ͨ���������ж������ۼ�
			if(	direct==1)        //�����֧�����У�����+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{  
					back_runed_arr=back_angle_to_arr(back_angle_lim); 
					back_limit_flag=1; 
					delay_ms(200);
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
					Wifi_Send("RunLim"); 
					delay_ms(200);
					u2_printf("GuardbarBackLim");
				}
				else
				{  back_runed_arr=back_runed_arr+arr_now;	}				
			}
			else                //�����֧�����У�����-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{	
					back_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Back_1");  
					delay_ms(200);
					Wifi_Send("RunRes");
					delay_ms(200);
					u2_printf("GuardbarBackRes");
				}
				else
				{
					back_runed_arr=back_runed_arr-arr_now;
				}						
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 								
		}
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��GB_Body_Left(void)  
 ��������    ����������
 ����        ����
 ���        ����
                           
************************************************************************/
void GB_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;               //��ǰһ�����������������������ۼ�
	u8 len;                    //���ܵ��ַ�������
	u16 arr_feed;              //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;           // �������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ� 
	u16 num1=0,num2=0,num3=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //�����brak������־λ      
	static u8 M345_Start;     //345����ӳ�ʼλ���˶�
	static u8 M345_End;       //345������е��ϼ���λ�ñ�־λ  
	static u16 k=0,m=0;
	u8 i=0;
	u8 mn;
	u8	kj;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	 //��ǰһ������������	
	
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ��������
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		if(body_left_flag==0)   //�����λ����ʼ״̬����ִ��С�෭����
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftGB"))
			{
			  //5�Ų෭��
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;				
				delay_ms(200);
				u2_printf("GuardbarBodyLeftStart");
				delay_ms(200);
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				u2_printf("body_left_flag=1");
				delay_ms(200);				
				DIR5=1; 				
				body_left_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	//�������	
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);   //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//�����λ
//						if((0==GD5_Left_End)&&(1==body_left_flag))  //����ʱ������翪�أ�����ѭ�� 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;					
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
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
						//������ϡ��������
//						if(1==Motor5_Alm)       
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("GuardbarBodyLeftOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;	
//							}								
//						}
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//����ͼƬָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);					
						if(mn<2)
						{
							m=n;	left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}							
						}
					}					
				}					      
				Motor_5_STOP();       //���ֹͣ
				TIM10_Stop();         //��ʱ���ر�
				break_flag=0;  	      //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//���ò�������		
			}	
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;		
		//����345�������	
		if(1==body_left_flag)
		{
			body_left_dir_flag=!body_left_dir_flag;	
			if(body_left_dir_flag==1)    //����
			{				
				if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
				{
				   //345��������				
					 DIR3=0;DIR4=0;DIR5=0;direct=1;
					 if(M345_Start==0)
					 {
						 delay_ms(200);
						 M345_Start=1;
						 u2_printf("Cartoon_Body_Left_1");
						 delay_ms(200);
					 }								 
					 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	  //�������			
					 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);									 
				}
			}
			else          //����
			{				
				if(body_left_runed_arr>0)
				{
				   //345��������
				   DIR3=1;DIR4=1;DIR5=1;direct=0;
					if(M345_End==1)
					 {
						 M345_End=0;
						 delay_ms(200);
						 u2_printf("Cartoon_Body_Left_8");	
						 delay_ms(200);
					 }											
				   Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	 //�������			
				   TIM10_Init(body_left_runed_arr,timer10_freq);	     //�򿪶�ʱ��		
				} 
			}
		}
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ		
		if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//�����λ
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))         //���������翪�أ�����ѭ����ֹͣ����
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");
//							break_flag=1;
//							break;
//						}
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//							break_flag=1;
//							break;
//						}
//					}
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftGB"))||(strstr((const char *)UART4_RX_BUF,(const char *)"Stop")))  //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
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
				
					//������ϡ��������
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_left_overload_3=1;
//							u2_printf("GuardbarBodyLeftOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_left_overload_4=1;
//							u2_printf("GuardbarBodyLeftOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_left_overload_5=1;
//							u2_printf("GuardbarBodyLeftOverload5");
//						}						
//						Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}						
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				k=body_left_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;   body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;		
						}
					}				
				}					
			}							
			Motor_3_4_5_STOP();   //���ֹͣ
			TIM10_Stop();         //��ʱ���ر�
			break_flag=0;         //���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //�ж��Ƿ񵽴︴λ״̬
			{
				arr_now=0;                  
				body_left_flag=0;	
				W25QXX_Write((u8*)&body_left_flag,33,1);			
			}		
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //��ȡ��ǰ����ֵarr_now
				body_left_flag=1;
			}
			//ͨ���������ж������ۼ�
			if(direct==1)       //���У�����+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Left_End)||(0==GD4_Left_End))            //�������е�����λ��
				{
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
					M345_End=1;	 
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("GuardbarBodyLeftLim");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else            //���У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))    //�������е�����λ��
				{					
					body_left_runed_arr=0;
					M345_Start=0;	
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ
//  		//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_left_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //���3û��λ
//				{
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD3Start");			
//							break;																		
//						}
//					}			
//					Motor_3_STOP();       //���ֹͣ
//					TIM10_Stop();         //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //���4û��λ
//				{
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);   //�������
//					TIM10_Init(add_arr,timer10_freq);                  //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD4Start");
//							break;																		
//						}
//					}			
//					Motor_4_STOP();                                     //���ֹͣ
//					TIM10_Stop();                                       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //���3/4��û��λ
//				{
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);//�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							break;
//							}																		
//						}
//					}			
//					Motor_3_4_5_STOP();   //���ֹͣ
//					TIM10_Stop();         //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
			//�෭��λ
			if((body_left_flag==0)&&(0==direct))     //345������λ����ʼ״̬���Ÿ�λ5�ŵ��
			{			
				//5�Ų෭��λ
				Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq); //�������
				body_left_runed_arr=0;
				
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);   //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);                //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{
						//�����λ
//						if(((0==GD5_Start)&&(0==body_left_flag)))            //������翪������ѭ�������ͣת 
//						{								
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
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
						//������ϡ��������
						if(1==Motor5_Alm)         
						{						
							delay_us(100);
							if(1==Motor5_Alm)
							{
								body_left_overload_5=1;						
								u2_printf("GuardbarBodyLeftOverload5");
								Breakdown_Treatment();
								break_flag=1;
								break;	
							}								
						}						
					}	
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;	m=left_motor5_picture_m;
					if(	m!=n)
					{			
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;    left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}	
						}				
					}	
				}      
				Motor_5_STOP();     //���ֹͣ
				TIM10_Stop();       //��ʱ���ر�
				break_flag=0;       //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");				
				delay_ms(200);
				u2_printf("body_left_flag==0");
				delay_ms(200);
				u2_printf("GuardbarBodyLeftRes");
				Wifi_Send("RunRes");
				//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}			
			}				
		}
	}
	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��GB_Body_Right(void)  
 ��������    �������ҷ���
 ����        ����
 ���        ����
                           
************************************************************************/
void GB_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;       //��ǰһ�����������������������ۼ�
	u8 len;            //���յ��ַ�������
	u16 arr_feed;      //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;   //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num1=0,num2=0,num3=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //�����break������־λ 
	static u16 k=0,m=0;
	static u8 M345R_Start;    //345����ӳ�ʼλ������
	static u8 M345R_End;      //345������е��ϼ���λ��
	u8 mn;
	u8 kj;
	u8 i=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	  //��ǰһ������������	
	
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ���ҷ�����
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{			
		if(body_right_flag==0)   //�����λ����ʼ״̬����ִ������
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightGB"))
			{
			  //5�Ų෭��
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				delay_ms(200);
				u2_printf("GuardbarBodyRightStart");
				Wifi_Send("RunStart");
				delay_ms(200);
				u2_printf("body_right_flag==1");				
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				DIR5=0; 
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	    //�������	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);    //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);                  //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{				
						//�����λ
//						if((0==GD5_Right_End)&&(1==body_right_flag))                      //����ʱ������翪�أ�����ѭ�� 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Right_End)
//							{
//								break_flag=1;
//								u2_printf("GD5RightEnd");
//								break;		
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
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
						//������ϡ��������
//						if(1==Motor5_Alm)         
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm) 
//							{
//								body_right_overload_5=1;
//								u2_printf("GuardbarBodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;		
//							}								
//						}					
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					} 
					//����ͼƬָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}	
						}					
					}		
				}						      
				Motor_5_STOP();        //���ֹͣ
				TIM10_Stop();          //��ʱ���ر�
				break_flag=0;          //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);//���ò�������
				}	
			}
			memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
			UART4_RX_LEN=0;			
			//����345�������	
			if(1==body_right_flag)
			{
				body_right_dir_flag=!body_right_dir_flag;
				if(body_right_dir_flag==1)
				{				
					if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
					{
					   //345��������		
					   DIR3=1;DIR4=1;DIR5=1;direct=1;
					   if(M345R_Start==0)
					   {
						   delay_ms(200);
						   M345R_Start=1;	
						   u2_printf("Cartoon_Body_Right_1");
						   delay_ms(200);
					   }									 		
					   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq); //�������				
					   TIM10_Init(body_angle_to_arr(body_right_angle_lim)-body_right_runed_arr,timer10_freq);//�򿪶�ʱ��			
					}
				}
				else
				{				
					if(body_right_runed_arr>0)
					{
					   //345��������
					   DIR3=0;DIR4=0;DIR5=0;direct=0;
					   if(M345R_End==1)
						 {
							 M345R_End=0;
							 delay_ms(200);
							 u2_printf("Cartoon_Body_Right_8");	
							 delay_ms(200);
						 }								 	
					   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //�������			
					   TIM10_Init(body_right_runed_arr,timer10_freq);	      //�򿪶�ʱ��		
					}
				}
			}
			memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
			UART4_RX_LEN=0;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ		
			if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
			{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");	
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");	
//							break_flag=1;
//							break;
//						}
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))    //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//							break_flag=1;
//							break;
//						}
//					}
					//ָֹͣ��
					if(UART4_RX_LEN&0x8000)
					{
						len=UART4_RX_LEN&0x3fff;				
						UART4_RX_BUF[len]=0;
						if((strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightGB"))||(strstr((const char *)UART4_RX_BUF,(const char *)"Stop")) )    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
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
			
					//������ϡ��������
//					if((1==Motor3_Alm) || (1==Motor4_Alm) || (1==Motor5_Alm))       
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_right_overload_3=1;
//							u2_printf("GuardbarBodyRightOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_right_overload_4=1;
//							u2_printf("GuardbarBodyRightOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("GuardbarBodyRightOverload5");
//						}										
//						Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}			
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);							
					if(kj<2)
					{
						k=j;   body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;	
						}
					}				
				}							
			}							
			Motor_3_4_5_STOP();   //���ֹͣ
			TIM10_Stop();         //��ʱ���ر�
			break_flag=0;	      //���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //�ж��Ƿ񵽴︴λ״̬
			{
				arr_now=0;                   //��ʱarr_nowΪ0
				body_right_flag=0;	
				W25QXX_Write((u8*)&body_right_flag,34,1);			
			}		
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);         //��ȡ��ǰ����ֵarr_now
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);	
			}
			//ͨ���������ж������ۼ�
			if(direct==1)      //���У�����+
			{ 
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End)) //�������е�����λ��
				{
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1;   
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					Wifi_Send("RunLim");
					delay_ms(200);
					u2_printf("GuardbarBodyRightLim");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}		
			}	
			else              //���У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))   //�������е�����λ��
				{					
					body_right_runed_arr=0;	
					M345R_Start=0;  
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);       //����жϱ�־λ
//  		//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ�
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //���3û��λ
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD3Start");			
//							break;																		
//						}
//					}			
//					Motor_3_STOP();     //���ֹͣ
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //���4û��λ
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);    //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD4Start");	
//							break;																		
//						}
//					}			
//					Motor_4_STOP();     //���ֹͣ
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //���3/4��û��λ
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);//�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//							}
//							break;																		
//						}
//					}			
//					Motor_3_4_5_STOP(); //���ֹͣ
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
			//�෭��λ
			if((body_right_flag==0)&&(0==direct))     //345������λ����ʼ״̬���Ÿ�λ5�ŵ��
			{			
				//5�Ų෭��λ
				Motor_4_Compensate(0,bodyright_compleate,motor_body_freq,motor_timer_freq);//���ò�������
				memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
				UART4_RX_LEN=0;
				DIR5=1;
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);             //�������
				body_right_runed_arr=0;
			
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                              //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��
				{				
					for(repeat_num=0;repeat_num<600;repeat_num++)
					{			
						//�����λ
//						if((0==GD5_Start)&&(0==body_right_flag))       //����ʱ������翪�أ�����ѭ�� 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break_flag=1;
//								break;	
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
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
						//������ϡ��������
//						if(1==Motor5_Alm)
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("GuardbarBodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;		
//							}								
//						}				
					}				
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
					 arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);			
					 n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					 n=9-n;
					 m=right_motor5_picture_m;
					 if(m!=n)
					 {	
						mn=abs(m,n);			
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}
						}				
					}	
				}      
				Motor_5_STOP();        //���ֹͣ
				TIM10_Stop();		   //��ʱ���ر�
				break_flag=0;          //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");				
			    delay_ms(200);
				u2_printf("body_right_flag==0");
				delay_ms(200);
				u2_printf("GuardbarBodyRightRes");
				Wifi_Send("RunRes");
				//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=1;
//					Motor_5_START(motor_body_freq,motor_timer_freq);   //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							u2_printf("GD5Start");
//							break;																		
//						}
//					}			
//					Motor_5_STOP();        //���ֹͣ
//					TIM10_Stop();		   //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}			
			}			
		}
	}

	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��GB_Back_Nursing(void)  
 ��������    ��������������
 ����        ����
 ���        ����
                           
************************************************************************/
void GB_Back_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //�����ʱ��������������󱳲�����
	{
		Fun_Back_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//�����ʱ�����ҷ���������ұ�������
	{
		Fun_Back_Nursing_Right();
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

/***********************************************************************
 ������      ��GB_Back_Nursing(void)  
 ��������    ��������������
 ����        ����
 ���        ����
                           
************************************************************************/
void GB_Waist_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //�����ʱ�����������������������
	{
		Fun_Waist_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//�����ʱ�����ҷ������������������
	{
		Fun_Waist_Nursing_Right();
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		LED0=1;
		LED1=1;	
		Wifi_Send("Interfere");
	}	
}

/***********************************************************************
 ������      ��GB_Lock(void)  
 ��������    ��һ����������
 ����        ����
 ���        ����
                           
************************************************************************/
u8 GB_Lock(void)
{	
	lock_dir_flag=!lock_dir_flag;
	if(lock_dir_flag==1)
	{
		lock_flag=0;	//��������
		Wifi_Send("Fun_Lock");
		u2_printf("GuardbarLock");
	}
	else
	{
		lock_flag=1;   //��������
		Wifi_Send("Fun_Unlock");
		u2_printf("GuardbarUnLock");
	}
    return 	lock_flag;
}

/***********************************************************************
 ������      ��Exp_Back(void)  
 ��������    ��ר��ϵͳ����-֧��
 ����        ����
 ���        ����
                           
************************************************************************/
void Exp_Back(void) 
{
	//֧����
	WriteInUART4("BackUpPhone");
	Fun_Back();                //֧��
	delay_ms(1000);	
	//֧����λ	
	WriteInUART4("BackDownPhone");
	Fun_Back();                //֧����λ	
}

/***********************************************************************
 ������      ��Exp_Body(void)  
 ��������    ��ר��ϵͳ����-����
 ����        ����
 ���        ����
                           
************************************************************************/
void Exp_Body(void) 
{
	//����
	WriteInUART4("BodyLeftUpPhone");		
	Fun_Body_Left();           //����	
	delay_ms(1000);
	WriteInUART4("BodyLeftDownPhone");		
	Fun_Body_Left();           //����λ
	delay_ms(1000);
			
	//�ҷ���
	WriteInUART4("BodyRightUpPhone");
	Fun_Body_Right();          //�ҷ���
	delay_ms(1000);
	WriteInUART4("BodyRightDownPhone");
	Fun_Body_Right();          //�ҷ���λ

}

/***********************************************************************
 ������      ��Exp_Leg(void)  
 ��������    ��ר��ϵͳ����-����
 ����        ����
 ���        ����
                           
************************************************************************/
void Exp_Leg(void) 
{
	//������
	WriteInUART4("LegUpUpPhone");
	Fun_Leg_Up();              //������
	delay_ms(1000);
	WriteInUART4("LegUpDownPhone");
	Fun_Leg_Up();              //�����ȸ�λ
	delay_ms(1000);	
	
	//������
	WriteInUART4("LegDownDownPhone");
	Fun_Leg_Down();            //������	
	delay_ms(1000);
	WriteInUART4("LegDownUpPhone");
	Fun_Leg_Down();            //�����ȸ�λ

}

/***********************************************************************
 ������      ��Exp_Washlet_Auto(void)  
 ��������    ��ר��ϵͳ����-�Զ�����
 ����        ����
 ���        ����
                           
************************************************************************/
void Exp_Washlet_Auto(void) 
{
	Washlet_Auto();	  //�Զ�����
}


/***********************************************************************
 ������      ��Auto_Arm_Leg_Left(void)  
 ��������    ��ִ����֫��������
 ����        ��t-������������
 ���        ����
                           
************************************************************************/
void Auto_Arm_Leg_Left(int t)
{
   u32 pulse;          //��������������
   int j;	
	//����ֻ��ƽ���ڴ��ϲ��ܽ��е��ҿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   { 		
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftAuto"))    //��첲
		{
			pulse=2000000;
			if(leg_fore_left_flag==0)       //��ֹ�����󴥷������±�־λ��λ
			{	
				arm_fore_left_flag=1;
				Wifi_Send("RunStart");
				u2_printf("ArmLeftAutoStart");
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftAuto"))    //����
		{
			pulse=3000000;
			if(arm_fore_left_flag==0)      //��ֹ�����󴥷������±�־λ��λ
			{	
				leg_fore_left_flag=1;
				Wifi_Send("RunStart");
				u2_printf("LegLeftAutoStart");
			}			
		}	        
		Auto_Hang_1_2(1,pulse);       //����̧֫��
		delay_ms(1000);	     
		   
		for(j=0;j<t;j++)              //����t����֫����ѵ��
		{
			Auto_Hang_1(1,75000);     //�����˶�
			delay_ms(1000);
			Auto_Hang_1(0,75000);     //�����˶�
			delay_ms(1000);	
		}	
		Auto_Hang_1_2(0,pulse);       //����֫��ƽ��ԭ����λ��
		//����־λ��λ
		if((0==arm_left_runed)&&(1==arm_fore_left_flag))         //��첲����������Ϊ�㣬��λ����ʼ״̬
		{
			arm_fore_left_flag=0;
			Wifi_Send("RunRes"); 
			u2_printf("ArmLeftAutoRes");
		}
		if((0==leg_left_runed)&&(1==leg_fore_left_flag))         //��������������Ϊ�㣬��λ����ʼ״̬
		{
			leg_fore_left_flag=0;
			Wifi_Send("RunRes");
			u2_printf("LegLeftAutoRes");
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}		
}

/***********************************************************************
 ������      ��Auto_Arm_Leg_Right(void)   
 ��������    ��ִ����֫���ҿ���ѵ��
 ����        ��t���Ҵ���
 ���        ���� 
                          
************************************************************************/
void Auto_Arm_Leg_Right(int t)
{
	u32 pulse;     //��������������
	//����ֻ��ƽ���ڴ��ϲ��ܽ��е��ҿ���ѵ��
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmRightAuto"))    //�Ҹ첲
		{
			pulse=2000000;
			if(leg_fore_right_flag==0)       //��ֹ�����󴥷������±�־λ��λ
			{	
				arm_fore_right_flag=1;
				Wifi_Send("RunStart");
				u2_printf("ArmRightAutoStart");				
			}			
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegRightAuto"))    //����
		{
			pulse=3000000;
			if(arm_fore_right_flag==0)       //��ֹ�����󴥷������±�־λ��λ
			{	
				leg_fore_right_flag=1;
				Wifi_Send("RunStart");
				u2_printf("LegRightAutoStart");
			}			
		}
	
		int j;
		Auto_Hang_3_4(1,pulse);       //����̧֫��
		delay_ms(1000);
		
		for(j=0;j<t;j++)              //����t����֫����ѵ��
		{		
			Auto_Hang_3(1,75000);     //�����˶�		
			delay_ms(1000);		
			Auto_Hang_3(0,75000);     //�����˶�
			delay_ms(1000);		
		}	
		Auto_Hang_3_4(0,pulse);       //����֫��ƽ��ԭ����λ��
		//����־λ��λ
		if((0==arm_right_runed)&&(1==arm_fore_right_flag))        //�Ҹ첲����������Ϊ�㣬��λ����ʼ״̬
		{
			arm_fore_right_flag=0;
			Wifi_Send("RunRes");
			u2_printf("ArmRightAutoRes");
		}
		if((0==leg_right_runed)&&(1==leg_fore_right_flag))       //��������������Ϊ�㣬��λ����ʼ״̬
		{
			leg_fore_right_flag=0;
			Wifi_Send("RunRes");
			u2_printf("LegRightAutoRes");
		}	
			PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	 {
		LED0=0;                 //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");	
 		 
		LED0=1;
		LED1=1;
	 }	
}

/***********************************************************************
 ������      ��Auto_Arm_Leg_Left_Right(void)   
 ��������    ��ִ������֫���ҿ���ѵ��
 ����        ��t���Ҵ���
 ���        ����  
                          
***********************************************************************/
void Auto_Arm_Leg_Left_Right(int t)
{
	u32 pulse;	         //��������������
	int j;
	//����ֻ��ƽ���ڴ��ϲ��ܽ��е��ҿ���ѵ��
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{		
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftRightAuto"))    //���Ҹ첲
		{
			pulse=2000000;
			if(leg_fore_left_right_flag==0)       //��ֹ�����󴥷������±�־λ��λ
			{	
				arm_fore_left_right_flag=1;	
				Wifi_Send("RunStart");
				u2_printf("ArmLeftRightAutoStart");
			}
			else
			{
				arm_fore_left_right_flag=0;	
			}
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftRightAuto"))    //������
		{
			pulse=3000000;
			if(arm_fore_left_right_flag==0)      //��ֹ�����󴥷������±�־λ��λ
			{	
				leg_fore_left_right_flag=1;
				Wifi_Send("RunStart");
				u2_printf("LegLeftRightAutoStart");
			}
			else
			{
				leg_fore_left_right_flag=0;	
			}
		}	
		Auto_Hang_1_2_3_4(1,pulse);     //������̧֫�ߣ�С��̧�ñȴ�۸�
		delay_ms(1000);	     
			   
		for(j=0;j<t;j++)                //����t�ο���ѵ��
		{
			LED1=0;                     //�ڿ���ѵ��������LED0��˸
			Auto_Hang_1_3(1,75000);     //С������һ���߶�
			LED1=1;
			delay_ms(1000);
			LED1=0;
			Auto_Hang_1_3(0,75000);     //С���½�һ���߶�
			delay_ms(1000);
			LED1=1;	
		}
			Auto_Hang_1_2_3_4(0,pulse); //������֫��ƽ��ԭ����λ��
		//����־λ��λ
		if((0==arm_left_right_runed)&&(1==arm_fore_left_right_flag))  //��첲����������Ϊ�㣬��λ����ʼ״̬
		{
			arm_fore_left_right_flag=0;
			Wifi_Send("RunRes"); 
			u2_printf("ArmLeftRightAutoRes");
		}
		if((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))  //��������������Ϊ�㣬��λ����ʼ״̬
		{
			leg_fore_left_right_flag=0;
			Wifi_Send("RunRes"); 
			u2_printf("LegLeftRightAutoRes");
		}	
			PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;               //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Fore_Left(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Fore_Left(void)
{
	 u8 direct;   //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag=0;
	//��������ֻ���ڼ����򿪡�֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{		
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		if(0==arm_fore_left_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("ArmForeLeftHandStart");
				arm_fore_left_flag=1;
				Hand_Hang_1_2(1,arm_left_lim);       //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}				
		//��С����������
		if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(arm_fore_left_lim>arm_fore_left_runed)
			{
				Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					arm_fore_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmForeLeftHandLim");
				}
				else
				{
					arm_fore_left_flag=1;		
				}								
			}	
		}
		//��С����������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftDownHand"))
		{	
			if(arm_fore_left_runed>0)
			{
				Hand_Hang_1(0,arm_fore_left_runed);					
			}
			if(0==arm_fore_left_runed)               //��С�۸�λ���򽫸첲��λ
			{
				Hand_Hang_1_2(0,arm_left_runed);     //����֫��λ��ˮƽ״̬ 
			}
			if((0==arm_fore_left_runed)&&(0==arm_left_runed))
			{
				arm_fore_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForeLeftHandRes");
			}
		}	
			PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Leg_Fore_Left(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Fore_Left(void)
{
	 u8 direct;   //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag=0;
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
	   PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 if(0==leg_fore_left_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("LegForeLeftHandStart");
				leg_fore_left_flag=1;
				Hand_Hang_1_2(1,leg_left_lim);      //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}
		if((strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(leg_fore_left_lim>leg_fore_left_runed)
			{
				Hand_Hang_1(1,leg_fore_left_lim-leg_fore_left_runed);	
				if(leg_fore_left_runed==leg_fore_left_lim)
				{
					leg_fore_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegForeLeftHandLim");
				}
				else
				{
					leg_fore_left_flag=1;		
				}								
			}	
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftDownHand"))
		{	
			if(leg_fore_left_runed>0)
			{
				Hand_Hang_1(0,leg_fore_left_runed);					
			}
			if(0==leg_fore_left_runed)             //��С������������Ϊ0���򽫸첲��λ
			{
				Hand_Hang_1_2(0,leg_left_runed);   //����֫��λ��ˮƽ״̬ 
			}
			if((0==leg_fore_left_runed)&&(0==leg_left_runed))
			{
				leg_fore_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForeLeftHandRes");
			}			
		}	
			PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Fore_Right(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Fore_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag=0;
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{		
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		if(0==arm_fore_right_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("ArmForeRightHandStart");
				arm_fore_right_flag=1;
				Hand_Hang_3_4(1,arm_right_lim);       //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}
		//��С����������
		if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))||(1==runed_flag))		
		{
			runed_flag=0;
			if(arm_fore_right_lim>arm_fore_right_runed)
			{
				Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);	
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					arm_fore_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmForeRightHandLim");
				}
				else
				{
					arm_fore_right_flag=1;						
				}								
			}	
		}			
		//��С����������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightDownHand"))
		{
			if(arm_fore_right_runed>0)    //��С�������˶�
			{
				Hand_Hang_3(0,arm_fore_right_runed);					
			}
			if(arm_fore_right_runed==0)    //����С�۸�λ�����Ҹ첲��λ��ˮƽλ��
			{
				Hand_Hang_3_4(0,arm_right_runed); 
			}
			if((arm_fore_right_runed==0)&&(0==arm_right_runed))   //��־λ��λ
			{
				arm_fore_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForeRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Leg_Fore_Right(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Fore_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag;
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
	   PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 if(0==leg_fore_right_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("LegForeRightHandStart");
				leg_fore_right_flag=1;
				Hand_Hang_3_4(1,leg_right_lim);       //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}
		//��С������
	   if((strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))||(runed_flag==1))
		{
			runed_flag=0;
			if(leg_fore_right_lim>leg_fore_right_runed)
			{
				Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);	
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					leg_fore_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegForeRightHandLim");
				}
				else
				{
					leg_fore_right_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightDownHand"))
		{
			if(leg_fore_right_runed>0)     //��С����������
			{
				Hand_Hang_3(0,leg_fore_right_runed);					
			}
			if(0==leg_fore_right_runed)    //����С�ȸ�λ������֫��ƽ��ˮƽλ��
			{
				Hand_Hang_3_4(0,leg_right_runed);
			}
			if((0==leg_fore_right_runed)&&(0==leg_right_runed))  //��־λ��λ
			{
				leg_fore_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForeRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Post_Left(void)  
 ��������    ���ֶ�ִ������
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Post_Left(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
	   PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //��������		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftUpHand"))
		{
			if(arm_post_left_lim>arm_post_left_runed)
			{
			    if(0==arm_post_left_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmPostLeftHandStart");
				}
				arm_post_left_flag=1;
				Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_post_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmPostLeftHandLim");
				}
				else
				{
					arm_post_left_flag=1;						
				}								
			}	
		}
         //��������		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftDownHand"))
		{
			if(arm_post_left_runed>0)
			{
				Hand_Hang_1_2(0,arm_post_left_runed);
				if(0==arm_post_left_runed)
				{					
					arm_post_left_flag=0;
					Wifi_Send("RunRes");
					u2_printf("ArmPostLeftHandRes");
				}
				else
				{
					arm_post_left_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_ArmLeg_Post_Left(void)  
 ��������    ���ֶ�ִ�������
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Post_Left(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
	   PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //��������		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftUpHand"))
		{
			if(leg_post_left_lim>leg_post_left_runed)
			{
			    if(0==leg_post_left_flag)
				{	
					Wifi_Send("RunStart");
					u2_printf("LegPostLeftHandStart");
				}
				leg_post_left_flag=1;
				Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_post_left_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegPostLeftHandLim");
				}
				else
				{
					leg_post_left_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftDownHand"))
		{
			if(leg_post_left_runed>0)
			{
				Hand_Hang_1_2(0,leg_post_left_runed);
				if(0==leg_post_left_runed)
				{					
					leg_post_left_flag=0;
					Wifi_Send("RunRes");
					u2_printf("LegPostLeftHandRes");
				}
				else
				{
					leg_post_left_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Post_Right(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightUpHand"))
		{
			if(arm_post_right_lim>arm_post_right_runed)
			{
			    if(0==arm_post_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmPostRightHandStart");
				}
				arm_post_right_flag=1;
				Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_post_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmPostRightHandLim");
				}
				else
				{
					arm_post_right_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightDownHand"))
		{
			if(arm_post_right_runed>0)
			{
				Hand_Hang_3_4(0,arm_post_right_runed);
				if(0==arm_post_right_runed)
				{
					arm_post_right_flag=0;
					Wifi_Send("RunRes");
					u2_printf("ArmPostRightHandRes");
				}
				else
				{
					arm_post_right_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Leg_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ���
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Post_Right(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightUpHand"))
		{
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_post_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegPostRightHandStart");
				}
				leg_post_right_flag=1;
				Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_post_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegPostRightHandLim");
				}
				else
				{
					leg_post_right_flag=1;					
				}								
			}	
		}
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightDownHand"))
		{
			if(leg_post_right_runed>0)
			{
				Hand_Hang_3_4(0,leg_post_right_runed);
				if(0==leg_post_right_runed)
				{
					leg_post_right_flag=0;
					Wifi_Send("RunRes");
					u2_printf("LegPostRightHandRes");
				}
				else
				{
					leg_post_right_flag=1;
				}					
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Fore_Post_Left(void)  
 ��������    ���ֶ�ִ�����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Fore_Post_Left(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
       PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //��������		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))
		{
			 //����������
			if(arm_post_left_lim>arm_post_left_runed)
			{
				if(0==arm_fore_post_left_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmForePostLeftHandStart");
				}
				arm_fore_post_left_flag=1;
				Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_fore_post_left_flag=1;
				}
				else
				{
					arm_fore_post_left_flag=1;						
				}								
			}
			//��������С��
			if((arm_fore_left_lim>arm_fore_left_runed)&&(arm_post_left_runed==arm_post_left_lim))  
			{
				Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("ArmForePostLeftHandLim");
				}
			}			
		}	
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))
		{
			//��������С��
			if(arm_fore_left_runed>0)   
			{
				Hand_Hang_1(0,arm_fore_left_runed);					
			}
			//����������
			if((arm_post_left_runed>0)&&(0==arm_fore_left_runed))
			{
				Hand_Hang_1_2(0,arm_post_left_runed);	
			}
			if((0==arm_post_left_runed)&&(0==arm_fore_left_runed))
			{
				arm_fore_post_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForePostLeftHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_ArmLeg_Fore_Post_Left(void)  
 ��������    ���ֶ�ִ�����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Fore_Post_Left(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
       PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //��������		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))
		{
			 //�����������
			if(leg_post_left_lim>leg_post_left_runed)
			{
				if(0==leg_fore_post_left_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegForePostLeftHandStart");
				}
				leg_fore_post_left_flag=1;
				Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_fore_post_left_flag=1;
				}
				else
				{
					leg_fore_post_left_flag=1;						
				}								
			}
			//��������С��
			if((leg_fore_left_lim>leg_fore_left_runed)&&(leg_post_left_runed==leg_post_left_lim))  
			{
				Hand_Hang_1(1,leg_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==leg_fore_left_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("LegForePostLeftHandLim");
				}
			}							
		}
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand"))
		{
			//��������С��
			if(leg_fore_left_runed>0)   
			{
				Hand_Hang_1(0,leg_fore_left_runed);					
			}
			//�����������
			if((leg_post_left_runed>0)&&(0==leg_fore_left_runed))
			{
				Hand_Hang_1_2(0,leg_post_left_runed);	
			}			
			if((0==leg_post_left_runed)&&(0==leg_fore_left_runed))
			{
				leg_fore_post_left_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForePostLeftHandRes");
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Fore_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Fore_Post_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))
		{
			 //�������Ҵ��
			if(arm_post_right_lim>arm_post_right_runed)
			{
				if(0==arm_fore_post_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmForePostRightHandStart");
				}
				arm_fore_post_right_flag=1;
				Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_fore_post_right_flag=1;
				}
				else
				{
					arm_fore_post_right_flag=1;						
				}								
			}
			//��������С��
			if((arm_fore_right_lim>arm_fore_right_runed)&&(arm_post_right_runed==arm_post_right_lim))  
			{
				Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);	
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("ArmForePostRightHandLim");
				}
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))
		{
			//��������С��
			if(arm_fore_right_runed>0)   
			{
				Hand_Hang_3(0,arm_fore_right_runed);					
			}
			//�������Ҵ��
			if((arm_post_right_runed>0)&&(0==arm_fore_right_runed))
			{
				Hand_Hang_3_4(0,arm_post_right_runed);	
			}
			if((0==arm_post_right_runed)&&(0==arm_fore_right_runed))
			{
				arm_fore_post_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForePostRightHandRes");
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Leg_Fore_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Fore_Post_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))
		{
			 //�������Ҵ���
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_fore_post_right_flag)
				{	
					Wifi_Send("RunStart");
					u2_printf("LegForePostRightHandStart");
				}
				leg_fore_post_right_flag=1;
				Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_fore_post_right_flag=1;
				}
				else
				{
					leg_fore_post_right_flag=1;						
				}								
			}
			//��������С��
			if((leg_fore_right_lim>leg_fore_right_runed)&&(leg_post_right_runed==leg_post_right_lim))  
			{
				Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);	
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("LegForePostRightHandLim");
				}
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand"))
		{
			//��������С��
			if(leg_fore_right_runed>0)   
			{
				Hand_Hang_3(0,leg_fore_right_runed);					
			}
			//�������Ҵ���
			if((leg_post_right_runed>0)&&(0==leg_fore_right_runed))
			{
				Hand_Hang_3_4(0,leg_post_right_runed);	
			}
			if((0==leg_post_right_runed)&&(0==leg_fore_right_runed))
			{
				leg_fore_post_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForePostRightHandRes");
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Fore_Left_Right(void)  
 ��������    ���ֶ�ִ������С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Fore_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag;
	//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ�������С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 if(0==arm_fore_left_right_flag)    //�Ƚ�����̧֫��һ���߶�
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("ArmForeLeftRightHandStart");
				arm_fore_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,arm_left_right_lim);
				runed_flag=1;
			}
		}
		//��������
		if((strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||(1==runed_flag))
		{	
			if(arm_fore_left_right_lim>arm_fore_left_right_runed)
			{
				runed_flag=0;
				Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);	
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					arm_fore_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmForeLeftRightHandLim");
				}
				else
				{
					arm_fore_left_right_flag=1;						
				}								
			}	
		}	
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))
		{
			if(arm_fore_left_right_runed>0)      //����С����������
			{
				Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			if(arm_fore_left_right_runed==0)     //������С�۸�λ��������֫��λ��ˮƽλ��
			{
				Hand_Hang_1_2_3_4(0,arm_left_right_runed);
			}
			if((arm_fore_left_right_runed==0)&&(0==arm_left_right_runed))  //��־λ��λ
			{
				arm_fore_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForeLeftRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Hand_Leg_Fore_Left_Right(void)  
 ��������    ���ֶ�ִ������С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Fore_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag;
	//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ�������С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {	    
	   PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 if(0==leg_fore_left_right_flag)
		{
			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))
			{
				Wifi_Send("RunStart");
				u2_printf("LegForeLeftRightHandStart");
				leg_fore_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,leg_left_right_lim);
				runed_flag=1;
			}				
		}
		//ͬ���������ж������ۼ�
		if((strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||(1==runed_flag))
		{
			if(leg_fore_left_right_lim>leg_fore_left_right_runed)
			{
				runed_flag=0;
				Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);	
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					leg_fore_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegForeLeftRightHandLim");
				}
				else
				{
					leg_fore_left_right_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightDownHand"))
		{
			if(leg_fore_left_right_runed>0)    //����С����������
			{
				Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			if(leg_fore_left_right_runed==0)    //������С�ȸ�λ���������ȸ�λ��ˮƽλ��
			{
				Hand_Hang_1_2_3_4(0,leg_left_right_runed);
			}
			if((leg_fore_left_right_runed==0)&&(0==leg_left_right_runed))   //��־λ��λ
			{
				leg_fore_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForeLeftRightHandRes");
			}
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Post_Left_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ������Ҵ��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //ͬ���������ж������ۼ�		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))
		{
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{
				if(0==arm_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmPostLeftRightHandStart");
				}
				arm_post_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_post_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("ArmPostLeftRightHandLim");
				}
				else
				{
					arm_post_left_right_flag=1;						
				}								
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))
		{
			if(arm_post_left_right_runed>0)
			{
				Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);
				if(0==arm_post_left_right_runed)
				{
					arm_post_left_right_flag=0;
					Wifi_Send("RunRes");
					u2_printf("ArmPostLeftRightHandRes");
				}
				else
				{
					arm_post_left_right_flag=1;			
				}					
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Hand_Leg_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ���
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Post_Left_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ������Ҵ��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {		
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //ͬ���������ж������ۼ�		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightUpHand"))
		{
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegPostLeftRightHandStart");
				}
				leg_post_left_right_flag=1;	
				Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_post_left_right_flag=1;
					Wifi_Send("RunLim");
					u2_printf("LegPostLeftRightHandLim");
				}
				else
				{
					leg_post_left_right_flag=1;							
				}								
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightDownHand"))
		{
			if(leg_post_left_right_runed>0)
			{
				Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);
				if(0==leg_post_left_right_runed)
				{
					leg_post_left_right_flag=0;	
					Wifi_Send("RunRes");
					u2_printf("LegPostLeftRightHandRes");
				}
				else
				{
					leg_post_left_right_flag=1;			
				}					
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Hand_Arm_Fore_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Arm_Fore_Post_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
     if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
     {
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
			 //�����˶�
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))
		{
			 //���������Ҵ��
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{				
				if(0==arm_fore_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("ArmForePostLeftRightHandStart");
				}
				arm_fore_post_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_fore_post_left_right_flag=1;
				}
				else
				{
					arm_fore_post_left_right_flag=1;						
				}								
			}
			//����������С��
			if((arm_fore_left_right_lim>arm_fore_left_right_runed)&&(arm_post_left_right_runed==arm_post_left_right_lim))  
			{
				Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("ArmForePostLeftRightHandLim");
				}
			}	
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))
		{
			//����������С��
			if(arm_fore_left_right_runed>0)   
			{
				Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			//���������Ҵ��
			if((arm_post_left_right_runed>0)&&(0==arm_fore_left_right_runed))
			{
				Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);	
			}
			if((0==arm_post_left_right_runed)&&(0==arm_fore_left_right_runed))
			{
				arm_fore_post_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("ArmForePostLeftRightHandRes");
			}				
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;	
	}	
}

/***********************************************************************
 ������      ��Hand_Leg_Fore_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Hand_Leg_Fore_Post_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		PCF8574_WriteBit(EXIO1,0);	       //�̵���
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))
		{
			 //���������Ҵ���
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_fore_post_left_right_flag)
				{
					Wifi_Send("RunStart");
					u2_printf("LegForePostLeftRightHandStart");
				}				
				leg_fore_post_left_right_flag=1;
				Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_fore_post_left_right_flag=1;
				}
				else
				{
					leg_fore_post_left_right_flag=1;						
				}								
			}
			//����������С��
			if((leg_fore_left_right_lim>leg_fore_left_right_runed)&&(leg_post_left_right_runed==leg_post_left_right_lim))  
			{
				Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);	
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					Wifi_Send("RunLim");
					u2_printf("LegForePostLeftRightHandLim");
				}
			}
		}			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand"))
		{
			//����������С��
			if(leg_fore_left_right_runed>0)   
			{
				Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			//���������Ҵ���
			if((leg_post_left_right_runed>0)&&(0==leg_fore_left_right_runed))
			{
				Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);	
			}
			if((0==leg_post_left_right_runed)&&(0==leg_fore_left_right_runed))
			{
				leg_fore_post_left_right_flag=0;
				Wifi_Send("RunRes");
				u2_printf("LegForePostLeftRightHandRes");
			}			
		}
		PCF8574_WriteBit(EXIO1,1);	       //�̵���
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		Wifi_Send("Interfere");
		LED0=1;
		LED1=1;	
	}	
}




/***********************************************************************
 ������      ��Res_Power_Down(void)  
 ��������    �����縴λ,ÿ�����ܺ���ֹͣ��������翪��
 ����        ����
 ���        ����
                           
************************************************************************/
void Res_Power_Down(void)
{
	if(lock_flag==1)
	{
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		UART4_RX_LEN=0;
//		//��ȡ����λ״̬��־λ
//		u8 	body_left_flag_buf[1];
//		u8 	body_right_flag_buf[1];
//		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //�ӵ�33��ַ��ʼ����ȡ1���ֽ�
//		W25QXX_Read((u8*)body_right_flag_buf,34,1);       //�ӵ�34��ַ��ʼ����ȡ1���ֽ�
//		body_left_flag=body_left_flag_buf[0];
//		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //�󷭸�λ
		{			
			Res_Body_Left();
		}		
		if(1==body_right_flag)    //�ҷ���λ
		{
			Res_Body_Right();
		}
		delay_ms(1000);
		
		washlet_picture_k=24;
		Washlet(1);              //��������λ       
		delay_ms(1000);
	
		Res_Back();              //֧����λ     
		delay_ms(1000);

		Res_Leg();               //���ȸ�λ
		delay_ms(1000);	
	
		Res_Desk();              //�칫����һ������λ   
	}
}

/***********************************************************************
 ������      ��Res_Back(void)  
 ��������    ��֧����λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Res_Back(void)
{
	back_runed_arr=back_angle_to_arr(back_angle_lim);
	back_picture_k=19;
	WriteInUART4("BackDownPhone");
	Fun_Back();                //֧����λ		
}

/***********************************************************************
 ������      ��Res_Leg(void)  
 ��������    �����ȸ�λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Res_Leg(void)
{
	leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);  //����������
	leg_down_picture_k=19;
	WriteInUART4("LegDownUpPhone");
	Fun_Leg_Down();           
	delay_ms(1000);	

	WriteInUART4("LegUpUpPhone");
	Fun_Leg_Up();              //������
	delay_ms(1000);
	WriteInUART4("LegUpDownPhone");
	Fun_Leg_Up();              //�����ȸ�λ
	delay_ms(1000);		
}

/***********************************************************************
 ������      ��Res_Desk(void)  
 ��������    ���Ͳ�����һ������λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Res_Desk(void)
{
	desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
	desk_picture_k=19;
	WriteInUART4("DeskDownPhone");
	Fun_Desk();             //С���Ӹ�λ
	delay_ms(1000);	
}

	
/***********************************************************************
 ������      ��Res_Motor5(void)  
 ��������    ��5�ŵ����λ:
 ����        ������λ0;����λ1
 ���        ����
                           
************************************************************************/
void Res_Motor5(u8 dir) 
{
	u8 key;
	u16 arr_feed;      //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;   //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	DIR5=dir;  
	u8 direct=0;       
	Motor_5_START(motor_body_freq,motor_timer_freq);	//�������	
	TIM10_Init(20000,timer10_freq);                     //�򿪶�ʱ��
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
	{				
		//�����λ
		if((0==GD5_Start)&&(0==direct))   //������翪������ѭ�������ͣת 
		{		
			delay_us(100);
			if(0==GD5_Start)
			{
				break;		
			}				
		}
		//�������
//		if(1==Motor5_Alm)      
//		{
//			delay_us(100);
//			if(1==Motor5_Alm)
//			{
//				if(body_left_flag==1)
//				{
//					body_left_overload_5=1;
//					u2_printf("Reset_Body_Left_Overload_5");
//				}
//				if(body_right_flag==1)
//				{
//					body_right_overload_5=1;
//					u2_printf("Reset_Body_Right_Overload_5");
//				}
//				Breakdown_Treatment();
//				break;
//			}			
//		}
		
//		//���ʧ�����쳣ֹͣ
//		key=KEY_Scan(0);              //����ɨ��
//		if(key==Motor5_Tim_PRES)
//		{
//			num++;
//		}	
//		arr_feed=__HAL_TIM_GET_COUNTER(&TIM10_Handler); 
//		pulse_num=arr_feed/7.44*timer10_freq*0.36/(motor_body_freq*motor_timer_freq);						

//		if((abs(pulse_num,num)>150)&&pulse_num>0)
//		{
//			if(body_left_flag==1)
//			{
//				body_left_losepulse=1;
//				u2_printf("Reset_Body_Left_Losepulse");
//			}
//			if(body_right_flag==1)
//			{
//				body_right_losepulse=1;
//				u2_printf("Reset_Body_Right_Losepulse");
//			}				
//			Breakdown_Treatment();
//			break;
//		}		
	}		      
	Motor_5_STOP();     //���ֹͣ
	TIM10_Stop();       //�رն�ʱ��
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
	//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ�
//	if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD5_Start==1)&&(direct==0))
//	{  
//		DIR5=dir;
//		Motor_5_START(motor_body_freq,motor_timer_freq);    //�������
//		TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//		{			
//			if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//			{
//				break;																		
//			}
//		}			
//		Motor_5_STOP();     //���ֹͣ
//	    TIM10_Stop();       //�رն�ʱ��
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//	}	
}


/***********************************************************************
 ������      ��Res_Body_Left(void)  
 ��������    ������λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Res_Body_Left(void)
{	
	if(lock_flag==1)
	{	
		u2_printf("ResetBodyLeftStart");
		//4�ŵ����λ
		lock_flag==1;
		body_left_flag=1;
		back_nursing_left_flag==0;
		waist_nursing_left_dir_flag=1;	
		waist_nursing_left_picture_k=19;
		body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
		Fun_Waist_Nursing_Left();    //����������λ
		
		//3�ŵ����λ
		lock_flag=1;
		body_left_flag=1;
		waist_nursing_left_flag=0;
		back_nursing_left_dir_flag=1;
		back_nursing_left_picture_k=19;
		body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
		Fun_Back_Nursing_Left();     //�󱳲�����λ
		
		//5�ŵ����λ
		Res_Motor5(0); 
		body_left_runed_arr=0;
		body_left_flag=0;
		back_nursing_left_flag=0;
		waist_nursing_left_flag=0;
		back_nursing_left_dir_flag=0;
		waist_nursing_left_dir_flag=0;
		u2_printf("ResetBodyLeftRes");
	}
}

/***********************************************************************
 ������      ��Res_Body_Right(void)  
 ��������    ���ҷ���λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Res_Body_Right(void)
{
	if(lock_flag==1)
	{	
		u2_printf("ResetBodyRightStart");
		//4�ŵ����λ
		lock_flag==1;
		body_right_flag=1;
		back_nursing_right_flag==0;
		waist_nursing_right_dir_flag=1;	
		waist_nursing_right_picture_k=19;
		body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
		Fun_Waist_Nursing_Right();   //����������λ
		
		//3�ŵ����λ
		lock_flag=1;
		body_right_flag=1;
		waist_nursing_right_flag=0;
		back_nursing_right_dir_flag=1;
		back_nursing_right_picture_k=19;
		body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
		Fun_Back_Nursing_Right();    //�ұ�������λ
		
		//5�ŵ����λ 
		Res_Motor5(1);
		body_right_runed_arr=0;
		body_right_flag=0;
		back_nursing_right_flag=0;
		waist_nursing_right_flag=0;
		back_nursing_right_dir_flag=0;
		waist_nursing_right_dir_flag=0;
		u2_printf("ResetBodyRightRes");
	}
}


/***********************************************************************
 ������      ��back_angle_to_arr  
 ��������    �����Ƕ�ֵת�����Զ���װ��ֵ
 ����        ���Ƕȣ�u8����
 ���        ��TIM3�Զ���װ��ֵ��u16����
                           
************************************************************************/
u16 back_angle_to_arr(u8 angle)   //֧��
{
	u16 res;
	res=560*angle;//600
	return res;
}

u16 leg_angle_to_arr(u8 angle)   //����
{
	u16 res;
	res=380*angle;//400
	return res;
}

u16 body_angle_to_arr(u8 angle)  //����
{
	u16 res;
	res=300*angle;//280
	return res;
}

u16 desk_distance_to_arr(u8 distance)   //����
{
	u16 res;
	res=655*distance;//600
	return res;
}

/***********************************************************************
 ������      ��get_newangle_usart2
 ��������    ��ͨ������2����µĽǶ�ֵ
 ����        ����
 ���        ���� 
 ɨ������     ��USART2_RX_BUF��������2
 ����˵��        ������+���롰+���ַ�֮���������ȡ������������ȫ�ֵĽǶȱ���
 ����        ��angleresetall+80+40+90+45+45+100+
               ֧���Ƕȣ�back_angle_lim��=80�������ȽǶȣ�leg_up_angle_lim��=40�������ȽǶȣ�leg_down_angle_lim��=90��
               �󷭽Ƕȣ�body_left_angle_lim��=45���ҷ��Ƕȣ�body_right_angle_lim��=45��С�����ƶ����루desk_distance_lim��=100��
************************************************************************/
void get_newangle_usart2(void)
{
	u8 ip1[10],ip2[10],ip3[10],ip4[10],ip5[10],ip6[10],ip7[10];
	u8 	back_angle_lim_buf[1];
	u8  leg_up_angle_lim_buf[1];
	u8  leg_down_angle_lim_buf[1];
	u8  body_left_angle_lim_buf[1];
	u8  body_right_angle_lim_buf[1];
	u8  desk_distance_lim_buf[1];
	u8  swash_dry_time_buf[1];
	
	char *presult1,*presult2,*presult3,*presult4,*presult5,*presult6,*presult7,*presult8;
	
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	presult3 = strstr( (const char *)presult2+1 , (const char *)"+");		
	presult4 = strstr( (const char *)presult3+1 , (const char *)"+");			
	presult5 = strstr( (const char *)presult4+1 , (const char *)"+");		
 	presult6 = strstr( (const char *)presult5+1 , (const char *)"+");
	presult7 = strstr( (const char *)presult6+1 , (const char *)"+");
	presult8 = strstr( (const char *)presult7+1 , (const char *)"+");
	
 //��presult1+1��presult2-presult1-1֮������ݸ��Ƶ�ip1������
	memcpy(ip1,presult1+1,presult2-presult1-1);   
	memcpy(ip2,presult2+1,presult3-presult2-1);
	memcpy(ip3,presult3+1,presult4-presult3-1);	
	memcpy(ip4,presult4+1,presult5-presult4-1);		
	memcpy(ip5,presult5+1,presult6-presult5-1);	
	memcpy(ip6,presult6+1,presult7-presult6-1);
	memcpy(ip7,presult7+1,presult8-presult7-1);

	back_angle_lim_buf[0]=usmart_strnum(ip1);              //֧��  //��ip1�����е��������ַ���ת��������
	leg_up_angle_lim_buf[0]=usmart_strnum(ip2);            //������
	leg_down_angle_lim_buf[0]=usmart_strnum(ip3);          //������
	body_left_angle_lim_buf[0]=usmart_strnum(ip4);         //��
	body_right_angle_lim_buf[0]=usmart_strnum(ip5);        //�ҷ�
	desk_distance_lim_buf[0]=usmart_strnum(ip6);           //�����ƶ�����
	swash_dry_time_buf[0]=usmart_strnum(ip7);              //��ϴ���ʱ��	

	if(back_angle_lim_buf[0]<=90)             //֧��
	{	back_angle_lim=back_angle_lim_buf[0];	}
	
	if(leg_up_angle_lim_buf[0]<=40)  	     //������
	{	leg_up_angle_lim=leg_up_angle_lim_buf[0]; 	} 
	
	if(leg_down_angle_lim_buf[0]<=90)	      //������
	{	leg_down_angle_lim=leg_down_angle_lim_buf[0]; 	}

	if(body_left_angle_lim_buf[0]<=90)	       //��
	{	body_left_angle_lim=body_left_angle_lim_buf[0];	}  
	
	if(body_left_angle_lim_buf[0]<=90)	       //�ҷ�	
	{	body_right_angle_lim=body_right_angle_lim_buf[0]; 	} 

	if(desk_distance_lim_buf[0]<=100)          //�����ƶ�����   
	{	desk_distance_lim=desk_distance_lim_buf[0];		}

	if(swash_dry_time_buf[0]<=5)             //��ϴ���ʱ��  
	{	swash_dry_time=swash_dry_time_buf[0];		}
 	

//	//����������λ���趨����ֵ
//	W25QXX_Write((u8*)back_angle_lim_buf,13,1);         //�ӵ�13��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)leg_up_angle_lim_buf,14,1);		//�ӵ�14��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)leg_down_angle_lim_buf,15,1);		//�ӵ�15��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)body_left_angle_lim_buf,16,1);	//�ӵ�16��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)body_right_angle_lim_buf,17,1);	//�ӵ�17��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		//�ӵ�18��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)swash_dry_time_buf,19,1);		    //�ӵ�19��ַ��ʼ��д��1���ֽ�	
}

/***********************************************************************
 ������      ��get_newangle_wifi()
 ��������    ��ͨ��WiFi����µĽǶ�ֵ
 ����        ����
 ���        ���� 
 ɨ������     ��UART4_RX_BUF��������4
 ����˵��        ������+���롰+���ַ�֮���������ȡ������������ȫ�ֵĽǶȱ���
 ����        ��angleresetall+80+40+90+45+45+100+
               ֧���Ƕ�  ��back_angle_lim��=80��
			   �����ȽǶȣ�leg_up_angle_lim��=40��
			   �����ȽǶȣ�leg_down_angle_lim��=90��
               �󷭽Ƕ�  ��body_left_angle_lim��=45��
			   �ҷ��Ƕ�  ��body_right_angle_lim��=45��
			   С�����ƶ����루desk_distance_lim��=100��
				��ϴ���ʱ�� 
		
************************************************************************/
void get_newangle_wifi(void)
{
	u8 ip1[10],ip2[10],ip3[10],ip4[10],ip5[10],ip6[10],ip7[10];
	
	u8 	back_angle_lim_buf[1];
	u8  leg_up_angle_lim_buf[1];
	u8  leg_down_angle_lim_buf[1];
	u8  body_left_angle_lim_buf[1];
	u8  body_right_angle_lim_buf[1];
	u8  desk_distance_lim_buf[1];
	u8  swash_dry_time_buf[1];
	
	char *presult1,*presult2,*presult3,*presult4,*presult5,*presult6,*presult7,*presult8;
	presult1 = strstr( (const char *)UART4_RX_BUF ,(const char *)"+");
	presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	presult3 = strstr( (const char *)presult2+1 , (const char *)"+");		
	presult4 = strstr( (const char *)presult3+1 , (const char *)"+");			
	presult5 = strstr( (const char *)presult4+1 , (const char *)"+");		
 	presult6 = strstr( (const char *)presult5+1 , (const char *)"+");
	presult7 = strstr( (const char *)presult6+1 , (const char *)"+");	
	presult8 = strstr( (const char *)presult7+1 , (const char *)"+");
	
	//��presult1+1��presult2-presult1-1֮������ݸ��Ƶ�ip1������
	memcpy(ip1,presult1+1,presult2-presult1-1);    
	memcpy(ip2,presult2+1,presult3-presult2-1);
	memcpy(ip3,presult3+1,presult4-presult3-1);	
	memcpy(ip4,presult4+1,presult5-presult4-1);		
	memcpy(ip5,presult5+1,presult6-presult5-1);	
	memcpy(ip6,presult6+1,presult7-presult6-1);	
	memcpy(ip7,presult7+1,presult8-presult7-1);
	
	//��ip1�����е��������ַ���ת��������
	back_angle_lim_buf[0]=usmart_strnum(ip1);              //֧��  
	leg_up_angle_lim_buf[0]=usmart_strnum(ip2);            //������
	leg_down_angle_lim_buf[0]=usmart_strnum(ip3);          //������
	body_left_angle_lim_buf[0]=usmart_strnum(ip4);         //��
	body_right_angle_lim_buf[0]=usmart_strnum(ip5);        //�ҷ�
	desk_distance_lim_buf[0]=usmart_strnum(ip6);           //�����ƶ�����
	swash_dry_time_buf[0]=usmart_strnum(ip7);              //��ϴ���ʱ��
	
	if(back_angle_lim_buf[0]<=90)             //֧��
	{	back_angle_lim=back_angle_lim_buf[0];	}
	
	if(leg_up_angle_lim_buf[0]<=40)  	     //������
	{	leg_up_angle_lim=leg_up_angle_lim_buf[0]; 	} 
	
	if(leg_down_angle_lim_buf[0]<=90)	      //������
	{	leg_down_angle_lim=leg_down_angle_lim_buf[0]; 	}

	if(body_left_angle_lim_buf[0]<=90)	       //��
	{	body_left_angle_lim=body_left_angle_lim_buf[0];	}  
	
	if(body_left_angle_lim_buf[0]<=90)	       //�ҷ�	
	{	body_right_angle_lim=body_right_angle_lim_buf[0]; 	} 

	if(desk_distance_lim_buf[0]<=100)         //�����ƶ�����   
	{	desk_distance_lim=desk_distance_lim_buf[0];		} 

	if(swash_dry_time_buf[0]<=5)             //��ϴ���ʱ��  
	{	swash_dry_time=swash_dry_time_buf[0];		}	
	
//	//����������λ���趨����ֵ	
//	W25QXX_Write((u8*)back_angle_lim_buf,13,1);         //�ӵ�13��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)leg_up_angle_lim_buf,14,1);	    //�ӵ�14��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)leg_down_angle_lim_buf,15,1);		//�ӵ�15��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)body_left_angle_lim_buf,16,1);	//�ӵ�16��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)body_right_angle_lim_buf,17,1);	//�ӵ�17��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		//�ӵ�18��ַ��ʼ��д��1���ֽ�
//	W25QXX_Write((u8*)swash_dry_time_buf,19,1);		    //�ӵ�19��ַ��ʼ��д��1���ֽ�	
}

/***********************************************************************
 ������      ��Read_Angle  
 ��������    ����ȡflash�������λ���趨�Ƕ�ֵ
 ����        ���Ƕȣ�u8����
 ���        ��TIM3�Զ���װ��ֵ��u16����
                           
************************************************************************/
void Read_Angle(void)
{
	u8 	back_angle_lim_buf[1];
	u8  leg_up_angle_lim_buf[1];
	u8  leg_down_angle_lim_buf[1];
	u8  body_left_angle_lim_buf[1];
	u8  body_right_angle_lim_buf[1];
	u8  desk_distance_lim_buf[1];
	u8  swash_dry_time_buf[1];
	
	//����������ݴ�flash�ж�����
	W25QXX_Read((u8*)back_angle_lim_buf,13,1);        //�ӵ�13��ַ��ʼ����ȡ1���ֽ�
	W25QXX_Read((u8*)leg_up_angle_lim_buf,14,1);	  //�ӵ�14��ַ��ʼ����ȡ1���ֽ�
	W25QXX_Read((u8*)leg_down_angle_lim_buf,15,1);	  //�ӵ�15��ַ��ʼ����ȡ1���ֽ�
	W25QXX_Read((u8*)body_left_angle_lim_buf,16,1);	  //�ӵ�16��ַ��ʼ����ȡ1���ֽ�
	W25QXX_Read((u8*)body_right_angle_lim_buf,17,1);  //�ӵ�17��ַ��ʼ����ȡ1���ֽ�
	W25QXX_Read((u8*)desk_distance_lim_buf,18,1);	  //�ӵ�18��ַ��ʼ����ȡ1���ֽ�
	W25QXX_Read((u8*)swash_dry_time_buf,19,1);		  //�ӵ�19��ַ��ʼ����ȡ1���ֽ�

	back_angle_lim=back_angle_lim_buf[0];              //֧��  
	leg_up_angle_lim=leg_up_angle_lim_buf[0];          //������
	leg_down_angle_lim=leg_down_angle_lim_buf[0];      //������
	body_left_angle_lim=body_left_angle_lim_buf[0];    //��
	body_right_angle_lim=body_right_angle_lim_buf[0];  //�ҷ�
	desk_distance_lim=desk_distance_lim_buf[0];        //�����ƶ�����
	swash_dry_time=swash_dry_time_buf[0];              //�����ϴ���ʱ��	
}

/***********************************************************************
 ������      ��Wifi_Send  
 ��������    ��WiFi���ͺ���
 ����        ��Ҫ���͵����ݻ�ָ��
 ���        ����
                           
************************************************************************/
void Wifi_Send(u8 *data)
{
	u16 i;
	u8 n;	
	i=strlen((const char*)data);                    //��ȡ���ݳ���	
	if(device_num>0)
	{
		u4_printf("AT+CIPSEND=%d,%d\r\n",0,i);      //����AT+CIPSENDָ��
		delay_ms(4);                         
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//�������4���ջ�����	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);                       //���������ַ�����data
		delay_ms(200);								//��ʱ200ms��оƬ����Ҫ�����ʱ��
		memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//�������4���ջ�����	
		UART4_RX_LEN=0;  				
	}
}

/***********************************************************************
 ������      ��Wifi_ToPC  
 ��������    ��WiFi��PC���ͺ���
 ����        ��Ҫ���͵����ݻ�ָ��
 ���        ����                      
************************************************************************/
void Wifi_ToPC(u8 *data)
{
	u16 i;
	u8 n;	
	if(PC_Ready==1)
	{
		i=strlen((const char*)data);                //��ȡ���ݳ���	
		u4_printf("AT+CIPSEND=%d,%d\r\n",PC,i);     //����AT+CIPSENDָ��
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	//�������4���ջ�����	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);       //���������ַ�����data
		delay_ms(200);				//��ʱ200ms��оƬ����Ҫ�����ʱ��
		memset(UART4_RX_BUF,0,20);	//�������4���ջ�����	
		UART4_RX_LEN=0;  		
	}
}

/***********************************************************************
 ������      ��Wifi_ToPhone  
 ��������    ��WiFi���ֻ����ͺ���
 ����        ��Ҫ���͵����ݻ�ָ��
 ���        ����
************************************************************************/
void Wifi_ToPhone(u8 *data)
{
	u16 i;
	u8 n;	
	if(Phone_Ready==1)
	{
		i=strlen((const char*)data);                //��ȡ���ݳ���	
		u4_printf("AT+CIPSEND=%d,%d\r\n",Phone,i);  //����AT+CIPSENDָ��
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	//�������4���ջ�����	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);       //���������ַ�����data
		delay_ms(200);				//��ʱ200ms��оƬ����Ҫ�����ʱ��
		memset(UART4_RX_BUF,0,20);	//�������4���ջ�����	
		UART4_RX_LEN=0;  		
	}
}


/***********************************************************************
 ������      ��Wifi_ToRemote  
 ��������    ��WiFi��ң�������ͺ���
 ����        ��Ҫ���͵����ݻ�ָ��
 ���        ����
************************************************************************/
void Wifi_ToRemote(u8 *data)
{
	u16 i;
	u8 n;	
	if(RemoteCtl_Ready==1)
	{
		i=strlen((const char*)data);                        //��ȡ���ݳ���	
		u4_printf("AT+CIPSEND=%d,%d\r\n",Remote_Ctl,i);     //����AT+CIPSENDָ��
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	//�������4���ջ�����	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);       //���������ַ�����data
		delay_ms(200);				//��ʱ200ms��оƬ����Ҫ�����ʱ��
		memset(UART4_RX_BUF,0,20);	//�������4���ջ�����	
		UART4_RX_LEN=0; 		
	}
}
/***********************************************************************
 ������      ��Wifi_ToGuard  
 ��������    ��WiFi�������ͺ���
 ����        ��Ҫ���͵����ݻ�ָ��
 ���        ����
************************************************************************/
void Wifi_ToGuard(u8 *data)
{
	u16 i;
	u8 n;	
	if(GuardCtl_Ready==1)
	{
		i=strlen((const char*)data);                       //��ȡ���ݳ���	
		u4_printf("AT+CIPSEND=%d,%d\r\n",Guard_Ctl,i);     //����AT+CIPSENDָ��
		delay_ms(4);
		memset(UART4_RX_BUF,0,20);	 //�������4���ջ�����	
		UART4_RX_LEN=0;                               
		u4_printf("%s",data);        //���������ַ�����data
		delay_ms(200);				 //��ʱ200ms��оƬ����Ҫ�����ʱ��
		memset(UART4_RX_BUF,0,20);	 //�������4���ջ�����	
		UART4_RX_LEN=0; 		
	}
}

/***********************************************************************
 ������      ��Uart_ToStick  
 ��������    ��������PC�����ͺ���
 ����        ��Ҫ���͵����ݻ�ָ��
 ���        ����
************************************************************************/
void Uart_ToStick(u8 *data)
{
	if(ComputerStick_Ready==1)
	{                             
		u2_printf("%s",data);	
		memset(USART2_RX_BUF,0,20);
		USART2_RX_LEN=0; 		
	}
}

/***********************************************************************
                    
					     ���ں���(������)
					
************************************************************************/
/***********************************************************************
 ������      ��Uart_Back(void)   
 ��������    ��ִ��֧������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Back(void)
{
	u8 direct,len;
	u16 arr_now;              //��ǰһ������������
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;          //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	static u8 k;             //����k�Ŷ���
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //��ǰһ����������ֵ
	static u8 back_limit_flag; //֧�����е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧��
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BackUpPhone"))
		{
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{				
				direct=1;
				back_dir_flag=1;      
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(400);
					u2_printf("back_flag==1");
					delay_ms(400);
					u2_printf("BackStart");
					delay_ms(400);
					u2_printf("Cartoon_Back_1");
					delay_ms(400);
				}
				Motor_1_START(1);                                                          //֧������
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq); //�򿪶�ʱ��
			}				
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownPhone"))
		{
			if(back_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				direct=0;
				back_dir_flag=0; 
				if(1==back_limit_flag)
				{
					back_limit_flag=0;
					delay_ms(200);
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);                                                          //֧������
				TIM10_Init(back_runed_arr,timer10_freq); //�򿪶�ʱ��
			}
		}
		
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;									
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
		
	 	if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{					
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
			{							
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{			
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BackUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownPhone")))    //�����յ�Stop,������ѭ��	
						{					
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;						
						}
						else 
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //��ǰһ������ֵ
				//���䶯��ָ��
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				k=back_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);					
					if(kj<2)
					{
						k=j;   back_picture_k=k;					
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Back_2");									
									break;
							case 2:	u2_printf("Cartoon_Back_3");
									break;					
							case 3:	u2_printf("Cartoon_Back_4");
									break;					
							case 4:	u2_printf("Cartoon_Back_5");
									break;	
							case 5:	u2_printf("Cartoon_Back_6");
									break;	
							case 6:	u2_printf("Cartoon_Back_7");
									break;
							case 7:	u2_printf("Cartoon_Back_8");
									break;
							case 8:	u2_printf("Cartoon_Back_9");
									break;						
							case 9:	u2_printf("Cartoon_Back_10");
									break;												
							case 10:u2_printf("Cartoon_Back_11");
									break;
							case 11:u2_printf("Cartoon_Back_12");									
									break;
							case 12:u2_printf("Cartoon_Back_13");
									break;					
							case 13:u2_printf("Cartoon_Back_14");
									break;					
							case 14:u2_printf("Cartoon_Back_15");
									break;	
							case 15:u2_printf("Cartoon_Back_16");
									break;	
							case 16:u2_printf("Cartoon_Back_17");
									break;
							case 17:u2_printf("Cartoon_Back_18");
									break;
							case 18:u2_printf("Cartoon_Back_19");
									break;		
						}
					}				
				}	
			}		
		Motor_1_STOP();    //���ֹͣ
		TIM10_Stop();      //�رն�ʱ��
		break_flag=0;      //��־λ����
		//�жϸ�λ
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
		{				
			arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
			back_flag=0;
			delay_ms(200);
			u2_printf("back_flag==0");
			delay_ms(200);			
		}		
		else
		{
			arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
			back_flag=1;
		}
      //ͨ���������ж������ۼ�		
		if(	direct==1)        //�����֧�����У�����+
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{  
				back_runed_arr=back_angle_to_arr(back_angle_lim);
				back_limit_flag=1; 
				delay_ms(200);
				u2_printf("Cartoon_Back_20");
				delay_ms(200);
				u2_printf("BackLim");
				delay_ms(200);
			}
			else
			{  back_runed_arr+=arr_now;	}				
		}
		else                //�����֧�����У�����-
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{	
				back_runed_arr=0;
				delay_ms(200);
				u2_printf("Cartoon_Back_1");				
				delay_ms(200);
				u2_printf("BackRes");
				delay_ms(200);
			}
			else
			{
				back_runed_arr-=arr_now;
			}						
		}		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
	}
}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}
/***********************************************************************
 ������      ��Uart_Leg_Up(void)   
 ��������    ��ִ�������Ȳ���
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Leg_Up(void)
{
	u16 arr_now;              //��ǰһ������������
	u8 len;                   //WiFi���ڽ����ַ�������
	u8 direct;	              //���з����־λ
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�жϳ����Ƿ�ͨ��break���� 
	static u8 k=0;            //�����k�Ŷ���ָ��
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;               //��ǰһ����������ֵ
	static u8 leg_up_limit_flag;//���������е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�������ȸ�λ����ܽ���������
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_down_flag==0)&&(lock_flag==1))
	{		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpPhone"))  //����������	
		{				   
			if(leg_angle_to_arr(leg_up_angle_lim)>leg_up_runed_arr)             //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=1; 
				if(leg_up_flag==0)
				{
					leg_up_flag=1;
					delay_ms(200);					
					u2_printf("leg_up_flag==1");
					delay_ms(200);
					u2_printf("LegUpStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_1");
					delay_ms(200);
				}			
								
				Push_Rod_Start(0);
				TIM10_Init(leg_angle_to_arr(leg_up_angle_lim)-leg_up_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz					
			} 
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownPhone"))  //����������
		{			
			if(leg_up_runed_arr>0)                          //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				direct=0;
				if(leg_up_limit_flag==1)
				{
					leg_up_limit_flag=0;
					delay_ms(200);
					u2_printf("Cartoon_Leg_Up_8");
					delay_ms(200);
				}										
				Push_Rod_Start(1);
				TIM10_Init(leg_up_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}
		}
		
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;				
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
	 	if(((leg_up_runed_arr!=leg_angle_to_arr(leg_up_angle_lim))&&(1==direct))||((0!=leg_up_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)	
				{	
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownPhone")))    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}						
						else
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//�жϴ��䶯��ָ��
				if(direct==1)
				{
					j=(leg_up_runed_arr+arr_send)/(leg_angle_to_arr(leg_up_angle_max)/7);
				}
				else
				{
					j=abs(leg_up_runed_arr,arr_send)/(leg_angle_to_arr(leg_up_angle_max)/7);
				}
				k=leg_up_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);					
					if(kj<2)
					{
						k=j;    leg_up_picture_k=k;						
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Leg_Up_2");									
									break;
							case 2:	u2_printf("Cartoon_Leg_Up_3");
									break;					
							case 3:	u2_printf("Cartoon_Leg_Up_4");
									break;					
							case 4:	u2_printf("Cartoon_Leg_Up_5");
									break;	
							case 5:	u2_printf("Cartoon_Leg_Up_6");
									break;	
							case 6:	u2_printf("Cartoon_Leg_Up_7");
									break;
						}
					}					
				}				
			}				//�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
	
		Push_Rod_Stop();    //�Ƹ�ֹͣ
		TIM10_Stop();       //�رն�ʱ��
		break_flag=0;       //��־λ����
		//�жϸ�λ
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0)) //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
		{
			arr_now=0;                  //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
			leg_up_flag=0;
			delay_ms(200);
			u2_printf("leg_up_flag==0");
			delay_ms(200);
		}
		else
		{
			arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);       
			leg_up_flag=1;
		}			
		 //ͨ���������ж������ۼ�	
		if(direct==1)    //��������������У�����+
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{
				leg_up_runed_arr=leg_angle_to_arr(leg_up_angle_lim);
				leg_up_limit_flag=1;	
				delay_ms(200);
				u2_printf("Cartoon_Leg_Up_8");
				delay_ms(200);
				u2_printf("LegUpLim");
				delay_ms(200);
			}
			 else
			{
				leg_up_runed_arr=leg_up_runed_arr+arr_now;
			}	
		}
		else     //��������������У�����-
		{
			if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
			{
				leg_up_runed_arr=0;	
				delay_ms(200);
				u2_printf("Cartoon_Leg_Up_1");
				delay_ms(200);
				u2_printf("LegUpRes");
				delay_ms(200);
			}
			 else
			{
				leg_up_runed_arr=leg_up_runed_arr-arr_now;
			}			
		}	
		 __HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 		
	 }
   }	
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("LegUpInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 ������      ��Uart_Leg_Down(void)   
 ��������    ��ִ�������Ȳ���
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Leg_Down(void)
{
	u16 arr_now;         //��ǰһ������������   
	u8 len;              //���յ��ַ�������
	u8 direct;           //����ĳ���������еķ����־��1-�������У�0-��������
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;     //�ж��Ƿ�ͨ��break����ѭ�� 
	static u8 k=0;       //���͵�K�Ŷ���ָ��
	static u8 kj=0;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;        //��ǰһ������������
	static u8 leg_down_limit_flag;//���������е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�������ȸ�λ����ܽ���������
	if((body_left_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(lock_flag==1))
	{
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownPhone"))   //����������
		{				
			if(leg_angle_to_arr(leg_down_angle_lim)>leg_down_runed_arr)             //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=1;
				if(leg_down_flag==0)
				{
					leg_down_flag=1;
					delay_ms(200);
					u2_printf("leg_down_flag==1");
					delay_ms(200);
					u2_printf("LegDownStart");
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_1");
					delay_ms(200);
				}							
				Push_Rod_Start(1);
				TIM10_Init(leg_angle_to_arr(leg_down_angle_lim)-leg_down_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
				leg_down_flag=1;
			}	
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpPhone"))  //����������
		{
			
			if(leg_down_runed_arr>0)     //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=0;
				if(leg_down_limit_flag==1)
				{
					leg_down_limit_flag=0;
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
				}							
				Push_Rod_Start(0);
				TIM10_Init(leg_down_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //����жϱ�־λ	

	   if(((leg_down_runed_arr!=leg_angle_to_arr(leg_down_angle_lim))&&(1==direct))||((0!=leg_down_runed_arr)&&(0==direct)))
	   {			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{						
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpPhone")))    //�����յ�Stop,������ѭ��	
						{				
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}					
					}
				 }
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				//���Ͷ���ָ��
				if(direct==1)
				{
					j=(leg_down_runed_arr+arr_send)/(leg_angle_to_arr(leg_down_angle_max)/19);
				}
				else
				{
					j=abs(leg_down_runed_arr,arr_send)/(leg_angle_to_arr(leg_down_angle_max)/19);
				}
				k=leg_down_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;   leg_down_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Leg_Down_2");									
									break;
							case 2:	u2_printf("Cartoon_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Leg_Down_12");									
									break;
							case 12:u2_printf("Cartoon_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Leg_Down_15");
									break;	
							case 15:u2_printf("Cartoon_Leg_Down_16");
									break;	
							case 16:u2_printf("Cartoon_Leg_Down_17");
									break;
							case 17:u2_printf("Cartoon_Leg_Down_18");
									break;
							case 18:u2_printf("Cartoon_Leg_Down_19");
									break;																		
						}
					}				
				}				 
			}				   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��	      
			Push_Rod_Stop();   //�Ƹ�ֹͣ
			TIM10_Stop();      //�رն�ʱ��
			break_flag=0;	   //���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;                 //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				leg_down_flag=0;
				delay_ms(200);			
				u2_printf("leg_down_flag==0");
				delay_ms(200);
			}
			else
			{			
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);     
				leg_down_flag=1;
			}			
			//ͨ���������ж������ۼ�
			if(direct==1)    //��������������У�����+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);
					leg_down_limit_flag=1;	   
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_20");
					delay_ms(200);
					u2_printf("LegDownLim");
					delay_ms(200);
				}
				 else
				{
					leg_down_runed_arr=leg_down_runed_arr+arr_now;
				}	
			}
			else		//��������������У�����-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{
					leg_down_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Leg_Down_1");
					delay_ms(200);
					u2_printf("LegDownRes");
					delay_ms(200);
				}
				 else
				{
					leg_down_runed_arr=leg_down_runed_arr-arr_now;
				}			
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 					
		}
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("LegDownInterfere");		
		LED0=1;
		LED1=1;	
	}
}

/***********************************************************************
 ������      ��Uart_Body_Left(void)  
 ��������    ��ִ���������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;                 //��ǰһ������������,���������ۼ�
	u8 len;                      //���յ��ַ�������
	u16 arr_feed;                //��������е�ǰһ�������������������жϵ��ʧ������
	u16 pulse_num=0;             //������۽��յ�������ֵ
	u16 num1=0,num2=0,num3=0;    //���ʵ�����е�����ֵ	
	static u8 motor5_run_flag;   //�ж�С�෭�Ƿ��Ѿ���������������λ��1 
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;            //�жϳ����Ƿ��break����
	static u8 k=0,m=0;
	u8 i=0,i1=0;
	u8 j=0,n=0;
	u16 arr_send,arr_send1;     //��ǰһ������������
	static u8 kj;
	static u8 M345_Start;       //345�����һ������
	static u8 M345_End;         //345������е��ϼ���λ��
	static u8 mn;
	u8 key1;
	
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ��������
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		//С�෭����
		if(body_left_flag==0)   //�����λ����ʼ״̬����ִ������
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				//5�Ų෭��			
				DIR5=1; 
				body_left_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1); 
				delay_ms(200);
				u2_printf("body_left_flag==1");
				delay_ms(200);
				u2_printf("BodyLeftStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	 //�������	
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);                      //�򿪶�ʱ��
				
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)) )
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{					
						//�����λ
//						if((GDCheck(GD5LE))&&(1==body_left_flag))
//						{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;
//						}
						
//						if((0==GD5_Left_End)&&(1==body_left_flag))        //������翪������ѭ�������ͣת 
//						{						
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;	
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
						if(USART2_RX_LEN&0x8000)
						{				
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;											
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
				  arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);						
						if(mn<2)
						{
							m=n;   left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;												
							}
						}
					}
				}
				Motor_5_STOP();            //���5ֹͣ
				TIM10_Stop();              //��ʱ���ر�
				break_flag=0;              //�����־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(0,bodyleft_compleate,motor_body_freq,motor_timer_freq);	//���ò�������
			}
		}	
		//����345�ŵ������	
		if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))||(1==motor5_run_flag))
		{				
			if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
			{
			     motor5_run_flag=0;
				 
				//345��������			
				 DIR3=0;DIR4=0;DIR5=0;direct=1;body_left_dir_flag=1;
				 if(M345_Start==0)
				 {
					 M345_Start=1;
					 delay_ms(100);
					 u2_printf("Cartoon_Body_Left_1");
					 //delay_ms(200);
				 }				 
				 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);				
				 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);			
			}
		}		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownPhone"))
		{					
			if(body_left_runed_arr>0)
			{			   
				//345��������	
			     DIR3=1;DIR4=1;DIR5=1;direct=0;body_left_dir_flag=0;
				 if(M345_End==1)
				 {
					 M345_End=0;
					 delay_ms(200);
					 u2_printf("Cartoon_Body_Left_8");	
					 delay_ms(200);
				 }
			     Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	//�������			
			     TIM10_Init(body_left_runed_arr,timer10_freq);			    //�򿪶�ʱ��
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             // ����жϱ�־λ
		if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//�����λ
//					if(GDCheck(GD34S)&&(0==direct))
//					{
//						if(0==GD3_Start)
//						{u2_printf("GD3S==0\r\n");}
//						if(0==GD4_Start)
//						{u2_printf("GD4S==0\r\n");}
//							break_flag=1;
//							break;			
//					}
//					else if(GDCheck(GD34LE)&&(1==direct))
//					{			     
//						 if(0==GD3_Left_End)
//						 {
//							 u2_printf("\r\n0==GD3_Left_End\r\n");
//						 }
//						 if(0==GD4_Left_End)
//						 {
//						  	u2_printf("\r\n0==GD4_Left_End\r\n");
//						 }
//						 break_flag=1;
//					   break;
//					}
					
					
					
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("\r\n0==GD3_Start\r\n");
//							break_flag=1;
//							break;							
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("\r\n0==GD4_Start\r\n");
//							break_flag=1;
//							break;					
//						}											
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{ 	
//						delay_us(100);
//						if(((0==GD3_Left_End)||(0==GD4_Left_End)))
//						{
//							break_flag=1;
//							if(0==GD3_Left_End)
//							{
//								u2_printf("\r\n0==GD3_Left_End\r\n");
//							}
//							else if(0==GD4_Left_End)
//							{
//								u2_printf("\r\n0==GD4_Left_End\r\n");
//							}
//						}						
//						break;
//					}
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownPhone")))    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						} 
						else 
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���								
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}				
					//������ϡ��������
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))       
//					{	
//						delay_us(20);		
//						if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))       
//						{	
//							if(1==Motor3_Alm)
//							{
//								body_left_overload_3=1;
//								u2_printf("\r\nBody_Left_Overload_3\r\n");
//							}
//							else if(1==Motor4_Alm)
//							{
//								body_left_overload_4=1;
//								u2_printf("\r\nBodyLeftOverload4\r\n");
//							}
//							else if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("\r\nBodyLeftOverload5\r\n");
//							}					
//							Breakdown_Treatment();
//							break_flag=1;
//							break;
//						}						
//					}						
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				k=body_left_picture_k;
				if(k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{	
						k=j;  body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;
						}
					}				
				}				
			}							
			Motor_3_4_5_STOP();    //���ֹͣ
			TIM10_Stop();          //�رն�ʱ��
			break_flag=0;		   //�����־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_left_flag=0;
//				W25QXX_Write((u8*)&body_left_flag,33,1);				
			}		
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now
				body_left_flag=1;
//				W25QXX_Write((u8*)&body_left_flag,33,1);														
			}
			//ͨ���������ж������ۼ�
			if(direct==1)           //���У�����+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
					M345_End=1;  					
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					u2_printf("BodyLeftLim");
					delay_ms(200);
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else     //���У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{
					body_left_runed_arr=0;
					M345_Start=0;  					
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
					delay_ms(200);
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;					
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);    //����жϱ�־λ
			
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //���3û��λ
//				{
//					u2_printf("\r\n3�ŵ���������е����λ��\r\n");
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{		
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								body_left_flag=0;			
//								u2_printf("GD3Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_3_STOP();
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))         //���4û��λ
//				{
//					u2_printf("\r\n4�ŵ���������е����λ��\r\n");
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD4_Start) 
//							{
//								body_left_flag=0;
//								u2_printf("GD4Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_4_STOP();       //���ֹͣ
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //���3/4��û��λ
//				{
//					u2_printf("\r\n3�ż�4�ŵ��ͬʱ�������е����λ��\r\n");
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if((0==GD3_Start)||(0==GD4_Start))
//							{
//								if(0==GD3_Start)
//								{
//									body_left_flag=0;
//									u2_printf("GD3Start");
//									break;
//								}
//								if(0==GD4_Start)
//								{
//									body_left_flag=0;
//									u2_printf("GD4Start");
//									break;
//								}
//							}								
//						}
//					}			
//					Motor_3_4_5_STOP();   //���ֹͣ
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
			//5�ŵ����λ
			if(body_left_flag==0)     //345������λ����ʼ״̬���Ÿ�λ5�ŵ��
			{			
				//5�Ų෭��λ
				Motor_4_Compensate(1,bodyleft_compleate,motor_body_freq,motor_timer_freq);
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;		
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);  //5�ŵ������
				body_left_runed_arr=0;			
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);                     //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//�����λ
//						if(GDCheck(GD5S)&&(0==body_left_flag))
//						{
//						   break_flag=1;
//						   break;
//						}
						
//						if(((0==GD5_Start)&&(0==body_left_flag)) )   //������翪������ѭ�������ͣת 
//						{	
//							delay_us(20);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;						
//							}		             
//						}
						  //�ж���û���յ���λ��ָ��		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
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
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;
					m=left_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;  left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}
						}
					}
				}      
				Motor_5_STOP();       //���ֹͣ
				TIM10_Stop();         //�رն�ʱ��
				break_flag=0;         //�����־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");				
				delay_ms(200);
				u2_printf("body_left_flag==0");
				delay_ms(200);
				u2_printf("BodyLeftRes");
				delay_ms(200);
				//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
			}
			W25QXX_Write((u8*)&body_left_flag,33,1);			
		}
	}
	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Body_Right(void)  
 ��������    ��ִ���ҷ������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;        //��ǰһ�����������������������ۼ�
	u8 len;             //���յ��ַ�������
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num1=0,num2=0,num3=0;
	
	static u8 motor5_run_flag;  //С�෭�����б�־λ
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //�жϳ����Ƿ��break���� 
	static u16 k=0,m=0;
	static u8 M345R_Start=0;  //345����ӳ�ʼλ������
	static u8 M345R_End=0;    //345��������ϼ���λ��
	u8 mn;
	u8 kj;

	u8 i=0,i1=0;
	u16 j=0,n=0;
	u16 arr_send,arr_send1;	  //��ǰһ������������
	
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ����������
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{
		if(body_right_flag==0)   //�����λ����ʼ״̬����ִ���ҷ���
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))
			{
			 //5�Ų෭��
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				DIR5=0; 
				body_right_flag=1;
				motor5_run_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);
				delay_ms(200);
				u2_printf("body_right_flag==1");
				delay_ms(200);
				u2_printf("BodyRightStart");
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);	//�������	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                     //�򿪶�ʱ��  
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//�����λ
//						if((GDCheck(GD5RE))&&(1==body_right_flag))
//						{
//								break_flag=1;
//								u2_printf("GD5RightEnd");
//								break;
//						}
										
//						if((0==GD5_Right_End)&&(1==body_right_flag))                     //������翪������ѭ�������ͣת 
//						{	
//							delay_us(100);
//							if(0==GD5_Right_End)                    //������翪������ѭ�������ͣת 
//							{
//								u2_printf("GD5RightEnd");
//								break_flag=1;
//								break;						
//							}		             
//						}
						  //�ж���û���յ���λ��ָ��		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}												
						//������ϡ��������
//						if(1==Motor5_Alm)        
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)        
//							{	
//								body_right_overload_5=1;
//								u2_printf("Body_Right_Overload_5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;						
//							}		             
//						}											
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//����ͼƬָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(	m!=n)
					{	
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch(m)
							{								
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;
							}
						}
					}
				}					      
				Motor_5_STOP();       //���ֹͣ
				TIM10_Stop();         //�رն�ʱ��
				break_flag=0;         //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,bodyright_compleate,motor_body_freq,motor_timer_freq);//���ò�������
			}			
		}	
		//����345�ŵ������	
		if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))||(1==motor5_run_flag))  //�ҷ�����
		{		
			if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
			{
				motor5_run_flag=0;
				//345��������
				DIR3=1;DIR4=1;DIR5=1;direct=1;body_right_dir_flag=1;
				if(M345R_Start==0)
				{
					M345R_Start=1;
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
					delay_ms(200);
				}			
				Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //�������	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim)-body_right_runed_arr,timer10_freq);	//��ʱ����			
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownPhone"))  //�ҷ�����
		{			
			if(body_right_runed_arr>0)
			{
				//345��������
			   DIR3=0;DIR4=0;DIR5=0;direct=0;body_right_dir_flag=0;
			   if(M345R_End==1)
			   {
					M345R_End=0; 
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");	
					//delay_ms(200);
			   }			
													
			   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //�������		
			   TIM10_Init(body_right_runed_arr,timer10_freq);		  //�򿪶�ʱ��
			}	
		}				
		  memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		  USART2_RX_LEN=0;
		  __HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ	 	

		 if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
		 {
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ
//					if(GDCheck(GD34S)&&(0==direct))
//					{
//							break_flag=1;
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3_Start");
//							}
//							else if(0==GD4_Start)
//							{
//								u2_printf("GD4_Start");
//							}
//							break;
//					}
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))                    //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if((0==GD3_Start)||(0==GD4_Start))                   //���������翪�أ�����ѭ����ֹͣ����
//						{
//							break_flag=1;
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3_Start");
//							}
//							else if(0==GD4_Start)
//							{
//								u2_printf("GD4_Start");
//							}
//							break;					
//						}	
//					}
//					if(((0==GD3_Right_End)||(0==GD4_Right_End))&&(1==direct))           //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if((0==GD3_Right_End)||(0==GD4_Right_End))          //���������翪�أ�����ѭ����ֹͣ����
//						{
//							break_flag=1;
//							if(0==GD3_Right_End)
//							{
//								u2_printf("GD3_Right_End");
//							}
//							if(0==GD4_Right_End)
//							{
//								u2_printf("GD4_Right_End");
//							}
//							break;
//						}
//					}
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))|| (strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownPhone"))) //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else 
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}					
					}
					//������ϡ��������
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{	
//						delay_us(100);
//						if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//						{
//							if(1==Motor3_Alm)
//							{
//								body_right_overload_3=1;
//								u2_printf("BodyRightOverload3");
//							}
//							if(1==Motor4_Alm)
//							{
//								body_right_overload_4=1;
//								u2_printf("BodyRightOverload4");
//							}
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//							}
//							Breakdown_Treatment();	
//							break_flag=1;
//						}						
//						break;
//					}					
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;		
						}
					}				
				}				
			}				
			 
			Motor_3_4_5_STOP();    //���ֹͣ
			TIM10_Stop();          //��ʱ���ر�
			break_flag=0;		   //�����־λ
			//�жϸ�λ	
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))
			{
				arr_now=0;
				body_right_flag=0;
//				W25QXX_Write((u8*)&body_right_flag,34,1);				
			}					
			else
			{							
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				body_right_flag=1;
//				W25QXX_Write((u8*)&body_right_flag,34,1);					
			}
			//ͨ���������ж������ۼ�
			if(direct==1)    //�������У�����+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End))
				{
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1; 					
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					u2_printf("BodyRightLim");
					delay_ms(200);
				}
				else
				{				
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}
			}
			else		//�������У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))
				{
					body_right_runed_arr=0;	
					M345R_Start=0; 					
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;
				}			
			}			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ�
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //���3û��λ
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								body_right_flag=0;
//								u2_printf("GD3Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_3_STOP();   //���ֹͣ
//					TIM10_Stop();     //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //���4û��λ
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD4_Start) 
//							{
//								body_right_flag=0;
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_4_STOP();    //���ֹͣ
//					TIM10_Stop();      //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //���3/4��û��λ
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD3_Start)
//							{
//								body_right_flag=0;
//								u2_printf("GD3Start");
//								break;
//							}
//							if(0==GD4_Start) 
//							{
//								body_right_flag=0;
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_3_4_5_STOP();   //���ֹͣ
//					TIM10_Stop();         //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
			//�෭��λ
			if(0==body_right_flag)      //ֻ�з���λ����ʼ״̬��С�෭�Ÿ�λ
			{			
				//5�Ų෭��λ
				Motor_4_Compensate(0,bodyright_compleate,motor_body_freq,motor_timer_freq);//���ò�������
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;		
				DIR5=1;
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");	
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq); //�������
				body_right_runed_arr=0;
				TIM10_Init(body_angle_to_arr(body_right_angle_lim)*1,timer10_freq);                     //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ

				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{
						//�����λ
//						if(((0==GD5_Start)&&(0==body_right_flag)))  //������翪������ѭ�������ͣת 
//						{	
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;					
//							}
//						}
						  //�ж���û���յ���λ��ָ��		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}										
						//������ϡ��������
//						if(1==Motor5_Alm)       
//						{	
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_right_overload_5=1;
//								u2_printf("BodyRightOverload5");
//								Breakdown_Treatment();
//								break_flag=1;
//								break;						
//							}

//						}										
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					n=9-n;	
					m=right_motor5_picture_m;
					if(	m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}
						}
					}
				}	     
				Motor_5_STOP();      //���ֹͣ
				TIM10_Stop();        //��ʱ���ر�
				break_flag=0;   
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");				
				delay_ms(200);			
				u2_printf("body_right_flag==0");
				delay_ms(200);
				u2_printf("BodyRightRes");
				//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_right_flag))
//				{
//					DIR5=1;
//					Motor_5_START(motor_body_freq,motor_timer_freq);   //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break;	
//							}								
//						}
//					}			
//					Motor_5_STOP();    //���ֹͣ
//					TIM10_Stop();      //�رն�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}			
			}	
			W25QXX_Write((u8*)&body_right_flag,34,1);
		}
	}
	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Desk(void)  
 ��������    ��С����
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Desk(void)
{
	u8 direct,key;    //��ʾ������з���1��С����ǰ����0��С���Ӻ���
	u16 arr_now;      //������������ֵ
	u8 len;           //��ʾ���յ��ַ����ĳ���
	u16 arr_feed;     //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;  //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�жϳ����Ƿ��break���� 
	static u8 k=0;            //���͵�k��ͼƬ
	static u8 kj=0;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;                 //��ǰһ������������
	static u8 desk_limit_flag;    //�ж�С�����Ƿ����е�����λ�ã����Ƿ��ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���С�����ƶ�
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))
		{
			DIR7=0;
			direct=1;
			if(desk_flag==0)
			{
				desk_flag=1;
				delay_ms(200);
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_1");
				delay_ms(200);
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))
		{
			DIR7=1;
			direct=0;
			if(1==desk_limit_flag)
			{
				desk_limit_flag=0;
				delay_ms(200);
				u2_printf("Cartoon_Desk_20");
				delay_ms(200);
			}
		}
		if(direct==1)   //�����С������ǰ
		{		
			if(desk_distance_to_arr(desk_distance_lim)>desk_runed_arr)  //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)-desk_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}				
		}
		else       //�����С���Ӻ���
		{
			if(desk_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
		 if(((desk_runed_arr!=desk_distance_to_arr(desk_distance_lim))&&(1==direct))||((0!=desk_runed_arr)&&(0==direct)))
		 {	 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��  
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ											   						
					if((0==GD7_End)&&(1==direct))
					{
						delay_us(100);
						if(0==GD7_End)
						{
							u2_printf("GD7End");					
							break_flag=1;	
							break;					
						}								
					}
					if((0==GD7_Start)&&(0==direct))
					{
						delay_us(100);
						if(0==GD7_Start)
						{
							u2_printf("GD7Start");					
							break_flag=1;
							break;						
						}					
					}
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone")))    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}				
					//������ϡ��������
//					if(PCF8574_ReadBit(1)==1)        
//					{	
//						delay_us(100);
//						if(PCF8574_ReadBit(1)==1)
//						{
//							desk_overload=1;
//							u2_printf("DeskOverload");
//							Uart_Breakdown_Treatment();
//							break_flag=1;
//							break;						
//						}								             
//					}								
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(desk_runed_arr+arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				else
				{
					j=abs(desk_runed_arr,arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				k=desk_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  desk_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Desk_2");									
									break;
							case 2:	u2_printf("Cartoon_Desk_3");
									break;					
							case 3:	u2_printf("Cartoon_Desk_4");
									break;					
							case 4:	u2_printf("Cartoon_Desk_5");
									break;	
							case 5:	u2_printf("Cartoon_Desk_6");
									break;	
							case 6:	u2_printf("Cartoon_Desk_7");
									break;
							case 7:	u2_printf("Cartoon_Desk_8");
									break;
							case 8:	u2_printf("Cartoon_Desk_9");
									break;						
							case 9:	u2_printf("Cartoon_Desk_10");
									break;												
							case 10:u2_printf("Cartoon_Desk_11");
									break;	
							case 11:u2_printf("Cartoon_Desk_12");									
									break;
							case 12:u2_printf("Cartoon_Desk_13");
									break;					
							case 13:u2_printf("Cartoon_Desk_14");
									break;					
							case 14:u2_printf("Cartoon_Desk_15");
									break;	
							case 15:u2_printf("Cartoon_Desk_16");
									break;	
							case 16:u2_printf("Cartoon_Desk_17");
									break;
							case 17:u2_printf("Cartoon_Desk_18");
									break;
							case 18:u2_printf("Cartoon_Desk_19");
									break;						
						}
					}					
				}				
			}				    
			Motor_7_STOP();     //���ֹͣ
			TIM10_Stop();       //��ʱ���ر�
			break_flag=0;		//���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
			{
				arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				desk_flag=0;
				delay_ms(200);
				u2_printf("desk_flag==0");
			}
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
				desk_flag=1;
			}	
			//ͨ���������ж������ۼ�
			if(direct==1)        //�����С����ǰ��������+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_End))
				{ 					
					desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
					desk_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Desk_20");
					delay_ms(200);
					u2_printf("DeskLim");
					delay_ms(200);
				}
				else
				{  desk_runed_arr=desk_runed_arr+arr_now;	}				
			}
			else                //�����С���Ӻ��ˣ�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_Start))
				{						
					desk_runed_arr=0; 
					delay_ms(200);
					u2_printf("Cartoon_Desk_1");
					delay_ms(200);
					u2_printf("DeskRes");
					delay_ms(200);
				}
				else
				{	 
					desk_runed_arr=desk_runed_arr-arr_now;
				}						
			}				
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	   			
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶�,��ɾ������ж���䣩
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD7_Start==1)&&(direct==0)&&(desk_flag==1))
//			{   
//				u2_printf("������������");
//				DIR7=1;
//				Motor_7_START(motor_desk_freq,motor_timer_freq);
//			    TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				
//				u2_printf("\r\n���븽���������У�ֱ��������翪��\r\n");
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{							
//					if(0==GD7_Start)  //����ʱ������翪�أ�����ѭ�� 
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");
//							break;
//						}							
//					}
//				}			
//				Motor_7_STOP();   //���ֹͣ
//				TIM10_Stop();     //�رն�ʱ��
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//			}			
		}
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("DeskInterfere");		
		LED0=1;
		LED1=1;	

	}		
}

void Uart_Desk1(void)
{
	u8 direct,key;    //��ʾ������з���1��С����ǰ����0��С���Ӻ���
	u16 arr_now;      //������������ֵ
	u8 len;           //��ʾ���յ��ַ����ĳ���
	u16 arr_feed;     //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;  //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	static u8 desk_front_flag;
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�жϳ����Ƿ��break���� 
	static u8 k=0;            //���͵�k��ͼƬ
	static u8 kj=0;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;                 //��ǰһ������������
	static u8 desk_limit_flag;    //�ж�С�����Ƿ����е�����λ�ã����Ƿ��ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���С�����ƶ�
	if((lock_flag==1)&&(body_left_flag==0)&&(body_right_flag==0))
	{	
		//С���ӵ��������һ��ʱ��
		if(desk_flag==0)
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				DIR7=0;
				desk_flag=1;
				desk_front_flag=1;
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_Front");
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init((u16)(desk_distance_to_arr(desk_distance_lim)/2.6),timer10_freq);
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
				{
					
				}
				Motor_7_STOP();     //���ֹͣ
				TIM10_Stop();       //��ʱ���ر�
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
				u2_printf("���7��һ�׶��������");
			}
		}


		
		if((strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))||(desk_front_flag==1))
		{
			desk_front_flag=0;
			DIR7=0;
			direct=1;
			if(desk_flag==0)
			{
				desk_flag=1;
				delay_ms(200);
				u2_printf("desk_flag==1");
				delay_ms(200);
				u2_printf("DeskStart");
				delay_ms(200);
				u2_printf("Cartoon_Desk_1");
				delay_ms(200);
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))
		{
			DIR7=1;
			direct=0;
			if(1==desk_limit_flag)
			{
				desk_limit_flag=0;
				delay_ms(200);
				u2_printf("Cartoon_Desk_20");
				delay_ms(200);
			}
		}
		if(direct==1)   //�����С������ǰ
		{		
			if(desk_distance_to_arr(desk_distance_lim)>desk_runed_arr)  //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)-desk_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}				
		}
		else       //�����С���Ӻ���
		{
			if(desk_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_runed_arr,timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz			
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;	
		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
		 if(((desk_runed_arr!=desk_distance_to_arr(desk_distance_lim))&&(1==direct))||((0!=desk_runed_arr)&&(0==direct)))
		 {	 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��  
			{			
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ											   						
//					if((0==GD7_End)&&(1==direct))
//					{
//						delay_us(100);
//						if(0==GD7_End)
//						{
//							u2_printf("GD7End");					
//							break_flag=1;	
//							break;					
//						}								
//					}
//					if((0==GD7_Start)&&(0==direct))
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");					
//							break_flag=1;
//							break;						
//						}					
//					}
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))||(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone")))    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���							
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}				
					//������ϡ��������
//					if(PCF8574_ReadBit(1)==1)        
//					{	
//						delay_us(100);
//						if(PCF8574_ReadBit(1)==1)
//						{
//							desk_overload=1;
//							u2_printf("DeskOverload");
//							Uart_Breakdown_Treatment();
//							break_flag=1;
//							break;						
//						}								             
//					}								
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(desk_runed_arr+arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				else
				{
					j=abs(desk_runed_arr,arr_send)/(desk_distance_to_arr(desk_distance_max)/19);
				}
				k=desk_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);				
					if(kj<2)
					{
						k=j;  desk_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Desk_2");									
									break;
							case 2:	u2_printf("Cartoon_Desk_3");
									break;					
							case 3:	u2_printf("Cartoon_Desk_4");
									break;					
							case 4:	u2_printf("Cartoon_Desk_5");
									break;	
							case 5:	u2_printf("Cartoon_Desk_6");
									break;	
							case 6:	u2_printf("Cartoon_Desk_7");
									break;
							case 7:	u2_printf("Cartoon_Desk_8");
									break;
							case 8:	u2_printf("Cartoon_Desk_9");
									break;						
							case 9:	u2_printf("Cartoon_Desk_10");
									break;												
							case 10:u2_printf("Cartoon_Desk_11");
									break;	
							case 11:u2_printf("Cartoon_Desk_12");									
									break;
							case 12:u2_printf("Cartoon_Desk_13");
									break;					
							case 13:u2_printf("Cartoon_Desk_14");
									break;					
							case 14:u2_printf("Cartoon_Desk_15");
									break;	
							case 15:u2_printf("Cartoon_Desk_16");
									break;	
							case 16:u2_printf("Cartoon_Desk_17");
									break;
							case 17:u2_printf("Cartoon_Desk_18");
									break;
							case 18:u2_printf("Cartoon_Desk_19");
									break;						
						}
					}					
				}				
			}				    
			Motor_7_STOP();     //���ֹͣ
			TIM10_Stop();       //��ʱ���ر�
			break_flag=0;		//���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
			{
				arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				desk_flag=0;
				delay_ms(100);
				u2_printf("desk_flag==0");
			}
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
				desk_flag=1;
			}	
			//ͨ���������ж������ۼ�
			if(direct==1)        //�����С����ǰ��������+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_End))
				{ 					
					desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
					desk_limit_flag=1;	
					delay_ms(200);
					u2_printf("Cartoon_Desk_20");
					delay_ms(200);
					u2_printf("DeskLim");
					delay_ms(200);
				}
				else
				{  desk_runed_arr=desk_runed_arr+arr_now;	}				
			}
			else                //�����С���Ӻ��ˣ�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD7_Start))
				{						
					desk_runed_arr=0; 
					//delay_ms(200);
					u2_printf("Cartoon_Desk_1");
					delay_ms(200);
					u2_printf("DeskRes");
					//delay_ms(200);
				}
				else
				{	 
					desk_runed_arr=desk_runed_arr-arr_now;
				}						
			}				
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	   			
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶�,��ɾ������ж���䣩
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD7_Start==1)&&(direct==0)&&(desk_flag==1))
//			{   
//				u2_printf("������������");
//				DIR7=1;
//				Motor_7_START(motor_desk_freq,motor_timer_freq);
//			    TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				
//				u2_printf("\r\n���븽���������У�ֱ��������翪��\r\n");
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{							
//					if(0==GD7_Start)  //����ʱ������翪�أ�����ѭ�� 
//					{
//						delay_us(100);
//						if(0==GD7_Start)
//						{
//							u2_printf("GD7Start");
//							break;
//						}							
//					}
//				}			
//				Motor_7_STOP();   //���ֹͣ
//				TIM10_Stop();     //�رն�ʱ��
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//			}			
		}
		if(desk_flag==0) 
		{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;	
				DIR7=1;
			  desk_runed_arr=0;
				Motor_7_START(motor_desk_freq,motor_timer_freq);
				TIM10_Init(desk_distance_to_arr(desk_distance_lim)/2.6,timer10_freq);
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);  //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) 
				{
					
				}
				Motor_7_STOP();     //���ֹͣ
				TIM10_Stop();       //��ʱ���ر�
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);
				u2_printf("���7��һ�׶η������");
		}
		
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("DeskInterfere");		
		LED0=1;
		LED1=1;	

	}		
}




/***********************************************************************
 ������      ��Uart_Back_Nursing_Left(void)  
 ��������    ���󱳲�����
 ����        ����
 ���        ���� 

************************************************************************/
void Uart_Back_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;    //�жϳ����Ƿ��break����
	static u8 k=0;      //���͵�k�Ŷ���ָ��
	u8 i=0;
	u8 j=0;	
	u16 arr_send;	    //��ǰһ������������
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//�������ܣ�ֻ������ִ��������������������λ�󣬲��ܽ����󱳲�����
	if((lock_flag==1)&&(body_left_flag==1)&&(waist_nursing_left_flag==0))
	{
		back_nursing_left_flag=!back_nursing_left_flag;	         //�󱳲�����λ״̬��־λȡ��
		back_nursing_left_dir_flag=!back_nursing_left_dir_flag;	 //�󱳲��������־λ		
		if(back_nursing_left_dir_flag==1)
		{ 
			DIR3=1; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_left_flag==1");
			delay_ms(200);
			u2_printf("BackNursingLeftStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=0; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_3_START(motor_body_freq,motor_timer_freq);	       //�������
		TIM10_Init(body_left_runed_arr,timer10_freq);              //�򿪶�ʱ��body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//�����λ
//				if((0==GD3_Left_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD3_Left_End)
//					{
//						break_flag=1;
//						u2_printf("GD3LeftEnd");
//						break;				
//					}
//	
//				}
//				if((0==GD3_Start)&&(0==direct))     //����
//				{	
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						break_flag=1;	
//						u2_printf("GD3_Start");
//					    break;					
//					}					
//				}
				  //�ж���û���յ���λ��ָ��		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;															
				}				
				//������ϡ��������
//				if(1==Motor3_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor3_Alm) 
//					{
//						back_nursing_left_overload=1;
//						u2_printf("BackNursingLeftOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;				
//					}								             
//				}							
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_left_picture_k;
			if(k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;   back_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Back_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Left_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Left_19");
								break;						
					}
				}
			}
		}				  
		Motor_3_STOP();     //���ֹͣ
		TIM10_Stop();       //�رն�ʱ��    
		break_flag=0;       //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		if(back_nursing_left_dir_flag==1)
		{			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_20");
			delay_ms(200);
			u2_printf("BackNursingLeftLim");
		}
		else
		{			
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Left_1");
			delay_ms(200);
			u2_printf("back_nursing_left_flag==0");
			delay_ms(200);
			u2_printf("BackNursingLeftRes");
		}	
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BackNursingLeftInterfere");		
		LED0=1;	
		LED1=1;

	}	
}

/***********************************************************************
 ������      ��Uart_Back_Nursing_Right(void)
 ��������    ���ұ�������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Back_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;     //�жϳ����break����
	static u8 k=0;       //���͵�k�Ŷ���ָ��
	u8 i=0;
	u8 j=0;	
	u16 arr_send;	    //��ǰһ������������
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;		
	
	//�������ܣ�ֻ������ִ���ҷ�����������������λ�󣬲��ܽ����ұ�������
	if((lock_flag==1)&&(1==body_right_flag)&&(waist_nursing_right_flag==0))
	{	
		back_nursing_right_flag=!back_nursing_right_flag;         //�ұ�������λ״̬��־λ
		back_nursing_right_dir_flag=!back_nursing_right_dir_flag; //�ұ����������־λ 	
		if(back_nursing_right_dir_flag==1)
		{ 
			DIR3=0; direct=1;
			delay_ms(200);
			u2_printf("back_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("BackNursingRightStart");
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_1");
			delay_ms(200);
		}
		else
		{ 
			DIR3=1; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Back_Nursing_Right_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_3_START(motor_body_freq*1.4,motor_timer_freq);              //�������
		TIM10_Init(body_right_runed_arr,timer10_freq);                //�򿪶�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ�� 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//�����λ����
//				if((0==GD3_Right_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD3_Right_End)
//					{
//						break_flag=1;
//						u2_printf("GD3RightEnd");
//						break;				
//					}	
//				}
//				if((0==GD3_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						break_flag=1;
//						u2_printf("GD3Start");
//						break;				
//					}						
//				}
				  //�ж���û���յ���λ��ָ��		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;												
				}
				//������ϡ��������
//				if(1==Motor3_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor3_Alm) 
//					{
//						back_nursing_right_overload=1;
//						u2_printf("BackNursingRightOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;				
//					}		             
//				}										
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=back_nursing_right_picture_k;
			if(	k!=j)
			{	
				kj=abs(k,j);				
				if(kj<2)
				{
					k=j;   back_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Back_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Back_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Back_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Back_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Back_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Back_Nursing_Right_7");
								break;						
						case 7:	u2_printf("Cartoon_Back_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Back_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Back_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Back_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Back_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Back_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Back_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Back_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Back_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Back_Nursing_Right_17");
								break;	
						case 17:u2_printf("Cartoon_Back_Nursing_Right_18");
								break;
						case 18:u2_printf("Cartoon_Back_Nursing_Right_19");
								break;	
					}
				}
			}			
		}			
		Motor_3_STOP();      //���ֹͣ
		TIM10_Stop();        //�رն�ʱ��
		break_flag=0;        //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		
		if(back_nursing_right_dir_flag==1)
		{ 		
			 delay_ms(200);
			 u2_printf("Cartoon_Back_Nursing_Right_20");
			 delay_ms(200);
			 u2_printf("BackNursingRightLim");
		}
		else
		{ 			
			 delay_ms(200);
			 u2_printf("Cartoon_Back_Nursing_Right_1");			 
			 delay_ms(200);
			 u2_printf("back_nursing_right_flag==0");
			 delay_ms(200);
			 u2_printf("BackNursingRightRes");
		}				
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BackNursingRightInterfere");
		LED0=1;	
		LED1=1;		
	}
}

/***********************************************************************
 ������      ��Uart_Waist_Nursing_Left(void)  
 ��������    ������������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Waist_Nursing_Left(void)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;    //�жϳ����break����
	static u8 k=0;      //���͵�k�Ŷ���ָ��
	u8 i=0;
	u8 j=0;	 
	u16 arr_send;	   //��ǰһ������������
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;		
	
	//�������ܣ�ֻ������ִ�����������󱳲�����λ�󣬲��ܽ�������������
	if((lock_flag==1)&&(1==body_left_flag)&&(back_nursing_left_flag==0))
	{
		waist_nursing_left_flag=!waist_nursing_left_flag;         //����������λ״̬��־λ
		waist_nursing_left_dir_flag=!waist_nursing_left_dir_flag; //�������������־λ		
		if(waist_nursing_left_dir_flag==1)
		{ 
			DIR4=1; direct=1;
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==1");
			delay_ms(200);
			u2_printf("WaistNursingLeftStart");
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1");	
			delay_ms(200);
		}
		else
		{ 
			DIR4=0; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_4_START((u16)(motor_body_freq*1.2),motor_timer_freq);             //�������
		TIM10_Init(body_left_runed_arr,timer10_freq);                //�򿪶�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ�� 
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//�����λ
//				if((0==GD4_Left_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD4_Left_End)
//					{
//						break_flag=1;
//						u2_printf("GD4LeftEnd");
//						break;					
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						break_flag=1;
//						u2_printf("GD4Start");
//						break;				
//					}						
//				}
				  //�ж���û���յ���λ��ָ��		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;												
				}				
				
				//������ϡ��������
//				if(1==Motor4_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor4_Alm)
//					{
//						waist_nursing_left_overload=1;
//						u2_printf("WaistNursingLeftOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;					
//					}	             
//				}						
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
            //���Ͷ���ָ��			
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_left_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_left_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);
				if(kj<2)
				{
					k=j;   waist_nursing_left_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Left_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Left_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Left_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Left_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Left_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Left_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Left_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Left_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Left_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Left_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Left_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Left_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Left_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Left_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Left_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Left_17");
								break;	
						case 17:u2_printf("Cartoon_Waist_Nursing_Left_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Left_19");
								break;		
					}
				}
			}			
		}				     
		Motor_4_STOP();     //���ֹͣ
		TIM10_Stop();       //��ʱ���ر�
		break_flag=0;       //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		
		if(waist_nursing_left_dir_flag==1)
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_20");  
			delay_ms(200);
			u2_printf("WaistNursingLeftLim");
		}
		else
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Left_1"); 			
			delay_ms(200);
			u2_printf("waist_nursing_left_flag==0");
			delay_ms(200);
			u2_printf("WaistNursingLeftRes");
		}		
		
//		//ʹ����������ˮƽ״̬
//		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD4_Start)&&(direct==1))
//		{
//			DIR4=1;
//			Motor_4_START(motor_body_freq,motor_timer_freq);              //�������
//			TIM10_Init(add_arr,timer10_freq);                             //�򿪶�ʱ��35000
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ	 	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
//			{
//				if(0==GD4_Start)      //���������ػ�������翪�أ�������ѭ�������ֹͣת�� 
//				{ 
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("GD4Start");			
//						break; 
//					}						
//				}				
//			}				                                 			
//			Motor_4_STOP();      //���ֹͣ
//			TIM10_Stop();        //��ʱ���ر�
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//		}
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("WaistNursingLeftInterfere");
		LED0=1;	
		LED1=1;
	}
}

/***********************************************************************
 ������      ��Uart_Waist_Nursing_Right(void)  
 ��������    ������������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Waist_Nursing_Right(void)
{
	u8 direct,key,len;
	u16 arr_feed;      //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;   //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;  
	u8 break_flag=0;      //�жϳ����break����
	static u8 k=0;        //���͵�k�Ŷ���ָ��
	u8 i=0;
	u8 j=0;	
	u16 arr_send;	      //��ǰһ������������
	static u8 kj;
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//�������ܣ�ֻ������ִ���ҷ��������ұ�������λ�󣬲��ܽ�������������
	if((lock_flag==1)&&(body_right_flag==1)&&(back_nursing_right_flag==0))
	{
		waist_nursing_right_flag=!waist_nursing_right_flag;	        //����������λ״̬��־λ
		waist_nursing_right_dir_flag=!waist_nursing_right_dir_flag;	//�������������־λ	
		if(waist_nursing_right_dir_flag==1)
		{ 
			DIR4=0; direct=1;
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");			
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==1");
			delay_ms(200);
			u2_printf("WaistNursingRightStart");	
			delay_ms(200);
		}
		else
		{ 
			DIR4=1; direct=0;
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_20");
			delay_ms(200);
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_4_START((u16)(motor_body_freq),motor_timer_freq);             //�������
		TIM10_Init(body_right_runed_arr,timer10_freq);               //�򿪶�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		{
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{			
				//�����λ
//				if((0==GD4_Right_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD4_Right_End)
//					{
//						break_flag=1;
//						u2_printf("GD4RightEnd");
//						break;				
//					}	
//				}
//				if((0==GD4_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{	
//						break_flag=1;
//						u2_printf("GD4Start");
//						break;									
//					}
//				}
				  //�ж���û���յ���λ��ָ��		
				if(USART2_RX_LEN&0x8000)
				{
					u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;												
				}
				//������ϡ��������
//				if(1==Motor4_Alm)        
//				{	
//					delay_us(100);
//					if(1==Motor4_Alm) 
//					{
//						waist_nursing_right_overload=1;
//						u2_printf("WaistNursingRightOverload");
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;				
//					}		             
//				}							
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}	
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(body_right_runed_arr/19);
			if(0==direct)
			{
				j=19-j;
			}
			k=waist_nursing_right_picture_k;
			if(	k!=j)
			{
				kj=abs(k,j);							
				if(kj<2)	
				{
					k=j;   waist_nursing_right_picture_k=k;
					switch (k)
					{						
						case 1:	u2_printf("Cartoon_Waist_Nursing_Right_2");									
								break;
						case 2:	u2_printf("Cartoon_Waist_Nursing_Right_3");
								break;					
						case 3:	u2_printf("Cartoon_Waist_Nursing_Right_4");
								break;					
						case 4:	u2_printf("Cartoon_Waist_Nursing_Right_5");
								break;	
						case 5:	u2_printf("Cartoon_Waist_Nursing_Right_6");
								break;	
						case 6:	u2_printf("Cartoon_Waist_Nursing_Right_7");
								break;
						case 7:	u2_printf("Cartoon_Waist_Nursing_Right_8");									
								break;
						case 8:	u2_printf("Cartoon_Waist_Nursing_Right_9");
								break;					
						case 9:	u2_printf("Cartoon_Waist_Nursing_Right_10");
								break;					
						case 10:u2_printf("Cartoon_Waist_Nursing_Right_11");
								break;	
						case 11:u2_printf("Cartoon_Waist_Nursing_Right_12");
								break;	
						case 12:u2_printf("Cartoon_Waist_Nursing_Right_13");
								break;
						case 13:u2_printf("Cartoon_Waist_Nursing_Right_14");
								break;	
						case 14:u2_printf("Cartoon_Waist_Nursing_Right_15");
								break;	
						case 15:u2_printf("Cartoon_Waist_Nursing_Right_16");
								break;
						case 16:u2_printf("Cartoon_Waist_Nursing_Right_17");
								break;
						case 17:u2_printf("Cartoon_Waist_Nursing_Right_18");
								break;	
						case 18:u2_printf("Cartoon_Waist_Nursing_Right_19");
								break;							
					}
				}
			}
		}				
      
		Motor_4_STOP();    //���ֹͣ
		TIM10_Stop();      //�رն�ʱ�� 
		break_flag=0;      //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		
		if(waist_nursing_right_dir_flag==1)
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_20");  
			delay_ms(200);
			u2_printf("WaistNursingRightLim");
		}
		else
		{ 
			delay_ms(200);
			u2_printf("Cartoon_Waist_Nursing_Right_1");		
			delay_ms(200);
			u2_printf("waist_nursing_right_flag==0");
			delay_ms(200);
			u2_printf("WaistNursingRightRes");
		}		
	}
	else
	{
		LED0=0;        //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("WaistNursingRightInterfere");		
		LED0=1;	
		LED1=1;
	}
}


/***********************************************************************
 ������      ��MOTOR111(void)   
 ��������    ��
 ����        ����
 ���        ����                           
************************************************************************/
void MOTOR111(u8 dir)
{
	u8 direct,len;
	u16 arr_now;              //��ǰһ������������
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;          //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	static u8 k;             //����k�Ŷ���
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //��ǰһ����������ֵ
	static u8 back_limit_flag; //֧�����е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧��	
		if(dir==1)
		{				
			Motor_1_START(1);                                                          //֧������
			u2_printf("Motor_1_START(1)");
		}
		else
		{
			Motor_1_START(0);                                                          //֧������
			u2_printf("Motor_1_START(0)");
		}
		TIM10_Init(30000,timer10_freq); //�򿪶�ʱ��

		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;									
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
							
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{		}		
		Motor_1_STOP();    //���ֹͣ
		TIM10_Stop();      //�رն�ʱ��
		break_flag=0;      //��־λ����
		//�жϸ�λ
		u2_printf("Motor_1_STOP()");
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
}

/***********************************************************************
 ������      ��MOTOR222(void)   
 ��������    ��
 ����        ����
 ���        ����                           
************************************************************************/
void MOTOR222(u8 dir)
{
	u8 direct,len;
	u16 arr_now;              //��ǰһ������������
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;          //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	static u8 k;             //����k�Ŷ���
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //��ǰһ����������ֵ
	static u8 back_limit_flag; //֧�����е�����λ����1�����ͼ���λ��ͼƬ
	
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧��	
		if(dir==1)
		{				
			Push_Rod_Start(1); 
            u2_printf("Push_Rod_Start(1)");           //֧������
		}
		else
		{
			Push_Rod_Start(0);                                                          //֧������
			u2_printf("Push_Rod_Start(0)");
		}
		
		TIM10_Init(30000,timer10_freq); //�򿪶�ʱ��		
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;									
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
							
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{		}		
		Push_Rod_Stop();   //���ֹͣ
		TIM10_Stop();      //�رն�ʱ��
		break_flag=0;      //��־λ����
		//�жϸ�λ
		u2_printf("Push_Rod_Stop()");
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
}


/***********************************************************************
 ������      ��MOTOR333(void)  
 ��������    ��
 ����        ����
 ���        ���� 

************************************************************************/
void MOTOR333(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;	 
	u8 break_flag=0;    //�жϳ����Ƿ��break����
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR3=1\r\n");
			DIR3=0; direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR3=0\r\n");
			DIR3=1; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_3_START(motor_body_freq,motor_timer_freq);	       //�������
		TIM10_Init(8000,timer10_freq);              //�򿪶�ʱ��body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{		
				//�����λ
//				if((0==GD3_Left_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD3_Left_End)
//					{
//						u2_printf("\r\nGD3_Left_End\r\n");
//						break;
//					}
//				}
//				if((0==GD3_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD3_Start)
//					{
//						u2_printf("\r\nGD3_Start\r\n");
//						break;
//					}
//				}
//				if((0==GD3_Right_End)&&(1==direct))     //����
//				{
//					u2_printf("\r\nGD3_Right_End\r\n");
//					break;						
//				}
				//������ϡ��������
//				if(1==Motor3_Alm)        
//				{
//					delay_us(100);
//					if(1==Motor3_Alm) 
//					{
//						u2_printf("\r\nMotor3_Alm\r\n");
//						break;		       
//					}						
//				}															
		}				  
		Motor_3_STOP();     //���ֹͣ
		TIM10_Stop();       //�رն�ʱ��    
		break_flag=0;       //���break��־λ
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ		
		u2_printf("\r\nMotor_3_STOP\r\n");
}

/***********************************************************************
 ������      ��MOTOR444(void)  
 ��������    ��
 ����        ����
 ���        ���� 

************************************************************************/
void MOTOR444(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	u8 break_flag=0;    //�жϳ����Ƿ��break����
					
		if(dir==1)
		{ 
			u2_printf("\r\n0IR4=1\r\n");
			DIR4=0;   direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR4=0\r\n");
			DIR4=1; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_4_START(motor_body_freq,motor_timer_freq);	       //�������
		TIM10_Init(8000,timer10_freq);              //�򿪶�ʱ��body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{		
				//�����λ
//				if((0==GD4_Left_End)&&(1==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD4_Left_End)
//					{
//						u2_printf("\r\nGD4_Left_End\r\n");
//						break;
//					}
//				}
//				if((0==GD4_Start)&&(0==direct))                     //����
//				{
//					delay_us(100);
//					if(0==GD4_Start)
//					{
//						u2_printf("\r\nGD4_Start\r\n");
//						break;
//					}
//											
//				}
//				if((0==GD4_Right_End)&&(1==direct))     //����
//				{
//					u2_printf("\r\nGD4_Right_End\r\n");
//					break;						
//				}
				//������ϡ��������
//				if(1==Motor4_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor4_Alm) 
//					{
//						u2_printf("\r\nMotor4_Alm\r\n");
//						break;
//					}
//				}													
		}				  
		Motor_4_STOP();     //���ֹͣ
		TIM10_Stop();       //�رն�ʱ��    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		u2_printf("\r\nMotor_4_STOP\r\n");
}


/************************************************************************
 ������      ��MOTOR555(void)  
 ��������    ��
 ����        ����
 ���        ���� 

************************************************************************/
void MOTOR555(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	u8 break_flag=0;    //�жϳ����Ƿ��break����
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR5=1\r\n");
			DIR5=1; direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR5=0\r\n");
			DIR5=0; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_5_START(motor_body_freq,motor_timer_freq);	       //�������
		TIM10_Init(8000,timer10_freq);              //�򿪶�ʱ��body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{		
				//�����λ
//				if((0==GD5_Left_End))  //����
//				{
//					u2_printf("\r\n0==GD5_Left_End\r\n");
//					break;	
//				}
						
//				if((0==GD5_Left_End)&&(1==direct))  //����
//				{
//					delay_us(100);
//					if(0==GD5_Left_End)
//					{
//						u2_printf("\r\n0==GD5_Left_End\r\n");
//						break;	
//					}
//				}
//				if((0==GD5_Start)&&(0==direct))     //����
//				{
//					delay_us(100);
//					if(0==GD5_Start)
//					{
//						u2_printf("\r\n0==GD5_Start\r\n");
//						break;
//					}
//				}
//				if((0==GD5_Right_End)&&(1==direct))     //����
//				{
//					u2_printf("\r\nGD5_Right_End\r\n");
//					break;						
//				}
				//������ϡ��������
//				if(1==Motor5_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor5_Alm)
//					{
//						u2_printf("\r\n1==Motor5_Alm\r\n");
//						break;
//					}
//				}														
		}				  
		Motor_5_STOP();     //���ֹͣ
		TIM10_Stop();       //�رն�ʱ��    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		u2_printf("\r\nMotor_5_STOP\r\n");
}

/************************************************************************
 ������      ��MOTOR666(void)  
 ��������    ��
 ����        ����
 ���        ���� 

************************************************************************/
void MOTOR666(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	u8 break_flag=0;    //�жϳ����Ƿ��break����
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR6=0\r\n");
			DIR6=0;   direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR6=1\r\n");
			DIR6=1;   direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_6_START(motor_body_freq,motor_timer_freq);	       //�������
		TIM10_Init(30000,timer10_freq);              //�򿪶�ʱ��body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{		
				//�����λ
//				if((0==GD6_End)&&(direct==1))  //����
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						u2_printf("\r\n0==GD6_End\r\n");
//						break;
//					}					
//				}
//				if((0==GD6_Start)&&(direct==0))     //����
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						u2_printf("\r\n0==GD6_Start\r\n");
//						break;
//					}
//				}
//			
//				//������ϡ��������
//				if(1==Motor6_Alm)        
//				{						
//					delay_us(100);
//					if(1==Motor6_Alm)
//					{
//						u2_printf("\r\nMotor6_Alm\r\n");
//						break;
//					}
//				}															
		}				  
		Motor_6_STOP();     //���ֹͣ
		TIM10_Stop();       //�رն�ʱ��    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		u2_printf("\r\nMotor_6_STOP\r\n");
}

/************************************************************************
 ������      ��MOTOR777(void)  
 ��������    ���󱳲�����
 ����        ����
 ���        ���� 

************************************************************************/
void MOTOR777(u8 dir)
{
	u8 direct,key,len;
	u16 arr_feed;       //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;    //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	u8 break_flag=0;    //�жϳ����Ƿ��break����
					
		if(dir==1)
		{ 
			u2_printf("\r\nDIR7=0\r\n");
			DIR7=0; direct=1;
		}
		else
		{ 
			u2_printf("\r\nDIR7=1\r\n");
			DIR7=1; direct=0;
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		Motor_7_START(motor_desk_freq,motor_timer_freq);	       //�������
		TIM10_Init(64000,timer10_freq);              //�򿪶�ʱ��body_left_runed_arr
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ	 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��   
		{		
				//�����λ
//				if((0==GD7_End)&&(1==direct))  //����
//				{
//					u2_printf("\r\n0==GD7_End\r\n");
//					break;	
//				}
//				if((0==GD7_Start)&&(0==direct))     //����
//				{
//					u2_printf("\r\n0==GD7_Start\r\n");
//					break;						
//				}
			
				//������ϡ��������
//				if(1==Motor7_Alm)        
//				{						
//					u2_printf("\r\nMotor7_Alm\r\n");
//					break;		             
//				}															
		}				  
		Motor_7_STOP();     //���ֹͣ
		TIM10_Stop();       //�رն�ʱ��    
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
		u2_printf("\r\nMotor_7_STOP\r\n");
}


/***********************************************************************
 ������      ��Liandong_Test(void)  
 ��������    ���������Ժ���
 ����        ����
 ���        ����                           
************************************************************************/
void WriteInUART2(char *p)
{
	u8 len=strlen(p);
	u8 i;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	for(i=0;i<len;i++)
	{
	  USART2_RX_BUF[i]=p[i];
	}	
}

void LDUART2(void)
{
	//��
	WriteInUART2("BodyLeftUpPhone");		
	Uart_Body_Left();           //����	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //�󱳲�����	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //�󱳲�����λ	
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //����������
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //����������λ	
	delay_ms(1000);
	WriteInUART2("BodyLeftDownPhone");
	Uart_Body_Left();           //����λ
	delay_ms(1000);

	//�ҷ�
//	WriteInUART2("BodyRightUpPhone");
//	Uart_Body_Right();          //�ҷ���
//	delay_ms(1000);
//	Uart_Back_Nursing_Right();  //�ұ�������
//	delay_ms(1000);
//	Uart_Back_Nursing_Right();  //�ұ�������λ
//	delay_ms(1000);
//	Uart_Waist_Nursing_Right(); //����������
//	delay_ms(1000);
//	Uart_Waist_Nursing_Right(); //����������λ
//	delay_ms(1000);
//	WriteInUART2("BodyRightDownPhone");
//	Uart_Body_Right();          //�ҷ���λ
//	delay_ms(1000);
	
	//������
	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //������
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //�����ȸ�λ
	delay_ms(1000);
	
	//�γ�����-֧���������ȡ���������С����
	
	//֧��
//	WriteInUART2("BackUpPhone");
//	Uart_Back();                //֧��
//	delay_ms(1000);
//	
//	//������
//	WriteInUART2("LegDownDownPhone");
//	Uart_Leg_Down();            //������	
//	delay_ms(1000);
//	
//	//������
//	Uart_Washlet(0);            //��������
//	delay_ms(1000);
//	Uart_Washlet(1);            //�������ر�
//	delay_ms(1000);
//	
//	//��ϴ���
//	washlet_flag=1;	//��
//	RELAY6=1; 
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//��
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//��
//	RELAY6=0;
//	delay_ms(1000);
//	
//	//����
//	washlet_flag=1;
//	RELAY6=1;                  //�̵����õ�
//	Uart_Washlet_Tig(0);
//	RELAY6=0;   
//	delay_ms(1000);

//�Զ�����
	WriteInUART2("WashletAutoPhone");
	Uart_Washlet_Auto();
  delay_ms(1000);
	
	//С����
	WriteInUART2("DeskUpPhone");
	Uart_Desk();               //С���ӿ���
	delay_ms(1000);
	WriteInUART2("DeskDownPhone");
	Uart_Desk();               //С���Ӻ���
	delay_ms(1000);	
	
	
//	WriteInUART2("LegDownUpPhone");
//	Uart_Leg_Down();            //�����ȸ�λ
//	delay_ms(1000);
//	
//	WriteInUART2("BackDownPhone");
//	Uart_Back();                //֧����λ

//	TestAll(1);
//	delay_ms(1000);
//	TestAll(0);

}


void LDUART2V2(void)
{
	u32 hangid;
	//��
	u2_printf("\r\n\r\n*****��******\r\n\r\n");
	WriteInUART2("BodyLeftUpPhone");		
	Uart_Body_Left();           //����	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //�󱳲�����	
	delay_ms(1000);
	Uart_Back_Nursing_Left();   //�󱳲�����λ	
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //����������
	delay_ms(1000);
	Uart_Waist_Nursing_Left();  //����������λ	
	delay_ms(1000);
	WriteInUART2("BodyLeftDownPhone");
	Uart_Body_Left();           //����λ
	delay_ms(1000);

	//�ҷ�
		u2_printf("\r\n\r\n*****�ҷ�******\r\n\r\n");
	WriteInUART2("BodyRightUpPhone");
	Uart_Body_Right();          //�ҷ���
	delay_ms(1000);
	Uart_Back_Nursing_Right();  //�ұ�������
	delay_ms(1000);
	Uart_Back_Nursing_Right();  //�ұ�������λ
	delay_ms(1000);
	Uart_Waist_Nursing_Right(); //����������
	delay_ms(1000);
	Uart_Waist_Nursing_Right(); //����������λ
	delay_ms(1000);
	WriteInUART2("BodyRightDownPhone");
	Uart_Body_Right();          //�ҷ���λ
	delay_ms(1000);
	
	//������
		u2_printf("\r\n\r\n*****������******\r\n\r\n");
	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //������
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //�����ȸ�λ
	delay_ms(1000);
	
	//�γ�����-֧���������ȡ���������С����
	
	//֧��
		u2_printf("\r\n\r\n*****֧��******\r\n\r\n");
	WriteInUART2("BackUpPhone");
	Uart_Back();                //֧��
	delay_ms(1000);
//	
//	//������
	u2_printf("\r\n\r\n*****������******\r\n\r\n");
	WriteInUART2("LegDownDownPhone");
	Uart_Leg_Down();            //������	
	delay_ms(1000);

//������
	u2_printf("\r\n\r\n*****������******\r\n\r\n");
	WriteInUART2("LegDownDownPhone");
	WashLet_V2(1,32950);

//	
//	//������
//	Uart_Washlet(0);            //��������
//	delay_ms(1000);
//	Uart_Washlet(1);            //�������ر�
//	delay_ms(1000);
//	
//	//��ϴ���
//	washlet_flag=1;	//��
//	RELAY6=1; 
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//��
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(1,5000);
//	delay_ms(1000);
//	Uart_Push_Rod_Swash(0,5000);	//��
//	RELAY6=0;
//	delay_ms(1000);
//	
//	//����
//	washlet_flag=1;
//	RELAY6=1;                  //�̵����õ�
//	Uart_Washlet_Tig(0);
//	RELAY6=0;   
//	delay_ms(1000);

//�Զ�����
//	WriteInUART2("WashletAutoPhone");
//	Uart_Washlet_Auto();
//  delay_ms(1000);
	

	
		u2_printf("\r\n\r\n*****�����ȸ�λ******\r\n\r\n");
	WriteInUART2("LegDownUpPhone");
	Uart_Leg_Down();            //�����ȸ�λ
	delay_ms(1000);
//	
	u2_printf("\r\n\r\n*****֧����λ******\r\n\r\n");
	WriteInUART2("BackDownPhone");
	Uart_Back();                //֧����λ
		
		Muscle_Massager();	
		delay_ms(5000);delay_ms(2000);
		Muscle_Massager();	
		
		
	WriteInUART2("HangRunUp+11+1100");
	DG_Relay=1;		//�̵����õ�
	hangid=usmart_strnum2(USART2_RX_BUF);			
	if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
	{
			HangRun(1,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
	{
		HangRun(0,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	
	
		WriteInUART2("HangRunUp+11+0011");
	DG_Relay=1;		//�̵����õ�
	hangid=usmart_strnum2(USART2_RX_BUF);			
	if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
	{
			HangRun(1,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
	{
		HangRun(0,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	
//	
	WriteInUART2("HangRunDown+11+1111");
	hangid=usmart_strnum2(USART2_RX_BUF);			
	if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
	{
			HangRun(1,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}
	else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
	{
		HangRun(0,hangid/10000,hangid/1000%10,hangid/100%10,hangid/10%10,hangid%10);
	}	
//	
	DG_Relay=0;		//�̵���ʧ��	
	u2_printf("\r\n\r\n*****���н���******\r\n\r\n");	
	FlagClear();
}




/***********************************************************************
 ������      ��Uart_GB_Back(void)  
 ��������    ������֧����������
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_GB_Back(void)
{
	static u8 back_limit_flag; //�ж�֧���Ƿ����е�����λ�ã����Ƿ��ͼ���λ��ͼƬ
	u8 direct,key;
	u16 arr_now;               //��ǰһ�����������������������ۼ� 
	u8 len;	                   //���ܵ��ַ�������
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�����break������־λ
	static u8 k=0;            //���͵�k��ͼƬָ��
	u8 i=0;
	u8 j=0;	
	u16 arr_send;             //��ǰһ������������
	static u8 kj;
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧��
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{			
		back_dir_flag=!back_dir_flag;    //֧�������־λ
		if(back_dir_flag==1)
		{
			if(back_angle_to_arr(back_angle_lim)>back_runed_arr)  //����ʱ���Ƚ�ֵΪ���м���װ��ֵ��У׼��ĵ���װ��ֵ�Ĳ�ֵ��������ֵ����Ϊ0
			{
				direct=1;
				if(back_flag==0)
				{
					back_flag=1;
					delay_ms(200);
					u2_printf("back_flag==1");
					delay_ms(200);
					u2_printf("Guardbar_Back_Start");
					delay_ms(200);
					u2_printf("Cartoon_Back_1");
					delay_ms(200);
				}
				Motor_1_START(1);
				TIM10_Init(back_angle_to_arr(back_angle_lim)-back_runed_arr,timer10_freq);
			}
		}
		if(back_dir_flag==0)
		{
			if(back_runed_arr>0)    //����ʱ���Ƚ�ֵΪУ׼��ĵ���װ��ֵ��0,�Ҳ���Ϊ0
			{
				direct=0;
				if(back_limit_flag==1)
				{
					back_limit_flag=0;
					u2_printf("Cartoon_Back_20");
					delay_ms(200);
				}
				Motor_1_START(0);
				TIM10_Init(back_runed_arr,timer10_freq);
			}
		}				
		
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;		
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ	 	
		
		if(((back_runed_arr!=back_angle_to_arr(back_angle_lim))&&(1==direct))||((0!=back_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ�� 
			{			
				for(repeat_num=0;repeat_num<700;repeat_num++)
				{
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BackGB"))||(strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))  )   //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						}
						else 
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���								
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}						
					}
				}	
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//����ͼƬָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(back_runed_arr+arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}
				else
				{
					j=abs(back_runed_arr,arr_send)/(back_angle_to_arr(back_angle_max)/19);
				}			
				k=back_picture_k;			
				if(	k!=j)
				{
					kj=abs(k,j);												
					if(kj<2)
					{
						k=j;	back_picture_k=k;
						switch (k)
						{					
							case 1:	u2_printf("Cartoon_Back_2");									
									break;
							case 2:	u2_printf("Cartoon_Back_3");
									break;					
							case 3:	u2_printf("Cartoon_Back_4");
									break;					
							case 4:	u2_printf("Cartoon_Back_5");
									break;	
							case 5:	u2_printf("Cartoon_Back_6");
									break;	
							case 6:	u2_printf("Cartoon_Back_7");
									break;
							case 7:	u2_printf("Cartoon_Back_8");
									break;
							case 8:	u2_printf("Cartoon_Back_9");
									break;						
							case 9:	u2_printf("Cartoon_Back_10");
									break;												
							case 10:u2_printf("Cartoon_Back_11");
									break;
							case 11:u2_printf("Cartoon_Back_12");									
									break;
							case 12:u2_printf("Cartoon_Back_13");
									break;					
							case 13:u2_printf("Cartoon_Back_14");
									break;					
							case 14:u2_printf("Cartoon_Back_15");
									break;	
							case 15:u2_printf("Cartoon_Back_16");
									break;	
							case 16:u2_printf("Cartoon_Back_17");
									break;
							case 17:u2_printf("Cartoon_Back_18");
									break;
							case 18:u2_printf("Cartoon_Back_19");
									break;						
						}	
					}				
				}							
			}
			Motor_1_STOP();        //���ֹͣ
			TIM10_Stop();          //�رն�ʱ��
			break_flag=0;	       //���break��־λ	
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))   //�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
			{				
				arr_now=0;         //��ʱ���ڸ�λ״̬����״ֵ̬����Ϊ0��
				back_flag=0;
				delay_ms(200);
				u2_printf("back_flag==0");
				delay_ms(200);
			}		
			else
			{
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);    //��ȡ��ǰ����ֵarr_now				
				back_flag=1;							 
			}	
			//ͨ���������ж������ۼ�
			if(	direct==1)        //�����֧�����У�����+
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{  
					back_runed_arr=back_angle_to_arr(back_angle_lim); 
					back_limit_flag=1; 
					delay_ms(200);
					u2_printf("Cartoon_Back_20");		
					delay_ms(200);
					u2_printf("GuardbarBackLim");
				}
				else
				{  back_runed_arr=back_runed_arr+arr_now;	}				
			}
			else                //�����֧�����У�����-
			{
				if(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))
				{	
					back_runed_arr=0;	
					delay_ms(200);
					u2_printf("Cartoon_Back_1"); 
					delay_ms(200);
					u2_printf("GuardbarBackRes");					
				}
				else
				{
					back_runed_arr=back_runed_arr-arr_now;
				}						
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 								
		}
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BackInterfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_GB_Body_Left(void)  
 ��������    ����������
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_GB_Body_Left(void)
{
	u8 direct,key;
	u16 arr_now;              //��ǰһ�����������������������ۼ�
	u8 len;                   //���ܵ��ַ�������
	u16 arr_feed;             //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;          // �������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ� 
	u16 num1=0,num2=0,num3=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //�����brak������־λ      
	static u8 M345_Start;     //345����ӳ�ʼλ���˶�
	static u8 M345_End;       //345������е��ϼ���λ�ñ�־λ  
	static u8 k=0,m=0;
	u8 i=0;
	u8 mn;
	u8 kj;
	u8 j=0,n=0;
	u16 arr_send,arr_send1;	 //��ǰһ������������	
	
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ��������
	if((lock_flag==1)&&(back_flag==0)&&(body_right_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_left_flag==0)&&(waist_nursing_left_flag==0))
	{			
		if(body_left_flag==0)   //�����λ����ʼ״̬����ִ��С�෭����
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftGB"))
			{
			    //5�Ų෭��
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				delay_ms(200);
				u2_printf("GuardbarBodyLeftStart");
				delay_ms(200);
				u2_printf("body_left_flag=1");				
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				DIR5=1; 				
				body_left_flag=1;
				W25QXX_Write((u8*)&body_left_flag,33,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	              //�������	
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq);  //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);               //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{				
						//�����λ
//						if((0==GD5_Left_End)&&(1==body_left_flag))  //����ʱ������翪�أ�����ѭ�� 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Left_End)
//							{
//								break_flag=1;
//								u2_printf("GD5LeftEnd");
//								break;			
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}											
						//������ϡ��������
//						if(1==Motor5_Alm)       
//						{						
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("GuardbarBodyLeftOverload5");
//								Uart_Breakdown_Treatment();
//								break_flag=1;
//								break;
//							}								
//						}						
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//����ͼƬָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					m=left_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);					
						if(mn<2)
						{
							m=n;	left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}							
						}
					}					
				}					      
				Motor_5_STOP();       //���ֹͣ
				TIM10_Stop();         //��ʱ���ر�
				break_flag=0;  	      //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(0,4500,motor_body_freq,motor_timer_freq);//���ò�������
			}	
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		//����345�������	
		if(1==body_left_flag)
		{
			body_left_dir_flag=!body_left_dir_flag;
			if(body_left_dir_flag==1)    //����
			{				
				if(body_angle_to_arr(body_left_angle_lim)>body_left_runed_arr)
				{
				   //345��������				
					 DIR3=0;DIR4=0;DIR5=0;direct=1;
					 if(M345_Start==0)
					 {
						 M345_Start=1;
						 u2_printf("Cartoon_Body_Left_1");
						 delay_ms(200);
					 }								 
					 Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);	  //�������			
					 TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);									 
				}
			}
			else          //����
			{				
				if(body_left_runed_arr>0)
				{
				   //345��������
				   DIR3=1;DIR4=1;DIR5=1;direct=0;
					if(M345_End==1)
					 {
						 M345_End=0;
						 u2_printf("Cartoon_Body_Left_8");
						 delay_ms(200);
					 }											
				   Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);   //�������			
				   TIM10_Init(body_left_runed_arr,timer10_freq);	           //�򿪶�ʱ��		
				} 
			}
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);              //����жϱ�־λ
		
		if(((body_left_runed_arr!=body_angle_to_arr(body_left_angle_lim))&&(1==direct))||((0!=body_left_runed_arr)&&(0==direct)))
		{			
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{
					//�����λ
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Start)
//						{
//							u2_printf("GD4Start");
//							break_flag=1;
//							break;
//						}
//						
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if((0==GD3_Left_End)||(0==GD4_Left_End))
//						{
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3LeftEnd");						
//							}
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4LeftEnd");
//							}
//							break_flag=1;
//							break;
//						}
//					}
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftGB"))||(strstr((const char *)USART2_RX_BUF,(const char *)"Stop")))  //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						} 
						else 
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}												
					}				
					//������ϡ��������
//					if((1==Motor3_Alm)||(1==Motor4_Alm)||(1==Motor5_Alm))         
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_left_overload_3=1;
//							u2_printf("GuardbarBodyLeftOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_left_overload_4=1;
//							u2_printf("GuardbarBodyLeftOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_left_overload_5=1;
//							u2_printf("GuardbarBodyLeftOverload5");
//						}						
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}									
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_left_runed_arr+arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				else
				{
					j=abs(body_left_runed_arr,arr_send)/(body_angle_to_arr(body_left_angle_max)/7);
				}
				k=body_left_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;   body_left_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Left_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Left_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Left_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Left_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Left_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Left_7");
									break;		
						}
					}				
				}					
			}							
			Motor_3_4_5_STOP();   //���ֹͣ
			TIM10_Stop();         //��ʱ���ر�
			break_flag=0;         //���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //�ж��Ƿ񵽴︴λ״̬
			{
				arr_now=0;                  
				body_left_flag=0;
//				W25QXX_Write((u8*)&body_left_flag,33,1);			
			}					
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);      //��ȡ��ǰ����ֵarr_now
				body_left_flag=1;
//				W25QXX_Write((u8*)&body_left_flag,33,1);	
			}
			//ͨ���������ж������ۼ�
			if(direct==1)       //���У�����+
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Left_End)||(0==GD4_Left_End))  //�������е�����λ��
				{
					body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);					
					M345_End=1;	 										
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_8");
					delay_ms(200);
					u2_printf("GuardbarBodyLeftLim");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr+arr_now;
				}		
			}	
			else                //���У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))      //�������е�����λ��
				{						
					body_left_runed_arr=0;
					M345_Start=0;											
					delay_ms(200);
					u2_printf("Cartoon_Body_Left_1");
				}
				else
				{
					body_left_runed_arr=body_left_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ
			
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_left_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //���3û��λ
//				{
//					DIR3=1;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								u2_printf("GD3Start");			
//								break;	
//							}								
//						}
//					}			
//					Motor_3_STOP();       //���ֹͣ
//					TIM10_Stop();         //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //���4û��λ
//				{
//					DIR4=1;
//					Motor_4_START(motor_body_freq,motor_timer_freq);   //�������
//					TIM10_Init(add_arr,timer10_freq);                  //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD4_Start) 
//							{
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_4_STOP();                                     //���ֹͣ
//					TIM10_Stop();                                       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //���3��4��û��λ
//				{
//					DIR3=1; DIR4=1; DIR5=1;
//					Motor_3_4_5_START_left(motor_body_freq,motor_timer_freq);//�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//								break;
//							}
//							if((0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//								break;
//							}
//						}
//					}			
//					Motor_3_4_5_STOP();   //���ֹͣ
//					TIM10_Stop();         //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
			//�෭��λ
			if((body_left_flag==0)&&(0==direct))     //345������λ����ʼ״̬���Ÿ�λ5�ŵ��
			{			
				//5�Ų෭��λ
//				Motor_4_Compensate(1,4500,motor_body_freq,motor_timer_freq);//���ò�������
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				DIR5=0;
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_10");
				delay_ms(200);
				Motor_5_START(motor_body_freq,motor_timer_freq);                 //�������
				body_left_runed_arr=0;
				
				TIM10_Init(body_angle_to_arr(body_left_angle_lim),timer10_freq); //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);              //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{				
					for(repeat_num1=0;repeat_num1<650;repeat_num1++)
					{
						//�����λ
//						if((0==GD5_Start)&&(0==body_left_flag))    //������翪������ѭ�������ͣת 
//						{								
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								break_flag=1;
//								u2_printf("GD5Start");
//								break;			
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}									
						//������ϡ��������
//						if(1==Motor5_Alm)         
//						{
//							delay_us(100);
//							if(1==Motor5_Alm)
//							{
//								body_left_overload_5=1;
//								u2_printf("GuardbarBodyLeftOverload5");
//								Uart_Breakdown_Treatment();
//								break_flag=1;
//								break;
//							}								
//						}						
					}	
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					}
					//���Ͷ���ָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_left_angle_lim)/9);
					n=9-n;	
					m=left_motor5_picture_m;
					if(	m!=n)
					{			
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;    left_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Left_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Left_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Left_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Left_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Left_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Left_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Left_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Left_Motor5_9");
										break;								
							}	
						}				
					}	
				}      
				Motor_5_STOP();     //���ֹͣ
				TIM10_Stop();       //��ʱ���ر�
				break_flag=0;       //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Left_Motor5_1");
				delay_ms(200);
				u2_printf("body_left_flag==0");				
				delay_ms(200);
				u2_printf("GuardbarBodyLeftRes");
				//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//				{
//					DIR5=0;
//					Motor_5_START(motor_body_freq,motor_timer_freq);
//					TIM10_Init(add_arr,timer10_freq);   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD5_Start)
//							{
//								u2_printf("GD5Start");
//								break;		
//							}								
//						}
//					}			
//					Motor_5_STOP();
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}			
			}				
		}
	}
	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BodyLeftInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_GB_Body_Right(void)  
 ��������    �������ҷ���
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_GB_Body_Right(void)
{
	u8 direct,key;
	u16 arr_now;              //��ǰһ�����������������������ۼ�
	u8 len;                   //���յ��ַ�������
	u16 arr_feed;             //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;          //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num1=0,num2=0,num3=0;
	//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num,repeat_num1;
	u8 break_flag=0;          //�����break������־λ 
	static u8 k=0,m=0;
	static u8 M345R_Start;    //345����ӳ�ʼλ������
	static u8 M345R_End;      //345������е��ϼ���λ��
	u8 mn;
	u8 kj;
	u8 i=0;
	u8 j=0,n=0;
	u16 arr_send,arr_send1;	  //��ǰһ������������	
	
	//�������ܣ�ֻ����֧�����������ȡ����㡢���Ӹ�λ�󣬲���ִ���ҷ�����
	if((lock_flag==1)&&(back_flag==0)&&(body_left_flag==0)&&(leg_up_flag==0)&&(leg_down_flag==0)&&(washlet_flag==0)&&(desk_flag==0)&&(back_nursing_right_flag==0)&&(waist_nursing_right_flag==0))
	{			
		if(body_right_flag==0)   //�����λ����ʼ״̬����ִ������
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightGB"))
			{
			  //5�Ų෭��
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				delay_ms(200);
				u2_printf("GuardbarBodyRightStart");
				delay_ms(200);
				u2_printf("body_right_flag==1");
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_1");
				delay_ms(200);
				DIR5=0; 
				body_right_flag=1;
				W25QXX_Write((u8*)&body_right_flag,34,1);				
				Motor_5_START(motor_body_freq,motor_timer_freq);	              //�������	
				TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq); //�򿪶�ʱ��                        //�򿪶�ʱ��
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);               //����жϱ�־λ
				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
				{	
					for(repeat_num1=0;repeat_num1<600;repeat_num1++)
					{				
						//�����λ
//						if((0==GD5_Right_End)&&(1==body_right_flag))                      //����ʱ������翪�أ�����ѭ�� 
//						{			   										
//							delay_us(100);
//							if(0==GD5_Right_End)
//							{
//								u2_printf("GD5RightEnd");
//								break_flag=1;
//								break;		
//							}								
//						}
						  //�ж���û���յ���λ��ָ��		
						if(USART2_RX_LEN&0x8000)
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;												
						}											
						//������ϡ��������
//						if(1==Motor5_Alm)         
//						{		
//							delay_us(100);
//							if(1==Motor5_Alm)  
//							{
//								body_right_overload_5=1;
//								u2_printf("GuardbarBodyRightOverload5");
//								Uart_Breakdown_Treatment();
//								break_flag=1;
//								break;	
//							}								
//						}											
					}
					if(break_flag==1)
					{
						u2_printf("break_flag==1");
						break;
					} 
					//����ͼƬָ��
					arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
					m=right_motor5_picture_m;
					if(m!=n)
					{
						mn=abs(m,n);
						if(mn<2)
						{
							m=n;   right_motor5_picture_m=m;
							switch (m)
							{						
								case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
										break;
								case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
										break;					
								case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
										break;					
								case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
										break;	
								case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
										break;	
								case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
										break;
								case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
										break;
								case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
										break;							
							}	
						}					
					}		
				}						      
				Motor_5_STOP();        //���ֹͣ
				TIM10_Stop();          //��ʱ���ر�
			    break_flag=0;          //���break��־λ
				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
				delay_ms(200);
				u2_printf("Cartoon_Body_Right_Motor5_10");
				delay_ms(200);
				Motor_4_Compensate(1,4500,motor_body_freq,motor_timer_freq);//���ò�������
			}	
		}
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		//����345�������	
		if(1==body_right_flag)
		{
			body_right_dir_flag=!body_right_dir_flag;
			if(body_right_dir_flag==1)
			{				
				if(body_angle_to_arr(body_right_angle_lim)>body_right_runed_arr)
				{
				   //345��������		
				   DIR3=1;DIR4=1;DIR5=1;direct=1;
				   if(M345R_Start==0)
				   {
					   M345R_Start=1;	
					   delay_ms(200);
					   u2_printf("Cartoon_Body_Right_1");
					   delay_ms(200);
				   }									 		
				   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq); //�������				
				   TIM10_Init(body_angle_to_arr(body_left_angle_lim)-body_left_runed_arr,timer10_freq);//�򿪶�ʱ��			
				}
			}
			else
			{				
				if(body_right_runed_arr>0)
				{
				   //345��������
				   DIR3=0;DIR4=0;DIR5=0;direct=0;
				   if(M345R_End==1)
				   {
						M345R_End=0;
						delay_ms(200);
					    u2_printf("Cartoon_Body_Right_8");	
					    delay_ms(200);
					}							 	
				   Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);	  //�������			
				   TIM10_Init(body_right_runed_arr,timer10_freq);	      //�򿪶�ʱ��		
				}
			}
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ		
		if(((body_right_runed_arr!=body_angle_to_arr(body_right_angle_lim))&&(1==direct))||((0!=body_right_runed_arr)&&(0==direct)))
		{
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ
//					if(((0==GD3_Start)||(0==GD4_Start))&&(0==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{						
//						delay_us(100);
//						if(0==GD3_Start)
//						{
//							u2_printf("GD3Start");
//							break_flag=1;
//							break;
//						}
//						if((0==GD4_Start))
//						{
//							u2_printf("GD4Start");
//							break_flag=1;
//							break;
//						}
//					}
//					if(((0==GD3_Left_End)||(0==GD4_Left_End))&&(1==direct))   //���������翪�أ�����ѭ����ֹͣ����
//					{
//						delay_us(100);
//						if(0==GD3_Left_End)
//						{
//							u2_printf("GD3LeftEnd");
//							break_flag=1;
//							break;
//						}
//						if(0==GD4_Left_End)
//						{
//							u2_printf("GD4LeftEnd");
//							break_flag=1;
//							break;
//						}
//					}
					//ָֹͣ��
					if(USART2_RX_LEN&0x8000)
					{
						len=USART2_RX_LEN&0x3fff;				
						USART2_RX_BUF[len]=0;
						if((strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightGB"))||(strstr((const char *)USART2_RX_BUF,(const char *)"Stop")) )    //�����յ�Stop,������ѭ��	
						{
							break_flag=1;
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
							break;
						} 
						else 
						{
							u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
							memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
							USART2_RX_LEN=0;
						}
					}			
					//������ϡ��������
//					if((1==Motor3_Alm) || (1==Motor4_Alm) || (1==Motor5_Alm))       
//					{						
//						delay_us(100);
//						if(1==Motor3_Alm)
//						{
//							body_right_overload_3=1;
//							u2_printf("GuardbarBodyRightOverload3");
//						}
//						if(1==Motor4_Alm)
//						{
//							body_right_overload_4=1;
//							u2_printf("GuardbarBodyRightOverload4");
//						}
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("GuardbarBodyRightOverload5");
//						}					
//						Uart_Breakdown_Treatment();
//						break_flag=1;
//						break;		             
//					}							
				}
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				if(direct==1)
				{
					j=(body_right_runed_arr+arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				else
				{
					j=abs(body_right_runed_arr,arr_send)/(body_angle_to_arr(body_right_angle_max)/7);
				}
				k=body_right_picture_k;
				if(	k!=j)
				{	
					kj=abs(k,j);			
					if(kj<2)
					{
						k=j;   body_right_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_7");
									break;	
						}
					}				
				}							
			}							
			Motor_3_4_5_STOP();   //���ֹͣ
			TIM10_Stop();         //��ʱ���ر�
			break_flag=0;	      //���break��־λ
			//�жϸ�λ
			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))  //�ж��Ƿ񵽴︴λ״̬
			{
				arr_now=0;        //��ʱarr_nowΪ0
				body_right_flag=0;	
//				W25QXX_Write((u8*)&body_right_flag,34,1);			
			}			
			else
			{					
				arr_now=__HAL_TIM_GET_COUNTER(&TIM10_Handler);         //��ȡ��ǰ����ֵarr_now
				body_right_flag=1;
//				W25QXX_Write((u8*)&body_right_flag,34,1);	
			}
			//ͨ���������ж������ۼ�
			if(direct==1)      //���У�����+
			{ 
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Right_End)||(0==GD4_Right_End)) //�������е�����λ��
				{
					body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
					M345R_End=1;    								
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_8");
					delay_ms(200);
					u2_printf("GuardbarBodyRightLim");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr+arr_now;
				}		
			}	
			else              //���У�����-
			{
				if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))||(0==GD3_Start)||(0==GD4_Start))   //�������е�����λ��
				{
					body_right_runed_arr=0;	
					M345R_Start=0;  								
					delay_ms(200);
					u2_printf("Cartoon_Body_Right_1");
				}
				else
				{
					body_right_runed_arr=body_right_runed_arr-arr_now;	
				}
			}					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);       //����жϱ�־λ
//		//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ�
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&((1==GD3_Start)||(1==GD4_Start))&&(direct==0)&&(body_right_flag==1))
//			{   			
//				if((1==GD3_Start)&&(0==GD4_Start))   //���3û��λ
//				{
//					DIR3=0;
//					Motor_3_START(motor_body_freq,motor_timer_freq);    //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD3_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD3_Start) 
//							{
//								u2_printf("GD3Start");
//								break;	
//							}							
//						}
//					}			
//					Motor_3_STOP();     //���ֹͣ
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
//				}
//				if((0==GD3_Start)&&(1==GD4_Start))   //���4û��λ
//				{
//					DIR4=0;
//					Motor_4_START(motor_body_freq,motor_timer_freq);    //�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if(0==GD4_Start)  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD4_Start)
//							{
//								u2_printf("GD4Start");
//								break;	
//							}							
//						}
//					}			
//					Motor_4_STOP();     //���ֹͣ
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ				
//				}
//				if((1==GD3_Start)&&(1==GD4_Start))   //���3/4��û��λ
//				{
//					DIR3=0; DIR4=0; DIR5=0;
//					Motor_3_4_5_START_right(motor_body_freq,motor_timer_freq);//�������
//					TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//					while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//					{			
//						if((0==GD3_Start)||(0==GD4_Start))  //����ʱ������翪�أ�����ѭ�� 
//						{
//							delay_us(100);
//							if(0==GD3_Start)
//							{
//								u2_printf("GD3Start");
//								break;	
//							}
//							if((0==GD4_Start) 
//							{
//								u2_printf("GD4Start");
//								break;	
//							}
//						}
//					}			
//					Motor_3_4_5_STOP(); //���ֹͣ
//					TIM10_Stop();       //��ʱ���ر�
//					__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//				}
//			}
		//�෭��λ
		if((body_right_flag==0)&&(0==direct))     //345������λ����ʼ״̬���Ÿ�λ5�ŵ��
		{			
			//5�Ų෭��λ
			Motor_4_Compensate(0,4500,motor_body_freq,motor_timer_freq);//���ò�������
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;
			DIR5=1;
			delay_ms(200);
			u2_printf("Cartoon_Body_Right_Motor5_10");
			delay_ms(200);
			Motor_5_START(motor_body_freq,motor_timer_freq);  //�������
			body_right_runed_arr=0;
		
			TIM10_Init(body_angle_to_arr(body_right_angle_lim),timer10_freq);                              //�򿪶�ʱ��
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))   //�ȴ���ʱʱ�䵽��
			{				
				for(repeat_num=0;repeat_num<600;repeat_num++)
				{			
					//�����λ
//					if((1==GD5_Start)&&(0==body_right_flag))       //����ʱ������翪�أ�����ѭ�� 
//					{			   										
//						delay_us(100);
//						if(1==GD5_Start)
//						{	
//							u2_printf("GD5Start");	
//							break_flag=1;
//							break;		
//						}							
//					}
					  //�ж���û���յ���λ��ָ��		
					if(USART2_RX_LEN&0x8000)
					{
						u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
						memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
						USART2_RX_LEN=0;												
					}					
					//������ϡ��������
//					if(1==Motor5_Alm)
//					{
//						delay_us(100);
//						if(1==Motor5_Alm)
//						{
//							body_right_overload_5=1;
//							u2_printf("GuardbarBodyRightOverload5");
//							Uart_Breakdown_Treatment();
//							break_flag=1;
//							break;		
//						}							
//					}									
				}				
				if(break_flag==1)
				{
					u2_printf("break_flag==1");
					break;
				}
				//���Ͷ���ָ��
				 arr_send1=__HAL_TIM_GET_COUNTER(&TIM10_Handler);			
				 n=arr_send1/(body_angle_to_arr(body_right_angle_lim)/9);
				 n=9-n;
				 m=right_motor5_picture_m;
				if(	m!=n)
				{	
					mn=abs(m,n);			
					if(mn<2)
					{
						m=n;   right_motor5_picture_m=m;
						switch (m)
						{						
							case 1:	u2_printf("Cartoon_Body_Right_Motor5_2");									
									break;
							case 2:	u2_printf("Cartoon_Body_Right_Motor5_3");
									break;					
							case 3:	u2_printf("Cartoon_Body_Right_Motor5_4");
									break;					
							case 4:	u2_printf("Cartoon_Body_Right_Motor5_5");
									break;	
							case 5:	u2_printf("Cartoon_Body_Right_Motor5_6");
									break;	
							case 6:	u2_printf("Cartoon_Body_Right_Motor5_7");
									break;
							case 7:	u2_printf("Cartoon_Body_Right_Motor5_8");
									break;
							case 8:	u2_printf("Cartoon_Body_Right_Motor5_9");
									break;							
						}
					}				
				}	
			}      
			Motor_5_STOP();        //���ֹͣ
			TIM10_Stop();		   //��ʱ���ر�
			break_flag=0;          //���break��־λ
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
			delay_ms(200);
			u2_printf("Cartoon_Body_Right_Motor5_1");
		    delay_ms(200);
			u2_printf("body_right_flag==0");			
		    delay_ms(200);
		    u2_printf("GuardbarBodyRightRes");
			//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ��������������ж�ɾ����
//			if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==GD5_Start)&&(direct==0)&&(0==body_left_flag))
//			{
//				DIR5=1;
//				Motor_5_START(motor_body_freq,motor_timer_freq);    //�������
//				TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//				while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//				{			
//					if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//					{
//						delay_us(100);
//						if(0==GD5_Start) 
//						{
//							u2_printf("GD5Start");
//							break;		
//						}							
//					}
//				}			
//				Motor_5_STOP();        //���ֹͣ
//				TIM10_Stop();		   //��ʱ���ر�
//				__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//			}			
		}			
	 }
  }

	else
	{
		LED0=0;   //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("BodyRightInterfere");		
		LED0=1;	
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_GB_Back_Nursing(void)  
 ��������    ��������������
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_GB_Back_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //�����ʱ��������������󱳲�����
	{
		Uart_Back_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//�����ʱ�����ҷ���������ұ�������
	{
		Uart_Back_Nursing_Right();
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

/***********************************************************************
 ������      ��Uart_GB_Back_Nursing(void)  
 ��������    ��������������
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_GB_Waist_Nursing(void)
{
	if((lock_flag==1)&&(1==body_left_flag))  //�����ʱ�����������������������
	{
		Uart_Waist_Nursing_Left(); 
	}
	if((lock_flag==1)&&(1==body_right_flag))//�����ʱ�����ҷ������������������
	{
		Uart_Waist_Nursing_Right();
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;	
		
	}	
}

/***********************************************************************
 ������      ��Uart_GB_Lock(void)  
 ��������    ��һ����������
 ����        ����
 ���        ����
                           
************************************************************************/
u8 Uart_GB_Lock(void)
{	
	lock_dir_flag=!lock_dir_flag;
	if(lock_dir_flag==1)
	{
		lock_flag=0;	//��������
		u2_printf("GBLock");
	}
	else
	{
		lock_flag=1;   //��������
		u2_printf("GBUnLock");
	}
    return 	lock_flag;
}

/***********************************************************************
 ������      ��Uart_Washlet_Auto(void)   
 ��������    ������ִ���Զ�����������
 ����        ����
 ���        ���� 
                          
************************************************************************/
void Uart_Washlet_Auto(void) 
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;	
	
	//�������ܣ�ֻ���������ȡ����ҷ���λ�󣬲���ִ������������	
	if((lock_flag==1)&&(0==body_left_flag)&&(0==body_right_flag)) 
	{
		if(0==washlet_auto_flag)
		{		
			delay_ms(200);		
			u2_printf("Uart_Washlet_Auto_Start");
			delay_ms(200);
		}                         
		washlet_auto_dir_flag=!washlet_auto_dir_flag; 
		
		if((washlet_auto_dir_flag==1)&&(0==washlet_auto_flag))     //�Զ��������г�
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
		else if((washlet_auto_dir_flag==0)&&(1==washlet_auto_flag))         //�Զ����㸴λ-֧���������ȸ�λ
		{
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
		if(washlet_auto_dir_flag==1)
		{
			Uart_Washlet(0);	            //�����			
			delay_ms(1000);
//			Uart_Washlet_Weight();          //������
//			delay_ms(1000);
			Uart_Swash_Dry();               //��ϴ���			
			Uart_Washlet(1);	            //����ر�
			if(washlet_flag==0)             //�ж������Ƿ��ڸ�λ״̬���ٽ���������ս�
			{									
				Uart_Washlet_Tig(1);        //������ս�
				delay_ms(100);
				Uart_Washlet_Auto();        //�ٴε��øú�����ʹ��־λȡ������λ
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


/***********************************************************************
 ������      ��Uart_Washlet(void)  
 ��������    ������ִ������������
 ����        ��dir: 0(������)��1���ر����㣩
 ���        ����
                           
************************************************************************/
void Uart_Washlet(u8 dir)
{
	u8 direct;       //����ĳ���������еķ����־��1-�������У�0-��������
	u8 key;          //����ɨ�躯������ֵ,�����жϵ��ʧ������
	u16 num,len;
	u16 arr_feed;    //��������е�ǰһ������������
	u16 pulse_num;   //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u16 repeat_num;
	u8 break_flag=0;          //�����break������־λ
	static u16 k=0;           //���͵�k��ͼƬָ��
	u8 i=0;
	u16 j=0;	
	u16 arr_send;             //��ǰһ������������
	static u8 kj;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	washlet_flag=1;
	//�������ܣ�ֻ���������ȡ����ҷ���λ�󣬲���ִ������������
	if((leg_up_flag==0)&&(body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{
		DIR6=dir;
		direct=!dir;		

		Motor_6_START(motor_washlet_freq,motor_timer_freq);           //�������
		TIM10_Init(washlet_arr_lim,timer10_freq);                     //�򿪶�ʱ��35000
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ
		if(1==direct)
		{
			washlet_flag=1;	
			u2_printf("Cartoon_Washlet_1");
			delay_ms(200);    
		}
		else
		{
			u2_printf("Cartoon_Washlet_25");
			delay_ms(200);
		}
		
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))    //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		{						
			for(repeat_num=0;repeat_num<600;repeat_num++)
			{	
				//�����λ
//				if((0==GD6_End)&&(1==direct))
//				{
//					delay_us(100);
//					if(0==GD6_End)
//					{
//						break_flag=1;
//						u2_printf("GD6End"); 
//						break ;
//					}						
//				}
//				if((0==GD6_Start)&&(0==direct))
//				{
//					delay_us(100);
//					if(0==GD6_Start)
//					{
//						break_flag=1;
//						u2_printf("GD6Start"); 
//						break ; 
//					}
//				}
				  //�ж���û���յ���λ��ָ��		
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
			}
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			//���Ͷ���ָ��
			arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
			j=arr_send/(washlet_arr_lim/24);
			if(0==direct)
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
		break_flag=0;      //���break��־λ
		//�жϸ�λ
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(direct==0))//�ж��Ƿ��ڸ�λ״̬����λ״̬��ǰ�������еĶ�ʱ������
		{		
			washlet_flag=0;	 
			delay_ms(200);
			u2_printf("Cartoon_Washlet_1");		
		}
		 else if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(1==direct))
		 {
			washlet_flag=1;    
			delay_ms(200);
			u2_printf("Cartoon_Washlet_25");
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ
		//ʹ�����λ����ʼ״̬����簲װ��ֱ�Ӵ򿪴˶Σ�
//		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(0==GD6_Start)&&(direct==0))  //��ʱʱ�䵽�����û��
//		{
//			DIR6=1;
//			Motor_6_START(motor_washlet_freq,motor_timer_freq);           //�������
//			TIM10_Init(add_arr,timer10_freq);                             //�򿪶�ʱ��35000
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);           //����жϱ�־λ	 	
//			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )  //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
//			{
//				if(0==GD6_Start)  //���������ػ�������翪�أ�������ѭ�������ֹͣת�� 
//				{  
//					delay_us(100);
//					if(0==GD6_Start) 
//					{
//						u2_printf("GD6Start");
//						break; 
//					}						
//				}				
//			}				                                 			
//			Motor_6_STOP();      //���ֹͣ
//			TIM10_Stop();        //�رն�ʱ��
//			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
//		}
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

/***********************************************************************
 ������      ��Uart_Washlet_Weight()   
 ��������    �����غ���
 ����        ����
 ���        ��1�������ڱ仯��0������δ�����仯 
                          
************************************************************************/
u8 Uart_Weight(void)  
{
    //��ʼ�������				
	if(num==1)
	{				         			
		num=2;
		
		u1=filter();
				u2_printf("\r\nu1=%d\r\n",u1);
		delay_ms(500);
		if(num==2)
		{	
			num=3;		
			u2=filter();
					u2_printf("\r\nu2=%d\r\n",u2);		
			delay_ms(500);
			if(num==3)
			{	 
				num=1;
				u3=filter();
					u2_printf("\r\nu3=%d\r\n",u3);
				delay_ms(500);
			}
		}
	}
	//������������Ƿ����仯
	 if((abs(u1,u2)<3999)&&(abs(u1,u3)<3999)&&(abs(u2,u3)<3999))//0x300=768
	 {
		 return 0;
	 }
	 else
	 {
		 return 1;
	 }	 	
}		

/***********************************************************************
 ������      ��Uart_Washlet_Tig()   
 ��������    ��������ս�
 ����        ��dir:������з����־��1-��ת��0-��ת
 ���        ����
                          
************************************************************************/	
void Uart_Washlet_Tig(u8 dir)
{	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;          //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	u16 k=0;                  //����k�Ŷ���
	u8 kj;
	u16 j=0;	
	u16 arr_send;                   //��ǰһ����������ֵ	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	u8 len;
	RELAY6=1;                       //�̵����õ磬��������պϣ��������������õ�
	delay_ms(1000);		
	Uart_Motor_6_2_START(1,17000);   //�����Ƹ����
	u2_printf("Cartoon_Washlet_Tig_1");	
	DIR6_1=dir;
	Motor_6_1_START(3600-1,motor_timer_freq);                  //�����������
//	Motor_6_1_START(7200-1,motor_timer_freq); 
	TIM2_Init(25000,timer10_freq);                              //�򿪶�ʱ��3500
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//�ж���û���յ���λ��ָ��		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //��ǰһ������ֵ
		//���䶯��ָ��
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
	Motor_6_1_STOP();                                           //���ֹͣ
	TIM2_Stop();                                                //�رն�ʱ��
	delay_ms(200);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
	Uart_Motor_6_2_START(0,17000);            //�����Ƹ�����
    delay_ms(1000);	
	RELAY6=0;                                //�̵�����λ���������������ϵ�
}	

void UartWashletTig(u8 dir,u32 TGArr,u32 SXArr)
{	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;          //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	u16 k=0;                  //����k�Ŷ���
	u8 kj;
	u16 j=0;	
	u16 arr_send;                   //��ǰһ����������ֵ	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	u8 len;
	RELAY6=1;                       //�̵����õ磬��������պϣ��������������õ�
	delay_ms(1000);		
	Uart_Motor_6_2_START(1,TGArr);   //�����Ƹ����
	u2_printf("Cartoon_Washlet_Tig_1");	
	DIR6_1=dir;
	Motor_6_1_START(3600-1,motor_timer_freq);                  //�����������
//	Motor_6_1_START(7200-1,motor_timer_freq); 
	TIM2_Init(SXArr,timer10_freq);                              //�򿪶�ʱ��3500
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//�ж���û���յ���λ��ָ��		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //��ǰһ������ֵ
		//���䶯��ָ��
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
	Motor_6_1_STOP();                                           //���ֹͣ
	TIM2_Stop();                                                //�رն�ʱ��
	delay_ms(200);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
	Uart_Motor_6_2_START(0,TGArr+4200);            //�����Ƹ�����
    delay_ms(1000);	
	RELAY6=0;                                //�̵�����λ���������������ϵ�
}	


void Uart_WashletTigOnly(u8 dir)
{	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;          //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	u16 k=0;                  //����k�Ŷ���
	u8 kj;
	u16 j=0;	
	u16 arr_send;                   //��ǰһ����������ֵ	
	
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	u8 len;
	RELAY6=1;                       //�̵����õ磬��������պϣ��������������õ�
	delay_ms(1000);		
//	Uart_Motor_6_2_START(1,16000);   //�����Ƹ����
	u2_printf("Cartoon_Washlet_Tig_1");	
	DIR6_1=dir;
	Motor_6_1_START(3600-1,motor_timer_freq);                  //�����������
//	Motor_6_1_START(7200-1,motor_timer_freq); 
	TIM2_Init(17000,timer10_freq);                              //�򿪶�ʱ��3500
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
	while(!(__HAL_TIM_GET_FLAG(&TIM2_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
	{
		for(repeat_num=0;repeat_num<700;repeat_num++)
		{ 			
			//�ж���û���յ���λ��ָ��		
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
		}
		 arr_send=__HAL_TIM_GET_COUNTER(&TIM2_Handler);      //��ǰһ������ֵ
		//���䶯��ָ��
		if(dir==1)
		{
			j=arr_send/(10500/6);
		}
		else
		{
			j=arr_send/(10500/6);
			j=6-j;
		}
		if(	k!=j)
		{	
			kj=abs(k,j);
			if(kj<2)
			{
				k=j;
				switch (k)
				{						
					case 1:	u2_printf("Cartoon_Washlet_Tig_2");									
							break;
					case 2:	u2_printf("Cartoon_Washlet_Tig_3");
							break;					
					case 3:	u2_printf("Cartoon_Washlet_Tig_4");
							break;					
					case 4:	u2_printf("Cartoon_Washlet_Tig_5");
							break;	
					case 5:	u2_printf("Cartoon_Washlet_Tig_6");
							break;																				
				}
			}
		}
	}
	__HAL_TIM_CLEAR_FLAG(&TIM2_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
	Motor_6_1_STOP();                                           //���ֹͣ
	TIM2_Stop();                                                //�رն�ʱ��
	delay_ms(200);
	u2_printf("Cartoon_Washlet_Tig_7");
	delay_ms(1000);
//	Uart_Motor_6_2_START(0,16000);            //�����Ƹ�����
    delay_ms(1000);	
	RELAY6=0;                                //�̵�����λ���������������ϵ�
}	



/***********************************************************************
 ������      ��Back_Leg()   
 ��������    ��֧����������ͬʱ���к���
 ����        ����
 ���        ���� 
                          
************************************************************************/
void Uart_Back_Leg(void)
{
	u8 len;
	u16 arr_now;               //��ǰһ������������
	
//ʵ����λ��ʵʱ��ʾ������ǰ�˶�״̬
	u8 break_flag=0;           //�жϳ����Ƿ��Ǵ�break������ 
	u16 repeat_num;
	static u8 k;               //����k�Ŷ���
	static u8 kj;
	u8 i=0;
	u8 j=0;	
	u16 arr_send;              //��ǰһ����������ֵ
		
	//�������ܣ�ֻ�������ҷ����ܸ�λ�󣬲��ܽ���֧����������
	if((body_left_flag==0)&&(body_right_flag==0)&&(lock_flag==1))
	{	
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;									
		//����֧���������Ȼ�֧����������ͬʱ����
		TIM10_Init(leg_angle_to_arr(leg_down_angle_lim),timer10_freq);  //��ʱ������=(freq1*freq_time1_1)/90mhz
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )    //��ʱʱ�䵽
		{
			 //�ж���û���յ����ָ��
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
			if(break_flag==1)
			{
				u2_printf("break_flag==1");
				break;
			}
			 //ֻ����֧������ָ��
			if((1==back_state_flag)&&(0==leg_down_state_flag))           //ִֻ��֧��
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);				
				j=arr_send/(back_angle_to_arr(back_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=back_picture_k;
				if(k!=j)
				{
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;   back_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_2");							
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_3");											
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_4");											
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_5");											
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_6");											
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_7");											
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_8");										
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_9");											
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_10");											
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_11");											
									break;
							case 11:u2_printf("Cartoon_Washlet_Back_12");																		
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_13");											
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_14");											
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_15");										
									break;	
							case 15:u2_printf("Cartoon_Washlet_Back_16");																		
									break;
							case 16:u2_printf("Cartoon_Washlet_Back_17");											
									break;					
							case 17:u2_printf("Cartoon_Washlet_Back_18");											
									break;					
							case 18:u2_printf("Cartoon_Washlet_Back_19");										
									break;							
						}
					}
				}
			}
			//ֻ�������ȶ���ָ��
			else if((0==back_state_flag)&&(1==leg_down_state_flag))      //ִֻ��������
			{
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(leg_angle_to_arr(leg_down_angle_max)/19);
				if(0==washlet_auto_dir_flag)
				{
					j=19-j;
				}
				k=leg_down_picture_k;
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;  leg_down_picture_k=k;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Leg_Down_15");
									break;	
							case 15:u2_printf("Cartoon_Washlet_Leg_Down_16");
									break;	
							case 16:u2_printf("Cartoon_Washlet_Leg_Down_17");	
									break;
							case 17:u2_printf("Cartoon_Washlet_Leg_Down_18");
									break;					
							case 18:u2_printf("Cartoon_Washlet_Leg_Down_19");
									break;					
							case 19:u2_printf("Cartoon_Washlet_Leg_Down_20");
									break;	
						}
					}				
				}			
			}			
			
			//����֧����������ͬʱ���ж���ָ��
			else if((1==back_state_flag)&&(1==leg_down_state_flag))      //֧����������ͬʱ����
			{				
				arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
				j=arr_send/(back_angle_to_arr(back_angle_max)/14);
				if(0==washlet_auto_dir_flag)
				{
					j=14-j;
				}
				if(	k!=j)
				{				
					kj=abs(k,j);
					if(kj<2)
					{
						k=j;
						switch (k)
						{						
							case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
									break;
							case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
									break;					
							case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
									break;					
							case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
									break;	
							case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
									break;	
							case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
									break;
							case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
									break;
							case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
									break;						
							case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
									break;												
							case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
									break;	
							case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
									break;
							case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
									break;					
							case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
									break;					
							case 14:u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
									break;																											
						}
					}				
				}								
			}										
		}
		Push_Rod_Stop();            //������ֹͣ
		TIM10_Stop();		        //�رն�ʱ��	
		//�ж������ȸ�λ
		if((washlet_auto_dir_flag==0)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=0; 
			leg_down_runed_arr=0; 
//			W25QXX_Write((u8*)&leg_down_flag,32,1);
			if(0==back_state_flag)                   //ֻ�����������У�֧��������
			{
				k=0;   leg_down_picture_k=0;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_1");
			}
		}
		if((washlet_auto_dir_flag==1)&&(1==leg_down_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
		{
			leg_down_flag=1; 
			leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim); 
//			W25QXX_Write((u8*)&leg_down_flag,32,1);
			if(0==back_state_flag)                   //ֻ�����������У�֧��������
			{
				k=19;   leg_down_picture_k=19;
				delay_ms(200);
				u2_printf("Cartoon_Washlet_Leg_Down_20");
			}
		}
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		
		//��������֧����֧����������ͬʱ��������
		if(back_state_flag==1)              //��������֧����֧����������ͬʱ��������
		{				
			TIM10_Init(back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim),timer10_freq);
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);          //����жϱ�־λ
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) ) //֧����ʱʱ�䵽
			{	
				 //�ж���û���յ����ָ��
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
				 if(break_flag==1)
				 {
					u2_printf("break_flag==1");
					break;
				 }
				 //��������֧�����ж���ָ��
				 if(0==leg_down_state_flag)        //ֻ��֧������
				 {
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/19);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/19);
					 }
					 k=back_picture_k;
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;   back_picture_k=k;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_2");							
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_3");											
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_4");											
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_5");											
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_6");											
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_7");											
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_8");										
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_9");											
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_10");											
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_11");											
										break;
								case 11:u2_printf("Cartoon_Washlet_Back_12");																		
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_13");											
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_14");											
										break;					
								case 14:u2_printf("Cartoon_Washlet_Back_15");										
										break;	
								case 15:u2_printf("Cartoon_Washlet_Back_16");																		
										break;
								case 16:u2_printf("Cartoon_Washlet_Back_17");											
										break;					
								case 17:u2_printf("Cartoon_Washlet_Back_18");											
										break;					
								case 18:u2_printf("Cartoon_Washlet_Back_19");										
										break;																																			
							}
						}				
					 }						
				 }
				 //�������������ȡ�֧��ͬʱ���еĶ���ָ��
				 if(1==leg_down_state_flag)       //֧�������ȶ�����
				 { 
					 arr_send=__HAL_TIM_GET_COUNTER(&TIM10_Handler);
					 if(washlet_auto_dir_flag==1)
					 {
						 j=(leg_angle_to_arr(leg_down_angle_lim)+arr_send)/(back_angle_to_arr(back_angle_max)/14);
					 }
					 else
					 {
						 arr_send=back_angle_to_arr(back_angle_lim)-leg_angle_to_arr(leg_down_angle_lim)-arr_send;
						 j=arr_send/(back_angle_to_arr(back_angle_max)/14);
					 }
					 if(k!=j)
					 {				
						kj=abs(k,j);
						if(kj<2)
						{
							k=j;
							switch (k)
							{						
								case 1:	u2_printf("Cartoon_Washlet_Back_Leg_Down_2");	
										break;
								case 2:	u2_printf("Cartoon_Washlet_Back_Leg_Down_3");
										break;					
								case 3:	u2_printf("Cartoon_Washlet_Back_Leg_Down_4");
										break;					
								case 4:	u2_printf("Cartoon_Washlet_Back_Leg_Down_5");
										break;	
								case 5:	u2_printf("Cartoon_Washlet_Back_Leg_Down_6");
										break;	
								case 6:	u2_printf("Cartoon_Washlet_Back_Leg_Down_7");
										break;
								case 7:	u2_printf("Cartoon_Washlet_Back_Leg_Down_8");
										break;
								case 8:	u2_printf("Cartoon_Washlet_Back_Leg_Down_9");
										break;						
								case 9:	u2_printf("Cartoon_Washlet_Back_Leg_Down_10");
										break;												
								case 10:u2_printf("Cartoon_Washlet_Back_Leg_Down_11");
										break;	
								case 11:u2_printf("Cartoon_Washlet_Back_Leg_Down_12");	
										break;
								case 12:u2_printf("Cartoon_Washlet_Back_Leg_Down_13");
										break;					
								case 13:u2_printf("Cartoon_Washlet_Back_Leg_Down_14");
										break;																																									
							}
						}				
					}	
				}						 
			}
			Motor_1_STOP();                                             //֧��ֹͣ
			TIM10_Stop();                                               //�رն�ʱ��
			//�ж�֧����λ
			if((washlet_auto_dir_flag==0)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=0; 
				back_runed_arr=0;
//				W25QXX_Write((u8*)&back_flag,30,1);
				if(0==leg_down_state_flag)         //ֻ��֧������
				{
					k=0;   back_picture_k=0;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_1");
				}
				if(1==leg_down_state_flag)         //֧��������ͬʱ����
				{
					k=0;   
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_1");
				}
			}
			if((washlet_auto_dir_flag==1)&&(1==back_state_flag)&&(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
			{
				back_flag=1; 
				back_runed_arr=back_angle_to_arr(back_angle_lim);
				if(0==leg_down_state_flag)         //ֻ��֧������
				{
					k=19;   back_picture_k=19;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_20");
				}
				if(1==leg_down_state_flag)         //֧��������ͬʱ����
				{
					k=14;
					delay_ms(200);
					u2_printf("Cartoon_Washlet_Back_Leg_Down_15");
				}
			}
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		}				
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

/***********************************************************************
 ������      ��Washlet_Weight(void)   
 ��������    ������ִ���Զ�����������
 ����        ����
 ���        ���� 
                          
************************************************************************/
u8  Uart_Washlet_Weight(void)
{
	u8 m=0,i=0;
	while(1)
	{
		TIM10_Init(5000-1,timer10_freq);      //�򿪶�ʱ��30S
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
		{
			if(Uart_Weight())                //�������ڱ仯����ѭ�������¿�ʼ��ʱ
			{  
				m++;
				switch(m)
				{
					case 1:u2_printf("Cartoon_Washlet_Weight_1");									
							break;
					case 2:u2_printf("Cartoon_Washlet_Weight_2");									
							break;
					case 3:u2_printf("Cartoon_Washlet_Weight_3");									
							break;
					case 4:u2_printf("Cartoon_Washlet_Weight_4");									
							break;
					case 5:u2_printf("Cartoon_Washlet_Weight_5");									
							break;
					case 6:u2_printf("Cartoon_Washlet_Weight_6");									
							break;
					case 7:u2_printf("Cartoon_Washlet_Weight_7");									
							break;
					case 8:u2_printf("Cartoon_Washlet_Weight_8");									
							break;
					case 9:u2_printf("Cartoon_Washlet_Weight_9");									
							break;
					case 10:u2_printf("Cartoon_Washlet_Weight_10");									
							break;
					case 11:u2_printf("Cartoon_Washlet_Weight_11");									
							break;
					case 12:u2_printf("Cartoon_Washlet_Weight_12");									
							break;
					case 13:u2_printf("Cartoon_Washlet_Weight_13");									
							break;
					case 14:u2_printf("Cartoon_Washlet_Weight_14");									
							break;
					case 15:u2_printf("Cartoon_Washlet_Weight_15");									
							break;
				}				
			}
		}
		//���30S��ʱ�䵽������û�з����仯
		if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(0==Uart_Weight())) 
		{ 				
			i++; 
			u2_printf("����i=%d",i);
		}
		else { i=0; u2_printf("����i=%d\r\n",i);}                    //�������¼�ʱ
		if(i==2)                         //����������û�з����仯����ʼ��ϴ���
		{
			u2_printf("Washlet_Over");   //�ű����
			u1=0;
			u2=0;
			u3=0;
			break;
		}
	}	   	 		
}

/***********************************************************************
 ������      ��Uart_Swash_Dry(void)   
 ��������    ������ִ�г�ϴ��ɹ���
 ����        ����
 ���        ����  
                          
************************************************************************/
void Uart_Swash_Dry()
{
	u8 num,len;    //�����ַ�������
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	USART2_RX_LEN=0;
	
    //�������ܣ�ֻ���������ʱ���ܽ����������
	if((lock_flag==1)&&(1==washlet_flag))		
	{
		//Uart_Push_Rod_Swash_Dry(1,29000);      //��ϴ����Ƹ����  
		u2_printf("�Ƹ����\r\n");	
		RELAY6=1;
		Uart_Push_Rod_Swash(1,30000);		
		RELAY6=0;
		delay_ms(50);
		
/********************��ʼ��ϴ**********************/
		u2_printf("��ʼ��ϴ\r\n");	
    Uart_Swash_Auto();                               //�Զ���ϴ���
		u2_printf("������ϴ\r\n");	
		//�Զ���ϴ�������ȴ�30S���ٴΰ��³�ϴ�����������ֶ����ڳ�ϴ
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);     //��ս��ռĴ���
		USART2_RX_LEN=0;
		TIM9_Init(4000,timer10_freq);                   //�򿪶�ʱ������ʱ30S60000
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
		{							
			if(USART2_RX_LEN&0x8000)
			{
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;	
				if(strstr((const char *)USART2_RX_BUF,(const char *)"SwashPhone"))   //�����յ�Stop,������ѭ��	
				{					
					if(1==Liq_Sensor)           //�㹻һ�γ�ϴ
					{						
						  Uart_Swash_Hand();
						  memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					    USART2_RX_LEN=0;
					}
					else                         //����һ�γ�ϴ
					{
						break;
					}
				}				
			}
		}
		TIM9_Stop();                                         //�رն�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);   //����жϱ�־λ
	
		//delay_ms(500);
		
/**********************************��ʼ���***********************************/
		u2_printf("��ʼ���\r\n");	
		Uart_Dry_Auto();       //�Զ����2����
		u2_printf("�������\r\n");	
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
		USART2_RX_LEN=0;
		
		//�Զ���ɽ������ȴ�30S���ٴΰ��º�ɰ����������ֶ����ں��		
		TIM9_Init(7000,timer10_freq);                             //�򿪶�ʱ������ʱ30S
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);         //����жϱ�־λ
		while(!(__HAL_TIM_GET_FLAG(&TIM9_Handler, TIM_SR_CC1IF)))  //�ȴ���ʱʱ�䵽
		{
			if(USART2_RX_LEN&0x8000)
			{
				len=USART2_RX_LEN&0x3fff;				
				USART2_RX_BUF[len]=0;
				if(strstr((const char *)USART2_RX_BUF,(const char *)"DryPhone"))    //�����յ�Stop,������ѭ��	
				{
					Uart_Dry_Hand();
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
					USART2_RX_LEN=0;
				}				
			}
		}
		
		TIM9_Stop();                                        //�رն�ʱ��
		__HAL_TIM_CLEAR_FLAG(&TIM9_Handler, TIM_SR_CC1IF);  //����жϱ�־λ		
		delay_ms(500);
		//Uart_Push_Rod_Swash_Dry(0,2000+swash_dry_runed_pulse);                    //��ϴ����Ƹ�����		
				RELAY6=1;
		Uart_Push_Rod_Swash(0,30000);
		RELAY6=0;		
		u2_printf("�Ƹ�����\r\n");	
	}
	else
	{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;
	}		
}
			
/***********************************************************************
 ������      ��Uart_Swash_Hand(void)   
 ��������    ������ִ�г�ϴ����
 ����        ����
 ���        ����  
                          
************************************************************************/
void Uart_Swash_Hand(void)
{
	u8 len;              //WiFi�����ַ�������
	u8 direct;           //�����־λ
	u8 i;
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	USART2_RX_LEN=0;
	
    //�������ܣ�ֻ���������ʱ���ܽ�����ˮ��ϴ
	if((lock_flag==1)&&(1==washlet_flag))		
	{								
		//��ˮ��ϴ
		RELAY6=1;             //�̵����õ�
		DIR_SB=1;             //ˮ�ÿ���PB12	
		swash_hand_flag=1;
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		for(i=0;i<2*swash_dry_time;i++)                                //��ϴ2*swash_dry_time����
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //�򿪶�ʱ��,��ʱ������Ϊ30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{ 					
				if(0==Liq_Sensor)	                                   //ˮλ�ڵ�ˮλ�£�����һ�γ�ϴ����ֱ������
				{ 
					u2_printf("LiquidLevellow");                       //���͸���λ��ָ���źţ���ʾ��ʱˮλƫ��
					break;
				}				
				if(USART2_RX_LEN&0x8000)
				{
					len=USART2_RX_LEN&0x3fff;				
					USART2_RX_BUF[len]=0;
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))    //�Ƹ����	
					{
						direct=1;
						Uart_Push_Rod_Swash(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone"))  //�Ƹ�����	
					{ 
						direct=0;
						Uart_Push_Rod_Swash(0,swash_dry_pulse_lim); 					
					}
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
					USART2_RX_LEN=0;	
				}	
			}			
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ   		
			TIM10_Stop();		                                //�رն�ʱ��				
		}
		swash_hand_flag=0;
		DIR_SB=0;             //ˮ�ùر�PB12
		if(swash_dry_runed_pulse>0)
		{
			direct=0;
			Uart_Push_Rod_Swash(0,swash_dry_runed_pulse); 
		}
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Swash_1");
		}
		RELAY6=0;        //�̵����ϵ�
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

/***********************************************************************
 ������      ��Uart_Dry_Hand(void)   
 ��������    ������ִ�к�ɹ���
 ����        ����
 ���        ����  
                          
************************************************************************/
void Uart_Dry_Hand(void)
{
	u8 len,i;
	u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������         
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
	USART2_RX_LEN=0;	
    //�������ܣ�ֻ���������ʱ���ܽ����������
	if((lock_flag==1)&&(1==washlet_flag))		
	{	
		//�������
		DIR_HG=1;             //����������Ŵ�PB10
		delay_ms(500);       //�ȴ����Ŵ�1S
		RELAY6=1;             //�̵����õ�
		DIR_QB=1;             //��������PH2
		dry_hand_flag=1;
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}		
		for(i=0;i<2*swash_dry_time;i++)      //���swash_dry_time����
		{
			TIM10_Init(60000-1,timer10_freq_1);                        //�򿪶�ʱ��,��ʱ������Ϊ30S					
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);        //����жϱ�־λ 
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))) //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			{ 					
				if(USART2_RX_LEN&0x8000)
				{
					len=USART2_RX_LEN&0x3fff;				
					USART2_RX_BUF[len]=0;
					//�ж�������
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodUpPhone"))    //�Ƹ����	
					{
						direct=1;
						Uart_Push_Rod_Dry(1,swash_dry_pulse_lim); 
					}
					if(strstr((const char *)USART2_RX_BUF,(const char *)"PushRodDown_hone"))  //�Ƹ�����	
					{ 
						direct=0;
						Uart_Push_Rod_Dry(0,swash_dry_pulse_lim); 					
					}
					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
					USART2_RX_LEN=0;
				}				
			}              		
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ   		
			TIM10_Stop();   //�رն�ʱ��		 	
		}
		dry_hand_flag=0;		
		if(swash_dry_runed_pulse>0)
		{
			direct=0;
			Uart_Push_Rod_Dry(0,swash_dry_runed_pulse); 
		}		
		if(0==swash_dry_runed_pulse)
		{
			u2_printf("Cartoon_Push_Rod_Dry_1");
		}
		RELAY6=0;       //�̵����ϵ�
		DIR_QB=0;       //���ùر�PH2
		DIR_HG=0;       //����������ر�PB10		
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

/***********************************************************************
 ������      ��Uart_Swash_Auto(void)   
 ��������    ������ִ�г�ϴ����
 ����        ����
 ���        ����  
                          
************************************************************************/
void Uart_Swash_Auto(void)
{
	u8 direct;   //����ûĳ���������еķ����־��1-�������У�0-��������
	u8 flag=1;   //�����Ƹ˷����л�
	u8 i;
	u8 num,len;
	
    //�������ܣ�ֻ���������ʱ���ܽ�����ˮ��ϴ
	if((lock_flag==1)&&(1==washlet_flag))		
	{						
//		//��ϴ֮ǰ���ˮλ��������ˮλ���£����������������ȴ�ˮ��עˮ
//		if(0==Liq_Sensor)  
//		{
//			delay_ms(100);
//			u2_printf("LiquidLevellow");  //���͸���λ��ָ���źţ���ʾ��ʱˮλƫ��
//		}
//		while(0==Liq_Sensor)                //ˮ��עˮ�󣬲��ܼ�������ִ��
//		{
//			PCF8574_WriteBit(BEEP_IO,0);	//���Ʒ���������	
//			if(USART2_RX_LEN&0x8000)
//			{
//				len=USART2_RX_LEN&0x3fff;				
//				USART2_RX_BUF[len]=0;
//				if(strstr((const char *)USART2_RX_BUF,(const char *)"BeepOff"))
//				{
//					PCF8574_WriteBit(BEEP_IO,1);                     //������ֹͣ����
//					memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);     //��ս��ռĴ���
//					USART2_RX_LEN=0;
//					break;
//				}
//			}		
//		}
////		while(0==Liq_Sensor);                //�ȴ�ˮ��ע��
//		PCF8574_WriteBit(BEEP_IO,1);
		RELAY6=1;                            //�̵����õ�
		//��ˮ��ϴ
		DIR_SB=1;                            //ˮ�ÿ���PB12	
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Swash_1");
//		}
//		for(i=0;i<2*swash_dry_time;i++)     //��ϴ����Ƹ��Զ�ѭ����ϴswash_dry_time����
//		{
//			flag=!flag;
//			Uart_Push_Rod_Swash(flag,30000);      //ÿ���������5S��
//			delay_ms(10);			
//		}
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Swash_1");
//		}
		delay_ms(5000);
		delay_ms(4000);
		RELAY6=0;             //�̵����ϵ�
		DIR_SB=0;             //ˮ�ùر�PB12		
	}
	else
	{
		LED0=0;               //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Dry_Auto(void)   
 ��������    ������ִ�к�ɹ���
 ����        ����
 ���        ����  
                          
************************************************************************/
void Uart_Dry_Auto(void)
{
	u8 flag=0;        //�����Ƹ˷����л�
	u8 i;
	
    //�������ܣ�ֻ���������ʱ���ܽ����������
	if((lock_flag==1)&&(1==washlet_flag))		
	{
		//�������
		DIR_HG=1;             //����������Ŵ�PB10
		delay_ms(500);       //�ȴ����Ŵ�1S
		RELAY6=1;             //�̵����õ�
		DIR_QB=1;             //��������PH2
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Dry_1");
//		}		
//		for(i=0;i<2*swash_dry_time;i++)     //��ϴ����Ƹ��Զ�ѭ�����swash_dry_time ����
//		{
//			flag=!flag;
//			Uart_Push_Rod_Dry(flag,swash_dry_pulse_lim);        //ÿ���������5S��
//			delay_ms(50);
//		}
		
		
//		if(0==swash_dry_runed_pulse)
//		{
//			u2_printf("Cartoon_Push_Rod_Dry_1");
//		}

		delay_ms(6000);
		delay_ms(6000);
		RELAY6=0;       //�̵����ϵ�
		DIR_QB=0;       //���ùر�PH2
		DIR_HG=0;       //����������ر�PB10
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

/***********************************************************************
 ������      ��Uart_Exp_Back(void)  
 ��������    ��ר��ϵͳ����-֧��
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Exp_Back(void) 
{
	//֧����
	WriteInUART2("BackUpPhone");
	Uart_Back();                //֧��
	delay_ms(1000);	
	//֧����λ	
	WriteInUART2("BackDownPhone");
	Uart_Back();                //֧����λ	
	
}

/***********************************************************************
 ������      ��Uart_Exp_Body(void)  
 ��������    ��ר��ϵͳ����-����
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Exp_Body(void) 
{
	//����
	WriteInUART2("BodyLeftUpPhone");		
	Uart_Body_Left();           //����	
	delay_ms(1000);
	WriteInUART2("BodyLeftDownPhone");		
	Uart_Body_Left();           //����	��λ
	delay_ms(1000);
			
	//�ҷ���
	WriteInUART2("BodyRightUpPhone");
	Uart_Body_Right();          //�ҷ���
	delay_ms(1000);
	WriteInUART2("BodyRightDownPhone");
	Uart_Body_Right();          //�ҷ���λ
		
}

/***********************************************************************
 ������      ��Uart_Exp_Leg(void)  
 ��������    ��ר��ϵͳ����-����
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Exp_Leg(void) 
{
	//������
	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //������
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //�����ȸ�λ
	delay_ms(1000);	
	
	//������
	WriteInUART2("LegDownDownPhone");
	Uart_Leg_Down();            //������	
	delay_ms(1000);
	WriteInUART2("LegDownUpPhone");
	Uart_Leg_Down();            //�����ȸ�λ
	
}

/***********************************************************************
 ������      ��Uart_Exp_Washlet_Auto(void)  
 ��������    ��ר��ϵͳ����-�Զ�����
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Exp_Washlet_Auto(void) 
{
	Uart_Washlet_Auto();	  //�Զ�����
}



/***********************************************************************
 ������      ��Uart_Auto_Arm_Leg_Left(void)  
 ��������    ��ִ����֫��������
 ����        ��t-������������
 ���        ����
                           
************************************************************************/
void Uart_Auto_Arm_Leg_Left(int t)
{
   u32 pulse;          //��������������
   int j;	
	//����ֻ��ƽ���ڴ��ϲ��ܽ��е��ҿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   { 
		
		DG_Relay=1;		//�̵����õ�
		 if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftAuto"))    //��첲
		{
			pulse=1000000;
			if(leg_fore_left_flag==0)       //��ֹ�����󴥷������±�־λ��λ
			{	
				arm_fore_left_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmLeftAutoStart");				
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftAuto"))    //����
		{
			pulse=1000000;
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
		
		Uart_Auto_Hang_1_2(1,pulse);       //����̧֫��
		delay_ms(1000);	     
		   
		for(j=0;j<t;j++)                   //����t����֫����ѵ��
		{
			Uart_Auto_Hang_1(1,75000);     //�����˶�
			delay_ms(1000);
			Uart_Auto_Hang_1(0,75000);     //�����˶�
			delay_ms(1000);	
		}	
		Uart_Auto_Hang_1_2(0,pulse);       //����֫��ƽ��ԭ����λ��
		//����־λ��λ
		if((0==arm_left_runed)&&(1==arm_fore_left_flag))         //��첲����������Ϊ�㣬��λ����ʼ״̬
		{
			arm_fore_left_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("ArmLeftAutoRes");
			delay_ms(200);
		}
		if((0==leg_left_runed)&&(1==leg_fore_left_flag))         //��������������Ϊ�㣬��λ����ʼ״̬
		{
			leg_fore_left_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("LegLeftAutoRes");
			delay_ms(200);
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}		
}

/***********************************************************************
 ������      ��Uart_Auto_Arm_Leg_Right(void)   
 ��������    ��ִ����֫���ҿ���ѵ��
 ����        ��t���Ҵ���
 ���        ���� 
                          
************************************************************************/
void Uart_Auto_Arm_Leg_Right(int t)
{
	u32 pulse;     //��������������
	//����ֻ��ƽ���ڴ��ϲ��ܽ��е��ҿ���ѵ��
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{
		DG_Relay=1;		//�̵����õ�
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmRightAuto"))    //�Ҹ첲
		{
			pulse=1000000;
			if(leg_fore_right_flag==0)          //��ֹ�����󴥷������±�־λ��λ
			{	
				arm_fore_right_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmRightAutoStart");
				delay_ms(200);
			}			
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegRightAuto"))    //����
		{
			pulse=1000000;
			if(arm_fore_right_flag==0)          //��ֹ�����󴥷������±�־λ��λ
			{	
				leg_fore_right_flag=1;
				delay_ms(200);				
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegRightAutoStart");	
				delay_ms(200);
			}			
		}
	
		int j;
		Uart_Auto_Hang_3_4(1,pulse);       //����̧֫��
		delay_ms(1000);
		
		for(j=0;j<t;j++)                   //����t����֫����ѵ��
		{		
			Uart_Auto_Hang_3(1,75000);     //�����˶�		
			delay_ms(1000);		
			Uart_Auto_Hang_3(0,75000);     //�����˶�
			delay_ms(1000);		
		}	
		Uart_Auto_Hang_3_4(0,pulse);       //����֫��ƽ��ԭ����λ��
		//����־λ��λ
		if((0==arm_right_runed)&&(1==arm_fore_right_flag))   //��첲����������Ϊ�㣬��λ����ʼ״̬
		{
			arm_fore_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);			
			u2_printf("ArmRightAutoRes");
			delay_ms(200);
		}
		if((0==leg_right_runed)&&(1==leg_fore_right_flag))   //��������������Ϊ�㣬��λ����ʼ״̬
		{
			leg_fore_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("LegRightAutoRes");
			delay_ms(200);
		}	
			DG_Relay=0;		//�̵���ʧ��
	}
	else
	 {
		LED0=0;                 //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		 
		LED0=1;
		LED1=1;
	 }	
}

/***********************************************************************
 ������      ��Uart_Auto_Arm_Leg_Left_Right(void)   
 ��������    ��ִ������֫���ҿ���ѵ��
 ����        ��t���Ҵ���
 ���        ����  
                          
***********************************************************************/
void Uart_Auto_Arm_Leg_Left_Right(int t)
{
	u32 pulse;	         //��������������
	int j;
	//����ֻ��ƽ���ڴ��ϲ��ܽ��е��ҿ���ѵ��
	if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
	{
		DG_Relay=1;		//�̵����õ�
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftRightAuto"))    //���Ҹ첲
		{
			pulse=1000000;
			if(leg_fore_left_right_flag==0)       //��ֹ�����󴥷������±�־λ��λ
			{	
				arm_fore_left_right_flag=1;	
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmLeftRightAutoStart");	
				delay_ms(200);				
			}
			else
			{
				arm_fore_left_right_flag=0;	
			}
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftRightAuto"))    //������
		{
			pulse=1000000;
			if(arm_fore_left_right_flag==0)      //��ֹ�����󴥷������±�־λ��λ
			{	
				leg_fore_left_right_flag=1;
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegLeftRightAutoStart");	
				delay_ms(200);				
			}
			else
			{
				leg_fore_left_right_flag=0;	
			}
		}	
		Uart_Auto_Hang_1_2_3_4(1,pulse);     //������̧֫�ߣ�С��̧�ñȴ�۸�
		delay_ms(1000);	     
			   
		for(j=0;j<t;j++)                     //����t�ο���ѵ��
		{
			LED1=0;                          //�ڿ���ѵ��������LED0��˸
			Uart_Auto_Hang_1_3(1,75000);     //С������һ���߶�
			LED1=1;
			delay_ms(1000);
			LED1=0;
			Uart_Auto_Hang_1_3(0,75000);     //С���½�һ���߶�
			delay_ms(1000);
			LED1=1;	
		}
			Uart_Auto_Hang_1_2_3_4(0,pulse); //������֫��ƽ��ԭ����λ��
		//����־λ��λ
		if((0==arm_left_right_runed)&&(1==arm_fore_left_right_flag))    //��첲����������Ϊ�㣬��λ����ʼ״̬
		{
			arm_fore_left_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("ArmLeftRightAutoRes");
			delay_ms(200);
		}
		if((0==leg_left_right_runed)&&(1==leg_fore_left_right_flag))    //��������������Ϊ�㣬��λ����ʼ״̬
		{
			leg_fore_left_right_flag=0;
			delay_ms(200);
			u2_printf("RunRes");
			delay_ms(200);
			u2_printf("LegLeftRightAutoRes");
			delay_ms(200);
		}	
			DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;               //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Fore_Left(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Fore_Left(void)
{
	 u8 direct;   //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag=0;
	//��������ֻ���ڼ����򿪡�֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 if(0==arm_fore_left_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmForeLeftHandStart");
				delay_ms(200);
				arm_fore_left_flag=1;
				Uart_Hand_Hang_1_2(1,arm_left_lim);  //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}				
		//��С����������
		if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(arm_fore_left_lim>arm_fore_left_runed)
			{
				Uart_Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					arm_fore_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForeLeftHandLim");
					delay_ms(200);
				}
				else
				{
					arm_fore_left_flag=1;		
				}								
			}	
		}
		//��С����������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftDownHand"))
		{	
			if(arm_fore_left_runed>0)
			{
				Uart_Hand_Hang_1(0,arm_fore_left_runed);					
			}
			if(0==arm_fore_left_runed)               //��С�۸�λ���򽫸첲��λ
			{
				Uart_Hand_Hang_1_2(0,arm_left_runed);     //����֫��λ��ˮƽ״̬ 
			}
			if((0==arm_fore_left_runed)&&(0==arm_left_runed))
			{
				arm_fore_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForeLeftHandRes");
				delay_ms(200);
			}
		}	
			DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Leg_Fore_Left(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Fore_Left(void)
{
	 u8 direct;   //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag=0;
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 if(0==leg_fore_left_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegForeLeftHandStart");
				delay_ms(200);
				leg_fore_left_flag=1;
				Uart_Hand_Hang_1_2(1,leg_left_lim);      //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}
		if((strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))||(1==runed_flag))
		{	
			runed_flag=0;
			if(leg_fore_left_lim>leg_fore_left_runed)
			{
				Uart_Hand_Hang_1(1,leg_fore_left_lim-leg_fore_left_runed);	
				if(leg_fore_left_runed==leg_fore_left_lim)
				{
					leg_fore_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForeLeftHandLim");
					delay_ms(200);
				}
				else
				{
					leg_fore_left_flag=1;		
				}								
			}	
		}
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftDownHand"))
		{	
			if(leg_fore_left_runed>0)
			{
				Uart_Hand_Hang_1(0,leg_fore_left_runed);					
			}
			if(0==leg_fore_left_runed)             //��С������������Ϊ0���򽫸첲��λ
			{
				Uart_Hand_Hang_1_2(0,leg_left_runed);   //����֫��λ��ˮƽ״̬ 
			}
			if((0==leg_fore_left_runed)&&(0==leg_left_runed))
			{
				leg_fore_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForeLeftHandRes");
				delay_ms(200);
			}			
		}	
			DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Fore_Right(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Fore_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag=0;
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 if(0==arm_fore_right_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmForeRightHandStart");
				delay_ms(200);
				arm_fore_right_flag=1;
				Uart_Hand_Hang_3_4(1,arm_right_lim);       //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}
		//��С����������
		if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))||(1==runed_flag))		
		{
			runed_flag=0;
			if(arm_fore_right_lim>arm_fore_right_runed)
			{
				Uart_Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);	
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					arm_fore_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForeRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_fore_right_flag=1;						
				}								
			}	
		}			
		//��С����������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightDownHand"))
		{
			if(arm_fore_right_runed>0)    //��С�������˶�
			{
				Uart_Hand_Hang_3(0,arm_fore_right_runed);					
			}
			if(arm_fore_right_runed==0)    //����С�۸�λ�����Ҹ첲��λ��ˮƽλ��
			{
				Uart_Hand_Hang_3_4(0,arm_right_runed); 
			}
			if((arm_fore_right_runed==0)&&(0==arm_right_runed))   //��־λ��λ
			{
				arm_fore_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForeRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Leg_Fore_Right(void)  
 ��������    ���ֶ�ִ����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Fore_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag;
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ���С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		 DG_Relay=1;		//�̵����õ�
		 if(0==leg_fore_right_flag)                    //�Ƚ���̧֫�ߵ�һ���߶�
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegForeRightHandStart");
				delay_ms(200);
				leg_fore_right_flag=1;
				Uart_Hand_Hang_3_4(1,leg_right_lim);       //����̧֫�ߵ�һ���߶ȣ���ʼС�ۿ���ѵ��
				runed_flag=1;
			}
		}
		//��С������
	   if((strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))||(runed_flag==1))
		{
			runed_flag=0;
			if(leg_fore_right_lim>leg_fore_right_runed)
			{
				Uart_Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);	
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					leg_fore_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForeRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_fore_right_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightDownHand"))
		{
			if(leg_fore_right_runed>0)     //��С����������
			{
				Uart_Hand_Hang_3(0,leg_fore_right_runed);					
			}
			if(0==leg_fore_right_runed)    //����С�ȸ�λ������֫��ƽ��ˮƽλ��
			{
				Uart_Hand_Hang_3_4(0,leg_right_runed);
			}
			if((0==leg_fore_right_runed)&&(0==leg_right_runed))  //��־λ��λ
			{
				leg_fore_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForeRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Post_Left(void)  
 ��������    ���ֶ�ִ������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Post_Left(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //��������		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftUpHand"))
		{
			if(arm_post_left_lim>arm_post_left_runed)
			{
			    if(0==arm_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmPostLeftHandStart");
					delay_ms(200);
				}
				arm_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_post_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmPostLeftHandLim");
					delay_ms(200);
				}
				else
				{
					arm_post_left_flag=1;						
				}								
			}	
		}
         //��������		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftDownHand"))
		{
			if(arm_post_left_runed>0)
			{
				Uart_Hand_Hang_1_2(0,arm_post_left_runed);
				if(0==arm_post_left_runed)
				{					
					arm_post_left_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("ArmPostLeftHandRes");
					delay_ms(200);
				}
				else
				{
					arm_post_left_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_ArmLeg_Post_Left(void)  
 ��������    ���ֶ�ִ�������
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Post_Left(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //��������		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftUpHand"))
		{
			if(leg_post_left_lim>leg_post_left_runed)
			{
				if(0==leg_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegPostLeftHandStart");
					delay_ms(200);
				}
				leg_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_post_left_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegPostLeftHandLim");
					delay_ms(200);
				}
				else
				{
					leg_post_left_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftDownHand"))
		{
			if(leg_post_left_runed>0)
			{
				Uart_Hand_Hang_1_2(0,leg_post_left_runed);
				if(0==leg_post_left_runed)
				{					
					leg_post_left_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostLeftHandRes");
					delay_ms(200);
				}
				else
				{
					leg_post_left_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Post_Right(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightUpHand"))
		{
			if(arm_post_right_lim>arm_post_right_runed)
			{
				if(0==arm_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmPostRightHandStart");
					delay_ms(200);
				}
				arm_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_post_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmPostRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_post_right_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightDownHand"))
		{
			if(arm_post_right_runed>0)
			{
				Uart_Hand_Hang_3_4(0,arm_post_right_runed);
				if(0==arm_post_right_runed)
				{
					arm_post_right_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("ArmPostRightHandRes");
					delay_ms(200);
				}
				else
				{
					arm_post_right_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Leg_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ���
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Post_Right(void)
{
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightUpHand"))
		{
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostRightHandStart");
					delay_ms(200);
				}
				leg_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_post_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegPostRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_post_right_flag=1;					
				}								
			}	
		}
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightDownHand"))
		{
			if(leg_post_right_runed>0)
			{
				Uart_Hand_Hang_3_4(0,leg_post_right_runed);
				if(0==leg_post_right_runed)
				{
					leg_post_right_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostRightHandRes");
					delay_ms(200);
				}
				else
				{
					leg_post_right_flag=1;
				}					
			}			
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Fore_Post_Left(void)  
 ��������    ���ֶ�ִ�����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Fore_Post_Left(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
     DG_Relay=1;		//�̵����õ�
		 //��������		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))
		{
			 //����������
			if(arm_post_left_lim>arm_post_left_runed)
			{
				if(0==arm_fore_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmForePostLeftHandStart");
					delay_ms(200);
				}
				arm_fore_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,arm_post_left_lim-arm_post_left_runed);	
				if(arm_post_left_runed==arm_post_left_lim)
				{
					arm_fore_post_left_flag=1;					
				}
				else
				{
					arm_fore_post_left_flag=1;						
				}								
			}
			//��������С��
			if((arm_fore_left_lim>arm_fore_left_runed)&&(arm_post_left_runed==arm_post_left_lim))  
			{
				Uart_Hand_Hang_1(1,arm_fore_left_lim-arm_fore_left_runed);	
				if(arm_fore_left_runed==arm_fore_left_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForePostLeftHandLim");
					delay_ms(200);
				}
			}			
		}	
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))
		{
			//��������С��
			if(arm_fore_left_runed>0)   
			{
				Uart_Hand_Hang_1(0,arm_fore_left_runed);					
			}
			//����������
			if((arm_post_left_runed>0)&&(0==arm_fore_left_runed))
			{
				Uart_Hand_Hang_1_2(0,arm_post_left_runed);	
			}
			if((0==arm_post_left_runed)&&(0==arm_fore_left_runed))
			{
				arm_fore_post_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForePostLeftHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_ArmLeg_Fore_Post_Left(void)  
 ��������    ���ֶ�ִ�����С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Fore_Post_Left(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //��������		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))
		{
			 //�����������
			if(leg_post_left_lim>leg_post_left_runed)
			{
				if(0==leg_fore_post_left_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegForePostLeftHandStart");
					delay_ms(200);
				}
				leg_fore_post_left_flag=1;
				Uart_Hand_Hang_1_2(1,leg_post_left_lim-leg_post_left_runed);	
				if(leg_post_left_runed==leg_post_left_lim)
				{
					leg_fore_post_left_flag=1;
				}
				else
				{
					leg_fore_post_left_flag=1;						
				}								
			}
			//��������С��
			if((leg_fore_left_lim>leg_fore_left_runed)&&(leg_post_left_runed==leg_post_left_lim))  
			{
				Uart_Hand_Hang_1(1,leg_fore_left_lim-arm_fore_left_runed);
				if(arm_fore_left_runed==leg_fore_left_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForePostLeftHandLim");
					delay_ms(200);
				}
			}							
		}
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand"))
		{
			//��������С��
			if(leg_fore_left_runed>0)   
			{
				Uart_Hand_Hang_1(0,leg_fore_left_runed);					
			}
			//�����������
			if((leg_post_left_runed>0)&&(0==leg_fore_left_runed))
			{
				Uart_Hand_Hang_1_2(0,leg_post_left_runed);	
			}			
			if((0==leg_post_left_runed)&&(0==leg_fore_left_runed))
			{
				leg_fore_post_left_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForePostLeftHandRes");
				delay_ms(200);
			}				
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Fore_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Fore_Post_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand"))
		{
			 //�������Ҵ��
			if(arm_post_right_lim>arm_post_right_runed)
			{
				if(0==arm_fore_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmForePostRightHandStart");
					delay_ms(200);
				}
				arm_fore_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,arm_post_right_lim-arm_post_right_runed);	
				if(arm_post_right_runed==arm_post_right_lim)
				{
					arm_fore_post_right_flag=1;
				}
				else
				{
					arm_fore_post_right_flag=1;						
				}								
			}
			//��������С��
			if((arm_fore_right_lim>arm_fore_right_runed)&&(arm_post_right_runed==arm_post_right_lim))  
			{
				Uart_Hand_Hang_3(1,arm_fore_right_lim-arm_fore_right_runed);
				if(arm_fore_right_runed==arm_fore_right_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForePostRightHandLim");
					delay_ms(200);
				}
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))
		{
			//��������С��
			if(arm_fore_right_runed>0)   
			{
				Uart_Hand_Hang_3(0,arm_fore_right_runed);					
			}
			//�������Ҵ��
			if((arm_post_right_runed>0)&&(0==arm_fore_right_runed))
			{
				Uart_Hand_Hang_3_4(0,arm_post_right_runed);	
			}
			if((0==arm_post_right_runed)&&(0==arm_fore_right_runed))
			{
				arm_fore_post_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForePostRightHandRes");
				delay_ms(200);
			}			
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Leg_Fore_Post_Right(void)  
 ��������    ���ֶ�ִ���Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Fore_Post_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	//��������ֻ���ڼ����򿪡���֫������֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))
		{
			 //�������Ҵ���
			if(leg_post_right_lim>leg_post_right_runed)
			{
				if(0==leg_fore_post_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegForePostRightHandStart");
					delay_ms(200);
				}
				leg_fore_post_right_flag=1;
				Uart_Hand_Hang_3_4(1,leg_post_right_lim-leg_post_right_runed);	
				if(leg_post_right_runed==leg_post_right_lim)
				{
					leg_fore_post_right_flag=1;
				}
				else
				{
					leg_fore_post_right_flag=1;						
				}								
			}
			//��������С��
			if((leg_fore_right_lim>leg_fore_right_runed)&&(leg_post_right_runed==leg_post_right_lim))  
			{
				Uart_Hand_Hang_3(1,leg_fore_right_lim-leg_fore_right_runed);
				if(leg_fore_right_runed==leg_fore_right_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForePostRightHandLim");
					delay_ms(200);
				}
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand"))
		{
			//��������С��
			if(leg_fore_right_runed>0)   
			{
				Uart_Hand_Hang_3(0,leg_fore_right_runed);					
			}
			//�������Ҵ���
			if((leg_post_right_runed>0)&&(0==leg_fore_right_runed))
			{
				Uart_Hand_Hang_3_4(0,leg_post_right_runed);	
			}
			if((0==leg_post_right_runed)&&(0==leg_fore_right_runed))
			{
				leg_fore_post_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForePostRightHandRes");
				delay_ms(200);
			}				
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");		
		LED0=1;
		LED1=1;
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Fore_Left_Right(void)  
 ��������    ���ֶ�ִ������С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Fore_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag;
	//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ�������С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 if(0==arm_fore_left_right_flag)    //�Ƚ�����̧֫��һ���߶�
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("ArmForeLeftRightHandStart");
				delay_ms(200);
				arm_fore_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,arm_left_right_lim);
				runed_flag=1;
			}
		}
		//��������
		if((strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))||(1==runed_flag))
		{	
			if(arm_fore_left_right_lim>arm_fore_left_right_runed)
			{
				runed_flag=0;
				Uart_Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);	
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					arm_fore_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForeLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_fore_left_right_flag=1;						
				}								
			}	
		}	
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))
		{
			if(arm_fore_left_right_runed>0)      //����С����������
			{
				Uart_Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			if(arm_fore_left_right_runed==0)     //������С�۸�λ��������֫��λ��ˮƽλ��
			{
				Uart_Hand_Hang_1_2_3_4(0,arm_left_right_runed);
			}
			if((arm_fore_left_right_runed==0)&&(0==arm_left_right_runed))  //��־λ��λ
			{
				arm_fore_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForeLeftRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Leg_Fore_Left_Right(void)  
 ��������    ���ֶ�ִ������С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Fore_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
	 static u8 runed_flag;
	//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ�������С��/С�ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
	   DG_Relay=1;		//�̵����õ�
		 if(0==leg_fore_left_right_flag)
		{
			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))
			{
				delay_ms(200);
				u2_printf("RunStart");
				delay_ms(200);
				u2_printf("LegForeLeftRightHandStart");
				delay_ms(200);
				leg_fore_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,leg_left_right_lim);
				runed_flag=1;
			}				
		}
		//ͬ���������ж������ۼ�
		if((strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))||(1==runed_flag))
		{
			if(leg_fore_left_right_lim>leg_fore_left_right_runed)
			{
				runed_flag=0;
				Uart_Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);	
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					leg_fore_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForeLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_fore_left_right_flag=1;						
				}								
			}	
		}
		//��������
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightDownHand"))
		{
			if(leg_fore_left_right_runed>0)    //����С����������
			{
				Uart_Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			if(leg_fore_left_right_runed==0)    //������С�ȸ�λ���������ȸ�λ��ˮƽλ��
			{
				Uart_Hand_Hang_1_2_3_4(0,leg_left_right_runed);
			}
			if((leg_fore_left_right_runed==0)&&(0==leg_left_right_runed))   //��־λ��λ
			{
				leg_fore_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForeLeftRightHandRes");
				delay_ms(200);
			}
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Post_Left_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ������Ҵ��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //ͬ���������ж������ۼ�		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))
		{
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{
				if(0==arm_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmPostLeftRightHandStart");
					delay_ms(200);
				}
				arm_post_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_post_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmPostLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					arm_post_left_right_flag=1;						
				}								
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))
		{
			if(arm_post_left_right_runed>0)
			{
				Uart_Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);
				if(0==arm_post_left_right_runed)
				{
					arm_post_left_right_flag=0;
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("ArmPostLeftRightHandRes");
					delay_ms(200);
				}
				else
				{
					arm_post_left_right_flag=1;			
				}					
			}				
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Leg_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ���
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Post_Left_Right(void)
{
	 u8 direct;    //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ������Ҵ��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //ͬ���������ж������ۼ�		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightUpHand"))
		{
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegPostLeftRightHandStart");
					delay_ms(200);
				}
				leg_post_left_right_flag=1;	
				Uart_Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_post_left_right_flag=1;
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegPostLeftRightHandLim");
					delay_ms(200);
				}
				else
				{
					leg_post_left_right_flag=1;							
				}								
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightDownHand"))
		{
			if(leg_post_left_right_runed>0)
			{
				Uart_Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);
				if(0==leg_post_left_right_runed)
				{
					leg_post_left_right_flag=0;	
					delay_ms(200);
					u2_printf("RunRes");
					delay_ms(200);
					u2_printf("LegPostLeftRightHandRes");
					delay_ms(200);
				}
				else
				{
					leg_post_left_right_flag=1;			
				}					
			}			
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;		
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Arm_Fore_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Arm_Fore_Post_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //�����˶�
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))
		{
			 //���������Ҵ��
			if(arm_post_left_right_lim>arm_post_left_right_runed)
			{
				if(0==arm_fore_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("ArmForePostLeftRightHandStart");
					delay_ms(200);
				}
				arm_fore_post_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,arm_post_left_right_lim-arm_post_left_right_runed);	
				if(arm_post_left_right_runed==arm_post_left_right_lim)
				{
					arm_fore_post_left_right_flag=1;
				}
				else
				{
					arm_fore_post_left_right_flag=1;						
				}								
			}
			//����������С��
			if((arm_fore_left_right_lim>arm_fore_left_right_runed)&&(arm_post_left_right_runed==arm_post_left_right_lim))  
			{
				Uart_Hand_Hang_1_3(1,arm_fore_left_right_lim-arm_fore_left_right_runed);
				if(arm_fore_left_right_runed==arm_fore_left_right_lim)
				{
					
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("ArmForePostLeftRightHandLim");
					delay_ms(200);
				}
			}	
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))
		{
			//����������С��
			if(arm_fore_left_right_runed>0)   
			{
				Uart_Hand_Hang_1_3(0,arm_fore_left_right_runed);					
			}
			//���������Ҵ��
			if((arm_post_left_right_runed>0)&&(0==arm_fore_left_right_runed))
			{
				Uart_Hand_Hang_1_2_3_4(0,arm_post_left_right_runed);	
			}
			if((0==arm_post_left_right_runed)&&(0==arm_fore_left_right_runed))
			{
				arm_fore_post_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("ArmForePostLeftRightHandRes");
				delay_ms(200);
			}				
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;	
	}	
}

/***********************************************************************
 ������      ��Uart_Hand_Leg_Fore_Post_Left_Right(void)  
 ��������    ���ֶ�ִ�����Ҵ�С��
 ����        ����
 ���        ����                           
************************************************************************/
void Uart_Hand_Leg_Fore_Post_Left_Right(void)
{
	 u8 direct;     //����ûĳ���������еķ����־��1-�������У�0-��������
//��������ֻ���ڼ����򿪡�����֫��ȥ��֧�������㡢����λ������²��ܽ��д��/���ȿ���ѵ��
   if((lock_flag==1)&&(0==back_flag)&&(0==leg_up_flag)&&(0==leg_down_flag)&&(0==washlet_flag)&&(0==body_left_flag)&&(0==body_right_flag))
   {
		DG_Relay=1;		//�̵����õ�
		 //ͬ���������ж������ۼ�
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))
		{
			 //���������Ҵ���
			if(leg_post_left_right_lim>leg_post_left_right_runed)
			{
				if(0==leg_fore_post_left_right_flag)
				{
					delay_ms(200);
					u2_printf("RunStart");
					delay_ms(200);
					u2_printf("LegForePostLeftRightHandStart");
					delay_ms(200);
				}
				leg_fore_post_left_right_flag=1;
				Uart_Hand_Hang_1_2_3_4(1,leg_post_left_right_lim-leg_post_left_right_runed);	
				if(leg_post_left_right_runed==leg_post_left_right_lim)
				{
					leg_fore_post_left_right_flag=1;
				}
				else
				{
					leg_fore_post_left_right_flag=1;						
				}								
			}
			//����������С��
			if((leg_fore_left_right_lim>leg_fore_left_right_runed)&&(leg_post_left_right_runed==leg_post_left_right_lim))  
			{
				Uart_Hand_Hang_1_3(1,leg_fore_left_right_lim-leg_fore_left_right_runed);
				if(leg_fore_left_right_runed==leg_fore_left_right_lim)
				{
					delay_ms(200);
					u2_printf("RunLim");
					delay_ms(200);
					u2_printf("LegForePostLeftRightHandLim");
					delay_ms(200);
				}
			}
		}			
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand"))
		{
			//����������С��
			if(leg_fore_left_right_runed>0)   
			{
				Uart_Hand_Hang_1_3(0,leg_fore_left_right_runed);					
			}
			//���������Ҵ���
			if((leg_post_left_right_runed>0)&&(0==leg_fore_left_right_runed))
			{
				Uart_Hand_Hang_1_2_3_4(0,leg_post_left_right_runed);	
			}
			if((0==leg_post_left_right_runed)&&(0==leg_fore_left_right_runed))
			{
				leg_fore_post_left_right_flag=0;
				delay_ms(200);
				u2_printf("RunRes");
				delay_ms(200);
				u2_printf("LegForePostLeftRightHandRes");
				delay_ms(200);
			}			
		}
		DG_Relay=0;		//�̵���ʧ��
	}
	else
	{
		LED0=0;             //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(100);
		u2_printf("Interfere");
		LED0=1;
		LED1=1;	
	}	
}

/***********************************************************************
 ������      ��Uart_Res_Power_Down(void)  
 ��������    �����縴λ,ÿ�����ܺ���ֹͣ��������翪��
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Res_Power_Down(void)
{	
	if(lock_flag==1)
	{
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		//��ȡ����λ״̬��־λ
		u8 	body_left_flag_buf[1];
		u8 	body_right_flag_buf[1];
		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //�ӵ�33��ַ��ʼ����ȡ1���ֽ�
		W25QXX_Read((u8*)body_right_flag_buf,34,1);    //�ӵ�34��ַ��ʼ����ȡ1���ֽ�
		body_left_flag=body_left_flag_buf[0];
		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //�󷭸�λ
		{			
			Uart_Res_Body_Left();    		
		}		
		if(1==body_right_flag)    //�ҷ���λ
		{
			Uart_Res_Body_Right();   			
		}	
		delay_ms(1000);
		
		washlet_picture_k=24;
		Uart_Washlet(1);          //��������λ       
		delay_ms(1000);
		
		Uart_Res_Leg();           //���ȸ�λ
		delay_ms(1000);
		
		Uart_Res_Back();          //֧����λ     
		delay_ms(1000);

		Uart_Res_Desk();          //�칫����һ������λ   
	}
}




void Uart_Res_Power_Down1(void)
{	
	if(lock_flag==1)
	{
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		//��ȡ����λ״̬��־λ
		u8 	body_left_flag_buf[1];
		u8 	body_right_flag_buf[1];
		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //�ӵ�33��ַ��ʼ����ȡ1���ֽ�
		W25QXX_Read((u8*)body_right_flag_buf,34,1);    //�ӵ�34��ַ��ʼ����ȡ1���ֽ�
		body_left_flag=body_left_flag_buf[0];
		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //�󷭸�λ
		{			
			Uart_Res_Body_Left();    		
		}		
		else if(1==body_right_flag)    //�ҷ���λ
		{
			Uart_Res_Body_Right();   			
		}	
		delay_ms(1000);

	
//		washlet_picture_k=24;
//		Uart_Washlet(1);          //��������λ       
//		delay_ms(1000);
		
		Uart_Res_Leg();           //���ȸ�λ
		delay_ms(1000);
		
		Uart_Res_Back();          //֧����λ     
		delay_ms(1000);


		if(GD7S==1)
		{
				Uart_Res_Desk();          //�칫����һ������λ 
		}		

		if(GD6S==1)
		{
			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,25000);  //��ϴ����Ƹ�����
			RELAY6=0;	 
			washlet_flag=0;	
			WashLet_V1(0,32950);	
		}
		
		
		
		
	}
}


void Uart_Res_Power_Down2(void)
{
	if(lock_flag==1)
	{
		memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
		USART2_RX_LEN=0;
		
		//��ȡ����λ״̬��־λ
		u8 	body_left_flag_buf[1];
		u8 	body_right_flag_buf[1];
		W25QXX_Read((u8*)body_left_flag_buf,33,1);        //�ӵ�33��ַ��ʼ����ȡ1���ֽ�
		W25QXX_Read((u8*)body_right_flag_buf,34,1);    //�ӵ�34��ַ��ʼ����ȡ1���ֽ�
		body_left_flag=body_left_flag_buf[0];
		body_right_flag=body_right_flag_buf[0];
		
		if(1==body_left_flag)     //�󷭸�λ
		{			
			Uart_Res_Body_Left();    		
		}		
		else if(1==body_right_flag)    //�ҷ���λ
		{
			Uart_Res_Body_Right();   			
		}	
		delay_ms(1000);

	
//		washlet_picture_k=24;
//		Uart_Washlet(1);          //��������λ       
//		delay_ms(1000);
		
		Uart_Res_Leg();           //���ȸ�λ
		delay_ms(1000);
		
		Uart_Res_Back();          //֧����λ     
		delay_ms(1000);


		if(GD7S==1)
		{
				Uart_Res_Desk();          //�칫����һ������λ 
		}		

		if(GD6S==1)
		{
			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,25000);  //��ϴ����Ƹ�����
			RELAY6=0;	 
			washlet_flag=0;			
			WashLet_V1(0,32950);	
		}
	}
}



/***********************************************************************
 ������      ��Uart_Res_Back(void)  
 ��������    ��֧����λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Res_Back(void)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	back_runed_arr=back_angle_to_arr(back_angle_lim);
	back_picture_k=19;
	WriteInUART2("BackDownPhone");
	Uart_Back();                //֧����λ		
}

/***********************************************************************
 ������      ��Uart_Res_Leg(void)  
 ��������    �����ȸ�λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Res_Leg(void)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	leg_down_runed_arr=leg_angle_to_arr(leg_down_angle_lim);  //����������
	leg_down_picture_k=19;
	WriteInUART2("LegDownUpPhone");
	Uart_Leg_Down();           
	delay_ms(1000);	

	WriteInUART2("LegUpUpPhone");
	Uart_Leg_Up();              //������
	delay_ms(1000);
	WriteInUART2("LegUpDownPhone");
	Uart_Leg_Up();              //�����ȸ�λ
	delay_ms(1000);		
}

/***********************************************************************
 ������      ��Uart_Res_Desk(void)  
 ��������    ���Ͳ�����һ������λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Res_Desk(void)
{
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0;
	desk_runed_arr=desk_distance_to_arr(desk_distance_lim);
	desk_picture_k=19;
	WriteInUART2("DeskDownPhone");
	Uart_Desk();             //С���Ӹ�λ
	delay_ms(1000);	
}

	
/***********************************************************************
 ������      ��Uart_Reset_Motor5_0(void)  
 ��������    ��5�ŵ����λ:
 ����        ������λ0;����λ1
 ���        ����
                           
************************************************************************/
void Uart_Res_Motor5(u8 dir) 
{
	u8 key;
	u16 arr_feed;      //��������е�ǰһ������������,�����жϵ��ʧ������
	u16 pulse_num=0;   //�������е�ǰһ����������������ת�������������ĵ�ǰ������������Ƚ�
	u16 num=0;
	DIR5=dir;  
	u8 direct=0;       
	Motor_5_START(motor_body_freq,motor_timer_freq);	//�������	
	TIM10_Init(20000,timer10_freq);                     //�򿪶�ʱ��
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
	{				
		//�����λ
		if((0==GD5_Start)&&(0==direct))   //������翪������ѭ�������ͣת 
		{
			delay_us(100);
			if(0==GD5_Start)
			{
				u2_printf("GD5Start");
				break;	
			}				
		}
		//�������
		if(1==Motor5_Alm)      
		{						
			delay_us(100);
			if(1==Motor5_Alm)
			{
				if(body_left_flag==1)
				{
					body_left_overload_5=1;
					u2_printf("Reset_Body_Left_Overload_5");
				}
				if(body_right_flag==1)
				{
					body_right_overload_5=1;
					u2_printf("Reset_Body_Right_Overload_5");
				}
				Uart_Breakdown_Treatment();
				break;
			}			
		}
		  //�ж���û���յ���λ��ָ��		
		if(USART2_RX_LEN&0x8000)
		{
			u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���	
			memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
			USART2_RX_LEN=0;												
		}		
		//���ʧ�����쳣ֹͣ
//		key=KEY_Scan(0);              //����ɨ��
//		if(key==Motor5_Tim_PRES)
//		{
//			num++;
//		}	
//		arr_feed=__HAL_TIM_GET_COUNTER(&TIM10_Handler); 
//		pulse_num=arr_feed/7.44*timer10_freq*0.36/(motor_body_freq*motor_timer_freq);						

//		if((abs(pulse_num,num)>150)&&pulse_num>0)
//		{
//			if(body_left_flag==1)
//			{
//				body_left_losepulse=1;
//				u2_printf("Reset_Body_Left_Losepulse_5");
//			}
//			if(body_right_flag==1)
//			{
//				body_right_losepulse=1;
//				u2_printf("Reset_Body_Right_Losepulse_5");
//			}
//			Uart_Breakdown_Treatment();
//			break;
//		}		
	}		      
	Motor_5_STOP();     //���ֹͣ
	TIM10_Stop();       //�رն�ʱ��
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	
	//����ʱʱ�䵽���ǹ��û��������������һ�ξ��룬�����翪��λ�ã���翪�ذ�װ��򿪴˶Σ�
//	if((__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF))&&(GD5_Start==1)&&(direct==0))
//	{  
//		DIR5=dir;
//		Motor_5_START(motor_body_freq,motor_timer_freq);    //�������
//		TIM10_Init(add_arr,timer10_freq);                   //�򿪶�ʱ��
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); //����жϱ�־λ	 	
//		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF)))
//		{			
//			if(0==GD5_Start)  //����ʱ������翪�أ�����ѭ�� 
//			{
//				delay_us(100);
//				if(0==GD5_Start) 
//				{
//					u2_printf("GD5Start");
//					break;	
//				}					
//			}
//		}			
//		Motor_5_STOP();     //���ֹͣ
//	    TIM10_Stop();       //�رն�ʱ��
//		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	
//	}	
}


/***********************************************************************
 ������      ��Uart_Res_Body_Left(void)  
 ��������    ������λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Res_Body_Left(void)
{		
	//4�ŵ����λ
	lock_flag==1;
	body_left_flag=1;
	back_nursing_left_flag==0;
	waist_nursing_left_dir_flag=0;
	waist_nursing_left_picture_k=19;
	body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
	Uart_Waist_Nursing_Left();    //����������λ
	
	//3�ŵ����λ
	lock_flag=1;
	body_left_flag=1;
	waist_nursing_left_flag=0;
	back_nursing_left_dir_flag=0;
	back_nursing_left_picture_k=19;
	body_left_runed_arr=body_angle_to_arr(body_left_angle_lim);
	Uart_Back_Nursing_Left();     //�󱳲�����λ
	
	//5�ŵ����λ
//	Res_Motor5(0); 
//	body_left_runed_arr=0;
//	body_left_flag=0;
//	back_nursing_left_flag=0;
//	waist_nursing_left_flag=0;
//	back_nursing_left_dir_flag=0;
//	waist_nursing_left_dir_flag=0;		
}

/***********************************************************************
 ������      ��Uart_Res_Body_Right(void)  
 ��������    ���ҷ���λ�����ڵ��縴λ
 ����        ����
 ���        ����
                           
************************************************************************/
void Uart_Res_Body_Right(void)
{
	//4�ŵ����λ
	lock_flag==1;
	body_right_flag=1;
	back_nursing_right_flag==0;
	waist_nursing_right_dir_flag=0;
	waist_nursing_right_picture_k=19;
	body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
	Uart_Waist_Nursing_Right();   //����������λ
	
	//3�ŵ����λ
	lock_flag=1;
	body_right_flag=1;
	waist_nursing_right_flag=0;
	back_nursing_right_dir_flag=0;
	back_nursing_right_picture_k=19;
	body_right_runed_arr=body_angle_to_arr(body_right_angle_lim);
	Uart_Back_Nursing_Right();    //�ұ�������λ 
	
	//5�ŵ����λ 
//	Res_Motor5(1);
//	body_right_runed_arr=0;
//	body_right_flag=0;
//	back_nursing_right_flag=0;
//	waist_nursing_right_flag=0;
//	back_nursing_right_dir_flag=0;
//	waist_nursing_right_dir_flag=0;	
}


/***********************************************************************
 ������      �� 
 ��������    �����Ҳ��Ժ���
 ����        ����
 ���        ����
***********************************************************************/                           

void  TestAll(u8 dir)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	HANG_DIR1=dir;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת 
	HANG_DIR2=!dir;
	HANG_DIR3=!dir;
	HANG_DIR4=dir;
	
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
	
    for(i=0;i<1500000;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM1=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ				 
		HANG_PWM2=flag;
		HANG_PWM3=flag;
		HANG_PWM4=flag;
	}
	HANG_PWM1=0;              //3�ŵ��ҵ����Ӧ������������� 
	HANG_PWM2=0;
	HANG_PWM2=0;
	HANG_PWM2=0;
	TIM10_Stop();             //�رն�ʱ��	
}	


/***********************************************************************
 ������      ��Hang1Test(u8 dir)
 ��������    �����Ҳ��Ժ���
 ����        ��dir��������
 ���        ����
***********************************************************************/   
void Hang1Test(u8 dir)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	HANG_DIR1=dir;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(50-1,motor_timer_freq);                       //�򿪶�ʱ��3
	
    for(i=0;i<1500000;i++)   //����ȷ��Ϊ3000000
    { 	
			while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
			flag = !flag ;
			__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
			HANG_PWM1=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ				 
	  }
	HANG_PWM1=0;              //3�ŵ��ҵ����Ӧ������������� 
	TIM10_Stop();             //�رն�ʱ��
}

void Hang2Test(u8 dir)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	HANG_DIR2=dir;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
	
    for(i=0;i<1500000;i++)  //����ȷ��Ϊ3000000
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM2=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ				 
	}
	HANG_PWM2=0;              //3�ŵ��ҵ����Ӧ������������� 
	TIM10_Stop();             //�رն�ʱ��
}

void Hang3Test(u8 dir)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	HANG_DIR3=dir;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
	
    for(i=0;i<1500000;i++)   //����ȷ��Ϊ3000000
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM3=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ				 
	}
	HANG_PWM3=0;              //3�ŵ��ҵ����Ӧ������������� 
	TIM10_Stop();             //�رն�ʱ��
}

void Hang4Test(u8 dir)
{
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	HANG_DIR4=dir;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת 
	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
	
    for(i=0;i<1500000;i++)    //����ȷ��Ϊ3000000
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM4=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ
		
	}
	HANG_PWM4=0;              //3�ŵ��ҵ����Ӧ������������� 
	TIM10_Stop();             //�رն�ʱ��
}


/***********************************************************************
 ������      ��Hang1Test(u8 dir)
 ��������    �����Ҳ��Ժ���
 ����        ��dir��������
 ���        ����
***********************************************************************/ 


void test(u8 dir)
{  
//���1
	Hang_Init();         //�������ڳ�ʼ������
	int i,flag=1;        //flagΪ����ߵͱ�־λ
	int j=0,k=0;
	HANG_DIR1=dir;       //3�ŵ��ҵ��ת��������ƣ��ߵ�ƽ��ת���͵�ƽ��ת 
	HANG_DIR2=dir;
	HANG_DIR3=dir;
	HANG_DIR4=dir;
	
 	memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
	USART2_RX_LEN=0; 
	
	TIM10_Init(motor_hang_freq,motor_timer_freq);                       //�򿪶�ʱ��3
	
    for(i=0;i<30000;i++)
    { 	
		while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) );   //�ȴ���ʱʱ�䵽��ʱ�䵽����ѭ��
		flag = !flag ;
		__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF);             //����жϱ�־λ
		HANG_PWM1=flag;               //3�ŵ��ҵ����������ڸߵ�ƽ				 
		HANG_PWM2=flag;
		HANG_PWM3=flag;
		HANG_PWM4=flag;
	}
	HANG_PWM1=0;              //3�ŵ��ҵ����Ӧ������������� 
	HANG_PWM2=0;
	HANG_PWM2=0;
	HANG_PWM2=0;
	TIM10_Stop();             //�رն�ʱ��
			
}


void FlagClear(void)
{
	back_flag=0;               //֧��
	leg_up_flag=0;             //������
	leg_down_flag=0;           //������
	body_left_flag=0;          //��
	body_right_flag=0;         //�ҷ�
	back_nursing_left_flag=0;  //�󱳲�����
	back_nursing_right_flag=0; //�ұ�������
	waist_nursing_left_flag=0; //����������
	waist_nursing_right_flag=0;//����������
	washlet_flag=0;            //������
	washlet_auto_flag=0;       //�Զ�������
	desk_flag=0;               //�Ͳ�����һ����
	jram_flag=0;               //���ⰴĦ
	swash_dry_flag=0;          //��ϴ���
	lock_flag=1;               //һ����������
	fault_flag=0;              //������ϱ�־λ

	swash_hand_flag=0;         //�ֶ���ϴ��־λ
	dry_hand_flag=0;           //�ֶ���ɱ�־λ

	//����
	armleg_left_flag=0;        //�ֶ���֫
	armleg_right_flag=0;       //�ֶ���֫
	armleg_left_right_flag=0;  //�ֶ�����֫

	arm_fore_left_flag=0;      //��С��
	leg_fore_left_flag=0;      //��С��

	arm_fore_right_flag=0;     //��С��
	leg_fore_right_flag=0;     //��С��

	arm_post_left_flag=0;      //����
	leg_post_left_flag=0;      //�����

	arm_post_right_flag=0;     //�Ҵ��
	leg_post_right_flag=0;     //�Ҵ���

	arm_fore_post_left_flag=0;       //���С��
	leg_fore_post_left_flag=0;       //���С��

	arm_fore_post_right_flag=0;      //�Ҵ�С��
	leg_fore_post_right_flag=0;      //�Ҵ�С��

	arm_fore_left_right_flag=0;      //����С��
	leg_fore_left_right_flag=0;      //����С��

	arm_post_left_right_flag=0;      //���Ҵ��
	leg_post_left_right_flag=0;      //���Ҵ���

	arm_fore_post_left_right_flag=0; //���Ҵ�С��
	leg_fore_post_left_right_flag=0; //���Ҵ�С��
}

