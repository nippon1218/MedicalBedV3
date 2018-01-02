#include "check.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "pump.h"
#include "function.h"
#include "common.h"
#include "motor.h"
#include "w25qxx.h"
#include "key.h"
#include "pcf8574.h"

/***********************************************************************
 ������      ��Breakdown_Judge(void)  
 ��������    ��������Ϻ������жϵ���Ƿ�������и������
 ����        ����
 ���        ��1�����ִ���0��δ���ִ���
                           
************************************************************************/
u8 Breakdown_Judge(void)
{		
	//֧��δ��λ������������
	if((1==back_flag)&&((1==body_left_flag)||(1==body_right_flag))) 
	{
		return 1;
	}
	//����δ��λ�����ȡ����ӡ���������������
	if(((1==body_left_flag)||(1==body_right_flag))&&((1==leg_up_flag)||(1==leg_down_flag)||(1==washlet_flag)||(1==desk_flag))) 
	{
		return 1;
	}
	//��/�ҷ���δ��λ����/����������
	if((1==body_left_flag)&&(1==body_right_flag))
	{
		return 1;
	}
	//�󱳲�����δ��λ�������������ҷ���������
	if((back_nursing_left_flag==1)&&((waist_nursing_left_flag==1)||(1==body_right_flag)))
	{
		return 1;
	}
	//����������δ��λ���󱳲������ҷ���������
	if((waist_nursing_left_flag==1)&&((back_nursing_left_flag==1)||(1==body_right_flag)))
	{
		return 1;
	}
	//�ұ�������δ��λ�������������󷭲�������
	if((back_nursing_right_flag==1)&&((waist_nursing_right_flag==1)||(1==body_left_flag)))
	{
		return 1;
	}
	//����������δ��λ���ұ��������󷭲�������
	if((waist_nursing_right_flag==1)&&((back_nursing_right_flag==1)||(1==body_left_flag)))
	{
		return 1;
	}
	else 
	{
		return 0;
	}
}

/***********************************************************************
 ������      ��Breakdown_Treatment(void)  
 ��������    ��������ϴ������������ڵ�����Ϻ󣬷�ָ�λ������
 ����        ����
 ���        ��1�������ų��󣬷���1                        
************************************************************************/
u8 Breakdown_Treatment(void)
{
	u8 len;
	Motor_All_Stop();	            //���е��ֹͣ
	delay_ms(100);
	//Wifi_Send("Fault_Warning");     //����λ���������ϱ����ź�
	u2_printf("Fault_Warning");
	PCF8574_WriteBit(BEEP_IO,0);	//���Ʒ���������������
	
/*************�жϵ�����ع���**********************************/		
	//����
	if(1==body_left_overload_3)     
	{	
		Wifi_Send("BodyLeftOverload3");	
	}
	if(1==body_left_overload_4)     
	{	
		Wifi_Send("BodyLeftOverload4");	
	}
	if(1==body_left_overload_5)     
	{	
		Wifi_Send("BodyLeftOverload5");	
	}	
	//�󱳲�����
	if(1==back_nursing_left_overload)      
	{	
		Wifi_Send("BackNursingLeftOverload");	
	}
	//����������
	if(1==waist_nursing_left_overload)     
	{	
		Wifi_Send("WaistNursingLeftOverload");		
	}	
	//�ҷ���
	if(1==body_right_overload_3)     
	{	
		Wifi_Send("BodyRightOverload3");		
	}
	if(1==body_right_overload_4)     
	{	
		Wifi_Send("BodyRightOverload4");		
	}
	if(1==body_right_overload_5)     
	{	
		Wifi_Send("BodyRightOverload5");		
	}	
	//�ұ�������
	if(1==back_nursing_right_overload)      
	{	
		Wifi_Send("BackNursingRightOverload");		
	}
	//����������
	if(1==waist_nursing_right_overload)      
	{	
		Wifi_Send("WaistNursingRightOverload");		
	}		
	 //�Զ�����
	if(1==washlet_auto_overload)   
	{	
		Wifi_Send("WashletAutoOverload");	
	}
	//С����
	if(1==desk_overload)            
	{	
		Wifi_Send("DeskOverload");	
	}
    
/*************�жϵ��ʧ������**********************************/	
		
	//����
	if(1==body_left_losepulse)     
	{	
		Wifi_Send("BodyLeftLosepulse");	
	}
	
	//�󱳲�����
	if(1==back_nursing_left_losepulse)      
	{	
		Wifi_Send("BackNursingLeftLosepulse");	
	}
	//����������
	if(1==waist_nursing_left_losepulse)     
	{	
		Wifi_Send("WaistNursingLeftLosepulse");		
	}	
	//�ҷ���
	if(1==body_right_losepulse)     
	{	
		Wifi_Send("BodyRightLosepulse");		
	}
	
	//�ұ�������
	if(1==back_nursing_right_losepulse)      
	{	
		Wifi_Send("BackNursingRightLosepulse");		
	}
	//����������
	if(1==waist_nursing_right_losepulse)      
	{	
		Wifi_Send("WaistNursingRightLosepulse");		
	}		
	 //�Զ�����
	if(1==washlet_auto_losepulse)   
	{	
		Wifi_Send("WashletAutoLosepulse");	
	}
	//С����
	if(1==desk_losepulse)            
	{	
		Wifi_Send("DeskLosepulse");	
	}
	memset(UART4_RX_BUF,0,len);   //�������
	UART4_RX_LEN=0;               //�����־λ 
	//��ͨ��ָ���ֶ���λ�����������������£������ų���ɷ��ص�ԭ������
	while(1)
	{
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BeepOff"))
			{
				PCF8574_WriteBit(BEEP_IO,1); //������ֹͣ����
			}
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BackFaultReset"))
			{
				Res_Back();     		    //֧����λ
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"LegFaultReset")) 
			{			
				Res_Leg();   		        //���ȸ�λ
			}
			if(strstr((const char *)UART4_RX_BUF,(const char *)"WashLetFaultReset"))
			{			
				Washlet(1);	               //��������λ
			}	
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftFaultReset")) 
			{			
				Res_Body_Left();         //����λ					
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightFaultReset"))
			{			
				Res_Body_Right();        //�ҷ���λ	
			}
									
			if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskFaultReset")) 
			{			
				Res_Desk();              //�Ͳ�����һ������λ
			}
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"All_Fault_Reset")) 
			{			
				Res_Power_Down();        //���й���һ��λ
			}			
		
			if(strstr((const char *)UART4_RX_BUF,(const char *)"FaultSolved"))   //�������ų�������ԭ����
			{			
				Wifi_Send("FaultSolved");
/*********************����־λ������Ϊ0***********************/
				
				//������ر�־λ			
				body_left_overload_3=0;          //����
				body_left_overload_4=0;          
				body_left_overload_5=0;          
				body_right_overload_3=0;         //�ҷ���
				body_right_overload_4=0;         
				body_right_overload_5=0;        
				washlet_auto_overload=0;         //�Զ�����
				desk_overload=0;                 //�Ͳ�����һ����
				back_nursing_left_overload=0;    //�󱳲�����
				back_nursing_right_overload=0;   //�ұ�������
				waist_nursing_left_overload=0;   //����������
				waist_nursing_right_overload=0;  //����������
				
				//���ʧ����־λ	
				body_left_losepulse=0;            //����
				body_right_losepulse=0;           //�ҷ��� 
				washlet_auto_losepulse=0;         //�Զ�����
				desk_losepulse=0;                 //�Ͳ�����һ����
				back_nursing_left_losepulse=0;    //�󱳲�����
				back_nursing_right_losepulse=0;   //�ұ�������
				waist_nursing_left_losepulse=0;   //����������
				waist_nursing_right_losepulse=0;  //����������
				
				break;
			}
			memset(UART4_RX_BUF,0,len);   //�������
			UART4_RX_LEN=0;               //�����־λ 
		}	
	}
	return 1;
}

/***********************************************************************
 ������      ��Uart_Breakdown_Treatment(void)  
 ��������    ��������ϴ������������ڵ�����Ϻ󣬷�ָ�λ������
 ����        ����
 ���        ��1�������ų��󣬷���1
                           
************************************************************************/
u8 Uart_Breakdown_Treatment(void)
{
	u8 len;
	Motor_All_Stop();	            //���е��ֹͣ
	u2_printf("Fault_Warning");     //����λ���������ϱ����ź�	
	PCF8574_WriteBit(BEEP_IO,0);	//���Ʒ���������������
								
	
/*************�жϵ�����ع���**********************************/	
		
	//����
	if(1==body_left_overload_3)     
	{	
		u2_printf("BodyLeftOverload3");	
	}
	if(1==body_left_overload_4)     
	{	
		u2_printf("BodyLeftOverload4");		
	}
	if(1==body_left_overload_5)     
	{	
		u2_printf("BodyLeftOverload5");		
	}	
	//�󱳲�����
	if(1==back_nursing_left_overload)      
	{	
		u2_printf("BackNursingLeftOverload");	
	}
	//����������
	if(1==waist_nursing_left_overload)     
	{	
		u2_printf("WaistNursingLeftOverload");		
	}	
	//�ҷ���
	if(1==body_right_overload_3)     
	{	
		u2_printf("BodyRightOverload3");		
	}
	if(1==body_right_overload_4)     
	{	
		u2_printf("BodyRightOverload4");		
	}
	if(1==body_right_overload_5)     
	{	
		u2_printf("BodyRightOverload5");		
	}	
	//�ұ�������
	if(1==back_nursing_left_overload)      
	{	
		u2_printf("BackNursingRightOverload");		
	}
	//����������
	if(1==waist_nursing_right_overload)      
	{	
		u2_printf("WaistNursingRightOverload");		
	}		
	 //�Զ�����
	if(1==washlet_auto_overload)   
	{	
		u2_printf("WashletAutoOverload");	
	}
	//С����
	if(1==desk_overload)            
	{	
		u2_printf("DeskOverload");	
	}
    
/*************�жϵ��ʧ������**********************************/	
		
	//����
	if(1==body_left_losepulse)     
	{	
		u2_printf("BodyLeftLosepulse");	
	}
	
	//�󱳲�����
	if(1==back_nursing_left_losepulse)      
	{	
		u2_printf("BackNursingLeftLosepulse");	
	}
	//����������
	if(1==waist_nursing_left_losepulse)     
	{	
		u2_printf("WaistNursingLeftLosepulse");		
	}	
	//�ҷ���
	if(1==body_right_losepulse)     
	{	
		u2_printf("BodyRightLosepulse");		
	}
	
	//�ұ�������
	if(1==back_nursing_right_losepulse)      
	{	
		u2_printf("BackNursingRightLosepulse");		
	}
	//����������
	if(1==waist_nursing_right_losepulse)      
	{	
		u2_printf("WaistNursingRightLosepulse");		
	}		
	 //�Զ�����
	if(1==washlet_auto_losepulse)   
	{	
		u2_printf("WashletAutoLosepulse");	
	}
	//С����
	if(1==desk_losepulse)            
	{	
		u2_printf("DeskLosepulse");	
	}
	memset(USART2_RX_BUF,0,len);   //�������
	USART2_RX_LEN=0;               //�����־λ 
	//��ͨ��ָ���ֶ���λ�����������������£������ų���ɷ��ص�ԭ������
	while(1)
	{
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BeepOff"))
			{
				PCF8574_WriteBit(BEEP_IO,1); //������ֹͣ����
			}
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BackFaultReset"))
			{
				Res_Back();     		    //֧����λ
			}

			if(strstr((const char *)USART2_RX_BUF,(const char *)"LegFaultReset")) 
			{			
				Res_Leg();   		        //���ȸ�λ
			}
			if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetFaultReset"))
			{			
				Washlet(1);	               //��������λ
			}	
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftFaultReset")) 
			{			
				Res_Body_Left();         //����λ					
			}

			if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightFaultReset"))
			{			
				Res_Body_Right();        //�ҷ���λ	
			}						
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskFaultReset")) 
			{			
				Res_Desk();              //�Ͳ�����һ������λ
			}
			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"All_Fault_Reset")) 
			{			
				Res_Power_Down();        //���й���һ��λ
			}			
		
			if(strstr((const char *)USART2_RX_BUF,(const char *)"FaultSolved"))   //�������ų�������ԭ����
			{			
				u2_printf("FaultSolved");				
			
/*********************����־λ������Ϊ0***********************/
				
				//������ر�־λ			
				body_left_overload_3=0;          //����
				body_left_overload_4=0;          
				body_left_overload_5=0;          
				body_right_overload_3=0;         //�ҷ���
				body_right_overload_4=0;         
				body_right_overload_5=0; 
				washlet_auto_overload=0;         //�Զ�����
				desk_overload=0;                 //�Ͳ�����һ����
				back_nursing_left_overload=0;    //�󱳲�����
				back_nursing_right_overload=0;   //�ұ�������
				waist_nursing_left_overload=0;   //����������
				waist_nursing_right_overload=0;  //����������
				
				//���ʧ����־λ	
				body_left_losepulse=0;           //���� 
				body_right_losepulse=0;          //�ҷ���
				washlet_auto_losepulse=0;        //�Զ�����
				desk_losepulse=0;                //�Ͳ�����һ����
				back_nursing_left_losepulse=0;   //�󱳲�����
				back_nursing_right_losepulse=0;  //�ұ�������
				waist_nursing_left_losepulse=0;  //����������
				waist_nursing_right_losepulse=0; //����������
				break;
			}
			memset(USART2_RX_BUF,0,len);        //�������
			USART2_RX_LEN=0;                    //�����־λ 
		}	
	}
	return 1;    
}


/***********************************************************************
 ������      ��GDCheck(u8 num)
 ��������    ������⺯��
 ����        ��������
 ���        ��1�������ˣ�0���󴥷�                      
************************************************************************/
u8 GDCheck(u8 num)
{
	u32 i=0,j=0,n;
	n=285000;
	switch(num)
	{
		case GD3S:
			for(i=0;i<n;i++)
			{
				if(GD3_Start==0)
				{
					j++;
				}
			}
			break;
		case GD3LE:
			for(i=0;i<n;i++)
			{
				if(GD3_Left_End==0)
				{
					j++;
				}
			}
			break;
		case GD3RE:
			for(i=0;i<n;i++)
			{
				if(GD3_Right_End==0)
				{
					j++;
				}
			}
			break;	
		case GD4S:
			for(i=0;i<n;i++)
			{
				if(GD4_Start==0)
				{
					j++;
				}
			}			
			break;
		case GD4LE:
			for(i=0;i<n;i++)
			{
				if(GD4_Left_End==0)
				{
					j++;
				}
			}				
			break;
		case GD4RE:
			for(i=0;i<n;i++)
			{
				if(GD4_Right_End==0)
				{
					j++;
				}
			}	
			break;		
		case GD5S:
			for(i=0;i<n;i++)
			{
				if(GD5_Start==0)
				{
					j++;
				}
			}	
			break;
		case GD5LE:
			for(i=0;i<n;i++)
			{
				if(GD5_Left_End==0)
				{
					j++;
				}
			}				
			break;
		case GD5RE:
			for(i=0;i<n;i++)
			{
				if(GD5_Right_End==0)
				{
					j++;
				}
			}	
			break;	
		case GD6S:
			for(i=0;i<n;i++)
			{
				if(GD6_Start==0)
				{
					j++;
				}
			}
			break;
		case GD6E:
			for(i=0;i<n;i++)
			{
				if(GD6_End==0)
				{
					j++;
				}
			}
			break;
		case GD7S:
			for(i=0;i<n;i++)
			{
				if(GD7_Start==0)
				{
					j++;
				}
			}
			break;
		case GD7E:
			for(i=0;i<n;i++)
			{
				if(GD7_End==0)
				{
					j++;
				}
			}		
			break;
		case GD34S:
			for(i=0;i<n;i++)
			{
				if((GD3_Start==0)||((GD4_Start==0)))
				{
					j++;
				}
			}		
			break;	
		case GD34LE:
			for(i=0;i<n;i++)
			{
				if((GD3_Left_End==0)||((GD4_Left_End==0)))
				{
					j++;
				}
			}		
			break;
		case GD34RE:
			for(i=0;i<n;i++)
			{
				if((GD3_Right_End==0)||((GD4_Right_End==0)))
				{
					j++;
				}
			}		
			break;

		
		default:
			break;
	}
		if(j<284000)
		{
			j=0;
			return 0;
		}
		else
		{
			j=0;
			return 1;
		}
}

void GDCheckAll()
{
	if(GDCheck(GD3S))
	{u2_printf("GD3S\r\n");}
	if(GDCheck(GD3LE))
	{u2_printf("GD3LE\r\n");}
	if(GDCheck(GD3RE))
	{u2_printf("GD3RE\r\n");}
	if(GDCheck(GD4S))
	{u2_printf("GD4S\r\n");}
	if(GDCheck(GD4LE))
	{u2_printf("GD4LE\r\n");}
	if(GDCheck(GD4RE))
	{u2_printf("GD4RE\r\n");}
	if(GDCheck(GD5S))
	{u2_printf("GD5S\r\n");}	
	if(GDCheck(GD5RE))
	{u2_printf("GD5RE\r\n");}	
	if(GDCheck(GD5RE))
	{u2_printf("GD5RE\r\n");}
	if(GDCheck(GD6S))
	{u2_printf("GD6S\r\n");}
	if(GDCheck(GD6E))
	{u2_printf("GD6E\r\n");}	
	if(GDCheck(GD7S))
	{u2_printf("GD7S\r\n");}	
	if(GDCheck(GD7E))
	{u2_printf("GD7E\r\n");}
}


//���������⺯��
u8 GDCheckDealy(u8 GDID,u16 timedelay)
{
	switch(GDID)
	{
		case GD3S:
			if(1==GD3_Start)
			{
				delay_us(100);
				{
					if(1==GD3_Start)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD3LE:
			if(1==GD3_Left_End)
			{
				delay_us(100);
				{
					if(1==GD3_Left_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD3RE:
			if(1==GD3_Right_End)
			{
				delay_us(100);
				{
					if(1==GD3_Right_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;	
		case GD4S:
			if(1==GD4_Start)
			{
				delay_us(100);
				{
					if(1==GD4_Start)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD4LE:
			if(1==GD4_Left_End)
			{
				delay_us(100);
				{
					if(1==GD4_Left_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD4RE:
			if(1==GD4_Right_End)
			{
				delay_us(100);
				{
					if(1==GD4_Right_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;		
		case GD5S:
			if(1==GD5_Start)
			{
				delay_us(100);
				{
					if(1==GD5_Start)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD5LE:
			if(1==GD5_Left_End)
			{
				delay_us(100);
				{
					if(1==GD5_Left_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD5RE:
			if(1==GD5_Right_End)
			{
				delay_us(100);
				{
					if(1==GD5_Right_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;	
		case GD6S:
			if(1==GD6_Start)
			{
				delay_us(100);
				{
					if(1==GD6_Start)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD6E:
			if(1==GD6_End)
			{
				delay_us(100);
				{
					if(1==GD6_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD7S:
			if(1==GD7_Start)
			{
				delay_us(100);
				{
					if(1==GD7_Start)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD7E:
			if(1==GD7_End)
			{
				delay_us(100);
				{
					if(1==GD7_End)
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD34S:	
			if((1==GD3_Start)||(1==GD4_Start))
			{
				delay_us(100);
				{
					if((1==GD3_Start)||(1==GD4_Start))
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
		
			break;	
		case GD34LE:
			if((1==GD3_Left_End)||(1==GD4_Left_End))
			{
				delay_us(100);
				{
					if((1==GD3_Left_End)||(1==GD4_Left_End))
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;
		case GD34RE:
			if((1==GD3_Right_End)||(1==GD4_Right_End))
			{
				delay_us(100);
				{
					if((1==GD3_Right_End)||(1==GD4_Right_End))
					{
						delay_ms(timedelay);	//��ʱtimedelay ms
					}
				}
			}
			break;

		default:
			break;
	}

}

/***********************************************************************
 ������      ��UsartCheck(u8 num)
 ��������    �������յ��Ĵ����ǲ���str;
 ����        ��
 ���        ��                    
************************************************************************/
u8 UsartCheck(u8* str)
{
		u8 len;
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;
			if(strstr((const char *)USART2_RX_BUF,(const char *)str))    //�����յ�Stop,������ѭ��	
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				return 1;
			} 
			else 
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���								
				return 0;
			}						
		}	
		else
		{
			return 0;
		}
}


/***********************************************************************
 ������      ��UsartCheck2
 ��������    �������յ��Ĵ����ǲ���str;
 ����        ��
 ���        ��                    
************************************************************************/
u8 UsartCheck2(u8* str1,u8* str2)
{
		u8 len;
		if(USART2_RX_LEN&0x8000)
		{
			len=USART2_RX_LEN&0x3fff;				
			USART2_RX_BUF[len]=0;
			if((strstr((const char *)USART2_RX_BUF,(const char *)str1))||(strstr((const char *)USART2_RX_BUF,(const char *)str2)))   //�����յ�Stop,������ѭ��	
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				return 1;
			} 
			else 
			{
				memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);			
				USART2_RX_LEN=0;
				u2_printf("NotRun");    //�յ�ָ�������ִ�е�ǰ�������޷�ȥִ��ȥ���յ���ָ���								
				return 0;
			}						
		}	
		else
		{
			return 0;
		}
}

u8 GDCheckAlm(u8 GDID,u8 num)
{
	switch(GDID)
	{
		case GD3S:
			if(1==GD3_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD3LE:
			if(1==GD3_Left_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD3RE:
			if(1==GD3_Right_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;	
		case GD4S:
			if(1==GD4_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD4LE:
			if(1==GD4_Left_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD4RE:
			if(1==GD4_Right_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;		
		case GD5S:
			if(1==GD5_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD5LE:
			if(1==GD5_Left_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD5RE:
			if(1==GD5_Right_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;	
		case GD6S:
			if(1==GD6_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD6E:
			if(1==GD6_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD7S:
			if(1==GD7_Start)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD7E:
			if(1==GD7_End)
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD34S:	
			if((1==GD3_Start)||(1==GD4_Start))
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
		
			break;	
		case GD34LE:
			if((1==GD3_Left_End)||(1==GD4_Left_End))
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;
		case GD34RE:
			if((1==GD3_Right_End)||(1==GD4_Right_End))
			{
				BeepRun(num,80);
				return 1;
			}
			else
			{return 0;}
			break;

		default:
			break;
	}
}

u8 GDSCheckAllAlm(u8 num)
{
if(GDCheckAlm(GD3S,num))
{u2_printf("GD3S\r\n");}
if(GDCheckAlm(GD4S,num))
{u2_printf("GD4S\r\n");}
if(GDCheckAlm(GD5S,num))
{u2_printf("GD5S\r\n");}
if(GDCheckAlm(GD6S,num))
{u2_printf("GD6S\r\n");}
if(GDCheckAlm(GD7S,num))
{u2_printf("GD7S\r\n");}
}

u8 GDCheckPrint(u8 GDID,u16 usdelay,u8* str)
{
	switch(GDID)
	{
		case GD3S:
			if(0==GD3_Start)
			{
				delay_us(usdelay);
				if(0==GD3_Start)
				{
					
				}
			}
			break;
		case GD3LE:
			if(1==GD3_Left_End)
			{
				delay_us(usdelay);
				if(0==GD3_Left_End)
				{
					
				}
			}
			break;
		case GD3RE:
			if(1==GD3_Right_End)
			{
				delay_us(usdelay);
				if(0==GD3_Right_End)
				{
					
				}
			}
			break;	
		case GD4S:
			if(1==GD4_Start)
			{
				delay_us(usdelay);
				if(0==GD4_Start)
				{
					
				}
			}
			break;
		case GD4LE:
			if(1==GD4_Left_End)
			{
				delay_us(usdelay);
				if(0==GD4_Left_End)
				{
					
				}
			}
			break;
		case GD4RE:
			if(1==GD4_Right_End)
			{
				delay_us(usdelay);
				if(0==GD4_Right_End)
				{
					
				}
			}
			break;		
		case GD5S:
			if(1==GD5_Start)
			{
				delay_us(usdelay);
				if(0==GD5_Start)
				{
					
				}
			}
			break;
		case GD5LE:
			if(1==GD5_Left_End)
			{
				delay_us(usdelay);
				if(0==GD5_Left_End)
				{
					
				}
			}
			break;
		case GD5RE:
			if(1==GD5_Right_End)
			{
				delay_us(usdelay);
				if(0==GD5_Right_End)
				{
					
				}
			}
			break;	
		case GD6S:
			if(1==GD6_Start)
			{
				delay_us(usdelay);
				if(0==GD6_Start)
				{
					
				}
			}
			break;
		case GD6E:
			if(1==GD6_End)
			{
				delay_us(usdelay);
				if(0==GD6_End)
				{
					
				}
			}
			break;
		case GD7S:
			if(1==GD7_Start)
			{
				delay_us(usdelay);
				if(0==GD7_Start)
				{
					
				}
			}
			break;
		case GD7E:
			if(1==GD7_End)
			{
				delay_us(usdelay);
				if(0==GD7_End)
				{
					
				}
			}
			break;
		case GD34S:	
			if((1==GD3_Start)||(1==GD4_Start))
			{
				delay_us(usdelay);
				if(0==GD3_Start)
				{
					
				}
				if(0==GD4_Start)
				{
					
				}
			}
		
			break;	
		case GD34LE:
			if((1==GD3_Left_End)||(1==GD4_Left_End))
			{
				delay_us(usdelay);
				if(0==GD3_Left_End)
				{
					
				}
				if(0==GD4_Left_End)
				{
					
				}
			}
			break;
		case GD34RE:
			if((1==GD3_Right_End)||(1==GD4_Right_End))
			{
				delay_us(usdelay);
				if(0==GD3_Right_End)
				{
					
				}
				if(0==GD4_Right_End)
				{
					
				}
			}
			break;

		default:
			break;
	}
}






