#include "mainjudge.h"
#include "sys.h"
#include "usart.h"
#include "pcf8574.h"
#include "function.h"
#include "check.h"
#include "Hang.h"
#include "washlet.h"
#include "reset.h"
#include "delay.h"
#include "common.h"
#include "pump.h"
#include "w25qxx.h"
#include "led.h"
#include "spi.h"
#include "nand.h"
#include "key.h"
#include "backleg.h"

void AllInit(void)
{
	HAL_Init();                            //��ʼ��HAL��   
	Stm32_Clock_Init(360,25,2,8);          //����ʱ��,180Mhz
	delay_init(180);                       //��ʼ����ʱ����
	uart_init(115200);                     //��ʼ��USART
	usart2_init(115200);                   //��ʼ������2
	usart3_init(115200);                   //��ʼ������3
	uart4_init(115200);	                   //��ʼ������4
	LED_Init();                            //��ʼ��LED 	
	SPI5_Init();					       //��ʼ��SPI��
	Motor_Dir_Init();                      //��ʼ����������
	Push_Rod_Init();                       //��ʼ�����ȵ������������
	Hang_Init();                           //��ʼ�����ҵ������������
	Push_Rod_Swash_Dry_Init();             //��ϴ��ɵ綯��
	Sensor_Init();                         //��ʼ����翪��
	KEY_Init();                            //��ʼ������
	Pump_Init();                           //��ʼ��ˮ��   
	PCF8574_Init();                        //��ʼ��PCF8574/������չоƬ
	NAND_Init();                           //��ʼ��NAND FLASH			
	W25QXX_Init();					       //��ʼ��w25q256/SPI FLASH
}


void WifiReceiveJudge(void)
{
	u8 len;
	u8 *p;
	
	if(UART4_RX_LEN&0x8000)
	{
		len=UART4_RX_LEN&0x3fff;   
		UART4_RX_BUF[len]=0;       
		//���㵱ǰ������WiFi���豸�������ж������ϵ��豸����
		if(strstr((const char *)UART4_RX_BUF,(const char *)"CONNECT")) 
		{	
			device_num++;                        //���ӵ��豸���ۼ�
			PCF8574_WriteBit(BEEP_IO,0);	     //���Ʒ�����
			delay_ms(200);
			PCF8574_WriteBit(BEEP_IO,1);	     //���Ʒ�����		
		}
		//�豸�Ͽ����豸���Զ���1�����ж϶Ͽ����豸����
		if(strstr((const char *)UART4_RX_BUF,(const char *)"CLOSED")) 
		{
			device_num--;                        //���ӵ��豸���ۼ�
			PCF8574_WriteBit(BEEP_IO,0);	     //���Ʒ�����
			delay_ms(200);
			PCF8574_WriteBit(BEEP_IO,1);	     //���Ʒ�����
//			ESP8266_Close_Controler_Type();      //��ȡ��ǰ�Ͽ����豸����
		}
		
		  
/*********************************************************************************			
		
			             ������λ�������趨�����趨�Ĳ���ֵ
			
**********************************************************************************/		
				
		//��λ������Ƕ�
		if(strstr((const char *)UART4_RX_BUF,(const char *)"angleresetall")) 
		  {			 
				get_newangle_wifi();
		  }
		  
		//֧���Ƕ�		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"backrunangle"))  
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"backrunangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=90) )  //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				back_angle_lim=usmart_strnum(p+1);                  //������λ���趨�ĽǶ�ֵ		
//				W25QXX_Write((u8*)back_angle_lim,13,1);             //����������λ���趨�Ƕ�ֵ
			}
			 else
			{
				u2_printf("\r\n֧��������������\r\n");
			}
		}
		
		//����Ƕ�		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnleftangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnleftangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))   //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{				
				body_left_angle_lim=usmart_strnum(p+1);             //������λ���趨�ĽǶ�ֵ
//				W25QXX_Write((u8*)body_left_angle_lim,14,1);        //����������λ���趨����ֵ
			}
			 else
			{
				u2_printf("\r\n���������������\r\n");
			}
		}
		
		//�ҷ���Ƕ�		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnrightangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"bodyturnrightangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))    //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				body_right_angle_lim=usmart_strnum(p+1);             //������λ���趨�ĽǶ�ֵ	
//				W25QXX_Write((u8*)body_right_angle_lim_buf,15,1);    //����������λ���趨����ֵ
			}
			 else
			{
				u2_printf("\r\n�ҷ��������������\r\n");
			}
		}
		
		//�����ȽǶ�			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"legrunupangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"legrunupangle");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=40))   //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				leg_up_angle_lim=usmart_strnum(p+1);                 //������λ���趨�ĽǶ�ֵ	
//				W25QXX_Write((u8*)leg_up_angle_lim_buf,16,1);        //����������λ���趨����ֵ
			}
		    else
			{
				u2_printf("\r\n�����Ȳ�����������\r\n");
			}
		}
		//�����ȽǶ�		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"legrundownangle")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"legrundownangle");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=90))   //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{				
				leg_down_angle_lim=usmart_strnum(p+1);               //������λ���趨�ĽǶ�ֵ	
//				W25QXX_Write((u8*)leg_down_angle_lim_buf,17,1);	     //����������λ���趨����ֵ
			}
			 else
			{
				u2_printf("\r\n�����Ȳ�����������\r\n");
			}
		}
		
		//С���Ӿ���		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"deskdistance")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"deskdistance");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=100))    //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				desk_distance_lim=usmart_strnum(p+1);                 //������λ���趨�ĽǶ�ֵ										
//				W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		  //����������λ���趨����ֵ
			}
		    else
			{
				u2_printf("\r\nС���Ӳ�����������\r\n");
			}
		}	

		//��ϴ���ʱ��		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"swashdry")) 
		{
			p = strstr((const char *)UART4_RX_BUF,(const char *)"swashdry");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=5))   //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				swash_dry_time=usmart_strnum(p+1);                 //������λ���趨�ĳ�ϴ���ʱ��										
//				W25QXX_Write((u8*)swash_dry_time_buf,19,1);	       //����������λ���趨����ֵ	
			}
		    else
			{
				u2_printf("\r\n��ϴ���ʱ����������\r\n");
			}
		}
		
/**********************************************************************************
		
		            WiFi������λ��ָ�ִ�й��ܺ���
		
**********************************************************************************/		
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"wifireset"))  //wifi���� 
		{
			delay_ms(100);
			get_wifiname_wifipassword(wifi_name,wifi_password);	          //��ȡ��WiFi�����ƺ�����			
//              esp_8266_apsta_Init(4);		                              //WiFi��ʼ��		
		}			
						
/***********************************************************************************	
                          
			               ���ܺ���
			
***********************************************************************************/		
	
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackUpPhone"))             //֧����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Back();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackDownPhone"))           //֧����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Back();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpUpPhone"))            //��������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Leg_Up();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegUpDownPhone"))          //��������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Leg_Up();					
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownDownPhone"))        //��������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Leg_Down();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegDownUpPhone"))          //��������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Leg_Down();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftUpPhone"))         //������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Body_Left();						
		}			

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftDownPhone"))       //������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Body_Left();					
		}	

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightUpPhone"))        //�ҷ�����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Body_Right();						
		}	
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightDownPhone"))      //�ҷ�����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Body_Right();						
		}						
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskUpPhone"))             //��������һ������ǰ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Desk();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskDownPhone"))           //��������һ��������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Desk();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackNursingLeftPhone"))    //�󱳲�����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Back_Nursing_Left();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackNursingRightPhone"))   //�ұ�������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Back_Nursing_Right();					
		}				
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WaistNursingLeftPhone"))   //����������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Waist_Nursing_Left();						
		}				

		if(strstr((const char *)UART4_RX_BUF,(const char *)"WaistNursingRightPhone"))  //����������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Fun_Waist_Nursing_Right();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WashletAutoPhone"))        //�Զ�����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Washlet_Auto();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftMuscleMassager"))   //��ۼ��ⰴĦ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Muscle_Massager();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmRightMuscleMassager"))   //�ұۼ��ⰴĦ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Muscle_Massager();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftMuscleMassager"))   //���ȼ��ⰴĦ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Muscle_Massager();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegRightMuscleMassager"))  //���ȼ��ⰴĦ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Muscle_Massager();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLegMuscleMassager"))    //����֫���ⰴĦ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Muscle_Massager();						
		}		
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackMuscleMassager"))      //�������ⰴĦ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Muscle_Massager();						
		}		
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"Heat"))                    //����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Heat();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ResetPhone"))              //���縴λ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Res_Power_Down();						
		}
	
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LDUART4"))                 //���ܺ�������         
		{
			LDUART4();	  //��������					
		}		
		
/***********************************************************************************	
                          
			                    ��������
			
***********************************************************************************/
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackGB"))           //֧��
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			GB_Back();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftGB"))      //����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			GB_Body_Left();						
		}				
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightGB"))     //�ҷ���
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			GB_Body_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackNursingGB"))   //��������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			GB_Back_Nursing();					
		}							
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WaistNursingGB"))  //��������
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			GB_Waist_Nursing();						
		}				
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WashletAutoGB"))   //�Զ�����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Washlet_Auto();					
		}						
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LockGB"))           //����
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			GB_Lock();
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ResetGB"))          //���縴λ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Res_Power_Down();					
		}						
			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"MuscleMassagerPhone"))     //���ⰴĦ
		{
			Muscle_Massager();						
		}

		
/***********************************************************************************	
                          
			               ����ѵ���Զ�ģʽ
			
***********************************************************************************/
					
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftAuto"))          //��첲        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Auto_Arm_Leg_Left(1); 					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmRightAuto"))         //�Ҹ첲        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Auto_Arm_Leg_Right(1);					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmLeftRightAuto"))    //���Ҹ첲        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Auto_Arm_Leg_Left_Right(1);					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftAuto"))          //����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Auto_Arm_Leg_Left(1); 					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegRightAuto"))         //����       
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Auto_Arm_Leg_Right(1);						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegLeftRightAuto"))    //������        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Auto_Arm_Leg_Left_Right(1);						
		}
			
/***********************************************************************************	
                          
			               ����ѵ���ֶ�ģʽ
			
***********************************************************************************/			
		
		//��첲						
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftUpHand"))            //��첲С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftDownHand"))          //��첲С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Left();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftUpHand"))            //��첲�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftDownHand"))          //��첲�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Post_Left();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftUpHand"))       //��첲��С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftDownHand"))     //��첲��С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Post_Left();						
		}			

		//�Ҹ첲			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightUpHand"))           //�Ҹ첲С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeRightDownHand"))          //�Ҹ첲С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightUpHand"))            //�Ҹ첲�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostRightDownHand"))          //�Ҹ첲�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Post_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightUpHand"))       //�Ҹ첲��С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostRightDownHand"))     //�Ҹ첲��С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Post_Right();						
		}
			
		//���Ҹ첲			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))       //���Ҹ첲С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))     //���Ҹ첲С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Left_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))       //���Ҹ첲�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))     //���Ҹ첲�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Post_Left_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))  //���Ҹ첲��С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ArmForePostLeftRightDownHand")) //���Ҹ첲��С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Arm_Fore_Post_Left_Right();						
		}			
			
		//����			
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftUpHand"))              //����С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftDownHand"))            //����С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Left();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftUpHand"))              //���ȴ�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftDownHand"))            //���ȴ�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Post_Left();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftUpHand"))         //���ȴ�С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Post_Left();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftDownHand"))       //���ȴ�С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Post_Left();						
		}			

		//����
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightUpHand"))             //����С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeRightDownHand"))           //����С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightUpHand"))             //���ȴ�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostRightDownHand"))           //���ȴ�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Post_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightUpHand"))        //���ȴ�С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Post_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostRightDownHand"))      //���ȴ�С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Post_Right();						
		}
			
		//������
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightUpHand"))        //������С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForeLeftRightDownHand"))      //������С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Left_Right();					
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightUpHand"))        //�����ȴ�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegPostLeftRightDownHand"))      //�����ȴ�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Post_Left_Right();						
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightUpHand"))   //�����ȴ�С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Post_Left_Right();					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegForePostLeftRightDownHand")) //�����ȴ�С����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Hand_Leg_Fore_Post_Left_Right();						
		}

/***********************************************************************************	
                          
			               WiFi����ר��ϵͳ����ָ��
			
***********************************************************************************/
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertBack"))           //ר��ϵͳ֧��     
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Exp_Back();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertBody"))          //ר��ϵͳ����      
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Exp_Body();						
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertLeg"))           //ר��ϵͳ����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Exp_Leg();						
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"ExpertWashletAuto"))  //ר��ϵͳ�Զ�����        
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Exp_Washlet_Auto();						
		}
		
/***********************************************************************************	
					  
					         WiFi���չ��ϴ������ָ��
		
***********************************************************************************/			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BackFaultReset"))	            //֧����λ
		{
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Res_Back();     		    
		}
		if(strstr((const char *)UART4_RX_BUF,(const char *)"LegFaultReset"))                //���ȸ�λ
		{			
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Res_Leg();   		       
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"WashLetFaultReset"))            //��������λ
		{			
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			washlet_picture_k=24;
			Washlet(1);	              
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"DeskFaultReset"))               //�Ͳ�����һ������λ
		{			
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Res_Desk();              
		}			
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyLeftFaultReset"))          //����λ
		{			
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Res_Body_Left();       					
		}

		if(strstr((const char *)UART4_RX_BUF,(const char *)"BodyRightFaultReset"))         //�ҷ���λ	
		{			
			if(ESP8266_Get_ID())	//���յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
			Res_Body_Right();        
		}
		
		if(strstr((const char *)UART4_RX_BUF,(const char *)"FaultSolved"))                   //�������ų������ϴ���������һ���ذ�ָ��
		{
			Wifi_Send("Received");
			Wifi_Send("FaultSolved");
			
/*********************����־λ������Ϊ0***********************/
			
			//������ر�־λ			
			body_left_overload_3=0;          //����
			body_left_overload_4=0;          //����
			body_left_overload_5=0;          //����
			body_right_overload_3=0;         //�ҷ���
			body_right_overload_4=0;         //�ҷ���
			body_right_overload_5=0;         //�ҷ���
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

		}					
		
/*****************************************************************************/
		
		memset(UART4_RX_BUF,0,len);			
		UART4_RX_LEN=0;				
	}		
}

void Usart2ReceiveJudge(void)
{
	if(USART2_RX_LEN&0x8000)
	{
		u8 *p;
		u8 len;
		u32 HangId;
		len=USART2_RX_LEN&0x3fff;				
		USART2_RX_BUF[len]=0;

/*****************************************************************************			
			             WiFi���ú���		
******************************************************************************/		
		
		if(strstr((const char *)USART2_RX_BUF,(const char *)"LEDChange"))        //WiFi����ΪAPģʽ
		{			 
			LED0=!LED0;
			LED1=!LED1;
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"mode"))          //��λ������WiFiģʽ
		{			 
			if(strstr((const char *)USART2_RX_BUF,(const char *)"AP"))        //WiFi����ΪAPģʽ
			{			 
				W25QXX_Write((u8*)modeap_buf,10,2);
				u2_printf("\r\n���ó�APģʽ\r\n");
			}
			
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"STA"))       //WiFi����ΪSTAģʽ
			{			 
				W25QXX_Write((u8*)modesta_buf,10,2);	
				u2_printf("\r\n���ó�STAģʽ\r\n");					
			}					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"staset"))        //wifi����-STAģʽ
		{ 
			get_wifiname_wifipassword(wifi_station,wifi_station_password);    //��ȡWiFi���ƺ�����
		}		

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"apset"))         //wifi����-APģʽ
		{ 
			get_apname_wifipassword(wifi_ssid,wifi_ssid_password);            //��ȡWiFi�����ƺ�����
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ipportset"))     //wifi����
		{ 
			get_ip_port(wifi_ap_ip,wifi_ap_port);                             //��ȡWiFi��ip��ַ�Ͷ˿ں�
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifireset"))     //wifi����
		{ 
			delay_ms(100);
			get_wifiname_wifipassword(wifi_name,wifi_password);	              //��ȡWiFi�����ƺ�����
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"stastart"))      //wifiģʽ��ʼ��
		{ 
			ESP8266_apsta_Init(4);	
		}			

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"apstart"))       //apģʽ��ʼ��
		{ 
			ESP8266_AP_Init(4);	
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"newangle"))      //��ȡ��λ�������趨ֵ
		{ 
			Read_Angle();
		}	
/*******************************************************************************

					     �ж������豸����
		
********************************************************************************/
		


		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifiphone"))      
		{			 
			ESP8266_send_data(Phone,"�����ֻ�");
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifipad"))       
		{			 
			ESP8266_send_data(Pad,"����ƽ��");
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifiremote"))      
		{			 
			ESP8266_send_data(Remote_Ctl,"����ң����");
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifiguard"))       
		{			 
			ESP8266_send_data(Guard_Ctl,"���Ի���");
		}		

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"wifipc"))          
		{			 
			ESP8266_send_data(PC,"���Ե���");
		}	

/*******************************************************************************

						������λ�������趨ֵ
		
********************************************************************************/
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"angleresetall"))  //��λ������Ƕ�
		{			 
			get_newangle_usart2();  //����µĽǶ�ֵ
		}		
		
		//֧��		  		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"backrunangle"))   //֧��
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"backrunangle");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=90))             //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				back_angle_lim=usmart_strnum(p+1);                             //������λ���趨�ĽǶ�ֵ																			
//				W25QXX_Write((u8*)back_angle_lim,13,1);                        //����������λ���趨�Ƕ�ֵ
			}
		}

		//����	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnleftangle"))//����
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnleftangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))                //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{				
				body_left_angle_lim=usmart_strnum(p+1);                          //������λ���趨�ĽǶ�ֵ										 								
//				W25QXX_Write((u8*)body_left_angle_lim,14,1);                     //����������λ���趨����ֵ
			}
		}

		//�ҷ���	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnrightangle"))//�ҷ���
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"bodyturnrightangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=60))                 //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				body_right_angle_lim=usmart_strnum(p+1);                          //������λ���趨�ĽǶ�ֵ															
//				W25QXX_Write((u8*)body_right_angle_lim_buf,15,1);                 //����������λ���趨����ֵ
			}
		}

		//������		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"legrunupangle"))     //������
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"legrunupangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=40))                 //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				leg_up_angle_lim=usmart_strnum(p+1);                              //������λ���趨�ĽǶ�ֵ													
//				W25QXX_Write((u8*)leg_up_angle_lim_buf,16,1);                     //����������λ���趨����ֵ				
			}
		}

		//������	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"legrundownangle"))   //������
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"legrundownangle");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=90))                 //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				leg_down_angle_lim=usmart_strnum(p+1);                            //������λ���趨�ĽǶ�ֵ													
//				W25QXX_Write((u8*)leg_down_angle_lim_buf,17,1);	                  //����������λ���趨����ֵ
			}
		}

		//��������һ����	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"deskdistance"))      //��������һ����
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"deskdistance");
			if((usmart_strnum(p+1)!=0) &&(usmart_strnum(p+1)<=100))               //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				desk_distance_lim=usmart_strnum(p+1);                             //������λ���趨�ĽǶ�ֵ											
//				W25QXX_Write((u8*)desk_distance_lim_buf,18,1);		              //����������λ���趨����ֵ
			}
		}
								
		//��ϴ���ʱ��		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"swashdry")) 
		{
			p = strstr((const char *)USART2_RX_BUF,(const char *)"swashdry");
			if((usmart_strnum(p+1)!=0)&&(usmart_strnum(p+1)<=5))    //����λ�����͵��ַ����е�����ת��Ϊʮ����
			{
				swash_dry_time=usmart_strnum(p+1);                  //������λ���趨�ĳ�ϴ���ʱ��														
//				W25QXX_Write((u8*)swash_dry_time_buf,19,1);	        //����������λ���趨����ֵ	
			}
		    else
			{
				u2_printf("\r\n��ϴ���ʱ����������\r\n");
			}
		}

/***********************************************************************************	                       
			                 ���ڽ����ƶ��豸ָ��	
***********************************************************************************/		
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackUpPhone"))            //֧����
		{
			Uart_Back();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownPhone"))           //֧����
		{
			Uart_Back();
		}

		
	/***�µ�֧�����ú���***/
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackUpNew"))    //֧����,����BackUpNew+85
		{
			u32 BackUpParam;
			BackUpParam=usmart_strnum2(USART2_RX_BUF);
			BackRun(1,BackUpParam%100);		
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackDownNew"))           //֧����,����BackUpNew+85
		{
			u32 BackDownParam;
			BackDownParam=usmart_strnum2(USART2_RX_BUF);
			BackRun(0,BackDownParam%100);	
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpPhone"))            //��������
		{
			Uart_Leg_Up();
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownPhone"))          //��������
		{
			Uart_Leg_Up();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpUpNew"))           //֧����,����BackUpNew+85
		{
			u32 LegUpParam;
			LegUpParam=usmart_strnum2(USART2_RX_BUF);
			LegUpRun(1,LegUpParam%100);	
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegUpDownNew"))           //֧����,����BackUpNew+85
		{
			u32 LegUpParam;
			LegUpParam=usmart_strnum2(USART2_RX_BUF);
			LegUpRun(0,LegUpParam%100);	
		}
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownPhone"))        //��������
		{
			Uart_Leg_Down();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpPhone"))          //��������
		{
			Uart_Leg_Down();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownDownNew"))           //֧����,����BackUpNew+85
		{
			u32 LegDownParam;
			LegDownParam=usmart_strnum2(USART2_RX_BUF);
			LegDownRun(1,LegDownParam%100);	
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegDownUpNew"))           //֧����,����BackUpNew+85
		{
			u32 LegDownParam;
			LegDownParam=usmart_strnum2(USART2_RX_BUF);
			LegDownRun(0,LegDownParam%100);	
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpPhone"))         //������
		{
			Uart_Body_Left();						
		}			

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownPhone"))       //������
		{
			Uart_Body_Left();					
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpPhone"))        //�ҷ�����
		{
			Uart_Body_Right();						
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownPhone"))     //�ҷ�����
		{
			Uart_Body_Right();						
		}						
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftUpNew")) 	//BodyLeftUpNew+45     
		{
			u32 BdyLftUpPara;
			BdyLftUpPara=usmart_strnum2(USART2_RX_BUF);		
			BodyLeftRun(1,motor_body_freq,motor_body_freq,BdyLftUpPara%100);
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftDownNew")) //BodyLeftDownNew+45        
		{
			u32 BdyLftDwnPara;
			BdyLftDwnPara=usmart_strnum2(USART2_RX_BUF);		
			BodyLeftRun(0,motor_body_freq,motor_body_freq,BdyLftDwnPara%100);
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightUpNew")) 	//BodyRightUpNew+45     
		{
			u32 BdyRhtUpPara;
			BdyRhtUpPara=usmart_strnum2(USART2_RX_BUF);		
			BodyRightRun(1,motor_body_freq,motor_body_freq,BdyRhtUpPara%100);
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightDownNew")) //BodyightDownNew+45        
		{
			u32 BdyRhtDwnPara;
			BdyRhtDwnPara=usmart_strnum2(USART2_RX_BUF);		
			BodyRightRun(0,motor_body_freq,motor_body_freq,BdyRhtDwnPara%100);
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))            //��������һ������ǰ
		{
			Uart_Desk1();
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))          //��������һ��������
		{
			Uart_Desk1();
		}
		

//		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskUpPhone"))            //��������һ������ǰ
//		{
//			Uart_Desk();
//		}
//		
//		if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskDownPhone"))          //��������һ��������
//		{
//			Uart_Desk();
//		}


		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackNursingLeftPhone"))    //�󱳲�����
		{
			Uart_Back_Nursing_Left();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackNursingRightPhone"))   //�ұ�������
		{
			Uart_Back_Nursing_Right();					
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WaistNursingLeftPhone"))   //����������
		{
			Uart_Waist_Nursing_Left();						
		}				

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WaistNursingRightPhone"))  //����������
		{
			Uart_Waist_Nursing_Right();
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MuscleMassagerPhone"))     //���ⰴĦ
		{
			Muscle_Massager();						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Stop"))   
		{
			Motor_6_1_STOP();			
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"JD0"))     //���ⰴĦ
		{
			DIR_QB=0;           //����ֹͣ
			//DIR_JRAM=0;         //��Ħ�������Źر�
			delay_ms(1000);     //�ȴ����Źر�
			//Wifi_Send("MuscleMassagerStop");
			u2_printf("MuscleMassagerStop");						
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashletAutoPhone"))        //�Զ�����
		{
			//Uart_Washlet_Auto();
			WashLet_V2(1,32950);
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DIR_HG1"))        //�Զ�����
		{
			DIR_HG=1;						
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DIR_HG0"))        //�Զ�����
		{
			DIR_HG=0;						
		}
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"SB1"))                    //ˮ�ô�
		{
			DIR_SB=1;
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"SB0"))                    //ˮ�ùر�
		{
			DIR_SB=0;
		}
				
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Heat"))                    //����
		{
			Heat();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ResetPhone"))             //���縴λ
		{
			Uart_Res_Power_Down();						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LDUART2"))                //���ܺ�������         
		{
			LDUART2();	  //��������					
		}
		
//************************���Ժ���*********************************
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart22222_Dry_Auto"))   //����������
		{
			washlet_flag=1;
			Uart_Dry_Auto();		//�Զ����				
		}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart22222_Swash_Auto"))   //����������
		{
			washlet_flag=1;
			Uart_Swash_Auto();	//�Զ���ϴ					
		}
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Push_Rod_SwashUp"))        //��ϴ���
		{
			RELAY6=1; 
			Uart_Push_Rod_Swash(1,30000);   //��ϴ����Ƹ����
			RELAY6=0; 
		}

		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Push_Rod_SwashDown"))          //��ϴ���       
		{
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,30000);  //��ϴ����Ƹ�����
			RELAY6=0;	      					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"weight"))          //��ϴ���       
		{	 
			Uart_Washlet_Weight(); 	
			PCF8574_WriteBit(BEEP_IO,0);	//���Ʒ�������
			delay_ms(300);
			PCF8574_WriteBit(BEEP_IO,1);	//���Ʒ�����ͣ			
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart2_Push_Rod_Swash1"))        //��ϴ���
		{
			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(1,30000);   //��ϴ����Ƹ����
			RELAY6=0; 
		}

		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart2_Push_Rod_Swash0"))          //��ϴ���       
		{
//			washlet_flag=1;
			RELAY6=1; 
			Uart_Push_Rod_Swash(0,30000);  //��ϴ����Ƹ�����
			RELAY6=0;	      					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Washlet_Tig0"))      //������ս�          
		{
//			washlet_flag=1;
			Uart_Washlet_Tig(0);	  	//���������	
		}		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Washlet_Tig1"))      //������ս�        
		{
//			washlet_flag=1;
			Uart_Washlet_Tig(1);	  		 //��������ߵ������
		}
		
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_WashletTigOnly"))      //������ս�        
		{
			//washlet_flag=1;
			Uart_WashletTigOnly(1);
		}


		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Motor_6_2_START1"))      //������Ƹ�       
		{
			washlet_flag=1;
			RELAY6=1;                       //�̵����õ�
			Uart_Motor_6_2_START(0,16500);   //�����Ƹ�����	
			RELAY6=0;             
		}		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart_Motor_6_2_START0"))      //������Ƹ�          
		{
			washlet_flag=1;
			RELAY6=1;             //�̵����õ�
			Uart_Motor_6_2_START(1,16500);            //�����Ƹ����	
			RELAY6=0;           
		}		

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetMotor"))      //������Ƹ�          
		{
			u32 WashLetMotorParam;
			WashLetMotorParam=usmart_strnum2(USART2_RX_BUF);
			u2_printf("dir=%d,Arr=%d",WashLetMotorParam/100000%10,WashLetMotorParam%100000);
			if(strstr((const char *)USART2_RX_BUF,(const char *)"CXHG"))
			{
				u2_printf("CXHG\r\n");
				MotorStart(8,WashLetMotorParam/100000%10,WashLetMotorParam%100000);
			}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"SXTG"))
			{
				u2_printf("SXTG\r\n");
				MotorStart(9,WashLetMotorParam/100000%10,WashLetMotorParam%100000);
			}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"SXDJ"))
			{
				u2_printf("SXDJ\r\n");
				MotorStart(10,WashLetMotorParam/100000%10,WashLetMotorParam%100000);		
			}				
		}


		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart222_Washlet1"))      //�����       
		{
			u2_printf("��������");
			Uart_Washlet(0);	  				
		}		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Uart222_Washlet0"))      //����ر�          
		{
			u2_printf("�������ر�");
			washlet_picture_k=24;
			Uart_Washlet(1);	  				
		}		
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"testhang1"))      //���         
		{
			TestAll(1);	  				
		}	

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"testhang0"))      //���         
		{
			TestAll(0);	  				
		}	
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"testhangAll_1"))      //���         
		{
			test(1);	  				
		}


		else if(strstr((const char *)USART2_RX_BUF,(const char *)"pcf1"))      //���         
		{
			PCF8574_WriteBit(EXIO1,1);	       //�̵���
			PCF8574_WriteBit(EXIO2,1);	       //�̵���
			PCF8574_WriteBit(EXIO3,1);	       //�̵���
			PCF8574_WriteBit(EXIO4,1);	       //�̵���
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"pcf0"))      //���         
		{
			PCF8574_WriteBit(EXIO1,0);	       //�̵���
			PCF8574_WriteBit(EXIO2,0);	       //�̵���
			PCF8574_WriteBit(EXIO3,0);	       //�̵���
			PCF8574_WriteBit(EXIO4,0);	       //�̵���
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang1_1"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang1Test(1);	  	
			DG_Relay=0;		//�̵���ʧ��
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang1_0"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang1Test(0);	
			DG_Relay=0;		//�̵���ʧ��
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang2_1"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang2Test(1);	  
			DG_Relay=0;		//�̵���ʧ��
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang2_0"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang2Test(0);	
			DG_Relay=0;		//�̵���ʧ��
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang3_1"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang3Test(1);	 
			DG_Relay=0;		//�̵���ʧ��
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang3_0"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang3Test(0);	
			DG_Relay=0;		//�̵���ʧ��
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang4_1"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang4Test(1);	 
			DG_Relay=0;		//�̵���ʧ��
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Hang4_0"))      //���         
		{
			DG_Relay=1;		//�̵����õ�
			Hang4Test(0);
			DG_Relay=0;		//�̵���ʧ��
		}
		
		//�������򿪻��߱պϣ�����WashLetNew+1+33000
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetNew"))      //�������򿪣�Ĭ��33000�������     
		{
			u32 WashLetPara;
			WashLetPara=usmart_strnum2(USART2_RX_BUF);
			WashLet_V1(WashLetPara/100000%10,WashLetPara%100000);		//1���򿪣�0���½�
		}
	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetRunNew"))  //WashLetRunNew+1+33000
		{
			u32 WashLetRunPara;
			WashLetRunPara=usmart_strnum2(USART2_RX_BUF);
			WashletRun(WashLetRunPara/100000%10,WashLetRunPara%100000,timer10_freq);
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskRunUpNew"))      //DeskRunUpNew+1+099         
		{
			u32 DeskRunPara;
			DeskRunPara=usmart_strnum2(USART2_RX_BUF);
			DeskRun1(DeskRunPara/1000,DeskRunPara%1000);
			
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskRunDownNew"))      //DeskRunDownNew+1+099         
		{
			u32 DeskRunPara;
			DeskRunPara=usmart_strnum2(USART2_RX_BUF);
			DeskRun1(DeskRunPara/1000,DeskRunPara%1000);
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLet_V2")) 
		{WashLet_V2(1,32950);}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"RunAllAuto"))      //���        
		{
			LDUART2V2();
			u2_printf("over");
		}

		
		//���ң����磺HangRunUp+6+1111
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"HangRun"))
		{
			int temp;
			DG_Relay=1;		//�̵����õ�
//			HangId=usmart_strnum(USART2_RX_BUF);	
				HangId=usmart_strnum2(USART2_RX_BUF);			
			if(strstr((const char *)USART2_RX_BUF,(const char *)"Up"))
			{
					HangRun(1,HangId/10000,HangId/1000%10,HangId/100%10,HangId/10%10,HangId%10);
			}
			else if(strstr((const char *)USART2_RX_BUF,(const char *)"Down"))
			{
				HangRun(0,HangId/10000,HangId/1000%10,HangId/100%10,HangId/10%10,HangId%10);
			}
			DG_Relay=0;		//�̵���ʧ��			
		}	
		
		//������HangLR+0+1+0+2+15(����֫+����+����+����+�߶�),(0+1+0+2+15)=(��֫+��+�����+2��+�߶�)
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"HangLR"))   
		{
			u32 HangLRPara;
			DG_Relay=0;		//�̵���ʧ��
			HangLRPara=usmart_strnum2(USART2_RX_BUF);
			if(HangLRPara)
			{
				HangRunAuto(HangLRPara/100000,HangLRPara/10000%10,HangLRPara/1000%10,HangLRPara/100%10,HangLRPara%100);
			}
			DG_Relay=0;		//�̵���ʧ��
		}

		
 //���1  
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR1111"))      //���1         
		{
			MOTOR111(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR1110"))      //���1         
		{
			MOTOR111(0);	  				
		} 
 
 //���2  
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR2221"))      //���2         
		{
			MOTOR222(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR2220"))      //���2         
		{
			MOTOR222(0);	  				
		} 
 
 //���3  
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR3331"))      //���3         
		{
			MOTOR333(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR3330"))      //���3         
		{
			MOTOR333(0);	  				
		}
 //���4 
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR4441"))      //���4         
		{
			MOTOR444(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR4440"))      //���4         
		{
			MOTOR444(0);	  				
		}
//���5 		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR5551"))      //���5         
		{
			MOTOR555(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR5550"))      //���5         
		{
			MOTOR555(0);	  				
		}
//���6 		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR6661"))      //���6         
		{
			MOTOR666(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR6660"))      //���6         
		{
			MOTOR666(0);	  				
		}
//���7		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR7771"))      //���7         
		{
			MOTOR777(1);	  				
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"MOTOR7770"))      //���7         
		{
			MOTOR777(0);	  				
		}

	
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"Nippon"))      //���7         
		{
			if(UsartCheck2("Japan","Nippon"))
			{u2_printf("true\r\n");}
		}
		
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ResetAllAuto"))      //���7         
		{
			ResetAll();
		}		

/***********************************************************************************	
                          
			               ���ڽ��ջ���ָ��
			
***********************************************************************************/
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackGB"))           //֧��
		{
			Uart_GB_Back();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftGB"))      //����
		{
			Uart_GB_Body_Left();						
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightGB"))     //�ҷ���
		{
			Uart_GB_Body_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackNursingGB"))   //��������
		{
			Uart_GB_Back_Nursing();					
		}							
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WaistNursingGB"))  //��������
		{
			Uart_GB_Waist_Nursing();						
		}				
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashletAutoGB"))   //�Զ�����
		{
			Uart_Washlet_Auto();					
		}						
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LockGB"))          //����
		{
			Uart_GB_Lock();
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ResetGB"))        //���縴λ
		{
			Uart_Res_Power_Down();					
		}	
	
			
/***********************************************************************************	
                          
			               ����ѵ���Զ�ģʽ
			
***********************************************************************************/
							
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftAuto"))        //��첲        
		{
			Uart_Auto_Arm_Leg_Left(1); 					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmRightAuto"))       //�Ҹ첲        
		{
			Uart_Auto_Arm_Leg_Right(1);					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmLeftRightAuto"))  //���Ҹ첲        
		{
			Uart_Auto_Arm_Leg_Left_Right(1);					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftAuto"))        //����        
		{
			Uart_Auto_Arm_Leg_Left(1); 					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegRightAuto"))       //����       
		{
			Uart_Auto_Arm_Leg_Right(1);						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegLeftRightAuto"))  //������        
		{
			Uart_Auto_Arm_Leg_Left_Right(1);						
		}
		
/*****************************************************************************************
									
							    ����ѵ���ֶ�ģʽ
		
/****************************************************************************************/
			
		//��첲						
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftUpHand"))      //��첲С����        
		{
			Uart_Hand_Arm_Fore_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftDownHand"))    //��첲С����        
		{
			Uart_Hand_Arm_Fore_Left();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftUpHand"))      //��첲�����        
		{
			Uart_Hand_Arm_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftDownHand"))    //��첲�����        
		{
			Uart_Hand_Arm_Post_Left();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftUpHand"))  //��첲��С����        
		{
			Uart_Hand_Arm_Fore_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftDownHand"))//��첲��С����        
		{
			Uart_Hand_Arm_Fore_Post_Left();						
		}			

		//�Ҹ첲			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightUpHand"))     //�Ҹ첲С����        
		{
			Uart_Hand_Arm_Fore_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeRightDownHand"))   //�Ҹ첲С����        
		{
			Uart_Hand_Arm_Fore_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightUpHand"))     //�Ҹ첲�����        
		{
			Uart_Hand_Arm_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostRightDownHand"))   //�Ҹ첲�����        
		{
			Uart_Hand_Arm_Post_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightUpHand")) //�Ҹ첲��С����        
		{
			Uart_Hand_Arm_Fore_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostRightDownHand"))//�Ҹ첲��С����        
		{
			Uart_Hand_Arm_Fore_Post_Right();						
		}
		
		//���Ҹ첲			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightUpHand"))  //���Ҹ첲С����        
		{
			Uart_Hand_Arm_Fore_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForeLeftRightDownHand"))//���Ҹ첲С����        
		{
			Uart_Hand_Arm_Fore_Left_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightUpHand"))  //���Ҹ첲�����        
		{
			Uart_Hand_Arm_Post_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmPostLeftRightDownHand"))//���Ҹ첲�����        
		{
			Uart_Hand_Arm_Post_Left_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightUpHand"))//���Ҹ첲��С����        
		{
			Uart_Hand_Arm_Fore_Post_Left_Right();					
		}

		if(strstr((const char *)USART2_RX_BUF,(const char *)"ArmForePostLeftRightDownHand"))//���Ҹ첲��С����        
		{
			Uart_Hand_Arm_Fore_Post_Left_Right();						
		}			
		
		//����			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftUpHand"))           //����С����        
		{
			Uart_Hand_Leg_Fore_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftDownHand"))         //����С����        
		{
			Uart_Hand_Leg_Fore_Left();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftUpHand"))           //���ȴ�����        
		{
			Uart_Hand_Leg_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftDownHand"))         //���ȴ�����        
		{
			Uart_Hand_Leg_Post_Left();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftUpHand"))       //���ȴ�С����        
		{
			Uart_Hand_Leg_Fore_Post_Left();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftDownHand"))     //���ȴ�С����        
		{
			Uart_Hand_Leg_Fore_Post_Left();						
		}			

		//����
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightUpHand"))          //����С����        
		{
			Uart_Hand_Leg_Fore_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeRightDownHand"))        //����С����        
		{
			Uart_Hand_Leg_Fore_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightUpHand"))          //���ȴ�����        
		{
			Uart_Hand_Leg_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostRightDownHand"))        //���ȴ�����        
		{
			Uart_Hand_Leg_Post_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightUpHand"))      //���ȴ�С����        
		{
			Uart_Hand_Leg_Fore_Post_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostRightDownHand"))   //���ȴ�С����        
		{ 
			Uart_Hand_Leg_Fore_Post_Right();						
		}
		
		//������
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightUpHand"))    //������С����        
		{
			Uart_Hand_Leg_Fore_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForeLeftRightDownHand"))  //������С����        
		{
			Uart_Hand_Leg_Fore_Left_Right();					
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightUpHand"))    //�����ȴ�����        
		{
			Uart_Hand_Leg_Post_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegPostLeftRightDownHand"))  //�����ȴ�����        
		{
			Uart_Hand_Leg_Post_Left_Right();						
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightUpHand")) //�����ȴ�С����        
		{
			Uart_Hand_Leg_Fore_Post_Left_Right();					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegForePostLeftRightDownHand"))//�����ȴ�С����        
		{
			Uart_Hand_Leg_Fore_Post_Left_Right();						
		}
		

/***********************************************************************************	
                          
			               ���ڽ���ר��ϵͳ����ָ��
			
***********************************************************************************/
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertBack"))         //ר��ϵͳ֧��     
		{
			Uart_Exp_Back();						
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertBody"))         //ר��ϵͳ����      
		{
			Uart_Exp_Body();						
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertLeg"))          //ר��ϵͳ����        
		{
			Uart_Exp_Leg();						
		}
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"ExpertWashletAuto"))  //ר��ϵͳ�Զ�����        
		{
			Uart_Exp_Washlet_Auto();						
		}
	
		
/***********************************************************************************	
                          
			               ���ڽ��չ��ϴ������ָ��
			
***********************************************************************************/			
			
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BackFaultReset"))	      //֧����λ
		{
			Uart_Res_Back();     		    
		}
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"LegFaultReset"))         //���ȸ�λ
		{			
			Uart_Res_Leg();   		       
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"WashLetFaultReset"))     //��������λ
		{
			washlet_picture_k=24;
			Uart_Washlet(1);	              
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"DeskFaultReset"))        //�Ͳ�����һ������λ
		{			
			Uart_Res_Desk();              
		}			
		
		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyLeftFaultReset"))    //����λ
		{			
			Uart_Res_Body_Left();       					
		}

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"BodyRightFaultReset"))   //�ҷ���λ	
		{			
			Uart_Res_Body_Right();        
		}					

		else if(strstr((const char *)USART2_RX_BUF,(const char *)"FaultSolved"))           //�������ų������ϴ���������һ���ذ�ָ��
		{
			u2_printf("Received");
			u2_printf("FaultSolved");
			
/*********************����־λ������Ϊ0***********************/
			
			//������ر�־λ			
			body_left_overload_3=0;          //����
			body_left_overload_4=0;          //����
			body_left_overload_5=0;          //����
			body_right_overload_3=0;         //�ҷ���
			body_right_overload_4=0;         //�ҷ���
			body_right_overload_5=0;         //�ҷ���
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
		}					
			
/********************************************************************************************************/	
				
		memset(USART2_RX_BUF,0,len);   //�������
		USART2_RX_LEN=0;               //�����־λ           	
	}	
}




