#include "common.h"
#include "usart.h"
#include "timer.h"
#include "w25qxx.h"

//Ĭ���豸��
u8  Phone=6;       //�ֻ�app
u8  Remote_Ctl=6;  //ң����
u8  Guard_Ctl=6;   //����
u8  PC=6;          //PC��
u8  Pad=6;         //Pad

//�豸����״̬ 0��δ���ӣ�1��������
u8 Phone_Ready=0;          //�ֻ�app
u8 PC_Ready=0;             //PC
u8 RemoteCtl_Ready=0;      //ң����
u8 GuardCtl_Ready=0;       //����
u8 ComputerStick_Ready=0;  //����

u8 ID_NUM;                 //�����ն˶�Ӧ���豸��

/***********************************************************************
 ������      ��atk_8266_at_response(u8 mode)  
 ��������    ���ô���2��ӡWiFi���յ������ݡ�
 ����        ��0��UART4_RX_LEN�����㣬1������
 ���        ����  
 
************************************************************************/
void atk_8266_at_response(u8 mode)
{
	if(UART4_RX_LEN&0X8000)		
	{ 
		UART4_RX_BUF[UART4_RX_LEN&0X7FFF]=0;
		u2_printf("%s",UART4_RX_BUF);	
		if(mode)UART4_RX_LEN=0;
	} 
}


/***********************************************************************
 ������      ��atk_8266_check_cmd(u8 *str)  
 ��������    ���ж�UART4_RX_BUF����������ָ���ַ���
 ����        ��0��UART4_RX_LEN�����㣬     1������
 ���        ��ָ���ַ�����һ���ַ��ĵ�ַ

************************************************************************/
u8* atk_8266_check_cmd(u8 *str)
{	
	char *strx=0;
	if(UART4_RX_LEN&0X8000)	
	{ 
		UART4_RX_BUF[UART4_RX_LEN&0X7FFF]=0;
		strx=strstr((const char*)UART4_RX_BUF,(const char*)str); 		
	} 
	return (u8*)strx;
}

/***********************************************************************
 ������      ��atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
 ��������    ������WiFiָ��
 ����        ��cmd:ָ��,ack:����ֵ,waittime:��ʱʱ��
 ���        ��1: ָ��ͳɹ�������ֵ��ȷ  0��ʧ��  

************************************************************************/
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u16 time=0;
	u8 res=0; 
	UART4_RX_LEN=0;
	u4_printf("%s\r\n",cmd);                  //����CMDָ��
	time = 0;
	while(time<waittime)	
	{
		delay_ms(10);		
		if(atk_8266_check_cmd(ack))       //�жϷ���ֵ
		{
//				printf("ack:%s\r\n",(u8*)ack); 
			break;
		}	
		time++;				
	}
	if(time<waittime)res=1;                   //����ڳ�ʱʱ����break��������res=1
//	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	
	return res;
} 

/***********************************************************************
 ������      ��atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
 ��������    ����������
 ����        ��
 ���        ��

************************************************************************/
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	UART4_RX_LEN=0;
	u4_printf("%s",data);	
	if(ack&&waittime)	
	{
		while(--waittime)	
		{
			delay_ms(5);
			if(UART4_RX_LEN&0X8000)
			{
				if(atk_8266_check_cmd(ack))break;
				UART4_RX_LEN=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}

/***********************************************************************
 ������      ��atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
 ��������    ���������ݸ��豸0
 ����        ���ַ���
 ���        ���� 

************************************************************************/
void esp8266_send_data(u8 *data)
{
	u16 i;
	i=strlen((const char*)data);          //��ȡ���ݳ���
	u4_printf("AT+CIPSEND=0,%d\r\n",i);   //����AT+CIPSENDָ��
	delay_ms(5);                         
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);		//�������4���ջ�����	
	UART4_RX_LEN=0;                               
	u4_printf("%s",data);                         //���������ַ�����data
}

/***********************************************************************
 ������      ��void ESP8266_send_data(u8 id,u8 *data)
 ��������    ���������ݸ��豸��Ϊid���豸
 ����        ��id���豸�ţ�data��Ҫ���͵�����
 ���        ���� 

************************************************************************/
void ESP8266_send_data(u8 id,u8 *data)
{
	u16 i;
	i=strlen((const char*)data);                //��ȡ���ݳ���
	u4_printf("AT+CIPSEND=%d,%d\r\n",id,i);     //����AT+CIPSENDָ��
	delay_ms(5);                         
	memset(UART4_RX_BUF,0,50);	                //�������4���ջ�����	
	UART4_RX_LEN=0;                               
	u4_printf("%s",data);                       //���������ַ�����data	
	delay_ms(200);
	memset(UART4_RX_BUF,0,50);	                //�������4���ջ�����	
	UART4_RX_LEN=0; 	
}

/***********************************************************************
 ������      ��atk_8266_quit_trans(void)
 ��������    ���˳�͸��
 ����        ����
 ���        ��1:�ɹ� 0:ʧ��

************************************************************************/
u8 atk_8266_quit_trans(void)
{
	while((UART4->SR&0X40)==0);   //ѭ������"+",3��
	UART4->DR='+';      
	delay_ms(15);				
	while((UART4->SR&0X40)==0);	
	UART4->DR='+';      
	delay_ms(15);	
	while((UART4->SR&0X40)==0);	
	UART4->DR='+';      
	delay_ms(500);	
	return atk_8266_send_cmd("AT","OK",20);
}


/***********************************************************************
 ������      ��atk_8266_get_wanip(u8* ipbuf)
 ��������    ����ȡipbuf�У�����"\"��"\"�е�����
 ����        ����
 ���        ����

************************************************************************/
void atk_8266_get_wanip(u8* ipbuf)
{
	  u8 *p,*p1;
		if(atk_8266_send_cmd("AT+CIFSR","OK",50))
		{
			ipbuf[0]=0;
			return;
		}		
		p=atk_8266_check_cmd("\"");
		p1=(u8*)strstr((const char*)(p+1),"\"");
		*p1=0;
		sprintf((char*)ipbuf,"%s",p+1);	
}

/***********************************************************************
 ������      ��esp8266_get_ip(u8* ipbuf)
 ��������    ����ipbuf�л�ȡ·���������STM��IP��ַ������"\"��"\"�е�����
 ����        ��ipbuf����ȡIP������
 ���        ����

************************************************************************/
void esp8266_get_ip(u8* ipbuf)
{
	  u8 *p,*p1;	
		if(atk_8266_send_cmd("AT+CIFSR","OK",50))
		{
			ipbuf[0]=0;
			return;
		}		
		p=atk_8266_check_cmd("\"");
		p1=(u8*)strstr((const char*)(p+1),"\"");
		*p1=0;
		sprintf((char*)ipbuf,"%s",p+1);
		u2_printf("\r\n%s\r\n",ipbuf);		
}

/***********************************************************************
 ������      ��atk_8266_SET_AP(u8 *ssid,u8 *password,u8 mode,u16 timeout)
 ��������    ��esp8266����APģʽ
 ����        ��ssid,password,ģʽ����ʱʱ��
 ���        ��0��ʧ��  1���ɹ�

************************************************************************/
u8 atk_8266_SET_AP(u8 *ssid,u8 *password,u8 mode,u16 timeout)
{
	u8 res=0;
	UART4_RX_LEN=0;
	u16 time =0;
	u4_printf("AT+CWSAP=\"%s\",\"%s\",4,%d\r\n",ssid,password,mode);  //����ָ��
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
		break;
		delay_ms(10);
		time++;
	}
	if(time<timeout) res =1;
	return res;
}

/***********************************************************************
 ������      ��atk_8266_SET_STA(u8 *ssid,u8 *password,u16 timeout)
 ��������    ��esp8266����STAģʽ
 ����        ��ssid,password,ģʽ����ʱʱ��
 ���        ��0��ʧ��  1���ɹ�

************************************************************************/
u8 atk_8266_SET_STA(u8 *ssid,u8 *password,u16 timeout)
{	
	 u8 res=0;
	 UART4_RX_LEN=0;
	 u16 time =0;

	u4_printf("AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,password);  //����ָ��
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
	    break;
	    delay_ms(40);
		time++;
	}
	if(time<timeout) res =1;
	return res;	 
}

/***********************************************************************
 ������      ��atk_8266_SET_IPPORT(u16 timeout)
 ��������    ��esp8266���ö�·���ӣ�������IP�˿�Ϊ192.168.1.115
 ����        ����ʱʱ��
 ���        ��0��ʧ��  1���ɹ�

************************************************************************/
u8 atk_8266_SET_IPPORT(u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
	if(atk_8266_send_cmd("AT+CIPMUX=1","OK",50))
	{
		u2_printf("\r\n������·���ӳɹ�\r\n");		
	}
	else
	{
		u2_printf("\r\n������·����ʧ��\r\n");			
	}
		
	if(atk_8266_send_cmd("AT+CIPAP=\"192.168.1.115\"","OK",100))   //����IP��ַ��192.168.1.115
	{
		u2_printf("����IP�ɹ�");	
	}
	else
	{
		u2_printf("����IPʧ��");	
	}
	
	u4_printf("AT+CIPSERVER=1,8086\r\n");    //���ö˿ں�Ϊ8086
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
		break;
		delay_ms(10);
		time++;
	}
	if(time<timeout) res =1;
	return res;
}

/***********************************************************************
 ������      ��atk_8266_SET_IPPORT(u16 timeout)
 ��������    �����ö˿ں�
 ����        ����ʱʱ��
 ���        ��0��ʧ��  1���ɹ�

************************************************************************/
u8 atk_8266_SET_PORT(u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
//  if(atk_8266_send_cmd("AT+CIPMUX=1","OK",450))
//	{
//		 u2_printf("\r\n������·���ӳɹ�\r\n");
//	}
//	  else
//	{
//	  u2_printf("\r\n������·����ʧ��\r\n");		
//	}
		
	if(atk_8266_send_cmd("AT+CIPSERVER=1,8080","OK",50))
	{
		delay_ms(1000); u2_printf("\r\n�˿����óɹ�\r\n");res=1;
	}
	else
	{
		u2_printf("\r\n�˿�����ʧ��\r\n");	res=0;
	}
//	time=0;
//   while(time<timeout)
//	 {
//	    if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
//	    break;
//	    delay_ms(20);
//			time++;
//	 }
//	 if(time<timeout) res =1;
	return res;
}


/***********************************************************************
 ������      ��ESP8266_SET_IP(u8 *Ip,u16 timeout)
 ��������    �����ö˿ں�
 ����        ����ʱʱ��
 ���        ��0��ʧ��  1���ɹ�

************************************************************************/
u8 ESP8266_SET_IP(u8 *Ip,u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//�������4���ջ�����	
	
  u4_printf("AT+CIPAP=\"%s\"\r\n",Ip);  //����ָ��	
	delay_ms(100);
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
	    break;
	    delay_ms(40);
		time++;
	}
	if(time<timeout) res =1;
	return res;	 
}

/***********************************************************************
 ������      ��ESP8266_SET_PORT(u8 *Ip,u16 timeout)
 ��������    �����ö˿ں�
 ����        ����ʱʱ��
 ���        ��0��ʧ��  1���ɹ�

************************************************************************/
u8 ESP8266_SET_PORT(u16 Port,u16 timeout)
{
	u8 res = 0;
	UART4_RX_LEN=0;
	u16 time =0;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);	//�������4���ջ�����	
	
  u4_printf("AT+CIPSERVER=1,%d\r\n",Port);  //����ָ��	
	time=0;
	while(time<timeout)
	{
		if(strstr((const char *)UART4_RX_BUF,(const char *)"OK"))
	    break;
	    delay_ms(40);
		time++;
	}
	if(time<timeout) res =1;
	return res;	 
}




/***********************************************************************
 ������      ��esp8266_Get_IP(u16 timeout)
 ��������    ��
 ����        ����
 ���        ����

************************************************************************/
void esp8266_Get_IP(void)
{
	if(atk_8266_send_cmd("AT+CIFSR","OK",500))
		{u2_printf("��ȷ������IP��ַ");}
	else
		{u2_printf("��ȡ����ֵʧ��");}
	atk_8266_at_response(1);
}

/***********************************************************************
 ��������esp8266_AP_Init
 ���ܣ�  ��ʼ��esp8266��APģʽ
 ������  ��
 ����ֵ����
 ����: �ȵ�����ATK_ESP8266;���룺12345678;�˿ںţ�8086

***********************************************************************/
void esp8266_AP_Init(void)
{
  u8 i;
	for(i=0;i<1;i++)
	{
//		delay_ms(50);		
		if(atk_8266_send_cmd("AT+CWMODE=2","OK",60))
		  {u2_printf("\r\nAPģʽ���óɹ�\r\n");}
		else
		  {u2_printf("\r\nAPģʽ����ʧ��\r\n");}		
		if(atk_8266_send_cmd("AT+CIPMUX=1","OK",60))
		  {u2_printf("\r\n�����ӿ���\r\n");}
		else
		  {u2_printf("\r\n�����ӿ���ʧ��\r\n");}		
		if(atk_8266_SET_AP(SSID,PASS,WPA_WPA2_PSK,500))
		  { u2_printf("\r\n�����������\r\n");}
		else
		  {u2_printf("\r\n��������ʧ��r\n");}	
		if (atk_8266_SET_IPPORT(50))
		  {  u2_printf("\r\nIP�Ͷ˿��������\r\n");  }		   		
		else
		  {u2_printf("\r\nIP�Ͷ˿�����ʧ��\r\n");}	
	}
}

/***********************************************************************
 ��������ESP8266_AP_Init
 ���ܣ�  ��ʼ��esp8266��APģʽ
 ������  ��
 ����ֵ����
 ����: �ȵ�����ATK_ESP8266;���룺12345678;�˿ںţ�8086

***********************************************************************/
void ESP8266_AP_Init(u8 n)
{
	 u8 i;
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CWMODE=2","OK",60))
		 {
			u2_printf("\r\nģʽ���óɹ�\r\n");
			break;
		 }
		 else
		 {
			u2_printf("\r\n��%d��ģʽ���ó���ʧ�ܣ�׼��������������\r\n",(i+1));
		 }
		delay_ms(100);			 
	 }

	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CIPMUX=1","OK",60))
		 {
			u2_printf("\r\n�����ӿ���\r\n");
			break;
		 }
		 else
		 {	u2_printf("\r\n��%d�ζ����ӿ���ʧ�ܣ�׼��������������\r\n",(i+1)); }
		 delay_ms(100);			 
	 }	 

	 for(i=0;i<n;i++)
	 {
			if(atk_8266_SET_AP(wifi_ssid,wifi_ssid_password,WPA_WPA2_PSK,500))
			{
				delay_ms(200);	
				u2_printf("\r\nAP���óɹ�\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n��%d��AP���ó���ʧ�ܣ�׼��������������\r\n",i+1);
			}
			delay_ms(100);	
	 }	 

	 for(i=0;i<n;i++)
	 {
			if(ESP8266_SET_IP(wifi_ap_ip,60))
			{
				u2_printf("\r\nIP���óɹ�\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n��%d��IP���ó���ʧ�ܣ�׼��������������\r\n",i+1);
			}		
			delay_ms(500);			
	 }

	 for(i=0;i<n;i++)
	 {		 
			if(ESP8266_SET_PORT(wifi_ap_port,50))
			{
				u2_printf("\r\n�˿����óɹ�\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n��%d�ζ˿����ó���ʧ�ܣ�׼��������������\r\n",i+1);
			}
			delay_ms(1000);					
	 }
}




/***********************************************************************
 ������    ��esp8266_STA_SERVER_Init
 ����      ����ʼ��esp8266��STAģʽ
 ����      ����
 ����ֵ    ����
 ����: 

***********************************************************************/
void esp8266_STA_SERVER_Init(void)
{
  u8 i;
	for(i=0;i<2;i++)
	{
		delay_ms(100);		
		if(atk_8266_send_cmd("AT+CWMODE=1","OK",500))
			{u2_printf("\r\nģʽ���óɹ�\r\n");}
		else
			{u2_printf("\r\nģʽ����ʧ��\r\n");}	
			
		if(atk_8266_send_cmd("AT+RST","OK",3000))
			{u2_printf("\r\n���óɹ�\r\n");}
		else
			{u2_printf("\r\n����ʧ��\r\n");}
			
		delay_ms(5000);
					
		if(atk_8266_SET_STA(WIFI_STATION,PASSWORD,600))
			{delay_ms(5000);u2_printf("\r\nWiFi���ӳɹ�\r\n");}
		else
			{u2_printf("\r\nWiFi����ʧ��\r\n");}
			
		delay_ms(3000);

		if(atk_8266_SET_PORT(600))
			{ u2_printf("\r\n �˿����óɹ� \r\n"); }		  		
		else	
			{ u2_printf("\r\n �˿����óɹ� \r\n"); }		
	}
}

/***********************************************************************
 ������    ��esp_8266_apsta_Init
 ����      ����ʼ��esp8266��AP������-TCP������ģʽ
 ����      ��n,ʧ����������
 ����ֵ    ����
 ����: 

***********************************************************************/
void esp_8266_apsta_Init(u8 n)
{
   u8 i;
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CWMODE=3","OK",100))
		 {
			u2_printf("\r\nģʽ���óɹ�\r\n");
			break;
		 }
		 else
		 {
			u2_printf("\r\n��%d��ģʽ���ó���ʧ�ܣ�׼��������������\r\n",(i+1));
		 }
	 }
	for(i=0;i<n;i++)
	{
		if(atk_8266_send_cmd("AT+RST","OK",100))
		{
			u2_printf("\r\n�������óɹ�\r\n");
			delay_ms(2000);	
			break;
		}
		else
		{
			u2_printf("\r\n��%d����������ʧ�ܣ�׼��������������\r\n",i+1);
		}
//			 delay_ms(1000);		 
	}
	for(i=0;i<n;i++)
	{
		if(atk_8266_SET_AP(wifi_ssid,wifi_ssid_password,WPA_WPA2_PSK,500))
		{
			u2_printf("\r\nAP���óɹ�\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n��%d��AP���ó���ʧ�ܣ�׼��������������\r\n",i+1);
		}
			delay_ms(100);
	 }

	 for(i=0;i<n;i++)
	 {
			if(ESP8266_SET_IP(wifi_ap_ip,60))
			{
				u2_printf("\r\nIP���óɹ�\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n��%d��IP���ó���ʧ�ܣ�׼��������������\r\n",i+1);
			}		
			delay_ms(500);			
	 }

	 for(i=0;i<n;i++)
	 {		 
			if(ESP8266_SET_PORT(wifi_ap_port,50))
			{
				u2_printf("\r\n�˿����óɹ�\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n��%d�ζ˿����ó���ʧ�ܣ�׼��������������\r\n",i+1);
			}
			delay_ms(1000);					
	 }

	
	for(i=0;i<n;i++)
	{
		if(atk_8266_SET_STA(wifi_station,wifi_station_password,700))
		{
			delay_ms(4000);
			u2_printf("\r\nWIFI-STATION���óɹ�\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n��%d��WIFI-STATION���ó���ʧ�ܣ�׼��������������\r\n",i+1);
		}
	 }
	
	 for(i=0;i<n;i++)
	 {
//	   delay_ms(50);
		if(atk_8266_SET_PORT(200))
		{
			u2_printf("\r\n�������˿����óɹ�\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n��%d�η������˿�����ʧ�ܣ�׼��������������\r\n",i+1);
		}
	 }	 
	 for(i=0;i<n;i++)
	 {
		delay_ms(20);
		if(atk_8266_send_cmd("AT+CIPSTO=2200","OK",500))
		{
			u2_printf("\r\n��ʱʱ�����óɹ�\r\n");
			break;
		}
		else
		{
			u2_printf("\r\n��%d�γ�ʱʱ�����ó���ʧ�ܣ�׼��������������\r\n",i+1);
		}
	 } 
//	 esp8266_get_ip(wifi_ip_address);
	 delay_ms(500);
	 esp8266_Get_IP();
	 u2_printf("\r\nready\r\n");
}


/***********************************************************************
 ������   ��get_wifiname_wifipassword(u8 *ip1,u8 *ip2)
 ����     ����ȡ�ַ����е�WiFi���ƺ�����
 ����     ��ip1������WiFi���ƣ�ip2������WiFi����
 ���     �� 

***********************************************************************/
void get_wifiname_wifipassword(u8 *ip1,u8 *ip2)
{
	u8 receive_data[20];
	char *presult1,*presult2,*presult3;
	USART2_RX_LEN=0;
	memset(ip1,0,20);
	memset(ip2,0,20);
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	if( presult1 != NULL )
	{
	  presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	  if( presult2 != NULL )
		{
			memcpy(ip1,presult1+1,presult2-presult1-1);
			W25QXX_Write((u8 *)ip1,60,presult2-presult1-1);
		  u2_printf("\r\nWIFI�ȵ�����:%s\r\n",ip1);
			memset(receive_data,0,20);			
			W25QXX_Read((u8*)receive_data,60,presult2-presult1-1); 
		  u2_printf("\r\n����WIFI�ȵ�����:%s\r\n",receive_data);
			memset(receive_data,0,20);			
			presult3 = strstr( (const char *)presult2+1 ,(const char *)"+");
			if( presult3 != NULL )
			{
				memcpy(ip2,presult2+1,presult3-presult2-1);
				W25QXX_Write((u8 *)ip2,90,presult3-presult2-1);
				u2_printf("\r\nWIFI����:%s\r\n",ip2);
				W25QXX_Read((u8*)receive_data,90,presult3-presult2-1); 
				u2_printf("\r\n����WIFI��������:%s\r\n",receive_data);					
			}				
		}		
	}
}

/***********************************************************************
 ������   ��get_apname_wifipassword(u8 *ip1,u8 *ip2)
 ����     ����ȡ�ַ����е�WiFi���ƺ�����
 ����     ��ip1������WiFi���ƣ�ip2������WiFi����
 ���     �� 
***********************************************************************/
void get_apname_wifipassword(u8 *ip1,u8 *ip2)
{
	u8 receive_data[20];
	char *presult1,*presult2,*presult3;
	USART2_RX_LEN=0;
	memset(ip1,0,20);
	memset(ip2,0,20);
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	if( presult1 != NULL )
	{
	  presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	  if( presult2 != NULL )
		{
			memcpy(ip1,presult1+1,presult2-presult1-1);
			W25QXX_Write((u8 *)ip1,120,presult2-presult1-1);
		  u2_printf("\r\n�趨AP-WIFI�ȵ�����:%s\r\n",ip1);	
			memset(receive_data,0,20);			
			W25QXX_Read((u8*)receive_data,120,presult2-presult1-1); 
		  u2_printf("\r\n����WIFI�ȵ�����:%s\r\n",receive_data);	
			memset(receive_data,0,20);
			presult3 = strstr( (const char *)presult2+1 ,(const char *)"+");
			if( presult3 != NULL )
			{
					memcpy(ip2,presult2+1,presult3-presult2-1);
					W25QXX_Write((u8 *)ip2,150,presult3-presult2-1);
					u2_printf("\r\n�趨AP-WIFI����:%s\r\n",ip2);
					W25QXX_Read((u8*)receive_data,150,presult3-presult2-1); 
					u2_printf("\r\n����WIFI��������:%s\r\n",receive_data);					
			}				
		}		
	}
}

/***********************************************************************
 ������   ��get_ip_port(u8 *ip1,u16 data)
 ����     ����ȡ�ַ����е�WiFi���ƺ�����
 ����     ��ip1������WiFi���ƣ�ip2������WiFi����
 ���     �� 
***********************************************************************/
void get_ip_port(u8 *ip1,u16 data)
{
	u8 receive_data[15];
	u8 tem[5];
	u8 portdata[2];
	u16 test;
	char *presult1,*presult2,*presult3;
	USART2_RX_LEN=0;
	memset(ip1,0,20);
	
	presult1 = strstr( (const char *)USART2_RX_BUF ,(const char *)"+");
	if( presult1 != NULL )
	{
	  presult2 = strstr( (const char *)presult1+1 , (const char *)"+");
	  if( presult2 != NULL )
		{
			memcpy(ip1,presult1+1,presult2-presult1-1);
			W25QXX_Write((u8 *)ip1,180,presult2-presult1-1);
		  u2_printf("\r\n�趨IP��ַΪ:%s\r\n",ip1);	
			memset(receive_data,0,15);
			
			W25QXX_Read((u8*)receive_data,180,presult2-presult1-1); 
		  u2_printf("\r\n����IP��ַΪ:%s\r\n",receive_data);	
			memset(receive_data,0,15);
			
			presult3 = strstr( (const char *)presult2+1 ,(const char *)"+");
			if( presult3 != NULL )
			{
					memcpy(tem,presult2+1,presult3-presult2-1);
				  data=usmart_strnum(tem);
				  wifi_ap_port=data;
					portdata[0]=data>>8;   //��8λ
					portdata[1]=data;   	 //��8λ				
					W25QXX_Write((u8 *)portdata,195,2);
					u2_printf("\r\n�趨�˿����Ϊ:%d\r\n",data);
					memset(portdata,0,2);			
					W25QXX_Read((u8*)portdata,195,2);
					test=	portdata[0]<<8;
					test=test|portdata[1];			
					u2_printf("\r\n����Ķ˿�Ϊ%d\r\n",test);					
			}				
		}		
	}
}


/***********************************************************************
 ������    ��ESP8266_apsta_Init
 ����      ����ʼ��esp8266��AP������-TCP������ģʽ
 ����      ��n,ʧ����������
 ����ֵ    ����
 ����      ��
***********************************************************************/
void ESP8266_apsta_Init(u8 n)
{
   u8 i;
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+CWMODE=3","OK",100))
		 {
		   u2_printf("\r\nģʽ���óɹ�\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n��%d��ģʽ���ó���ʧ�ܣ�׼��������������\r\n",(i+1));
		 }
	 }
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_send_cmd("AT+RST","OK",100))
		 {
		   u2_printf("\r\n�������óɹ�\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n��%d����������ʧ�ܣ�׼��������������\r\n",i+1);
		 }	 
	 }
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_SET_AP(wifi_ssid,wifi_ssid_password,WPA_WPA2_PSK,500))
		 {
		   u2_printf("\r\nAP���óɹ�\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n��%d��AP���ó���ʧ�ܣ�׼��������������\r\n",i+1);
		 }
		 delay_ms(100);
	 }
	 
	 for(i=0;i<n;i++)
	 {
			if(ESP8266_SET_IP(wifi_ap_ip,60))
			{
				u2_printf("\r\nIP���óɹ�\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n��%d��IP���ó���ʧ�ܣ�׼��������������\r\n",i+1);
			}		
			delay_ms(1000);			
	 }
	 
	 for(i=0;i<n;i++)
	 {
		 if(atk_8266_SET_STA(wifi_station,wifi_station_password,3000))
		 {
		   u2_printf("\r\nWIFI-STATION���óɹ�\r\n");
			 delay_ms(9000);
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n��%d��WIFI-STATION���ó���ʧ�ܣ�׼��������������\r\n",i+1);
		 }
		 memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		 delay_ms(8000);
	 }

	 for(i=0;i<n;i++)
	 {
			if(atk_8266_send_cmd("AT+CIPMUX=1","OK",450))
			{
				 u2_printf("\r\n������·���ӳɹ�\r\n");
				 break;
			}
				else
			{
				u2_printf("\r\n��%d�ζ�·����ʧ�ܣ�׼��������������\r\n",i+1);		
			}
		 memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		 delay_ms(1000);
	 }	

	 for(i=0;i<n;i++)
	 {		 
			if(ESP8266_SET_PORT(wifi_ap_port,50))
			{
				u2_printf("\r\n�˿����óɹ�\r\n");
				break;
			}
			else
			{
				u2_printf("\r\n��%d�ζ˿����ó���ʧ�ܣ�׼��������������\r\n",i+1);
			}
			delay_ms(1000);					
	 }	 

	 
	 for(i=0;i<n;i++)
	 {
	   delay_ms(20);
		 if(atk_8266_send_cmd("AT+CIPSTO=2200","OK",500))
		 {
		   u2_printf("\r\n��ʱʱ�����óɹ�\r\n");
			 break;
		 }
		 else
		 {
		   u2_printf("\r\n��%d�γ�ʱʱ�����ó���ʧ�ܣ�׼��������������\r\n",i+1);
		 }
		 memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);			
		 delay_ms(1000);
	 } 
	 esp8266_Get_IP();
	 u2_printf("\r\nready\r\n");
}

/***********************************************************************
 ������    ��ESP8266_Get_Controler_Type()
 ����      ����WiFi�豸����ʱ��������Ӧ���豸��
 ����      ����ʱʱ��
 ���      ����
 ����      ��

***********************************************************************/
void ESP8266_Get_Controler_Type(u16 timeout)
{
	u8 ID,num,len;
	u8 time=0;
	num=1;
	ID=UART4_RX_BUF[0]-48;   //���Ƶ�ת��
	USART2->DR =ID;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	
	u2_printf("\r\nID=%x,num=%d\r\n",ID,num);
	UART4_RX_LEN=0;
	ESP8266_send_data(ID,"who_are_you");
	delay_ms(300);
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	UART4_RX_LEN=0;	
	TIM10_Init(30000,21000);
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
	while(!(__HAL_TIM_GET_FLAG(&TIM10_Handler, TIM_SR_CC1IF) ) )
	{
		if(UART4_RX_LEN&0x8000)
		{
			len=UART4_RX_LEN&0x3fff;				
			UART4_RX_BUF[len]=0;		
			if(strstr((const char *)UART4_RX_BUF,(const char *)"Phone"))  //�ֻ�
			{
				 Phone=ID;
				Phone_Ready=1;
				u2_printf("\r\nPhone=%d\r\n",Phone);				
				break;
			}		
			
			if(strstr((const char *)UART4_RX_BUF,(const char *)"Remote"))  //ң����
			{			
				Remote_Ctl =ID;
				RemoteCtl_Ready=1;
				u2_printf("\r\nRemote=%d\r\n",Remote_Ctl);					
				break;
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"Guard"))  //����
			{			
				Guard_Ctl= ID;
				GuardCtl_Ready=1;				
				u2_printf("\r\nGuard=%d\r\n",Guard_Ctl);					
				break;
			}

			if(strstr((const char *)UART4_RX_BUF,(const char *)"PC"))  //����
			{		
				 PC =ID;
				PC_Ready=1;				
				u2_printf("\r\nPC=%d\r\n",PC);					
				break;
			}
		 }	
	 }
	__HAL_TIM_CLEAR_FLAG(&TIM10_Handler, TIM_SR_CC1IF); // ����жϱ�־λ	 	
	u2_printf("\r\nfinish\r\n");
}

/***********************************************************************
 ������    ��ESP8266_Close_Controler_Type()
 ����      ��wifi�豸�˳�ʱ���ͷŷ�����豸��
 ����      ����
 ���      ����
 ����      ��
***********************************************************************/
void ESP8266_Close_Controler_Type(void)
{
	u8 ID,len;
	u8 time=0;
	ID=UART4_RX_BUF[0]-48;   //���Ƶ�ת��
	USART2->DR =ID;
	memset(UART4_RX_BUF,0,UART4_MAX_RECV_LEN);
	UART4_RX_LEN=0;	
	u2_printf("\r\nID=%x\r\n",ID);

	if(ID==Phone)
	{Phone=6;Phone_Ready=0;u2_printf("�ָ�Phone\r\n");}
	if(ID==Remote_Ctl)
	{Remote_Ctl=6;RemoteCtl_Ready=0;u2_printf("�ָ�Remote_Ctl\r\n");}
	if(ID==Guard_Ctl)
	{Guard_Ctl=6;	GuardCtl_Ready=0;	u2_printf("�ָ�Guard_Ctl\r\n");}
	if(ID==PC)
	{PC=6;	PC_Ready=0;	u2_printf("�ָ�PC\r\n");}	
	u2_printf("\r\nPhone=%d,Remote_Ctl=%d,Guard_Ctl=%d,PC=%d\r\n",Phone,Remote_Ctl,Guard_Ctl,Guard_Ctl,PC);
}

/***********************************************************************
 ������    ��ESP8266_Get_ID()
 ����      �����յ���ǰ�豸���͵�ָ���������䷵��һ��Receivedָ��
 ����      ����
 ���      ��0��ʧ�ܣ�1���ɹ�
 ����      ��
***********************************************************************/
u8 ESP8266_Get_ID(void)
{
	u8 i,len;
	u8 res=0;                     //��������ֵ
	u8 temp_buf[50];              //��������WiFi���յĹ���ָ��
	ID_NUM=UART4_RX_BUF[7]-48;    //��ȡ����ָ����豸��
	len=UART4_RX_LEN&0x3fff;      //WiFi���յ��ַ�������
	for(i=0;i<len;i++)            //����UART4_RX_BUF�е�����
	{
		temp_buf[i]=UART4_RX_BUF[i+2];             
	}
	if((ID_NUM==0)||(ID_NUM==1)||(ID_NUM==2)||(ID_NUM==3)||(ID_NUM==4))
	{
		res=1;
		ESP8266_send_data(ID_NUM,"Received");   //����ָ����豸����Received����
	}
	else
	{	res=0;   }
	
	for(i=0;i<len;i++)
	{
		UART4_RX_BUF[i]=temp_buf[i];           //��WiFi���ܵĹ���ָ������д��UART4_RX_BUF��
	}	
	return res;
}

/***********************************************************************
 ������    ��Flash_Read_STA()
 ����      ����Flash�ڶ�ȡ�����·�������ƺ�����
 ����      ����
 ���      ����
 ����      ��
***********************************************************************/
void Flash_Read_STA(void)
{
	u8 receive_data[20];
	memset(receive_data,0,20);
	W25QXX_Read(receive_data,60,20);				//��spi flash�е�60λ��ַ��ʼ��ȡ20���ֽ�����
	u2_printf("\r\nflash�ڵ�sta_wifi����:%s\r\n",receive_data);
	W25QXX_Read(receive_data,90,20);				//��spi flash�е�90λ��ַ��ʼ��ȡ20���ֽ�����
	u2_printf("\r\nflash�ڵ�sta_wifi����:%s\r\n",receive_data);
}


/***********************************************************************
 ������    ��Flash_Read_AP()
 ����      ����Flash�ڶ�ȡ������ȵ����ƺͶ�Ӧ����
 ����      ����
 ���      ����
 ����      ��
***********************************************************************/
void Flash_Read_AP(void)
{
	u8 receive_data[20];
	memset(receive_data,0,20);
	W25QXX_Read(receive_data,120,20);  //��spi flash�е�120λ��ַ��ʼ��ȡ20���ֽ�����
	u2_printf("\r\nflash�ڵ�sta_wifi����:%s\r\n",receive_data);
	memset(receive_data,0,20);	
	W25QXX_Read(receive_data,150,20);		//��spi flash�е�150λ��ַ��ʼ��ȡ20���ֽ�����
	u2_printf("\r\nflash�ڵ�sta_wifi����:%s\r\n",receive_data);
}


/***********************************************************************
 ������    ��Flash_Read_AP_IPPORT()
 ����      ����Flash�ڶ�ȡ�����IP��ַ�Ͷ˿ں�
 ����      ����
 ���      ����
 ����      ��
***********************************************************************/
void Flash_Read_AP_IPPORT(void)
{
	u8 receive_data[15];
	u8 receive_tem[2];
	u16 receive_port;
	memset(receive_data,0,15);
	W25QXX_Read(receive_data,180,20);		   //��spi flash�е�180λ��ַ��ʼ��ȡ20���ֽ�����
	u2_printf("\r\nflash�ڵ�APģʽIP��ַΪ:%s\r\n",receive_data);
	memset(receive_tem,0,2);
	W25QXX_Read(receive_tem,195,2);	      //��spi flash�е�195λ��ַ��ʼ��ȡ2���ֽ�����
	receive_port=	receive_tem[0]<<8;           //��ȡ��8λ
	receive_port=receive_port|receive_tem[1];			//��8λ���8λ���16λ����
	u2_printf("\r\nflash�ڵĶ˿�Ϊ%d\r\n",receive_port);					
}

 

/***********************************************************************
 ������    ��stick_send_cmd()
 ����      ��ͨ�����ڣ���PC��������Ϣ���ȴ�PC����Ӧ��
 ����      ����
 ���      ��0��Ӧ��ʧ��  1��Ӧ��ɹ�
 ����      ��
***********************************************************************/
u8 stick_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u16 time=0;
	u8 res=0; 
	USART2_RX_LEN=0;
	u2_printf("%s\r\n",cmd); 
	time = 0;
	while(time<waittime)	
	{
			delay_us(10);		
			if(stick_check_cmd(ack))       //�жϷ���ֵ
			{
				break;
			}	
			time++;				
	}
	if(time<waittime)res=1;  //����ڳ�ʱʱ����break��������res=1	
	return res;
} 


/***********************************************************************
 ������    ��stick_check_cmd()
 ����      ���жϴ��ڷ���ֵ���趨ֵ�Ƿ�һ��
 ����      ����
 ���      ��0����һ��   1��һ��
 ����      ��
***********************************************************************/
u8* stick_check_cmd(u8 *str)
{	
	char *strx=0;
	if(USART2_RX_LEN&0X8000)	
	{ 
		USART2_RX_BUF[USART2_RX_LEN&0X7FFF]=0;
		strx=strstr((const char*)USART2_RX_BUF,(const char*)str); 		
	} 
	return (u8*)strx;
}


