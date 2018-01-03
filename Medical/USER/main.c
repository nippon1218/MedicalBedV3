#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"
#include "nand.h"
#include "key.h"
#include "function.h"
#include "common.h"
#include "pcf8574.h"
#include "ds18b20.h"
#include "check_cmd.h"
#include "motor.h"
#include "pump.h"
#include "hx711.h"
#include "modbus_master.h"
#include "bsp_user_lib.h"
#include "spi.h"
#include "washlet.h"
#include "w25qxx.h"
#include "check.h"
#include "Hang.h"
#include "washlet.h"
#include "reset.h"
#include "mainjudge.h"

//��λ���趨�Ƕȼ���ֵ-Ĭ��ֵ
u8  back_angle_lim=90;                      //��λ���趨֧�����нǶ�
u8  leg_up_angle_lim=30;                    //��λ���趨���������нǶ�
u8  leg_down_angle_lim=80;                  //��λ���趨���������нǶ�
u8  body_left_angle_lim=50;                 //��λ���趨�����нǶ�
u8  body_right_angle_lim=55;                //��λ���趨�ҷ����нǶ�
u8  desk_distance_lim=100;                   //��λ���趨����С�����˶�����
u8  swash_dry_time=1;                       //��λ���趨���������ϴ���ʱ��/����


//WiFi����-�ȵ�-Ĭ�����ƺ�����
u8 wifi_ssid[20]="Medical_Bed";	    //�ȵ�����
u8 wifi_ssid_password[20]="12345678";	    //�ȵ�����    

//WiFi����-·��-Ĭ��ֵ
u8 wifi_station[20]="tmqs";					//·��������
u8 wifi_station_password[20]="daizhiwen";	//·��������
u8 wifi_ap_ip[15]="192.168.1.115";          //IP��ַ
u16 wifi_ap_port=8086;                      //�˿ں�

//WiFi��������
u8 wifi_ip_address[20]={0};                 //ip��ַ
u8 wifi_name[20]={0};                       //WiFi����
u8 wifi_password[20]={0};                   //����

const	u8 modeap_buf[2]={"AP"};            //apģʽд�����飬��ģʽд��AP
const	u8 modesta_buf[2]={"ST"};           //STAģʽд������,��ģʽд��ST

int main(void)
{
	u8 num;                                //��¼���ӵ��豸��
	u8 k;
	u8 len;                                //���ڽ��յ��ַ�������
	u8 key;                                //����ɨ�践��ֵ
	u8 mode_buf[2];                        //�洢WiFiģʽ��AP����STAģʽ
	u8 angle_buf[14];                      //�洢��λ�趨�Ƕ�ֵ

	AllInit();
//	Read_Angle();                          //��ȡ��λ���趨�Ƕ�ֵ
	
//�ж��ϵ�ʱ��WiFiģʽ-Ĭ��ΪAPģʽ
//	W25QXX_Read(mode_buf,10,2);            //�ӵ�10����ַ��ʼ����2���ֽڣ��ж���AP����STAģʽ
//	if(strstr((const char *)mode_buf,(const char *)"ST"))	  //����·������������Ϊ·����ģʽ	
//	{
//		ESP8266_apsta_Init(4);
//	}
//	else      //Ĭ��ΪAPģʽ
//	{
//		ESP8266_AP_Init(4);		
//	}
//PCF8574_WriteBit(EXIO1,0);
//	ESP8266_AP_Init(4);	

//��մ���2������WiFi���ռĴ���
	UART4_RX_LEN=0;      
	USART2_RX_LEN=0;			

	PCF8574_WriteBit(BEEP_IO,0);	//���Ʒ�������
	delay_ms(300);
	PCF8574_WriteBit(BEEP_IO,1);	//���Ʒ�����ͣ
	LED0=0;
	LED1=0;	
//	IO_TEST();
<<<<<<< HEAD

=======
>>>>>>> Test1
//			washlet_flag=1;
//			RELAY6=1; 
//			Uart_Push_Rod_Swash(0,25000);  //��ϴ����Ƹ�����
//			RELAY6=0;	 
//			washlet_flag=0;

//		GDSCheckAllAlm(2);
    while(1)
    {
			WifiReceiveJudge();		//ң���������жϺ���
			Usart2ReceiveJudge();	//����2���պ���
		}	
}
