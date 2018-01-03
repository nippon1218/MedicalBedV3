#ifndef __MAINJUDGE_
#define __MAINJUDGE_
#include "sys.h"
#include "usart.h"


//WiFi����-�ȵ�-Ĭ�����ƺ�����
extern u8 wifi_ssid[20];	    //�ȵ�����
extern u8 wifi_ssid_password[20];	    //�ȵ�����    

//WiFi����-·��-Ĭ��ֵ
extern u8 wifi_station[20];					//·��������
extern u8 wifi_station_password[20];	//·��������
extern u8 wifi_ap_ip[15];          //IP��ַ
extern u16 wifi_ap_port;                      //�˿ں�

//WiFi��������
extern u8 wifi_ip_address[20];                 //ip��ַ
extern u8 wifi_name[20];                       //WiFi����
extern u8 wifi_password[20];                   //����

extern const	u8 modeap_buf[2];            //apģʽд�����飬��ģʽд��AP
extern const	u8 modesta_buf[2];           //STAģʽд������,��ģʽд��ST



void WifiReceiveJudge(void);
void Usart2ReceiveJudge(void);
void AllInit(void);



#endif


