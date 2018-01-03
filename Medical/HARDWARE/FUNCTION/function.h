#ifndef __FUN_H
#define __FUN_H
#include "sys.h"
#include "usart.h"

extern u8 device_num;              //���ӵ�WiFi�������ն˵ĸ���

/********************���������������Ƕ�********************************/

extern u8  back_angle_max;         //��λ���趨֧�����нǶ�
extern u8  leg_up_angle_max;       //��λ���趨���������нǶ�
extern u8  leg_down_angle_max;     //��λ���趨���������нǶ�
extern u8  body_left_angle_max;    //��λ���趨�����нǶ�
extern u8  body_right_angle_max;   //��λ���趨�ҷ����нǶ�
extern u8  desk_distance_max;      //��λ���趨����С�����˶�����
extern u8  swash_dry_time_max;     //��λ���趨���������ϴ���ʱ��/����

/*****************��λ״̬��־λ��1��δ��λ �� 0����λ����ʼ״̬***********/

extern u8 back_flag;               //֧��
extern u8 leg_up_flag;             //������
extern u8 leg_down_flag;           //������
extern u8 body_left_flag;          //��
extern u8 body_right_flag;         //�ҷ�
extern u8 back_nursing_left_flag;  //�󱳲�����
extern u8 back_nursing_right_flag; //�ұ�������
extern u8 waist_nursing_left_flag; //����������
extern u8 waist_nursing_right_flag;//����������
extern u8 washlet_flag;            //������
extern u8 washlet_auto_flag;       //�Զ�������
extern u8 desk_flag;               //�Ͳ�����һ����
extern u8 jram_flag;               //���ⰴĦ
extern u8 swash_dry_flag;          //��ϴ���
extern u8 lock_flag;               //һ����������
extern u8 fault_flag;              //������ϱ�־λ
extern u8 desk_front_flag;

extern u8 back_dir_flag;

//
extern u8 back_nursing_left_dir_flag;       //�󱳲��������־λ
extern u8 back_nursing_right_dir_flag;      //�ұ����������־λ
extern u8 waist_nursing_left_dir_flag;      //�������������־λ
extern u8 waist_nursing_right_dir_flag;     //�������������־λ
extern u16 body_left_runed_arr;           //�󷭣���������Զ���װ��ֵ
extern u16 body_right_runed_arr;          //�ҷ�����������Զ���װ��ֵ
extern u16 desk_runed_arr;								//���ӣ���������Զ���װ��ֵ




extern u8 swash_hand_flag;         //�ֶ���ϴ��־λ
extern u8 dry_hand_flag;           //�ֶ���ɱ�־λ

extern u8 leg_down_state_flag;     //�Զ�����ʱ��¼�����Ƿ��Ѵ��ڶ���״̬
extern u8 back_state_flag;         //�Զ�����ʱ��¼֧���Ƿ��Ѵ��ڶ���״̬

extern u8 armleg_left_flag;        //�ֶ���֫
extern u8 armleg_right_flag;       //�ֶ���֫
extern u8 armleg_left_right_flag;  //�ֶ�����֫

extern u8 arm_fore_left_flag;      //��С��
extern u8 leg_fore_left_flag;      //��С��

extern u8 arm_fore_right_flag;     //��С��
extern u8 leg_fore_right_flag;     //��С��

extern u8 arm_post_left_flag;      //����
extern u8 leg_post_left_flag;      //�����

extern u8 arm_post_right_flag;     //�Ҵ��
extern u8 leg_post_right_flag;     //�Ҵ���

extern u8 arm_fore_post_left_flag;       //���С��
extern u8 leg_fore_post_left_flag;       //���С��

extern u8 arm_fore_post_right_flag;      //�Ҵ�С��
extern u8 leg_fore_post_right_flag;      //�Ҵ�С��

extern u8 arm_fore_left_right_flag;      //����С��
extern u8 leg_fore_left_right_flag;      //����С��

extern u8 arm_post_left_right_flag;      //���Ҵ��
extern u8 leg_post_left_right_flag;      //���Ҵ���

extern u8 arm_fore_post_left_right_flag; //���Ҵ�С��
extern u8 leg_fore_post_left_right_flag; //���Ҵ�С��

/****************����״̬��־λ******************************/

//����������-�������

extern u8 body_left_overload_3;        //����
extern u8 body_left_overload_4;        //����
extern u8 body_left_overload_5;        //����

extern u8 body_right_overload_3;       //�ҷ���
extern u8 body_right_overload_4;       //�ҷ���
extern u8 body_right_overload_5;       //�ҷ���

extern u8 washlet_auto_overload;       //�Զ�����
extern u8 desk_overload;               //�Ͳ�����һ����
extern u8 back_nursing_left_overload;  //�󱳲�����
extern u8 back_nursing_right_overload; //�ұ�������
extern u8 waist_nursing_left_overload; //����������
extern u8 waist_nursing_right_overload;//����������

//����������-���ʧ��
extern u8 body_left_losepulse;          //����
extern u8 body_right_losepulse;         //�ҷ���
extern u8 washlet_auto_losepulse;       //�Զ�����
extern u8 desk_losepulse;               //�Ͳ�����һ����
extern u8 back_nursing_left_losepulse;  //�󱳲�����
extern u8 back_nursing_right_losepulse; //�ұ�������
extern u8 waist_nursing_left_losepulse; //����������
extern u8 waist_nursing_right_losepulse;//����������

//�������г��ֶ��������־λ
extern u8 back_interfere;                 //֧��
extern u8 leg_up_interfere;               //������
extern u8 leg_down_interfere;             //������    
extern u8 leg_interfere;                  //����
extern u8 body_left_interfere;            //����
extern u8 body_right_interfere;           //�ҷ���
extern u8 washlet_auto_interfere;         //������
extern u8 desk_interfere;                 //С����
extern u8 back_nursing_left_interfere;    //�󱳲�����
extern u8 back_nursing_right_interfere;   //�ұ�������
extern u8 waist_nursing_left_interfere;   //����������
extern u8 waist_nursing_right_interfere;  //����������

/****************����ͼƬ******************************/
extern u8 back_picture_k;                 //֧��
extern u8 leg_up_picture_k;               //������
extern u8 leg_down_picture_k;             //������
extern u8 desk_picture_k;                 //С����
extern u8 washlet_picture_k;              //������
extern u8 body_left_picture_k;            //����
extern u8 left_motor5_picture_m;          //��С�෭
extern u8 body_right_picture_k;           //�ҷ���
extern u8 right_motor5_picture_m;         //��С�෭

extern u8 back_nursing_left_picture_k;    //�󱳲�����
extern u8 waist_nursing_left_picture_k;   //����������
extern u8 back_nursing_right_picture_k;   //�ұ�������
extern u8 waist_nursing_right_picture_k;  //����������




/***************�����趨�������ü������в���*********************/

extern u8  back_angle_lim;                 //��λ���趨֧�����нǶ�
extern u8  leg_up_angle_lim;               //��λ���趨���������нǶ�
extern u8  leg_down_angle_lim;             //��λ���趨���������нǶ�
extern u8  body_left_angle_lim;            //��λ���趨�����нǶ�
extern u8  body_right_angle_lim;           //��λ���趨�ҷ����нǶ�
extern u8  desk_distance_lim;              //��λ���趨����С�����˶�����
extern u8  swash_dry_time;                 //��λ���趨���������ϴ���ʱ��
extern u16 washlet_arr_lim;                //�����������£���ʱ����arr����װ��ֵ��-6�ŵ��

/****************���ҿ���ѵ������ֵ����*******************/

extern u16 back_runed_arr;                 //֧������������Զ���װ��ֵ
extern u16 leg_up_runed_arr;
extern u16 leg_down_runed_arr;
//��֫
extern u32 arm_left_runed;                 //�����ʼλ��������������   
extern u32 arm_left_lim;                   //��������
extern u32 leg_left_runed;                 //�����ʼλ��������������   
extern u32 leg_left_lim;                   //��������

//��֫
extern u32 arm_right_runed;                //�����ʼλ��������������  
extern u32 arm_right_lim;                  //��������
extern u32 leg_right_runed;                //�����ʼλ��������������  
extern u32 leg_right_lim;                  //��������

//����֫
extern u32 arm_left_right_runed;           //�����ʼλ��������������  
extern u32 arm_left_right_lim;             //��������
extern u32 leg_left_right_runed;           //�����ʼλ��������������  
extern u32 leg_left_right_lim;             //��������

//��С��/С��
extern u32 arm_fore_left_runed;            //�����ʼλ��������������  
extern u32 arm_fore_left_lim;              //��������
extern u32 leg_fore_left_runed;            //�����ʼλ��������������  
extern u32 leg_fore_left_lim;              //��������

//����/����
extern u32 arm_post_left_lim;              //��������    
extern u32 arm_post_left_runed;            //�����ʼλ��������������
extern u32 leg_post_left_lim;              //��������    
extern u32 leg_post_left_runed;            //�����ʼλ��������������

//��С��/С��
extern u32 arm_fore_right_lim;             //��������     
extern u32 arm_fore_right_runed;           //�����ʼλ��������������
extern u32 leg_fore_right_lim;             //��������     
extern u32 leg_fore_right_runed;           //�����ʼλ��������������

//�Ҵ��/����
extern u32 arm_post_right_lim;             //��������   
extern u32 arm_post_right_runed;           //�����ʼλ��������������
extern u32 leg_post_right_lim;             //��������   
extern u32 leg_post_right_runed;           //�����ʼλ��������������

//����С��/С��
extern u32 arm_fore_left_right_lim;        //��������     
extern u32 arm_fore_left_right_runed;      //�����ʼλ��������������
extern u32 leg_fore_left_right_lim;        //��������     
extern u32 leg_fore_left_right_runed;      //�����ʼλ��������������

//���Ҵ��/����
extern u32 arm_post_left_right_lim;        //��������     
extern u32 arm_post_left_right_runed;      //�����ʼλ��������������
extern u32 leg_post_left_right_lim;        //��������     
extern u32 leg_post_left_right_runed;      //�����ʼλ��������������

/**************************��ϴ����Ƹ�******************************/

extern u32 push_rod_runed_pulse;           //�����ʼλ��������������
extern u32 push_rod_pulse_lim;             //��������

extern u32 swash_dry_runed_pulse;          //�����ʼλ��������������  
extern u32 swash_dry_pulse_lim;            //��������

/********************������ս�ǰ�ƶ����Ƹ�***************************/

extern u32 push_rod_tig_runed_pulse;       //�����ʼλ��������������  
extern u32 push_rod_tig_pulse_now;         //��ǰһ������������
extern u32 push_rod_tig_pulse_lim;         //��������

/*********��������Զ���װ��ֵ�����Ƶ�������ٶ�******************/

extern unsigned int motor_back_freq;        //֧��
extern unsigned int motor_body_freq;        //����
extern unsigned int motor_washlet_freq;     //���� 
extern unsigned int motor_desk_freq;        //С���� 
extern unsigned int motor_hang_freq;        //���� 

/**********��ʱ����Ƶֵpsc�����Ƶ�������ٶ�**********************/

extern unsigned int motor_timer_freq;      //��ʱ����Ƶֵpsc   ��ʱ��Ƶ��=90M/50
extern unsigned int motor_timer_freq_1; 

/**********��ʱ����Ƶֵpsc�����Ƶ��������ʱ��*******************/

extern u16 timer10_freq;
extern unsigned int timer10_freq_1;       //��ʱ����Ƶֵ,
extern unsigned int timer10_arr_1s;       //��ʱ1����


extern u16 bodyleft_compleate;
extern u16 bodyright_compleate;
extern u8 body_left_dir_flag;
extern u8 body_right_dir_flag;
extern u8 muscle_massager_flag;

extern unsigned int motor_timer_freq;


/*********���ܺ���(�ֻ�APP��ң����������)-˫��******************************/

void Fun_Back(void);                     //֧��
void Fun_Leg_Up(void);                   //������
void Fun_Leg_Down(void);                 //������
void Fun_Body_Left(void);                //��
void Fun_Body_Right(void);               //�ҷ�
void Fun_Desk(void);                     //�칫����һ����
void Fun_Back_Nursing_Left(void);        //�󱳲�����
void Fun_Back_Nursing_Right(void);       //�ұ������� 
void Fun_Waist_Nursing_Left(void);       //����������
void Fun_Waist_Nursing_Right(void);      //����������

//��������
void LDUART4(void);                      //��������
void WriteInUART4(char *p);              //���ַ���д������4��

/**************�������ܺ���-����*********************/

void GB_Back(void);                     //֧��
void GB_Body_Left(void);                //����
void GB_Body_Right(void);               //�ҷ���
void GB_Back_Nursing(void);             //��������
void GB_Waist_Nursing(void);            //��������
u8   GB_Lock(void);                     //һ������

/**************�Զ�����******************************/

void Washlet_Auto(void);                //�Զ�����
void Washlet(u8 dir);                   //��������/�ر�
void Back_Leg(void);                    //֧�������������к���
u8   Weight(void);                      //����
u8   Washlet_Weight(void); 
void Washlet_Tig(u8 dir);               //������ս�


void Swash_Dry(void);                   //��ϴ���
void Swash_Hand(void);                  //�ֶ���ϴ
void Dry_Hand(void);                    //�ֶ����
void Swash_Auto(void);                  //�Զ���ϴ
void Dry_Auto(void);                    //�Զ����

void Muscle_Massager(void);             //���ⰴĦ
void Heat(void);                        //����

void IO_TEST(void);                     //IO�ڲ��Ժ��� 

/**********************ר��ϵͳ******************************/

void Exp_Back(void);                    //ר��ϵͳ֧��
void Exp_Body(void);                    //ר��ϵͳ����
void Exp_Leg(void);                     //ר��ϵͳ����
void Exp_Washlet_Auto(void);            //ר��ϵͳ�Զ�����

/****************���ҿ���ѵ��******************************/

//�Զ�
void Auto_Arm_Leg_Left(int t);             //��֫����ѵ��
void Auto_Arm_Leg_Right(int t);            //��֫����ѵ��
void Auto_Arm_Leg_Left_Right(int t);       //����֫����ѵ�� 

//�ֶ�
void Hand_Arm_Fore_Left(void);             //��С��
void Hand_Arm_Post_Left(void);             //����
void Hand_Arm_Fore_Post_Left(void);        //���С��

void Hand_Leg_Fore_Left(void);             //��С��
void Hand_Leg_Post_Left(void);             //�����
void Hand_Leg_Fore_Post_Left(void);        //���С��

void Hand_Arm_Fore_Right(void);            //��С��
void Hand_Arm_Post_Right(void);            //�Ҵ��
void Hand_Arm_Fore_Post_Right(void);       //�Ҵ�С��

void Hand_Leg_Fore_Right(void);            //��С��
void Hand_Leg_Post_Right(void);            //�Ҵ���
void Hand_Leg_Fore_Post_Right(void);       //�Ҵ�С��

void Hand_Arm_Fore_Left_Right(void);       //����С��
void Hand_Arm_Post_Left_Right(void);       //���Ҵ��
void Hand_Arm_Fore_Post_Left_Right(void);  //���Ҵ�С��

void Hand_Leg_Fore_Left_Right(void);       //����С��
void Hand_Leg_Post_Left_Right(void);       //���Ҵ���
void Hand_Leg_Fore_Post_Left_Right(void);  //���Ҵ�С��

/****************���ڵ��縴λ����********************************/

void Res_Power_Down(void);                 //���縴λ����
void Res_Back(void);                       //֧����λ 
void Res_Leg(void);                        //���ȸ�λ
void Res_Desk(void);                       //�Ͳ�����һ������λ
void Res_Motor5(u8 dir);                   //5�ŵ������λ0;����λ1
void Res_Body_Left(void);                  //����λ
void Res_Body_Right(void);                 //�ҷ���λ

/*********��ȡ��λ���µ�����ֵ********************************************/

void get_newangle_usart2(void);            //ͨ������2��ȡ��λ���µ�����ֵ
void get_newangle_wifi(void);              //ͨ��WiFi��ȡ��λ���µ�����ֵ

/*********�����нǶ�ת��Ϊ������*****************************************/

u16 back_angle_to_arr(u8 angle);           //��֧�����нǶ�ת��Ϊ������
u16 leg_angle_to_arr(u8 angle);            //���������нǶ�ת��Ϊ������
u16 body_angle_to_arr(u8 angle);           //���������нǶ�ת��Ϊ������
u16 desk_distance_to_arr(u8 distance);     //���������нǶ�ת��Ϊ������

void Read_Angle(void);                     //��ȡ��λ�����õĽǶ�ֵ�;���ֵ

void Wifi_Send(u8 *data);                  //WiFi���ͺ���
void Wifi_ToPC(u8 *data);                  //WiFi���͸�PC����
void Wifi_ToPhone(u8 *data);               //WiFi���͸��ֻ�APP����
void Wifi_ToRemote(u8 *data);              //WiFi���͸�ң��������
void Wifi_ToGuard(u8 *data);               //WiFi���͸���������
void Uart_ToStick(u8 *data);               //WiFi���͸�PC������

/***********************************************************************
                    
					     ����2��������������
					
************************************************************************/

/*********���ܺ���(�ƶ������ն�)-˫��******************************/

void Uart_Back(void);                  //֧��
void Uart_Leg_Up(void);                //������
void Uart_Leg_Down(void);              //������
void Uart_Body_Left(void);             //��
void Uart_Body_Right(void);            //�ҷ�
void Uart_Desk(void);                  //�칫����һ����
void Uart_Desk1(void);
void Uart_Back_Nursing_Left(void);     //�󱳲�����
void Uart_Back_Nursing_Right(void);    //�ұ������� 
void Uart_Waist_Nursing_Left(void);    //����������
void Uart_Waist_Nursing_Right(void);   //����������

//�������Ժ���
void LDUART2(void);                    //���ܺ�������
void WriteInUART2(char *p);            //���ַ���д�뵽����2��USART2_RX_BUF��
void LDUART2V2(void);
//���Ե������
void MOTOR111(u8 dir);
void MOTOR222(u8 dir);
void MOTOR333(u8 dir);
void MOTOR444(u8 dir);
void MOTOR555(u8 dir);
void MOTOR666(u8 dir);
void MOTOR777(u8 dir);

void  TestAll(u8 dir);
void Hang1Test(u8 dir);
void Hang2Test(u8 dir);
void Hang3Test(u8 dir);
void Hang4Test(u8 dir);

/*********�������ܺ���-����************************************************/

void Uart_GB_Back(void);                        //֧��
void Uart_GB_Body_Left(void);                   //����
void Uart_GB_Body_Right(void);                  //�ҷ���
void Uart_GB_Back_Nursing(void);                //��������
void Uart_GB_Waist_Nursing(void);               //��������
u8   Uart_GB_Lock(void);                        //һ������

/**************�Զ�����******************************/

void Uart_Washlet_Auto(void);                   //�Զ�����
void Uart_Washlet(u8 dir);                      //��������/�ر�
void Uart_Swash_Dry(void);                      //��ϴ���
void Uart_Swash_Hand(void);                     //�ֶ���ϴ
void Uart_Dry_Hand(void);                       //�ֶ����
void Uart_Swash_Auto(void);                     //�Զ���ϴ
void Uart_Dry_Auto(void);                       //�Զ����
u8   Uart_Weight(void);                         //����
void Uart_Washlet_Tig(u8 dir);                  //������ս�
void UartWashletTig(u8 dir,u32 TGArr,u32 SXArr);
void Uart_WashletTigOnly(u8 dir);
u8   Uart_Washlet_Weight(void);                 //����
void Uart_Back_Leg(void);                       //֧��������ͬʱ����

/**********************ר��ϵͳ******************************/

void Uart_Exp_Back(void);                       //ר��ϵͳ֧��
void Uart_Exp_Body(void);                       //ר��ϵͳ����
void Uart_Exp_Leg(void);                        //ר��ϵͳ����
void Uart_Exp_Washlet_Auto(void);               //ר��ϵͳ�Զ�����

/****************���ҿ���ѵ��******************************/

//�Զ�
void Uart_Auto_Arm_Leg_Left(int t);             //��֫����ѵ��
void Uart_Auto_Arm_Leg_Right(int t);            //��֫����ѵ��
void Uart_Auto_Arm_Leg_Left_Right(int t);       //����֫����ѵ�� 

//�ֶ�
void Uart_Hand_Arm_Fore_Left(void);             //��С��
void Uart_Hand_Arm_Post_Left(void);             //����
void Uart_Hand_Arm_Fore_Post_Left(void);        //���С��

void Uart_Hand_Leg_Fore_Left(void);             //��С��
void Uart_Hand_Leg_Post_Left(void);             //�����
void Uart_Hand_Leg_Fore_Post_Left(void);        //���С��

void Uart_Hand_Arm_Fore_Right(void);            //��С��
void Uart_Hand_Arm_Post_Right(void);            //�Ҵ��
void Uart_Hand_Arm_Fore_Post_Right(void);       //�Ҵ�С��

void Uart_Hand_Leg_Fore_Right(void);            //��С��
void Uart_Hand_Leg_Post_Right(void);            //�Ҵ���
void Uart_Hand_Leg_Fore_Post_Right(void);       //�Ҵ�С��

void Uart_Hand_Arm_Fore_Left_Right(void);       //����С��
void Uart_Hand_Arm_Post_Left_Right(void);       //���Ҵ��
void Uart_Hand_Arm_Fore_Post_Left_Right(void);  //���Ҵ�С��

void Uart_Hand_Leg_Fore_Left_Right(void);       //����С��
void Uart_Hand_Leg_Post_Left_Right(void);       //���Ҵ���
void Uart_Hand_Leg_Fore_Post_Left_Right(void);  //���Ҵ�С��

/****************���ڵ��縴λ����********************************/

void Uart_Res_Power_Down(void);                 //���縴λ����
void Uart_Res_Power_Down1(void);
void Uart_Res_Power_Down2(void);

void Uart_Res_Back(void);                       //֧����λ
void Uart_Res_Leg(void);                        //���ȸ�λ
void Uart_Res_Desk(void);                       //�Ͳ�����һ������λ
void Uart_Res_Motor5(u8 dir);                   //5�ŵ������λ0;����λ1
void Uart_Res_Body_Left(void);                  //����λ
void Uart_Res_Body_Right(void);                 //�ҷ���λ

void FlagClear(void);

#endif

