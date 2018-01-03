#ifndef _MODBUS_MASTER_H
#define _MODBUS_MASTER_H
#include "sys.h"
#include "usart.h"

#define SlaveAddr 0x01      //����ӻ���ַ
#define HBAUD485		UART3_BAUD

//01H ��ǿ�Ƶ���Ȧ
//05H дǿ�Ƶ���Ȧ
#define REG_D01 0x0101
#define REG_D02 0x0102
#define REG_D03 0x0103
#define REG_D04 0x0104
#define REG_DXX REG_D04

// 02H ��ȡ����״̬
#define REG_T01 0x0201
#define REG_T02 0x0202
#define REG_T03 0x0203
#define REG_TXX REG_T03

// 03H �����ּĴ���
// 06H д���ּĴ���
// 10H д�������Ĵ���
#define REG_P01		0x0022		
#define REG_P02		0x0302	
#define REG_P03   0x0020
#define REG_P04   0x0095
#define REG_P05   0x0200

// 04H ��ȡ����Ĵ���(ģ���ź�)
#define REG_A01		0x0401
#define REG_AXX		REG_A01

// RTU Ӧ����� 
#define RSP_OK				0		// �ɹ�
#define RSP_ERR_CMD			0x01	//��֧�ֵĹ�����
#define RSP_ERR_REG_ADDR	0x02	// �Ĵ�����ַ���� 
#define RSP_ERR_VALUE		0x03	// ����ֵ����� 
#define RSP_ERR_WRITE		0x04	// д��ʧ�� 

#define H_RX_BUF_SIZE		64
#define H_TX_BUF_SIZE      	128

#define MODH_RECEIVE_LEN 		15

extern u16 MODH_RECEIVE;
extern u8 MODH_RECEIVE_BUFFER[MODH_RECEIVE_LEN];

typedef struct
{
	u8 RxBuf[H_RX_BUF_SIZE];
	u8 RxCount;
	u8 RxStatus;
	u8 RxNewFlag;
	u8 RspCode;
	u8 TxBuf[H_TX_BUF_SIZE];
	u8	TxCount;
	
	u16 Reg01H;         //�����������͵ļĴ����׵�ַ
	u16 Reg02H;
	u16 Reg03H;
	u16 Reg04H;	

	u8 RegNum;		//�Ĵ�������
	u8	fAck01H;		// Ӧ�������־ 0 ��ʾִ��ʧ�� 1��ʾִ�гɹ�
	u8	fAck02H;
	u8 fAck03H;						
	u8	fAck04H;
	u8	fAck05H;
	u8	fAck06H;
	u8	fAck10H;
}MODH_T;


typedef struct
{
	// 03H 06H ��д���ּĴ���
	u16 P01;
	u16 P02;
	
	/* 02H ��д��ɢ����Ĵ��� */
	u16 T01;
	u16 T02;
	u16 T03;
	
	/* 04H ��ȡģ�����Ĵ��� */
	u16 A01;
	
	/* 01H 05H ��д����ǿ����Ȧ */
	u16 D01;
	u16 D02;
	u16 D03;
	u16 D04;
	
}VAR_T;

void MODH_Poll(void);
u8 MODH_ReadParam_01H(uint16_t _reg, uint16_t _num);
u8 MODH_ReadParam_02H(uint16_t _reg, uint16_t _num);
u8 MODH_ReadParam_03H(uint16_t _reg, uint16_t _num);
u8 MODH_ReadParam_04H(uint16_t _reg, uint16_t _num);
u8 MODH_WriteParam_05H(uint16_t _reg, uint16_t _value);
u8 MODH_WriteParam_06H(uint16_t _reg, uint16_t _value);
u8 MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf);
void MODH_ReciveNew(uint8_t _data);
void MODH_Save_Data(uint8_t _data);
void test_modbus(u8 num);

u8 get_electric_current_feedback(u8 num);
u8 get_position_error(u8 num);
u8 get_speed_feedback(u8 num);
u8 get_position_feedback(u8 num);
u8 get_speed_set(u8 num);
u8 get_position_set(u8 num);

extern MODH_T g_tModH;

#endif



