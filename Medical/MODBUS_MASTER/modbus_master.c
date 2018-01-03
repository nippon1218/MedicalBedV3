/*
*********************************************************************************************************
*��: modbus_master.c
*	����˵��: modbus��վ����������������ж�
*	�����㣬2017��5��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#include "modbus_master.h"
#include "sys.h"
#include "bsp_user_lib.h"
#include "main.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pcf8574.h"

#define TIMEOUT		8		/* �������ʱʱ��, ��λms */
#define NUM			1		/* ѭ�����ʹ��� */

u16 MODH_RECEIVE=0;
u8 MODH_RECEIVE_BUFFER[MODH_RECEIVE_LEN];
extern u8 modbus_ready;


/* ����ÿ���ӻ��ļ�����ֵ */

MODH_T g_tModH;   /*** �ض���ṹ��MODH_T,�ýṹ�屣���˷��ͽ������ݻ���
                  ��Ϣ�����飬���ȣ����Ĵ�����ַ���Ĵ�����������Ӧ���־***/

u8 g_modh_timeout = 0;

static void MODH_RxTimeOut(void);
static void MODH_AnalyzeApp(void);
static void MODH_Read_01H(void);
static void MODH_Read_02H(void);
static void MODH_Read_03H(void);
static void MODH_Read_04H(void);
static void MODH_Read_05H(void);
static void MODH_Read_06H(void);
static void MODH_Read_10H(void);


VAR_T g_tVar;   /***��д�Ĵ�������ȡ�Ĵ�������дǿ����Ȧ��***/
                    

/*
*********************************************************************************************************
*	�� �� ��: MODH_SendPacket
*	����˵��: �������ݰ� COM1��
*	��    ��: _buf : ���ݻ�����
*			  _len : ���ݳ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_SendPacket(uint8_t *_buf, uint16_t _len)
{
	u16 i;
	for(i=0;i<_len;i++)
	{
	  while((USART1->SR&0X40)==0);//ѭ������,ֱ���������  
		USART1->DR =_buf[i];
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_SendAckWithCRC
*	����˵��: ����Ӧ��,�Զ���CRC.  
*	��    ��: �ޡ����������� g_tModH.TxBuf[], [g_tModH.TxCount
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_SendAckWithCRC(void)
{
	u16 crc;
	crc=CRC16_Modbus(g_tModH.TxBuf,g_tModH.TxCount);
	g_tModH.TxBuf[g_tModH.TxCount++]=crc>>8;
	g_tModH.TxBuf[g_tModH.TxCount++]=crc;
	MODH_SendPacket(g_tModH.TxBuf,g_tModH.TxCount);
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_AnalyzeApp
*	����˵��: ����Ӧ�ò�Э�顣����Ӧ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_AnalyzeApp(void)
{	
  switch (g_tModH.RxBuf[1])    //�ڶ����ֽڣ�������
	{
	  case 0x01:          //��ȡ��Ȧ״̬
			MODH_Read_01H();
      break;
	  
		case 0x02:      //��ȡ����״̬
			MODH_Read_02H();
		break;
	
		case 0x03:   //��ȡ����Ĵ�������һ����������Ĵ�����ȡ�õ�ǰ�Ķ�����ֵ
		  MODH_Read_03H();
	  break;
		
		case 0x04:      //��ȡ����Ĵ���
			MODH_Read_04H();
		break;		
		
		case 0x05:      //ǿ�Ƶ���Ȧ
			MODH_Read_05H();
		break;
		
		case 0x06:	  //д�����Ĵ���
		  MODH_Read_06H();
		break;		
	
		case 0x10:	// д����Ĵ���
			MODH_Read_10H();
		break;
		
		default:
		break;	
		
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send01H
*	����˵��: ����01Hָ���ѯ1���������ּĴ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;	
	g_tModH.TxBuf[g_tModH.TxCount++]=_addr;      //��վ��ַ
	g_tModH.TxBuf[g_tModH.TxCount++]=0x01;       //������
	g_tModH.TxBuf[g_tModH.TxCount++]=_reg>>8;    //�Ĵ�����ţ����ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++]=_reg;       //�Ĵ�����ţ����ֽ�
    g_tModH.TxBuf[g_tModH.TxCount++]=_num>>8;    //�Ĵ������������ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++]=_num;       //�Ĵ������������ֽ�
	
	MODH_SendAckWithCRC();         //�������ݣ��Զ���CRC;
	g_tModH.fAck01H=0;            //����ձ�־
	g_tModH.RegNum=_num;            //�Ĵ�������
	g_tModH.Reg01H=_reg;           //����03Hָ���еļĴ�����ַ������Ӧ�����ݽ��з���
	
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send02H
*	����˵��: ����02Hָ�����ɢ����Ĵ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		// ��վ��ַ
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x02;		// ������	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	// �Ĵ������ ���ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		// �Ĵ������ ���ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	// �Ĵ������� ���ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		// �Ĵ������� ���ֽ� 
	
	MODH_SendAckWithCRC();		// �������ݣ��Զ���CRC
	g_tModH.fAck02H = 0;		// ����ձ�־
	g_tModH.RegNum = _num;		// �Ĵ�������
	g_tModH.Reg02H = _reg;		// ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з���
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send03H
*	����˵��: ����03Hָ���ѯ1���������ּĴ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		// ��վ��ַ
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		// ������ 	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	// �Ĵ������ ���ֽ� 
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		// �Ĵ������ ���ֽ� 
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	// �Ĵ������� ���ֽ� 
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		// �Ĵ������� ���ֽ� 
	
	MODH_SendAckWithCRC();		// �������ݣ��Զ���CRC
	g_tModH.fAck03H = 0;		// ����ձ�־
	g_tModH.RegNum = _num;		// �Ĵ�������
	g_tModH.Reg03H = _reg;		// ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� 
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send04H
*	����˵��: ����04Hָ�������Ĵ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		// ��վ��ַ
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x04;		// ������ 
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	// �Ĵ������ ���ֽ� 
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		// �Ĵ������ ���ֽ� 
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	// �Ĵ������� ���ֽ� 
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		// �Ĵ������� ���ֽ� 
	
	MODH_SendAckWithCRC();		// �������ݣ��Զ���CRC
	g_tModH.fAck04H = 0;		// ����ձ�־
	g_tModH.RegNum = _num;		// �Ĵ�������
	g_tModH.Reg04H = _reg;		// ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з���
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send05H
*	����˵��: ����05Hָ�дǿ�õ���Ȧ
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _value : �Ĵ���ֵ,2�ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send05H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			// ��վ��ַ
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x05;			// ������	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		// �Ĵ������ ���ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			// �Ĵ������ ���ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		// �Ĵ���ֵ ���ֽ�
	g_tModH.TxBuf[g_tModH.TxCount++] = _value;			// �Ĵ���ֵ ���ֽ�
	
	MODH_SendAckWithCRC();		// �������ݣ��Զ���CRC

	g_tModH.fAck05H = 0;		// ����յ��ӻ���Ӧ���������־����Ϊ1,����ձ�־
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send06H
*	����˵��: ����06Hָ�д1�����ּĴ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _value : �Ĵ���ֵ,2�ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* ��վ��ַ */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x06;			/* ������ */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* �Ĵ������ ���ֽ� */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* �Ĵ������ ���ֽ� */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* �Ĵ���ֵ ���ֽ� */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* �Ĵ���ֵ ���ֽ� */
	
	MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */
	
	g_tModH.fAck06H = 0;		/* ����յ��ӻ���Ӧ���������־����Ϊ1 */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send10H
*	����˵��: ����10Hָ�����д������ּĴ���. ���һ��֧��23���Ĵ�����
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������n (ÿ���Ĵ���2���ֽ�) ֵ��
*			  _buf : n���Ĵ��������ݡ����� = 2 * n
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
	uint16_t i;
	
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* ��վ��ַ */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;		/* ��վ��ַ */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* �Ĵ������ ���ֽ� */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* �Ĵ������� ���ֽ� */
	g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* �����ֽ��� */	
	
	for (i = 0; i < 2 * _num; i++)
	{
		if (g_tModH.TxCount > H_RX_BUF_SIZE - 3)
		{
			return;		/* ���ݳ������������ȣ�ֱ�Ӷ��������� */
		}
		g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i];		/* ��������ݳ��� */
	}	
	MODH_SendAckWithCRC();	/* �������ݣ��Զ���CRC */
}


/*
*********************************************************************************************************
*	�� �� ��: MODH_ReciveNew
*	����˵��: ���ڽ����жϷ���������ñ����������յ�һ���ֽ�ʱ��ִ��һ�α�������
*	��    ��: 
*	�� �� ֵ: 1 ��ʾ������
*********************************************************************************************************
*/
void MODH_ReciveNew(uint8_t _data)
{
	/*
		3.5���ַ���ʱ������ֻ������RTUģʽ���棬��ΪRTUģʽû�п�ʼ���ͽ�������
		�������ݰ�֮��ֻ�ܿ�ʱ���������֣�Modbus�����ڲ�ͬ�Ĳ������£����ʱ���ǲ�һ���ģ�
		���Ծ���3.5���ַ���ʱ�䣬�����ʸߣ����ʱ������С�������ʵͣ����ʱ������Ӧ�ʹ�
		4800  = 7.297ms
		9600  = 3.646ms
		19200  = 1.771ms
		38400  = 0.885ms
	*/	
	uint32_t timeout;	
	g_modh_timeout = 0;
//	timeout = 35000 / HBAUD485;   //���㳬ʱʱ�䣬��λ��us:350000000;

  if(g_tModH.RxCount<H_RX_BUF_SIZE)
	{
	  g_tModH.RxBuf[g_tModH.RxCount++]=_data;
	}
  
	// Ӳ����ʱ�жϣ���ʱ����us Ӳ����ʱ��2����MODBUS�ӻ�, ��ʱ��3����MODBUS�ӻ�����*/
//	delay_us(5);
	LED0=!LED0;

	
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_RxTimeOut
*	����˵��: ����3.5���ַ�ʱ���ִ�б������� ����ȫ�ֱ��� g_rtu_timeout = 1; ֪ͨ������ʼ���롣
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

static void MODH_RxTimeOut(void)
{
	g_modh_timeout = 1;
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Poll
*	����˵��: ���տ�����ָ��. 1ms ��Ӧʱ�䡣
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ������ 1��ʾ�յ���ȷ����
*********************************************************************************************************
*/
 void MODH_Poll(void)
{	 
	u16 crc1;
	if(g_modh_timeout ==0)
	{
	  // û�г�ʱ���������ա���Ҫ���� g_tModH.RxCount
				return ;	
	}	
	/* �յ�����
		05 06 00 88 04 57 3B70 (8 �ֽ�)
			05    :  ��������ĺ�վ��
			06    :  ָ��
			00 88 :  �����������ʾ�Ĵ���
			04 57 :  ����,,,ת���� 10 ������ 1111.��λ��ǰ,
			3B70  :  �����ֽ� CRC ��	��05�� 57��У��
	*/
		g_modh_timeout = 0;
	
		if (g_tModH.RxCount < 4)
	{
		goto err_ret;
	}
	
	/* ����CRCУ��� */
	crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
	if (crc1 != 0)
	{
		goto err_ret;
	}	

	/* ����Ӧ�ò�Э�� */
	MODH_AnalyzeApp();

err_ret:
#if 0	/* �˲���Ϊ�˴��ڴ�ӡ���,ʵ�������пɲ�Ҫ */
	g_tPrint.Rxlen = g_tModH.RxCount;
	memcpy(g_tPrint.RxBuf, g_tModH.RxBuf, g_tModH.RxCount);
#endif
	
	g_tModH.RxCount = 0;	/* ��������������������´�֡ͬ�� */
	
}


/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_01H
*	����˵��: ����01Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_Read_01H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* ���ݳ��� �ֽ��� */				
		switch (g_tModH.Reg01H)   //�����������͵ļĴ����׵�ַ
		{
			case REG_D01:
				if (bytes == 8)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.D01 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					g_tVar.D02 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					g_tVar.D03 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					g_tVar.D04 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */
					
					g_tModH.fAck01H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_02H
*	����˵��: ����02Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_Read_02H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* ���ݳ��� �ֽ��� */				
		switch (g_tModH.Reg02H)
		{
			case REG_T01:
				if (bytes == 6)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.T01 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					g_tVar.T02 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					g_tVar.T03 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					
					g_tModH.fAck02H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_04H
*	����˵��: ����04Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_Read_04H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* ���ݳ��� �ֽ��� */				
		switch (g_tModH.Reg04H)
		{
			case REG_T01:
				if (bytes == 2)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.A01 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					
					g_tModH.fAck04H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_05H
*	����˵��: ����05Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_Read_05H(void)
{
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck05H = 1;		/* ���յ�Ӧ�� */
		}
	};
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_06H
*	����˵��: ����06Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_Read_06H(void)
{
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck06H = 1;		/* ���յ�Ӧ�� */
		}
	}
}


/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_03H
*	����˵��: ����03Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Read_03H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* ���ݳ��� �ֽ��� */				
		switch (g_tModH.Reg03H)
		{
			case REG_P01:
				if (bytes == 32)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.P01 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
					g_tVar.P02 = BEBufToUint16(p); p += 2;	/* �Ĵ��� */	
		
					g_tModH.fAck03H = 1;
				}
				break;
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_10H
*	����˵��: ����10Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Read_10H(void)
{
	/*
		10Hָ���Ӧ��:
			�ӻ���ַ                11
			������                  10
			�Ĵ�����ʼ��ַ���ֽ�	00
			�Ĵ�����ʼ��ַ���ֽ�    01
			�Ĵ����������ֽ�        00
			�Ĵ����������ֽ�        02
			CRCУ����ֽ�           12
			CRCУ����ֽ�           98
	*/
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck10H = 1;		// ���յ�Ӧ��
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_ReadParam_01H
*	����˵��: ��������. ͨ������01Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_01H(uint16_t _reg, uint16_t _num)
{
	u16 time1=0;
	u8 i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send01H (SlaveAddr, _reg, _num);		  // ��������

		while (time1<TIMEOUT)				// �ȴ�Ӧ��,��ʱ����յ�Ӧ����break
		{
			MODH_Poll();		//���տ�����ָ��
			delay_ms(1);
			time1++;
			
			if (g_tModH.fAck01H > 0)    // Ӧ�������־ 0 ��ʾִ��ʧ�� 1��ʾִ�гɹ�
			{
				break;		// ���յ�Ӧ��
			}
		}
		
		if (g_tModH.fAck01H > 0)
		{
			break;			// ѭ��NUM�Σ�������յ�������breakѭ��
		}
	}
	
	if (g_tModH.fAck01H == 0)   // Ӧ�������־ 0 ��ʾִ��ʧ�� 1��ʾִ�гɹ�
	{
		return 0;
	}
	else 
	{
		return 1;	// 01H ���ɹ�
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_ReadParam_02H
*	����˵��: ��������. ͨ������02Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_02H(uint16_t _reg, uint16_t _num)
{
	u16 time1=0;
	u8 i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send02H (SlaveAddr, _reg, _num);
		
		while (time1<TIMEOUT)				// �ȴ�Ӧ��,��ʱ����յ�Ӧ����break
		{
			MODH_Poll();		
			delay_ms(1);
			time1++;
			
			if (g_tModH.fAck02H > 0)
			{
				break;		// ���յ�Ӧ��
			}
		}
		
		if (g_tModH.fAck02H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck02H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 02H ���ɹ� */
	}
}
/*
*********************************************************************************************************
*	�� �� ��: MODH_ReadParam_03H
*	����˵��: ��������. ͨ������03Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_03H(uint16_t _reg, uint16_t _num)
{
	u16 time1=0;
	u8 i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send03H (SlaveAddr, _reg, _num);
		
		while (time1<TIMEOUT)				// �ȴ�Ӧ��,��ʱ����յ�Ӧ����break
		{
			MODH_Poll();		
			delay_ms(1);
			time1++;
			
			if (g_tModH.fAck03H > 0)
			{
				break;		// ���յ�Ӧ��
			}
		}
		
		if (g_tModH.fAck03H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck03H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 03H ���ɹ� */
	}

}


/*
*********************************************************************************************************
*	�� �� ��: MODH_ReadParam_04H
*	����˵��: ��������. ͨ������04Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_04H(uint16_t _reg, uint16_t _num)
{
	u16 time1=0;
	u8 i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send04H (SlaveAddr, _reg, _num);
		
		while (time1<TIMEOUT)				// �ȴ�Ӧ��,��ʱ����յ�Ӧ����break
		{
			MODH_Poll();		
			delay_ms(1);
			time1++;
			
			if (g_tModH.fAck04H > 0)
			{
				break;		// ���յ�Ӧ��
			}
		}
		
		if (g_tModH.fAck04H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck04H == 0)
	{
		return 0;
	}
	
	else 
	{
		return 1;	/* 04H ���ɹ� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_WriteParam_05H
*	����˵��: ��������. ͨ������05Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_05H(uint16_t _reg, uint16_t _value)
{
	u16 time1=0;
	u8 i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send05H (SlaveAddr, _reg, _value);
		
		while (time1<TIMEOUT)				// �ȴ�Ӧ��,��ʱ����յ�Ӧ����break
		{
			MODH_Poll();		
			delay_ms(1);
			time1++;
			
			if (g_tModH.fAck05H > 0)
			{
				break;		// ���յ�Ӧ��
			}
		}
		
		if (g_tModH.fAck05H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck05H == 0)
	{
		return 0;
	}
	
	else 
	{
		return 1;	/* 05H ���ɹ� */
	}	
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_WriteParam_06H
*	����˵��: ��������. ͨ������06Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��ѭ��NUM��д����
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_06H(uint16_t _reg, uint16_t _value)
{
	u16 time1=0;
	u8 i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send06H (SlaveAddr, _reg, _value);
		
		while (time1<TIMEOUT)				// �ȴ�Ӧ��,��ʱ����յ�Ӧ����break
		{
			MODH_Poll();		
			delay_ms(1);
			time1++;
			
			if (g_tModH.fAck06H > 0)
			{
				break;		// ���յ�Ӧ��
			}
		}
		
		if (g_tModH.fAck06H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck06H == 0)
	{
		return 0;
	}
	
	else 
	{
		return 1;	/* 06H ���ɹ� */
	}		
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_WriteParam_10H
*	����˵��: ��������. ͨ������10Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��ѭ��NUM��д����
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
	u16 time1=0;
	u8 i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send10H(SlaveAddr, _reg, _num, _buf);
		
		while (time1<TIMEOUT)				// �ȴ�Ӧ��,��ʱ����յ�Ӧ����break
		{
			MODH_Poll();		
			delay_ms(1);
			time1++;
			
			if (g_tModH.fAck10H > 0)
			{
				break;		// ���յ�Ӧ��
			}
		}
		
		if (g_tModH.fAck10H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck10H == 0)
	{
		return 0;
	}
	
	else 
	{
		return 1;	// 10H ���ɹ�
	}		
}


/*
*********************************************************************************************************
*	�� �� ��: MODH_Save_Data
*	����˵��: ��modbus���ص�ֵ��������MODH_RECEIVE_BUFFER
*	��    ��: 
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Save_Data(uint8_t _data)
{
  MODH_RECEIVE_BUFFER[MODH_RECEIVE++]=_data;	
}



/*
*********************************************************************************************************
*	�� �� ��: test_modbus
*	����˵��: modbus���Ժ���
*	��    ��: 
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void test_modbus(u8 num)
{
  u8 i;
  for(i=0;i<num;i++)
	{
			if (MODH_WriteParam_06H(REG_P01, 0x0080) == 1)
				 {LED1=!LED1;}
			if (MODH_WriteParam_06H(REG_P03, 0x0001) == 1)					      
				 {LED1=!LED1;}					
			if (MODH_ReadParam_03H(REG_P04, 0x0001) == 1)
				 {LED1=!LED1 ;	}
			modbus_ready=1;									 
			if (MODH_ReadParam_03H(REG_P05, 0x0005) == 1)
				 {LED1=!LED1;}
			modbus_ready=0; 
			MODH_RECEIVE=0;
	  
	    delay_ms(100);
			if(caluate_value_10(MODH_RECEIVE_BUFFER)>500)
      {
  			LED1=!LED1;
				PCF8574_WriteBit(BEEP_IO,0);	//���Ʒ�����
				delay_ms(100);
				PCF8574_WriteBit(BEEP_IO,1);	//���Ʒ�����
			}				
			delay_ms(10);
			MODH_RECEIVE=0;
      memset(MODH_RECEIVE_BUFFER,0,15);
      memset(aRxBuffer,0,RXBUFFERSIZE);				
	}		
}

u8 get_electric_current_feedback(u8 num)
{
  u8 i;
	u8 res=0;
	u2_printf("\r\n��������\r\n");
  for(i=0;i<num;i++)
	{ 
    MODH_WriteParam_06H(REG_P01, 0x0080);
		MODH_WriteParam_06H(REG_P03, 0x0001);			      			
		MODH_ReadParam_03H(REG_P04, 0x0001);
		modbus_ready=1;									 
		MODH_ReadParam_03H(REG_P05, 0x0005);
		modbus_ready=0; 			
		MODH_RECEIVE=0;	  
			if(caluate_value_10(MODH_RECEIVE_BUFFER)>500)
      {
  			LED1=!LED1;
        u2_printf("\r\nƽ��ֵ����500\r\n");
				res=1;
			}				
			MODH_RECEIVE=0;
      memset(MODH_RECEIVE_BUFFER,0,15);
      memset(aRxBuffer,0,RXBUFFERSIZE);				
	}		
		return res;
}



u8 get_position_error(u8 num)
{
  u8 i;
	u8 res=0;
	u2_printf("\r\nλ�����\r\n");
  for(i=0;i<num;i++)
	{
    MODH_WriteParam_06H(REG_P01, 0x0001);
		MODH_WriteParam_06H(REG_P03, 0x0001);			      			
		MODH_ReadParam_03H(REG_P04, 0x0001);
		modbus_ready=1;									 
		MODH_ReadParam_03H(REG_P05, 0x0005);
		modbus_ready=0; 			
		MODH_RECEIVE=0;	  
			if(caluate_value_10(MODH_RECEIVE_BUFFER)>500)
      {
  			LED1=!LED1;
        u2_printf("\r\nƽ��ֵ����500\r\n");
				res=1;
			}				
			MODH_RECEIVE=0;	
      memset(MODH_RECEIVE_BUFFER,0,15);
      memset(aRxBuffer,0,RXBUFFERSIZE);				
	}		
		return res;
}

u8 get_speed_feedback(u8 num)
{
  u8 i;
	u8 res=0;
	u2_printf("\r\n�ٶȷ���\r\n");
  for(i=0;i<num;i++)
	{
    MODH_WriteParam_06H(REG_P01, 0x0002);
		MODH_WriteParam_06H(REG_P03, 0x0001);			      			
		MODH_ReadParam_03H(REG_P04, 0x0001);
		modbus_ready=1;									 
		MODH_ReadParam_03H(REG_P05, 0x0005);
		modbus_ready=0; 			
		MODH_RECEIVE=0;	  
			if(caluate_value_10(MODH_RECEIVE_BUFFER)>500)
      {
  			LED1=!LED1;
        u2_printf("\r\nƽ��ֵ����500\r\n");
				res=1;
			}				
			MODH_RECEIVE=0;	
      memset(MODH_RECEIVE_BUFFER,0,15);
      memset(aRxBuffer,0,RXBUFFERSIZE);				
	}		
		return res;
}

u8 get_position_feedback(u8 num)
{
  u8 i;
	u8 res=0;
	u2_printf("\r\nλ�÷���\r\n");
  for(i=0;i<num;i++)
	{
    MODH_WriteParam_06H(REG_P01, 0x0004);
		MODH_WriteParam_06H(REG_P03, 0x0001);			      			
		MODH_ReadParam_03H(REG_P04, 0x0001);
		modbus_ready=1;									 
		MODH_ReadParam_03H(REG_P05, 0x0005);
		modbus_ready=0; 			
		MODH_RECEIVE=0;	  
			if(caluate_value_10(MODH_RECEIVE_BUFFER)>500)
      {
  			LED1=!LED1;
        u2_printf("\r\nƽ��ֵ����500\r\n");
				res=1;
			}				
			MODH_RECEIVE=0;
      memset(MODH_RECEIVE_BUFFER,0,15);
      memset(aRxBuffer,0,RXBUFFERSIZE);					
	}
		return res;
}



u8 get_speed_set(u8 num)
{
  u8 i;
	u8 res=0;
  for(i=0;i<num;i++)
	{
    MODH_WriteParam_06H(REG_P01, 0x0008);
		MODH_WriteParam_06H(REG_P03, 0x0001);			      			
		MODH_ReadParam_03H(REG_P04, 0x0001);
		modbus_ready=1;									 
		MODH_ReadParam_03H(REG_P05, 0x0005);
		modbus_ready=0; 			
		MODH_RECEIVE=0;	  
			if(caluate_value_10(MODH_RECEIVE_BUFFER)>500)
      {
  			LED1=!LED1;
        u2_printf("\r\nƽ��ֵ����500\r\n");
				res=1;
			}				
			MODH_RECEIVE=0;
      memset(MODH_RECEIVE_BUFFER,0,15);
      memset(aRxBuffer,0,RXBUFFERSIZE);				
	}
		return res;
}


u8 get_position_set(u8 num)
{
  u8 i;
	u8 res=0;
  for(i=0;i<num;i++)
	{
    MODH_WriteParam_06H(REG_P01, 0x0010);
		MODH_WriteParam_06H(REG_P03, 0x0001);			      			
		MODH_ReadParam_03H(REG_P04, 0x0001);
		modbus_ready=1;									 
		MODH_ReadParam_03H(REG_P05, 0x0005);
		modbus_ready=0; 			
		MODH_RECEIVE=0;	  
			if(caluate_value_10(MODH_RECEIVE_BUFFER)>500)
      {
  			LED1=!LED1;
        u2_printf("\r\nƽ��ֵ����500\r\n");
				res=1;
			}				
			MODH_RECEIVE=0;
      memset(MODH_RECEIVE_BUFFER,0,15);
      memset(aRxBuffer,0,RXBUFFERSIZE);					
	}
		return res;
}
