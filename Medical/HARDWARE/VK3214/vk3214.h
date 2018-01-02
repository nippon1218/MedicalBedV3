#ifndef _VK3224_H
#define _VK3224_H
#include "sys.h"

#define UART_RECEIVE_LEN 	200

/********************ȫ�ּĴ����б�*********************/

#define GCR			((uint8_t)0x01)	//ȫ�ֿ��ƼĴ���
#define GMUCR		((uint8_t)0x02)	//ȫ�������ڿ��ƼĴ���
#define GIR		    ((uint8_t)0x03)	//ȫ���жϼĴ���

/********************�Ӵ��ڼĴ����б�*********************/

#define SCTLR		((uint8_t)0x06)	//�Ӵ��ڿ��ƼĴ��� 
#define SCONR		((uint8_t)0x07)	//�Ӵ������üĴ��� 
#define SFWCR       ((uint8_t)0x08) //�Ӵ����������ƼĴ���
#define SFOCR		((uint8_t)0x09)	//�Ӵ���FIFO���ƼĴ��� 
#define	SIER		((uint8_t)0x0B)	//�Ӵ����ж�ʹ�ܼĴ��� 
#define	SIFR		((uint8_t)0x0C)	//�Ӵ����жϱ�־�Ĵ��� 
#define	SSR			((uint8_t)0x0D)	//�Ӵ���״̬�Ĵ��� 
#define	SFSR		((uint8_t)0x0E)	//�Ӵ���FIFO״̬�Ĵ��� 
#define	SFDR		((uint8_t)0x0F)	//�Ӵ���FIFO���ݼĴ���

extern u16 UART_RECEIVE;
extern u8 UART_SAVE_BUFFER[UART_RECEIVE_LEN];

//�Ӵ�����2��ʾ����������22��ʾ
/********************�Ӵ���UART��������*********************/

typedef struct
{
  uint8_t UART2_BaudRate;                                       
  uint8_t UART2_WordLength;         
  uint8_t UART2_StopBits;	
  uint8_t UART2_Parity;   			 
  uint8_t UART2_Mode;
} UART2_Base_InitTypeDef;   


typedef struct
{
  uint8_t UART2_TFTL;  //����FIFO�������                                              
  uint8_t UART2_RFTL;  //����FIFO�������                                             
  uint8_t UART2_TFEN;  //����FIFOʹ�ܿ���λ                                                
  uint8_t UART2_RFEN;  //����FIFOʹ��			
  uint8_t UART2_TFCL;  //�������FIFO
  uint8_t UART2_RFCL;  // �������FIFO 
} UART2_FIFO_InitTypeDef;

/********************������UART��������*********************/

typedef struct
{
  uint8_t UART22_BaudRate;                                             
  uint8_t UART22_WordLength;          
  uint8_t UART22_StopBits;	
  uint8_t UART22_Parity;   
} UART22_Base_InitTypeDef;   

#define SET 					 ((uint8_t)0x01)
#define RESET 					 ((uint8_t)0x00)
#define IS_FUNCTION_STATE(STATE) (((STATE) == SET) || ((STATE) == RESET))
                                
/********************�����Ӵ���*********************/

#define COM1 								((uint8_t)0x00)
#define COM2								((uint8_t)0x10)
#define COM3 								((uint8_t)0x20)
#define COM4 								((uint8_t)0x30)
#define IS_UART2_ALL_PERIPH(COM)			(((COM) == COM1) || ((COM) == COM2) || ((COM) == COM3) ||((COM) == COM4)) 
 
//�Ӵ��ڿ��ƼĴ���SCTLR                                			 								 			                
/********************�����Ӵ��ڲ�����*********************/

#define UART2_BaudRate_0              		 ((uint8_t)0x00)
#define UART2_BaudRate_1                 	 ((uint8_t)0x10)
#define UART2_BaudRate_2                     ((uint8_t)0x20)
#define UART2_BaudRate_3                  	 ((uint8_t)0x30)
#define UART2_BaudRate_4                	 ((uint8_t)0x40)
#define UART2_BaudRate_5                 	 ((uint8_t)0x50)
#define UART2_BaudRate_6                	 ((uint8_t)0x60)
#define UART2_BaudRate_7                     ((uint8_t)0x70)
#define UART2_BaudRate_8               		 ((uint8_t)0x80)
#define UART2_BaudRate_9                 	 ((uint8_t)0x90)
#define UART2_BaudRate_10                    ((uint8_t)0xa0)
#define UART2_BaudRate_11                	 ((uint8_t)0xb0)
#define UART2_BaudRate_12                	 ((uint8_t)0xc0)
#define UART2_BaudRate_13                	 ((uint8_t)0xd0)
#define UART2_BaudRate_14                	 ((uint8_t)0xe0)
#define UART2_BaudRate_15                	 ((uint8_t)0xf0)
#define IS_UART2_BAUDRATE(BAUDRATE)			 (((BAUDRATE)== UART2_BaudRate_0 ) || \
                                 			 ((BAUDRATE) == UART2_BaudRate_1 ) || \
                                 			 ((BAUDRATE) == UART2_BaudRate_2 ) || \
								 			 ((BAUDRATE) == UART2_BaudRate_3 ) || \
								 			 ((BAUDRATE) == UART2_BaudRate_4 ) || \
											 ((BAUDRATE) == UART2_BaudRate_5 ) || \
											 ((BAUDRATE) == UART2_BaudRate_6 ) || \
											 ((BAUDRATE) == UART2_BaudRate_7 ) || \
											 ((BAUDRATE) == UART2_BaudRate_8 ) || \
											 ((BAUDRATE) == UART2_BaudRate_9 ) || \
											 ((BAUDRATE) == UART2_BaudRate_10) || \
											 ((BAUDRATE) == UART2_BaudRate_11) || \
											 ((BAUDRATE) == UART2_BaudRate_12) || \
											 ((BAUDRATE) == UART2_BaudRate_13) || \
											 ((BAUDRATE) == UART2_BaudRate_14) || \
											 ((BAUDRATE) == UART2_BaudRate_15))

//�Ӵ������üĴ���SCONR
/********************�����Ӵ�������λ*********************/

#define UART2_WordLength_8b                  ((uint8_t)0x00)
#define UART2_WordLength_9b                  ((uint8_t)0x40)                                  
#define IS_UART2_WORD_LENGTH(LENGTH)         (((LENGTH) == UART2_WordLength_8b) || \
                                             ((LENGTH) == UART2_WordLength_9b))
											 
/********************�����Ӵ���ֹͣλ*********************/

#define UART2_StopBits_1                     ((uint8_t)0x00)
#define UART2_StopBits_2                     ((uint8_t)0x80)
#define IS_UART2_STOPBITS(STOPBITS)          (((STOPBITS) == UART2_StopBits_1) || \
                                             ((STOPBITS) == UART2_StopBits_2) )
											 
/********************�����Ӵ�����żУ��λ*********************/

#define UART2_Parity_0                       ((uint8_t)0x00)
#define UART2_Parity_Even                    ((uint8_t)0x10)//żУ��
#define UART2_Parity_Odd                     ((uint8_t)0x08)//��У��
#define UART2_Parity_1                       ((uint8_t)0x18)
#define IS_UART2_PARITY(PARITY)              (((PARITY) == UART2_Parity_0) || \
                                             ((PARITY) == UART2_Parity_Even) || \
                                             ((PARITY) == UART2_Parity_Odd)	 || \
								             ((PARITY) == UART2_Parity_1)	)

/********************���������ڲ�����*********************/

#define UART22_BaudRate_0              		 ((uint8_t)0x00)
#define UART22_BaudRate_1                 	 ((uint8_t)0x10)
#define UART22_BaudRate_2                    ((uint8_t)0x20)
#define UART22_BaudRate_3                  	 ((uint8_t)0x30)
#define UART22_BaudRate_4                	 ((uint8_t)0x40)
#define UART22_BaudRate_5                 	 ((uint8_t)0x50)
#define UART22_BaudRate_6                	 ((uint8_t)0x60)
#define UART22_BaudRate_7                    ((uint8_t)0x70)
#define UART22_BaudRate_8               	 ((uint8_t)0x80)
#define UART22_BaudRate_9                 	 ((uint8_t)0x90)
#define UART22_BaudRate_10                   ((uint8_t)0xa0)
#define UART22_BaudRate_11                	 ((uint8_t)0xb0)
#define UART22_BaudRate_12                	 ((uint8_t)0xc0)
#define UART22_BaudRate_13                	 ((uint8_t)0xd0)
#define UART22_BaudRate_14                	 ((uint8_t)0xe0)
#define UART22_BaudRate_15                	 ((uint8_t)0xf0)
#define IS_UART22_BAUDRATE(BAUDRATE)		 (((BAUDRATE) == UART22_BaudRate_0  ) || \
                                 			 ((BAUDRATE) == UART22_BaudRate_1 ) || \
                                 			 ((BAUDRATE) == UART22_BaudRate_2 )    || \
								 			 ((BAUDRATE) == UART22_BaudRate_3	)   || \
								 			 ((BAUDRATE) == UART22_BaudRate_4	)   || \
											 ((BAUDRATE) == UART22_BaudRate_5	)   || \
											 ((BAUDRATE) == UART22_BaudRate_6  )	|| \
											 ((BAUDRATE) == UART22_BaudRate_7  )	|| \
											 ((BAUDRATE) == UART22_BaudRate_8	)   || \
											 ((BAUDRATE) == UART22_BaudRate_9	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_10	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_11	)   || \
											 ((BAUDRATE) == UART22_BaudRate_12	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_13	)	|| \
											 ((BAUDRATE) == UART22_BaudRate_14 )   || \
											 ((BAUDRATE) == UART22_BaudRate_15))

/********************��������������λ*********************/

#define UART22_WordLength_8b                  ((uint8_t)0x00)
#define UART22_WordLength_9b                  ((uint8_t)0x08)                                  
#define IS_UART22_WORD_LENGTH(LENGTH)         (((LENGTH) == UART_WordLength22_8b) || \
                                              ((LENGTH) == UART_WordLength22_9b))

/********************����������ֹͣλ*********************/

#define UART22_StopBits_1                     ((uint8_t)0x00)
#define UART22_StopBits_2                     ((uint8_t)0x04)
#define IS_UART22_STOPBITS(STOPBITS)          (((STOPBITS) == UART22_StopBits_1) || \
                                              ((STOPBITS) == UART22_StopBits_2) )
											  
/********************������������żУ��λ*********************/

#define UART22_Parity_0                     ((uint8_t)0x00)
#define UART22_Parity_Even                  ((uint8_t)0x02)//żУ��
#define UART22_Parity_Odd                   ((uint8_t)0x01)//��У��
#define UART22_Parity_1                     ((uint8_t)0x03)
#define IS_UART22_PARITY(PARITY)            (((PARITY) == UART22_Parity_0) || \
                                            ((PARITY) == UART22_Parity_Even) || \
                                            ((PARITY) == UART22_Parity_Odd)	 || \
								            ((PARITY) == UART22_Parity_1))
											
//�Ӵ��ڿ��ƼĴ���SCTLR								 
/********************�����Ӵ���ģʽ*********************/								 

#define UART2_Mode_RS232                    ((uint8_t)0x00)
#define UART2_Mode_NONE                     ((uint8_t)0x04)
#define IS_UART2_MODE(MODE) 			    (((MODE) == UART2_Mode_RS232) || \
                                            ((MODE) == UART2_Mode_NONE) ||)
//�Ӵ���FIFO���ƼĴ���
/********************����FIFO�������*********************/	

#define UART2_TFTL_0BYTE					((uint8_t)0x00)
#define UART2_TFTL_4BYTE					((uint8_t)0x40)
#define UART2_TFTL_8BYTE					((uint8_t)0x80)
#define UART2_TFTL_12BYTE				    ((uint8_t)0xC0)
#define IS_UART2_TFTL(TFTL) 		        (((TFTL) == UART2_TFTL_0BYTE	) || \
                                            ((TFTL) == UART2_TFTL_4BYTE	) || \
                                            ((TFTL) == UART2_TFTL_8BYTE	)	 || \
								            ((TFTL) == UART2_TFTL_12BYTE)	)

/********************����FIFO�������*********************/	

#define UART2_RFTL_1BYTE					((uint8_t)0x00)
#define UART2_RFTL_4BYTE					((uint8_t)0x10)
#define UART2_RFTL_8BYTE					((uint8_t)0x20)
#define UART2_RFTL_14BYTE					((uint8_t)0x30)
#define IS_UART2_RFTL(RFTL) 		 		(((RFTL) == UART2_RFTL_0BYTE	) || \
											((RFTL) == UART2_RFTL_4BYTE	) || \
											((RFTL) == UART2_RFTL_8BYTE	)	 || \
								            ((RFTL) == UART2_RFTL_12BYTE)	)

//�Ӵ����ж�ʹ�ܼĴ���SIER
/********************�ж�ʹ�ܼĴ���*********************/	

#define UART2_IT_FOEIEN                     ((uint8_t)0x40)		//FIFO���ݴ����ж�ʹ��λ
#define UART2_IT_TRIEN                      ((uint8_t)0x02)		//����FIFO�����ж�ʹ��λ
#define UART2_IT_RFIEN                      ((uint8_t)0x01)		//ʹ�ܽ���FIFO�����ж� 
#define IS_UART2_CONFIG_IT(IT) 				(((IT) == UART2_IT_FOEIEN ) || \
								            ((IT) == UART2_IT_TRIEN	) || \
								            ((IT) == UART2_IT_RFIEN	) )

/********************�ж��ж�״̬*********************/

#define IS_UART2_GET_IT(IT)		            (((IT) == UART2_IT_FOEIEN ) || \
								            ((IT) == UART2_IT_TRIEN	) || \
								            ((IT) == UART2_IT_RFIEN	) )

/********************����ж�*************************/

#define IS_UART2_CLEAR_IT(IT)	            (((IT) == UART2_IT_FOEIEN ) || \
								            ((IT) == UART2_IT_TRIEN	) || \
								            ((IT) == UART2_IT_RFIEN	) )

/********************�Ӵ���״̬�Ĵ���SSR*************************/

#define UART2_FLAG_OE                       ((uint8_t)0x80)		//�Ӵ��ڽ��� FIFO �е�ǰ����(����д��)����������־λ
#define UART2_FLAG_FE                       ((uint8_t)0x40)		//�Ӵ��ڽ��� FIFO �е�ǰ����(����д��)��֡�����־λ
#define UART2_FLAG_PE                       ((uint8_t)0x20)		//�Ӵ��ڽ��� FIFO �е�ǰ����(����д��)��У������־λ
#define UART2_FLAG_TFFL                     ((uint8_t)0x08)	   	//�Ӵ��ڷ��� FIFO ����־
#define UART2_FLAG_TFEM                     ((uint8_t)0x04)		//�Ӵ��ڷ��� FIFO �ձ�־
#define UART2_FLAG_TXBY                     ((uint8_t)0x02)		//�Ӵ��ڷ��� TX æ��־
#define UART2_FLAG_RFEM                     ((uint8_t)0x01)		//�Ӵ��ڽ��� FIFO �ձ�־
#define IS_UART2_FLAG(FLAG)  (((FLAG) == UART2_FLAG_OE)   || ((FLAG) == UART2_FLAG_FE) || \
                             ((FLAG) == UART2_FLAG_PE)    || ((FLAG) == UART2_FLAG_TFFL) || \
                             ((FLAG) == UART2_FLAG_TFEM ) || ((FLAG) == UART2_FLAG_TXBY) || \
                             ((FLAG) == UART2_FLAG_RFEM) )

void Uart22_Send_Data( uint8_t _cmd,uint8_t data );                                  //�����ڷ��ͺ���
uint8_t Uart22_Receive_Data( uint8_t value);                                         //�����ڽ��պ���
void Init_vk3214_UART( void );                                                       //����STM32��vk3214��UARTͨ��
void UART22_Base_Init( UART22_Base_InitTypeDef* UART22_InitStruct );		         //��ʼ��VK3224�����ڵĻ�������״̬
void Uart_Master_Config(void);                                                       //���������ڲ���
void UART22_StructInit( UART22_Base_InitTypeDef* UART22_InitStruct );                //��ʼ��vk3214�����ڵĽṹ�����
void UART2_Base_Init( uint8_t UARTx, UART2_Base_InitTypeDef* UART2_InitStruct );     //��ʼ��VK3224�Ӵ��ڵ�UARTx�Ļ�������״̬
void UART2_StructInit( UART2_Base_InitTypeDef* UART2_InitStruct );                   //��ʼ��vk3214��UARTx�Ľṹ�����
void UART2_FIFO_Init( uint8_t UARTx, UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct );//��ʼ��VK3224��UARTx����FIFO��������״̬
void UART2_FIFO_StructInit( UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct );         //��ʼ��vk3214��UARTx��FIFO�ṹ�����
void UART2_ITConfig( uint8_t UARTx, uint8_t UART2_IT, uint8_t FunctionalState );	 //����VK3224��UARTx�ж�
void UART2_Cmd( uint8_t UARTx, uint8_t FunctionalState );		                     //ʹ���Ӵ���
void UART2_SendData( uint8_t UARTx, uint8_t data );			                         //�Ӵ��ڷ�������
uint8_t UART2_ReceiveData( uint8_t UARTx );			                                 //�Ӵ��ڽ�������
uint8_t UART2__FIFO_ReceiveData( uint8_t UARTx );                                    //��UARTx��FIFO�Ĵ�������ֵ
uint8_t Get_IT_UARTx( void );						                                 //��ȡ�Ӵ����жϺ�
uint8_t UART2_GetFlagStatus( uint8_t UARTx, uint8_t UART2_FLAG );		             //��ѯ�Ӵ��ڵĸ���״̬
uint8_t UART2_GetITStatus( uint8_t UARTx, uint8_t UART2_IT);			             //��ѯ�Ӵ��ڵ��ж�״̬
void UART2_ClearITPendingBit( uint8_t UARTx, uint8_t UART2_IT );		             //����Ӵ��ڵ��жϱ�־λ
uint8_t UART2_Get_Num_TXFIFO( uint8_t UARTx );							             //��ѯ����FIFO���ֽ���
uint8_t UART2_Get_Num_RXFIFO( uint8_t UARTx );							             //��ѯ����FIFO���ֽ���
void UART2_Clear_TXFIFO( uint8_t UARTx );								             //��շ���FIFO
void UART2_Clear_RXFIFO( uint8_t UARTx );								             //��ս���FIFO
void Save_Uart_Data(u8 data);                                                        //���洮������
void UART_Slave1_Config(void);                                                       //�����Ӵ���1����
void UART_Slave2_Config(void);                                                       //�����Ӵ���2����
void UART_Slave3_Config(void);                                                       //�����Ӵ���3����
void UART_Slave4_Config(void);                                                       //�����Ӵ���4����
void UART_EX_Config(void);                                                           //�������д��ڲ���
 
#endif
