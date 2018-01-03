#include "vk3214.h"
#include "usart.h"
#include "sys.h"
#include "led.h"
#include "delay.h"

u16 UART_RECEIVE=0;
u8 UART_SAVE_BUFFER[UART_RECEIVE_LEN];

/***************************************************************
 * ��������Uart22_Send_Data();
 * ����  ���������ݵ�vk3214
 * ����  : uint8_t value
 * ���  ��uint8_t rxd
 * ����  ����
 * ע��  ����
 
*****************************************************************/

void Uart22_Send_Data( uint8_t _cmd,uint8_t data )//��һ���������������ֽڣ��ڶ�����������Ҫ���͵�����
{
  u8 i; 
	for(i=0;i<1;i++)							  //ѭ����������
	{
		while((USART6->SR&0X40)==0);			  //ѭ������,ֱ���������   
		USART6->DR=_cmd;
		USART2->DR=_cmd; 		
	} 
//	delay_us (50);				
	for(i=0;i<1;i++)							  //ѭ����������
	{
		while((USART6->SR&0X40)==0);			  //ѭ������,ֱ���������   
		USART6->DR=data;
		USART2->DR=data; 		
	} 	
}

/***************************************************************
 * ��������UART22_Receive_Data();
 * ����  ����vk3214��ȡ����
 * ����  : uint8_t value
 * ���  ��uint8_t  rbuf
 * ����  ����
 * ע��  ����

*****************************************************************/
uint8_t Uart22_Receive_Data( uint8_t data )
{
	u16 i; 
	for(i=0;i<1;i++)							//ѭ����������
	{
		while((USART6->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART6->DR=data;
		USART2->DR=data;
	}   
		delay_us (250);
//		for(i=0;i<1;i++)					    //ѭ����������
//	{
//		 while((USART2->SR&0X40)==0);			//ѭ������,ֱ���������  
////     delay_ms(1);		
//		 USART2->DR=UART_SAVE_BUFFER[0]; 
//	} 	
   return UART_SAVE_BUFFER[0];
}


/***************************************************************
 * ��������Init_vk3214_UART( );
 * ����  ������STM32��vk3214��UARTͨ��
 * ����  : ��
 * ���  ����
 * ����  ����
 * ע��  ����

*****************************************************************/

void Init_vk3214_UART( void )
{
	uint8_t flag ,i;
	uint8_t ch, addr, data1, data2, data;
	ch = COM1;
	addr = SCTLR ;
	data1 = 0X80 | ch | addr;
    data2	= UART22_BaudRate_1;
	Uart22_Send_Data( data1, data2 );    //�������ݵ�vk3214	
	delay_ms(10);		
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 ); //��vk3214��ȡ���ݣ�8λ
	  
	if( data != UART22_BaudRate_1 ) 
	{
		flag=0;			
		for(i=0; i<4; i++)
		{
			LED1=!LED1;                 //��ͨ�Ų��ɹ�����LED1��2��
			LED0=!LED0;
			delay_ms(10);
		}
			u2_printf("\r\nͨ�Ų��ɹ��������³���\r\n");			
	}
	else
	{
		flag=1;			
		for(i=0; i<4; i++)
		{
			LED0=!LED0;                 //��ͨ�ųɹ�����LED0��2��
			delay_ms(10);
		}
			u2_printf("\r\nVK3214ͨ�ųɹ�\r\n");			
	}
}

/***************************************************************
 * ��������UART_Base_Init;
 * ����  ����ʼ��VK3214�����ڵĻ�������״̬
 * ����  : uint8_t UARTx��UART_Base_InitTypeDef* UART_InitStruct
 * ���  ����
 * ����  ��UART_Base_Init( COM1,  &UART1_InitStruct )
 * ע��  ����
	
*****************************************************************/
                   
void UART22_Base_Init( UART22_Base_InitTypeDef* UART22_InitStruct )
{
	uint8_t data, addr, data1, data2;
	 /* Check the parameters */
  	assert_param(IS_UART22_BAUDRATE(UART22_InitStruct->UART22_BaudRate));  
  	assert_param(IS_UART22_WORD_LENGTH(UART22_InitStruct->UART22_WordLength));
  	assert_param(IS_UART22_STOPBITS(UART22_InitStruct->UART22_StopBits));
  	assert_param(IS_UART22_PARITY(UART22_InitStruct->UART22_Parity));                         
	addr  = GMUCR ;
	data1 = 0x80 | addr;
	data2 = (UART22_InitStruct->UART22_BaudRate) |  (UART22_InitStruct->UART22_WordLength) | ( UART22_InitStruct->UART22_StopBits ) | ( UART22_InitStruct->UART22_Parity );    
	Uart22_Send_Data( data1 ,data2 );      //����GMUCR
	delay_ms(10);		
	data1 = addr;
	data = Uart22_Receive_Data( data1 );   //��GMUCR�Ĵ����е�����   
    if(data==data2)
	{
		u2_printf("\r\n���������óɹ�\r\n");
	}
	else
	{
		u2_printf("\r\n����������ʧ��\r\n");
	}			
}

/***************************************************************
 * ��������UART_StructInit;
 * ����  ����ʼ��vk3214�����ڵĽṹ�����
 * ����  : UART_Base_InitTypeDef* UART_InitStruc
 * ���  ����
 * ����  ��UART_StructInit( &UART1_InitStruct )
 * ע��  ����

*****************************************************************/
void UART22_StructInit( UART22_Base_InitTypeDef* UART22_InitStruct )
{
/* UART_InitStruct members default value */
	UART22_InitStruct->UART22_BaudRate = UART22_BaudRate_1;
	UART22_InitStruct->UART22_WordLength = UART22_WordLength_8b;
	UART22_InitStruct->UART22_StopBits = UART22_StopBits_1;
	UART22_InitStruct->UART22_Parity = UART22_Parity_0;
}

/***************************************************************
 * ��������UART_Base_Init;
 * ����  ����ʼ��vk3214��UARTx�Ļ�������״̬--4���Ӵ���
 * ����  : uint8_t UARTx��UART_Base_InitTypeDef* UART_InitStruct
 * ���  ����
 * ����  ��UART_Base_Init( COM1,  &UART1_InitStruct )
 * ע��  ����

*****************************************************************/
                   
void UART2_Base_Init( uint8_t UARTx, UART2_Base_InitTypeDef* UART2_InitStruct )
{
	uint8_t ch, data, addr, data1, data2;
	 /* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(USARTx));//�ж����ĸ��Ӵ���
  	assert_param(IS_UART2_BAUDRATE(UART2_InitStruct->UART2_BaudRate));  
  	assert_param(IS_UART2_WORD_LENGTH(UART2_InitStruct->UART2_WordLength));
  	assert_param(IS_UART2_STOPBITS(UART2_InitStruct->UART2_StopBits));
  	assert_param(IS_UART2_PARITY(UART2_InitStruct->UART2_Parity));
  	assert_param(IS_UART2_MODE(UART2_InitStruct->UART2_Mode));

	ch = UARTx;                          //SCTLR����
	addr = SCTLR ;
	data1 = ch | 0x80 | addr ;	
    data2 =(UART2_InitStruct->UART2_BaudRate) | (UART2_InitStruct->UART2_Mode);
    data2=data2 | 0x08;                  //ʹ���Ӵ��ڽ��������շ�
	Uart22_Send_Data( data1 ,data2 );    //����SCTLR	
    delay_ms(15);
	
	data1 = ch | addr;	
	data = Uart22_Receive_Data( data1 ); //��ȡSCTLR�Ĵ����е�����
	delay_ms(15);
    if(data==data2)
	{
	  u2_printf("\r\n�Ӵ��ڲ��������óɹ�\r\n"); delay_ms(10);
	}
	else
	{
		 u2_printf("\r\n�Ӵ��ڲ���������ʧ��\r\n"); delay_ms(10);
	}

	addr = SCONR ;
	data1 = ch | 0x80 | addr ;		
	data2 =(UART2_InitStruct->UART2_WordLength) | ( UART2_InitStruct->UART2_StopBits ) | ( UART2_InitStruct->UART2_Parity );
	delay_ms(5);
	Uart22_Send_Data( data1, data2 );     //����SCONR
    delay_ms(15);
		
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
    delay_ms(15);		
    if(data==data2)
	{
		 u2_printf("\r\n�Ӵ������ݳ���У��ģʽ���óɹ�\r\n");    delay_ms(10);
	}
	else
	{
		u2_printf("\r\n�Ӵ������ݳ���У��ģʽ����ʧ��\r\n");    delay_ms(10);
	}
}


/***************************************************************
 * ��������UART_StructInit;
 * ����  ����ʼ��vk3214��UARTx�Ľṹ�����
 * ����  : UART_Base_InitTypeDef* UART_InitStruc
 * ���  ����
 * ����  ��UART_StructInit( &UART1_InitStruct )
 * ע��  ����

*****************************************************************/
void UART2_StructInit( UART2_Base_InitTypeDef* UART2_InitStruct )
{
/* UART_InitStruct members default value */
	UART2_InitStruct->UART2_BaudRate = UART2_BaudRate_1;  //�Ӵ��ڲ���������Ϊ115200
	UART2_InitStruct->UART2_WordLength = UART2_WordLength_8b;
	UART2_InitStruct->UART2_StopBits = UART2_StopBits_1;
	UART2_InitStruct->UART2_Parity = UART2_Parity_0;
	UART2_InitStruct->UART2_Mode = UART2_Mode_RS232;
}


/***************************************************************
 * ��������UART_FIFO_Init;
 * ����  ����ʼ��vk3214��UARTx����FIFO��������״̬
 * ����  : uint8_t UARTx; UART_FIFO_InitTypeDef* UART_FIFO_InitStruct
 * ���  ����
 * ����  ��UART_FIFO_Init( COM1,  &UART1_FIFO_InitStruct )
 * ע��  ����

*****************************************************************/

void UART2_FIFO_Init( uint8_t UARTx, UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct )
{
	uint8_t ch, data, addr, data1, data2;
	
	/* Check the parameters */
	assert_param(IS_UART2_ALL_PERIPH(USARTx));
  	assert_param(IS_UART2_TFTL(UART2_FIFO_InitStruct->UART2_TFTL));  
  	assert_param(IS_UART2_RFTL(UART2_FIFO_InitStruct->UART2_RFTL));
  	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_TFEN));
  	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_RFEN));
  	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_TFCL));
	assert_param(IS_FUNCTION_STATE(UART2_FIFO_InitStruct->UART2_RFCL));		

	ch = UARTx;
	addr = SFOCR ;
	data2 = (UART2_FIFO_InitStruct->UART2_TFTL) | (UART2_FIFO_InitStruct->UART2_RFTL) | (UART2_FIFO_InitStruct->UART2_RFCL);
	data = ((UART2_FIFO_InitStruct->UART2_TFEN) << 3); //����FIFOʹ��λ
	data2 = data | data2;
	data = (UART2_FIFO_InitStruct->UART2_RFEN) << 2;   //����FIFOʹ��λ
	data2 = data | data2;
	data = (UART2_FIFO_InitStruct->UART2_TFCL) << 1;   //�������FIFOλ
	data2 = data | data2;	 
	data1 = ch | 0x80 | addr ;                         //����SFOCR �Ӵ���FIFO���ƼĴ���
	Uart22_Send_Data( data1, data2 );
	delay_ms(5);
	
	data1 = ch | addr;		
	data = Uart22_Receive_Data( data1 );               //��ȡSFOCR�Ĵ����е�����
	delay_ms(5);
	if(data==data2)
	{
		u2_printf("\r\n\r\nSFOCR�Ĵ������óɹ�\r\n");
	}
	else
	{
		u2_printf("\r\n\r\nSFOCR�Ĵ�������ʧ��\r\n"); 
	}	
}

/***************************************************************
 * ��������UART_FIFO_StructInit;
 * ����  ����ʼ��vk3214��UARTx��FIFO�ṹ�����
 * ����  : UART_FIFO_InitTypeDef* UART_FIFO_InitStruct
 * ���  ����
 * ����  ��UART_FIFO_StructInit( &UART1_FIFO_InitStruct )
 * ע��  ����

*****************************************************************/

void UART2_FIFO_StructInit( UART2_FIFO_InitTypeDef* UART2_FIFO_InitStruct )
{
/* UART_FIFO_InitStruct members default value */

	UART2_FIFO_InitStruct->UART2_TFTL = UART2_TFTL_0BYTE	;    //��������SFOCR
	UART2_FIFO_InitStruct->UART2_RFTL = UART2_RFTL_1BYTE	;    //��ֹ���ͣ���ֹ����FIFO,��������͡�����FIFO�е���������
	UART2_FIFO_InitStruct->UART2_TFEN = SET;
	UART2_FIFO_InitStruct->UART2_RFEN = SET;
	UART2_FIFO_InitStruct->UART2_TFCL = RESET;
	UART2_FIFO_InitStruct->UART2_RFCL = RESET;
}

/***************************************************************
 * ��������UART_ITConfig;
 * ����  ������vk3214��UARTx�ж�
 * ����  :uint8_t UARTx, uint8_t UART_IT, uint8_t FunctionalState
 * ���  ����
 * ����  ��UART_ITConfig( UART1, UART_IT_FOEIEN , SET )
 * ע��  ����

*****************************************************************/

void UART2_ITConfig( uint8_t UARTx, uint8_t UART2_IT, uint8_t FunctionalState )//����UART_IT����Ҫ���õ��ж�����
{
	uint8_t ch, addr;
	uint8_t data1, data2, data ;
	uint8_t IR;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_UART2_CONFIG_IT(UART2_IT));      //ʹ�����ݴ����жϡ����ʹ����жϣ����մ����ж�
  	assert_param(IS_FUNCTION_STATE(FunctionalState));//����Ϊ0��1

	ch = UARTx;
	addr = SIER ;
	data1 =  ch | addr ;
	data = Uart22_Receive_Data( data1 );             //��ȡ�Ӵ����ж�ʹ�ܼĴ������������	
	IR = Uart22_Receive_Data( 0x03 );			     //��GIR�Ĵ�����ֵ

/*	�ر�UARTx��ȫ���ж�		*/                
	if( UARTx == COM1 )                              //ȫ���жϼĴ���������GIR,�Ȱ������Ӵ����жϹر�
	{
		data1 = 0x83;
		data2 = (IR&0XEF);
		Uart22_Send_Data( data1, data2 );            //�ر��Ӵ���1�ж�
	}
	else if( UARTx == COM2 ) 
	{		
		data1 = 0x83;
		data2 = (IR&0XdF);
		Uart22_Send_Data( data1, data2 );            //�ر��Ӵ���2�ж�
	}
	else if( UARTx == COM3 ) 
	{			
		data1 = 0x83;
		data2 = (IR&0XbF);
		Uart22_Send_Data( data1, data2 );            //�ر��Ӵ���3�ж�
	}
	else
	{
		data1 = 0x83;
		data2 = (IR&0X7F);
		Uart22_Send_Data( data1, data2 );            //�ر��Ӵ���4�ж�
	}
				
	if( FunctionalState == SET )//��GIR�ж�Ӧ�����жϿڣ������Ӵ����жϼĴ����ض��ж�λ��������
	{		
		data1 = 0x80 | ch | addr ;
		data2 = data | UART2_IT;
		Uart22_Send_Data( data1, data2 );            //����SIER

	/*	ʹ��UARTx��ȫ���ж�		*/
		if( UARTx == COM1 ) 
		{			
			data1 = 0x83;
			data2 = (IR | 0X10);
			Uart22_Send_Data( data1, data2 );       //ʹ���Ӵ���1�ж�
		}
		else if( UARTx == COM2 ) 
		{			
			data1 = 0x83;
			data2 = (IR | 0X20);
			Uart22_Send_Data( data1, data2 );       //ʹ���Ӵ���2�ж�
		}
		else if( UARTx == COM3 ) 
		{			
			data1 = 0x83;
			data2 = (IR | 0X40);
			Uart22_Send_Data( data1, data2 );       //ʹ���Ӵ���3�ж�
		}
		else           
		{	
			data1 = 0x83;
			data2 = (IR | 0X80);
			Uart22_Send_Data( data1, data2 );        //ʹ���Ӵ���4�ж�
		}
	}
	else            //�����Ӵ����жϣ�ֻ���Ӵ�����Ӧ�ж�λ��������
	{
		if( UART2_IT == UART2_IT_FOEIEN ) 		data2 = data & 0xbf;
		else if( UART2_IT == UART2_IT_TRIEN )	data2 = data & 0xfd;
		else 									data2 = data & 0xfe;

		data1 =  0x80 | ch | addr ;
		Uart22_Send_Data( data1, data2 );
	}
}

/***************************************************************
 * ��������UART_Cmd;
 * ����  ��ʹ���Ӵ���
 * ����  : uint8_t UARTx, uint8_t UART_IT, uint8_t FunctionalState
 * ���  ����
 * ����  ��UART_Cmd( COM1, SET )
 * ע��  ����
*****************************************************************/

void UART2_Cmd( uint8_t UARTx, uint8_t FunctionalState )//�Լ�����ʹ�ܻ�ʹ��ĳ���Ӵ��ڣ�FunctionalState=1ʹ�ܣ�FunctionalState=0��ʹ��
{
	uint8_t ch, data, data1, data2, addr;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_FUNCTION_STATE(FunctionalState));

  	ch = UARTx;
  	addr = SCTLR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );

	if(  FunctionalState == SET )
	{
		data1 = 0x80 | ch | addr  ;
		data2 = data | 0x08 ;
		Uart22_Send_Data( data1, data2 );//ʹ���Ӵ��ڣ����Խ������������շ�
	}
	else
	{
		data1 = 0x80 | ch | addr  ;	
		data2 = ( data & 0xF7);	
		Uart22_Send_Data( data1, data2 );//��ʹ���Ӵ��ڣ���ʱ���Ӵ��ڲ��ܽ��������շ�
	}
}

/***************************************************************
 * ��������UART_SendData;
 * ����  ���Ӵ��ڷ�������
 * ����  : uint8_t UARTx,  uint8_t data
 * ���  ����
 * ����  ��UART_SendData( com1, 0xffff )
 * ע��  ��

*****************************************************************/

void UART2_SendData( uint8_t UARTx, uint8_t data )
{
	uint8_t ch, data1,data2, addr;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
	
	ch = UARTx;
	addr = SFDR ;
	data1 = 0x80 | ch | addr ;
	data2 = data ;

	Uart22_Send_Data( data1, data2 );
}

/***************************************************************
 * ��������UART_ReceiveData;
 * ����  ���Ӵ��ڽ�������
 * ����  : uint8_t UARTx
 * ���  ����UARTx��FIFO�Ĵ�������ֵ
 * ����  ��UART_ReceiveData( com1 )
 * ע��  ��

*****************************************************************/

uint8_t UART2_ReceiveData( uint8_t UARTx )
{
	uint8_t ch, addr, data1 ;
	uint8_t data,data2;
	 
  /* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  
  /* Receive Data */
	ch =  UARTx;
	addr = SFDR ;
	data1 = ch | addr;

	data = Uart22_Receive_Data( data1 );
	data2=data;
//	delay_us(100);	
	UART2_SendData( ch, data2 );
	return data ;
}

/***************************************************************
 * ��������UART2__FIFO_ReceiveData();
 * ����  ��ͨ������UARTxͨ���������һ�ε�����
 * ����  : UARTx���Ӵ��ں�
 * ���  ����UARTx��FIFO�Ĵ�������ֵ
 * ����  ��UART_ReceiveData( com1 )
 * ע��  ��

*****************************************************************/
uint8_t UART2__FIFO_ReceiveData( uint8_t UARTx )
{
	uint8_t ch, addr, data1 ;
	uint8_t data,data2;
	 
  /* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  
  /* Receive Data */
	ch =  UARTx;
//	addr = SFDR ;
	data1 = ch | 0x40;

	data = Uart22_Receive_Data( data1 );
	data2=data;
//	delay_us(100);
	
	UART2_SendData( ch, data2 );
	return data ;
}

/***************************************************************
 * ��������Get_IT_UARTx;
 * ����  :��ȡ�Ӵ����жϺ�
 * ����  :��
 * ���  �������Ӵ��ں�
 * ����  ��
 * ע��  ��

*****************************************************************/

uint8_t Get_IT_UARTx( void )
{
	uint8_t addr, data1 ;
	uint8_t data;

	addr = GIR ;
	data1 = addr;
	data = Uart22_Receive_Data( data1 );	
	data = data & 0x0f;

	if( (data & 0x08) == 0x08 )				  return COM4;
    else if( (data & 0x04) == 0x04 )		  return COM3;
	else if( (data & 0x02) == 0x02 )		  return COM2;
	else									  return COM1;
}	

/***************************************************************
 * ��������UART_GetFlagStatus;
 * ����  :��ѯ�Ӵ��ڵĸ���״̬
 * ����  :uint8_t UARTx, uint8_t UART_FLAG
 * ���  ��
 * ����  ��UART_GetFlagStatus( COM1, UART_FLAG_OE);
 * ע��  ��

*****************************************************************/

uint8_t UART2_GetFlagStatus( uint8_t UARTx, uint8_t UART2_FLAG )//UART_FLAGΪҪ��ѯ��ĳλ״̬��
{
	uint8_t ch, addr, data1;
	uint8_t data;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_UART2_FLAG(UART2_FLAG));

  	ch = UARTx;
  	addr = SSR ;
  	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );     //��ȡSSR�Ĵ���������

	if( (data & UART2_FLAG) == UART2_FLAG ) 	return SET;
	else 									    return RESET;
}

/***************************************************************
 * ��������UART_GetITStatus;
 * ����  :��ѯ�Ӵ��ڵ��ж�״̬
 * ����  :uint8_t UARTx, uint8_t UART_IT
 * ���  ��
 * ����  ��UART_GetITStatus( COM1, UART_IT_FOEIEN );
 * ע��  ��

*****************************************************************/

uint8_t UART2_GetITStatus( uint8_t UARTx, uint8_t UART2_IT)
{
	uint8_t ch, addr, data1;
	uint8_t data;
	
	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
 	assert_param(IS_UART2_GET_IT(UART_IT));

	ch = UARTx;
	addr = SIFR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );    //��ȡ�Ӵ���SIFR�Ĵ�������ֵ�����жϷ����˺����ж�

	if( (data & UART2_IT)  == UART2_IT ) 	return 	SET;
	else							        return  RESET;
}
	
/***************************************************************
 * ��������UART_ClearITPendingBit;
 * ����  :����Ӵ��ڵ��жϱ�־λ
 * ����  :uint8_t UARTx, uint8_t UART_IT
 * ���  ��
 * ����  ��UART_ClearITPendingBit( COM1, UART_IT_FOEIEN );
 * ע��  ��

*****************************************************************/

void UART2_ClearITPendingBit( uint8_t UARTx, uint8_t UART2_IT )
{	
	uint8_t ch, addr, data1, data2;
	uint8_t data, IFR;
	
	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
  	assert_param(IS_UART2_CLEAR_IT(UART2_IT));
	
	ch = UARTx;
	addr = SIFR ;
	data1 = ch | addr;
	IFR = Uart22_Receive_Data( data1 );
	if(  UART2_IT == UART2_IT_FOEIEN  )  	data2 = IFR & 0XBF;   //����Ӵ���FIFO���ݴ����жϱ�־
	else if( UART2_IT == UART2_IT_TRIEN )  	data2 = IFR & 0Xfd;   //����Ӵ��ڷ���FIFO�����жϱ�־λ
	else									data2 = IFR & 0Xfe;   //����Ӵ��ڽ���FIFO�����жϱ�־λ	
	data1 = data1 | 0x80 ;
	Uart22_Send_Data( data1, data2 );
}

/***************************************************************
 * �������� UART_Get_Num_TXFIFO;
 * ����  : ��ѯ����FIFO���ֽ���
 * ����  : uint8_t UARTx
 * ���  ��uint8_t num;
 * ����  ��UART_Get_Num_TXFIFO( COM1 )
 * ע��  ��

*****************************************************************/

uint8_t UART2_Get_Num_TXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data;
	uint8_t num;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFSR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data >> 4;
	num = ( (uint8_t) data );
	return num;
}

/***************************************************************
 * �������� UART_Get_Num_RXFIFO;
 * ����  : ��ѯ����FIFO���ֽ���
 * ����  : uint8_t UARTx
 * ���  ��uint8_t num;
 * ����  ��UART_Get_Num_RXFIFO( COM1 )
 * ע��  ��

*****************************************************************/

uint8_t UART2_Get_Num_RXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data;
	uint8_t num;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFSR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data & 0X0F;
	num = ( (uint8_t) data );
	return num;
}

/***************************************************************
 * �������� UART_Clear_TXFIFO
 * ����  : ��շ���FIFO
 * ����  : uint8_t UARTx
 * ���  ����
 * ����  ��UART_Clear_TXFIFO( COM1 )
 * ע��  ��

*****************************************************************/

void UART2_Clear_TXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data, data2 ;

	/* Check the parameters */
	assert_param(IS_UART2_ALL_PERIPH(UARTx));

	ch = UARTx;
	addr = SFOCR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data | 0x02;
	data1 = 0x80 | ch | addr ;
	data2 = data;
	Uart22_Send_Data( data1, data2 );                                //��շ���FIFO�е���������
	while( UART2_GetFlagStatus(  UARTx, UART2_FLAG_TFEM ) == RESET );//�ж��Ӵ��ڷ���FIFO�Ƿ�Ϊ�գ����Ƿ�������	
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data & 0xfd;                                              //���������FIFO�еļĴ���
	data1 = 0x80 | ch | addr ; 
	data2 = data; 
	Uart22_Send_Data( data1, data2 );
}

/***************************************************************
 * �������� UART_Clear_RXFIFO
 * ����  : ��ս���FIFO
 * ����  : uint8_t UARTx
 * ���  ����
 * ����  ��UART_Clear_RXFIFO( COM1 )
 * ע��  ��

*****************************************************************/

void UART2_Clear_RXFIFO( uint8_t UARTx )
{
	uint8_t ch, addr, data1, data2, data;

	/* Check the parameters */
  	assert_param(IS_UART2_ALL_PERIPH(UARTx));
	ch = UARTx;
	addr = SFOCR ;
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data | 0x01;
	data1 = 0x80 | ch | addr ;                                       //�������FIFO�Ĵ���������
	data2 = data;
	Uart22_Send_Data( data1, data2 );
	while( UART2_GetFlagStatus(  UARTx, UART2_FLAG_RFEM ) == RESET );//�Ӵ��ڽ���FIFO��
	data1 = ch | addr;
	data = Uart22_Receive_Data( data1 );
	data = data & 0xfE;
	data1 = 0x80 | ch | addr ;                                       //���������FIFO�е�����
	data2 = data;
	Uart22_Send_Data( data1, data2);
}

/***************************************************************
 * �������� Save_Uart_Data
 * ����  : ���洮������
 * ����  : u8 data
 * ���  ����

*****************************************************************/
void Save_Uart_Data(u8 data)
{ 
	UART_SAVE_BUFFER[0]=data;
}

/***************************************************************
 * ��������Uart_Master_Config;
 * ����  �����������ڲ���
 * ����  : ��
 * ���  ����
 * ����  ��
 * ע��  ����
*****************************************************************/

void Uart_Master_Config(void)
{
	UART22_Base_InitTypeDef UART22_InitStruct;	
	UART22_InitStruct.UART22_BaudRate = UART22_BaudRate_1;
  	UART22_InitStruct.UART22_WordLength = UART22_WordLength_8b;
	UART22_InitStruct.UART22_StopBits = UART22_StopBits_1;
  	UART22_InitStruct.UART22_Parity = UART22_Parity_0;
	UART22_Base_Init(  &UART22_InitStruct );
//	  delay_ms(10);
//	  u2_printf( "\r\n����������,������115200,����λ��8λ,����żУ��\r\n" );
    usart6_init(115200);
}

/***************************************************************
 * ��������UART_Slave1_Config;
 * ����  �������Ӵ���1����
 * ����  : ��
 * ���  ����
 * ����  ��
 * ע��  ����

*****************************************************************/
void UART_Slave1_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);	
	
	UART2_Base_Init( COM1, &UART2_InitStruct );	
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);
	UART2_FIFO_Init( COM1, &UART2_FIFO_InitStruct );
	
	UART2_Clear_TXFIFO( COM1 );
	UART2_Clear_RXFIFO( COM1 );
	UART2_ITConfig( COM1, UART2_IT_RFIEN, SET );
		
	UART2_Cmd( COM1, SET );
		
//		delay_ms(100);
//		u2_printf( "\r\n���ô���1;�����ʣ�14400(��֪Ϊ�Σ�ʵ��ֵΪ230400)������λ��8λ������żУ��\r\n" );
//		delay_ms(100);
}


/***************************************************************
 * ��������UART_Slave2_Config;
 * ����  �������Ӵ���2����
 * ����  : ��
 * ���  ����
 * ����  ��
 * ע��  ����

*****************************************************************/
void UART_Slave2_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);
	UART2_Base_Init( COM2, &UART2_InitStruct );
	
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);	
	UART2_FIFO_Init( COM2, &UART2_FIFO_InitStruct );
	UART2_Clear_TXFIFO( COM2 );
	UART2_Clear_RXFIFO( COM2 );
	UART2_ITConfig( COM2, UART2_IT_RFIEN, SET );
	UART2_Cmd( COM2, SET );
//		delay_ms(100);
//		u2_printf( "\r\n ���ô���2;�����ʣ�14400������λ��8λ������żУ��\r\n" );
//		delay_ms(100);
}

/***************************************************************
 * ��������UART_Slave3_Config;
 * ����  �������Ӵ���3����
 * ����  : ��
 * ���  ����
 * ����  ��
 * ע��  ����

*****************************************************************/
void UART_Slave3_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);	
	UART2_Base_Init( COM3, &UART2_InitStruct );
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);
	UART2_FIFO_Init( COM3, &UART2_FIFO_InitStruct );
	UART2_Clear_TXFIFO( COM3 );
	UART2_Clear_RXFIFO( COM3 );
	UART2_ITConfig( COM3, UART2_IT_RFIEN, SET );
	UART2_Cmd( COM3, SET );
//		delay_ms(100);
//		u2_printf( "\r\n ���ô���3;�����ʣ�14400������λ��8λ��żУ��\r\n" );
//		delay_ms(100);
}

/***************************************************************
 * ��������UART_Slave4_Config;
 * ����  �������Ӵ���4����
 * ����  : ��
 * ���  ����
 * ����  ��
 * ע��  ����

*****************************************************************/
void UART_Slave4_Config(void)
{
	UART2_Base_InitTypeDef UART2_InitStruct;
	UART2_FIFO_InitTypeDef UART2_FIFO_InitStruct;
    UART2_StructInit(&UART2_InitStruct);
	UART2_Base_Init( COM4, &UART2_InitStruct );	
    UART2_FIFO_StructInit(&UART2_FIFO_InitStruct);
	UART2_FIFO_Init( COM4, &UART2_FIFO_InitStruct );

	UART2_Clear_TXFIFO( COM4 );

	UART2_Clear_RXFIFO( COM4 );

	UART2_ITConfig( COM4, UART2_IT_RFIEN, SET );
	UART2_Cmd( COM4, SET );
//		delay_ms(100);
//		u2_printf( "\r\n ���ô���4;�����ʣ�14400������λ��8λ��żУ��\r\n" );
//		delay_ms(100);
}

/***************************************************************
 * ��������UART_EX_Config;
 * ����  �����ô��ڲ���
 * ����  : ��
 * ���  ����
 * ����  ��
 * ע��  ����

*****************************************************************/
void UART_EX_Config(void)
{
	Uart_Master_Config();
	delay_ms(10);
	UART_Slave1_Config();
	delay_ms(10);
	UART_Slave2_Config();
	delay_ms(10);
	UART_Slave3_Config();
	delay_ms(10);
	UART_Slave4_Config();	
}