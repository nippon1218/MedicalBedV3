#include "pcf8574.h"
#include "delay.h"

/***********************************************************************
 ������      ��PCF8574_Init(void) 
 ��������    ��PCF8574��ʼ������
 ����        ����
 ���        ��temp��1������Ӧ��ʧ�ܣ�0������Ӧ��ɹ�
                           
************************************************************************/
u8 PCF8574_Init(void)
{
    u8 temp=0;
    GPIO_InitTypeDef GPIO_Initure;          //����ṹ�����GPIO_Initure
    __HAL_RCC_GPIOB_CLK_ENABLE();           //ʹ��GPIOBʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_12;           //PB12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //��ʼ��
    IIC_Init();					            //IIC��ʼ�� 	
	//���PCF8574�Ƿ���λ
    IIC_Start();                            //����IIC��ʼ�ź�	 	   
		IIC_Send_Byte(PCF8574_ADDR);            //д��ַ			   
		temp=IIC_Wait_Ack();		            //�ȴ�Ӧ��,ͨ���ж��Ƿ���ACKӦ��,���ж�PCF8574��״̬
    IIC_Stop();					            //����һ��ֹͣ����
    PCF8574_WriteOneByte(0XFF);	            //Ĭ�����������IO����ߵ�ƽ
	return temp;                            //����Ӧ���ź�                         
}

/***********************************************************************
 ������      ��PCF8574_ReadOneByte(void) 
 ��������    ����ȡPCF8574��8λIO��ֵ
 ����        ����
 ���        ������������
                           
************************************************************************/
u8 PCF8574_ReadOneByte(void)
{				  
	u8 temp=0;		  	    																 
    IIC_Start();                        //����IIC��ʼ�ź�  	 	   
	IIC_Send_Byte(PCF8574_ADDR|0X01);   //�������ģʽ			   
	IIC_Wait_Ack();	                    //�ȴ�Ӧ���źŵ���
    temp=IIC_Read_Byte(0);	            //��1���ֽ�	   
    IIC_Stop();							//����һ��ֹͣ����	    
	return temp;                        //���ض�ȡ����ֵ
}

/***********************************************************************
 ������      ��PCF8574_WriteOneByte(u8 DataToWrite) 
 ��������    ����PCF8574д��8λIOֵ 
 ����        ��DataToWrite:Ҫд�������
 ���        ����
                           
************************************************************************/
void PCF8574_WriteOneByte(u8 DataToWrite)
{				   	  	    																 
    IIC_Start();                        //����IIC��ʼ�ź�	
    IIC_Send_Byte(PCF8574_ADDR|0X00);   //����������ַ0X40,д���� 	 
	IIC_Wait_Ack();	                    //�ȴ�Ӧ���źŵ���    										  		   
	IIC_Send_Byte(DataToWrite);    	 	//�����ֽ�							   
	IIC_Wait_Ack();                     //�ȴ�Ӧ���źŵ���     
    IIC_Stop();							//����һ��ֹͣ���� 	 
}

/***********************************************************************
 ������      ��PCF8574_WriteBit(u8 bit,u8 sta) 
 ��������    ������PCF8574ĳ��IO�ĸߵ͵�ƽ 
 ����        ��bit:Ҫ���õ�IO���,0~7
               sta:IOҪ��λ��״̬;0���͵�ƽ��1���ߵ�ƽ
 ���        ����
                           
************************************************************************/
void PCF8574_WriteBit(u8 bit,u8 sta)
{
    u8 data;
    data=PCF8574_ReadOneByte(); //�ȶ���ԭ��������
    if(sta==0) data&=~(1<<bit); //��IO��0  
    else data|=1<<bit;          //��IO��1
    PCF8574_WriteOneByte(data); //д���µ�����
	
}

/***********************************************************************
 ������      ��PCF8574_ReadBit(u8 bit)
 ��������    ����ȡPCF8574��ĳ��IO��ֵ 
 ����        ��bit:Ҫ���õ�IO���,0~7
 ���        ����IO�ڵ�ֵ;0���͵�ƽ��1���ߵ�ƽ
                           
************************************************************************/
u8 PCF8574_ReadBit(u8 bit)
{
    u8 data;
    data=PCF8574_ReadOneByte(); //�ȶ�ȡ���8λIO��ֵ 
    if(data&(1<<bit))return 1;
    else return 0;   
}  
    

void BeepRun(u8 num,u16 time)
{
	u8 i;
	for(i=0;i<num;i++)
	{
		PCF8574_WriteBit(BEEP_IO,0);delay_ms(time);	//���Ʒ�������
		PCF8574_WriteBit(BEEP_IO,1);delay_ms(time);	//���Ʒ�����ͣ			
	}
}



