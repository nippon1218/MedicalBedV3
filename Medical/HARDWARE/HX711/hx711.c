#include "hx711.h"
#include "usart.h"
#include "delay.h"
#include "sys.h"

#define WeightN 10    //�����˲�ʱ��Ų���ֵ�ԵĶ��г���

//u8 WeightN=10;
unsigned long Weight_Shiwu;  //��õ�ʵ�������
//int WeightN;

/***********************************************************************
 ������      ��HX711_Init() 
 ��������    ����ʼ�����ش�����HX711��IO��
 ����        ����
 ���        ����
                          
************************************************************************/
void HX711_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();		    //����GPIOBʱ��
	__HAL_RCC_GPIOH_CLK_ENABLE();		    //����GPIOHʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_5;            //PB5
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������GPIO_MODE_OUTPUT_PP
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //��ʼ��
	
	GPIO_Initure.Pin=GPIO_PIN_10;           //PH10
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_NOPULL;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);     //��ʼ��
}


/***********************************************************************
 ������      ��HX711_Read()   
 ��������    �����غ���
 ����        ����
 ���        ���� 
 ˵��        ��4������Ϊ5kg�ĳ��ش��������������Ϊ20kg��
               AD����������ֵΪ8589933
                          
************************************************************************/
unsigned long HX711_Read(void)	       //����128���ɽ��ɼ����źŷŴ�128��
{
	unsigned long  val;                //��Ų�������
	unsigned char i; 
	
	HX711_DOUT_OUTPUT();               //DOUT����Ϊ���ģʽ
  	HX711_DOUT_OUT=1;                  //DOUT�ø�	
  	delay_us(1);
    HX711_SCK_OUTPUT();                //SCK����Ϊ���ģʽ
  	HX711_SCK_OUT=0;                   //ʹ��AD(SCK�õ�)	
  	val=0;
  	HX711_DOUT_INPUT();                //DOUT����Ϊ����ģʽ
	while(HX711_DOUT_IN);              //�ȴ�ADת������������ʼ��ȡ,���ȴ�HX711_DOUT_IN=0
    delay_us(1);
	for(i=0;i<24;i++)
	{ 
	  	HX711_SCK_OUT=1;               //SCK �ø�(��������)
	  	val=val<<1; 
		delay_us(1);
	    HX711_SCK_OUT=0;               //SCK �õ�
		HX711_DOUT_INPUT();            //����Ϊ����ģʽ��������Ÿߵ͵�ƽ
	    if(HX711_DOUT_IN)              //DOUT=1
	    {
	        val++;
			delay_us(1);
	    }				
	} 
	HX711_SCK_OUT=1;
	val=val^0x800000;                  //��25�������½�����ʱ��ת������
  	delay_us(2);
	HX711_SCK_OUT=0;
	delay_us(1);
	return(val);
}

/***********************************************************************
 ������      ��Get_Weight()  
 ��������    ���ɼ����ݺ���
 ����        ����
 ���        ���� 
                          
************************************************************************/
unsigned long Get_Weight(void)
{   
	unsigned long HX711_Buffer = 0;
     
    Weight_Shiwu = 0;
	HX711_Buffer = HX711_Read();
	HX711_Buffer = HX711_Buffer/100;                                //���ں�������
		 
	Weight_Shiwu = HX711_Buffer;                                    //��ȡʵ���AD��������	 
	Weight_Shiwu = (unsigned int)((float)Weight_Shiwu /4.22+0.05); 	//����ʵ���ʵ��������4.22����ֵ���ɵ���
																	//��������������ƫС����С����ֵ��һ����4.0-5.0֮��
//	Weight_Shiwu =(unsigned int)((18128-Weight_Shiwu )/0.6012);																//0.05Ϊ����������ٷ�λ
//	u2_printf("\r\nWeight_Shiwu=%d\r\n",Weight_Shiwu);	
    return(Weight_Shiwu);
} 


/***********************************************************************
 ������      ��filter() 
 ��������    �����ݴ����������������ɼ������ݽ��д���
 ����        ����
 ���        ��sum/(N - 2) 
                          
************************************************************************/
unsigned long filter(void)
{
	unsigned long temp =0;
	unsigned long t;
	unsigned long r;
	unsigned long s;
	unsigned long value_buf[WeightN];    //��Ųɼ���N����
	long sum=0;                    //��Ųɼ����ݵĺ�
	
	for(t = 0;t<WeightN;t++)             //�ɼ�ʮ������	
	{
		value_buf[t] =Get_Weight();
		delay_us(1);
	}
	for(r = 0;r < WeightN - 1;r++)      //��N�����ݴ�С��������
	{
	 	for (s=0;s<WeightN-r;s++)            
	 	{
	 		if(value_buf[s] > value_buf[s+1])
	 		{
	 			temp = value_buf[s];
	 			value_buf[s]   = value_buf[s+1];
	 			value_buf[s+1] = temp;	
	  	}
	 	}
	}
	for(t = 3;t <WeightN -3 ;t++)     //��ȡ�м��ĸ����󱾴β���ƽ��ֵ
	{
		sum += value_buf[t];
	}
	sum=(long)(sum/(WeightN- 6));
	sum =(long)((18158-sum )/0.6012);
	if(sum < 0)
		return -sum;
	else
		return sum;
}



