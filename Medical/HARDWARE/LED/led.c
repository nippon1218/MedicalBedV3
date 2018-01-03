#include "led.h"
#include "usart.h"
#include "delay.h"
/***********************************************************************
 ������      ��LED_Init(void)  
 ��������    ��LED��ʼ������
 ����        ����
 ���        ����
                           
************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;          //����ṹ�����GPIO_Initure
    __HAL_RCC_GPIOB_CLK_ENABLE();           //����GPIOBʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1; //PB1,0
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB1��1 
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB0��1  
}

void LedAlm(u16 time,u8* str)
{
		LED0=0;          //��������������LED0/LED1��һ��
		LED1=0;
		delay_ms(time);
		u2_printf("%s",str);
		LED0=1;
		LED1=1;
}


