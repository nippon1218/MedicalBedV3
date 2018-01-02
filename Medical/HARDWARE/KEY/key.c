#include "key.h"
#include "delay.h"

/***********************************************************************
 ������      ��KEY_Init(void)  
 ��������    ��������ʼ������
 ����        ����
 ���        ����
                           
************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;          //����ṹ�����GPIO_Initure
    __HAL_RCC_GPIOA_CLK_ENABLE();           //ʹ��GPIOAʱ�� 
    __HAL_RCC_GPIOC_CLK_ENABLE();           //����GPIOCʱ��
    __HAL_RCC_GPIOE_CLK_ENABLE();           //����GPIOEʱ��
    __HAL_RCC_GPIOH_CLK_ENABLE();           //����GPIOHʱ��
    __HAL_RCC_GPIOG_CLK_ENABLE();           //����GPIOGʱ��
    __HAL_RCC_GPIOI_CLK_ENABLE();           //����GPIOIʱ��
		
    GPIO_Initure.Pin=GPIO_PIN_0;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);	

//    GPIO_Initure.Pin=GPIO_PIN_12;            //PA12
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;       //����
//    GPIO_Initure.Pull=GPIO_PULLUP;           //����
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;      //����
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);		
	
    GPIO_Initure.Pin=GPIO_PIN_5; 
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP; 
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);	

    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_4; //PI4
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);	
}

/***********************************************************************
 ������      ��KEY_Scan()  
 ��������    ������������
 ����        ��mode:0,��֧��������;1,֧��������;
 ���        �����ذ���ֵ:0��û���κΰ�������;2��WKUP���� WK_UP
                           
************************************************************************/
u8 KEY_Scan(u8 mode)
{
    static u8 key_up=1;     //�����ɿ���־
    if(mode==1)key_up=1;    //mode=1֧������
    if(key_up&&(WK_UP==1||Motor3_Tim==1||Motor3_Alm==1||Motor4_Tim==1||Motor4_Alm==1||Motor5_Tim==1||Motor5_Alm==1||Motor6_Tim==1||Motor6_Alm==1||Motor7_Tim==1))  
    {
        delay_us(5);       //��������
        key_up=0;  
		if(WK_UP==1) return WKUP_PRES;                     //���WK_UP�������£�����WKUP_PRES
		else if(Motor3_Tim==1) return Motor3_Tim_PRES;     //3�ŵ����������
		else if(Motor3_Alm==1) return Motor3_Alm_PRES;     //3�ŵ�����ϱ���
		else if(Motor4_Tim==1) return Motor4_Tim_PRES;     //4�ŵ����������
		else if(Motor4_Alm==1) return Motor4_Alm_PRES;     //4�ŵ�����ϱ���
		else if(Motor5_Tim==1) return Motor5_Tim_PRES;     //5�ŵ����������
		else if(Motor5_Alm==1) return Motor5_Alm_PRES;     //5�ŵ�����ϱ���
		else if(Motor6_Tim==1) return Motor6_Tim_PRES;     //6�ŵ����������
		else if(Motor6_Alm==1) return Motor6_Alm_PRES;     //6�ŵ�����ϱ���
		else if(Motor7_Tim==1) return Motor7_Tim_PRES;     //7�ŵ����������
    }
	else if(WK_UP==0&&Motor3_Tim==0&&Motor3_Alm==0&&Motor4_Tim==0&&Motor4_Alm==0&&Motor5_Tim==0&&Motor5_Alm==0&&Motor6_Tim==0&&Motor6_Alm==0&&Motor7_Tim==0)key_up=1; 
    return 0;   //�ް�������
}

u8 KeyCheck(u8 num)
{
	u32 i=0,j=0,n;
	n=285000;
	switch(num)
	{
		case M3TIM:
			for(i=0;i<n;i++)
			{
				if(Motor3_Tim==1)
				{	j++;	}
			}
			break;	
		case M3ALM:
			for(i=0;i<n;i++)
			{
				if(Motor3_Alm==1)
				{	j++;	}
			}
			break;	
		case M4TIM:
			for(i=0;i<n;i++)
			{
				if(Motor4_Tim==1)
				{	j++;	}
			}
			break;	
		case M4ALM:
			for(i=0;i<n;i++)
			{
				if(Motor4_Alm==1)
				{	j++;	}
			}
			break;	
			case M5TIM:
			for(i=0;i<n;i++)
			{
				if(Motor5_Tim==1)
				{	j++;	}
			}
			break;	
		case M5ALM:
			for(i=0;i<n;i++)
			{
				if(Motor5_Alm==1)
				{	j++;	}
			}
			break;
		case M6TIM:
			for(i=0;i<n;i++)
			{
				if(Motor6_Tim==1)
				{	j++;	}
			}
			break;	
		case M6ALM:
			for(i=0;i<n;i++)
			{
				if(Motor6_Alm==1)
				{	j++;	}
			}
			break;	
			case M7TIM:
			for(i=0;i<n;i++)
			{
				if(Motor7_Tim==1)
				{	j++;	}
			}
			break;	
	}
		if(j<284000)
		{	j=0;
			return 0;	
		}
		else
		{	j=0;		
			return 1;
		}
}

void KeyCheckAll(void)
{
	if(KeyCheck(M3TIM))
	{u2_printf("M3TIM\r\n");}
	if(KeyCheck(M3ALM))
	{u2_printf("M3ALM\r\n");}
	if(KeyCheck(M4TIM))
	{u2_printf("M4TIM\r\n");}
	if(KeyCheck(M4ALM))
	{u2_printf("M4ALM\r\n");}
	if(KeyCheck(M5TIM))
	{u2_printf("M5TIM\r\n");}
	if(KeyCheck(M5ALM))
	{u2_printf("M5ALM\r\n");}
	if(KeyCheck(M6TIM))
	{u2_printf("M6TIM\r\n");}
	if(KeyCheck(M6ALM))
	{u2_printf("M6ALM\r\n");}
	if(KeyCheck(M7TIM))
	{u2_printf("M7TIM\r\n");}
	
}

