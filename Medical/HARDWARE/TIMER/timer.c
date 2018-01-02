#include "timer.h"
#include "led.h"

extern vu16 UART4_RX_LEN;            //����4����״̬��־λ

TIM_HandleTypeDef TIM10_Handler;     //��ʱ��10��������ڹ��ܺ�����ʱʱ�����
TIM_HandleTypeDef TIM9_Handler;		 //��ʱ��2 ���
TIM_HandleTypeDef TIM7_Handler;      //��ʱ��7 ���������WiFi
TIM_HandleTypeDef TIM2_Handler;      //��ʱ��2 ��������ں�ɳ�ϴ�Ƹ�

/***********************************************************************
 ������      ��TIM2_Init(u16 arr,u16 psc)  
 ��������    ����ʱ����ʼ������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM2_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM2_CLK_ENABLE();			                  //ʹ�ܶ�ʱ��2
	
	TIM2_Handler.Instance=TIM2;                               //ͨ�ö�ʱ��2
    TIM2_Handler.Init.Prescaler=psc;                          //��Ƶϵ��
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //���ϼ�����
    TIM2_Handler.Init.Period=arr;                             //�Զ�װ��ֵ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   //ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM2_Handler);    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); //ʹ�ܶ�ʱ��2�Ͷ�ʱ��2�����жϣ�TIM_IT_UPDATE									 
}

/***********************************************************************
 ������      ��TIM7_Init(u16 arr,u16 psc)  
 ��������    ��������ʱ��7�жϳ�ʼ������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM7_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM7_CLK_ENABLE();			                  //ʹ�ܶ�ʱ��7
	
	TIM7_Handler.Instance=TIM7;                               //ͨ�ö�ʱ��7
    TIM7_Handler.Init.Prescaler=psc;                          //��Ƶϵ��
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //���ϼ�����
    TIM7_Handler.Init.Period=arr;                             //�Զ�װ��ֵ
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   //ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM7_Handler);    
    HAL_TIM_Base_Start_IT(&TIM7_Handler); //ʹ�ܶ�ʱ��7�Ͷ�ʱ��7�����жϣ�TIM_IT_UPDATE									 
}

/***********************************************************************
 ������      ��TIM9_Init(u16 arr,u16 psc)  
 ��������    ��������ʱ��7�жϳ�ʼ������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM9_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM9_CLK_ENABLE();			                  //ʹ�ܶ�ʱ��9
	
	TIM9_Handler.Instance=TIM9;                               //ͨ�ö�ʱ��9
    TIM9_Handler.Init.Prescaler=psc;                          //��Ƶϵ��
    TIM9_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //���ϼ�����
    TIM9_Handler.Init.Period=arr;                             //�Զ�װ��ֵ
    TIM9_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   //ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM9_Handler);    
    HAL_TIM_Base_Start_IT(&TIM9_Handler); //ʹ�ܶ�ʱ��9�Ͷ�ʱ��9�����жϣ�TIM_IT_UPDATE									 
}

/***********************************************************************
 ������      ��TIM10_Init(u16 arr,u16 psc) 
 ��������    ����ʱ��10��ʼ������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM10_Init(u16 arr,u16 psc)
{
    __HAL_RCC_TIM10_CLK_ENABLE();			                  //ʹ�ܶ�ʱ��10
	
		TIM10_Handler.Instance=TIM10;                             //ͨ�ö�ʱ��10
    TIM10_Handler.Init.Prescaler=psc;                         //��Ƶϵ��
    TIM10_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //���ϼ�����
    TIM10_Handler.Init.Period=arr;                            //�Զ���װ��ֵ
    TIM10_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;  //ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM10_Handler);   
    HAL_TIM_Base_Start_IT(&TIM10_Handler); //ʹ�ܶ�ʱ��10�Ͷ�ʱ��10�����жϣ�TIM_IT_UPDATE   
}


/***********************************************************************
 ������      ��HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)  
 ��������    ����ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
               �˺����ᱻHAL_TIM_Base_Init()��������
 ����        ����ʱ��
 ���        ����
                           
************************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM7)
	{
		__HAL_RCC_TIM7_CLK_ENABLE();            //ʹ��TIM7ʱ��
		HAL_NVIC_SetPriority(TIM7_IRQn,0,1);    //�����ж����ȼ�����ռ���ȼ�0�������ȼ�1
		HAL_NVIC_EnableIRQ(TIM7_IRQn);          //����TIM7�ж�   
	}		
}


/***********************************************************************
 ������      ��TIM7_IRQHandler(void)
 ��������    ����ʱ��7�жϷ������
 ����        ����
 ���        ����
                           
************************************************************************/
void TIM7_IRQHandler(void)
{ 	    		    
	UART4_RX_LEN|=1<<15;	                                       //��ǽ�����ɱ�־λ
	__HAL_TIM_CLEAR_FLAG(&TIM7_Handler,TIM_EventSource_Update );   //���TIM7�����жϱ�־  
	TIM7->CR1&=~(1<<0);     			                           //�رն�ʱ��7     											 
} 

/***********************************************************************
 ������      ��TIM2_Stop(void)
 ��������    ���رն�ʱ��2
 ����        ����
 ���        ����
                           
************************************************************************/
void TIM2_Stop(void)
{
	  HAL_TIM_Base_Stop_IT(&TIM2_Handler);  //�رն�ʱ��10�Ͷ�ʱ��10�����ж�
}

/***********************************************************************
 ������      ��TIM2_Stop(void)
 ��������    ���رն�ʱ��2
 ����        ����
 ���        ����
                           
************************************************************************/
void TIM9_Stop(void)
{
	  HAL_TIM_Base_Stop_IT(&TIM9_Handler);  //�رն�ʱ��10�Ͷ�ʱ��10�����ж�
}


/***********************************************************************
 ������      ��TIM10_Stop(void)
 ��������    ���رն�ʱ��10
 ����        ����
 ���        ����
                           
************************************************************************/
void TIM10_Stop(void)
{
	  HAL_TIM_Base_Stop_IT(&TIM10_Handler);  //�رն�ʱ��10�Ͷ�ʱ��10�����ж�
}

