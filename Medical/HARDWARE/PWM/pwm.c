#include "pwm.h"
#include "led.h"
#include "delay.h"
 	 
TIM_HandleTypeDef TIM_Handler;          //��ʱ��PWM��� 
TIM_OC_InitTypeDef TIM_CHXHandler;	    //��ʱ��ͨ�����


/***********************************************************************
 ������      ��TIM8_PWM_CHANNEL_1_START(u16 arr,u16 psc)  
 ��������    ��TIM8ͨ�� 1 PWM�������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM8_PWM_CHANNEL_1_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                              //��ʱ��8
    TIM_Handler.Init.Prescaler=psc;                         //��ʱ����Ƶ
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //���ϼ���ģʽ
    TIM_Handler.Init.Period=arr;                            //�Զ���װ��ֵ
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                         //��ʼ��PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                  //ģʽѡ��PWM1
    TIM_CHXHandler.Pulse=arr/2;                             //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;           //����Ƚϼ���Ϊ�� 
	HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_1);//����TIM8ͨ��1
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_1);          //����PWMͨ��1
}

/***********************************************************************
 ������      ��TIM8_PWM_CHANNEL_2_START(u16 arr,u16 psc)  
 ��������    ��TIM8ͨ�� 2 PWM�������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM8_PWM_CHANNEL_2_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                               //��ʱ��8
    TIM_Handler.Init.Prescaler=psc;                          //��ʱ����Ƶ
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //���ϼ���ģʽ
    TIM_Handler.Init.Period=arr;                             //�Զ���װ��ֵ
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                          //��ʼ��PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                   //ģʽѡ��PWM1 
    TIM_CHXHandler.Pulse=arr/2;                              //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;            //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_2);//����TIM8ͨ��2
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_2);           //����PWMͨ��2
}

/***********************************************************************
 ������      ��TIM8_PWM_CHANNEL_3_START(u16 arr,u16 psc)  
 ��������    ��TIM8ͨ�� 3 PWM�������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM8_PWM_CHANNEL_3_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                              //��ʱ��8
    TIM_Handler.Init.Prescaler=psc;                         //��ʱ����Ƶ
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //���ϼ���ģʽ
    TIM_Handler.Init.Period=arr;                            //�Զ���װ��ֵ
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                         //��ʼ��PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                  //ģʽѡ��PWM1
    TIM_CHXHandler.Pulse=arr/2;                             //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;           //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_3);//����TIM8ͨ��3
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_3);          //����PWMͨ��3
}

/***********************************************************************
 ������      ��TIM8_PWM_CHANNEL_4_START(u16 arr,u16 psc)  
 ��������    ��TIM8ͨ�� 4 PWM�������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM8_PWM_CHANNEL_4_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM8;                               //��ʱ��8
    TIM_Handler.Init.Prescaler=psc;                          //��ʱ����Ƶ
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //���ϼ���ģʽ
    TIM_Handler.Init.Period=arr;                             //�Զ���װ��ֵ
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                          //��ʼ��PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                   //ģʽѡ��PWM1
    TIM_CHXHandler.Pulse=arr/2;                              //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;            //����Ƚϼ���Ϊ�� 
	  HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_4);  //����TIM8ͨ��4
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_4);           //����PWMͨ��4
}

/***********************************************************************
 ������      ��TIM5_PWM_CHANNEL_4_START(u16 arr,u16 psc)  
 ��������    ��TIM5ͨ�� 4 PWM�������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM5_PWM_CHANNEL_4_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM5;                               //��ʱ��5
    TIM_Handler.Init.Prescaler=psc;                          //��ʱ����Ƶ
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;         //���ϼ���ģʽ
    TIM_Handler.Init.Period=arr;                             //�Զ���װ��ֵ
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                          //��ʼ��PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                   //ģʽѡ��PWM1
    TIM_CHXHandler.Pulse=arr/2;                              //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;            //����Ƚϼ���Ϊ�� 
	  HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_4);  //����TIM5ͨ��4
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_4);           //����PWMͨ��4
}

/***********************************************************************
 ������      ��TIM3_PWM_CHANNEL_2_START(u16 arr,u16 psc)  
 ��������    ��TIM3ͨ�� 2 PWM�������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM3_PWM_CHANNEL_2_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM3;                              //��ʱ��3
    TIM_Handler.Init.Prescaler=psc;                         //��ʱ����Ƶ
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //���ϼ���ģʽ
    TIM_Handler.Init.Period=arr;                            //�Զ���װ��ֵ
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                         //��ʼ��PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                  //ģʽѡ��PWM1
    TIM_CHXHandler.Pulse=arr/2;                             //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;           //����Ƚϼ���Ϊ�� 
	HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_2);   //����TIM3ͨ��2
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_2);          //����PWMͨ��2
}

/***********************************************************************
 ������      ��TIM1_PWM_CHANNEL_4_START(u16 arr,u16 psc)  
 ��������    ��TIM1ͨ�� 4 PWM�������
 ����        ��arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
 ���        ����
                           
************************************************************************/
void TIM1_PWM_CHANNEL_4_START(u16 arr,u16 psc)
{ 
    TIM_Handler.Instance=TIM1;                                //��ʱ��1
    TIM_Handler.Init.Prescaler=psc;                           //��ʱ����Ƶ
    TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;          //���ϼ���ģʽ
    TIM_Handler.Init.Period=arr;                              //�Զ���װ��ֵ
    TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM_Handler);                            //��ʼ��PWM
    
    TIM_CHXHandler.OCMode=TIM_OCMODE_PWM1;                     //ģʽѡ��PWM1
    TIM_CHXHandler.Pulse=arr/2;                                //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM_CHXHandler.OCPolarity=TIM_OCPOLARITY_LOW;              //����Ƚϼ���Ϊ�� 
	  HAL_TIM_PWM_ConfigChannel(&TIM_Handler,&TIM_CHXHandler,TIM_CHANNEL_4);//����TIM1ͨ��4
    HAL_TIM_PWM_Start(&TIM_Handler,TIM_CHANNEL_4);             //����PWMͨ��4
}

/***********************************************************************
 ������      ��HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) 
 ��������    ����ʱ���ײ�������ʱ��ʹ�ܣ���������
			   �˺����ᱻHAL_TIM_PWM_Init()����
 ����        ��htim:��ʱ�����
 ���        ����
                           
************************************************************************/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM1_CLK_ENABLE();			//ʹ�ܶ�ʱ��1
	__HAL_RCC_TIM3_CLK_ENABLE();			//ʹ�ܶ�ʱ��3
	__HAL_RCC_TIM5_CLK_ENABLE();			//ʹ�ܶ�ʱ��5
	__HAL_RCC_TIM8_CLK_ENABLE();			//ʹ�ܶ�ʱ��8
    __HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
	__HAL_RCC_GPIOI_CLK_ENABLE();           //����GPIOIʱ��
	  
	GPIO_Initure.Pin=GPIO_PIN_11;           //PA11
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	GPIO_Initure.Alternate= GPIO_AF1_TIM1;	//PC����ΪTIM1_CH4	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		
    GPIO_Initure.Pin=GPIO_PIN_7;            //PA7
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	GPIO_Initure.Alternate= GPIO_AF2_TIM3;  //PC����ΪTIM3_CH2	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		
	GPIO_Initure.Pin=GPIO_PIN_0;            //PI0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	GPIO_Initure.Alternate= GPIO_AF2_TIM5;  //PC����ΪTIM5_CH4	
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7; //PI2/5/6/7
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;     //�����������
	GPIO_Initure.Pull=GPIO_PULLUP;         //����
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;    //����
	GPIO_Initure.Alternate=GPIO_AF3_TIM8;  //PC����ΪTIM8
	HAL_GPIO_Init(GPIOI,&GPIO_Initure);	
}





