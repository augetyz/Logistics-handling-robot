#include "pwm.h"


static void TIM8_Mode_Config(void)
{        
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	
	RCC->APB2ENR|=1<<13;       //ʹ��TIM8ʱ��    
	RCC->APB2ENR|=1<<4;        //PORTCʱ��ʹ��     
	GPIOC->CRH&=0XFFFFFF00;    //PORTC8�������
	GPIOC->CRH|=0X000000BB;    //PORTC8�������
	
	GPIOC->CRL&=0X00FFFFFF;    //PORTC6 7�������
	GPIOC->CRL|=0XBB000000;    //PORTC6 7�������
	TIM8->ARR=999;             //�趨�������Զ���װֵ 
	TIM8->PSC=71;             //Ԥ��Ƶ������Ƶ
	TIM8->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM8->CCMR1|=6<<12;        //CH2 PWM1ģʽ	
	TIM8->CCMR2|=6<<4;         //CH3 PWM1ģʽ	
	TIM8->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	
	TIM8->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	  
	TIM8->CCMR1|=1<<11;        //CH2Ԥװ��ʹ��	 
	TIM8->CCMR2|=1<<3;         //CH3Ԥװ��ʹ��	  
	TIM8->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	  
//	
	TIM8->CCER|=1<<0;         //CH1���ʹ��	
	TIM8->CCER|=1<<4;         //CH2���ʹ��	   
	TIM8->CCER|=1<<8;         //CH3���ʹ��	 
	TIM8->CCER|=1<<12;        //CH4���ʹ��
	TIM8->CCR1=0;
	TIM8->CCR2=0;
	TIM8->CCR3=0;
	TIM8->CCR4=0;
	TIM8->BDTR |= 1<<15;       //TIM8����Ҫ��仰�������PWM
	TIM8->CR1=0x8000;          //ARPEʹ�� 
	TIM8->CR1|=0x01;          //ʹ�ܶ�ʱ��1  
} 

static void TIM3_Mode_Config(void)
{     
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;	
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	u16 CCR1_Val = 0;                        
    u16 CCR4_Val = 0; 
	u16 CCR2_Val = 0;                        
    u16 CCR3_Val = 0;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);     
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // �����������  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // �����������  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
    TIM_TimeBaseStructure.TIM_Period = 999;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����  
     TIM_TimeBaseStructure.TIM_Prescaler = 71;             
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        //����ʱ�ӷ�Ƶϵ��������Ƶ           
 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ  
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                   
 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //����ΪPWMģʽ1 
	
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                       
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;           //��������ֵ�������������������ֵʱ����ƽ�������� 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ  
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);         //ʹ��ͨ��1  
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);    
 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                       
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;           //��������ֵ�������������������ֵʱ����ƽ�������� 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ  
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);         //ʹ��ͨ��2  
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                       
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;           //��������ֵ�������������������ֵʱ����ƽ�������� 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ  
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);         //ʹ��ͨ��3
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;        //����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ	 
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);        //ʹ��ͨ��4 
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  
 
    TIM_ARRPreloadConfig(TIM3, ENABLE);                         // ʹ��TIM1���ؼĴ���ARR    
	TIM_CtrlPWMOutputs(TIM3, ENABLE); 
	TIM_Cmd(TIM3, ENABLE);  
}

void Motor_Config(void)
{	
	TIM8_Mode_Config();
	TIM3_Mode_Config();
}
