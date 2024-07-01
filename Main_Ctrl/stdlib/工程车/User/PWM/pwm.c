#include "pwm.h"

static void TIM_GPIO_Config(void)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);     
	
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // �����������  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // �����������  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // �����������  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  }
static void TIM8_Mode_Config(void)
{        
      TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;        
      TIM_OCInitTypeDef  TIM_OCInitStructure;       
      u16 CCR1_Val = 0;                
      u16 CCR2_Val = 0;       
      u16 CCR3_Val = 0;        
      u16 CCR4_Val = 0; 
  
     TIM_TimeBaseStructure.TIM_Period = 999;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����  
     TIM_TimeBaseStructure.TIM_Prescaler = 71;             
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        //����ʱ�ӷ�Ƶϵ��������Ƶ           
 
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ  
     TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);                   
 
     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //����ΪPWMģʽ1   
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;               
    // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;         
     TIM_OCInitStructure.TIM_Pulse = CCR1_Val;           //��������ֵ�������������������ֵʱ����ƽ�������� 
     TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ  
     TIM_OC1Init(TIM8, &TIM_OCInitStructure);         //ʹ��ͨ��1  
     TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  
 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;          //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM   
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);          //ʹ��ͨ��2 
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);          
 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;        //����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM   
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);         //ʹ��ͨ��3   
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);    
 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;        //����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM  
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);        //ʹ��ͨ��4 
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  
 
    TIM_ARRPreloadConfig(TIM8, ENABLE);                         // ʹ��TIM8���ؼĴ���ARR    
	TIM_CtrlPWMOutputs(TIM8, ENABLE); 
	TIM_Cmd(TIM8, ENABLE);  
} 

static void TIM3_Mode_Config(void)
{     
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;        
    TIM_OCInitTypeDef  TIM_OCInitStructure;       
    u16 CCR1_Val = 0;                        
    u16 CCR4_Val = 0; 
	u16 CCR2_Val = 0;                        
    u16 CCR3_Val = 0;
	
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
	TIM_GPIO_Config();
	TIM8_Mode_Config();
	TIM3_Mode_Config();
}
