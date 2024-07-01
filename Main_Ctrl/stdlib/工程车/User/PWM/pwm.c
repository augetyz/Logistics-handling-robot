#include "pwm.h"

static void TIM_GPIO_Config(void)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);     
	
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // 复用推挽输出  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // 复用推挽输出  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    // 复用推挽输出  
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
  
     TIM_TimeBaseStructure.TIM_Period = 999;       //当定时器从0计数到999，即为1000次，为一个定时周期  
     TIM_TimeBaseStructure.TIM_Prescaler = 71;             
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        //设置时钟分频系数：不分频           
 
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式  
     TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);                   
 
     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //配置为PWM模式1   
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;               
    // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;         
     TIM_OCInitStructure.TIM_Pulse = CCR1_Val;           //设置跳变值，当计数器计数到这个值时，电平发生跳变 
     TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平  
     TIM_OC1Init(TIM8, &TIM_OCInitStructure);         //使能通道1  
     TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  
 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;          //设置通道2的电平跳变值，输出另外一个占空比的PWM   
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);          //使能通道2 
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);          
 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;        //设置通道3的电平跳变值，输出另外一个占空比的PWM   
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);         //使能通道3   
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);    
 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;        //设置通道4的电平跳变值，输出另外一个占空比的PWM  
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);        //使能通道4 
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  
 
    TIM_ARRPreloadConfig(TIM8, ENABLE);                         // 使能TIM8重载寄存器ARR    
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
	
    TIM_TimeBaseStructure.TIM_Period = 999;       //当定时器从0计数到999，即为1000次，为一个定时周期  
     TIM_TimeBaseStructure.TIM_Prescaler = 71;             
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        //设置时钟分频系数：不分频           
 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式  
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                   
 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //配置为PWM模式1 
	
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                       
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;           //设置跳变值，当计数器计数到这个值时，电平发生跳变 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平  
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);         //使能通道1  
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);    
 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                       
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;           //设置跳变值，当计数器计数到这个值时，电平发生跳变 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平  
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);         //使能通道2  
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                       
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;           //设置跳变值，当计数器计数到这个值时，电平发生跳变 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平  
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);         //使能通道3
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;        //设置通道4的电平跳变值，输出另外一个占空比的PWM 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平	 
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);        //使能通道4 
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  
 
    TIM_ARRPreloadConfig(TIM3, ENABLE);                         // 使能TIM1重载寄存器ARR    
	TIM_CtrlPWMOutputs(TIM3, ENABLE); 
	TIM_Cmd(TIM3, ENABLE);  
}

void Motor_Config(void)
{
	TIM_GPIO_Config();
	TIM8_Mode_Config();
	TIM3_Mode_Config();
}
