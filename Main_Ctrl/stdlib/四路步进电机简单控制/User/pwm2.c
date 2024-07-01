 #include"pwm2.h"
 


void TIM2_PWM_Init(u16 arr,u16 psc){
    GPIO_InitTypeDef     GPIO_InitStrue;
    TIM_OCInitTypeDef     TIM_OCInitStrue;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStrue;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2_Def,ENABLE);  // TIM
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_TIM2_Def,ENABLE);  //GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    
    /* reflect */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);  // reflect PB3 TIM2_CH2
    
    GPIO_InitStrue.GPIO_Pin=GPIO_Pin_TIM2_Def;  //Pin
    GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStrue.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIO_TIM2_Def,&GPIO_InitStrue);  // GPIO
    
    TIM_TimeBaseInitStrue.TIM_Period=arr;
    TIM_TimeBaseInitStrue.TIM_Prescaler=psc;
    TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM2_Def,&TIM_TimeBaseInitStrue); //TIM
    
    TIM_OCInitStrue.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OCInitStrue.TIM_OCPolarity=TIM_OCPolarity_High;
    TIM_OCInitStrue.TIM_OutputState=TIM_OutputState_Enable;
    
    TIM2_OC_CH_Init(TIM2_Def,&TIM_OCInitStrue);  //TIM_CH
    
    TIM2_OC_CH_PreloadConfig(TIM2_Def,TIM_OCPreload_Enable); //TIM_CH
    
    TIM_Cmd(TIM2_Def,ENABLE);  //TIM
}

