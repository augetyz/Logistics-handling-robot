#ifndef _TIMER_H
#define _TIMER_H

#include "stm32f10x.h"

#include "systick_delay.h"

#define RCC_APB1Periph_TIM2_Def        RCC_APB1Periph_TIM2
#define RCC_APB2Periph_GPIO_TIM2_Def   RCC_APB2Periph_GPIOB
#define GPIO_Pin_TIM2_Def              GPIO_Pin_3
#define GPIO_TIM2_Def                  GPIOB
#define TIM2_Def                       TIM2
#define TIM2_OC_CH_Init                TIM_OC2Init
#define TIM2_OC_CH_PreloadConfig       TIM_OC2PreloadConfig


void TIM2_PWM_Init(u16 arr,u16 psc);



#endif 


