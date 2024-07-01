#ifndef  __PWM4_H
#define  __PWM4_H
#include "systick_delay.h"

#define RCC_APB1Periph_TIM4_Def        RCC_APB1Periph_TIM4
#define RCC_APB2Periph_GPIO_TIM4_Def   RCC_APB2Periph_GPIOB
#define GPIO_Pin_TIM4_Def              GPIO_Pin_9
#define GPIO_TIM4_Def                  GPIOB
#define TIM4_Def                       TIM4
#define TIM4_OC_CH_Init                TIM_OC4Init
#define TIM4_OC_CH_PreloadConfig       TIM_OC4PreloadConfig

void TIM4_PWM_Init(u16 arr,u16 psc);

#endif

