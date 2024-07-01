#ifndef _LED_H
#define _LED_H

#define digitalToggle(p,i) {p->ODR ^=i;} 
#define TOGGLE		 digitalToggle(GPIOB,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11)

#include "stm32f10x.h"
void LED_Init(void);
void BJ_Init(void);
#endif 
