#ifndef __DELAY_H
#define	__DELAY_H

#include "stm32f4xx.h"

void SysTick_Init(void);
void delay_us( __IO uint32_t us);
void delay_ms( __IO uint32_t ms);
#endif

