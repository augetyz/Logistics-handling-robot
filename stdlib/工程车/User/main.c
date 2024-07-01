#include "stm32f10x.h"  
#include "pwm.h"
#include "motor.h"
#include "sys.h"
#include "delay.h"
#include "LED.h"
#include "usart.h"
#include <stdio.h>
#include "encoder.h"
extern int biu;
extern u8 biubiu[3];
main()
{
	u16 speed;	
	USART_Config();
	delay_init();
	printf("ƒ„ «÷Ì");
	Motor_Config();	
	Encoder_Init_TIM2();
	Encoder_Init_TIM5();
	Encoder_Init_TIM4();
	Encoder_Init_TIM1();
	LED_Config();
	
	while(1)
	{
		speed=getTIMx_DetaCnt(TIM1);printf("TIM1 CNT:%5d ",speed);speed=0;
		speed=getTIMx_DetaCnt(TIM2);printf("TIM2 CNT:%5d ",speed);speed=0;
		speed=getTIMx_DetaCnt(TIM4);printf("TIM4 CNT:%5d ",speed);speed=0;
		speed=getTIMx_DetaCnt(TIM5);printf("TIM5 CNT:%5d\n",speed);speed=0;
		delay_ms(1000);
	}
}
