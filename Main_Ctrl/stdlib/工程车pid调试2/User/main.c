#include "stm32f10x.h"  
#include "pwm.h"
#include "motor.h"
#include "sys.h"
#include "delay.h"
#include "LED.h"
#include "usart.h"
#include <stdio.h>
#include "pid.h"
#include "encoder.h"
#include "TiMbase.h" 
extern int biu;
extern u8 biubiu[3];
extern float kp,kd,ki;
main()
{
	USART_Config();
	delay_init();
	printf("ƒ„ «÷Ì");
	Motor_Config();	
	Encoder_Init_TIM2();
	Encoder_Init_TIM5();
	Encoder_Init_TIM4();
	Encoder_Init_TIM1();
	speed_control(1,0);speed_control(2,0);speed_control(3,0);speed_control(4,0);
	BASIC_TIM_Init();
	while(1)
	{
		
	}
}
