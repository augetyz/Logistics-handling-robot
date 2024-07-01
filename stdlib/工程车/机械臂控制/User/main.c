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
#include "arm.h"
extern int biu;
extern u8 biubiu[3];
extern float kp,kd,ki;
main()
{
	USART3_Config();
	USART2_Config();
	delay_init();
	printf("ƒ„ «÷Ì");
	//Motor_Config();	
	//BASIC_TIM_Init();
//	control_arm(1,1800,2000);delay_ms(100);
	control_arm(2,900,500);delay_ms(100);
	control_arm(3,0,2000);delay_ms(100);
	control_arm(4,500,2000);delay_ms(100);
	control_arm(0
	 ,1900,2000);delay_ms(100);
	while(1)
	{
		
	}
}
