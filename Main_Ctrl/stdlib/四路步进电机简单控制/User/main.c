#include "stm32f10x.h"
#include "systick_delay.h"
#include "pwm3.h"
#include "LED.h"
/*
PA6 ¡¢PA7  PB0 ¡¢ PB1   PWMÊä³ö¶Ë¿Ú PUL+

 PB12 PB5 PB6  PB11                  EN 
PB7 PB8 PB9 PB10                     DIRn
*/

#define BJunable GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7)
#define BJenable GPIO_SetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7)

#define BJ_Change_go GPIO_ResetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11)
#define BJ_Change_back GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11)
int main(void)
{
    SysTick_Init();
    LED_Init(); 
    TIM3_Mode_Config();
    BJ_Init();
    while(1)
    {
        GPIO_ResetBits(GPIOC,GPIO_Pin_13);      
        BJunable;
        SysTick_Delay_Ms(2500);
        GPIO_SetBits(GPIOC,GPIO_Pin_13);        
        BJenable;TOGGLE;
        SysTick_Delay_Ms(1000);
    }

}

	
	
	



