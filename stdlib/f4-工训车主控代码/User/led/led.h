#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

//引脚定义
/*******************************************************/
//R 红色灯
#define LED1_PIN                  GPIO_Pin_6                 
#define LED1_GPIO_PORT            GPIOF                      
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOF

//G 绿色灯
#define LED2_PIN                  GPIO_Pin_7                 
#define LED2_GPIO_PORT            GPIOF                      
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOF

//B 蓝色灯
#define LED3_PIN                  GPIO_Pin_8                 
#define LED3_GPIO_PORT            GPIOF                       
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOF
/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  0
#define OFF 1

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_PIN)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED3_GPIO_PORT,LED3_PIN);\
					else		\
					GPIO_ResetBits(LED3_GPIO_PORT,LED3_PIN)


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 {p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态

/* 定义控制IO的宏 */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF			digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON				digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE		digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF			digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON				digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE		digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF			digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON				digitalLo(LED3_GPIO_PORT,LED3_PIN)

/* 基本混色，后面高级用法使用PWM可混出全彩颜色,且效果更好 */

//红
#define LED_RED  \
					LED1_ON;\
					LED2_OFF;\
					LED3_OFF

//绿
#define LED_GREEN		\
					LED1_OFF;\
					LED2_ON;\
					LED3_OFF

//蓝
#define LED_BLUE	\
					LED1_OFF;\
					LED2_OFF;\
					LED3_ON

					
//黄(红+绿)					
#define LED_YELLOW	\
					LED1_ON;\
					LED2_ON;\
					LED3_OFF
//紫(红+蓝)
#define LED_PURPLE	\
					LED1_ON;\
					LED2_OFF;\
					LED3_ON

//青(绿+蓝)
#define LED_CYAN \
					LED1_OFF;\
					LED2_ON;\
					LED3_ON
					
//白(红+绿+蓝)
#define LED_WHITE	\
					LED1_ON;\
					LED2_ON;\
					LED3_ON
					
//黑(全部关闭)
#define LED_RGBOFF	\
					LED1_OFF;\
					LED2_OFF;\
					LED3_OFF		

#define COLOR_TIM_GPIO_CLK             (RCC_AHB1Periph_GPIOF)

/************红灯***************/
#define COLOR_RED_TIM           						TIM10
#define COLOR_RED_TIM_CLK       						RCC_APB2Periph_TIM10
#define COLOR_RED_TIM_APBxClock_FUN        RCC_APB2PeriphClockCmd
/*计算说明见c文件*/
/*部分通用定时器的时钟为HCLK/4，部分为HCLK/2，注意要把三个通道的定时器频率配置为一致*/
#define COLOR_RED_TIM_PRESCALER						(((SystemCoreClock)/1000000)*30-1)

/************绿灯***************/
#define COLOR_GREEN_TIM           						TIM11
#define COLOR_GREEN_TIM_CLK       						RCC_APB2Periph_TIM11
#define COLOR_GREEN_TIM_APBxClock_FUN        RCC_APB2PeriphClockCmd
/*部分通用定时器的时钟为HCLK/4，部分为HCLK/2，注意要把三个通道的定时器频率配置为一致*/
#define COLOR_GREEN_TIM_PRESCALER						(((SystemCoreClock)/1000000)*30-1)

/************蓝灯***************/
#define COLOR_BLUE_TIM           							TIM13
#define COLOR_BLUE_TIM_CLK       						RCC_APB1Periph_TIM13
#define COLOR_BLUE_TIM_APBxClock_FUN        RCC_APB1PeriphClockCmd
/*部分通用定时器的时钟为HCLK/4，部分为HCLK/2，注意要把三个通道的定时器频率配置为一致*/
#define COLOR_BLUE_TIM_PRESCALER						(((SystemCoreClock/2)/1000000)*30-1)



/************红灯***************/

#define COLOR_RED_PIN                  GPIO_Pin_6                 
#define COLOR_RED_GPIO_PORT            GPIOF                      
#define COLOR_RED_PINSOURCE				GPIO_PinSource6
#define COLOR_RED_AF					GPIO_AF_TIM10

#define  COLOR_RED_TIM_OCxInit                TIM_OC1Init            //通道初始化函数
#define  COLOR_RED_TIM_OCxPreloadConfig       TIM_OC1PreloadConfig //通道重载配置函数

//通道比较寄存器，以TIMx->CCRx方式可访问该寄存器，设置新的比较值，控制占空比
//以宏封装后，使用这种形式：COLOR_TIMx->COLOR_RED_CCRx ，可访问该通道的比较寄存器
#define  COLOR_RED_CCRx                       	CCR1		

/************绿灯***************/
#define COLOR_GREEN_PIN                  GPIO_Pin_7                 
#define COLOR_GREEN_GPIO_PORT            GPIOF                      
#define COLOR_GREEN_PINSOURCE						GPIO_PinSource7
#define COLOR_GREEN_AF										GPIO_AF_TIM11

#define  COLOR_GREEN_TIM_OCxInit                TIM_OC1Init            //通道初始化函数
#define  COLOR_GREEN_TIM_OCxPreloadConfig       TIM_OC1PreloadConfig //通道重载配置函数

//通道比较寄存器，以TIMx->CCRx方式可访问该寄存器，设置新的比较值，控制占空比
//以宏封装后，使用这种形式：COLOR_TIMx->COLOR_GREEN_CCRx ，可访问该通道的比较寄存器
#define  COLOR_GREEN_CCRx                       CCR1

/************蓝灯***************/
#define COLOR_BLUE_PIN                  GPIO_Pin_8                 
#define COLOR_BLUE_GPIO_PORT            GPIOF                       
#define COLOR_BLUE_PINSOURCE						GPIO_PinSource8
#define COLOR_BLUE_AF										GPIO_AF_TIM13

#define   COLOR_BLUE_TIM_OCxInit              TIM_OC1Init            //通道初始化函数
#define   COLOR_BLUE_TIM_OCxPreloadConfig    TIM_OC1PreloadConfig  //通道重载配置函数

//通道比较寄存器，以TIMx->CCRx方式可访问该寄存器，设置新的比较值，控制占空比
//以宏封装后，使用这种形式：COLOR_TIMx->COLOR_BLUE_CCRx ，可访问该通道的比较寄存器
#define   COLOR_BLUE_CCRx                      CCR1


/************************************************************/

/* RGB颜色值 24位*/
#define COLOR_WHITE          0xFFFFFF
#define COLOR_BLACK          0x000000
#define COLOR_GREY           0xC0C0C0
#define COLOR_BLUE           0x0000FF
#define COLOR_RED            0xFF0000
#define COLOR_MAGENTA        0xFF00FF
#define COLOR_GREEN          0x00FF00
#define COLOR_CYAN           0x00FFFF
#define COLOR_YELLOW         0xFFFF00


void LED_GPIO_Config(void);
void ColorLED_Config(void);
void SetRGBColor(uint32_t rgb);
void SetColorValue(uint8_t r,uint8_t g,uint8_t b);
#endif /* __LED_H */
