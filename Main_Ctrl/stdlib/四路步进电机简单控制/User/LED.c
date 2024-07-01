 #include "LED.h"
 
void LED_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
 	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	 //使能RCC时钟

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
		GPIO_Init(GPIOC, &GPIO_InitStructure);					 //初始化输出
		
		GPIO_SetBits(GPIOC,GPIO_Pin_13);// 输出低电平
}
void BJ_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
 	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	 //使能RCC时钟

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
		GPIO_Init(GPIOB, &GPIO_InitStructure);					 //初始化输出
		
		GPIO_SetBits(GPIOB,GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);// 输出低电平
}

