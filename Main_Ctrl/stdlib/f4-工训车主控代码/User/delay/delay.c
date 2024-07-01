#include "delay.h" 

void SysTick_Init(void)
{
/* SystemFrequency / 1000 1ms 中断一次
 * SystemFrequency / 100000 10us 中断一次
 * SystemFrequency / 1000000 1us 中断一次
 */
 	if (SysTick_Config(SystemCoreClock / 100000)) 
 	{
/* Capture error */
	 while (1);
	}
 }
void delay_us( __IO uint32_t us)
{
 	uint32_t i;
 	SysTick_Config(SystemCoreClock/1000000);

	for (i=0; i<us; i++) {
// 当计数器的值减小到 0 的时候， CRTL 寄存器的位 16 会置 1
 	while ( !((SysTick->CTRL)&(1<<16)) );
}
// 关闭 SysTick 定时器
 SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}

void delay_ms( __IO uint32_t ms)
{
	uint32_t i;
 	SysTick_Config(SystemCoreClock/1000);

	for (i=0; i<ms; i++) 
	{
// 当计数器的值减小到 0 的时候， CRTL 寄存器的位 16 会置 1
// 当置 1 时，读取该位会清 0
 	while ( !((SysTick->CTRL)&(1<<16)) );
	 }
 // 关闭 SysTick 定时器
 	SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
}


