#include "delay.h" 

void SysTick_Init(void)
{
/* SystemFrequency / 1000 1ms �ж�һ��
 * SystemFrequency / 100000 10us �ж�һ��
 * SystemFrequency / 1000000 1us �ж�һ��
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
// ����������ֵ��С�� 0 ��ʱ�� CRTL �Ĵ�����λ 16 ���� 1
 	while ( !((SysTick->CTRL)&(1<<16)) );
}
// �ر� SysTick ��ʱ��
 SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}

void delay_ms( __IO uint32_t ms)
{
	uint32_t i;
 	SysTick_Config(SystemCoreClock/1000);

	for (i=0; i<ms; i++) 
	{
// ����������ֵ��С�� 0 ��ʱ�� CRTL �Ĵ�����λ 16 ���� 1
// ���� 1 ʱ����ȡ��λ���� 0
 	while ( !((SysTick->CTRL)&(1<<16)) );
	 }
 // �ر� SysTick ��ʱ��
 	SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
}


