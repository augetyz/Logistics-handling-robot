#include "encoder.h"

/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //开启AFIO时钟
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//使能JTAGDisable，即禁用JTAG接口
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);   //使能PA端口时钟
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//根据设定参数初始化GPIOA

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              //预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM2,0);
	//===============================================
	TIM2->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM2, ENABLE); 
}
/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //使能定时器4的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);   //使能PB端口时钟
	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//根据设定参数初始化GPIOB
	
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM4,0);
	//===============================================
	TIM4->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM4, ENABLE); 
}
void Encoder_Init_TIM5(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);    //使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);   //使能PA端口时钟

	//GPIO_PinRemapConfig(GPIO_Remap_TIM5CH4_LSI,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//根据设定参数初始化GPIOA

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              //预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM5,0);
	//===============================================
	TIM5->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM5, ENABLE); 
}
void Encoder_Init_TIM1(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);    //使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1, ENABLE);   //使能PA端口时钟
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource8|GPIO_PinSource9,GPIO_AF_TIM5);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//根据设定参数初始化GPIOA

	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              //预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM1,0);
	//===============================================
	TIM1->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM1, ENABLE); 
}
/**************************************************************************
函数功能：读取编码器脉冲差值
入口参数：TIM_TypeDef * TIMx
返回  值：无
**************************************************************************/
s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx)
{
	s16 cnt;
	cnt = TIMx->CNT-0x7fff;
	TIMx->CNT = 0x7fff;
	return cnt;
}


int Get_Motor_Speed(u8 lunzi)
{
	int speed;
		switch(lunzi)
	{
		case 1:
			speed=getTIMx_DetaCnt(TIM1);
			break;
		case 2:
			speed=-getTIMx_DetaCnt(TIM2);
			break;
		case 3:
			speed=getTIMx_DetaCnt(TIM4);
			break;
		case 4:	
			speed=-getTIMx_DetaCnt(TIM5);
			break;
	}
	return speed;
}


