#include "stm32f4xx.h"
#include "led.h"  
#include "key.h" 
#include "GPIO.h"
#include "delay.h"
#include "usart.h"
#include <stdio.h>
#include "JY61.h"
//此代码配套板子的晶振为8Mhz，注意，野火的板子的晶振为25Mhz

extern char s_cDataUpdate, s_cCmd;
float fAcc[3], fGyro[3], fAngle[3];
void MPU6050_date_get(void);


int main(void)
{  
/************************ SYSTEM_START ***************************/    
     
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//  嵌套中断向量分组选择器——选择2组   2:2   共4个抢占优先级  4个子优先级 
    
/************************* BSP_START *****************************/     
	USART1_Config();
    USART2_Config();
    USART3_Config();
	Key_GPIO_Config();
	LED_GPIO_Config();
/**************************  START  *****************************/      
    Usart_SendByte(USART1,0X41);
    Usart_SendByte(USART2,0X42);
    Usart_SendByte(USART3,0X43);
    GPIOC->ODR^=GPIO_Pin_13;
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);

	while (1)
	{

		MPU6050_date_get();
        GPIOC->ODR|=GPIO_Pin_13;
        delay_ms(100);
        Usart_SendString( USART3, "123456789\n");
        GPIOC->ODR&=~GPIO_Pin_13;
        delay_ms(100);
	}

}
void MPU6050_date_get(void)
{
    int i;
    if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			printf("%.2f,%.2f,%.2f\n",fAngle[0],fAngle[1],fAngle[2]);		            
		}
}


