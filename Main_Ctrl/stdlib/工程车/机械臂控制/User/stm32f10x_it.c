/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "TiMbase.h"
#include "usart.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "pwm.h"
#define kp kp3
#define ki ki3
#define kd kd3
#define goal goal3
extern int goal_speed;
extern int actual_speed;
extern u8 get_date[24];
extern int dif_speed;//差值
extern float   kp1,ki1,kd1;
extern float   kp2,ki2,kd2;
extern float   kp3,ki3,kd3;
extern float   kp4,ki4,kd4;
int goal1=300, 
	goal2=300, 
	goal3=300, 
	goal4=300;
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}


void  BASIC_TIM_IRQHandler (void)
{
	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) != RESET ) 
	{			
		
		TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update); 
	}		 	
}


void USART3_IRQHandler(void)                	//串口1中断服务程序
	{
		char a;
	if(USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET)  
	{
		a=USART_ReceiveData(DEBUG_USARTx);
		switch (a)
		{
			case '1':goal1=300, 
	goal2=300, 
	goal3=300, 
	goal4=300;break;
			case '2':goal1=-300, 
	goal2=-300, 
	goal3=-300, 
	goal4=-300;break;
			case '3':goal1=400, 
	goal2=300, 
	goal3=-400, 
	goal4=-300;break;
			case '4':goal1=-300, 
	goal2=-400, 
	goal3=300, 
	goal4=400;break;
			case '5':kp=kp+1;break;
			case '6':kp=kp-1;break;
			case '7':kp=kp-0.1;break;
			case '8':ki=ki+0.1;break;
			case '9':ki=ki+1;break;
			case '0':ki=ki-1;break;
			case 'a':ki=ki-0.1;break;
			case 'b':kd=kd+0.1;break;
			case 'c':kd=kd+1;break;
			case 'd':kd=kd-1;break;
			case 'e':kd=kd-0.1;break;
			case 'f':printf("%d,%f,%f,%f\n",goal,kp,ki,kd);break;
		}
	}
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
