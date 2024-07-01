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

extern u8 get_date[24];
extern int dif_speed;//差值
extern float   kp1,ki1,kd1;
extern float   kp2,ki2,kd2;
extern float   kp3,ki3,kd3;
extern float   kp4,ki4,kd4;
int speed1,speed2,speed3,speed4;
int goal1=30, 
	goal2=30, 
	goal3=30, 
	goal4=30;
int i;
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
		
//		KalmanFilter(Get_Motor_Speed(1));
//		KalmanFilter(Get_Motor_Speed(2));
//		KalmanFilter(Get_Motor_Speed(3));
//		KalmanFilter(Get_Motor_Speed(4));
		
		speed1+=Get_Motor_Speed(1);speed2+=Get_Motor_Speed(2);speed3+=Get_Motor_Speed(3);speed4+=Get_Motor_Speed(4);
		if(i==9)
		{
			motor_control(goal1,goal2,goal3,goal4);
			i=0;
			speed1=0,speed2=0,speed3=0,speed4=0;
			pidsend();
		}
		else
		{
			
			i++;	
		}
		
//		if(i>120)
//		{
//			i=0;
//			goal1=-goal1, 
//			goal2=-goal2, 
//			goal3=-goal3, 
//			goal4=-goal4; 	
//		}
//		else
//			i++;
		TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update); 
	}		 	
}


void DEBUG_USART_IRQHandler(void)                	//串口1中断服务程序
	{
		static u8 a[20];static short num;
	if(USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET)  
	{
		a[num]=USART_ReceiveData(DEBUG_USARTx);
		if(a[num]!=0x0a)
			num++;
		else
		{
			date_deal(a);
			for(;num>0;num--)
				a[num]=0;
		}
		if(num>20)
			num=0;
			
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
