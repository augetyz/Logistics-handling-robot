/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention



  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "JY61.h"
#include "motor_ctrl.h"
#include "servo.h"
#include "fashion_star_uart_servo.h"
#include "ring_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t usart4SendBuf[USART_SEND_BUF_SIZE + 1]; // 发送缓冲区
uint8_t usart4RecvBuf[USART_RECV_BUF_SIZE + 1]; // 接收缓冲区
RingBufferTypeDef usart4SendRingBuf;            // 发送缓冲区结构体         
RingBufferTypeDef usart4RecvRingBuf;             // 接收缓冲区结构体
Usart_DataTypeDef usart4;                      // 串口结构体
extern uint8_t date;                         // 串口接收数据
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  motor_tim_config(); // 电机PWM初始化、编码器定时器初始化

  //  servo_config(); // 舵机PWM初始化
  // 电机速度控制为0
  speed_ctrl(Motor1, 0);  
  speed_ctrl(Motor2, 0);
  speed_ctrl(Motor3, 0);
  speed_ctrl(Motor4, 0);

  printf("BSP初始化OK\n");
  // MPU6050初始化。
//  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
//  WitSerialWriteRegister(SensorUartSend);
//  WitRegisterCallBack(SensorDataUpdata);
//  WitDelayMsRegister(Delayms);
  uint8_t i = 0;
  HAL_TIM_Base_Start_IT(&htim6); // 基础定时器初始化
  // 赋值结构体usart指针
  usart4.pUSARTx = &huart4;
  // 初始化缓冲区(环形队列)
  RingBuffer_Init(&usart4SendRingBuf, USART_SEND_BUF_SIZE, usart4SendBuf);
  RingBuffer_Init(&usart4RecvRingBuf, USART_RECV_BUF_SIZE, usart4RecvBuf);
  usart4.recvBuf = &usart4RecvRingBuf;
  usart4.sendBuf = &usart4SendRingBuf;

  HAL_Delay(200);
  FSUS_SetServoAngle(&usart4, 1, 2, 2000, 0, 0);  // 舵机初始化
  HAL_Delay(200);
  FSUS_SetServoAngle(&usart4, 2, 5, 2000, 0, 0);  // 舵机初始化
  HAL_Delay(200);
  FSUS_SetServoAngle(&usart4, 3, -135, 2000, 0, 0);   // 舵机初始化
  HAL_Delay(200);
  FSUS_SetServoAngle(&usart4, 4, 55, 2000, 0, 0);  // 舵机初始化
  HAL_Delay(200);
  FSUS_SetServoAngle(&usart4, 5, 0, 2000, 0, 0);  // 舵机初始化
  HAL_Delay(200);

  HAL_UART_Receive_IT(&huart4, &date, 1); // 串口4中断接收
  while ((GPIOC->IDR & (1 << 1)) != 0)
  {
    if (i > 100) // 1s内未按下按键，进入正常模式
    {
      
      V831_die(); 
      V831_2_die(); 
      i = 0;
    }
    i++;
    HAL_Delay(10);
  }
  printf("开始了\n");
  HAL_Delay(1000);
  
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
volatile uint32_t ulHighFrequencyTimerTicks;
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  extern __IO uint32_t sysTickCnt;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6)
  {
    if (sysTickCnt > 0)
    {
      sysTickCnt--;
    }
    else
    {
      sysTickCnt = 0;
    }
  }
  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
