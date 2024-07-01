#include "soft_user.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

__IO uint32_t sysTickCnt;

void delay(uint32_t time)
{
    uint32_t i, m;
    for (i = 0; i < time; i++)
    {
        for (m = 0; m < 21000; m++)
            ;
    }
}
void delay_us(uint32_t time)
{
    uint32_t i, m;
    for (i = 0; i < time; i++)
    {
        for (m = 0; m < 21; m++)
            ;
    }
}
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return (ch);
}
void Usart_SendByte(UART_HandleTypeDef *pUSARTx, uint8_t ch)
{
    /* 发送一个字节到USART */
    HAL_UART_Transmit(pUSARTx, (uint8_t *)&ch, 1, 100);
}
// 将串口发送缓冲区的内容全部发出去
void Usart_SendAll(Usart_DataTypeDef *usart)
{
    uint8_t value;
    while (RingBuffer_GetByteUsed(usart->sendBuf))
    {
        value = RingBuffer_Pop(usart->sendBuf);
        // printf("Usart_SendAll pop: %d", value);
        Usart_SendByte(usart->pUSARTx, value);
    }
}
void Usart_SendString(uint8_t *str)
{
    unsigned int k = 0;
    do
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)(str + k), 1, 1000);
        k++;
    } while (*(str + k) != '\0');
}
void Usart6_SendString(uint8_t *str)
{
    unsigned int k = 0;
    do
    {
        HAL_UART_Transmit(&huart6, (uint8_t *)(str + k), 1, 100);
        k++;
    } while (*(str + k) != '\0');
}

void Uart5_LCD_send_task(uint8_t task)
{
    char date[60];
    switch (task)
    {
    case 0:
        sprintf(date, "main.task.txt=\"开始咯\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    case 1:
        sprintf(date, "main.task.txt=\"扫描二维码\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    case 2:
        sprintf(date, "main.task.txt=\"拿取物料\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    case 3:
        sprintf(date, "main.task.txt=\"走咯biubiu\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    case 4:
        sprintf(date, "main.task.txt=\"放置物料\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    case 5:
        sprintf(date, "main.task.txt=\"再拿起来（真费事）\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    case 6:
        sprintf(date, "main.task.txt=\"码垛呜呜\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    case 7:
        sprintf(date, "main.task.txt=\"回家咯\"\xff\xff\xff");
        HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
        break;
    default:
        break;
    }
}
void Uart5_LCD_show_string(char *string)
{
    char date[60];
    char str_1[] = "main.task.txt=\"";
    char str_2[] = "\"\xff\xff\xff";
    strcpy(date, str_1);
    strcat(date, string);
    strcat(date, str_2);
    HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
}
void Uart5_LCD_show_QR(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f)
{
    char date[40];
    sprintf(date, "main.num.txt=\"%d%d%d+%d%d%d\"\xff\xff\xff", a, b, c, d, e, f);
    HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
}
void Uart5_LCD_show_Angle(float angle)
{
    int i = 0;
    char date[40];
    i = (int)(angle * 100);
    sprintf(date, "main.angle.val=%d\xff\xff\xff", i);
    HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
}
void Uart5_LCD_show_X_Y(uint16_t x, uint16_t y)
{

    char date[40];
    sprintf(date, "main.x.val=%d\xff\xff\xff", x);
    HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);

    sprintf(date, "main.y.val=%d\xff\xff\xff", y);
    HAL_UART_Transmit(&huart5, (uint8_t *)date, strlen(date), 100);
}
void clearArray(uint8_t *array, uint16_t length)
{
    int i = 0;
    for (i = 0; i < length; i++)
    {
        array[i] = 0;
    }
}

// 等待计时完成
void SysTick_Wait()
{
    // 定时器使能
    TIM6->CR1 |= TIM_CR1_CEN;
    //	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    // 等待直到计时器变为0
    while (sysTickCnt > 0)
        ;
    // 定时器失能
    //	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    TIM6->CR1 &= (uint16_t)~TIM_CR1_CEN;
}

// 延时us
void SysTick_DelayUs(__IO uint32_t nTime)
{
    // 设置时钟中断为us级
    //    SysTick_Config(SystemCoreClock / 1000000);
    TIM6->PSC = 41;
    sysTickCnt = nTime;
    // 等待计时完成
    SysTick_Wait();
    // 重新设置系统中断为ms级
    TIM6->PSC = 41999;
    // 定时器失能
    TIM6->CR1 &= (uint16_t)~TIM_CR1_CEN;
}

// 延时ms
void SysTick_DelayMs(__IO uint32_t nTime)
{
    sysTickCnt = nTime;
    SysTick_Wait();
}

// 延时s
void SysTick_DelayS(__IO uint32_t nTime)
{
    SysTick_DelayMs(nTime * 1000);
}

// 设置倒计时(非阻塞式)
void SysTick_CountdownBegin(__IO uint32_t nTime)
{
    // 这里设置为1ms中断一次
    sysTickCnt = nTime;
    // 定时器使能
    TIM6->CR1 |= TIM_CR1_CEN;
}

// 撤销倒计时
void SysTick_CountdownCancel(void)
{
    // 重置嘀嗒计时器的计数值
    sysTickCnt = 0;
    // systick 定时器失能
    TIM6->CR1 &= (uint16_t)~TIM_CR1_CEN;
}

// 判断倒计时是否超时
uint8_t SysTick_CountdownIsTimeout(void)
{
    return sysTickCnt == 0;
}

void Camera_QuickMark_do(void)
{

    Usart_SendString((uint8_t *)"A");
}
void Camera_Color_Find_do(void)
{
    Usart_SendString((uint8_t *)"B");
}
void Camera_die(void)
{
    Usart_SendString((uint8_t *)"D");
}
void Camera_2_die(void)
{
    Usart6_SendString((uint8_t *)"D");
}
void Camera_2_color_circle_do(void)
{
    Usart6_SendString((uint8_t *)"B");
}
void Camera_2_black_line_Find_do(void)
{
    Usart6_SendString((uint8_t *)"A");
}



ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PB1     ------> ADC1_IN9
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_DISABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PB1     ------> ADC1_IN9
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}