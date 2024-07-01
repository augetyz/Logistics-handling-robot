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
    default:
        break;
    }
}
void Uart5_LCD_show_QR(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f)
{
    char date[40];
    sprintf(date, "main.num.txt=\"%d%d%d+%d%d%d\"\xff\xff\xff", a, b, c, d, e, f);
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

void V831_QuickMark_do(void)
{

    Usart_SendString((uint8_t *)"A");
}
void V831_Color_Find_do(void)
{
    Usart_SendString((uint8_t *)"B");
}
void V831_die(void)
{
    Usart_SendString((uint8_t *)"D");
}
void V831_2_die(void)
{
    Usart6_SendString((uint8_t *)"D");
}
void V831_2_color_circle_do(void)
{

    Usart6_SendString((uint8_t *)"B");
}
void V831_2_black_line_Find_do(void)
{
    Usart6_SendString((uint8_t *)"A");
}
