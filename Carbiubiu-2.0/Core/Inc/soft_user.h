#ifndef _SOFT_USER_H_
#define _SOFT_USER_H_

#include "main.h"
#include <stdio.h>
#include "ring_buffer.h"

#define USART_RECV_BUF_SIZE 2048
#define USART_SEND_BUF_SIZE 2048

typedef struct
{  
    UART_HandleTypeDef *pUSARTx;
    // 发送端缓冲区
    RingBufferTypeDef *sendBuf;
		// 接收端缓冲区
    RingBufferTypeDef *recvBuf;
} Usart_DataTypeDef;



void delay(uint32_t time);
void delay_us(uint32_t time);
void Usart_SendString(uint8_t *str);
void clearArray(uint8_t * array,uint16_t length);

void V831_QuickMark_do(void);
void V831_Color_Find_do(void);
void V831_die(void);

void V831_2_color_circle_do(void);
void V831_2_black_line_Find_do(void);
void V831_2_die(void);


// 延时
// 延时 us
void SysTick_DelayUs(__IO uint32_t nTime);
// 延时 ms
void SysTick_DelayMs(__IO uint32_t nTime);
// 延时 s
void SysTick_DelayS(__IO uint32_t nTime);

// 倒计时
// 设置倒计时(非阻塞式)
void SysTick_CountdownBegin(__IO uint32_t nTime);
// 撤销倒计时
void SysTick_CountdownCancel(void);
// 判断倒计时是否超时
uint8_t SysTick_CountdownIsTimeout(void);
void Usart_SendAll(Usart_DataTypeDef *usart);

#endif // !_SOFT_USER_H_
