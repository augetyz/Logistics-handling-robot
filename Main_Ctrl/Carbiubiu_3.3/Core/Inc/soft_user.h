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
  // ���Ͷ˻�����
  RingBufferTypeDef *sendBuf;
  // ���ն˻�����
  RingBufferTypeDef *recvBuf;
} Usart_DataTypeDef;

void delay(uint32_t time);
void delay_us(uint32_t time);
void Usart_SendString(uint8_t *str);
void clearArray(uint8_t *array, uint16_t length);

void V831_QuickMark_do(void);
void V831_Color_Find_do(void);
void V831_die(void);

void V831_2_color_circle_do(void);
void V831_2_black_line_Find_do(void);
void V831_2_die(void);
void Uart5_LCD_show_QR(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f);
void Uart5_LCD_send_task(uint8_t task);
void Uart5_LCD_show_Angle(float angle);
void Uart5_LCD_show_string(char *string);

// ��ʱ
// ��ʱ us
void SysTick_DelayUs(__IO uint32_t nTime);
// ��ʱ ms
void SysTick_DelayMs(__IO uint32_t nTime);
// ��ʱ s
void SysTick_DelayS(__IO uint32_t nTime);

// ����ʱ
// ���õ���ʱ(������ʽ)
void SysTick_CountdownBegin(__IO uint32_t nTime);
// ��������ʱ
void SysTick_CountdownCancel(void);
// �жϵ���ʱ�Ƿ�ʱ
uint8_t SysTick_CountdownIsTimeout(void);
void Usart_SendAll(Usart_DataTypeDef *usart);

#endif // !_SOFT_USER_H_
