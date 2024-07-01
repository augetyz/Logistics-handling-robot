#ifndef _SOFT_USER_H_
#define _SOFT_USER_H_

#include "main.h"
#include <stdio.h>
#include "ring_buffer.h"

#define USART_RECV_BUF_SIZE 2048
#define USART_SEND_BUF_SIZE 2048
extern ADC_HandleTypeDef hadc1;
typedef struct
{
  UART_HandleTypeDef *pUSARTx;
  // ??????????
  RingBufferTypeDef *sendBuf;
  // ??????????
  RingBufferTypeDef *recvBuf;
} Usart_DataTypeDef;

void delay(uint32_t time);
void delay_us(uint32_t time);
void Usart_SendString(uint8_t *str);
void clearArray(uint8_t *array, uint16_t length);

void Camera_QuickMark_do(void);
void Camera_Color_Find_do(void);
void Camera_die(void);
void Camera_color_circle_location(void);

void Camera_2_color_circle_do(void);
void Camera_2_black_line_Find_do(void);
void Camera_2_die(void);
void Uart5_LCD_show_QR(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f);
void Uart5_LCD_send_task(uint8_t task);
void Uart5_LCD_show_Angle(float angle);
void Uart5_LCD_show_string(char *string);
void Uart5_LCD_show_X_Y(int16_t x, int16_t y);
// ???
// ??? us
void SysTick_DelayUs(__IO uint32_t nTime);
// ??? ms
void SysTick_DelayMs(__IO uint32_t nTime);
// ??? s
void SysTick_DelayS(__IO uint32_t nTime);

// ?????
// ????????(???????)
void SysTick_CountdownBegin(__IO uint32_t nTime);
// ?????????
void SysTick_CountdownCancel(void);
// ?��?????????
uint8_t SysTick_CountdownIsTimeout(void);
void Usart_SendAll(Usart_DataTypeDef *usart);




void MX_ADC1_Init(void);
#endif // !_SOFT_USER_H_
