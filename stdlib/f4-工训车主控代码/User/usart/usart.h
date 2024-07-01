#ifndef USART_H
#define	USART_H

#include "stm32f4xx.h"
#include <stdio.h>



void USART1_Config(void);
void USART2_Config(void);
void USART3_Config(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Uart2Send(unsigned char *p_data, unsigned int uiSize);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

#endif /* __USART1_H */
