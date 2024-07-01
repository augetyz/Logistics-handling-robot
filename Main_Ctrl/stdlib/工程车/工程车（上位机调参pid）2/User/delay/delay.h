#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"
#define Delay_ms(x) delay_ms(x);
#define Delay_us(x) delay_us(x);
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























