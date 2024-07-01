#ifndef _ARM_H
#define _ARM_H

#include "usart.h"

void control_arm(u8 id,int16_t angle,u16 speed);
u8 check_sum(u8 *num);
#endif

