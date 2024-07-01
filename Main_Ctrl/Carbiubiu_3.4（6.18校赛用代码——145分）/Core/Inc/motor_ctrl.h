#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define Motor1 0X08
#define Motor2 0X0A
#define Motor3 0X0C
#define Motor4 0X0E

#define Mode_brake 0X03
#define Mode_gogo  0X02
#define Mode_back  0X01
#define Mode_free  0X00

void motor_tim_config(void);
void Motor_ctrl(uint8_t motor, uint16_t status);
void speed_ctrl(uint8_t motor, int speed);



#endif // ! _MOTOR_CTRL_H_
