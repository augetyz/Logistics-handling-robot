#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define Value_Ctrl 180 // 舵机角度控制范围：0~Value_Ctrl  用于控制函数servo_ctrl的修整参数

#define servo1 0X01
#define servo2 0X02
#define servo3 0X04
#define servo4 0X08
#define servo5 0X10
#define servo6 0X20

#define PWM1 TIM10->CCR1
#define PWM2 TIM11->CCR1
#define PWM3 TIM12->CCR1
#define PWM4 TIM12->CCR2
#define PWM5 TIM9->CCR2
#define PWM6 TIM9->CCR1

#define PI     					 3.14159265359f
#define L1     					 128.4
#define L2     					 138.0
#define Initial_angle_2  85.0f //二号舵机初始角度  减去计算角度即可
#define Initial_angle_3  43.5f //三号舵机初始角度  减去计算角度即可




typedef struct
{
   float value_goal[6];
   uint16_t time;
   int8_t sign;
} servo_status;

servo_status servo_date_deal(uint8_t *date);
void servo_ctrl(uint8_t servo, uint16_t value);
void servo_config(void);
void servo_Ctrl(servo_status servo);
void servo_level_move(int16_t num);
void servo_vertical_move(servo_status Servo);
void servo_all_move(servo_status Servo);
uint16_t get_Servo_Time_Max(uint16_t arr[], uint16_t size);
servo_status MaxArm_landing(int high, int y, servo_status Servo);
servo_status servo_change_biu(int x_error, int y_error, servo_status Servo);
servo_status CR_Arm_adjust(biu_int_16 color_date, servo_status Servo_Inital);
servo_status servo_change_personal(int x, int y, servo_status Servo);


#endif // !_SERVO_H_
