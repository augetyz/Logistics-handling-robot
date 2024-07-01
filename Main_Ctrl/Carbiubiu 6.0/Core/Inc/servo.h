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

#define use1 0,8000,15000,1500
#define use2 4150,7000,6300,1500 //瞄准物料
#define use3 4150,14400,6300,1500 //降落，准备抓
#define use4 4150,100,17000,950  // 合爪 并升起 收缩臂

#define use5 1200,100,15800,950  //红色物料 移动 瞄准 准备放置
#define use6 1200,100,15800,1400 //直接丢下去
#define use7 1200,1200,15800,1400 //下降 准备拿取红色物料
#define use8 1200,1200,15800,950  //合爪

#define use9 0,100,20000,950  //绿色物料 移动 瞄准 准备放置
#define use10 0,100,20000,1400 //直接丢下去
#define use11 0,1200,20000,1400 //下降 准备拿取绿色物料
#define use12 0,1200,20000,950  //合爪

#define use13 15300,100,15800,950  //蓝色色物料 移动 瞄准 准备放置
#define use14 15300,100,15800,1400 //直接丢下去
#define use15 15300,1200,15800,1400 //下降 准备拿取蓝色物料
#define use16 15300,1200,15800,950  //合爪

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

#endif // !_SERVO_H_
