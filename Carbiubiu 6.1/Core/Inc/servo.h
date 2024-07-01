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

//           A  B  C  S
#define use1 10,10,300,1500
#define use2 800,300,300,1500 //瞄准物料
#define use3 800,800,300,1500 //降落，准备抓
#define use4 800,10,300,950  // 合爪 并升起 收缩臂

#define use5 260,10,1040,950  //蓝色物料 移动 瞄准 准备放置
#define use6 260,10,1040,1500 //直接丢下去
#define use7 260,260,1040,1500 //下降 准备拿取蓝色物料
#define use8 260,260,1040,950  //合爪

#define use9  30,10,1550,950  //绿色物料 移动 瞄准 准备放置
#define use10 30,10,1550,1500  //直接丢下去
#define use11 30,10,1550,1500  //下降 准备拿取绿色物料
#define use12 30,10,1300,950  //合爪
#define use13 30,10,10,950    //绕过红色的
#define use14 30,10,10,1500


#define use15 -240,10,1040,1500 //直接丢下去
#define use16 -240,260,1040,1500 //下降 准备拿取红色物料
#define use17 -240,260,1040,950  //合爪
#define use18 -240,10,1040,950  //红色物料 移动 瞄准 准备放置
#define use19 -240,10,10,950  //绕大圈走过来

#define use20 800,10,300,1500 //抬高去瞄准。

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
