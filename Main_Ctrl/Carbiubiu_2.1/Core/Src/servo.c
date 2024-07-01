#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "soft_user.h"
// 使用定时器参数
/*

TIM9、TIM10、TIM11、TIM12

PSC 167 or 83
CNT 19999

舵机占空比 500~2500

*/

extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;

servo_status Servo;

void servo_ctrl(uint8_t servo, uint16_t value)
{
    value = (float)value * 100 / Value_Ctrl + 25;

    if (servo & (1 << 0))
        PWM1 = value;
    if (servo & (1 << 1))
        PWM2 = value;
    if (servo & (1 << 2))
        PWM3 = value;
    if (servo & (1 << 3))
        PWM4 = value;
    if (servo & (1 << 4))
        PWM5 = value;
    if (servo & (1 << 5))
        PWM6 = value;
}

void servo_config(void)
{
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    //    PWM1=00;
    //    PWM2=00;
    //    PWM3=00;
    //    PWM4=00;
    //    PWM5=00;
    //    PWM6=00;

    PWM1 = 500;
    PWM2 = 2400;
    PWM3 = 1180;
    PWM4 = 2000;
    PWM5 = 1200;
    PWM6 = 1500;
    //    PWM1=1500;
    //    PWM2=1500;
    //    PWM3=1500;
    //    PWM4=1500;
    //    PWM5=1500;
    //    PWM6=1500;
}
void servo_level_move(int16_t num)
{
    extern QueueHandle_t servo_Queue;
    servo_status Servo = {800, 550, 2200, 1900, 1900, 1500, 1000, 1};
    ;
    num = num > 500 ? num : (num < -500 ? -500 : num);
    Servo.value_goal[0] = PWM1;
    Servo.value_goal[1] = PWM2;
    Servo.value_goal[2] = PWM3;
    Servo.value_goal[3] = PWM4;
    Servo.value_goal[4] = PWM5;
    Servo.value_goal[5] = PWM6 + num;
    Servo.time = 100;
    xQueueSend(servo_Queue, &Servo, 0);
}
void servo_vertical_move(servo_status Servo)
{
    extern QueueHandle_t servo_Queue;

    Servo.value_goal[5] = PWM6;

    xQueueSend(servo_Queue, &Servo, 0);
}
void servo_all_move(servo_status Servo)
{
    extern QueueHandle_t servo_Queue;

    xQueueSend(servo_Queue, &Servo, 0);
}

servo_status servo_motion[] =
    {
        //    1     2      3     4    5     6   T
        {0, -37.2, -89.4, -36.2, -30, 0, 500},       // 中间动作0
        {44.1, 0, -126.3, 16.7, 15, 0, 2000},        // 红色物料放置1
        {-34.3, 0, -126.3, 16.7, 15, 0, 2000},       // 蓝色物料放置2
        {2.4, 17.1, -112.6, 17.7, 15, 0, 2000},      // 绿色物料放置3
        {-89.9, 33.0, -82.6, 31.4, 30.3, 0, 1000},   // 准备抓取物料4
        {-90.1, 84.5, -88.3, 82.9, 30.3, 0, 1000},   // 抓取物料中间动作5
        {-90.1, 84.5, -88.3, 82.9, -40.3, 0, 200},   // 抓取物料中间动作6
        {-89.9, -11.3, -43.7, 13.5, -40.3, 0, 1000}, // 准备抓取物料7
                                                     // 1号物料台内容
        {-30.8, -40.5, -145, 46.6, -40.3, 0, 1000},  // 准备放置到一号物料台8
        {-30.8, 9.1, -145, 46.6, -40.3, 0, 1000},    // 放到一号物料台9
        {-30.8, 9.1, -145, 46.6, -15.8, 0, 200},     // 松爪10
        {-30.8, -40.5, -145, 46.6, -15.8, 0, 1000},  // 完成放置11
                                                     // 3号物料台内容
        {35.1, -40.5, -145, 46.6, -40.3, 0, 1000},   // 准备放置到三号物料台12
        {35.1, 9.1, -145, 46.6, -40.3, 0, 1000},     // 放到三号物料台13
        {35.1, 9.1, -145, 46.6, -15.8, 0, 200},      // 松爪14
        {35.1, -40.5, -145, 46.6, -15.8, 0, 1000},   // 完成放置15
                                                     // 二号物料台内容
        {0.7, -37.8, -117.9, 14.7, -40.3, 0, 1000},  // 准备放置到二号物料台16
        {0.7, 13.0, -117.9, 14.7, -40.3, 0, 1000},   // 放到二号物料台17
        {0.7, 13.6, -117.9, 14.7, -22.7, 0, 200},    // 松爪18
        {0.7, -37.8, -117.9, 14.7, -22.7, 0, 1000},  // 完成放置19

};
