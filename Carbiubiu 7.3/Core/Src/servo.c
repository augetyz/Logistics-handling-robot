#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "soft_user.h"
#include "pid.h"
#include <math.h>
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
    servo_status Servo = { 800, 550, 2200, 1900, 1900, 1500, 1000, 1 };
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
    { 90.0, -10.2,  30.4, -40.2, 2.0, 6, 2000}, // 初始动作0
    {-89.5,  31.6,  18.5, -39.2, -25.3, 0, 1000}, // 色环识别动作1
    {-89.5, -36.1,  59.2, -39.1, -25.3, 0, 1000}, // 移动时机械臂动作2
    // 从原料台上抓取物料
    {  2.5,  33.6,  26.8, -38.3, -25.3, 0, 1000}, // 物料识别动作3
    {  1.1,  15.4, -18.1, -38.1, -25.3, 0, 500}, // 准备抓取物料4
    {  2.8,  15.4, -18.1,  -4.6, -25.3, 0, 300}, // 抓取物料动作5
    // 放置到物料台上
    { 77.0, -17.3,  56.3,  -4.6, -25.3, 0, 1000},  // 准备放置物料动作6
    { 72.6, -43.3,  57.7,  -4.6, -25.4, 0, 300},  // 放置物料到台子7
    { 72.6, -43.3,  57.7,   -30, -25.4, 0, 200},  //  在物料台松爪8  
    { 72.1, -33.3,  58.3, -29.7, -25.3, 0, 300},  // 抬高中间动作9       
    { 70.7, -17.5,  57.5, -29.7, -25.3, 0, 1000},  // 完成放置10

    {  1.3,  -2.1,  33.8, -30.1, -14.4, 0, 300}, // 中间动作11  3动作和4动作之间的中间动作11

    // 从物料台上拿取物料
    { 72.1, -17.3,  56.3, -36.5, -43.0, 0, 2500}, // 准备抓取物料12
    { 72.6, -43.3,  57.7, -36.5, -43.0, 0, 550}, // 下落到物料台上13
    { 72.6, -43.3,  57.7,  -4.6, -14.0, 0, 300}, // 合爪14
    { 77.0, -17.3,  56.3,  -4.6, -14.0, 0, 300}, // 抓起15
    {-89.5, -36.1,  59.2,  -4.6, -43.0, 0, 1200}, // 将物料拿到正前方，准备放置16
    //二号色环放置
    {-88.3,57.7,-81.6,  -4.6, -43.0, 0, 1000}, // 放到二号色环17
    {-88.3,57.7,-81.6, -38.5, -14.0, 0, 300}, // 松爪18
    {-87.1,  30.1, -59.5, -38.5, -14.0, 0, 300}, // 完成放置抬起19
    //一号色环放置
    {-56.5,60.6,-56.2,  -4.6, -43.0, 0, 1000}, // 放到一号色环20
    {-56.5,60.6,-56.2, -38.5, -14.0, 0, 300}, // 松爪21
    {-56.2,  29.3, -24.3, -38.5, -14.0, 0, 300}, // 完成放置抬起22

    //三号色环放置
    {-120.3,60.6,-55.3,  -4.6, -43.0, 0, 1000}, // 放到三号色环23
    {-120.3,60.6,-55.3, -38.5, -14.0, 0, 300}, // 松爪24
    {-120.4, 24.5, -28.3, -38.5, -14.0, 0, 300}, // 完成放置抬起25

    {-89.2,-31.9,  44.6, -38.5,  -0.6, 0, 500}, // 放置物料中间动作26

    //一号色环码垛

    {-89.2,-31.9,  44.6,   -4.6,  -5.8, 0, 800},  // 码垛放置27
    {-89.2,-31.9,  44.6,  -38.5,  -5.8, 0, 300}, // 码垛松爪28
    {-89.3,-11.5,  22.9,  -45.7,  -5.8, 0,500}, // 码垛抬起29

    //二号色环码垛
    {-104.5, 9.3, -40.2,  -4.6, -40.0, 0, 800}, // 码垛放置30
    {-104.5, 9.3, -40.2, -38.6, -40.0, 0, 300}, //码垛松爪31
    {-104.7,-19.1,23.3, -45.7, -40.0, 0, 500}, // 码垛抬起32

    //三号色环码垛
    {-135.5,25.4,-31.1,  -4.6, -40.0, 0, 800}, // 码垛放置33
    {-132.5,25.4,-31.1, -38.6, -40.0, 0, 300}, //码垛松爪34
    {-133.5,9.0,8.1, -45.7, -40.0, 0, 500},   // 码垛抬起35

    {-34.6,-120.8,  26.0, -54.9, -43.0, 0, 300}, // 红色色环放置单独急速抓取36
    {  0.8,-110.6,  66.5, -94.2, -43.0, 0, 300}, // 绿色色环放置单独急速抓取37
    { 35.5,-100.4,  41.8, -65.3, -43.0, 0, 300}, // 蓝色色环放置单独急速抓取38

    {-34.2, -84.7,  72.2, -62.7,  -5.8, 0, 100}, // 红色色环码垛单独急速松爪_1_39
    {  2.3, -83.9, 108.9, -98.6,  -5.8, 0, 100}, // 绿色色环码垛单独急速松爪_1_40
    { 35.8, -84.7,  72.2, -62.7,  -5.8, 0, 100}, // 蓝色色环码垛单独急速松爪_1_41

    {-34.6,-120.8,  26.0, -54.9,  -5.8, 0, 100}, // 红色色环放置急速松爪_1_42
    {  0.8,-115.6,  73.4, -94.2,  -5.8, 0, 100}, // 绿色色环放置急速松爪_1_43
    { 35.5,-115.4,  45.8, -70.3,  -5.8, 0, 100}, // 蓝色色环放置急速松爪_1_44



    {-25.0, -50.0,  15.0, 110.0, -40.0, 0,1000}, //45


    { 28.8,-115.9,  48.5, -65.7,  -5.8, 0,1000}, // 蓝色色环拿取专用46
    { -1.3, -42.0,  68.4, -94.2, -43.7, 0, 200}, // 暂存区放置绿色专用中间缓冲动作47
    { 25.0, -32.5,  33.5, -54.7, -43.7, 0, 200}, // 暂存区放置蓝色专用中间缓冲动作48
    { -1.3, -42.0,  72.2, -62.7, -43.0, 0, 200}, // 暂存区放置红色专用中间缓冲动作49
};

uint16_t get_Servo_Time_Max(uint16_t arr[], uint16_t size)
{
    uint16_t max = arr[0];
    for (uint8_t i = 1; i < size; i++)
    {
        if (arr[i] > max)
        {
            max = arr[i];
        }
    }
    return max;
}
servo_status servo_change(int x_error, int y_error, servo_status Servo)
{
    float r = 250.0f, r2 = 230.0f, k = 1;
    float angle1_write, angle2_write;
    float angle1 = Servo.value_goal[0];
    float angle2 = Servo.value_goal[2];
    x_error = x_error * k;
    y_error = y_error * k;
    if (x_error > 0 || x_error == 0)
    {
        if (y_error > 0 || y_error == 0)
        {
            angle1_write = (angle1 + (double)asin((double)(x_error / r)) / DEC);
            angle2_write = (angle2 + ((double)((180.0f * y_error) / (r2 * PI))));
        }
        else
        {
            angle1_write = (angle1 + (double)asin((double)(x_error / r)) / DEC);
            angle2_write = (angle2 - ((double)((180.0f * (-y_error)) / (r2 * PI))));
        }
    }
    else
    {
        if (y_error > 0 || y_error == 0)
        {
            angle1_write = (angle1 - (double)(asin((double)((-x_error) / r))) / DEC);
            angle2_write = (angle2 + ((double)((180.0f * y_error) / (r2 * PI))));
        }
        else
        {
            angle1_write = (angle1 - (double)(asin((double)((-x_error) / r))) / DEC);
            angle2_write = (angle2 - ((double)((180.0f * (-y_error)) / (r2 * PI))));
        }
    }
    Servo.value_goal[0] = angle1_write;
    Servo.value_goal[2] = angle2_write;

    return Servo;
}
servo_status servo_change_biu(int x_error, int y_error, servo_status Servo)
{
    float k1 = 0.4, k2 = 0.5;
    Servo.value_goal[0] += x_error * k1;
    Servo.value_goal[2] += y_error * k2;
    return Servo;
}

