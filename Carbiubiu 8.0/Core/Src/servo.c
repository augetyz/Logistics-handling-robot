#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "soft_user.h"
#include "pid.h"
#include <math.h>
#include "fashion_star_uart_servo.h"

//色环逆解调
//red
#define  Red_Xangle_Kp      0.07
#define  Red_Ychange_K      0.5
#define  Red_Xangle         0      //左减右加
#define  Red_Ychange       -286     //前减后加
//green
#define  Green_Xangle_Kp    0.07
#define  Green_Ychange_K    0.4
#define  Green_Xangle       -9    
#define  Green_Ychange     -335
//blue
#define  Blue_Xangle_Kp     0.07
#define  Blue_Ychange_K     0.5
#define  Blue_Xangle        -1
#define  Blue_Ychange      -286

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
    //    1       2       3       4       5       6   T
    {  90.0,  -10.2,   30.4,  -35.2,    2.0,    6, 2000}, // 初始动作0
    { -89.5,   31.6,   18.5,  -35.2,  -25.3,    0, 1000}, // 色环识别动作1
    { -89.5,  -36.1,   59.2,  -35.1,  -25.3,    0, 1000}, // 移动时机械臂动作2
    {   2.5,   33.6,   26.8,  -35.3,  -25.3,    0, 500}, // 物料识别动作3
    {   1.1,   15.4,  -18.1,  -35.1,  -25.3,    0,  300}, // 准备抓取物料4
    {   2.8,   35.4,  -10.1,   20.0,  -25.3,    0,  300}, // 抓取物料动作5
    {  73.0,  -17.3,   59.3,   20.0,  -25.3,    0, 600}, // 准备放置物料动作6
    
    {  72.6,  -32.4,   52.6,   20.0,  -25.4,    0,  500}, // 放置物料到台子7
    {  72.6,  -32.4,   52.6,  -35.0,  -25.4,    0,  300}, // 在物料台松爪8
    {  73.1,  -33.3,   62.3,  -35.7,  -25.3,    0,  400}, // 抬高中间动作9
    {  73.7,  -17.5,   57.5,  -35.7,  -25.3,    0, 500}, // 完成放置10
    
    {   1.3,   -2.1,   33.8,  -35.1,  -14.4,    0,  300}, // 中间动作11  3动作和4动作之间的中间动作11
    
    {  73.1,  -25.3,   65.3,  -26.5,  -40.0,    0, 1000}, // 准备抓取物料12
    {  72.6,  -32.4,   52.6,  -26.5,  -40.0,    0,  500}, // 下落到物料台上13
    {  72.6,  -32.4,   52.6,   20.0,  -20.0,    0,  300}, // 合爪14
    {  73.1,  -17.3,   56.3,   20.0,  -20.0,    0,  400}, // 抓起15
    
    { -89.5,  -36.1,   59.2,   20.0,  -43.0,    0, 1000}, // 将物料拿到正前方，准备放置16
    
    { -88.3,   57.7,  -81.6,   20.0,  -30.0,    0, 1000}, // 放到二号色环17
    { -88.3,   57.7,  -81.6,  -35.5,  -20.0,    0,  300}, // 松爪18
    { -87.1,    5.1,  -11.1,  -35.5,  -20.0,    0,  300}, // 完成放置抬起19
    
    { -56.5,   65.6,  -51.2,   20.0,  -30.0,    0, 1000}, // 放到一号色环20
    { -56.5,   65.6,  -51.2,  -35.5,  -20.0,    0,  300}, // 松爪21
    { -56.2,   29.3,  -24.3,  -35.5,  -20.0,    0,  300}, // 完成放置抬起22
    
    { -120.3,  60.6,  -55.3,   20.0,  -43.0,    0, 1000}, // 放到三号色环23
    { -120.3,  60.6,  -55.3,  -35.5,  -20.0,    0,  300}, // 松爪24
    { -120.4,  24.5,  -28.3,  -35.5,  -20.0,    0,  300}, // 完成放置抬起25
    
    { -89.2,  -31.9,   44.6,  -35.5,   -0.6,    0,  500}, // 放置物料中间动作26
    //红色物料码垛
    { -50.9,   29.2,  -28.4,   20.0,   -5.8,    0,  500}, // 码垛放置27
    { -50.9,   29.2,  -28.4,  -35.5,   -5.8,    0,  300}, // 码垛松爪28
    { -50.9,   -9.0,   24.6,  -35.7,   -5.8,    0,  300}, // 码垛抬起29
    //绿色物料码垛
    { -84.0,   10.4,  -40.7,   20.0,  -40.0,    0,  500}, // 码垛放置30
    { -84.0,   10.4,  -40.7,  -35.6,  -40.0,    0,  300}, // 码垛松爪31
    { -85.7,  -26.7,   34.9,  -35.7,  -40.0,    0,  300}, // 码垛抬起32
    //蓝色物料码垛
    { -121.7,  20.6,  -33.5,   20.0,  -40.0,    0,  500}, // 码垛放置33
    { -121.7,  20.6,  -33.5,  -35.6,  -40.0,    0,  300}, // 码垛松爪34
    { -121.4, -17.2,   32.5,  -35.7,  -40.0,    0,  300}, // 码垛抬起35
    
    { -119.0,  45.8,   18.2,   20.0,  -40.0,    0,  500}, // 拿着物料扫描红色色环36
    { -88.5,   21.6,   22.4,   20.0,  -40.0,    0,  500}, // 拿着物料扫描绿色色环37
    { -57.9,   45.8,   18.2,   20.0,  -40.0,    0,  500}, // 拿着物料扫描蓝色色环38
    
    { -86.0,  -28.3,   53.8,   20.0,  -40.0,    0,  800}, // 码垛中间动作合爪 39
    { -86.0,  -28.3,   53.8,  -30.7,  -40.0,    0,  500}, // 码垛中间动作松爪 40
    
    { -55.8,   49.0,  -39.3,   20.0,  -28.7,    0,  1000},         // 41 1号物料合爪
    { -88.1,   42.3,  -61.5,   20.0,  -28.7,    0,  1000},         // 42 2号物料
    { -120.3,  48.6,  -42.9,   20.0,      0,    0,  1000},         // 43 3号物料
    
    { -55.8,   49.0,  -39.3,  -30.7,  -28.7,    0,  1000},         // 44 1号物料松爪
    { -88.1,   42.3,  -61.5,  -30.7,  -28.7,    0,  1000},         // 45 2号物料松爪
    { -120.3,  48.6,  -42.9,  -30.7,      0,    0,  1000},         // 46 3号物料松爪
    
    {    0,  37.2,     11.3,   20.0,      0,    0,  1000},        // 47 在成品区拿着物料执行扫描
    {    0,   6.2,     -3.4,   20.0,      0,    0,  1000},        // 48 在成品区放置
    {    0,   6.2,     -3.4,  -30.7,      0,    0,  200},        // 49 放置，松爪
    {    0,  -5.2,     40.4,  -30.7,      0,    0,  400},        // 50 放置，抬高
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

//红色色环pid
servo_status CR_Arm_adjust_red(biu_int_16 color_date, servo_status Servo_Inital)
{
    extern Usart_DataTypeDef* servoUsart;
    static  servo_status Servo_now;
    static uint8_t sign = 0;
    float kp = Red_Xangle_Kp, ki = 0, angle_dert = 0, angle_dert_sum = 0, angle_out = 0;
    angle_dert = Red_Xangle - color_date.date[0];
    if (abs((int)angle_dert) < 3)
    {
        sign = 0;
        angle_dert_sum = 0;
        Servo_now.sign = 1;
        return Servo_now;
    }
    if (sign == 0)
    {
        Servo_now = Servo_Inital;
        sign = 1;
    }
    else
    {
        angle_dert_sum += angle_dert;
        angle_out = kp * angle_dert + ki * angle_dert_sum;
        angle_out = angle_out > 3 ? 3 : (angle_out < -3 ? -3 : angle_out);
        if (angle_out < 0.3f && angle_out> -0.3f)
            angle_out = angle_out > 0 ? 0.3 : (angle_out < 0 ? -0.3 : 0);
        Servo_now.value_goal[0] += angle_out;
        Servo_now.time = angle_out * 100;
        FSUS_SetServoAngle(servoUsart, 1, Servo_now.value_goal[0], Servo_now.time, 0, 0);
        osDelay(Servo_now.time + 300);
    }
    Servo_now.sign = 0;
    return Servo_now;
}
//绿色色环pid
servo_status CR_Arm_adjust_green(biu_int_16 color_date, servo_status Servo_Inital)
{
    extern Usart_DataTypeDef* servoUsart;
    static  servo_status Servo_now;
    static uint8_t sign = 0;
    float kp = Green_Xangle_Kp, ki = 0, angle_dert = 0, angle_dert_sum = 0, angle_out = 0;
    angle_dert = Green_Xangle - color_date.date[0];
    if (abs((int)angle_dert) < 3)
    {
        sign = 0;
        angle_dert_sum = 0;
        Servo_now.sign = 1;
        return Servo_now;
    }
    if (sign == 0)
    {
        Servo_now = Servo_Inital;
        sign = 1;
    }
    else
    {
        angle_dert_sum += angle_dert;
        angle_out = kp * angle_dert + ki * angle_dert_sum;
        angle_out = angle_out > 3 ? 3 : (angle_out < -3 ? -3 : angle_out);
        if (angle_out < 0.3f && angle_out> -0.3f)
            angle_out = angle_out > 0 ? 0.3 : (angle_out < 0 ? -0.3 : 0);
        Servo_now.value_goal[0] += angle_out;
        Servo_now.time = angle_out * 100;
        FSUS_SetServoAngle(servoUsart, 1, Servo_now.value_goal[0], Servo_now.time, 0, 0);
        osDelay(Servo_now.time + 300);
    }
    Servo_now.sign = 0;
    return Servo_now;
}
//蓝色色环pid
servo_status CR_Arm_adjust_blue(biu_int_16 color_date, servo_status Servo_Inital)
{
    extern Usart_DataTypeDef* servoUsart;
    static  servo_status Servo_now;
    static uint8_t sign = 0;
    float kp = Blue_Xangle_Kp, ki = 0, angle_dert = 0, angle_dert_sum = 0, angle_out = 0;
    angle_dert = Blue_Xangle - color_date.date[0];
    if (abs((int)angle_dert) < 3)
    {
        sign = 0;
        angle_dert_sum = 0;
        Servo_now.sign = 1;
        return Servo_now;
    }
    if (sign == 0)
    {
        Servo_now = Servo_Inital;
        sign = 1;
    }
    else
    {
        angle_dert_sum += angle_dert;
        angle_out = kp * angle_dert + ki * angle_dert_sum;
        angle_out = angle_out > 3 ? 3 : (angle_out < -3 ? -3 : angle_out);
        if (angle_out < 0.3f && angle_out> -0.3f)
            angle_out = angle_out > 0 ? 0.3 : (angle_out < 0 ? -0.3 : 0);
        Servo_now.value_goal[0] += angle_out;
        Servo_now.time = angle_out * 100;
        FSUS_SetServoAngle(servoUsart, 1, Servo_now.value_goal[0], Servo_now.time, 0, 0);
        osDelay(Servo_now.time + 300);
    }
    Servo_now.sign = 0;
    return Servo_now;
}
/*
high:需要调整高度（正升高负降低）
y：输入相机偏差值y
输出值改变在结构体Servo.value_change数组中
若需要直接作用在Servo.value_goal中只需改变return上面两行
*/

servo_status MaxArm_landing_red(int high, int y, servo_status Servo)
{
    double h, w, h_error, y_error;
    double A, B;
    double k = Red_Ychange_K;
    if (y > 0)
        y = -y;
    h_error = high;
    y = Red_Ychange - y;
    y_error = k * y;
    A = Initial_angle_2 - Servo.value_goal[1];
    B = Initial_angle_3 - Servo.value_goal[2];
    h = L1 * sin(B * DEC) - L2 * sin(A * DEC);
    w = L1 * cos(B * DEC) + L2 * cos(A * DEC);

    h = h_error;
    w = w + y_error;

    A = (90 - (((asin((pow(w, 2) + pow(L1, 2) - pow(L2, 2) + pow(h, 2)) / (2 * L1 * sqrt(pow(w, 2) + pow(h, 2))))) - atan(h / w)) / DEC));
    B = (acos((w - L1 * cos(A * DEC)) / L2) / DEC);
    Servo.value_goal[1] = Initial_angle_2 - A;
    Servo.value_goal[2] = Initial_angle_3 - B;
    return Servo;
}
servo_status MaxArm_landing_green(int high, int y, servo_status Servo)
{
    double h, w, h_error, y_error;
    double A, B;
    double k = Green_Ychange_K;
    if (y > 0)
        y = -y;
    h_error = high;
    y = Green_Ychange - y;
    y_error = k * y;
    A = Initial_angle_2 - Servo.value_goal[1];
    B = Initial_angle_3 - Servo.value_goal[2];
    h = L1 * sin(B * DEC) - L2 * sin(A * DEC);
    w = L1 * cos(B * DEC) + L2 * cos(A * DEC);

    h = h_error;
    w = w + y_error;

    A = (90 - (((asin((pow(w, 2) + pow(L1, 2) - pow(L2, 2) + pow(h, 2)) / (2 * L1 * sqrt(pow(w, 2) + pow(h, 2))))) - atan(h / w)) / DEC));
    B = (acos((w - L1 * cos(A * DEC)) / L2) / DEC);
    Servo.value_goal[1] = Initial_angle_2 - A;
    Servo.value_goal[2] = Initial_angle_3 - B;
    return Servo;
}
servo_status MaxArm_landing_blue(int high, int y, servo_status Servo)
{
    double h, w, h_error, y_error;
    double A, B;
    double k = Blue_Ychange_K;
    if (y > 0)
        y = -y;
    h_error = high;
    y = Blue_Ychange - y;
    y_error = k * y;
    A = Initial_angle_2 - Servo.value_goal[1];
    B = Initial_angle_3 - Servo.value_goal[2];
    h = L1 * sin(B * DEC) - L2 * sin(A * DEC);
    w = L1 * cos(B * DEC) + L2 * cos(A * DEC);

    h = h_error;
    w = w + y_error;

    A = (90 - (((asin((pow(w, 2) + pow(L1, 2) - pow(L2, 2) + pow(h, 2)) / (2 * L1 * sqrt(pow(w, 2) + pow(h, 2))))) - atan(h / w)) / DEC));
    B = (acos((w - L1 * cos(A * DEC)) / L2) / DEC);
    Servo.value_goal[1] = Initial_angle_2 - A;
    Servo.value_goal[2] = Initial_angle_3 - B;
    return Servo;
}
/*
wise:前后坐标（给多少就到哪）
high:高度坐标（给多少就到哪）
输出值改变在结构体Servo.value_change数组中
若需要直接作用在Servo.value_goal中只需改变return上面两行
high=-80时，完成放置
*/
servo_status servo_change_personal(int wise, int high, servo_status Servo)
{
    double h, w;
    w = wise;
    h = high;
    double A, B;
    A = (90 - (((asin((pow(w, 2) + pow(L1, 2) - pow(L2, 2) + pow(h, 2)) / (2 * L1 * sqrt(pow(w, 2) + pow(h, 2))))) - atan(h / w)) / DEC));
    B = (acos((w - L1 * cos(A * DEC)) / L2) / DEC);

    Servo.value_goal[1] = Initial_angle_2 - A;
    Servo.value_goal[2] = Initial_angle_3 - B;
    return Servo;
}


