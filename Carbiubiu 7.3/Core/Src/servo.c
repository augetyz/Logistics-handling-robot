#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "soft_user.h"
#include "pid.h"
#include <math.h>
// ʹ�ö�ʱ������
/*

TIM9��TIM10��TIM11��TIM12

PSC 167 or 83
CNT 19999

���ռ�ձ� 500~2500

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
    { 90.0, -10.2,  30.4, -40.2, 2.0, 6, 2000}, // ��ʼ����0
    {-89.5,  31.6,  18.5, -39.2, -25.3, 0, 1000}, // ɫ��ʶ����1
    {-89.5, -36.1,  59.2, -39.1, -25.3, 0, 1000}, // �ƶ�ʱ��е�۶���2
    // ��ԭ��̨��ץȡ����
    {  2.5,  33.6,  26.8, -38.3, -25.3, 0, 1000}, // ����ʶ����3
    {  1.1,  15.4, -18.1, -38.1, -25.3, 0, 500}, // ׼��ץȡ����4
    {  2.8,  15.4, -18.1,  -4.6, -25.3, 0, 300}, // ץȡ���϶���5
    // ���õ�����̨��
    { 77.0, -17.3,  56.3,  -4.6, -25.3, 0, 1000},  // ׼���������϶���6
    { 72.6, -43.3,  57.7,  -4.6, -25.4, 0, 300},  // �������ϵ�̨��7
    { 72.6, -43.3,  57.7,   -30, -25.4, 0, 200},  //  ������̨��צ8  
    { 72.1, -33.3,  58.3, -29.7, -25.3, 0, 300},  // ̧���м䶯��9       
    { 70.7, -17.5,  57.5, -29.7, -25.3, 0, 1000},  // ��ɷ���10

    {  1.3,  -2.1,  33.8, -30.1, -14.4, 0, 300}, // �м䶯��11  3������4����֮����м䶯��11

    // ������̨����ȡ����
    { 72.1, -17.3,  56.3, -36.5, -43.0, 0, 2500}, // ׼��ץȡ����12
    { 72.6, -43.3,  57.7, -36.5, -43.0, 0, 550}, // ���䵽����̨��13
    { 72.6, -43.3,  57.7,  -4.6, -14.0, 0, 300}, // ��צ14
    { 77.0, -17.3,  56.3,  -4.6, -14.0, 0, 300}, // ץ��15
    {-89.5, -36.1,  59.2,  -4.6, -43.0, 0, 1200}, // �������õ���ǰ����׼������16
    //����ɫ������
    {-88.3,57.7,-81.6,  -4.6, -43.0, 0, 1000}, // �ŵ�����ɫ��17
    {-88.3,57.7,-81.6, -38.5, -14.0, 0, 300}, // ��צ18
    {-87.1,  30.1, -59.5, -38.5, -14.0, 0, 300}, // ��ɷ���̧��19
    //һ��ɫ������
    {-56.5,60.6,-56.2,  -4.6, -43.0, 0, 1000}, // �ŵ�һ��ɫ��20
    {-56.5,60.6,-56.2, -38.5, -14.0, 0, 300}, // ��צ21
    {-56.2,  29.3, -24.3, -38.5, -14.0, 0, 300}, // ��ɷ���̧��22

    //����ɫ������
    {-120.3,60.6,-55.3,  -4.6, -43.0, 0, 1000}, // �ŵ�����ɫ��23
    {-120.3,60.6,-55.3, -38.5, -14.0, 0, 300}, // ��צ24
    {-120.4, 24.5, -28.3, -38.5, -14.0, 0, 300}, // ��ɷ���̧��25

    {-89.2,-31.9,  44.6, -38.5,  -0.6, 0, 500}, // ���������м䶯��26

    //һ��ɫ�����

    {-89.2,-31.9,  44.6,   -4.6,  -5.8, 0, 800},  // ������27
    {-89.2,-31.9,  44.6,  -38.5,  -5.8, 0, 300}, // �����צ28
    {-89.3,-11.5,  22.9,  -45.7,  -5.8, 0,500}, // ���̧��29

    //����ɫ�����
    {-104.5, 9.3, -40.2,  -4.6, -40.0, 0, 800}, // ������30
    {-104.5, 9.3, -40.2, -38.6, -40.0, 0, 300}, //�����צ31
    {-104.7,-19.1,23.3, -45.7, -40.0, 0, 500}, // ���̧��32

    //����ɫ�����
    {-135.5,25.4,-31.1,  -4.6, -40.0, 0, 800}, // ������33
    {-132.5,25.4,-31.1, -38.6, -40.0, 0, 300}, //�����צ34
    {-133.5,9.0,8.1, -45.7, -40.0, 0, 500},   // ���̧��35

    {-34.6,-120.8,  26.0, -54.9, -43.0, 0, 300}, // ��ɫɫ�����õ�������ץȡ36
    {  0.8,-110.6,  66.5, -94.2, -43.0, 0, 300}, // ��ɫɫ�����õ�������ץȡ37
    { 35.5,-100.4,  41.8, -65.3, -43.0, 0, 300}, // ��ɫɫ�����õ�������ץȡ38

    {-34.2, -84.7,  72.2, -62.7,  -5.8, 0, 100}, // ��ɫɫ����ⵥ��������צ_1_39
    {  2.3, -83.9, 108.9, -98.6,  -5.8, 0, 100}, // ��ɫɫ����ⵥ��������צ_1_40
    { 35.8, -84.7,  72.2, -62.7,  -5.8, 0, 100}, // ��ɫɫ����ⵥ��������צ_1_41

    {-34.6,-120.8,  26.0, -54.9,  -5.8, 0, 100}, // ��ɫɫ�����ü�����צ_1_42
    {  0.8,-115.6,  73.4, -94.2,  -5.8, 0, 100}, // ��ɫɫ�����ü�����צ_1_43
    { 35.5,-115.4,  45.8, -70.3,  -5.8, 0, 100}, // ��ɫɫ�����ü�����צ_1_44



    {-25.0, -50.0,  15.0, 110.0, -40.0, 0,1000}, //45


    { 28.8,-115.9,  48.5, -65.7,  -5.8, 0,1000}, // ��ɫɫ����ȡר��46
    { -1.3, -42.0,  68.4, -94.2, -43.7, 0, 200}, // �ݴ���������ɫר���м仺�嶯��47
    { 25.0, -32.5,  33.5, -54.7, -43.7, 0, 200}, // �ݴ���������ɫר���м仺�嶯��48
    { -1.3, -42.0,  72.2, -62.7, -43.0, 0, 200}, // �ݴ������ú�ɫר���м仺�嶯��49
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

