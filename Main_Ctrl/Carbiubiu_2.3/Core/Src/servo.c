#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "soft_user.h"
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
        {0, -37.2, -89.4, -36.2, -30, 0, 500},         // �м䶯��0
        {0, -22, -78, -37, -9, 0, 1000},               // ��ά��ʶ���м䶯��1
        {0, 34, -60, -15, -44.5, 0, 1000},             // ��ά��ʶ����2
        {-89.9, -37.2, -89.4, -36.2, -30, 0, 200},     // �м䶯��3
        {-89.9, 82.0, -101.6, 94.4, 0.3, 0, 1000},     // ׼��ץȡ����4
        {-90.1, 80.1, -66.9, 60.0, 0.3, 0, 200},       // ץȡ�����м䶯��5
        {-90.1, 80.1, -66.9, 60.0, -40.3, 0, 200},     // ץȡ�����м䶯��6
        {-89.9, -11.3, -43.7, 13.5, -40.3, 0, 500},    // ׼��ץȡ����7
                                                       // 1������̨����
        {-30.5, -41.2, -122.9, -16.2, -43, 0, 1000},   // ׼�����õ�һ������̨8
        {-30.6, 1.5, -120.8, 14.1, -42, 0, 1000},      // �ŵ�һ������̨9
        {-30.6, 1.5, -120.8, 14.1, -15.8, 0, 200},     // ��צ10
        {-30.5, -41.2, -122.9, -16.2, -15.8, 0, 1000}, // ��ɷ���11
                                                       // 3������̨����
        {32.1, -56.8, -128.4, -25.9, -43, 0, 1000},    // ׼�����õ���������̨12
        {32.0, -0.1, -117.4, 1.3, -43, 0, 1000},       // �ŵ���������̨13
        {32.0, -0.1, -117.4, 1.3, -15.8, 0, 200},      // ��צ14
        {32.1, -56.8, -128.4, -25.9, -15.8, 0, 1000},  // ��ɷ���15
                                                       // ��������̨����
        {1.8, -62.5, -102.8, -51.4, -43, 0, 1000},     // ׼�����õ���������̨16
        {1.8, 1.4, -128.8, 21.0, -43, 0, 1000},        // �ŵ���������̨17
        {1.8, 1.4, -128.8, 21.0, -20.8, 0, 200},       // ��צ18
        {1.8, -62.5, -102.8, -51.4, -22.7, 0, 1000},   // ��ɷ���19
                                                       //
        {-35.6, -121.0, 22.7, -50.2, -43, 0, 2000},    // ��ɫɫ������20
        {-0.1, -112.5, 74.8, -93.5, -43, 0, 2000},     // ��ɫɫ������21
        {35.8, -121.0, 29.5, -59.7, -43, 0, 2000},     // ��ɫɫ������22

        {-35.6, -85.0, 117.7, -109.2, -15.8, 0, 500}, // ��ɫɫ��������צ23
        {-0.1, -73.5, 136.8, -109.5, -15.8, 0, 500},  // ��ɫɫ��������צ24
        {35.8, -85.0, 117.7, -109.2, -15.8, 0, 500},  // ��ɫɫ��������צ25

};
