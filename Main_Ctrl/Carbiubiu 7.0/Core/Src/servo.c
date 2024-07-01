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
        {  0.0, -37.2, -89.4, -36.2, -30.0, 0, 1000}, // �м䶯��0
        {  0.0, -22.0, -78.0, -37.0,  -9.0, 0, 500}, // ��ά��ʶ���м䶯��1
        {-90.8, -73.9,-139.8, -45.5, -42.9, 0, 400}, // ��ά��ʶ����2
        {-97.9, -37.2, -89.4, -36.2, -30.0, 0, 300}, // �м䶯��3
        {  -97,  60.0, -98.6,  90.4,   0.3, 0, 300}, // ׼��ץȡ����4
        {-97.1,  75.1, -80.0,  68.0,   0.3, 0, 300}, // ץȡ�����м䶯��5
        {-97.1,  75.1, -70.0,  60.0, -43.3, 0, 300}, // ץȡ�����м䶯��6
        {-97.5, -11.3, -42.7,  13.5, -43.3, 0, 300}, // ׼��ץȡ����7
        
        {-29.5, -32.5,-129.6,  53.5, -43.0, 0, 300}, // ׼�����õ�һ������̨8  
        {-29.5,  28.5,-129.6,  53.5, -43.0, 0, 400}, // �ŵ�һ������̨9
        {-29.5,  28.5,-129.6,  53.5, -14.0, 0, 300}, // ��צ10
        {-29.5, -35.5,-129.6,  53.5, -14.4, 0, 300}, // ��ɷ���11
        
        { 26.0, -32.5,-129.6,  53.5, -43.0, 0, 300}, // ׼�����õ���������̨12
        { 26.0,  18.5,-129.6,  53.5, -43.0, 0, 550}, // �ŵ���������̨13
        { 26.0,  18.5,-129.6,  53.5, -14.0, 0, 300}, // ��צ14
        { 26.0, -32.5,-129.6,  53.5, -14.0, 0, 300}, // ��ɷ���15
        
        { -1.3, -42.0,-141.0,  60.5, -43.0, 0, 300}, // ׼�����õ���������̨16
        { -1.3,   0.0,-141.0,  37.0, -43.0, 0, 400}, // �ŵ���������̨17
        { -1.3,   0.0,-141.0,  37.5, -14.0, 0, 300}, // ��צ18
        { -1.3, -42.0,-141.0,  60.5, -14.0, 0, 300}, // ��ɷ���19
        
        {-33.6,-115.8,  22.0, -50.9, -43.4, 0, 1100}, // ��ɫɫ������20
        {  0.8,-115.6,  68.4, -94.2, -43.7, 0, 1100}, // ��ɫɫ������21
        { 35.5,-112.4,  45.8, -70.3, -43.0, 0, 1100}, // ��ɫɫ������22
        
        {-35.6, -85.0, 117.7,-109.2,  -5.8, 0, 500}, // ��ɫɫ��������צ���ջ�_2_23
        { -0.1, -73.5, 138.8,-126.5,  -5.8, 0, 500}, // ��ɫɫ��������צ���ջ�_2_24
        { 35.8, -85.0, 117.7,-115.2,  -5.8, 0, 500}, // ��ɫɫ��������צ���ջ�_2_25
        
        {-97.5,  24.1, -75.4, -52.0,  -0.6, 0, 500}, // ��ɫʶ����26
        
        {-35.6,-121.0,  20.5, -50.2,  -5.8, 0, 800}, // ��ɫɫ������27
        { -0.1,-100.5,  80.0, -93.5,  -5.8, 0, 800}, // ��ɫɫ������28
        { 32.8,-115.9,  30.5, -50.7,  -5.8, 0,1000}, // ��ɫɫ������29  ��ɫɫ����ȡר�ö�����46
        
        
        {-34.2, -84.7,  72.2, -62.7, -40.0, 0, 900}, // ��ɫɫ�����30
        {  2.3, -76.9, 100.9, -98.6, -40.0, 0, 1200}, // ��ɫɫ�����31
        { 35.8, -80.7,  75.2, -62.7, -40.0, 0, 900}, // ��ɫɫ�����32
        
        {-32.9, -41.2, 135.8, -127.0, -5.8, 0, 500}, // ��ɫɫ�������צ���ջ�_2_33
        {  2.3, -30.4, 135.8, -127.0, -5.8, 0, 500}, // ��ɫɫ�������צ���ջ�_2_34
        { 35.8, -41.2, 135.8, -127.0, -5.8, 0, 500}, // ��ɫɫ�������צ���ջ�_2_35
        
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