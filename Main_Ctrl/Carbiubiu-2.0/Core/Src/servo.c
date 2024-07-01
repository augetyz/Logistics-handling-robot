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
    
    PWM1=500;
    PWM2=2400;
    PWM3=1180;
    PWM4=2000;
    PWM5=1200;
    PWM6=1500;
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
    servo_status Servo={800,550,2200,1900,1900,1500,1000,1};;
    num=num>500?num:(num<-500?-500:num);
    Servo.value_goal[0]=PWM1;
    Servo.value_goal[1]=PWM2;
    Servo.value_goal[2]=PWM3;
    Servo.value_goal[3]=PWM4;
    Servo.value_goal[4]=PWM5;
    Servo.value_goal[5]=PWM6+num;
    Servo.time=100;
    xQueueSend(servo_Queue,&Servo,0);
}
void servo_vertical_move(servo_status Servo)
{
    extern QueueHandle_t servo_Queue;
    
    Servo.value_goal[5]=PWM6;

    xQueueSend(servo_Queue,&Servo,0);
}
void servo_all_move(servo_status Servo)
{
    extern QueueHandle_t servo_Queue;
    
    xQueueSend(servo_Queue,&Servo,0);
}

servo_status servo_motion[]=
{  // 1    2    3   4    5   6   T 
    { 0,   0,   0,  0,   0,  0, 1000},//�м䶯������ʼ����0
    {90 ,90,30,-40,20,-20,2000},
    {500 ,550, 2000,1900,1900,2200,2000},
    {500 ,500, 1300,1400,2500, 800,2000},
    {500 ,2400,1000,2400, 900,2200,1000},//ץȡ������ʼ4
    {500 ,2400,2030,2300,1800,2200,1000},//5
    {1200,2400,2030,2300,1800,2200,1000},
    {1300,2400,1180,1900,1000,1500,1000},
    {1300,2400, 900,2000,1200,1770,1000},//���ú�ɫ��鶯��8
    {1300,2300,1200,2300,1000,1770,1000},
    {500 ,2300,1200,2300,1000,1770,1000},
    {500 ,2400,1300,1900,1200,1770, 500},
    {1300,2400, 900,2000,1200,1530,1000},//������ɫ���϶���12
    {1300,2400,1500,2400,1100,1500,1000},
    {500 ,2400,1500,2400,1100,1500,1000},
    {500 ,2400,1300,1900,1200,1500,500 },
    {1200,2450, 900,2000,1200,1230,1000},//������ɫ���϶���16
    {1200,2450,1350,2350,1000,1230,1000},
    {500 ,2450,1350,2300,1100,1230,500 },
    {500 ,2400,1300,1900,1200,1230,1000},   

    
    {500,2400,500,500,2000,1450,1000}, // ��е��������׼��ɨ��ǰ������20
//    {500,2400,500,500,1100,1450,1000},  // ��е�ŵ�����λ��21
    {500,2400,1500,600,2000,1450,1000},  // ��е�ŵ�����λ��21

    // ��ȡ���ϵ���ɫ����
    {500, 2400, 1180, 2000, 1200, 1500, 1000},   // ���в�תͷ 22 
    {500, 2400, 1600, 2500, 1000, 1500, 1000},  // ��е���³� 23
    {500, 2400, 1200, 2200, 1200, 1500, 1000},  // ��е�۵���λ�� 24
    {1300, 2400, 1200, 2200, 1200, 1500, 400},  // ץȡ 25
    {1300, 2400, 1180, 2000, 1200, 1450, 1000}, // �����м�λ�� 26

    //������ɫ���
    {1300, 500, 500, 500, 2000, 1450, 1000},  // ��е������ 27
    {1300, 500, 500, 700, 750, 1450, 1000},     //�ŵ�λ��  28
    {500, 500, 500, 600, 750, 1450, 1000},      //���� 29
    {500, 500, 500, 600, 2000, 1450, 1000},     //�ص��м�λ�� 30
    
    //ɨ���ɫɫ��
    {500, 2400, 1500, 1400, 800, 1700, 1000}, // ɨ���ɫɫ������Ҫ�Ѿ���ɶ���23   31
};





