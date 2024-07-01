#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define Value_Ctrl 180 // ����Ƕȿ��Ʒ�Χ��0~Value_Ctrl  ���ڿ��ƺ���servo_ctrl����������

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
#define use2 4150,7000,6300,1500 //��׼����
#define use3 4150,14400,6300,1500 //���䣬׼��ץ
#define use4 4150,100,17000,950  // ��צ ������ ������

#define use5 1200,100,15800,950  //��ɫ���� �ƶ� ��׼ ׼������
#define use6 1200,100,15800,1400 //ֱ�Ӷ���ȥ
#define use7 1200,1200,15800,1400 //�½� ׼����ȡ��ɫ����
#define use8 1200,1200,15800,950  //��צ

#define use9 0,100,20000,950  //��ɫ���� �ƶ� ��׼ ׼������
#define use10 0,100,20000,1400 //ֱ�Ӷ���ȥ
#define use11 0,1200,20000,1400 //�½� ׼����ȡ��ɫ����
#define use12 0,1200,20000,950  //��צ

#define use13 15300,100,15800,950  //��ɫɫ���� �ƶ� ��׼ ׼������
#define use14 15300,100,15800,1400 //ֱ�Ӷ���ȥ
#define use15 15300,1200,15800,1400 //�½� ׼����ȡ��ɫ����
#define use16 15300,1200,15800,950  //��צ

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
