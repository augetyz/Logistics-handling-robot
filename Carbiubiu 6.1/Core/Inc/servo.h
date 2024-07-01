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

//           A  B  C  S
#define use1 10,10,300,1500
#define use2 800,300,300,1500 //��׼����
#define use3 800,800,300,1500 //���䣬׼��ץ
#define use4 800,10,300,950  // ��צ ������ ������

#define use5 260,10,1040,950  //��ɫ���� �ƶ� ��׼ ׼������
#define use6 260,10,1040,1500 //ֱ�Ӷ���ȥ
#define use7 260,260,1040,1500 //�½� ׼����ȡ��ɫ����
#define use8 260,260,1040,950  //��צ

#define use9  30,10,1550,950  //��ɫ���� �ƶ� ��׼ ׼������
#define use10 30,10,1550,1500  //ֱ�Ӷ���ȥ
#define use11 30,10,1550,1500  //�½� ׼����ȡ��ɫ����
#define use12 30,10,1300,950  //��צ
#define use13 30,10,10,950    //�ƹ���ɫ��
#define use14 30,10,10,1500


#define use15 -240,10,1040,1500 //ֱ�Ӷ���ȥ
#define use16 -240,260,1040,1500 //�½� ׼����ȡ��ɫ����
#define use17 -240,260,1040,950  //��צ
#define use18 -240,10,1040,950  //��ɫ���� �ƶ� ��׼ ׼������
#define use19 -240,10,10,950  //�ƴ�Ȧ�߹���

#define use20 800,10,300,1500 //̧��ȥ��׼��

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
