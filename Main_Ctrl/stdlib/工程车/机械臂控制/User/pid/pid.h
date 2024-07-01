#ifndef _PID_H
#define _PID_H
#include "stm32f10x.h" 
#include "usart.h"
#include "encoder.h"
#include "motor.h"
//ͨ������
#define CH1 0X01
#define CH2 0X02
#define CH3 0X03
#define CH4 0X04

//����ָ��
#define s_order_GOAL   	 0X01		//Ŀ��ֵ
#define s_order_ACTUAL   0X02		//ʵ��ֵ
#define s_order_PID  	 0X03		//PID
#define s_order_PERIOD 	 0X06		//����


//����ָ��
#define r_order_PID  	 0X10		//PID
#define r_order_GOAL   	 0X11		//Ŀ��ֵ
#define r_order_PERIOD 	 0X15		//����

void Togo(u8 CHx,u8 order);
void date_product(u8 CHx,u8 *date,u8 order);
void pid_send(u8 *date);
float test_4hex_to_float(u8* add);//ʮ����������ת��Ϊ��Ӧfloat����,��λ��ǰ
void test_float_to_4hex(float num,u8* add);//floatת��Ϊ�ĸ��ֽڵ�ʮ����������
u8 check_sum(u8 *addr);//У���
void pid_control(int lunzi,int goal);
void date_deal(u8 *data,int num);
void motor_control(int goal_1,int goal_2,int goal_3,int goal_4);

#endif
