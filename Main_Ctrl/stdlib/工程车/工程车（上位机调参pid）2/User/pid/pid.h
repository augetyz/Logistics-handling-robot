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


void pid_control(int lunzi,int goal);

void motor_control(int goal_1,int goal_2,int goal_3,int goal_4);
