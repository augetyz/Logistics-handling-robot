#include "stm32f10x.h" 
#include "usart.h"
#include "encoder.h"
#include "motor.h"
#include "sys.h"
//通道定义
#define CH1 0X01
#define CH2 0X02
#define CH3 0X03
#define CH4 0X04
#define control_motor 1

void motor_control(int goal_1,int goal_2,int goal_3,int goal_4);
void go(float x,float y,int speed);
uint32_t KalmanFilter(int32_t ResrcData);
void date_deal(u8 *p);
void pidsend(void);

