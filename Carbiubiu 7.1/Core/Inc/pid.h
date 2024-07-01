#ifndef _PID_H_
#define _PID_H_

#include "motor_ctrl.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

void pid_do(biu  speed_goal,biu  speed_now);
void speed_CTRL(int speed1,int speed2,int speed3,int speed4);
uint8_t direction_Set(float angle,imu imu_date);
float dert(float a,float b);
void advance_angle(float angle,imu imu_date,int speed_goal);
void crosswise_angle(float angle,imu imu_date,int speed_goal);
uint8_t advance_angle_distance(float angle,imu imu_date,biu distance_now,int distance_goal);    
uint8_t crosswise_angle_distance(float angle,imu imu_date,biu distance_now,int distance_goal);
uint8_t direction_Set_biu(float angle, imu imu_date);
uint8_t Directional_move_distance(float angle, imu imu_date, biu distance_now, int distance_goal,float angle_goal);
#endif 
