#ifndef __KALMAN_H
#define __KALMAN_H 

//卡尔曼解算法库

extern float Angle_X_Final;			//解算后俯仰角
extern float Angle_Y_Final;			//解算后横滚角
extern float temperature;			//陀螺仪温度数据
extern short aacx,aacy,aacz;		//加速度传感器原始数据  angular acceleration
extern short gyrox,gyroy,gyroz;		//陀螺仪原始数据  gyroscope


void Angle_Calcu(short* Accel,short* Gyro);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);


#endif
