#ifndef __KALMAN_H
#define __KALMAN_H 

//���������㷨��

extern float Angle_X_Final;			//���������
extern float Angle_Y_Final;			//���������
extern float temperature;			//�������¶�����
extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����  angular acceleration
extern short gyrox,gyroy,gyroz;		//������ԭʼ����  gyroscope


void Angle_Calcu(short* Accel,short* Gyro);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);


#endif
