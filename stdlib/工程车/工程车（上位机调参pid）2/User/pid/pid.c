#include "pid.h"

//float   kp1=9.6,ki1=0.64,kd1=34.56;
float   kp1=7.38,ki1=0.434,kd1=3.11;
float   kp2=2.5,ki2=0,kd2=0;
float   kp3=2.5,ki3=0,kd3=0;
float   kp4=2.5,ki4=0,kd4=0;
int goal_speed=40;//目标速度
int actual_speed;//实际速度
int dif_speed;//差值
u8 get_date[24];

void pid_control(int lunzi,int goal)
{
	int pwm_out=goal;
	static int dif_speed_sum;
	static int dif_speed_last;
	dif_speed=goal-actual_speed;
	dif_speed_sum+=dif_speed;	
	pwm_out=pwm_out+kp1*dif_speed+ki1*dif_speed_sum+kd1*(dif_speed-dif_speed_last)+45;	
	speed_control(lunzi,pwm_out);
	dif_speed_last=dif_speed;
}
void motor_control(int goal_1,int goal_2,int goal_3,int goal_4)
{
	int go1=1,go2=1,go3=1,go4=1;
	static int pwm_out1,pwm_out2,pwm_out3,pwm_out4;
	int speed1,speed2,speed3,speed4;
	int differ_speed1,differ_speed2,differ_speed3,differ_speed4;
	static int dif_speed_sum1;static int dif_speed_last1,dif_speed_last1_2;
	static int dif_speed_sum2;static int dif_speed_last2,dif_speed_last2_2;
	static int dif_speed_sum3;static int dif_speed_last3,dif_speed_last3_2;
	static int dif_speed_sum4;static int dif_speed_last4,dif_speed_last4_2;

	speed1=Get_Motor_Speed(1);speed2=Get_Motor_Speed(2);speed3=Get_Motor_Speed(3);speed4=Get_Motor_Speed(4);
	printf("%d,%d,%d,%d,%d\n",goal_speed,speed1,speed2,speed3,speed4);
	
	differ_speed1=goal_1-speed1;
	differ_speed2=goal_2-speed2;
	differ_speed3=goal_3-speed3;
	differ_speed4=goal_4-speed4;
	
	dif_speed_sum1+=differ_speed1;
	dif_speed_sum2+=differ_speed2;
	dif_speed_sum3+=differ_speed3;
	dif_speed_sum4+=differ_speed4;
	if(dif_speed_sum1>10000||dif_speed_sum1<-10000)
		dif_speed_sum1=0;
	if(dif_speed_sum2>10000||dif_speed_sum2<-10000)
		dif_speed_sum2=0;
	if(dif_speed_sum3>10000||dif_speed_sum3<-10000)
		dif_speed_sum3=0;
	if(dif_speed_sum4>10000||dif_speed_sum4<-10000)
		dif_speed_sum4=0;
	pwm_out1+=kp1*differ_speed1+ki1*dif_speed_sum1+kd1*(differ_speed1-2*dif_speed_last1+dif_speed_last1_2);
	pwm_out2+=kp2*differ_speed2+ki2*dif_speed_sum2+kd2*(differ_speed2-2*dif_speed_last2+dif_speed_last2_2);
	pwm_out3+=kp3*differ_speed3+ki3*dif_speed_sum3+kd3*(differ_speed3-2*dif_speed_last3+dif_speed_last3_2);
	pwm_out4+=kp4*differ_speed4+ki4*dif_speed_sum4+kd4*(differ_speed4-2*dif_speed_last4+dif_speed_last4_2);
	
	dif_speed_last1_2=dif_speed_last1;
	dif_speed_last2_2=dif_speed_last2;
	dif_speed_last3_2=dif_speed_last3;
	dif_speed_last4_2=dif_speed_last4;
	
	dif_speed_last1=differ_speed1;
	dif_speed_last2=differ_speed2;
	dif_speed_last3=differ_speed3;
	dif_speed_last4=differ_speed4;
	
	speed_control(1,pwm_out1*go1);speed_control(2,pwm_out2*go2);speed_control(3,pwm_out3*go3);speed_control(4,pwm_out4*go4);
}









