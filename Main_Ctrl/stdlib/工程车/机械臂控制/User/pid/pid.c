#include "pid.h"

float   kp1=2.7,ki1=0.4,kd1=1.2;
float   kp2=2.7,ki2=0.5,kd2=1.2;
float   kp3=2.5,ki3=0.4,kd3=1.2;
float   kp4=3.0,ki4=0.4,kd4=0;
int goal_speed=300;//目标速度
int actual_speed;//实际速度
int dif_speed;//差值

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
	int pwm_out1,pwm_out2,pwm_out3,pwm_out4;int speed1,speed2,speed3,speed4;
	int differ_speed1,differ_speed2,differ_speed3,differ_speed4;
	static int dif_speed_sum1;static int dif_speed_last1;
	static int dif_speed_sum2;static int dif_speed_last2;
	static int dif_speed_sum3;static int dif_speed_last3;
	static int dif_speed_sum4;static int dif_speed_last4;
	if(goal_1<0)
		go1=-1,goal_1=-goal_1;
	if(goal_2<0)
		go2=-1,goal_2=-goal_2;
	if(goal_3<0)
		go3=-1,goal_3=-goal_3;
	if(goal_4<0)
		go4=-1,goal_4=-goal_4;
	speed1=Get_Motor_Speed(1);speed2=Get_Motor_Speed(2);speed3=Get_Motor_Speed(3);speed4=Get_Motor_Speed(4);
	printf("%d,%d,%d,%d\n",speed1,speed2,speed3,speed4);
	differ_speed1=goal_1-speed1;
	differ_speed2=goal_2-speed2;
	differ_speed3=goal_3-speed3;
	differ_speed4=goal_4-speed4;
	
	dif_speed_sum1+=differ_speed1;
	dif_speed_sum2+=differ_speed2;
	dif_speed_sum3+=differ_speed3;
	dif_speed_sum4+=differ_speed4;
	
	pwm_out1=goal_1+kp1*differ_speed1+ki1*dif_speed_sum1+kd1*(differ_speed1-dif_speed_last1);
	pwm_out2=goal_2+kp2*differ_speed2+ki2*dif_speed_sum2+kd2*(differ_speed2-dif_speed_last2);
	pwm_out3=goal_3+kp3*differ_speed3+ki3*dif_speed_sum3+kd3*(differ_speed3-dif_speed_last3);
	pwm_out4=goal_4+kp4*differ_speed4+ki4*dif_speed_sum4+kd4*(differ_speed4-dif_speed_last4);
	
	dif_speed_last1=differ_speed1;
	dif_speed_last2=differ_speed2;
	dif_speed_last3=differ_speed3;
	dif_speed_last4=differ_speed4;
	
	speed_control(1,pwm_out1*go1);speed_control(2,pwm_out2*go2);speed_control(3,pwm_out3*go3);speed_control(4,pwm_out4*go4);
}









