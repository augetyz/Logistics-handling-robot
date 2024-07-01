#include "pid.h"

float   kp1=0.9,ki1=0.01,kd1=0;//红
float   kp2=0.9,ki2=0.02,kd2=0;//绿
float   kp3=0.9,ki3=0,kd3=0;//蓝
float   kp4=0.9,ki4=0,kd4=0;  //黄
extern int goal1, 
	goal2, 
	goal3, 
	goal4;
int x0=0,
	y0=0;
uint8_t goal;
void motor_control(int goal_1,int goal_2,int goal_3,int goal_4)
{
	int go1=1,go2=1,go3=1,go4=1;
	static int pwm_out1,pwm_out2,pwm_out3,pwm_out4;
	extern int speed1,speed2,speed3,speed4;
	int differ_speed1,differ_speed2,differ_speed3,differ_speed4;
	static int dif_speed_sum1;static int dif_speed_last1,dif_speed_lastlast1;
	static int dif_speed_sum2;static int dif_speed_last2,dif_speed_lastlast2;
	static int dif_speed_sum3;static int dif_speed_last3,dif_speed_lastlast3;
	static int dif_speed_sum4;static int dif_speed_last4,dif_speed_lastlast4;
	speed1=speed1/10;speed2=speed2/10;speed3=speed3/10;speed4=speed4/10;
	printf("%d,%d,%d,%d,%d,%d,%d,%d\n",speed1,speed2,speed3,speed4,goal_1,goal_2,goal_3,goal_4);
	differ_speed1=goal_1-speed1;
	differ_speed2=goal_2-speed2;
	differ_speed3=goal_3-speed3;
	differ_speed4=goal_4-speed4;
	
	dif_speed_sum1+=differ_speed1;
	dif_speed_sum2+=differ_speed2;
	dif_speed_sum3+=differ_speed3;
	dif_speed_sum4+=differ_speed4;
	
	pwm_out1+=kp1*differ_speed1+ki1*dif_speed_sum1+kd1*(differ_speed1-2*dif_speed_last1+dif_speed_lastlast1);
	pwm_out2+=kp2*differ_speed2+ki2*dif_speed_sum2+kd2*(differ_speed2-2*dif_speed_last2+dif_speed_lastlast2);
	pwm_out3+=kp3*differ_speed3+ki3*dif_speed_sum3+kd3*(differ_speed3-2*dif_speed_last3+dif_speed_lastlast3);
	pwm_out4+=kp4*differ_speed4+ki4*dif_speed_sum4+kd4*(differ_speed4-2*dif_speed_last4+dif_speed_lastlast4);
	
	dif_speed_lastlast1=dif_speed_last1;
	dif_speed_lastlast2=dif_speed_last2;
	dif_speed_lastlast3=dif_speed_last3;
	dif_speed_lastlast4=dif_speed_last4;
	
	dif_speed_last1=differ_speed1;
	dif_speed_last2=differ_speed2;
	dif_speed_last3=differ_speed3;
	dif_speed_last4=differ_speed4;
	
	speed_control(1,pwm_out1*go1);speed_control(2,pwm_out2*go2);speed_control(3,pwm_out3*go3);speed_control(4,pwm_out4*go4);
}

void go(float x,float y,int speed)
{
	float dx,dy;float k;
	extern int goal1,goal2,goal3,goal4;
	dx=x-x0;dy=y-y0;
	k=(dy-dx)/dy;
	goal1=goal4=speed;
	goal2=goal3=k*speed;
	//motor_control(goal1,goal2,goal3,goal4);
}
uint32_t KalmanFilter(int32_t ResrcData)
{
    /*-------------------------------------------------------------------------------------------------------------*/
    /*
            Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
            R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
    */
    /*-------------------------------------------------------------------------------------------------------------*/
    static int32_t R = (int32_t)(128*1024);
    static int32_t Q = (int32_t)4;
    static uint32_t Counter1 = 0;
    static uint32_t Counter2 = 0;
    static int32_t x_last = 0;
        static int32_t p_last;   // 应赋初始估计值
    int32_t x_mid;
    int32_t x_now;
    int32_t p_mid ;
    int32_t p_now;
 
    ResrcData *= 1024;
    x_now = ResrcData - x_last;
    if(x_now < 0)
    {
        x_now *= -1; // 取绝对值
    }
    if(x_now >= 32*1024)   // 如果测量值连续比估计值大或小 相信测量值，加速迭代
    {
        Counter1++;
        Counter2 = 0;
        if(Counter1 > 10)
        {
            R = 512;;
            Q = 128;
        }
    }
    else                 // 数据比较稳定，加强滤波 
    {
        Counter1 = 0;
        Counter2++;
        if(Counter2 > 10)  
        {
            R = (int32_t)(128*1024);
            Q = (int32_t)4;
        }
    }
    x_mid = x_last;   // x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = p_last + Q; // p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
//    kg = p_mid/(p_mid + R); //kg为kalman filter，R为噪声
//    x_now = x_mid+kg*(ResrcData - x_mid);// 估计出的最优值
    x_now = x_mid + (p_mid*(ResrcData - x_mid))/(p_mid + R);
//    p_now = (1 - kg)*p_mid; // 最优值对应的covariance
    p_now = p_mid - p_mid*p_mid/(p_mid + R); // 最优值对应的covariance
    p_last = p_now;  // 更新covariance值
    x_last = x_now;  // 更新系统状态值
    x_now /= 1024;
    if((x_now > 4096)||( x_now < 0))
    {
        x_last = ResrcData;
        p_now = ResrcData;
        x_now = ResrcData/1024;
    }
    return (u32)x_now;
}
void pidsend(void)
{
	switch(goal)
	{
		case 1:printf("%f,%f,%f\n",kp1,ki1,kd1);break;
		case 2:printf("%f,%f,%f\n",kp2,ki2,kd2);break;
		case 3:printf("%f,%f,%f\n",kp3,ki3,kd3);break;
		case 4:printf("%f,%f,%f\n",kp4,ki4,kd4);break;
	}
	
}
void date_deal(u8 *p)
{
	char order=p[0];
	int v;float k,t=1;uint8_t n,m;
	for(n=0;p[n]!=0x0a;n++);
	switch(order)
	{
		case 'G':
			if(p[2]==0x30)
				switch(goal)
				{
					case 1:goal1=0;break;
					case 2:goal2=0;break;
					case 3:goal3=0;break;
					case 4:goal4=0;break;
				}
			else
				if(p[2]==0x31)
					switch(goal)
					{
						case 1:goal1=30;break;
						case 2:goal2=-30;break;
						case 3:goal3=30;break;
						case 4:goal4=-30;break;
					}
			PAout(11)=0;pidsend();break;
			
		case 'L':
			goal=p[2]-0x30;
			PAout(11)=0;pidsend();break;
			
		case 'v':
			if(p[2]==0x2d)
				v=-(int)((p[3]-0x30)*10+(p[4]-0x30));
			else
				v=(p[2]-0x30)*10+(p[3]-0x30);
			switch(goal)
				{
					case 1:goal1=v;break;
					case 2:goal2=v;break;
					case 3:goal3=v;break;
					case 4:goal4=v;break;
				}
			PAout(11)=0;pidsend();break;
			
		case 'p':
			for(n=n-1;n>=4;n--)
				{
					for(m=n;m>=4;m--)
						t*=0.1;
					k+=t*(int)(p[n]-0x30);
				}
			k+=(int)(p[3]-0x30);
			switch(goal)
			{
				case 1:kp1=k;break;
				case 2:kp2=k;break;
				case 3:kp3=k;break;
				case 4:kp4=k;break;
			}
			PAout(11)=0;pidsend();break;
			
		case 'i':
			for(n=n-1;n>=4;n--)
				{
					for(m=n;m>=4;m--)
						t*=0.1;
					k+=t*(int)(p[n]-0x30);
				}
			k+=(int)(p[3]-0x30);
			switch(goal)
			{
				case 1:ki1=k;break;
				case 2:ki2=k;break;
				case 3:ki3=k;break;
				case 4:ki4=k;break;
			}
			PAout(11)=0;pidsend();break;
			
		case 'd':
			for(n=n-1;n>=4;n--)
				{
					for(m=n;m>=4;m--)
						t*=0.1;
					k+=t*(int)(p[n]-0x30);
				}
			k+=(int)(p[3]-0x30);
			switch(goal)
			{
				case 1:kd1=k;break;
				case 2:kd2=k;break;
				case 3:kd3=k;break;
				case 4:kd4=k;break;
			}
			PAout(11)=0;pidsend();break;
			
		default:PAout(11)=1;break;
	}
	
}





