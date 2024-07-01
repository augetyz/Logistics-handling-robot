#include "pid.h"

float   kp1=2.7,ki1=0.4,kd1=1.2;
float   kp2=2.7,ki2=0.5,kd2=1.2;
float   kp3=2.5,ki3=0.4,kd3=1.2;
float   kp4=3.0,ki4=0.4,kd4=0;
int goal_speed=300;//目标速度
int actual_speed;//实际速度
int dif_speed;//差值
u8 get_date[24];
u8 check_sum(u8 *addr)
{
  int biu;int sum=0;
  u8* biubiu=addr;
  for(biu=0;biu<24;biu++)
  {
  	sum+=biubiu[biu];
  	if(sum>0xff)
  	sum=sum&0xff;
  //	printf("%x\n",sum);
  }
  sum=sum+0x4e;
  if(sum>0xff)
  sum=sum&0xff;
  return sum;
}
u8 check_sum2(u8 *addr)
{
  int biu;int sum=0;
  u8* biubiu=addr;
  for(biu=0;biu<24;biu++)
  {
  	sum+=biubiu[biu];
  	if(sum>0xff)
  	sum=sum&0xff;
  //	printf("%x\n",sum);
  }
  if(sum>0xff)
  sum=sum&0xff;
  return sum;
}
void test_float_to_4hex(float num,u8* add)//float转化为四个字节的十六进制数组
{
	unsigned char tbuf[4];
	unsigned char *p = (unsigned char*)&num + 3;//指针p先指向float的最高字节

	//获取对应的4个字节，从低位到高位，这时就可以用于hex格式的数据传输了
	tbuf[0] = *(p-3);
	tbuf[1] = *(p-2);
	tbuf[2] = *(p-1);
	tbuf[3] = *p;
	*add=*tbuf;*(add+1)=*(tbuf+1);*(add+2)=*(tbuf+2);*(add+3)=*(tbuf+3);
}
void test_int_to_4hex(int num,u8* add)//float转化为四个字节的十六进制数组
{
	unsigned char tbuf[4];
	unsigned char *p = (unsigned char*)&num + 3;//指针p先指向float的最高字节

	//获取对应的4个字节，从低位到高位，这时就可以用于hex格式的数据传输了
	tbuf[0] = *(p-3);
	tbuf[1] = *(p-2);
	tbuf[2] = *(p-1);
	tbuf[3] = *p;
	*add=*tbuf;*(add+1)=*(tbuf+1);*(add+2)=*(tbuf+2);*(add+3)=*(tbuf+3);
}

float test_4hex_to_float(u8* add)//十六进制数组转化为对应float型量,低位在前
{
	float res;//验证float拆分为4个字节后，重组为float的结果

	//对拆分后的4个字节进行重组，模拟接收到hex后的数据还原过程
	unsigned char *pp = (unsigned char*)&res;
	pp[0] = add[0];
	pp[1] = add[1]; 
	pp[2] = add[2];
	pp[3] = add[3];
	return res;
}

//date:参数十六进制集合，num:参数数量
void pid_send(u8 *date)
{
	u8 i;
	Usart_SendHalf32Word(DEBUG_USARTx, 0X59485A53);//包头，低位在前
	Usart_SendByte(DEBUG_USARTx,date[0]);//发送通道
	switch(date[1])
	{
		case s_order_GOAL:
			Usart_SendHalf32Word(DEBUG_USARTx,(0x0b+4));//数据长度
		Usart_SendByte(DEBUG_USARTx,date[1]);//发送指令
			for(i=2;i<6;i++)	
			Usart_SendByte(DEBUG_USARTx,date[i]);break;
		case s_order_ACTUAL:
			Usart_SendHalf32Word(DEBUG_USARTx,(0x0b+4));
		Usart_SendByte(DEBUG_USARTx,date[1]);//发送指令
			for(i=2;i<6;i++)
			Usart_SendByte(DEBUG_USARTx,date[i]);break;
		case s_order_PID:
			Usart_SendHalf32Word(DEBUG_USARTx,(0x0b+12));
		Usart_SendByte(DEBUG_USARTx,date[1]);//发送指令
			for(i=2;i<14;i++)
			Usart_SendByte(DEBUG_USARTx,date[i]);break;
		case s_order_PERIOD:
			Usart_SendHalf32Word(DEBUG_USARTx,(0x0b+4));
		Usart_SendByte(DEBUG_USARTx,date[1]);//发送指令
			for(i=2;i<6;i++)
			Usart_SendByte(DEBUG_USARTx,date[i]);break;
	}
	Usart_SendByte(DEBUG_USARTx,check_sum(date));//校验和
}

//CHx:通道选择；*date:数据集合数组;/数组中仅存放：通道值，指令值，参数
//type:指令
void date_product(u8 CHx,u8 *date,u8 order)
{
	int speed;
	int i;
	u8 fbiu[4];
	date[0]=CHx;
	date[1]=order;
	switch(order)
	{
		case s_order_GOAL:
			speed=goal_speed;
			test_int_to_4hex(speed,fbiu);		
			for(i=0;i<4;i++)
			date[i+2]=fbiu[i];
			break;
		
		case s_order_ACTUAL:
			speed=actual_speed;			
			test_int_to_4hex(speed,fbiu);		
			for(i=0;i<4;i++)
			date[i+2]=fbiu[i];
			break;
		
		case s_order_PID:
			
			test_float_to_4hex(kp1,fbiu);
			for(i=0;i<4;i++)
			date[i+2]=fbiu[i];
		
			test_float_to_4hex(ki1,fbiu);
			for(i=0;i<4;i++)
			date[i+6]=fbiu[i];
		
			test_float_to_4hex(kd1,fbiu);
			for(i=0;i<4;i++)
			date[i+10]=fbiu[i];
		
			break;
		
		case s_order_PERIOD:
			
			break;
	}
}

//CHx:通道选择
//order：根据指令order，发送不同参数
void Togo(u8 CHx,u8 order)
{
	u8 date[24];
	date_product(CHx,date,order);
	pid_send(date);
}


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









