#include "arm.h"

u8 check_sum(u8 *num)
{
	u16 check=0;
	u8 i;
	for(i=0;i<11;i++)
	check+=num[i];
	check=check%256;
	return (u8)check;
}
void control_arm(u8 id,int16_t angle,u16 speed)
{
	u8 num[12];u8 order=0x08;int i;
	num[0]=0x12,num[1]=0x4c;
	num[2]=order;
	num[3]=0x07;
	num[4]=id;
	num[5]=angle&0xff;
	num[6]=angle>>8&0xff;
	num[7]=speed&0xff;
	num[8]=speed>>8&0xff;
	num[9]=num[10]=0;
	num[11]=check_sum(num);
	for(i=0;i<=11;i++)
	{
		Usart_SendByte(USART2,num[i]);
	}
}




