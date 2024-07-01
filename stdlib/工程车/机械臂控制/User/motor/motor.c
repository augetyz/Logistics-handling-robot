#include "motor.h"

void speed_control(int lunzi,int v)
{
	int go=1;
	v=(v>999)?999:((v<-999)?-999:v);
	if(v<0)
		go=0,v=-v;
	switch(lunzi)
	{
		case 1:
			if(go)
				TIM_SetCompare1(TIM8,v),TIM_SetCompare2(TIM8,0);
			else
				TIM_SetCompare2(TIM8,v),TIM_SetCompare1(TIM8,0);break;
		case 2:
			if(go)
				TIM_SetCompare4(TIM8,v),TIM_SetCompare3(TIM8,0);
			else
				TIM_SetCompare3(TIM8,v),TIM_SetCompare4(TIM8,0);break;
		case 3:			
			if(go)
				TIM_SetCompare3(TIM3,v),TIM_SetCompare4(TIM3,0);
			else
				TIM_SetCompare4(TIM3,v),TIM_SetCompare3(TIM3,0);break;
		case 4:	
			if(go)
				TIM_SetCompare2(TIM3,v),TIM_SetCompare1(TIM3,0);
			else
				TIM_SetCompare1(TIM3,v),TIM_SetCompare2(TIM3,0);break;
	}
	
	
}

