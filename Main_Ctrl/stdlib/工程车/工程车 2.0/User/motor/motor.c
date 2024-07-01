#include "motor.h"

void speed_control(int lunzi,int v)
{
	int go=1;
	if(v<0)
		v=-v,go=0;
	switch(lunzi)
	{
		case 1:
			if(go)
				TIM_SetCompare1(TIM8,v),TIM_SetCompare2(TIM8,1);
			else
				TIM_SetCompare2(TIM8,v),TIM_SetCompare1(TIM8,1);break;
		case 2:
			if(go)
				TIM_SetCompare4(TIM8,v),TIM_SetCompare3(TIM8,1);
			else
				TIM_SetCompare3(TIM8,v),TIM_SetCompare4(TIM8,1);break;
		case 3:
			if(go)
				TIM_SetCompare2(TIM3,v),TIM_SetCompare1(TIM3,1);
			else
				TIM_SetCompare1(TIM3,v),TIM_SetCompare2(TIM3,1);break;
		case 4:	
			if(go)
				TIM_SetCompare3(TIM3,v),TIM_SetCompare4(TIM3,1);
			else
				TIM_SetCompare4(TIM3,v),TIM_SetCompare3(TIM3,1);break;
	}
	
	
}

