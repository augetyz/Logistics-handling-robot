/*
 * DataFrames.c
 *
 *  Created on: Jul 17, 2023
 *      Author: 18277
 */

#include "DataFrames.h"

void MotorDataFrameSend(MotorDataFrame* DataFrame,UART_HandleTypeDef *huart)
{
	uint8_t Data[6]={0};
	uint8_t *Pointer=&DataFrame->Head;

	for (int i = 0; i < FrameSizeMotor; i++) {
		Data[i]=*Pointer;
		Pointer++;
	}

	HAL_UART_Transmit(&Transmit_USART, Data,FrameSizeMotor,1000);
}

void MotorWorkMissionSend(uint8_t MotorNum,short Step){
	MotorDataFrame Mission;

	Mission.Head=MotorDataType;
	Mission.ID=MotorMission;
	Mission.MotorNum=MotorNum;
	Mission.DataHeigh=Step>>8;
	Mission.DataLow=Step&0x00ff;
	Mission.Tail=FrameTail;

	MotorDataFrameSend(&Mission,&huart1);
}

void MotorMoveToSend(uint8_t MotorNum,uint16_t Position){
	MotorDataFrame Mission;

	Mission.Head=MotorDataType;
	Mission.ID=MotorMoveTo;
	Mission.MotorNum=MotorNum;
	Mission.DataHeigh=Position>>8;
	Mission.DataLow=Position&0x00ff;
	Mission.Tail=FrameTail;

	MotorDataFrameSend(&Mission,&huart1);	
}

void ServoMissionSend(uint16_t CCR){	
	MotorDataFrame Mission;

	Mission.Head=MotorDataType;
	Mission.ID=ServoMission;
	Mission.MotorNum=0x15;
	Mission.DataHeigh=CCR>>8;
	Mission.DataLow=CCR&0x00ff;
	Mission.Tail=FrameTail;

	MotorDataFrameSend(&Mission,&huart1);	
}

void MotorRecoordinateSend(void){
	MotorDataFrame Notification;

	Notification.Head=MotorDataType;
	Notification.ID=MotorRecoordinate;
	Notification.MotorNum=0x13;
	Notification.DataHeigh=DataNone;
	Notification.DataLow=DataNone;
	Notification.Tail=FrameTail;

	MotorDataFrameSend(&Notification,&huart1);
}

