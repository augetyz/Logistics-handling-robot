/*
 * DataFrames.h
 *
 *  Created on: Jul 17, 2023
 *      Author: 18277
 */

#ifndef INC_DATAFRAMES_H_
#define INC_DATAFRAMES_H_

#include "main.h"
#include "usart.h"

#define Transmit_USART huart4

/*电机数据帧结构体*/
typedef struct{
	uint8_t Head;
	uint8_t ID;
	uint8_t MotorNum;
	uint8_t DataHeigh;
	uint8_t DataLow;
	uint8_t Tail;
}MotorDataFrame;

/*通用数据帧定义*/
#define FrameTail 					0XFF//通用数据帧尾
#define DataTransmit 				0XDD//数据传输
#define DataTransmitOver			0XF0//数据传输结束
#define DataRequest					0XD0//数据请求
#define DataNone					0xDF//空数据（无意义数据）
#define FrameSize					0X05//通用数据帧大小

/*步进电机控制数据帧定义*/
#define FrameSizeMotor					0X06//数据帧大小
#define MotorDataType					0x60//电机数据类型帧头
#define MotorMissionOver				0x61//电机当前步骤执行完毕
#define MotorMission					0x62//电机任务数据ID
#define MotorMoveTo						0x63//移动到
#define MotorLimition					0x64//电机限位通知
#define MotorRecoordinate				0X67//电机重新定位

#define ServoMission					0x65//舵机任务

#define MotorA 0X11	//底部电机
#define MotorB 0x12	//垂直丝杆电机
#define MotorC 0x13	//水平丝杆电机
#define MotorD 0x14	//不存在

/*
 * 函数声明
 * */
void MotorDataFrameSend(MotorDataFrame* DataFrame,UART_HandleTypeDef *huart);

void MotorWorkMissionSend(uint8_t MotorNum,short Step);//增量式任务发送
void MotorRecoordinateSend(void);//重定位任务发送
void ServoMissionSend(uint16_t CCR);//舵机任务发送
void MotorMoveToSend(uint8_t MotorNum,uint16_t Position);//位置式任务发送

#endif /* INC_DATAFRAMES_H_ */
