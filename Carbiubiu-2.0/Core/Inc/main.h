/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
  int Car_speed[4];
  int goal_speed[4];
  uint16_t ditance_x;
  uint16_t ditance_y;
  uint8_t task;
  float kp[4],ki[4],kd[4];
} Car_status;
typedef struct
{
    float IMU[3];
}imu;
typedef struct
{
    int date[4];
}biu;
typedef struct
{
    int16_t date[4];
}biu_int_16;
typedef struct
{
    float kp[4],ki[4],kd[4];
}PID;
typedef enum {
        Start,
        Start_Calibration,  // 开始后校准
        To_QR_Code,           // 二维码  
        At_QR_Code, // 处于二维码扫描区
        To_RMA,         // 抬起1 Raw material area
        At_RMA, // 处于抬起1
        Do_capture,
        Take_color_thing,
        To_Turn_1,         // 转向1
        At_Turn_1, // 处于转向1
        At_Turn_1_Find_Line, // 处于转向1并寻找黑线
        To_Put_down_1,    // 放下1
        At_Put_down_1, // 处于放下1
        At_Put_down_1_2,
        At_Put_down_1_3,
        To_Turn_2,        // 转向2
        At_Turn_2, // 处于转向2
        To_Put_down_2,   // 放下2
        At_Put_down_2, // 处于放下2
        Stop,          // 停止
        Go_lengthways,           //距离执行辅助任务
        Go_crosswise,
        End,           // 结束

    }Location;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
