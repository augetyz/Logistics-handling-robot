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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define servopower 2000
#define pi 3.1415926535898
#define DEC (pi/180)
    
    typedef struct
{
    int Car_speed[4];
    int goal_speed[4];
    uint16_t ditance_x;
    uint16_t ditance_y;
    uint8_t task;
    float kp[4], ki[4], kd[4];
} Car_status;
typedef struct
{
    float IMU[3];
} imu;
typedef struct
{
    int date[4];
} biu;
typedef struct
{
    int16_t date[4];
} biu_int_16;
typedef struct
{
    float kp[4], ki[4], kd[4];
} PID;
typedef struct
{
    uint16_t IDR_X;
    uint16_t IDR_Y;
} IDR_date;
typedef enum
{
    Start,              // 0 - 开始
    Start_Calibration,  // 1 - 开始后校准
    To_QR_Code,         // 2 - 二维码
    At_QR_Code,         // 3 - 处于二维码扫描区
    QR_Code_turn,       // 4 - 执行转向
    To_RMA,             // 5 - 抬起1 Raw material area
    At_RMA,             // 6 - 处于抬起1
    At_RMA1,            // 7
    Do_capture,         // 8
    Take_color_thing,   // 9
    To_Turn_1,          // 10 - 转向1
    At_Turn_1,          // 11 - 处于转向1
    To_Put_down_1,      // 12 - 放下1
    At_Put_down_1,      // 13 - 处于放下1
    At_Put_down_1_2,    // 14
    At_Put_down_1_3,    // 15
    To_Turn_2,          // 16 - 转向2
    At_Turn_2,          // 17 - 处于转向2
    To_Put_down_2,      // 18 - 放下2
    At_Put_down_2,      // 19 - 处于放下2
    At_Put_down_2_1,    // 20
    AT_Put_down_2_2,    // 21
    Back_Take_thing_1,  // 22
    Back_Take_thing_2,  // 23
    Back_Take_thing_3,  // 24
    Back_Take_thing_4,  // 25
    Do_capture_2,       // 26
    Take_color_thing_2, // 27
    Angle_direction_1,  // 28
    Angle_direction_2,  // 29
    Angle_direction_3,  // 30
    Stacking,           // 31
    Back_home_1,        // 32
    Back_home_2,        // 33
    Back_home_3,        // 34
    Stop,               // 35 - 停止
    Go_lengthways,      // 36 - 距离执行辅助任务
    Go_crosswise,       // 37
    target_location,    // 38
    End,                // 39 - 结束
    Color_cicle_location, // 40
    At_Put_down_1_1,    // 41
    At_Put_down_2_0,    // 42
    To_Put_down_3,      // 43
    AT_Put_down_2_3,    // 44
    At_Turn_2_angle,    // 45
    Take_color_thing_2_1,
    S_To_Turn_1,          // 
    S_At_Turn_1,          // 
    S_To_Put_down_1,      // 
    S_At_Put_down_1,      // 
    S_At_Put_down_1_2,    // 
    S_At_Put_down_1_3,    // 
    S_To_Turn_2,          // 
    S_At_Turn_2,          // 
    S_To_Put_down_2,      // 
    S_At_Put_down_2,      // 
    S_At_Put_down_2_1,    // 
    S_AT_Put_down_2_2,    // 
    S_To_Put_down_3,
    S_At_Turn_2_angle,
    S_At_Put_down_1_1,
    S_Angle_direction_2,
    Adjust_position,
    Small_moving,
    At_Put_down_1_angle,
    Debug,
} Location;
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
