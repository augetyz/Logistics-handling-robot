/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "soft_user.h"
#include "string.h"
#include "JY61.h"
#include "servo.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor_ctrl.h"
#include "pid.h"
#include "HI229.h"
#include <stdlib.h>
#include <math.h>
#include "DataFrames.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Led_Toggle GPIOC->ODR ^= GPIO_PIN_13 // LED_Toggle函数宏定义
#define IMU_speed 80                         // IMU数据采集处理频率控制参数
// #define JY61
#define turn_angle 0f

#define UNUSED_VARIABLE(X) ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)
#define symbol(y) (y >= 0 ? 1 : -1)

#define IDR_color_grab_x 1732 // 原料区定位
#define IDR_color_grab_y 1956

#define IDR_color_put_x 1890 // 1890 // 粗加工区定位
// #define IDR_color_put_y 1175
#define IDR_color_put_y 1099 // 1100

#define IDR_color_put2_x 1743 // 暂存区定位
#define IDR_color_put2_y 1112 // 1112

#define IDR_color_home_x 50 // 启停区定位
#define IDR_color_home_y 53

#define IMU_excursion -1.15f // 十字激光初始化陀螺仪后，对陀螺仪数据的偏移值

#define color_circle_x 0   // 面向暂存区， 
#define color_circle_y 0  // 面向粗加工区，暂存区为-，原料区为+
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern char s_cDataUpdate, s_cCmd; // MPU6050移植参数，不用管

extern servo_status servo_motion[];



uint8_t OS_status = 0; // 任务初始化标志位，为1则初始化OK，为0则还没有嘞

uint8_t debug_date[600] = {0}; // debug数据缓存区，用于缓存需要用来上传的debug数据。

uint8_t IMU_date[IMU_speed] = {0}; // MPU6050数据缓冲区、采用DMA+串口方式接收

uint8_t usart_3_date[25] = {0};
uint8_t usart_6_date[33] = {0};

uint8_t servo_sign; // 1表示上次动作组已经执行完毕， 0表示上次动作组正在执行

uint16_t modetime = 100;



raw_t raw = {0};     /* IMU stram read/control struct */
uint8_t decode_succ; /* 0: no new frame arrived, 1: new frame arrived */
uint8_t Task_verify = Start;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
SemaphoreHandle_t IMUdate_RX_Sem_Handle = NULL; // 信号量初始化，用于作为IMU数据接收完成后的回响，开启IMU_TASK函数的执行。

SemaphoreHandle_t usart3_RX_Sem_Handle = NULL;

SemaphoreHandle_t usart6_RX_Sem_Handle = NULL;

SemaphoreHandle_t Servo_Sem_Handle = NULL;

SemaphoreHandle_t Distance_Sem_Handle = NULL;

QueueHandle_t Speed_Debug_Queue = NULL;
QueueHandle_t Speed_Queue = NULL;
QueueHandle_t goal_Queue = NULL;
QueueHandle_t IMU_Queue = NULL;
QueueHandle_t debug_Queue = NULL;
QueueHandle_t angle_Queue = NULL;
QueueHandle_t servo_Queue = NULL;
QueueHandle_t servo_RX_Queue = NULL;
QueueHandle_t distance_Queue = NULL;
QueueHandle_t color_Queue = NULL;
QueueHandle_t IDR_date_Queue = NULL;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask_keyHandle;
osThreadId myTask_ledHandle;
osThreadId myTask_IMUHandle;
osThreadId myTask_debugHandle;
osThreadId debug_getHandle;
osThreadId myTask_usartHandle;
osThreadId myTask_oledHandle;
osThreadId myTask_servoHandle;
osThreadId myTask_speedHandle;
osThreadId myTask_pidHandle;
osThreadId myTask_doingHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void key_Task(void const *argument);       // 一键启动与一键调试
void led_Task(void const *argument);       // LED闪烁
void IMU_Task(void const *argument);       // 获取IMU数据
void deubg_Task(void const *argument);     // 一键调试模式
void debug_get_Task(void const *argument); // 未实现
void usart_Task(void const *argument);     // 获取激光数据与树莓派数据
void oled_Task(void const *argument);      // 未实现
void servo_Task(void const *argument);     // 控制机械臂动作组
void speed_Task(void const *argument);     // 获取四个轮子编码器的值，并统计行走距离
void pid_Task(void const *argument);       // 负责每个轮子的闭环控制
void doing_Task(void const *argument);     // 主控任务

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);
void Arm_CTL(uint16_t MOTORA_step,uint16_t MOTORB_step,uint16_t MOTORC_step,uint16_t Servo);
/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    // IMUdate_RX_Sem_Handle二值信号量创建
    IMUdate_RX_Sem_Handle = xSemaphoreCreateBinary();

    usart3_RX_Sem_Handle = xSemaphoreCreateBinary();

    usart6_RX_Sem_Handle = xSemaphoreCreateBinary();

    Servo_Sem_Handle = xSemaphoreCreateBinary();
    
    servo_RX_Queue = xSemaphoreCreateBinary();

    Distance_Sem_Handle = xSemaphoreCreateBinary();
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    Speed_Debug_Queue = xQueueCreate((UBaseType_t)1, /* 消息队列的长度 */ // 实时速度参数传递消息队列
                               (UBaseType_t)sizeof(biu));           /* 消息的大小 */
    Speed_Queue = xQueueCreate((UBaseType_t)1, /* 消息队列的长度 */ // 实时速度参数传递消息队列
                               (UBaseType_t)sizeof(biu));           /* 消息的大小 */
    goal_Queue = xQueueCreate((UBaseType_t)2, /* 消息队列的长度 */  // 目标速度传递消息队列
                              (UBaseType_t)sizeof(biu));            /* 消息的大小 */
    IMU_Queue = xQueueCreate((UBaseType_t)1,                        /* 消息队列的长度 */
                             (UBaseType_t)sizeof(imu));             /* 消息的大小 */
    debug_Queue = xQueueCreate((UBaseType_t)1,                      /* 消息队列的长度 */
                               (UBaseType_t)sizeof(biu));           /* 消息的大小 */
    angle_Queue = xQueueCreate((UBaseType_t)1,                      /* 消息队列的长度 */
                               (UBaseType_t)sizeof(imu));           /* 消息的大小 */
    servo_Queue = xQueueCreate((UBaseType_t)1,                      /* 消息队列的长度 */
                               (UBaseType_t)sizeof(servo_status));  /* 消息的大小 */
    distance_Queue = xQueueCreate((UBaseType_t)1,                   /* 消息队列的长度 */
                                  (UBaseType_t)sizeof(biu));        /* 消息的大小 */
    color_Queue = xQueueCreate((UBaseType_t)1,                      /* 消息队列的长度 */
                               (UBaseType_t)sizeof(biu_int_16));    /* 消息的大小 */
    IDR_date_Queue = xQueueCreate((UBaseType_t)1,                   /* 消息队列的长度 */
                                  (UBaseType_t)sizeof(IDR_date));   /* 消息的大小 */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of myTask_key */
    osThreadDef(myTask_key, key_Task, osPriorityAboveNormal, 0, 128);
    myTask_keyHandle = osThreadCreate(osThread(myTask_key), NULL);

    /* definition and creation of myTask_led */
    osThreadDef(myTask_led, led_Task, osPriorityIdle, 0, 128);
    myTask_ledHandle = osThreadCreate(osThread(myTask_led), NULL);

    /* definition and creation of myTask_IMU */
    osThreadDef(myTask_IMU, IMU_Task, osPriorityNormal, 0, 256);
    myTask_IMUHandle = osThreadCreate(osThread(myTask_IMU), NULL);

    /* definition and creation of myTask_debug */
    osThreadDef(myTask_debug, deubg_Task, osPriorityLow, 0, 256);
    myTask_debugHandle = osThreadCreate(osThread(myTask_debug), NULL);

    /* definition and creation of debug_get */
    osThreadDef(debug_get, debug_get_Task, osPriorityAboveNormal, 0, 128);
    debug_getHandle = osThreadCreate(osThread(debug_get), NULL);

    /* definition and creation of myTask_usart */
    osThreadDef(myTask_usart, usart_Task, osPriorityNormal, 0, 128);
    myTask_usartHandle = osThreadCreate(osThread(myTask_usart), NULL);

    /* definition and creation of myTask_oled */
    osThreadDef(myTask_oled, oled_Task, osPriorityLow, 0, 128);
    myTask_oledHandle = osThreadCreate(osThread(myTask_oled), NULL);

    /* definition and creation of myTask_servo */
    osThreadDef(myTask_servo, servo_Task, osPriorityHigh, 0, 128);
    myTask_servoHandle = osThreadCreate(osThread(myTask_servo), NULL);

    /* definition and creation of myTask_speed */
    osThreadDef(myTask_speed, speed_Task, osPriorityHigh, 0, 128);
    myTask_speedHandle = osThreadCreate(osThread(myTask_speed), NULL);

    /* definition and creation of myTask_pid */
    osThreadDef(myTask_pid, pid_Task, osPriorityHigh, 0, 512);
    myTask_pidHandle = osThreadCreate(osThread(myTask_pid), NULL);

    /* definition and creation of myTask_doing */
    osThreadDef(myTask_doing, doing_Task, osPriorityHigh, 0, 512);
    myTask_doingHandle = osThreadCreate(osThread(myTask_doing), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  优先级最低的任务，点个灯在里面，可以通过观察led闪烁情况观察任务运行情况，
 *          如果不稳定，则任务优先级、缓存分配存在问题
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        Led_Toggle;

        osDelay(100);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_key_Task */
/**
 * @brief 按键任务，用于检测按键触发，没有使用中断方式（不稳定而且占用资源），这里使用轮询方式依然很nice
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_key_Task */
void key_Task(void const *argument)
{
    /* USER CODE BEGIN key_Task */
    /* Infinite loop */
    char IMU_AT_date[10];
    sprintf(IMU_AT_date, "AT+RST\n");
    for (;;)
    {
        // if ((GPIOC->IDR & (1 << 0)) == 0)
        // {
        //     osDelay(10);
        //     if ((GPIOC->IDR & (1 << 0)) == 0)
        //     {
        //         while ((GPIOC->IDR & (1 << 0)) == 0)
        //         {
        //             osDelay(1);
        //         }
        //         /*do something*/

        //         vTaskSuspend(myTask_doingHandle); // 进入手掰模式
        //         osDelay(10);
        //         vTaskResume(myTask_debugHandle);
        //         FSUS_DampingMode(&usart4, 1, 0);
        //         osDelay(100);
        //         FSUS_DampingMode(&usart4, 2, 0);
        //         osDelay(100);
        //         FSUS_DampingMode(&usart4, 3, 0);
        //         osDelay(100);
        //         FSUS_DampingMode(&usart4, 4, 0);
        //         osDelay(100);
        //         FSUS_DampingMode(&usart4, 5, 0);
        //         osDelay(100);
        //     }
        // }
        // if ((GPIOC->IDR & (1 << 1)) == 0)
        // {
        //     osDelay(10);
        //     if ((GPIOC->IDR & (1 << 1)) == 0)
        //     {
        //         while ((GPIOC->IDR & (1 << 1)) == 0)
        //         {
        //             osDelay(1);
        //         }
        //         /*do something*/

        //         vTaskResume(myTask_doingHandle); // 进入工作模式
        //         vTaskSuspend(myTask_debugHandle);
        //         servo_all_move(servo_motion[1]);
        //     }
        // }

        if ((GPIOA->IDR & (1 << 15)) == 0)
        {
            osDelay(10);
            if ((GPIOA->IDR & (1 << 15)) == 0)
            {
                while ((GPIOA->IDR & (1 << 15)) == 0)
                {
                    osDelay(1);
                }
                /*do something*/
                //                HAL_UART_Transmit(&huart2, (uint8_t *)IMU_AT_date, strlen(IMU_AT_date), 100);

                // servo_all_move(servo_motion[0]);
            }
        }
        osDelay(1);
    }
    /* USER CODE END key_Task */
}

/* USER CODE BEGIN Header_led_Task */
/**
 * @brief RGB led任务，用于显示当前状态
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_led_Task */
void led_Task(void const *argument)
{
    /* USER CODE BEGIN led_Task */
    /* Infinite loop */
    for (;;)
    {
        GPIOC->ODR ^= GPIO_PIN_3;
        osDelay(50);
    }
    /* USER CODE END led_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
 * @brief IMU数据处理任务、优先级较高，但收到二值信息量控制，当二值信息量被释放后，此任务才可以执行，
 * 信息量释放速度受宏定义IMU_speed控制。
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IMU_Task */
void IMU_Task(void const *argument)
{
    /* USER CODE BEGIN IMU_Task */
    /* Infinite loop */
    int i;
    imu imu_car;
    OS_status = 1;
    HAL_UART_Receive_DMA(&huart2, IMU_date, IMU_speed);
    for (;;)
    {
        // 获取二值信号量 xSemaphore,没获取到则一直等待
        xSemaphoreTake(IMUdate_RX_Sem_Handle, portMAX_DELAY); /* 等待时间 */
        taskENTER_CRITICAL();
#ifdef JY61
        for (i = 0; i < IMU_speed; i++)
        {
            WitSerialDataIn(IMU_date[i]);
        }
        if (s_cDataUpdate)
        {
            for (i = 0; i < 3; i++)
            {
                imu_car.IMU[i] = sReg[Roll + i] / 32768.0f * 180.0f;
            }
            imu_car.IMU[2] += IMU_excursion;
            xQueueSend(IMU_Queue, &imu_car, 0);
            xQueueSend(angle_Queue, &imu_car, 0);
        }
#else
        for (i = 0; i < IMU_speed; i++)
        {
            decode_succ = ch_serial_input(&raw, IMU_date[i]);
            if (decode_succ == 1)
                break;
        }

        if (decode_succ)
        {
            for (i = 0; i < 3; i++)
            {
                imu_car.IMU[i] = raw.imu[0].eul[i];
            }
            imu_car.IMU[2] += IMU_excursion;
            xQueueSend(IMU_Queue, &imu_car, 0);
            xQueueSend(angle_Queue, &imu_car, 0);
        }
#endif

        HAL_UART_Receive_DMA(&huart2, IMU_date, IMU_speed);
        taskEXIT_CRITICAL();
    }
    /* USER CODE END IMU_Task */
}
/* USER CODE BEGIN Header_deubg_Task */
/**
 * @brief 小车状态上传任务，使用串口一DMA发送，运行周期1000ms
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_deubg_Task */
void deubg_Task(void const *argument)
{
    /* USER CODE BEGIN deubg_Task */
    /* Infinite loop */
    imu imu_car={0,0,0};
    biu speed_now;
    biu speed_goal;

    for (;;)
    {

        xQueueReceive(IMU_Queue, &imu_car, 10);
        xQueueReceive(debug_Queue, &speed_goal, 10);
        xQueueReceive(Speed_Debug_Queue, &speed_now, 10);



        taskENTER_CRITICAL();
         sprintf((char *)debug_date, "%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d\n",
                 imu_car.IMU[0], imu_car.IMU[1], imu_car.IMU[2],
                 speed_now.date[0], speed_goal.date[0], speed_now.date[1], speed_goal.date[1],
                 speed_now.date[2], speed_goal.date[2], speed_now.date[3], speed_goal.date[3]);
//        sprintf((char *)debug_date, "%.1f,%.1f,%.1f,%.1f,%.1f,%.2f\n", angle[0], angle[1], angle[2], angle[3], angle[4], imu_car.IMU[2]);
        HAL_UART_Transmit_DMA(&huart1, debug_date, strlen((char *)debug_date));
        taskEXIT_CRITICAL();

        osDelay(10);
    }
    /* USER CODE END deubg_Task */
}

/* USER CODE BEGIN Header_debug_get_Task */
/**
 * @brief Function implementing the debug_get thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_debug_get_Task */
void debug_get_Task(void const *argument)
{
    /* USER CODE BEGIN motor_Task */
    /* Infinite loop */
    for (;;)
    {
        osDelay(10);
    }
    /* USER CODE END motor_Task */
}

/* USER CODE BEGIN Header_usart_Task */
/**
 * @brief Function implementing the myTask_usart thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_usart_Task */
void usart_Task(void const *argument)
{
    /* USER CODE BEGIN usart_Task */
    /* Infinite loop */

    taskENTER_CRITICAL();

    int i = 0, num, sign = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t wide = 0;
    int16_t height = 0;

    char *f = (char *)&x;
    char *h = (char *)&y;
    char *u = (char *)&wide;
    char *q = (char *)&height;
    biu_int_16 color_date;
    HAL_UART_Receive_DMA(&huart3, usart_3_date, 22);
    HAL_UART_Receive_DMA(&huart6, usart_6_date, 16);

    taskEXIT_CRITICAL();

    for (;;)
    {
        // USART3
        if (xSemaphoreTake(usart3_RX_Sem_Handle, 10) == pdTRUE) /* 等待时间 */
        {
            vTaskSuspendAll();

            for (i = 0; i < 8; i++)
            {
                if (usart_3_date[i] == 0X2C && usart_3_date[i + 10] == 0X5B)
                {
                    num = i;
                    if (usart_3_date[num + 1] == 0X12)
                        sign = 1; // 当前获取到的数据是物块颜色
                    else if (usart_3_date[num + 1] == 0X21)
                        sign = 2; // 当前获取到的数据是二维码值
                    else if (usart_3_date[num + 1] == 0X3C)
                        sign = 3; // 当前获取到的数据是色环定位数据
                    else
                        sign = 0;
                    break;
                }
                sign = 0;
            }
            // HAL_UART_Transmit_DMA(&huart1, usart_3_date, 22);
            if (sign == 1) // 识别物块颜色
            {

                for (i = 0; i < 2; i++)
                {
                    *(f + i) = *(usart_3_date + num + i + 2);
                }
                //          i=*(f);*(f)=*(f+1);*(f+1)=i;

                for (i = 0; i < 2; i++)
                {
                    *(h + i) = *(usart_3_date + num + i + 4);
                }
                for (i = 0; i < 2; i++)
                {
                    *(u + i) = *(usart_3_date + num + i + 6);
                }
                for (i = 0; i < 2; i++)
                {
                    *(q + i) = *(usart_3_date + num + i + 8);
                }
                //          i=*(h);*(h)=*(h+1);*(h+1)=i;
                if ((x != 520 && y != 520) && (y != 9))
                {
                    color_date.date[0] = x;
                    color_date.date[1] = y;
                    color_date.date[2] = 11;
                    color_date.date[3] = height;
                    xQueueSend(color_Queue, &color_date, 0);
                }
                else
                {
                    // error
                }
            }
            else if (sign == 2) // 获取二维码值
            {
                usart_3_date[num + 6] = usart_3_date[num + 7];
                usart_3_date[num + 7] = usart_3_date[num + 8];
                usart_3_date[num + 8] = usart_3_date[num + 9];
                x = 0;
                for (i = 0; i < 6; i++)
                {
                    if (usart_3_date[num + 3 + i] == '1')
                        x |= 0X01 << (5 - i) * 2;
                    else if (usart_3_date[num + 3 + i] == '2')
                        x |= 0X02 << (5 - i) * 2;
                    else if (usart_3_date[num + 3 + i] == '3')
                        x |= 0X03 << (5 - i) * 2;
                    else
                    {
                        x = 0;
                        break;
                    }
                }
                if (x != 0)
                {
                    if ((x & 3 << 0) != ((x & 3 << 2) >> 2) && (x & 3 << 0) != ((x & 3 << 4) >> 4) && (x & 3 << 2) >> 2 != ((x & 3 << 4) >> 4))
                    {

                        color_date.date[3] = x;
                        color_date.date[2] = 22;
                        xQueueSend(color_Queue, &color_date, 0);
                        // printf("%d\n", x);
                    }
                    else
                    {
                        // error
                    }
                }
            }else if (sign == 3) // 色环定位数据
            {

                for (i = 0; i < 2; i++)
                {
                    *(f + i) = *(usart_3_date + num + i + 2);
                }
                //          i=*(f);*(f)=*(f+1);*(f+1)=i;

                for (i = 0; i < 2; i++)
                {
                    *(h + i) = *(usart_3_date + num + i + 4);
                }
                for (i = 0; i < 2; i++)
                {
                    *(u + i) = *(usart_3_date + num + i + 6);
                }
                for (i = 0; i < 2; i++)
                {
                    *(q + i) = *(usart_3_date + num + i + 8);
                }
                //          i=*(h);*(h)=*(h+1);*(h+1)=i;
                if ((x != 520 && y != 520) && (y != 9))
                {
                    color_date.date[0] = x;
                    color_date.date[1] = y;
                    color_date.date[2] = 33;
                    color_date.date[3] = 0;
                    xQueueSend(color_Queue, &color_date, 0);
                }
                else
                {
                    // error
                }
            }

            HAL_UART_Receive_DMA(&huart3, usart_3_date, 22);
            xTaskResumeAll();
        }

        // USART6
        if (xSemaphoreTake(usart6_RX_Sem_Handle, 10) == pdTRUE) /* 等待时间 */
        {
            vTaskSuspendAll();
            i = 0;
            
            for (i = 0; i < 24; i++)
            {
                if (usart_6_date[i] == 0X40 &&  usart_6_date[i+8] == 0X0D)
                {
                    sign=1;
                    break;
                }
                else
                    sign = 0;
            }
            num = i;
            if (sign == 1)
            {
                usart_6_date[num + 4] = usart_6_date[num + 5];
                usart_6_date[num + 5] = usart_6_date[num + 6];
                usart_6_date[num + 6] = usart_6_date[num + 7];
                x = 0;
                for (i = 0; i < 6; i++)
                {
                    if (usart_6_date[num + 1 + i] == '1')
                        x |= 0X01 << (5 - i) * 2;
                    else if (usart_6_date[num + 1 + i] == '2')
                        x |= 0X02 << (5 - i) * 2;
                    else if (usart_6_date[num + 1 + i] == '3')
                        x |= 0X03 << (5 - i) * 2;
                    else
                    {
                        x = 0;
                        break;
                    }
                }
                if (x != 0)
                {
                    if ((x & 3 << 0) != ((x & 3 << 2) >> 2) && (x & 3 << 0) != ((x & 3 << 4) >> 4) && (x & 3 << 2) >> 2 != ((x & 3 << 4) >> 4))
                    {

                        color_date.date[3] = x;
                        color_date.date[2] = 22;
                        xQueueSend(color_Queue, &color_date, 0);
                        // printf("%d\n", x);
                    }
                    else
                    {
                        // error
                    }
                }                 
            }         
            HAL_UART_Receive_DMA(&huart6, usart_6_date, 32);
            xTaskResumeAll();
        }
        osDelay(10);
    }
    /* USER CODE END usart_Task */
}

/* USER CODE BEGIN Header_oled_Task */
/**
 * @brief Function implementing the myTask_oled thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_oled_Task */
float adc_vlaue=0.0f;
void oled_Task(void const *argument)
{
    /* USER CODE BEGIN oled_Task */
    /* Infinite loop */
    uint32_t ADC_value=0;

    HAL_ADC_Start_DMA(&hadc1,&ADC_value,1);
    GPIOD->ODR|=GPIO_PIN_7;
    osDelay(1000);
    GPIOD->ODR&=~GPIO_PIN_7;
    for (;;)
    {
        adc_vlaue=(float)ADC_value*3.3f*11/4096.0f;
        Uart5_LCD_show_ADC(adc_vlaue);
        if(adc_vlaue<11.7f)
            GPIOD->ODR|=GPIO_PIN_7;
        else
            GPIOD->ODR&=~GPIO_PIN_7;
        osDelay(1000);
    }
    /* USER CODE END oled_Task */
}

/* USER CODE BEGIN Header_servo_Task */
/**
 * @brief Function implementing the myTask_servo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_servo_Task */
void servo_Task(void const *argument)
{
    /* USER CODE BEGIN servo_Task */
    /* Infinite loop */

    uint8_t Rx_buffer[10];
    uint8_t Tx_buffer[2]={0X60,0X61};
    HAL_UART_Receive_DMA(&huart4,Rx_buffer,4);
    for (;;)
    {
        if(xSemaphoreTake(servo_RX_Queue,10) == pdTRUE)
        {
            xSemaphoreGive(Servo_Sem_Handle);
            HAL_UART_Transmit(&huart4,Tx_buffer,2,100);
            osDelay(1000);
            HAL_UART_Receive_DMA(&huart4,Rx_buffer,4);
        }
        else           
            osDelay(10);
    }
    /* USER CODE END servo_Task */
}

/* USER CODE BEGIN Header_speed_Task */
/**
 * @brief Function implementing the myTask_speed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_speed_Task */
void speed_Task(void const *argument)
{
    /* USER CODE BEGIN speed_Task */
    /* Infinite loop */
    static biu speed = {0, 0, 0, 0};
    static biu distance = {0, 0, 0, 0};
    uint8_t i = 0;
    for (;;)
    {
        if (xSemaphoreTake(Distance_Sem_Handle, 1) == pdTRUE)
        {
            for (i = 0; i < 4; i++)
            {
                
                distance.date[i] = 0;
                
            }
        }
        vTaskSuspendAll();
        speed.date[0] = TIM2->CNT - 6720;
        TIM2->CNT = 6720;

        speed.date[1] = 6720 -TIM3->CNT;
        TIM3->CNT = 6720;

        speed.date[2] = TIM4->CNT - 6720;
        TIM4->CNT = 6720;

        speed.date[3] = 6720 - TIM5->CNT;
        TIM5->CNT = 6720;
        for (i = 0; i < 4; i++)
        {
            distance.date[i] += speed.date[i];
        }
        xTaskResumeAll();
        xQueueSend(Speed_Queue, &speed, 0); /* 等待时间 0 */
        xQueueSend(distance_Queue, &distance, 0);
        xQueueSend(Speed_Debug_Queue, &speed, 0);
        osDelay(10);
    }
    /* USER CODE END speed_Task */
}

/* USER CODE BEGIN Header_pid_Task */
/**
 * @brief Function implementing the myTask_pid thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_pid_Task */
void pid_Task(void const *argument)
{
    /* USER CODE BEGIN pid_Task */
    /* Infinite loop */
    biu speed_goal = {0};
    biu speed_now = {0};
    speed_ctrl(Motor1, 0);
    speed_ctrl(Motor2, 0);
    speed_ctrl(Motor3, 0);
    speed_ctrl(Motor4, 0);
    for (;;)
    {
        xQueueReceive(Speed_Queue, /* 消息队列的句柄 */
                      &speed_now,  /* 发送的消息内容 */
                      portMAX_DELAY);          /* 等待时间 一直等 */
        xQueueReceive(goal_Queue,  /* 消息队列的句柄 */
                      &speed_goal, /* 发送的消息内容 */
                      5);          /* 等待时间 一直等 */

        taskENTER_CRITICAL();

        pid_do(speed_goal, speed_now);

        taskEXIT_CRITICAL();

        osDelay(10);
    }
    /* USER CODE END pid_Task */
}

/* USER CODE BEGIN Header_doing_Task */
/**
 * @brief Function implementing the myTask_doing thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_doing_Task */
uint16_t servo_ccr=1500;
int16_t Position_A=0;
uint16_t Position_B=0;
uint16_t Position_C=0;
void doing_Task(void const *argument)
{
    /* USER CODE BEGIN doing_Task */
    /* Infinite loop */

    imu imu_date;            // 陀螺仪数据结构体
    static biu distance_now; // 距离记录结构体
    biu_int_16 color_date;
    float angle_standard = 0.0f, Direction_KP = 4.0f, Direction_KI = 0.2f;
    static int back_sign = 0,color_speed_use=0,color_speed_sum1 = 0,color_speed_sum2 = 0,color_speed_dert = 0;
    static int16_t distance_use = 0;

    static uint8_t Task_select = Start,
                   color_sign = 0;  // 0表示当前正在从原料区拿取第一个物料，1表示当前正在从原料区拿取第二个物料，2表示当前正在从原料区拿取第三个物料,3表示拿取已经结束
     
    
    int8_t color[6] = {1, 2, 3, 3, 2, 1}; // 颜色顺序存储

    uint8_t Arm_do_order = 0, // 表示需要执行动作组的编号
            color_check,
            color_first_find = 0,
            Camera_date_status = 0, // 摄像头数据接收标志位，当串口任务接收到正确的摄像头数据
                                    // 会通过color_quene消息队列传递数据，doing_task轮询接收，并改变标志位。
            color_sign_now,             // 表示当前原料区扫描检测到的物块颜色
            color_location_sign=0,//色环定位标志位
            Color_cicle_location_sign = 0,
            Do_thing_status = 0,
            i,
            target_location_sign = 0,
            angle_num = 0,
            start_status = 0,
            use_num;

    Uart5_LCD_send_task(0);           // LCD屏幕显示当前任务为开始
    vTaskSuspend(myTask_debugHandle); // 挂起调试任务
    vTaskSuspend(myTask_ledHandle);   // 挂起Led任务
    UNUSED_VARIABLE(distance_use);
    Task_verify = Task_select = Start; // 设置当前任务为开始
    speed_CTRL(0, 0, 0, 0);            // 速度归零

    for (;;)
    {
        if (xQueueReceive(angle_Queue, &imu_date, 1) == pdTRUE) // 接收偏航角数据
        {
            angle_num++;
            if (angle_num > 10) // 每250毫秒更新一次LCD屏幕上的偏航角数据
            {
                angle_num = 0;
                Uart5_LCD_show_Angle(imu_date.IMU[2]);
            }
        }
        xQueueReceive(distance_Queue, &distance_now, 1); // 接收当前轮子所走过的距离

        
        if (xSemaphoreTake(Servo_Sem_Handle, 10) == pdTRUE) // 上次动作组已经执行完毕
        {
            servo_sign = 1;
        }
        else
        {
            servo_sign = 1;
        }

        if (xQueueReceive(color_Queue, &color_date, 10) == pdTRUE) // 获取颜色顺序
        {
            Camera_date_status = 1;
            if(color_date.date[2]==33)
            {
//                color_date.date[0] = color_date_last.date[0] * 0.1 + color_date.date[0] * 0.9;
//                color_date.date[1] = color_date_last.date[1] * 0.1 + color_date.date[1] * 0.9;
//                color_date_last.date[0]=color_date.date[0];
//                color_date_last.date[1]=color_date.date[1];
                use_num++;
                if(use_num > 3)
                {
                    Uart5_LCD_show_X_Y(color_date.date[0],color_date.date[1]);
                    use_num=0;
                }
            }
        }
        if ((GPIOC->IDR & (1 << 2)) == 0)
        {
            osDelay(10);
            if ((GPIOC->IDR & (1 << 2)) == 0)
            {
                while ((GPIOC->IDR & (1 << 2)) == 0)
                {
                    osDelay(1);
                }
                /*do something*/
                speed_CTRL(0,0,0,0);
                start_status = 0;
            }
        }
        if ((GPIOC->IDR & (1 << 0)) == 0)
        {
            osDelay(10);
            if ((GPIOC->IDR & (1 << 0)) == 0)
            {
                while ((GPIOC->IDR & (1 << 0)) == 0)
                {
                    osDelay(1);
                }
                /*do something*/
                start_status = 1;
                Task_select = Task_verify = Do_capture;
//                Arm_CTL(Position_A,Position_B,Position_C,servo_ccr);
            }
        }
        if ((GPIOC->IDR & (1 << 1)) != 0 && start_status == 0)
        {
            // 从开机到现在都未按下启动按键
            if (i > 100)
            {
                Camera_die(); // 每1s发送一次关闭树莓派识别命令
                i = 0;
            }
            i++;
            osDelay(10);
            continue; // 退出本次循环，不再执行下面的所有代码
        }
        else
        {
            // 按下启动按键， 并记录'已经按下启动按键'
            if (start_status == 0)
            {
                vTaskResume(myTask_ledHandle); // LED闪烁任务启动
                osDelay(1000);
            }
            start_status = 1;
        }

        Task_select = Task_select == Task_verify ? Task_select : Task_verify; // 校验任务选择变量有没有出错

        switch (Task_select)
        {
        case Start:                                                                          // 开始向左走
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, -120) == 1) // 离开出发点向左 170 mm
            {
                speed_CTRL(0, 0, 0, 0);                 // 刹车
                Task_select = Task_verify = To_QR_Code; // 跳转任务
                Uart5_LCD_show_string("To_QR_Code");
            }
            break;
            
        case Start_Calibration:                               // 第一次方向校准
            if (direction_Set(angle_standard, imu_date) == 1) // 方向校准
            {
                speed_CTRL(0, 0, 0, 0); // 刹车

                vTaskSuspendAll();        // 临界区
                Task_select = To_QR_Code; // 跳转任务
                Task_verify = To_QR_Code;
                GM65_work(); // 发送指令到GM65执行二维码扫描
                xTaskResumeAll();
            }
            break;

        case To_QR_Code:                                                                  // 前进到二维码区域
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 360) == 1) // 前进 440 mm到达二维码扫描区
            {
                speed_CTRL(0, 0, 0, 0);   // 刹车
                vTaskSuspendAll();        // 进入临界区
                Task_select = At_QR_Code; // 任务跳转
                Task_verify = At_QR_Code;
                Camera_date_status = 0;          // 摄像头识别标志位归零（之前发送的数据无效，重新读取）
                Uart5_LCD_show_string("At_QR_Code");
                xTaskResumeAll();
                Arm_CTL(use2); //机械臂在原料区的预瞄准动作，针对物料
            }
            break;

        case At_QR_Code:                 // 扫描二维码
            if (Camera_date_status == 0) // 如果摄像头标志位未触发
            {
                GM65_work(); // 串口指令使能摄像头
                osDelay(200);
            }
            else
            {
                if (color_date.date[2] != 22) // 如果收到的数据有问题
                {
                    Camera_date_status = 0;
                }
                else // 没啥问题
                {
                    speed_CTRL(0, 0, 0, 0);                            // 刹车
                    vTaskSuspendAll();                                 // 临界区
                    color[0] = (color_date.date[3] & (3 << 10)) >> 10; // 数据读取
                    color[1] = (color_date.date[3] & (3 << 8)) >> 8;
                    color[2] = (color_date.date[3] & (3 << 6)) >> 6;
                    color[3] = (color_date.date[3] & (3 << 4)) >> 4;
                    color[4] = (color_date.date[3] & (3 << 2)) >> 2;
                    color[5] = (color_date.date[3] & (3 << 0)) >> 0;
                    Uart5_LCD_show_QR(color[0], color[1], color[2], color[3], color[4], color[5]); // LCD显示二维码数据
                    xTaskResumeAll();
//                    Camera_die(); // 关机吧
                    osDelay(10);
                    vTaskSuspendAll();
                    Task_select = To_RMA; // 任务跳转
                    Task_verify = To_RMA;

                    Camera_date_status = 0; // 摄像头标志位归零
                    Uart5_LCD_show_string("To_RMA");
                    servo_all_move(servo_motion[0]); // 机械臂回中
                    xTaskResumeAll();
                }
            }
            break;
        case To_RMA:                                                                      // 前进到达原料区
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 660) == 1) // 前进 850 mm
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = At_RMA1;
                Task_verify = At_RMA1;
                Uart5_LCD_show_string("At_RMA1");
                Camera_date_status = 0;
                Camera_die();
                xTaskResumeAll();
            }
            break;
        case At_RMA1: 
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now,50) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = Do_capture;
                Task_verify = Do_capture;
                Uart5_LCD_show_string("Do_capture");
                xTaskResumeAll();

            }
            break;
        case Do_capture:                                                                                         
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = At_RMA;
                Task_verify = At_RMA;
                Uart5_LCD_show_string("Do_capture");
                xTaskResumeAll();
                Arm_CTL(use2);
            }
            break;
        case At_RMA:                     // 摄像头数据判断校验
            if (Camera_date_status == 0) // 判断有无摄像头信息
            {
                osDelay(200);
                Camera_Color_Find_do(); // 开始扫描物块颜色
            }
            else
            {
                if (color_date.date[2] != 11) // 校验
                {
                    Camera_die();
                    Camera_date_status = 0;
                    osDelay(200);
                }
                else // 校验通过
                {
                    if (color[0] == color_date.date[1] && color_first_find == 0)
                    {
                        osDelay(200);
                    }
                    else
                    {
                        speed_CTRL(0, 0, 0, 0);
                        vTaskSuspendAll();
//                        color_first_find=1;
                        Task_select = Task_verify = Take_color_thing; // 任务跳转
                        Uart5_LCD_show_string("Take_color_thing");
                        servo_sign = 0;
                        Arm_do_order = 2;
                        xTaskResumeAll();
                    }
                }
            }

            break;
        case Take_color_thing: // 在原料区 拿取物料任务
        {
            switch (Arm_do_order) // 机械臂动作执行标志位
            {

            case 0: // 抓取第一步
                
                if (color_sign == 3)
                {
                    Arm_do_order = 250; // 进入结束动作组
                }
                else
                {
                    while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                    {
                        Arm_CTL(use20);
                    }
                    while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                    {
                        Arm_CTL(use2);
                    }                   
                    Arm_do_order=2;
                }
                
                break;
            case 2:
                if(Camera_date_status==1)
                {
                    if (color[color_sign] == color_date.date[1])
                    {
                        if (color_check > 2)
                        {
                            // 若连续900微秒扫描到的都是这个颜色，则执行下面的代码
                            color_sign++;
                            Arm_do_order++;
                            color_sign_now = color_date.date[1];
                            color_check = 0;
                        }
                        else
                        {
                            color_date.date[1] = 0;
                            color_check++;
                            osDelay(10);
                        }
                    }
                    else
                    {
                        color_check = 0;
                        osDelay(15);

                    }
                    Camera_date_status=0;
                }
                break;
            case 3: // 抓取第二步 
                    while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                    {
                        Arm_CTL(use3);
                    }
                    
                    while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                    {
                        Arm_CTL(use4);
                    }
                    Arm_do_order++;
                break;

            case 4:
                switch (color_sign_now)
                {
                case 1:
                    Arm_do_order = 11;
                    break;
                case 2:
                    Arm_do_order = 33;
                    break;
                case 3:
                    Arm_do_order = 22;
                    break;
                }
                break;
            case 11: //红色
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use19);
                }
                osDelay(1000);
                xSemaphoreTake(Servo_Sem_Handle, 500);
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use18);
                }
                osDelay(1000);
                xSemaphoreTake(Servo_Sem_Handle, 500);
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use15);
                }
                osDelay(1000);
                servo_sign = 0;
                Arm_do_order=0;                
                break;
            
            case 22: //蓝色
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use5);
                }
                osDelay(1000);
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use6);
                }
                servo_sign = 0;
                Arm_do_order=0;
                
                break;
            case 33: // 绿色
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use13);
                }
                osDelay(1000);
                xSemaphoreTake(Servo_Sem_Handle, 500);
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use9);
                }
                osDelay(1500);
                xSemaphoreTake(Servo_Sem_Handle, 500);
                while(xSemaphoreTake(Servo_Sem_Handle, 500) != pdTRUE)
                {
                    Arm_CTL(use10);
                }
                servo_sign = 0;
                Arm_do_order=0;
                
                break;

            default:
                Task_select = Task_verify = To_Turn_1;
                Uart5_LCD_show_string("To_Turn_1");
                Arm_CTL(use1);
                break;
            }
        }
        break;

        case To_Turn_1: // 直线前进
            Uart5_LCD_show_string("To_Turn_1");
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 140) == 1) // 转向1
            {
                Uart5_LCD_show_string("To_Put_down_1");
                Task_select = Task_verify = To_Put_down_1;
            }
            break;

        case At_Turn_1: // 转弯处，进行方向调整（防止压线）
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                vTaskSuspendAll();
                Task_select = Task_verify = To_Put_down_1;
                Uart5_LCD_show_string("To_Put_down_1");
                xTaskResumeAll();
            }
            break;

        case To_Put_down_1:                                                                  // 左行达到粗加工区
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, -626) == 1) // 放下1
            {

                vTaskSuspendAll();

                Task_select = Task_verify = Angle_direction_2;
                // Uart5_LCD_send_?task(4);
                xTaskResumeAll();
            }
            break;
        case Angle_direction_2: // 在粗加工区进行方向调整
            if (direction_Set_biu(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();

//                Task_select = At_Put_down_1;
//                Task_verify = At_Put_down_1;
                xTaskResumeAll();
            }
            break;
        case At_Put_down_1: // 到达粗加工区，进行色环定位
            if (color_location_sign==1)
            {
                vTaskSuspendAll();
                back_sign = At_Put_down_1;
                Task_select = Color_cicle_location;
                Task_verify = Color_cicle_location;
                xTaskResumeAll();
            }
            else
            {
                vTaskSuspendAll();
                Arm_do_order = color_sign = servo_sign = 0;
                if (Do_thing_status == 1)
                    color_sign = 3;
                Task_select = At_Put_down_1_2;
                Task_verify = At_Put_down_1_2;
                xTaskResumeAll();
            }
            break;

        case At_Put_down_1_2: // 在粗加工区 放置物料,色环
            switch (Arm_do_order)
            {
            case 0:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    if (color_sign == 3)
                    {
                        if (Do_thing_status == 0)
                            Arm_do_order = 250;
                        else
                        {
                            servo_all_move(servo_motion[0]);
                            osDelay(servo_motion[0].time + 200);
                        }
                    }
                    else
                    {
                        if (color_sign == 6)
                            Arm_do_order = 250;
                        else
                        {
                            servo_all_move(servo_motion[0]);
                            osDelay(servo_motion[0].time + 200);
                        }
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1: // 红色
                    Arm_do_order = 11;
                    color_sign++;
                    break;
                case 2: // 绿色
                    Arm_do_order = 33;
                    color_sign++;
                    break;
                case 3: // 蓝色
                    Arm_do_order = 22;
                    color_sign++;
                    break;
                }
                break;
            case 11:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[11]);
                    osDelay(servo_motion[11].time + 70);
                }
                break;
            case 12:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[10]);
                    osDelay(servo_motion[10].time + 70);
                }
                break;
            case 13:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[9]);
                    osDelay(servo_motion[9].time + 70);
                }
                break;
            case 14:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[8]);
                    osDelay(servo_motion[8].time + 70);
                }
                break;
            case 15:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[20]);
                    osDelay(servo_motion[20].time + 70);
                }
                break;
            case 16:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {

                    servo_all_move(servo_motion[42]);
                    osDelay(servo_motion[42].time + 70);

                    servo_all_move(servo_motion[23]);
                    osDelay(servo_motion[23].time + 70);
                }
                break;
            case 22:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[15]);
                    osDelay(servo_motion[15].time + 70);
                }
                break;
            case 23:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[14]);
                    osDelay(servo_motion[14].time + 70);
                }
                break;
            case 24:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[13]);
                    osDelay(servo_motion[13].time + 70);
                }
                break;
            case 25:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[12]);
                    osDelay(servo_motion[12].time + 70);
                }
                break;
            case 26:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[22]);
                    osDelay(servo_motion[22].time + 70);
                }
                break;
            case 27:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {

                    servo_all_move(servo_motion[44]);
                    osDelay(servo_motion[44].time + 70);
                    servo_all_move(servo_motion[25]);
                    osDelay(servo_motion[25].time + 70);
                }
                break;
            case 33:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[19]);
                    osDelay(servo_motion[19].time + 70);
                }
                break;
            case 34:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[18]);
                    osDelay(servo_motion[18].time + 70);
                }
                break;
            case 35:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[17]);
                    osDelay(servo_motion[17].time + 70);
                }
                break;
            case 36:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[16]);
                    osDelay(servo_motion[16].time + 70);
                }
                break;
            case 37:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[21]);
                    osDelay(servo_motion[21].time + 70);
                }
                break;
            case 38:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {

                    servo_all_move(servo_motion[43]);
                    osDelay(servo_motion[43].time + 70);
                    servo_all_move(servo_motion[24]);
                    osDelay(servo_motion[24].time + 70);
                }
                break;

            default:
                servo_all_move(servo_motion[0]);
                osDelay(servo_motion[0].time + 200);
                Task_select = Task_verify = At_Put_down_1_3;
                Arm_do_order = color_sign = servo_sign = 0;
                if (Do_thing_status == 1)
                {
                    color_sign = 3;
                }
                break;
            }
            break;
        case At_Put_down_1_3: // 在粗加工区把放下的物料拿起来
            switch (Arm_do_order)
            {
            case 0:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    if (color_sign == 3)
                    {
                        if (Do_thing_status == 0)
                            Arm_do_order = 250;
                        else
                        {
                            servo_all_move(servo_motion[0]);
                            osDelay(servo_motion[0].time + 70);
                        }
                    }
                    else
                    {
                        if (color_sign == 6)
                            Arm_do_order = 250;
                        else
                        {
                            servo_all_move(servo_motion[0]);
                            osDelay(servo_motion[0].time + 70);
                        }
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1:                // 红色
                    Arm_do_order = 11; // 动作执行顺序：23 20 8 9 10 11
                    color_sign++;
                    break;
                case 2:                // 绿色
                    Arm_do_order = 33; // 动作执行顺序:25 22 12 13 14 15
                    color_sign++;
                    break;
                case 3:                // 蓝色
                    Arm_do_order = 22; // 动作执行顺序：24 21 16 17 18 19
                    color_sign++;
                    break;
                }
                break;
            case 11:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[23]);
                    osDelay(servo_motion[23].time + 70);
                    servo_all_move(servo_motion[27]);
                    osDelay(servo_motion[27].time + 70);
                    printf("拿起红色\n");
                }
                break;
            case 12:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[36]);
                    osDelay(servo_motion[36].time + 70);
                }
                break;
            case 13:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[8]);
                    osDelay(servo_motion[8].time + 70);
                }
                break;
            case 14:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[9]);
                    osDelay(servo_motion[9].time + 70);
                }
                break;
            case 15:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[10]);
                    osDelay(servo_motion[10].time + 70);
                }
                break;
            case 16:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[11]);
                    osDelay(servo_motion[11].time + 70);
                }
                break;
            case 22:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[25]);
                    osDelay(servo_motion[25].time + 70);
                    servo_all_move(servo_motion[29]);
                    osDelay(servo_motion[29].time + 70);
//                    printf("拿起蓝色\n");
                }
                break;
            case 23:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[38]);//合爪收回动作
                    osDelay(servo_motion[38].time + 70);
                }
                break;
            case 24:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[12]);
                    osDelay(servo_motion[12].time + 70);
                }
                break;
            case 25:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[13]);
                    osDelay(servo_motion[13].time + 70);
                }
                break;
            case 26:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[14]);
                    osDelay(servo_motion[14].time + 70);
                }
                break;
            case 27:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[15]);
                    osDelay(servo_motion[15].time + 70);
                }
                break;
            case 33:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[24]);
                    osDelay(servo_motion[24].time + 70);
                    servo_all_move(servo_motion[28]);
                    osDelay(servo_motion[28].time + 70);
                }
                break;
            case 34:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[37]);
                    osDelay(servo_motion[37].time + 70);
                }
                break;
            case 35:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[16]);
                    osDelay(servo_motion[16].time + 70);
                }
                break;
            case 36:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[17]);
                    osDelay(servo_motion[17].time + 70);
                }
                break;
            case 37:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[18]);
                    osDelay(servo_motion[18].time + 70);
                }
                break;
            case 38:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[19]);
                    osDelay(servo_motion[19].time + 70);
                }
                break;

            default:
                servo_all_move(servo_motion[0]);
                osDelay(servo_motion[0].time + 500);
                Task_select = Task_verify = To_Turn_2;
                break;
            }
            break;
        case To_Turn_2:                                                                      // 左行前往暂存区
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, -528) == 1) //
            {

                vTaskSuspendAll();
                Task_select = Task_verify = At_Turn_2;
                xTaskResumeAll();
            }
            break;
        case At_Turn_2:                                                                    // 直线前进到达暂存区
            if (advance_angle_distance(angle_standard, imu_date, distance_now, -577) == 1) //
            {

                vTaskSuspendAll();
                angle_standard = 90.2f;
                Task_select = Task_verify = To_Put_down_2;
                xTaskResumeAll();
            }
            break;
        case To_Put_down_2: // 在暂存区调整车身角度
            if (direction_Set_biu(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                osDelay(500);
                xQueueReceive(angle_Queue, &imu_date, portMAX_DELAY);
                xQueueReceive(angle_Queue, &imu_date, portMAX_DELAY);
                if (direction_Set(angle_standard, imu_date) == 1)
                {
                    vTaskSuspendAll();

                    Task_select = At_Put_down_2;
                    Task_verify = At_Put_down_2;
                    xTaskResumeAll();
                }
            }
            break;
        case At_Put_down_2: // 在暂存区的两次定位，使用变量Do_thing_status进行功能选择执行
            if (Color_cicle_location==0)
            {
                vTaskSuspendAll();
                back_sign = At_Put_down_2;
                Task_select = Color_cicle_location;
                Task_verify = Color_cicle_location;

                xTaskResumeAll();
            }
            else
            {
                vTaskSuspendAll();
                Arm_do_order = color_sign = servo_sign = 0;
                if (Do_thing_status == 0)
                {
                    Task_select = At_Put_down_2_1;
                    Task_verify = At_Put_down_2_1;
                    Uart5_LCD_show_string("At_Put_down_2");
                }
                else
                {
                    Uart5_LCD_show_string("At_Stacking");
                    color_sign = 3;
                    Task_select = Stacking;
                    Task_verify = Stacking;
                }

                xTaskResumeAll();
            }
            break;
        case At_Put_down_2_1:
            switch (Arm_do_order)
            {
            case 0:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    if (color_sign == 3)
                    {
                        Arm_do_order = 250;
                    }
                    else
                    {
                        servo_all_move(servo_motion[0]);
                        osDelay(servo_motion[0].time + 500);
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1: // 红色
                    Arm_do_order = 11;
                    color_sign++;
                    break;
                case 2: // 绿色
                    Arm_do_order = 33;
                    color_sign++;
                    break;
                case 3: // 蓝色
                    Arm_do_order = 22;
                    color_sign++;
                    break;
                }
                break;
            case 11:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[11]);
                    osDelay(servo_motion[11].time + 70);
                }
                break;
            case 12:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[10]);
                    osDelay(servo_motion[10].time + 70);
                }
                break;
            case 13:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[9]);
                    osDelay(servo_motion[9].time + 70);
                }
                break;
            case 14:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[8]);
                    osDelay(servo_motion[8].time + 70);
                }
                break;
            case 15:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[20]);
                    osDelay(servo_motion[20].time + 70);
                }
                break;
            case 16:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {

                    servo_all_move(servo_motion[42]);
                    osDelay(servo_motion[42].time + 70);
                    servo_all_move(servo_motion[23]);
                    osDelay(servo_motion[23].time + 70);
                }
                break;
            case 22:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[15]);
                    osDelay(servo_motion[15].time + 70);
                }
                break;
            case 23:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[14]);
                    osDelay(servo_motion[14].time + 70);
                }
                break;
            case 24:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[13]);
                    osDelay(servo_motion[13].time + 70);
                }
                break;
            case 25:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[12]);
                    osDelay(servo_motion[12].time + 70);
                }
                break;
            case 26:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[22]);
                    osDelay(servo_motion[22].time + 70);
                }
                break;
            case 27:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {

                    servo_all_move(servo_motion[44]);
                    osDelay(servo_motion[44].time + 70);
                    servo_all_move(servo_motion[25]);
                    osDelay(servo_motion[25].time + 70);
                }
                break;
            case 33:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[19]);
                    osDelay(servo_motion[19].time + 70);
                }
                break;
            case 34:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[18]);
                    osDelay(servo_motion[18].time + 70);
                }
                break;
            case 35:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[17]);
                    osDelay(servo_motion[17].time + 70);
                }
                break;
            case 36:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[16]);
                    osDelay(servo_motion[16].time + 70);
                }
                break;
            case 37:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[21]);
                    osDelay(servo_motion[21].time + 70);
                }
                break;
            case 38:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {

                    servo_all_move(servo_motion[43]);
                    osDelay(servo_motion[43].time + 70);
                    servo_all_move(servo_motion[24]);
                    osDelay(servo_motion[24].time + 70);
                }
                break;

            default:
                servo_all_move(servo_motion[0]);
                osDelay(servo_motion[0].time + 500);
                Task_select = Task_verify = AT_Put_down_2_2;
                angle_standard = 0.0;
                Arm_do_order = color_sign = servo_sign = 0;
                break;
            }
            break;
        case AT_Put_down_2_2:
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Uart5_LCD_show_string("Back_Take_thing");
                Task_select = Back_Take_thing_1;
                Task_verify = Back_Take_thing_1;
                xTaskResumeAll();
            }
            break;
        case Back_Take_thing_1:
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 640) == 1) //
            {

                vTaskSuspendAll();
                Task_select = Task_verify = Back_Take_thing_2;
                xTaskResumeAll();
            }
            break;
        case Back_Take_thing_2:
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, 534) == 1) //
            {

                vTaskSuspendAll();
                Task_select = Task_verify = Back_Take_thing_4;
                xTaskResumeAll();
            }
            break;
        
        case Back_Take_thing_4:
            vTaskSuspendAll();
            
            Task_select = Task_verify = Angle_direction_3;
            xTaskResumeAll();
            break;

        case Angle_direction_3: // 第二次在原料区调整角度
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = Do_capture_2;
                Task_verify = Do_capture_2;
                xTaskResumeAll();
            }
            break;
        case Do_capture_2: // 第二次在原料区定位
            
                vTaskSuspendAll();
                Camera_Color_Find_do();
                Arm_do_order = servo_sign = 0;
                color_sign = 3;
                Task_select = Take_color_thing_2;
                Task_verify = Take_color_thing_2;
                Uart5_LCD_show_string("Do_capture_2");
                xTaskResumeAll();
            
            break;
        case Take_color_thing_2: // 第二次去原料区拿取物料
        {
            switch (Arm_do_order)
            {
            case 0:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[0]);
                    osDelay(servo_motion[0].time + 100);
                    servo_all_move(servo_motion[3]);
                    osDelay(servo_motion[3].time + 100);
                }
                break;
            case 1: // 抓取第一步
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    if (color_sign == 6)
                    {
                        Arm_do_order = 250;
                    }
                    else
                    {
                        servo_all_move(servo_motion[26]);
                        osDelay(servo_motion[26].time + 70);
                    }
                }
                break;
            case 2:
                if(Camera_date_status==1)
                {
                    if (color[color_sign] == color_date.date[1] && color_sign == 3 && color_first_find == 0)
                    {
                        osDelay(100);
                        break;
                    }
                    else
                    {
                        color_first_find = 1;
                    }
                    if (color[color_sign] == color_date.date[1])
                    {
                        if (color_check > 4)
                        {

                            color_sign++;
                            Arm_do_order++;
                            color_sign_now = color_date.date[1];
                            color_check = 0;
                        }
                        else
                        {

                            color_date.date[1] = 0;
                            color_check++;
                            osDelay(150);
                        }
                    }
                    else
                    {
                        color_check = 0;
                        osDelay(150);
                    }
                    printf("%d,%d\n", color[color_sign], color_date.date[1]);
                    
                    Camera_date_status=0;
                }
                break;
            case 3: // 抓取第二步
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[3]);
                    osDelay(servo_motion[3].time + 70);
                    servo_all_move(servo_motion[4]);
                    osDelay(servo_motion[4].time + 70);
                    servo_all_move(servo_motion[5]);
                    osDelay(servo_motion[5].time + 70);
                }
                break;
            case 4: // 抓取第三步
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[6]);
                    osDelay(servo_motion[6].time + 70);
                }
                break;
            case 5: // 抓取第四步
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[7]);
                    osDelay(servo_motion[7].time + 70);
                }
                break;
            case 6:
                switch (color_sign_now)
                {
                case 1:
                    Arm_do_order = 11;
                    break;
                case 2:
                    Arm_do_order = 33;
                    break;
                case 3:
                    Arm_do_order = 22;
                    break;
                }
                break;
            case 11:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[8]);
                    osDelay(servo_motion[8].time + 70);
                }
                break;
            case 12:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[9]);
                    osDelay(servo_motion[9].time + 70);
                }
                break;
            case 13:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[10]);
                    osDelay(servo_motion[10].time + 70);
                }
                break;
            case 14:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[11]);
                    osDelay(servo_motion[11].time + 70);
                }
                break;
            case 22:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[12]);
                    osDelay(servo_motion[12].time + 70);
                }
                break;
            case 23:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[13]);
                    osDelay(servo_motion[13].time + 70);
                }
                break;
            case 24:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[14]);
                    osDelay(servo_motion[14].time + 70);
                }
                break;
            case 25:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[15]);
                    osDelay(servo_motion[15].time + 70);
                }
                break;
            case 33:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[16]);
                    osDelay(servo_motion[16].time + 70);
                }
                break;
            case 34:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[17]);
                    osDelay(servo_motion[17].time + 70);
                }
                break;
            case 35:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[18]);
                    osDelay(servo_motion[18].time + 70);
                }
                break;
            case 36:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[19]);
                    osDelay(servo_motion[19].time + 70);
                }
                break;

            default:
                Do_thing_status = 1;
                Task_select = Task_verify = To_Turn_1;
                break;
            }
        }
        break;
        case Stacking: // 码垛
            switch (Arm_do_order)
            {
            case 0:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    if (color_sign == 3)
                    {
                        if (Do_thing_status == 0)
                            Arm_do_order = 250;
                        else
                        {
                            servo_all_move(servo_motion[0]);
                            osDelay(servo_motion[0].time + 500);
                        }
                    }
                    else
                    {
                        if (color_sign == 6)
                            Arm_do_order = 250;
                        else
                        {
                            servo_all_move(servo_motion[0]);
                            osDelay(servo_motion[0].time + 500);
                        }
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1: // 红色
                    Arm_do_order = 11;
                    color_sign++;
                    break;
                case 2: // 绿色
                    Arm_do_order = 33;
                    color_sign++;
                    break;
                case 3: // 蓝色
                    Arm_do_order = 22;
                    color_sign++;
                    break;
                }
                break;
            case 11:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[11]);
                    osDelay(servo_motion[11].time + 70);
                }
                break;
            case 12:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[10]);
                    osDelay(servo_motion[10].time + 70);
                }
                break;
            case 13:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[9]);
                    osDelay(servo_motion[9].time + 70);
                }
                break;
            case 14:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[8]);
                    osDelay(servo_motion[8].time + 70);
                }
                break;
            case 15:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[30]);
                    osDelay(servo_motion[30].time + 70);
                }
                break;
            case 16:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[39]);
                    osDelay(servo_motion[39].time + 70);

                    servo_all_move(servo_motion[33]);
                    osDelay(servo_motion[33].time + 70);
                }
                break;
            case 22:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[15]);
                    osDelay(servo_motion[15].time + 70);
                }
                break;
            case 23:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[14]);
                    osDelay(servo_motion[14].time + 70);
                }
                break;
            case 24:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[13]);
                    osDelay(servo_motion[13].time + 70);
                }
                break;
            case 25:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[12]);
                    osDelay(servo_motion[12].time + 70);
                }
                break;
            case 26:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[32]);
                    osDelay(servo_motion[32].time + 70);
                }
                break;
            case 27:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {

                    servo_all_move(servo_motion[41]);
                    osDelay(servo_motion[41].time + 70);

                    servo_all_move(servo_motion[35]);
                    osDelay(servo_motion[35].time + 70);
                }
                break;
            case 33:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[19]);
                    osDelay(servo_motion[19].time + 70);
                }
                break;
            case 34:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[18]);
                    osDelay(servo_motion[18].time + 70);
                }
                break;
            case 35:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[17]);
                    osDelay(servo_motion[17].time + 70);
                }
                break;
            case 36:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[16]);
                    osDelay(servo_motion[16].time + 70);
                }
                break;
            case 37:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_all_move(servo_motion[31]);
                    osDelay(servo_motion[31].time + 70);
                }
                break;
            case 38:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order = 0;
                }
                else
                {
                    servo_all_move(servo_motion[40]);
                    osDelay(servo_motion[40].time + 70);
                    servo_all_move(servo_motion[34]);
                    osDelay(servo_motion[34].time + 70);
                }
                break;

            default:
                servo_all_move(servo_motion[0]);
                osDelay(servo_motion[0].time + 500);
                Task_select = Task_verify = Back_home_1;
                Arm_do_order = color_sign = servo_sign = 0;
                break;
            }
            break;
        case Back_home_1:
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, -735) == 1) //
            {

                vTaskSuspendAll();
                Uart5_LCD_show_string("Back_home");
                Task_select = Task_verify = Back_home_2;
                xTaskResumeAll();
            }
            break;
        case Back_home_2:
            if (advance_angle_distance(angle_standard, imu_date, distance_now, -1145) == 1) //
            {

                vTaskSuspendAll();


                Task_select = Task_verify = Stop;
                xTaskResumeAll();
            }
            break;
        case Stop:
            
            break;
        case Color_cicle_location:  // 判断颜色圈位置
            if(Camera_date_status==1)  // 如果相机数据状态为1
            {
                if (abs(color_date.date[0] - color_circle_x) > 2)  // 如果颜色数据的第一个元素与颜色圈的x坐标的差的绝对值大于2
                {
                    color_speed_dert = color_date.date[0] - color_circle_x;  // 颜色速度差等于颜色数据的第一个元素减去颜色圈的x坐标
                    color_speed_dert = abs(color_speed_dert) > 5 ? color_speed_dert : color_speed_dert * 5;  // 如果颜色速度差的绝对值大于5，则颜色速度差不变，否则颜色速度差乘以5
                    color_speed_sum2 += color_speed_dert;  // 颜色速度和等于颜色速度和加上颜色速度差
                    color_speed_sum2 = color_speed_sum2 > 2000 ? 2000 : (color_speed_sum2 < -2000 ? -2000 : color_speed_sum2);  // 如果颜色速度和大于2000，则颜色速度和等于2000，否则如果颜色速度和小于-2000，则颜色速度和等于-2000，否则颜色速度和不变
                    color_speed_use = color_speed_dert * Direction_KP + color_speed_sum2 * Direction_KI;  // 使用的颜色速度等于颜色速度差乘以方向KP加上颜色速度和乘以方向KI
                    color_speed_use = color_speed_use > 1000 ? 1000 : (color_speed_use < -1000 ? -1000 : color_speed_use);  // 如果使用的颜色速度大于1000，则使用的颜色速度等于1000，否则如果使用的颜色速度小于-1000，则使用的颜色速度等于-1000，否则使用的颜色速度不变
                    advance_angle(imu_date.IMU[2], imu_date, color_speed_use);  // 调用advance_angle函数，参数为imu_date的第三个元素，imu_date，使用的颜色速度
                }
                else if (abs(color_date.date[1] - color_circle_y) > 2)  // 否则，如果颜色数据的第二个元素与颜色圈的y坐标的差的绝对值大于2
                {
                    color_speed_sum2 = 0;  // 颜色速度和等于0
                    color_speed_dert = color_circle_y-color_date.date[1] ;  // 颜色速度差等于颜色圈的y坐标减去颜色数据的第二个元素
                    color_speed_dert = abs(color_speed_dert) > 5 ? color_speed_dert : color_speed_dert * 5;  // 如果颜色速度差的绝对值大于5，则颜色速度差不变，否则颜色速度差乘以5
                    color_speed_sum1 += color_speed_dert;  // 颜色速度和等于颜色速度和加上颜色速度差
                    color_speed_sum1 = color_speed_sum1 > 2000 ? 2000 : (color_speed_sum1 < -2000 ? -2000 : color_speed_sum1);  // 如果颜色速度和大于2000，则颜色速度和等于2000，否则如果颜色速度和小于-2000，则颜色速度和等于-2000，否则颜色速度和不变
                    color_speed_use = color_speed_dert * Direction_KP + color_speed_sum1 * Direction_KI;  // 使用的颜色速度等于颜色速度差乘以方向KP加上颜色速度和乘以方向KI
                    color_speed_use = color_speed_use > 1000 ? 1000 : (color_speed_use < -1000 ? -1000 : color_speed_use);  // 如果使用的颜色速度大于1000，则使用的颜色速度等于1000，否则如果使用的颜色速度小于-1000，则使用的颜色速度等于-1000，否则使用的颜色速度不变
                    crosswise_angle(imu_date.IMU[2], imu_date, color_speed_use);  // 调用crosswise_angle函数，参数为imu_date的第三个元素，imu_date，使用的颜色速度
                }
                else  // 否则
                {
                    speed_CTRL(0, 0, 0, 0);  // 调用speed_CTRL函数，参数都为0
                    color_speed_sum1 = color_speed_sum2 = 0;  // 颜色速度和等于0
                    osDelay(200);  // 延迟200毫秒
                    if (target_location_sign < 6)  // 如果颜色圈位置标志小于6
                    {
                        target_location_sign++;  // 颜色圈位置标志加1
                    }
                    else  // 否则
                    {
                        target_location_sign = 0;  // 颜色圈位置标志等于0
                        Camera_die();  // 调用Camera_die函数
                        Task_select = back_sign;  // 任务选择等于返回标志
                        Task_verify = back_sign;  // 任务验证等于返回标志
                        color_location_sign=1;  // 颜色位置标志等于1
                    }
                }
                Camera_date_status=0;  // 相机数据状态等于0
            }
            else  // 否则
                osDelay(10);  // 延迟10毫秒
            break;  // 跳出循环 
        case End:
            speed_CTRL(0, 0, 0, 0);
            vTaskSuspend(NULL);
            break;
        }

        osDelay(10);
    }
    /* USER CODE END doing_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Arm_CTL(uint16_t MOTORA_step,uint16_t MOTORB_step,uint16_t MOTORC_step,uint16_t Servo)
{
    ServoMissionSend(Servo);
    osDelay(100);
    MotorMoveToSend(MotorA,MOTORA_step);
    osDelay(100);
    MotorMoveToSend(MotorB,MOTORB_step);
    osDelay(100);
    MotorMoveToSend(MotorC,MOTORC_step);
    osDelay(100); 

}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
