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
#include "fashion_star_uart_servo.h"
// #include "HI229.h"
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

#define IDR_color_grab_x 1090
#define IDR_color_grab_y 1860

#define IDR_color_put_x 1943
#define IDR_color_put_y 1190

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern char s_cDataUpdate, s_cCmd; // MPU6050移植参数，不用管

extern servo_status servo_motion[];

extern Usart_DataTypeDef usart4;

uint8_t OS_status = 0; // 任务初始化标志位，为1则初始化OK，为0则还没有嘞

uint8_t debug_date[600] = {0}; // debug数据缓存区，用于缓存需要用来上传的debug数据。

uint8_t IMU_date[IMU_speed] = {0}; // MPU6050数据缓冲区、采用DMA+串口方式接收

uint8_t usart_3_date[25] = {0};
uint8_t usart_6_date[22] = {0};

uint16_t modetime = 100;

Usart_DataTypeDef *servoUsart = &usart4;

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

QueueHandle_t Speed_Queue = NULL;
QueueHandle_t goal_Queue = NULL;
QueueHandle_t IMU_Queue = NULL;
QueueHandle_t debug_Queue = NULL;
QueueHandle_t angle_Queue = NULL;
QueueHandle_t servo_Queue = NULL;
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
void key_Task(void const *argument);
void led_Task(void const *argument);
void IMU_Task(void const *argument);
void deubg_Task(void const *argument);
void debug_get_Task(void const *argument);
void usart_Task(void const *argument);
void oled_Task(void const *argument);
void servo_Task(void const *argument);
void speed_Task(void const *argument);
void pid_Task(void const *argument);
void doing_Task(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

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
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
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
    osThreadDef(myTask_debug, deubg_Task, osPriorityLow, 0, 512);
    myTask_debugHandle = osThreadCreate(osThread(myTask_debug), NULL);

    /* definition and creation of debug_get */
    osThreadDef(debug_get, debug_get_Task, osPriorityAboveNormal, 0, 128);
    debug_getHandle = osThreadCreate(osThread(debug_get), NULL);

    /* definition and creation of myTask_usart */
    osThreadDef(myTask_usart, usart_Task, osPriorityNormal, 0, 512);
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
    osThreadDef(myTask_pid, pid_Task, osPriorityHigh, 0, 128);
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

    for (;;)
    {
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

                vTaskSuspend(myTask_doingHandle); // 进入手掰模式
                vTaskResume(myTask_debugHandle);
                FSUS_DampingMode(&usart4, 1, 0);
                osDelay(100);
                FSUS_DampingMode(&usart4, 2, 0);
                osDelay(100);
                FSUS_DampingMode(&usart4, 3, 0);
                osDelay(100);
                FSUS_DampingMode(&usart4, 4, 0);
                osDelay(100);
                FSUS_DampingMode(&usart4, 5, 0);
                osDelay(100);
            }
        }
        if ((GPIOC->IDR & (1 << 1)) == 0)
        {
            osDelay(10);
            if ((GPIOE->IDR & (1 << 1)) == 0)
            {
                while ((GPIOE->IDR & (1 << 1)) == 0)
                {
                    osDelay(1);
                }
                /*do something*/

                vTaskResume(myTask_doingHandle); // 进入工作模式
                vTaskSuspend(myTask_debugHandle);
                servo_all_move(servo_motion[1]);
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
        GPIOC->ODR ^= GPIO_PIN_2;
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
        }
#endif
        HAL_UART_Receive_DMA(&huart2, IMU_date, IMU_speed);
        taskEXIT_CRITICAL();
        xQueueSend(IMU_Queue, &imu_car, 0);
        xQueueSend(angle_Queue, &imu_car, 0);
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
    imu imu_car;
    biu speed_now;
    biu speed_goal;
    float angle[5];
    uint16_t i = 0;
    for (;;)
    {

        xQueueReceive(IMU_Queue, &imu_car, 10);
        xQueueReceive(debug_Queue, &speed_goal, 10);
        xQueuePeek(Speed_Queue, &speed_now, 10);

        for (i = 0; i < 5; i++)
        {
            FSUS_QueryServoAngle(servoUsart, i + 1, &angle[i]);
            osDelay(10);
        }

        taskENTER_CRITICAL();
        //        sprintf((char *)debug_date, "%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d\n",
        //                imu_car.IMU[0], imu_car.IMU[1], imu_car.IMU[2],
        //                speed_now.date[0], speed_goal.date[0], speed_now.date[1], speed_goal.date[1],
        //                speed_now.date[2], speed_goal.date[2], speed_now.date[3], speed_goal.date[3]);
        sprintf((char *)debug_date, "%.1f,%.1f,%.1f,%.1f,%.1f,%.2f\n", angle[0], angle[1], angle[2], angle[3], angle[4], imu_car.IMU[2]);
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
    IDR_date IDR_date_get;
    HAL_UART_Receive_DMA(&huart3, usart_3_date, 22);
    HAL_UART_Receive_DMA(&huart6, usart_6_date, 22);

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
                        sign = 1;
                    else if (usart_3_date[num + 1] == 0X21)
                        sign = 2;
                    else
                        sign = 0;
                    break;
                }
                sign = 0;
            }
            // HAL_UART_Transmit_DMA(&huart1, usart_3_date, 22);
            if (sign == 1)
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
            else if (sign == 2)
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
                        printf("%d\n", x);
                    }
                    else
                    {
                        // error
                    }
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
            if (usart_6_date[0] == '#' && usart_6_date[10] == '&')
            {
                num = 0;
                if (usart_6_date[5] == '!')
                    sign = 1;
                else
                    sign = 0;
            }

            //            HAL_UART_Transmit_DMA(&huart1, usart_6_date, 12);
            // usart_6_date[11]='\0';
            // printf("%s\n",usart_6_date);
            if (sign == 1)
            {

                for (num = i + 1; usart_6_date[num] != '!' && num < i + 1 + 5; num++)
                {
                    if (usart_6_date[num] < '0')
                    {
                        ;
                    }
                    else
                        usart_6_date[num] = usart_6_date[num] - '0';
                }
                if (num < i + 1 + 5)
                    IDR_date_get.IDR_X = (usart_6_date[i + 1]) * 1000 + (usart_6_date[i + 2]) * 100 + (usart_6_date[i + 3]) * 10 + (usart_6_date[i + 4]);

                for (num = i + 6; usart_6_date[num] != '&' && num < i + 6 + 5; num++)
                {
                    if (usart_6_date[num] < '0')
                    {
                        ;
                    }
                    else
                        usart_6_date[num] = usart_6_date[num] - '0';
                }
                if (num < i + 6 + 5)
                    IDR_date_get.IDR_Y = (usart_6_date[i + 6]) * 1000 + (usart_6_date[i + 7]) * 100 + (usart_6_date[i + 8]) * 10 + (usart_6_date[i + 9]);

                printf("%d %d\n", IDR_date_get.IDR_X, IDR_date_get.IDR_Y);
                xQueueSend(IDR_date_Queue, &IDR_date_get, 0);
            }
            else if (sign == 2)
            {
                printf("error1\n");
            }
            else
            {
                printf("error2\n");
            }
            HAL_UART_Receive_DMA(&huart6, usart_6_date, 11);
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
void oled_Task(void const *argument)
{
    /* USER CODE BEGIN oled_Task */
    /* Infinite loop */
    for (;;)
    {
        osDelay(100);
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

    uint16_t time = 0;

    static servo_status servo;
    for (;;)
    {
        xQueueReceive(servo_Queue, &servo, portMAX_DELAY);
        if (servo.time == 0)
            time = 1000;
        else
            time = servo.time;
        FSUS_SetServoAngle(servoUsart, 5, servo.value_goal[4], time, 0, 0);
        osDelay(100);
        FSUS_SetServoAngle(servoUsart, 1, servo.value_goal[0], time, 0, 0);
        osDelay(50);
        FSUS_SetServoAngle(servoUsart, 2, servo.value_goal[1], time, 0, 0);
        osDelay(50);
        FSUS_SetServoAngle(servoUsart, 3, servo.value_goal[2], time, 0, 0);
        osDelay(50);
        FSUS_SetServoAngle(servoUsart, 4, servo.value_goal[3], time, 0, 0);
        osDelay(50);

        osDelay(time / 2);

        xSemaphoreGive(Servo_Sem_Handle);

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
    biu speed;
    static biu distance = {0, 0, 0, 0};
    uint8_t i = 0;
    for (;;)
    {

        vTaskSuspendAll();
        speed.date[0] = 6720 - TIM2->CNT;
        TIM2->CNT = 6720;

        speed.date[1] = TIM3->CNT - 6720;
        TIM3->CNT = 6720;

        speed.date[2] = 6720 - TIM4->CNT;
        TIM4->CNT = 6720;

        speed.date[3] = TIM5->CNT - 6720;
        TIM5->CNT = 6720;
        for (i = 0; i < 4; i++)
        {
            distance.date[i] += speed.date[i];
        }
        xTaskResumeAll();
        xQueueSend(Speed_Queue, &speed, 0); /* 等待时间 0 */
        xQueueSend(distance_Queue, &distance, 0);
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
    biu speed_goal;
    biu speed_now;

    for (;;)
    {
        xQueueReceive(Speed_Queue, /* 消息队列的句柄 */
                      &speed_now,  /* 发送的消息内容 */
                      5);          /* 等待时间 一直等 */
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
void doing_Task(void const *argument)
{
    /* USER CODE BEGIN doing_Task */
    /* Infinite loop */

    imu imu_date;            // 陀螺仪数据结构体
    static biu distance_now; // 距离记录结构体
    biu_int_16 color_date;
    IDR_date IDR_date_use, IDR_date_goal;
    int IDR_speed_use = 0;

    static int back_sign = 0, IDR_speed_sum1 = 0, IDR_speed_dert = 0, IDR_speed_sum2 = 0;
    static int16_t distance_use = 0;

    static uint8_t Task_select = Start, color_sign = 0; // 执行调度参数

    static float angle_adjust = 0;

    int8_t color[6] = {1, 2, 3, 0, 0, 0}; // 颜色顺序存储

    uint8_t Arm_do_order = 0,
            servo_sign,
            color_check,
            V831_1_status = 0,
            color_sign_now,
            target_location_sign = 0;
    vTaskSuspend(myTask_debugHandle);
    UNUSED_VARIABLE(distance_use);
    Task_verify = Task_select = Start;
    speed_CTRL(0, 0, 0, 0);
    for (;;)
    {
        xQueueReceive(angle_Queue, &imu_date, 10);        /* 等待时间 10 */
        xQueueReceive(distance_Queue, &distance_now, 10); /* 等待时间 10 */
        xQueueReceive(IDR_date_Queue, &IDR_date_use, 10); /* 等待时间 10 */
        //      imu_date.IMU[2]=0;
        if (xSemaphoreTake(Servo_Sem_Handle, 0) == pdTRUE)
            servo_sign = 1;

        if (xQueueReceive(color_Queue, &color_date, 0) == pdTRUE)
        {
            V831_1_status = 1;
        }

        Task_select = Task_select == Task_verify ? Task_select : Task_verify;

        switch (Task_select)
        {
        case Start:
            if (advance_angle_distance(0, imu_date, distance_now, 270) == 1) // 离开出发点前进28cm
            {
                speed_CTRL(0, 0, 0, 0);
                Task_select = Task_verify = Start_Calibration;
                servo_all_move(servo_motion[1]);
            }
            break;

        case Start_Calibration:
            if (direction_Set(0, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = To_QR_Code;
                Task_verify = To_QR_Code;
                //                V831_QuickMark_do();
                xTaskResumeAll();
            }
            break;

        case To_QR_Code:
            if (crosswise_angle_distance(0, imu_date, distance_now, 540) == 1) // 到达二维码扫描区
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = At_QR_Code;
                Task_verify = At_QR_Code;
                // Task_select = To_RMA;
                // Task_verify = To_RMA;
                servo_all_move(servo_motion[2]);
                V831_1_status = 0;
                xTaskResumeAll();
            }

            break;

        case At_QR_Code: // 扫描二维码
            if (V831_1_status == 0)
            {
                V831_QuickMark_do();
                osDelay(200);
            }
            else
            {
                if (color_date.date[2] != 22)
                {
                    V831_die();
                    V831_1_status = 0;
                }
                else
                {
                    speed_CTRL(0, 0, 0, 0);
                    vTaskSuspendAll();
                    color[0] = (color_date.date[3] & (3 << 10)) >> 10;
                    color[1] = (color_date.date[3] & (3 << 8)) >> 8;
                    color[2] = (color_date.date[3] & (3 << 6)) >> 6;
                    color[3] = (color_date.date[3] & (3 << 4)) >> 4;
                    color[4] = (color_date.date[3] & (3 << 2)) >> 2;
                    color[5] = (color_date.date[3] & (3 << 0)) >> 0;
                    xTaskResumeAll();

                    V831_die();
                    vTaskSuspendAll();
                    Task_select = QR_Code_turn;
                    Task_verify = QR_Code_turn;
                    V831_1_status = 0;
                    servo_all_move(servo_motion[0]);
                    xTaskResumeAll();
                    Task_select = To_RMA;
                    Task_verify = To_RMA;
                }
            }
            break;
        case To_RMA:
            if (advance_angle_distance(0, imu_date, distance_now, 495) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = Do_capture;
                Task_verify = Do_capture;

                V831_1_status = 0;
                IDR_date_goal.IDR_X = IDR_color_grab_x;
                IDR_date_goal.IDR_Y = IDR_color_grab_y;
                V831_die();
                xTaskResumeAll();
            }
            break;
        case Do_capture:
            if (abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) > 4 && abs(IDR_date_goal.IDR_Y - IDR_date_use.IDR_Y) > 4)
            {
                vTaskSuspendAll();
                back_sign = Do_capture;
                Task_select = target_location;
                Task_verify = target_location;
                xTaskResumeAll();
            }
            else
            {
                vTaskSuspendAll();
                Task_select = At_RMA;
                Task_verify = At_RMA;
                V831_1_status = 0;
                V831_die();
                xTaskResumeAll();
                servo_all_move(servo_motion[0]);
                osDelay(servo_motion[0].time);
                servo_all_move(servo_motion[3]);
                osDelay(servo_motion[3].time);
                servo_all_move(servo_motion[26]);
                osDelay(servo_motion[26].time + 200);
            }
            break;
        case At_RMA: // 原料区拿取物料
            if (V831_1_status == 0)
            {
                osDelay(200);
                V831_Color_Find_do();
            }
            else
            {
                if (color_date.date[2] != 11)
                {
                    V831_die();
                    V831_1_status = 0;
                    osDelay(200);
                }
                else
                {
                    speed_CTRL(0, 0, 0, 0);
                    vTaskSuspendAll();
                    Task_select = Task_verify = Take_color_thing;
                    servo_sign = 0;
                    Arm_do_order = 2;
                    xTaskResumeAll();
                }
            }

            break;
        case Take_color_thing:
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
                    osDelay(servo_motion[0].time);
                    servo_all_move(servo_motion[3]);
                    osDelay(servo_motion[3].time);
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
                    if (color_sign == 3)
                    {
                        Arm_do_order = 250;
                    }
                    else
                    {
                        servo_all_move(servo_motion[26]);
                        osDelay(servo_motion[26].time + 200);
                    }
                }
                break;
            case 2:
                if (color[color_sign] == color_date.date[1])
                {
                    if (color_check > 10)
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
                    osDelay(servo_motion[3].time + 200);
                    servo_all_move(servo_motion[4]);
                    osDelay(servo_motion[4].time + 200);
                    servo_all_move(servo_motion[5]);
                    osDelay(servo_motion[5].time + 200);
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
                    osDelay(servo_motion[6].time + 200);
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
                    osDelay(servo_motion[7].time + 200);
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
                    osDelay(servo_motion[8].time + 200);
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
                    osDelay(servo_motion[9].time + 200);
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
                    osDelay(servo_motion[10].time + 200);
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
                    osDelay(servo_motion[11].time + 200);
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
                    osDelay(servo_motion[12].time + 200);
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
                    osDelay(servo_motion[13].time + 200);
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
                    osDelay(servo_motion[14].time + 200);
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
                    osDelay(servo_motion[15].time + 200);
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
                    osDelay(servo_motion[16].time + 200);
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
                    osDelay(servo_motion[17].time + 200);
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
                    osDelay(servo_motion[18].time + 200);
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
                    osDelay(servo_motion[19].time + 200);
                }
                break;

            default:
                Task_select = Task_verify = To_Turn_1;
                break;
            }
        }
        break;

        case To_Turn_1:
            if (advance_angle_distance(0, imu_date, distance_now, 610) == 1) // 转向1
            {

                Task_select = Task_verify = At_Turn_1;
            }
            break;

        case At_Turn_1:
            if (direction_Set(0, imu_date) == 1)
            {
                vTaskSuspendAll();
                Task_select = Task_verify = To_Put_down_1;
                xTaskResumeAll();
            }
            break;

        case To_Put_down_1:

            if (crosswise_angle_distance(angle_adjust, imu_date, distance_now, -540) == 1) // 放下1
            {

                vTaskSuspendAll();
                IDR_date_goal.IDR_X = IDR_color_put_x;
                IDR_date_goal.IDR_Y = IDR_color_put_y;
                Task_select = Task_verify = At_Put_down_1;
                xTaskResumeAll();
            }
            break;

        case At_Put_down_1: // 到达半加工区，进行激光校准
            if (abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) > 4 && abs(IDR_date_goal.IDR_Y - IDR_date_use.IDR_Y) > 4)
            {
                vTaskSuspendAll();
                back_sign = At_Put_down_1;
                Task_select = target_location;
                Task_verify = target_location;

                xTaskResumeAll();
            }
            else
            {
                vTaskSuspendAll();
                Arm_do_order = color_sign = servo_sign = 0;
                Task_select = At_Put_down_1_2;
                Task_verify = At_Put_down_1_2;
                xTaskResumeAll();
            }
            break;

        case At_Put_down_1_2: // 放置物料,色环
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
                    osDelay(servo_motion[11].time + 200);
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
                    osDelay(servo_motion[10].time + 200);
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
                    osDelay(servo_motion[9].time + 200);
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
                    osDelay(servo_motion[8].time + 200);
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
                    osDelay(servo_motion[20].time + 200);
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
                    servo_all_move(servo_motion[23]);
                    osDelay(servo_motion[23].time + 200);
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
                    osDelay(servo_motion[15].time + 200);
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
                    osDelay(servo_motion[14].time + 200);
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
                    osDelay(servo_motion[13].time + 200);
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
                    osDelay(servo_motion[12].time + 200);
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
                    osDelay(servo_motion[22].time + 200);
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
                    servo_all_move(servo_motion[25]);
                    osDelay(servo_motion[25].time + 200);
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
                    osDelay(servo_motion[19].time + 200);
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
                    osDelay(servo_motion[18].time + 200);
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
                    osDelay(servo_motion[17].time + 200);
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
                    osDelay(servo_motion[16].time + 200);
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
                    osDelay(servo_motion[21].time + 200);
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
                    servo_all_move(servo_motion[24]);
                    osDelay(servo_motion[24].time + 200);
                }
                break;

            default:
                servo_all_move(servo_motion[0]);
                osDelay(servo_motion[0].time + 500);
                Task_select = Task_verify = At_Put_down_1_3;
                Arm_do_order = color_sign = servo_sign = 0;
                break;
            }
            break;
        case At_Put_down_1_3: // 把放下的物料拿起来
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
                    osDelay(servo_motion[23].time + 200);
                    servo_all_move(servo_motion[27]);
                    osDelay(servo_motion[27].time + 200);
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
                    servo_all_move(servo_motion[20]);
                    osDelay(servo_motion[20].time + 200);
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
                    osDelay(servo_motion[8].time + 200);
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
                    osDelay(servo_motion[9].time + 200);
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
                    osDelay(servo_motion[10].time + 200);
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
                    osDelay(servo_motion[11].time + 200);
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
                    osDelay(servo_motion[25].time + 200);
                    servo_all_move(servo_motion[29]);
                    osDelay(servo_motion[29].time + 200);
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
                    servo_all_move(servo_motion[22]);
                    osDelay(servo_motion[22].time + 200);
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
                    osDelay(servo_motion[12].time + 200);
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
                    osDelay(servo_motion[13].time + 200);
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
                    osDelay(servo_motion[14].time + 200);
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
                    osDelay(servo_motion[15].time + 200);
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
                    osDelay(servo_motion[24].time + 200);
                    servo_all_move(servo_motion[28]);
                    osDelay(servo_motion[28].time + 200);
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
                    servo_all_move(servo_motion[21]);
                    osDelay(servo_motion[21].time + 200);
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
                    osDelay(servo_motion[16].time + 200);
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
                    osDelay(servo_motion[17].time + 200);
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
                    osDelay(servo_motion[18].time + 200);
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
                    osDelay(servo_motion[19].time + 200);
                }
                break;

            default:
                servo_all_move(servo_motion[0]);
                osDelay(servo_motion[0].time + 500);
                Task_select = Task_verify = To_Turn_2;
                break;
            }
            break;
        case To_Turn_2:
            osDelay(100);
            break;
        case target_location:
            if (abs(IDR_date_goal.IDR_Y - IDR_date_use.IDR_Y) > 4)
            {

                // IDR_speed_dert = symbol(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) *
                //                  (abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) > 20 ? 20 : abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X));
                IDR_speed_dert = IDR_date_goal.IDR_Y - IDR_date_use.IDR_Y;
                IDR_speed_sum2 += IDR_speed_dert;
                IDR_speed_sum2 = IDR_speed_sum2 > 2000 ? 1000 : (IDR_speed_sum2 < -2000 ? -2000 : IDR_speed_sum2);
                IDR_speed_use = IDR_speed_dert * 5 + IDR_speed_sum2 * 0.04;
                IDR_speed_use = IDR_speed_use > 100 ? 100 : (IDR_speed_use < -100 ? -100 : IDR_speed_use);
                crosswise_angle(0, imu_date, IDR_speed_use);
            }
            else if (abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) > 4)
            {

                // IDR_speed_dert = symbol(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) *
                //                  (abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) > 20 ? 20 : abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X));
                IDR_speed_dert = IDR_date_goal.IDR_X - IDR_date_use.IDR_X;
                IDR_speed_sum1 += IDR_speed_dert;
                IDR_speed_sum1 = IDR_speed_sum1 > 2000 ? 2000 : (IDR_speed_sum1 < -2000 ? -2000 : IDR_speed_sum1);
                IDR_speed_use = IDR_speed_dert * 5 + IDR_speed_sum1 * 0.04;
                IDR_speed_use = IDR_speed_use > 100 ? 100 : (IDR_speed_use < -100 ? -100 : IDR_speed_use);
                advance_angle(0, imu_date, IDR_speed_use);
            }
            else
            {
                speed_CTRL(0, 0, 0, 0);
                IDR_speed_sum1 = IDR_speed_sum2 = 0;
                osDelay(200);
                if (target_location_sign < 3)
                {
                    if (abs(IDR_date_goal.IDR_X - IDR_date_use.IDR_X) <= 4 && abs(IDR_date_goal.IDR_Y - IDR_date_use.IDR_Y) <= 4)
                        target_location_sign++;
                    break;
                }
                Task_select = back_sign;
                Task_verify = back_sign;
            }
            break;
        default:
            vTaskSuspend(NULL);
            break;
        }

        osDelay(10);
    }
    /* USER CODE END doing_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void show_QR(uint16_t num1, uint16_t num2)
{
    uint8_t ch[7];
    ch[0] = num1 / 100 + '0';
    ch[1] = num1 % 100 / 10 + '0';
    ch[2] = num1 % 10 + '0';
    ch[3] = '+';
    ch[4] = num2 / 100 + '0';
    ch[5] = num2 % 100 / 10 + '0';
    ch[6] = num2 % 10 + '0';

    HAL_UART_Transmit(&huart5, (uint8_t *)"main.QRcode.txt=\"", 16, 0xff);
    HAL_UART_Transmit(&huart5, ch, 7, 0xff);
    HAL_UART_Transmit(&huart5, (uint8_t *)"\"\xff\xff\xff", 4, 0xff);
}

void show_Task(uint8_t i)
{
    uint8_t ch[2];
    ch[0] = i;
    HAL_UART_Transmit(&huart5, (uint8_t *)"main.n0.val=", 12, 0xff);
    HAL_UART_Transmit(&huart5, ch, 1, 0xff);
    HAL_UART_Transmit(&huart5, (uint8_t *)"\xff\xff\xff", 3, 0xff);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
