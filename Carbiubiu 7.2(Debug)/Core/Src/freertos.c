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
#include "usb_user.h"
// #include "HI229.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Led_Toggle GPIOC->ODR ^= GPIO_PIN_13 // LED_Toggle�����궨��
#define IMU_speed 45                         // IMU���ݲɼ�����Ƶ�ʿ��Ʋ���
#define Wit_IMU
#define HWT101
#define turn_angle 0f

#define UNUSED_VARIABLE(X) ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)
#define symbol(y) (y >= 0 ? 1 : -1)


#define IMU_excursion   0.0f //-1.15f // ʮ�ּ����ʼ�������Ǻ󣬶����������ݵ�ƫ��ֵ


#define color_circle_x_RM 95   // �����ݴ����� ǰ- �� +
#define color_circle_y_RM -44  // ����ּӹ������ݴ���Ϊ-��ԭ����Ϊ+
//�ļ���ϰ�� �ּӹ�  rough machining

#define color_circle_x_TX 100   // �����ݴ����� 
#define color_circle_y_TX -39  // ����ּӹ������ݴ���Ϊ-��ԭ����Ϊ+
//�ļ���ϰ���ݴ� temporary storage

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern char s_cDataUpdate, s_cCmd; // MPU6050��ֲ���������ù�

extern servo_status servo_motion[];

extern Usart_DataTypeDef usart4;

uint8_t OS_status = 0; // �����ʼ����־λ��Ϊ1���ʼ��OK��Ϊ0��û����

uint8_t debug_date[600] = { 0 }; // debug���ݻ����������ڻ�����Ҫ�����ϴ���debug���ݡ�

uint8_t IMU_date[IMU_speed] = { 0 }; // MPU6050���ݻ�����������DMA+���ڷ�ʽ����

uint8_t usart_3_date[25] = { 0 };
uint8_t usart_6_date[22] = { 0 };

uint8_t servo_sign; // 1��ʾ�ϴζ������Ѿ�ִ����ϣ� 0��ʾ�ϴζ���������ִ��

uint16_t modetime = 100;

Usart_DataTypeDef* servoUsart = &usart4;

raw_t raw = { 0 };     /* IMU stram read/control struct */
uint8_t decode_succ; /* 0: no new frame arrived, 1: new frame arrived */
uint8_t Task_verify = Start;

uint32_t time_test[2] = { 0 };

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
SemaphoreHandle_t IMUdate_RX_Sem_Handle = NULL; // �ź�����ʼ����������ΪIMU���ݽ�����ɺ�Ļ��죬����IMU_TASK������ִ�С�

SemaphoreHandle_t usart3_RX_Sem_Handle = NULL;

SemaphoreHandle_t usart6_RX_Sem_Handle = NULL;

SemaphoreHandle_t Servo_Sem_Handle = NULL;

SemaphoreHandle_t Distance_Sem_Handle = NULL;

SemaphoreHandle_t key_debug_Sem_Handle = NULL;

SemaphoreHandle_t key_start_Sem_Handle = NULL;

SemaphoreHandle_t key_stop_Sem_Handle = NULL;

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
osThreadId myTask_usbHandle;
osThreadId myTask_usartHandle;
osThreadId myTask_oledHandle;
osThreadId myTask_servoHandle;
osThreadId myTask_speedHandle;
osThreadId myTask_pidHandle;
osThreadId myTask_doingHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const* argument);
void key_Task(void const* argument);       // һ��������һ������
void led_Task(void const* argument);       // LED��˸
void IMU_Task(void const* argument);       // ��ȡIMU����
void deubg_Task(void const* argument);     // һ������ģʽ
void usb_Task(void const* argument);        // δʵ��
void usart_Task(void const* argument);     // ��ȡ������������ݮ������
void oled_Task(void const* argument);      // δʵ��
void servo_Task(void const* argument);     // ���ƻ�е�۶�����
void speed_Task(void const* argument);     // ��ȡ�ĸ����ӱ�������ֵ����ͳ�����߾���
void pid_Task(void const* argument);       // ����ÿ�����ӵıջ�����
void doing_Task(void const* argument);     // ��������

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize)
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
    // IMUdate_RX_Sem_Handle��ֵ�ź�������
    IMUdate_RX_Sem_Handle = xSemaphoreCreateBinary();

    usart3_RX_Sem_Handle = xSemaphoreCreateBinary();

    usart6_RX_Sem_Handle = xSemaphoreCreateBinary();

    Servo_Sem_Handle = xSemaphoreCreateBinary();

    Distance_Sem_Handle = xSemaphoreCreateBinary();

    key_debug_Sem_Handle = xSemaphoreCreateBinary();

    key_start_Sem_Handle = xSemaphoreCreateBinary();

    key_stop_Sem_Handle = xSemaphoreCreateBinary();
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    Speed_Queue = xQueueCreate((UBaseType_t)1, /* ��Ϣ���еĳ��� */ // ʵʱ�ٶȲ���������Ϣ����
        (UBaseType_t)sizeof(biu));           /* ��Ϣ�Ĵ�С */
    goal_Queue = xQueueCreate((UBaseType_t)2, /* ��Ϣ���еĳ��� */  // Ŀ���ٶȴ�����Ϣ����
        (UBaseType_t)sizeof(biu));            /* ��Ϣ�Ĵ�С */
    IMU_Queue = xQueueCreate((UBaseType_t)1,                        /* ��Ϣ���еĳ��� */
        (UBaseType_t)sizeof(imu));             /* ��Ϣ�Ĵ�С */
    debug_Queue = xQueueCreate((UBaseType_t)1,                      /* ��Ϣ���еĳ��� */
        (UBaseType_t)sizeof(biu));           /* ��Ϣ�Ĵ�С */
    angle_Queue = xQueueCreate((UBaseType_t)1,                      /* ��Ϣ���еĳ��� */
        (UBaseType_t)sizeof(imu));           /* ��Ϣ�Ĵ�С */
    servo_Queue = xQueueCreate((UBaseType_t)1,                      /* ��Ϣ���еĳ��� */
        (UBaseType_t)sizeof(servo_status));  /* ��Ϣ�Ĵ�С */
    distance_Queue = xQueueCreate((UBaseType_t)1,                   /* ��Ϣ���еĳ��� */
        (UBaseType_t)sizeof(biu));        /* ��Ϣ�Ĵ�С */
    color_Queue = xQueueCreate((UBaseType_t)1,                      /* ��Ϣ���еĳ��� */
        (UBaseType_t)sizeof(biu_int_16));    /* ��Ϣ�Ĵ�С */
    IDR_date_Queue = xQueueCreate((UBaseType_t)1,                   /* ��Ϣ���еĳ��� */
        (UBaseType_t)sizeof(IDR_date));   /* ��Ϣ�Ĵ�С */
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
    osThreadDef(myTask_usb, usb_Task, osPriorityAboveNormal, 0, 128);
    myTask_usbHandle = osThreadCreate(osThread(myTask_usb), NULL);

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
    osThreadDef(myTask_speed, speed_Task, osPriorityRealtime, 0, 128);
    myTask_speedHandle = osThreadCreate(osThread(myTask_speed), NULL);

    /* definition and creation of myTask_pid */
    osThreadDef(myTask_pid, pid_Task, osPriorityRealtime, 0, 512);
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
 * @brief  ���ȼ���͵����񣬵���������棬����ͨ���۲�led��˸����۲��������������
 *          ������ȶ������������ȼ�����������������
 * @param  argument: Not used
 * @retval None
 */
 /* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const* argument)
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
 * @brief �����������ڼ�ⰴ��������û��ʹ���жϷ�ʽ�����ȶ�����ռ����Դ��������ʹ����ѯ��ʽ��Ȼ��nice
 * @param argument: Not used
 * @retval None
 */
 /* USER CODE END Header_key_Task */
void key_Task(void const* argument)
{
    /* USER CODE BEGIN key_Task */
    /* Infinite loop */

    for (;;)
    {

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
                xSemaphoreGive(key_start_Sem_Handle);
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
                xSemaphoreGive(key_start_Sem_Handle);
            }
        }
        if ((GPIOC->IDR & (1 << 1)) == 0)
        {
            osDelay(10);
            if ((GPIOC->IDR & (1 << 1)) == 0)
            {
                while ((GPIOC->IDR & (1 << 1)) == 0)
                {
                    osDelay(1);
                }
                /*do something*/
                xSemaphoreGive(key_stop_Sem_Handle);
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
                xSemaphoreGive(key_debug_Sem_Handle);
                vTaskResume(myTask_debugHandle);
                FSUS_DampingMode(servoUsart,1,600);
                osDelay(100);
                FSUS_DampingMode(servoUsart,2,600);
                osDelay(100);
                FSUS_DampingMode(servoUsart,3,600);
                osDelay(100);
                FSUS_DampingMode(servoUsart,4,600);
                osDelay(100);
                FSUS_DampingMode(servoUsart,6,600);
                osDelay(100);
            }
        }
        osDelay(1);
    }
    /* USER CODE END key_Task */
}

/* USER CODE BEGIN Header_led_Task */
/**
 * @brief RGB led����������ʾ��ǰ״̬
 * @param argument: Not used
 * @retval None
 */
 /* USER CODE END Header_led_Task */
void led_Task(void const* argument)
{
    /* USER CODE BEGIN led_Task */
    /* Infinite loop */
    GPIOC->ODR&=~GPIO_PIN_3;
    for (;;)
    {
        GPIOC->ODR ^= GPIO_PIN_4;
        osDelay(50);
    }
    /* USER CODE END led_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
 * @brief IMU���ݴ����������ȼ��ϸߣ����յ���ֵ��Ϣ�����ƣ�����ֵ��Ϣ�����ͷź󣬴�����ſ���ִ�У�
 * ��Ϣ���ͷ��ٶ��ܺ궨��IMU_speed���ơ�
 * @param argument: Not used
 * @retval None
 */
 /* USER CODE END Header_IMU_Task */
void IMU_Task(void const* argument)
{
    /* USER CODE BEGIN IMU_Task */
    /* Infinite loop */
#ifndef HWT101
    int i;
#endif
#ifdef HWT101
    uint8_t date_set_0[5]={0XFF,0XAA,0X76,0X00,0X00}; // Z��Ƕȹ�������
    HAL_UART_Transmit(&huart2,date_set_0,5,100);//Z��Ƕȹ���  
#endif
    imu imu_car;
    OS_status = 1;
    HAL_UART_Receive_DMA(&huart2, IMU_date, IMU_speed);
    for (;;)
    {
        // ��ȡ��ֵ�ź��� xSemaphore,û��ȡ����һֱ�ȴ�
        xSemaphoreTake(IMUdate_RX_Sem_Handle, portMAX_DELAY); /* �ȴ�ʱ�� */
        taskENTER_CRITICAL();
#ifdef Wit_IMU
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
#endif 
#ifdef HWT101
        imu_car.IMU[2] = Wit_HWT101_date_deal(IMU_date);
        if (imu_car.IMU[2] != 400)
        {
            imu_car.IMU[2] += IMU_excursion;
            xQueueSend(IMU_Queue, &imu_car, 0);
            xQueueSend(angle_Queue, &imu_car, 0);
        }
#endif // DEBUG

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
 * @brief С��״̬�ϴ�����ʹ�ô���һDMA���ͣ���������1000ms
 * @param argument: Not used
 * @retval None
 */
 /* USER CODE END Header_deubg_Task */
void deubg_Task(void const* argument)
{
    /* USER CODE BEGIN deubg_Task */
    /* Infinite loop */
    imu imu_car = { 0 };
    biu speed_now;
    biu speed_goal;
    float serve_angle[5]={0,0,0,0,0};
    uint8_t i=0;
    for (;;)
    {

//        xQueueReceive(IMU_Queue, &imu_car, 3);
//        xQueueReceive(debug_Queue, &speed_goal, 3);
//        xQueuePeek(Speed_Queue, &speed_now, 3);
        for(i=1;i<5;i++)
        {
            FSUS_QueryServoAngle(servoUsart,i,&serve_angle[i-1]);
            osDelay(10);
        }
        FSUS_QueryServoAngle(servoUsart,6,&serve_angle[4]);
        
//        sprintf((char*)debug_date, "%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d\n",
//            imu_car.IMU[0], imu_car.IMU[1], imu_car.IMU[2],
//            speed_now.date[0], speed_goal.date[0], speed_now.date[1], speed_goal.date[1],
//            speed_now.date[2], speed_goal.date[2], speed_now.date[3], speed_goal.date[3]);
        sprintf((char*)debug_date,"%3.1f,%3.1f,%3.1f,%3.1f,%3.1f\n",serve_angle[0],serve_angle[1],serve_angle[2],serve_angle[3],serve_angle[4]);
        HAL_UART_Transmit_DMA(&huart1, debug_date, strlen((char*)debug_date));       

        osDelay(50);
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
void usb_Task(void const* argument)
{
    /* USER CODE BEGIN motor_Task */
    /* Infinite loop */

    MX_USB_DEVICE_Init();
    uint8_t RX_Buffer[20];
    for (;;)
    {
        if (usb_vbc_Receive(RX_Buffer, 16, HAL_MAX_DELAY))
        {
            usb_printf("biubiu\n");
            osDelay(10);
            CDC_Transmit_FS(RX_Buffer, 16);

        }
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
void usart_Task(void const* argument)
{
    /* USER CODE BEGIN usart_Task */
    /* Infinite loop */

    taskENTER_CRITICAL();

    int i = 0, num, sign = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t wide = 0;
    int16_t height = 0;

    char* f = (char*)&x;
    char* h = (char*)&y;
    char* u = (char*)&wide;
    char* q = (char*)&height;
    biu_int_16 color_date;
    HAL_UART_Receive_DMA(&huart3, usart_3_date, 22);
    HAL_UART_Receive_DMA(&huart6, usart_6_date, 22);

    taskEXIT_CRITICAL();

    for (;;)
    {
        // USART3
        if (xSemaphoreTake(usart3_RX_Sem_Handle, 10) == pdTRUE) /* �ȴ�ʱ�� */
        {
            vTaskSuspendAll();

            for (i = 0; i < 8; i++)
            {
                if (usart_3_date[i] == 0X2C && usart_3_date[i + 10] == 0X5B)
                {
                    num = i;
                    if (usart_3_date[num + 1] == 0X12)
                        sign = 1; // ��ǰ��ȡ���������������ɫ
                    else if (usart_3_date[num + 1] == 0X21)
                        sign = 2; // ��ǰ��ȡ���������Ƕ�ά��ֵ
                    else if (usart_3_date[num + 1] == 0X3C)
                        sign = 3; // ��ǰ��ȡ����������ɫ����λ����
                    else
                        sign = 0;
                    break;
                }
                sign = 0;
            }
            // HAL_UART_Transmit_DMA(&huart1, usart_3_date, 22);
            if (sign == 1) // ʶ�������ɫ
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
            else if (sign == 2) // ��ȡ��ά��ֵ
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
            }
            else if (sign == 3) // ɫ����λ����
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
//      // USART6
        if (xSemaphoreTake(usart6_RX_Sem_Handle, 10) == pdTRUE) /* �ȴ�ʱ�� */
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
float adc_vlaue = 0.0f;
void oled_Task(void const* argument)
{
    /* USER CODE BEGIN oled_Task */
    /* Infinite loop */
    uint32_t ADC_value = 0;

    HAL_ADC_Start_DMA(&hadc1, &ADC_value, 1);
    GPIOD->ODR |= GPIO_PIN_7;
    osDelay(1000);
    GPIOD->ODR &= ~GPIO_PIN_7;
    for (;;)
    {
        adc_vlaue = (float)ADC_value * 3.3f * 11 / 4096.0f;
        if (adc_vlaue < 11.4f)
            GPIOD->ODR |= GPIO_PIN_7;
        else
            GPIOD->ODR &= ~GPIO_PIN_7;
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
void servo_Task(void const* argument)
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
        osDelay(10);
        FSUS_SetServoAngle(servoUsart, 1, servo.value_goal[0], time, 0, 0);
        osDelay(10);
        FSUS_SetServoAngle(servoUsart, 2, servo.value_goal[1], time, 0, 0);
        osDelay(10);
        FSUS_SetServoAngle(servoUsart, 3, servo.value_goal[2], time, 0, 0);
        osDelay(10);
        FSUS_SetServoAngle(servoUsart, 4, servo.value_goal[3], time, 0, 0);
        osDelay(10);

        if (time > 300)
            osDelay(time - 300);
        else
            osDelay(10);
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
void speed_Task(void const* argument)
{
    /* USER CODE BEGIN speed_Task */
    /* Infinite loop */
    biu speed;
    static biu distance = { 0, 0, 0, 0 };
    uint8_t i = 0;
    for (;;)
    {
        if (xSemaphoreTake(Distance_Sem_Handle, 0) == pdTRUE)
        {
            for (i = 0; i < 4; i++)
            {
                distance.date[i] = 0;
            }
        }
        vTaskSuspendAll();
        speed.date[0] = TIM2->CNT - 6720;
        TIM2->CNT = 6720;

        speed.date[1] = 6720 - TIM3->CNT;
        TIM3->CNT = 6720;

        speed.date[2] = TIM4->CNT - 6720;
        TIM4->CNT = 6720;

        speed.date[3] = 6720 - TIM5->CNT;
        TIM5->CNT = 6720;
        for (i = 0; i < 4; i++)
        {
            distance.date[i] += abs(speed.date[i]);
        }
        xTaskResumeAll();
        xQueueSend(Speed_Queue, &speed, 0); /* �ȴ�ʱ�� 0 */
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
void pid_Task(void const* argument)
{
    /* USER CODE BEGIN pid_Task */
    /* Infinite loop */
    biu speed_goal = { 0 };
    biu speed_now = { 0 };
    speed_ctrl(Motor1, 0);
    speed_ctrl(Motor2, 0);
    speed_ctrl(Motor3, 0);
    speed_ctrl(Motor4, 0);
    for (;;)
    {
        xQueueReceive(Speed_Queue, /* ��Ϣ���еľ�� */
            &speed_now,  /* ���͵���Ϣ���� */
            5);          /* �ȴ�ʱ�� */
        xQueueReceive(goal_Queue,  /* ��Ϣ���еľ�� */
            &speed_goal, /* ���͵���Ϣ���� */
            5);          /* �ȴ�ʱ�� */

        taskENTER_CRITICAL();

        pid_do(speed_goal, speed_now);

        taskEXIT_CRITICAL();

        osDelay(5);

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
void doing_Task(void const* argument)
{
    /* USER CODE BEGIN doing_Task */
    /* Infinite loop */

    imu imu_date;            // ���������ݽṹ��
    static biu distance_now; // �����¼�ṹ��
    biu_int_16 color_date;

    float angle_standard = 0.0f, Direction_KP = 4.0f, Direction_KI = 0.2f;
    static int back_sign = 0, color_speed_use = 0, color_speed_sum1 = 0, color_speed_sum2 = 0, color_speed_dert = 0;
    static int16_t distance_use = 0, color_circle_x = 0, color_circle_y = 0;

    static uint8_t Task_select = Start,
        color_sign = 0,    // 0��ʾ��ǰ���ڴ�ԭ������ȡ��һ�����ϣ�1��ʾ��ǰ���ڴ�ԭ������ȡ�ڶ������ϣ�2��ʾ��ǰ���ڴ�ԭ������ȡ����������,3��ʾ��ȡ�Ѿ�����
        use_num = 0;        // ִ�е��Ȳ���
    servo_status servo_time_use = { 0 };

    int8_t color[6] = { 1, 2, 3, 3, 2, 1 }; // ��ɫ˳��洢

    uint8_t Arm_do_order = 0, // ��ʾ��Ҫִ�ж�����ı��
        color_check,
        color_first_find = 0,
        Camera_date_status = 0, // ����ͷ���ݽ��ձ�־λ��������������յ���ȷ������ͷ����
        // ��ͨ��color_quene��Ϣ���д������ݣ�doing_task��ѯ���գ����ı��־λ��
        color_sign_now,             // ��ʾ��ǰԭ����ɨ���⵽�������ɫ
        target_location_sign = 0,
        Do_thing_status = 0,
        i,
        color_location_sign = 0,//ɫ����λ��־λ
        angle_num = 0,
        start_status = 0,
        distance_sign=0;

    Uart5_LCD_send_task(0);           // LCD��Ļ��ʾ��ǰ����Ϊ��ʼ
    vTaskSuspend(myTask_debugHandle); // �����������
    vTaskSuspend(myTask_ledHandle);   // ����Led����
    UNUSED_VARIABLE(distance_use);
    Task_verify = Task_select = Start; // ���õ�ǰ����Ϊ��ʼ
    speed_CTRL(0, 0, 0, 0);            // �ٶȹ���

    for (;;)
    {

        if (xQueueReceive(angle_Queue, &imu_date, 1) == pdTRUE) // ����ƫ��������
        {
            angle_num++;
            if (angle_num > 5) // ÿ250�������һ��LCD��Ļ�ϵ�ƫ��������
            {
                angle_num = 0;
                Uart5_LCD_show_Angle(imu_date.IMU[2]);
            }
        }

        if (xQueueReceive(distance_Queue, &distance_now, 1) == pdTRUE) // ���յ�ǰ�������߹��ľ���
        {
            distance_sign=1;
        }
        if (xQueueReceive(color_Queue, &color_date, 10) == pdTRUE) // ��ȡ��ɫ˳��
        {
            Camera_date_status = 1;
            if (color_date.date[2] == 33)
            {
                use_num++;
                if (use_num > 2)
                {
                    Uart5_LCD_show_X_Y(color_date.date[0], color_date.date[1]);
                    use_num = 0;
                }
            }
        }

        if (xSemaphoreTake(Servo_Sem_Handle, 10) == pdTRUE) // �ϴζ������Ѿ�ִ�����
        {
            servo_sign = 1;
        }
        if (xSemaphoreTake(key_start_Sem_Handle, 10) == pdTRUE) // ����������������
        {
            osDelay(1000);
            GM65_work(); // ����ָ�GM65ִ�ж�ά��ɨ��
            start_status = 1;
            vTaskResume(myTask_ledHandle); // LED��˸��������
        }
        if (xSemaphoreTake(key_stop_Sem_Handle, 10) == pdTRUE)  // ��������ֹͣ����
        {
            start_status = 0;
            vTaskSuspend(myTask_ledHandle);
        }
        if (xSemaphoreTake(key_debug_Sem_Handle, 10) == pdTRUE) // ����������������
        {
            vTaskSuspend(NULL);
        }
        if (start_status == 0)
        {
            if (i > 100)
            {
                Camera_die(); // ÿ1s����һ�ιر���ݮ��ʶ������
                i = 0;
            }
            i++;
            speed_CTRL(0, 0, 0, 0);            // �ٶȹ���
            osDelay(10);
            continue; // �˳�����ѭ��������ִ����������д���
        }


        Task_select = Task_select == Task_verify ? Task_select : Task_verify; // У������ѡ�������û�г���

        switch (Task_select)
        {
        case Start:// ��ʼ������
            if(distance_sign==1)
            {
                if(Directional_move_distance(angle_standard, imu_date, distance_now, 800,100)==1)
                {
                    speed_CTRL(0, 0, 0, 0);
                    Task_select = Task_verify = At_QR_Code; // ��ת����
                    Uart5_LCD_show_string("To_QR_Code");
                    GM65_work(); // ����ָ�GM65ִ�ж�ά��ɨ��
                }    
                distance_sign=0;
            } 
            break;

        case Start_Calibration:                               // ��һ�η���У׼
            if (direction_Set_biu(angle_standard, imu_date) == 1) // ����У׼
            {
                speed_CTRL(0, 0, 0, 0); // ɲ��

                vTaskSuspendAll();        // �ٽ���
//                Task_select = To_QR_Code; // ��ת����
//                Task_verify = To_QR_Code;
                GM65_work(); // ����ָ�GM65ִ�ж�ά��ɨ��
//                servo_all_move(servo_motion[2]); // ��е�۶�������
                xTaskResumeAll();
            }
            break;

        case To_QR_Code:                                                                  // ǰ������ά������
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 1000) == 1) // ǰ�� 440 mm�����ά��ɨ����
            {
                speed_CTRL(0, 0, 0, 0);   // ɲ��
                vTaskSuspendAll();        // �����ٽ���
                Task_select = At_QR_Code; // ������ת
                Task_verify = At_QR_Code;
                Uart5_LCD_show_string("At_QR_Code");
                xTaskResumeAll();
            }
            break;

        case At_QR_Code:                 // ɨ���ά��
            if (Camera_date_status == 0) // �������ͷ��־λδ����
            {
                GM65_work(); // ����ָ��ʹ��GM65
                osDelay(200);
            }
            else
            {
                if (color_date.date[2] != 22) // ����յ�������������
                {
                    Camera_die(); // �ػ���������
                    Camera_date_status = 0;
                }
                else // ûɶ����
                {
                    speed_CTRL(0, 0, 0, 0);                            // ɲ��
                    vTaskSuspendAll();                                 // �ٽ���
                    color[0] = (color_date.date[3] & (3 << 10)) >> 10; // ���ݶ�ȡ
                    color[1] = (color_date.date[3] & (3 << 8)) >> 8;
                    color[2] = (color_date.date[3] & (3 << 6)) >> 6;
                    color[3] = (color_date.date[3] & (3 << 4)) >> 4;
                    color[4] = (color_date.date[3] & (3 << 2)) >> 2;
                    color[5] = (color_date.date[3] & (3 << 0)) >> 0;
                    Uart5_LCD_show_QR(color[0], color[1], color[2], color[3], color[4], color[5]); // LCD��ʾ��ά������
                    xTaskResumeAll();
                    Camera_die(); // �ػ���
                    osDelay(10);
                    vTaskSuspendAll();
                    Task_select = To_RMA; // ������ת
                    Task_verify = To_RMA;

                    Camera_date_status = 0; // ����ͷ��־λ����
                    Uart5_LCD_show_string("To_RMA");
//                    servo_all_move(servo_motion[0]); // ��е�ۻ���
                    xTaskResumeAll();
                }
            }
            break;
        case To_RMA:                                                                      // ǰ������ԭ����
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 845) == 1) // ǰ�� 845 mm  , ��ԭ���������޸�ʱ����һת�Ǿ���Ҳ��Ҫ�޸�
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
        case At_RMA1: // �ﵽԭ���������з���У׼
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
//                Task_select = At_RMA;
//                Task_verify = At_RMA;
                Uart5_LCD_show_string("Do_capture");
                color_first_find = 0;
                xTaskResumeAll();
//                servo_all_move(servo_motion[3]);
                osDelay(servo_motion[3].time + 70);
//                servo_all_move(servo_motion[26]);
                osDelay(servo_motion[26].time + 70);
            }
            break;
        case At_RMA:                     // ����ͷ�����ж�У��
            if (Camera_date_status == 0) // �ж���������ͷ��Ϣ
            {
                osDelay(200);
                Camera_Color_Find_do(); // ��ʼɨ�������ɫ
            }
            else
            {
                if (color_date.date[2] == 11) // У��
                {

                    if (color[0] == color_date.date[1] && color_first_find == 0)
                    {
                        osDelay(200);
                    }
                    else
                    {
                        speed_CTRL(0, 0, 0, 0);
                        vTaskSuspendAll();
                        Task_select = Task_verify = Take_color_thing; // ������ת
                        Uart5_LCD_show_string("Take_color_thing");

                        servo_sign = 0;
                        Arm_do_order = 2;
                        xTaskResumeAll();
                    }
                }
                else // У�鲻ͨ��
                {
                    Camera_die();
                    Camera_date_status = 0;
                    osDelay(200);
                }
            }

            break;
        case Take_color_thing: // ��ԭ���� ��ȡ��������
        {
            switch (Arm_do_order) // ��е�۶���ִ�б�־λ
            {
            case 0:
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    servo_time_use = servo_motion[0];
                    servo_time_use.time = 200;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
                    servo_all_move(servo_motion[3]);
                    osDelay(servo_motion[3].time + 70);
                }
                break;
            case 1: // ץȡ��һ��
                if (servo_sign == 1)
                {
                    servo_sign = 0;
                    Arm_do_order++;
                }
                else
                {
                    if (color_sign == 3)
                    {
                        Arm_do_order = 250; // �������������
                    }
                    else
                    {
                        servo_all_move(servo_motion[26]);
                        osDelay(servo_motion[26].time + 100);
                        xQueueReceive(color_Queue, &color_date, 10);
                        Camera_date_status = 0;
                        color_date.date[1] = 0;
                    }
                }
                break;
            case 2:
                if (Camera_date_status == 1)
                {
                    if (color[color_sign] == color_date.date[1])
                    {
                        if (color_check > 4)
                        {
                            // ������900΢��ɨ�赽�Ķ��������ɫ����ִ������Ĵ���
                            color_sign++;
                            Arm_do_order++;
                            color_sign_now = color_date.date[1];
                            color_check = 0;
                        }
                        else
                        {
                            //                        printf("%d,%d\n", color[color_sign], color_date.date[1]);
                            color_date.date[1] = 0;
                            color_check++;
                            osDelay(100);
                        }
                    }
                    else
                    {
                        color_check = 0;
                        osDelay(100);
                        //                    printf("%d,%d\n", color[color_sign], color_date.date[1]);
                    }
                    Camera_date_status = 0;
                }
                break;
            case 3: // ץȡ�ڶ���
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
            case 4: // ץȡ������
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
            case 5: // ץȡ���Ĳ�
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
                case 1: // ʶ�𵽵ĺ�ɫ���Ϸ���
                    Arm_do_order = 11;
                    break;
                case 2: // ʶ�𵽵���ɫ���Ϸ���
                    Arm_do_order = 33;
                    break;
                case 3: // ʶ�𵽵���ɫ���Ϸ���
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
                Task_select = Task_verify = To_Turn_1;
                Camera_die();
                Uart5_LCD_show_string("To_Turn_1");
                break;
            }
        }
        break;

        case To_Turn_1: // ֱ��ǰ��
            Uart5_LCD_show_string("To_Turn_1");
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 118) == 1) // ��ǰǰ��ת��1
            {
                Uart5_LCD_show_string("To_Put_down_1");
                Task_select = Task_verify = To_Put_down_1;
            }
            break;
        case To_Put_down_1:                                                                  // ���дﵽ�ּӹ���
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, -636) == 1) // ����1
            {

                vTaskSuspendAll();
                Task_select = Task_verify = Angle_direction_2;
                xTaskResumeAll();
            }
            break;
        case Angle_direction_2: // �ڴּӹ������з������
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = At_Put_down_1_1;
                Task_verify = At_Put_down_1_1;
                //                Task_select = At_Put_down_1_2;
                //                Task_verify = At_Put_down_1_2;

                color_location_sign = 0;
                Arm_do_order = color_sign = servo_sign = 0;
                if (Do_thing_status == 1)
                    color_sign = 3;

                xTaskResumeAll();
            }
            break;

        case At_Put_down_1_1: // �ּӹ����Ӿ���λ
        {
            if (color_location_sign == 0)
            {
                Camera_color_circle_location();
                servo_all_move(servo_motion[45]);
                osDelay(servo_motion[45].time + 100);
                vTaskSuspendAll();
                back_sign = At_Put_down_1_1;
                if (Camera_date_status == 1 && color_date.date[2] == 33)
                {
                    color_circle_x = color_circle_x_RM;
                    color_circle_y = color_circle_y_RM;
                    Task_select = Color_cicle_location;
                    Task_verify = Color_cicle_location;
                    back_sign = At_Put_down_1_1;
                }
                else
                {
                    Camera_color_circle_location();
                }

                xTaskResumeAll();
            }
            else
            {
                vTaskSuspendAll();
                color_location_sign = 0;
                Arm_do_order = color_sign = servo_sign = 0;
                if (Do_thing_status == 1)
                    color_sign = 3;
                Task_select = At_Put_down_1_2;
                Task_verify = At_Put_down_1_2;
                xTaskResumeAll();
            }
        }
        break;
        case At_Put_down_1_2: // �ڴּӹ��� ��������,ɫ��
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
                            servo_time_use = servo_motion[0];
                            servo_time_use.time = 200;
                            servo_all_move(servo_time_use);
                            osDelay(servo_time_use.time + 70);
                        }
                    }
                    else
                    {
                        if (color_sign == 6)
                            Arm_do_order = 250;
                        else
                        {
                            servo_time_use = servo_motion[0];
                            servo_time_use.time = 200;
                            servo_all_move(servo_time_use);
                            osDelay(servo_time_use.time + 70);
                        }
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1: // ��ɫ
                    Arm_do_order = 11;
                    color_sign++;
                    break;
                case 2: // ��ɫ
                    Arm_do_order = 33;
                    color_sign++;
                    break;
                case 3: // ��ɫ
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
                    servo_time_use = servo_motion[11];
                    servo_time_use.time = 800;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
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

                    servo_all_move(servo_motion[47]);
                    osDelay(servo_motion[47].time + 70);
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
                    osDelay(servo_motion[42].time + 200);

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
                    servo_time_use = servo_motion[15];
                    servo_time_use.time = 800;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
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

                    servo_all_move(servo_motion[48]);
                    osDelay(servo_motion[48].time + 70);
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
                    osDelay(servo_motion[44].time + 200);
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
                    servo_time_use = servo_motion[19];
                    servo_time_use.time = 800;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
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

                    servo_all_move(servo_motion[47]);
                    osDelay(servo_motion[47].time + 70);
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
                    osDelay(servo_motion[43].time + 200);
                    servo_all_move(servo_motion[24]);
                    osDelay(servo_motion[24].time + 70);
                }
                break;

            default:
                servo_time_use = servo_motion[0];
                servo_time_use.time = 200;
                servo_all_move(servo_time_use);
                osDelay(servo_time_use.time + 70);
                Task_select = Task_verify = At_Put_down_1_3;
                Arm_do_order = color_sign = servo_sign = 0;
                if (Do_thing_status == 1)
                {
                    color_sign = 3;
                }
                break;
            }
            break;
        case At_Put_down_1_3: // �ڴּӹ����ѷ��µ�����������
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
                            servo_time_use = servo_motion[0];
                            servo_time_use.time = 200;
                            servo_all_move(servo_time_use);
                            osDelay(servo_time_use.time + 70);
                        }
                    }
                    else
                    {
                        if (color_sign == 6)
                            Arm_do_order = 250;
                        else
                        {
                            servo_time_use = servo_motion[0];
                            servo_time_use.time = 200;
                            servo_all_move(servo_time_use);
                            osDelay(servo_time_use.time + 70);
                        }
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1:                // ��ɫ
                    Arm_do_order = 11; // ����ִ��˳��23 20 8 9 10 11
                    color_sign++;

                    break;
                case 2:                // ��ɫ
                    Arm_do_order = 33; // ����ִ��˳��:25 22 12 13 14 15
                    color_sign++;

                    break;
                case 3:                // ��ɫ
                    Arm_do_order = 22; // ����ִ��˳��24 21 16 17 18 19
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
                    printf("������ɫ\n");
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
                    servo_time_use = servo_motion[8];
                    servo_time_use.time = 1000;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
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
                    servo_all_move(servo_motion[46]);
                    osDelay(servo_motion[46].time + 70);
                    //                    printf("������ɫ\n");
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
                    servo_all_move(servo_motion[38]);//��צ�ջض���
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
                    servo_time_use = servo_motion[12];
                    servo_time_use.time = 1000;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
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
                    servo_time_use = servo_motion[16];
                    servo_time_use.time = 1000;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
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
                servo_time_use = servo_motion[0];
                servo_time_use.time = 200;
                servo_all_move(servo_time_use);
                osDelay(servo_time_use.time + 70);
                Task_select = Task_verify = To_Turn_2;
                break;
            }
            break;
        case To_Turn_2:                                                                      // ����ǰ���ݴ���
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, -511) == 1) //
            {
                vTaskSuspendAll();
                if (Do_thing_status == 1)
                {
                    Task_select = Task_verify = At_Turn_2_angle;
                }
                else
                    Task_select = Task_verify = At_Turn_2;
                xTaskResumeAll();
            }
            break;
        case At_Turn_2_angle: // �ڴּӹ������з������
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Task_select = At_Turn_2;
                Task_verify = At_Turn_2;
                xTaskResumeAll();
            }

            break;
        case At_Turn_2:                                                                    // ֱ��ǰ�������ݴ���
            if (advance_angle_distance(angle_standard, imu_date, distance_now, -605) == 1) //
            {
                vTaskSuspendAll();
                angle_standard = 90.2f;
                Task_select = Task_verify = To_Put_down_2;
                xTaskResumeAll();
            }
            break;
        case To_Put_down_2: // ���ݴ�����������Ƕ� 
            if (direction_Set_biu(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                osDelay(100);
                vTaskSuspendAll();
                Task_select = To_Put_down_3;
                Task_verify = To_Put_down_3;
                color_location_sign = 0;
                color_date.date[2] = 0;
                Camera_date_status = 0;
                xTaskResumeAll();
                xQueueReceive(angle_Queue, &imu_date, portMAX_DELAY);
            }
            break;
        case To_Put_down_3: // ���ݴ�����������Ƕ� 
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                osDelay(100);

                vTaskSuspendAll();

                Task_select = At_Put_down_2_0;
                Task_verify = At_Put_down_2_0;
                color_location_sign = 0;
                Arm_do_order = color_sign = servo_sign = 0;
                color_date.date[2] = 0;
                Camera_date_status = 0;
                if (Do_thing_status == 1)
                {
                    Arm_do_order = color_sign = servo_sign = 0;
                    color_sign = 3;
                    Task_select = Stacking;
                    Task_verify = Stacking;
                }
                xTaskResumeAll();

            }
            break;
        case At_Put_down_2_0: // �ݴ����Ӿ���λ
        {
            if (color_location_sign == 0)
            {
                Camera_color_circle_location();
                servo_all_move(servo_motion[45]);
                osDelay(servo_motion[45].time + 100);
                vTaskSuspendAll();
                back_sign = At_Put_down_2_0;
                if (Camera_date_status == 1 && color_date.date[2] == 33)
                {
                    color_circle_x = color_circle_x_TX;
                    color_circle_y = color_circle_y_TX;
                    Task_select = Color_cicle_location;
                    Task_verify = Color_cicle_location;
                    back_sign = At_Put_down_2_0;
                }
                else
                {
                    Camera_color_circle_location();
                }

                xTaskResumeAll();
            }
            else
            {
                vTaskSuspendAll();
                color_location_sign = 0;

                if (Do_thing_status == 1)
                {
                    color_sign = 3;
                    Task_select = Stacking;
                    Task_verify = Stacking;
                }
                else
                {
                    Arm_do_order = color_sign = servo_sign = 0;
                    Task_select = At_Put_down_2_1;
                    Task_verify = At_Put_down_2_1;
                }
                xTaskResumeAll();
            }
            break;
        }

        case At_Put_down_2_1: //�ݴ�������
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
                        servo_time_use = servo_motion[0];
                        servo_time_use.time = 800;
                        servo_all_move(servo_time_use);
                        osDelay(servo_time_use.time + 70);
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1: // ��ɫ
                    Arm_do_order = 11;
                    color_sign++;
                    break;
                case 2: // ��ɫ
                    Arm_do_order = 33;
                    color_sign++;
                    break;
                case 3: // ��ɫ
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

                    servo_all_move(servo_motion[47]);
                    osDelay(servo_motion[47].time + 70);
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


                    servo_all_move(servo_motion[48]);
                    osDelay(servo_motion[48].time + 70);
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

                    servo_all_move(servo_motion[47]);
                    osDelay(servo_motion[47].time + 70);
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
                servo_time_use = servo_motion[0];
                servo_time_use.time = 200;
                servo_all_move(servo_time_use);
                osDelay(servo_time_use.time + 70);
                Task_select = Task_verify = AT_Put_down_2_2;
                angle_standard = 0.0;
                Arm_do_order = servo_sign = 0;
                break;
            }
            break;
        case AT_Put_down_2_2:
            if (direction_Set_biu(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Uart5_LCD_show_string("Back_Take_thing");
                Task_select = AT_Put_down_2_3;
                Task_verify = AT_Put_down_2_3;
                xTaskResumeAll();
                xQueueReceive(angle_Queue, &imu_date, portMAX_DELAY);
            }
            break;
        case AT_Put_down_2_3:
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
        case Back_Take_thing_1:                             //ǰ���뿪�ݴ���
            if (advance_angle_distance(angle_standard, imu_date, distance_now, 640) == 1) //
            {

                vTaskSuspendAll();
                Task_select = Task_verify = Back_Take_thing_2;
                xTaskResumeAll();
            }
            break;
        case Back_Take_thing_2:                            //����ǰ��ԭ����
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, 1134) == 1) //
            {

                vTaskSuspendAll();
                Task_select = Task_verify = Back_Take_thing_3;
                xTaskResumeAll();
            }
            break;

        case Back_Take_thing_3:                            //�����һ��
            if (advance_angle_distance(angle_standard, imu_date, distance_now, -190) == 1) //
            {

                vTaskSuspendAll();
                Do_thing_status = 1;
                Task_select = Task_verify = Angle_direction_3;
                xTaskResumeAll();
            }
            break;

        case Angle_direction_3: // �ڶ�����ԭ���������Ƕ�
            if (direction_Set(angle_standard, imu_date) == 1)
            {
                speed_CTRL(0, 0, 0, 0);
                vTaskSuspendAll();
                Do_thing_status = 1;
                color_first_find = 0;
                Task_select = Take_color_thing_2;
                Task_verify = Take_color_thing_2;
                xTaskResumeAll();
            }
            break;

        case Take_color_thing_2: // �ڶ���ȥԭ������ȡ����
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
                    servo_time_use = servo_motion[0];
                    servo_time_use.time = 200;
                    servo_all_move(servo_time_use);
                    osDelay(servo_time_use.time + 70);
                    servo_all_move(servo_motion[3]);
                    osDelay(servo_motion[3].time + 100);
                }
                break;
            case 1: // ץȡ��һ��
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
                        xQueueReceive(color_Queue, &color_date, 10);
                        Camera_date_status = 0;
                        color_date.date[1] = 0;
                    }
                }
                break;
            case 2:
                if (Camera_date_status == 1)
                {
                    if (color[color_sign] == color_date.date[1] && color_sign == 3 && color_first_find == 0)
                    {
                        osDelay(100);
                        Camera_date_status = 0;
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
                    Camera_date_status = 0;

                }
                else
                {
                    Camera_Color_Find_do();
                    osDelay(200);
                }
                break;
            case 3: // ץȡ�ڶ���
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
            case 4: // ץȡ������
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
            case 5: // ץȡ���Ĳ�
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
                case 1: // ʶ�𵽵ĺ�ɫ���Ϸ���
                    Arm_do_order = 11;
                    break;
                case 2: // // ʶ�𵽵���ɫ���Ϸ���
                    Arm_do_order = 33;
                    break;
                case 3: // ʶ�𵽵���ɫ���Ϸ���
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
        case Stacking: // ���
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
                            servo_time_use = servo_motion[0];
                            servo_time_use.time = 800;
                            servo_all_move(servo_time_use);
                            osDelay(servo_time_use.time + 70);
                        }
                    }
                    else
                    {
                        if (color_sign == 6)
                            Arm_do_order = 250;
                        else
                        {
                            servo_time_use = servo_motion[0];
                            servo_time_use.time = 800;
                            servo_all_move(servo_time_use);
                            osDelay(servo_time_use.time + 70);
                        }
                    }
                }
                break;
            case 1:
                switch (color[color_sign])
                {
                case 1: // ��ɫ
                    Arm_do_order = 11;
                    color_sign++;
                    break;
                case 2: // ��ɫ
                    Arm_do_order = 33;
                    color_sign++;
                    break;
                case 3: // ��ɫ
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
            if (crosswise_angle_distance(angle_standard, imu_date, distance_now, -780) == 1) //
            {

                vTaskSuspendAll();
                Uart5_LCD_show_string("Back_home");
                Task_select = Task_verify = Back_home_2;
                xTaskResumeAll();
            }
            break;
        case Back_home_2:
            if (advance_angle_distance(angle_standard, imu_date, distance_now, -1290) == 1) //
            {

                vTaskSuspendAll();
                Task_select = Task_verify = End;
                xTaskResumeAll();
            }
            break;

        case Color_cicle_location:  // �ж���ɫȦλ��
            if (Camera_date_status == 1)  // ����������״̬Ϊ1
            {
                if (abs(color_date.date[0] - color_circle_x) > 2)  // �����ɫ���ݵĵ�һ��Ԫ������ɫȦ��x����Ĳ�ľ���ֵ����2
                {
                    color_speed_dert = color_date.date[0] - color_circle_x;  // ��ɫ�ٶȲ������ɫ���ݵĵ�һ��Ԫ�ؼ�ȥ��ɫȦ��x����
                    color_speed_dert = abs(color_speed_dert) > 5 ? color_speed_dert : color_speed_dert * 5;  // �����ɫ�ٶȲ�ľ���ֵ����5������ɫ�ٶȲ�䣬������ɫ�ٶȲ����5
                    color_speed_sum2 += color_speed_dert;  // ��ɫ�ٶȺ͵�����ɫ�ٶȺͼ�����ɫ�ٶȲ�
                    color_speed_sum2 = color_speed_sum2 > 2000 ? 2000 : (color_speed_sum2 < -2000 ? -2000 : color_speed_sum2);  // �����ɫ�ٶȺʹ���2000������ɫ�ٶȺ͵���2000�����������ɫ�ٶȺ�С��-2000������ɫ�ٶȺ͵���-2000��������ɫ�ٶȺͲ���
                    color_speed_use = color_speed_dert * Direction_KP + color_speed_sum2 * Direction_KI;  // ʹ�õ���ɫ�ٶȵ�����ɫ�ٶȲ���Է���KP������ɫ�ٶȺͳ��Է���KI
                    color_speed_use = color_speed_use > 1000 ? 1000 : (color_speed_use < -1000 ? -1000 : color_speed_use);  // ���ʹ�õ���ɫ�ٶȴ���1000����ʹ�õ���ɫ�ٶȵ���1000���������ʹ�õ���ɫ�ٶ�С��-1000����ʹ�õ���ɫ�ٶȵ���-1000������ʹ�õ���ɫ�ٶȲ���
                    advance_angle(imu_date.IMU[2], imu_date, color_speed_use);  // ����advance_angle����������Ϊimu_date�ĵ�����Ԫ�أ�imu_date��ʹ�õ���ɫ�ٶ�
                }
                else if (abs(color_date.date[1] - color_circle_y) > 2)  // ���������ɫ���ݵĵڶ���Ԫ������ɫȦ��y����Ĳ�ľ���ֵ����2
                {
                    color_speed_sum2 = 0;  // ��ɫ�ٶȺ͵���0
                    color_speed_dert = color_circle_y - color_date.date[1];  // ��ɫ�ٶȲ������ɫȦ��y�����ȥ��ɫ���ݵĵڶ���Ԫ��
                    color_speed_dert = abs(color_speed_dert) > 5 ? color_speed_dert : color_speed_dert * 5;  // �����ɫ�ٶȲ�ľ���ֵ����5������ɫ�ٶȲ�䣬������ɫ�ٶȲ����5
                    color_speed_sum1 += color_speed_dert;  // ��ɫ�ٶȺ͵�����ɫ�ٶȺͼ�����ɫ�ٶȲ�
                    color_speed_sum1 = color_speed_sum1 > 2000 ? 2000 : (color_speed_sum1 < -2000 ? -2000 : color_speed_sum1);  // �����ɫ�ٶȺʹ���2000������ɫ�ٶȺ͵���2000�����������ɫ�ٶȺ�С��-2000������ɫ�ٶȺ͵���-2000��������ɫ�ٶȺͲ���
                    color_speed_use = color_speed_dert * Direction_KP + color_speed_sum1 * Direction_KI;  // ʹ�õ���ɫ�ٶȵ�����ɫ�ٶȲ���Է���KP������ɫ�ٶȺͳ��Է���KI
                    color_speed_use = color_speed_use > 1000 ? 1000 : (color_speed_use < -1000 ? -1000 : color_speed_use);  // ���ʹ�õ���ɫ�ٶȴ���1000����ʹ�õ���ɫ�ٶȵ���1000���������ʹ�õ���ɫ�ٶ�С��-1000����ʹ�õ���ɫ�ٶȵ���-1000������ʹ�õ���ɫ�ٶȲ���
                    crosswise_angle(imu_date.IMU[2], imu_date, color_speed_use);  // ����crosswise_angle����������Ϊimu_date�ĵ�����Ԫ�أ�imu_date��ʹ�õ���ɫ�ٶ�
                }
                else  // ����
                {
                    speed_CTRL(0, 0, 0, 0);  // ����speed_CTRL������������Ϊ0
                    color_speed_sum1 = color_speed_sum2 = 0;  // ��ɫ�ٶȺ͵���0
                    osDelay(200);  // �ӳ�200����
                    if (target_location_sign < 6)  // �����ɫȦλ�ñ�־С��6
                    {
                        target_location_sign++;  // ��ɫȦλ�ñ�־��1
                    }
                    else  // ����
                    {
                        target_location_sign = 0;  // ��ɫȦλ�ñ�־����0
                        Camera_die();  // ����Camera_die����
                        Task_select = back_sign;  // ����ѡ����ڷ��ر�־
                        Task_verify = back_sign;  // ������֤���ڷ��ر�־
                        color_location_sign = 1;  // ��ɫλ�ñ�־����1
                    }
                }
                Camera_date_status = 0;  // �������״̬����0
            }
            else  // ����
                osDelay(10);  // �ӳ�10����
            break;  // ����ѭ��
        case End:
            if(distance_sign==1)
            {
                if(Directional_move_distance(angle_standard, imu_date, distance_now, 1000,110)==1)
                {
                    speed_CTRL(0, 0, 0, 0);
                    start_status=0;
                    vTaskResume(myTask_ledHandle); // LED��˸��������
                }    
                distance_sign=0;
            }                
//            vTaskSuspend(NULL);
            break;
        }

        osDelay(10);
    }
    /* USER CODE END doing_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
