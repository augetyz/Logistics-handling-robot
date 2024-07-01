#include "motor_ctrl.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern Car_status car_status;
void motor_tim_config(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // ������������ʱ��
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);     // ������������ʱ�������ж�,���������
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // ������������ʱ��
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);     // ������������ʱ�������ж�,���������
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // ������������ʱ��
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);     // ������������ʱ�������ж�,���������
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // ������������ʱ��
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);     // ������������ʱ�������ж�,���������

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    TIM2->CNT = 6720;
    TIM3->CNT = 6720;
    TIM4->CNT = 6720;
    TIM5->CNT = 6720;
}
void Motor_ctrl(uint8_t motor, uint16_t status)
{
    //     GPIOD->ODR |= (status << motor); // Ӳ����������Ļ���һ�仰�Ϳ��Կ����ĸ���������ĸ�״̬
    switch (motor)
    {
    case Motor1:
        GPIOD->ODR &= ~(0X03 << 8);
        GPIOD->ODR |= status << 8;
        break;
    case Motor2:
        GPIOD->ODR &= ~(0X03 << 10);
        GPIOD->ODR |= status << 10;
        break;
    case Motor3:
        GPIOD->ODR &= ~(0X03 << 12);
        GPIOD->ODR |= status << 12;
        break;
    case Motor4:
        GPIOD->ODR &= ~(0X03 << 14);
        GPIOD->ODR |= status << 14;
        break;
    }
}

void speed_ctrl(uint8_t motor, int speed)
{
    speed = speed > 3000 ? 3000 : (speed < -3000 ? -3000 : speed);
    Motor_ctrl(motor,
               speed > 0 ? Mode_gogo : (speed < 0 ? Mode_back : Mode_free));

    switch (motor)
    {
    case Motor1:
        TIM1->CCR1 = speed >= 0 ? speed : -speed;
        break;
    case Motor2:
        TIM1->CCR2 = speed >= 0 ? speed : -speed;
        break;
    case Motor3:
        TIM1->CCR3 = speed >= 0 ? speed : -speed;
        break;
    case Motor4:
        TIM1->CCR4 = speed >= 0 ? speed : -speed;
        break;
    }
}
