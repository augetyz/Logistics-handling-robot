#include "pid.h"
#include "soft_user.h"

#include <stdio.h>
/*********************************************************/
// ������������ѣ���С����˳��飻
// ���Ǳ�������֣�����ٰ�΢�ּӣ�
// �����񵴺�Ƶ������������Ҫ�Ŵ�
// ����Ư���ƴ��壬����������С�⣻
// ����ƫ��ظ���������ʱ�����½���
// ���߲������ڳ�������ʱ���ټӳ���
// ������Ƶ�ʿ죬�Ȱ�΢�ֽ�������
// ���������������΢��ʱ��Ӧ�ӳ���
// ����������������ǰ�ߺ���ı�һ��
// һ�������������������������ͣ�

// ��Ҫ��Ӧ���죬����P��СI��

// ��Ҫ��Ӧ��������СP����I��

// �������̫�󣬻�����ϵͳ�𵴣�

// �������̫�󣬻�����ϵͳ�ٶۡ�
/**********************************************************/
extern QueueHandle_t Speed_Queue;
extern QueueHandle_t goal_Queue;
extern QueueHandle_t IMU_Queue;
extern QueueHandle_t debug_Queue;
extern SemaphoreHandle_t Distance_Sem_Handle;
void pid_do(biu speed_goal, biu speed_now)
{
    static int16_t speed_last[4], speed_last_last[4];
    static int16_t speed_sum[4], speed[4];
    int16_t speed_dert[4];
    uint8_t i = 0;
    static PID pid; // �ú��������������ʹ�ã����Բ�ʹ��ȫ�ֱ�������ʹ��static���Ʒ���
    pid.kp[0] = 0.6;
    pid.kp[1] = 0.6;
    pid.kp[2] = 0.6;
    pid.kp[3] = 0.6;
    pid.ki[0] = 0.12;
    pid.ki[1] = 0.12;
    pid.ki[2] = 0.12;
    pid.ki[3] = 0.12;
    pid.kd[0] = 0;
    pid.kd[1] = 0;
    pid.kd[2] = 0.0;
    pid.kd[3] = 0.0;

    for (i = 0; i < 4; i++)
    {
        speed_dert[i] = speed_goal.date[i] - speed_now.date[i]; // ���β�ֵ����

        speed_sum[i] += speed_dert[i];                                                                 // �ۼƲ�ֵ����
        speed_sum[i] = speed_sum[i] > 20000 ? 20000 : (speed_sum[i] < -20000 ? -20000 : speed_sum[i]); // �����޷�
        speed_last_last[i] = speed_last[i];                                                            // ���ϴβ�ֵ����
        speed_last[i] = speed_dert[i];                                                                 // �ϴβ�ֵ����
    }
    for (i = 0; i < 4; i++) // pid�������������ֵ
    {
        speed[i] = pid.kp[i] * speed_dert[i] +
                   pid.ki[i] * speed_sum[i] +
                   pid.kd[i] * (speed_dert[i] - speed_last[i] * 2 + speed_last_last[i]);
    }

    speed_ctrl(Motor1, speed[0]);
    speed_ctrl(Motor2, speed[1]);
    speed_ctrl(Motor3, speed[2]);
    speed_ctrl(Motor4, speed[3]);
    // printf("%d,%d,%d,%d\n", speed[0], speed[1], speed[2], speed[3]);
}

void speed_CTRL(int speed1, int speed2, int speed3, int speed4)
{
    biu speed_goal;
    speed_goal.date[0] = speed1;
    speed_goal.date[1] = speed2;
    speed_goal.date[2] = speed3;
    speed_goal.date[3] = speed4;
    xQueueSend(goal_Queue, &speed_goal, 0);
    xQueueSend(debug_Queue, &speed_goal, 0);
}
float dert(float a, float b)
{
    float c = a - b;
    if (c > 0 && c <= 180)
    {
        return c;
    }
    else
    {
        if (c >= -180 && c <= 0)
            return c;
        else if (c < -180)
            return 360 + c;
        else
            return c - 360;
    }
}
uint8_t direction_Set(float angle, imu imu_date)
{
    int speed = 0, goal_out;
    //    float Yaw=imu_date.IMU[2];
    float angle_dert;
    static float angle_dert_sum, angle_dert_last;
    float kp3 = 1.5, ki3 = 0.05, kd3 = 0;
    angle_dert = dert(angle, imu_date.IMU[2]);
    angle_dert_sum += angle_dert;
    angle_dert_sum = angle_dert_sum > 5000 ? 5000 : (angle_dert_sum > -5000 ? angle_dert_sum : -5000);
    angle_dert_last = angle_dert;
    if (angle_dert > -0.5f && angle_dert < 0.5f)
    {
        speed_CTRL(0, 0, 0, 0);
        angle_dert_sum = 0;
        angle_dert_last = 0;
        speed_CTRL(0, 0, 0, 0);
        return 1;
    }
    else
    {

        goal_out = angle_dert * kp3 + angle_dert_sum * ki3 + (angle_dert - angle_dert_last) * kd3;
        goal_out = goal_out > 500 ? 500 : (goal_out > -500 ? goal_out : -500);
    }

    speed = -goal_out;
    speed_CTRL(speed, -speed, speed, -speed);
    return 0;
}
void advance_angle(float angle, imu imu_date, int speed_goal)
{
    int speed = 0, goal_out;
    //    float Yaw=imu_date.IMU[2];
    float angle_dert;
    static float angle_dert_sum, angle_dert_last;
    float kp3 = 3.6, ki3 = 0.12, kd3 = 0;
    if (dert(angle, imu_date.IMU[2]) > 30)
    {
        direction_Set(angle, imu_date);
    }
    else
    {
        angle_dert = dert(angle, imu_date.IMU[2]);
        angle_dert_sum += angle_dert;
        angle_dert_sum = angle_dert_sum > 30000 ? 30000 : (angle_dert_sum > -30000 ? angle_dert_sum : -30000);
        angle_dert_last = angle_dert;

        goal_out = angle_dert * kp3 + angle_dert_sum * ki3 + (angle_dert - angle_dert_last) * kd3;
        goal_out = goal_out > 2000 ? 2000 : (goal_out > -2000 ? goal_out : -2000);

        speed = -goal_out;
        speed_CTRL(speed_goal + speed, speed_goal - speed, speed_goal + speed, speed_goal - speed);
        return;
    }
}
void crosswise_angle(float angle, imu imu_date, int speed_goal)
{
    int speed = 0, goal_out;
    //    float Yaw=imu_date.IMU[2];
    float angle_dert;
    static float angle_dert_sum, angle_dert_last;
    float kp3 = 2.6, ki3 = 0, kd3 = 1;
    if (dert(angle, imu_date.IMU[2]) > 30)
    {
        direction_Set(angle, imu_date);
    }
    else
    {
        angle_dert = dert(angle, imu_date.IMU[2]);
        angle_dert_sum += angle_dert;
        angle_dert_sum = angle_dert_sum > 30000 ? 30000 : (angle_dert_sum > -30000 ? angle_dert_sum : -30000);
        angle_dert_last = angle_dert;

        goal_out = angle_dert * kp3 + angle_dert_sum * ki3 + (angle_dert - angle_dert_last) * kd3;
        goal_out = goal_out > 2000 ? 2000 : (goal_out > -2000 ? goal_out : -2000);

        speed = -goal_out;
        speed_CTRL(speed_goal + speed, -speed_goal + speed, -speed_goal - speed, speed_goal - speed);
        return;
    }
}
// �����ƶ�
uint8_t advance_angle_distance(float angle, imu imu_date, biu distance_now, int distance_goal)
{
    vTaskSuspendAll();
    int k = 285 * 5, d = 0;
    distance_goal = distance_goal * k + d;
    int speed = 0, goal_out;
    int speed_goal, speed_best = 2600;
    float angle_dert;
    int distance_dert = 0, distance_done = 0; // ��¼�����߹��ľ���
    static float angle_dert_sum, angle_dert_last;
    static int distance_dert_last, sign = 0, distance_dert_sum;
    static biu distance_first;
    float kp3 = 2, ki3 = 0.1, kd3 = 0;      // �����Ƿ���PID����
    float kp4 = 0.01, ki4 = 0.002, kd4 = 0; // ���������뻷PID����

    if (sign != distance_goal)
    {
        distance_first = distance_now;
        sign = distance_goal;
    }
    else
    {
        distance_done = (distance_now.date[0] - distance_first.date[0] + distance_now.date[1] - distance_first.date[1] +
                         distance_now.date[2] - distance_first.date[2] + distance_now.date[3] - distance_first.date[3]) /
                        4;
    }

    distance_dert = distance_goal - distance_done;
    distance_dert_sum += distance_dert;
    distance_dert_sum = distance_dert_sum > 4000 ? 4000 : (distance_dert_sum > -4000 ? distance_dert_sum : -4000);
    speed_goal = kp4 * distance_dert + kd4 * (distance_dert - distance_dert_last) + ki4 * distance_dert_sum;
    speed_goal = speed_goal > speed_best ? speed_best : (speed_goal > -speed_best ? speed_goal : -speed_best);

    distance_dert_last = distance_dert;
    xTaskResumeAll();
    if (distance_dert < 25 && distance_dert > -25)
    {
        speed_ctrl(Motor1, 0);
        speed_ctrl(Motor2, 0);
        speed_ctrl(Motor3, 0);
        speed_ctrl(Motor4, 0);
        speed_CTRL(0, 0, 0, 0);
        angle_dert_sum = 0;
        distance_dert_sum = 0;
        xSemaphoreGive(Distance_Sem_Handle);
        return 1;
    }
    if (dert(angle, imu_date.IMU[2]) > 30)
    {
        direction_Set(angle, imu_date);
    }
    else
    {
        angle_dert = dert(angle, imu_date.IMU[2]);
        angle_dert_sum += angle_dert;
        angle_dert_sum = angle_dert_sum > 1000 ? 1000 : (angle_dert_sum > -1000 ? angle_dert_sum : -1000);
        angle_dert_last = angle_dert;

        goal_out = angle_dert * kp3 + angle_dert_sum * ki3 + (angle_dert - angle_dert_last) * kd3;
        goal_out = goal_out > speed_best / 2 ? speed_best / 2 : (goal_out > -speed_best / 2 ? goal_out : -speed_best / 2);

        speed = -goal_out;
        speed_CTRL(speed_goal + speed, speed_goal - speed, speed_goal + speed, speed_goal - speed);
        // printf("%d   ,%d   ,%.2f   ,%d   \n", speed, speed_goal, angle_dert, distance_dert);
        return 0;
    }
    return 0;
}


//// �����ƶ�
//uint8_t crosswise_angle_distance(float angle, imu imu_date, biu distance_now, int distance_goal)
//{
//    vTaskSuspendAll();
//                

//                
//    static IDR_date IDR_date_use, IDR_date_goal;

//    int k = 305 * 5, d = 0;
//    distance_goal = distance_goal * k + d;
//    int speed = 0;static int goal_out;
//    int speed_goal, speed_best = 2000;
//    float angle_dert;
//    int distance_dert = 0, distance_done = 0; // ��¼�����߹��ľ���
//    static float angle_dert_sum, angle_dert_last;
//    static int distance_dert_last, sign = 0, distance_dert_sum;
//    static biu distance_first;
//    float kp3 = 4, ki3 = 0, kd3 = 0;      // �����Ƿ���PID����
//    float kp4 = 0.04, ki4 = 0.008, kd4 = 0; // ���������뻷PID����
//    // float kp4 = 0.00, ki4 = 0.000, kd4 = 0; // ���������뻷PID����
//    if (sign != distance_goal)
//    {
//        distance_first = distance_now;
//        sign = distance_goal;
//        distance_dert_sum = angle_dert_last = 0;
//        angle_dert_sum = angle_dert_last = 0;
//    }
//    else
//    {
//        distance_done = (distance_now.date[0] - distance_first.date[0] + distance_now.date[3] - distance_first.date[3]) / 2;
//    }

//    distance_dert = distance_goal - distance_done;
//    distance_dert_sum += distance_dert;
//    distance_dert_sum = distance_dert_sum > 4000 ? 4000 : (distance_dert_sum > -4000 ? distance_dert_sum : -4000);
//    speed_goal = kp4 * distance_dert + kd4 * (distance_dert - distance_dert_last) + ki4 * distance_dert_sum;
//    speed_goal = speed_goal > speed_best ? speed_best : (speed_goal > -speed_best ? speed_goal : -speed_best);

//    distance_dert_last = distance_dert;
//    
//    xTaskResumeAll();

//    if (distance_dert < 25 && distance_dert > -25)
//    {
//        
//				speed_ctrl(1,0);
//				speed_ctrl(2,0);
//				speed_ctrl(3,0);
//				speed_ctrl(4,0);
//        angle_dert_sum = 0;
//        distance_dert_sum = 0;
//				goal_out=0;
//        xSemaphoreGive(Distance_Sem_Handle);
//				speed_CTRL(0, 0, 0, 0);
//        return 1;
//    }
//    if (dert(angle, imu_date.IMU[2]) > 30)
//    {
//        direction_Set(angle, imu_date);
//    }
//    else
//    {

//        angle_dert = dert(angle, imu_date.IMU[2]);
//        angle_dert_sum += angle_dert;
//        angle_dert_sum = angle_dert_sum > 4000 ? 4000 : (angle_dert_sum > -4000 ? angle_dert_sum : -4000);
//        angle_dert_last = angle_dert;

//        goal_out += angle_dert * kp3 + angle_dert_sum * ki3 + (angle_dert - angle_dert_last) * kd3;
//        goal_out = goal_out > speed_best / 2 ? speed_best / 2 : (goal_out > -speed_best / 2 ? goal_out : -speed_best / 2);

//        speed = -goal_out;
//        
//        speed_CTRL(speed_goal + speed, -speed_goal - speed, -speed_goal + speed, speed_goal - speed);
//        // printf("%d,%d\n", speed, speed_goal);
//        return 0;
//    }
//    return 0;
//}
// �����ƶ�
uint8_t crosswise_angle_distance(float angle, imu imu_date, biu distance_now, int distance_goal)
{
    int k = 305 * 5, d = 0;
    distance_goal = distance_goal * k + d;
    int speed = 0, goal_out;
    int speed_goal, speed_best = 2000;
    float angle_dert;
    int distance_dert = 0, distance_done = 0; // ��¼�����߹��ľ���
    static float angle_dert_sum, angle_dert_last;
    static int distance_dert_last, sign = 0, distance_dert_sum;
    static biu distance_first;
    float kp3 = 3.6, ki3 = 1, kd3 = 0;      // �����Ƿ���PID����
    float kp4 = 0.04, ki4 = 0.008, kd4 = 0; // ���������뻷PID����
    if (sign != distance_goal)
    {
        distance_first = distance_now;
        sign = distance_goal;
        distance_dert_sum = angle_dert_last = 0;
        angle_dert_sum = angle_dert_last = 0;
    }
    else
    {
        distance_done = (distance_now.date[0] - distance_first.date[0] + distance_now.date[3] - distance_first.date[3]) / 2;
    }

    distance_dert = distance_goal - distance_done;
    distance_dert_sum += distance_dert;
    distance_dert_sum = distance_dert_sum > 4000 ? 4000 : (distance_dert_sum > -4000 ? distance_dert_sum : -4000);
    speed_goal = kp4 * distance_dert + kd4 * (distance_dert - distance_dert_last) + ki4 * distance_dert_sum;
    speed_goal = speed_goal > speed_best ? speed_best : (speed_goal > -speed_best ? speed_goal : -speed_best);

    distance_dert_last = distance_dert;

    if (distance_dert < 10 && distance_dert > -10)
    {
        speed_CTRL(0, 0, 0, 0);
        angle_dert_sum = 0;
        distance_dert_sum = 0;
        xSemaphoreGive(Distance_Sem_Handle);
        return 1;
    }
    if (dert(angle, imu_date.IMU[2]) > 30)
    {
        direction_Set(angle, imu_date);
    }
    else
    {

        angle_dert = dert(angle, imu_date.IMU[2]);

        angle_dert_sum += angle_dert;
        angle_dert_sum = angle_dert_sum > 4000 ? 4000 : (angle_dert_sum > -4000 ? angle_dert_sum : -4000);
        angle_dert_last = angle_dert;

        goal_out = angle_dert * kp3 + angle_dert_sum * ki3 + (angle_dert - angle_dert_last) * kd3;
        goal_out = goal_out > speed_best / 4 ? speed_best / 4 : (goal_out > -speed_best / 4 ? goal_out : -speed_best / 4);

        speed = -goal_out;

        speed_CTRL(speed_goal + speed, -speed_goal - speed, -speed_goal + speed, speed_goal - speed);
        // printf("%d,%d\n", speed, speed_goal);
        return 0;
    }
    return 0;
}
