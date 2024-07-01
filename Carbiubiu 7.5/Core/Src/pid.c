#include "pid.h"
#include <stdio.h>
#include "math.h"
#include <stdlib.h>

#define Tolerance  260
/*********************************************************/
// 参数整定找最佳，从小到大顺序查；
// 先是比例后积分，最后再把微分加；
// 曲线振荡很频繁，比例度盘要放大；
// 曲线漂浮绕大湾，比例度盘往小扳；
// 曲线偏离回复慢，积分时间往下降；
// 曲线波动周期长，积分时间再加长；
// 曲线振荡频率快，先把微分降下来；
// 动差大来波动慢。微分时间应加长；
// 理想曲线两个波，前高后低四比一；
// 一看二调多分析，调节质量不会低；

// 若要反应增快，增大P减小I；

// 若要反应减慢，减小P增大I；

// 如果比例太大，会引起系统震荡；

// 如果积分太大，会引起系统迟钝。
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
    static PID pid; // 该函数在任务调度中使用，所以不使用全局变量，即使用static限制访问
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
        speed_dert[i] = speed_goal.date[i] - speed_now.date[i]; // 本次差值计算

        speed_sum[i] += speed_dert[i];                                                                 // 累计差值计算
        speed_sum[i] = speed_sum[i] > 30000 ? 30000 : (speed_sum[i] < -30000 ? -30000 : speed_sum[i]); // 积分限幅
        speed_last_last[i] = speed_last[i];                                                            // 上上次差值计算
        speed_last[i] = speed_dert[i];                                                                 // 上次差值计算
    }
    for (i = 0; i < 4; i++) // pid参数作用于输出值
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
    float angle_dert;
    static float angle_dert_sum, angle_dert_last;
    float kp3 = 9.1, ki3 = 0.06, kd3 = 0;
    static uint16_t num = 0;
    angle_dert = dert(angle, imu_date.IMU[2]);
    angle_dert_sum += angle_dert;
    angle_dert_sum = angle_dert_sum > 6000 ? 6000 : (angle_dert_sum > -6000 ? angle_dert_sum : -6000);
    angle_dert_last = angle_dert;
    if (angle_dert > -5.0f && angle_dert < 5.0f)
    {
        angle_dert_sum = angle_dert_sum > 1000 ? 1000 : (angle_dert_sum > -1000 ? angle_dert_sum : -1000);
    }
    else
    {
        angle_dert_sum = angle_dert_sum > 100 ? 100 : (angle_dert_sum > -100 ? angle_dert_sum : -100);
        kp3 = 25;ki3 = 1;
    }
        if (angle_dert > -0.4f && angle_dert < 0.4f)
    {
        speed_CTRL(0, 0, 0, 0);
        angle_dert_sum = 0;
        angle_dert_last = 0;
        speed_CTRL(0, 0, 0, 0);
        num++;
        if (num > 2)
        {
            num = 0;
            return 1;
        }
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
uint8_t direction_Set_biu(float angle, imu imu_date)
{
    int speed = 0, goal_out;
    //    float Yaw=imu_date.IMU[2];
    float angle_dert;
    static float angle_dert_sum, angle_dert_last;
    float kp3 = 8.1, ki3 = 0.07, kd3 = 0;
    angle_dert = dert(angle, imu_date.IMU[2]);
    angle_dert_sum += angle_dert;
    angle_dert_sum = angle_dert_sum > 6000 ? 6000 : (angle_dert_sum > -6000 ? angle_dert_sum : -6000);
    angle_dert_last = angle_dert;
    if (angle_dert > -2.0f && angle_dert < 2.0f)
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
        goal_out = goal_out > 1200 ? 1200 : (goal_out > -1200 ? goal_out : -1200);
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
// 直径77mm   转一圈242mm
//一圈161280个脉冲  k=90*448*4/242  
// 纵向移动
uint8_t advance_angle_distance(float angle, imu imu_date, biu distance_now, int distance_goal)
{
    vTaskSuspendAll();
    int k = 370, d = 0;
    distance_goal = distance_goal * k + d;
    int speed = 0, goal_out;
    int speed_goal, speed_best = 1200;
    float angle_dert;
    int distance_dert = 0, distance_done = 0; // 记录距离差、走过的距离
    static float angle_dert_sum, angle_dert_last;
    static int distance_dert_last, sign = 0, distance_dert_sum;
    static biu distance_first = { 0,0,0,0 };
    float kp3 = 20, ki3 = 0, kd3 = 0;      // 陀螺仪方向环PID参数
    //    float kp4 = 0.04, ki4 = 0.008, kd4 = 0; // 编码器距离环PID参数
    float kp4 = 0.01, ki4 = 0.001, kd4 = 0; // 编码器距离环PID参数
    if (sign != distance_goal)
    {
        //        distance_first = distance_now;
        sign = distance_goal;
        xSemaphoreGive(Distance_Sem_Handle);
    }
    else
    {
        distance_done = distance_now.date[0] - distance_first.date[0];
    }

    distance_dert = distance_goal - distance_done;
    distance_dert_sum += distance_dert;
    distance_dert_sum = distance_dert_sum > 6000 ? 6000 : (distance_dert_sum > -6000 ? distance_dert_sum : -6000);
    speed_goal = kp4 * distance_dert + kd4 * (distance_dert - distance_dert_last) + ki4 * distance_dert_sum;
    speed_goal = speed_goal > speed_best ? speed_best : (speed_goal > -speed_best ? speed_goal : -speed_best);

    distance_dert_last = distance_dert;
    xTaskResumeAll();
    if (distance_dert < 500 && distance_dert > -500)
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
// 横向移动
uint8_t crosswise_angle_distance(float angle, imu imu_date, biu distance_now, int distance_goal)
{
    vTaskSuspendAll();

    int k = 370, d = 0;
    distance_goal = distance_goal * k + d;
    int speed = 0, goal_out;
    int speed_goal, speed_best = 1200;
    float angle_dert;
    int distance_dert = 0, distance_done = 0; // 记录距离差、走过的距离
    static float angle_dert_sum, angle_dert_last;
    static int distance_dert_last, sign = 0, distance_dert_sum;
    static biu distance_first = { 0,0,0,0 };
    float kp3 = 20, ki3 = 0, kd3 = 0;      // 陀螺仪方向环PID参数
    //    float kp3 = 0, ki3 = 0, kd3 = 0;      // 陀螺仪方向环PID参数
    float kp4 = 0.01, ki4 = 0.001, kd4 = 0; // 编码器距离环PID参数
    if (sign != distance_goal)
    {
        //        distance_first = distance_now;
        sign = distance_goal;
        distance_dert_sum = angle_dert_last = 0;
        angle_dert_sum = angle_dert_last = 0;
        xSemaphoreGive(Distance_Sem_Handle);
    }
    else
    {
        distance_done = distance_now.date[0] - distance_first.date[0];
    }

    distance_dert = distance_goal - distance_done;
    distance_dert_sum += distance_dert;
    distance_dert_sum = distance_dert_sum > 6000 ? 6000 : (distance_dert_sum > -6000 ? distance_dert_sum : -6000);
    speed_goal = kp4 * distance_dert + kd4 * (distance_dert - distance_dert_last) + ki4 * distance_dert_sum;
    speed_goal = speed_goal > speed_best ? speed_best : (speed_goal > -speed_best ? speed_goal : -speed_best);

    distance_dert_last = distance_dert;

    xTaskResumeAll();

    if (distance_dert < Tolerance && distance_dert > -Tolerance)
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
        goal_out = goal_out > speed_best / 2 ? speed_best / 2 : (goal_out > -speed_best / 2 ? goal_out : -speed_best / 2);

        speed = -goal_out;

        speed_CTRL(speed_goal + speed, -speed_goal - speed, -speed_goal + speed, speed_goal - speed);
        // printf("%d,%d\n", speed, speed_goal);
        return 0;
    }
    return 0;
}
uint8_t Directional_move_distance(float angle, imu imu_date, biu distance_now, int distance_goal, float angle_goal)
{
    vTaskSuspendAll();

    int k = 370, d = 0;
    distance_goal = distance_goal * k + d;
    int speed = 0, goal_out = 0;
    int speed_goal = 0, speed_goal_1 = 0, speed_goal_2 = 0, speed_best = 1100;
    float angle_dert;
    int distance_dert = 0, distance_done = 0; // 记录距离差、走过的距离
    static float angle_dert_sum, angle_dert_last;
    static int distance_dert_last, sign = 0, distance_dert_sum;

    //    float kp3 = 7.0, ki3 = 1.5, kd3 = 0;      // 陀螺仪方向环PID参数
    float kp3 = 0, ki3 = 0.0, kd3 = 0;      // 陀螺仪方向环PID参数
    float kp4 = 0.013, ki4 = 0.0015, kd4 = 0; // 编码器距离环PID参数
    if (sign != distance_goal)
    {
        //        distance_first = distance_now;
        sign = distance_goal;
        distance_dert_sum = angle_dert_last = 0;
        angle_dert_sum = angle_dert_last = 0;
        xSemaphoreGive(Distance_Sem_Handle);
    }
    else
    {
        distance_done = (abs(distance_now.date[0]) + abs(distance_now.date[1])) / 2;
    }

    distance_dert = distance_goal - distance_done;
    distance_dert_sum += distance_dert;
    distance_dert_sum = distance_dert_sum > 6000 ? 6000 : (distance_dert_sum > -6000 ? distance_dert_sum : -6000);
    speed_goal = kp4 * distance_dert + kd4 * (distance_dert - distance_dert_last) + ki4 * distance_dert_sum;
    speed_goal = speed_goal > speed_best ? speed_best : (speed_goal > -speed_best ? speed_goal : -speed_best);

    distance_dert_last = distance_dert;

    xTaskResumeAll();

    if (distance_dert < Tolerance && distance_dert > -Tolerance)
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
        goal_out = goal_out > speed_best / 2 ? speed_best / 2 : (goal_out > -speed_best / 2 ? goal_out : -speed_best / 2);

        speed = -goal_out;
        speed_goal_1 = speed_goal * sin((angle_goal + 45) * DEC);
        speed_goal_2 = speed_goal * sin((angle_goal - 45) * DEC);
        speed_CTRL(speed_goal_1 + speed, speed_goal_2 + speed, speed_goal_2 - speed, speed_goal_1 - speed);
        // printf("%d,%d\n", speed, speed_goal);
        return 0;
    }
    return 0;
}


