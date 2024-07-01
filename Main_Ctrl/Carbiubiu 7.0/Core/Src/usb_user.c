#include "usbd_cdc_if.h"
#include "usb_user.h"
#include <stdarg.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/*
*** 移植注意usbd_cdc_if.c文件中修改的内容，包括
*** 头文件包含
*** CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)函数内容添加
*** 全局变量使用
*/
/*---------------------------------缓存区定义，相关大小在cubemx中定义-------------------------------------*/

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
/** Data to send over USB CDC are stored in this buffer   */
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
/*---------------------------------相关全局变量-------------------------------------*/
uint16_t Rx_Date_Num=0,RX_goal_num=0;       //保存缓存区数据量，保存用户接收数据缓存区宽度。
uint8_t Rx_status=2;                        //完成接收标志位
uint8_t* p=NULL;                            //用于保存用户缓存区指针，在接收到指定数量数据后，通过指针，将数据转存到用户缓存区

/*-------------------------应用函数定义，模仿HAL库uart收发-------------------------------------*/
/**
  * @brief  USB虚拟串口格式化输出printf实现
  * @param  格式化输入
  * @retval 无
  */
void usb_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;
 
    va_start(args, format);
    length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
    CDC_Transmit_FS(UserTxBufferFS, length);
}
/**
  * @brief  这个函数用于获取USB接收缓存区的数据数量
  * @param  无
  * @retval 返回接收的数据数量
  */

uint16_t usb_Rx_Get_Num(void)
{
    return Rx_Date_Num;
}

/**
  * @brief  这个函数用于接收USB虚拟串口的数据
  * @param  Rx_Buffer: 接收缓冲区
  * @param  num: 需要接收的数据数量
  * @param  overtime: 超时时间
  * @retval 如果接收成功，返回1，如果超时，返回0
  */
 
uint8_t usb_vbc_Receive(uint8_t* Rx_Buffer,uint16_t num,uint32_t overtime)
{
    uint32_t time=0;
    overtime=overtime/2;
    if(Rx_Date_Num>=num)
    {
        Rx_buffer_copy(Rx_Buffer,UserRxBuffer,num);
        Rx_Date_Num-=num;
        return 1;
    }
    else
    {
        if(overtime==HAL_MAX_DELAY)
        {
            while(1)
            {
                if(Rx_Date_Num>=num)
                {
                    Rx_buffer_copy(Rx_Buffer,UserRxBuffer,num);
                    Rx_Date_Num-=num;
                    return 1;
                }
#ifdef INC_FREERTOS_H               
                osDelay(1);    
#else
                HAL_Delay(1);
#endif
            }
        }
        else
        {
            while(1)
            {
                if(Rx_Date_Num>=num)
                {
                    Rx_buffer_copy(Rx_Buffer,UserRxBuffer,num);
                    Rx_Date_Num-=num;
                    return 1;
                }
                else
                    time++;
                if(time>overtime)
                    return 0;
#ifdef INC_FREERTOS_H               
                osDelay(1);    
#else
                HAL_Delay(1);
#endif          
            }
        }
    }
}
/**
  * @brief  开启接收数据，不堵塞，完成接收任务后，全局变量Rx_status置一，否则为0
  * @param  Rx_Buffer: 接收缓冲区
  * @param  num: 需要接收的数据数量
  * @retval 无
  */
void usb_vbc_Receive_It(uint8_t* Rx_Buffer,uint16_t num)
{
    p=Rx_Buffer;
    RX_goal_num=num;
    Rx_status=0;
}
/**
  * @brief  这个函数用于复制接收缓冲区的内容，并将缓存区数据移位
  * @param  Buffer_get: 获取缓冲区
  * @param  Buffer_put: 放置缓冲区
  * @param  num: 要复制的元素数量
  * @retval 无
  */

void Rx_buffer_copy(uint8_t* Buffer_get,uint8_t* Buffer_put,uint16_t num)
{
    uint16_t i=0;
    for(i=0;i<num;i++)//复制数据
    {
        Buffer_get[i]=Buffer_put[i];
    }
    for(i=0;i<Rx_Date_Num-num;i++)//剩余数据移位
    {
        Buffer_put[i]=Buffer_put[i+num];
    }
}

/**
  * @brief  这个函数用于将一个数组的内容复制到另一个数组中，而不会丢失接收数组中的原始数据
  * @param  src: 源数组
  * @param  dest: 目标数组
  * @param  n: 源数组中的元素数量
  * @retval 无
  */
void Rx_date_save(uint8_t* src, uint8_t* dest, uint16_t n)
{
    uint16_t i=0,num=Rx_Date_Num;
    if(num+n>APP_RX_DATA_SIZE)
    return;//超出缓存区大小，这里直接停止。
    for(i=0;i<n;i++)
        dest[i+num]=src[i];
}


