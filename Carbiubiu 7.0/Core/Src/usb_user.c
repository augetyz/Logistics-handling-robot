#include "usbd_cdc_if.h"
#include "usb_user.h"
#include <stdarg.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/*
*** ��ֲע��usbd_cdc_if.c�ļ����޸ĵ����ݣ�����
*** ͷ�ļ�����
*** CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)�����������
*** ȫ�ֱ���ʹ��
*/
/*---------------------------------���������壬��ش�С��cubemx�ж���-------------------------------------*/

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
/** Data to send over USB CDC are stored in this buffer   */
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
/*---------------------------------���ȫ�ֱ���-------------------------------------*/
uint16_t Rx_Date_Num=0,RX_goal_num=0;       //���滺�����������������û��������ݻ�������ȡ�
uint8_t Rx_status=2;                        //��ɽ��ձ�־λ
uint8_t* p=NULL;                            //���ڱ����û�������ָ�룬�ڽ��յ�ָ���������ݺ�ͨ��ָ�룬������ת�浽�û�������

/*-------------------------Ӧ�ú������壬ģ��HAL��uart�շ�-------------------------------------*/
/**
  * @brief  USB���⴮�ڸ�ʽ�����printfʵ��
  * @param  ��ʽ������
  * @retval ��
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
  * @brief  ����������ڻ�ȡUSB���ջ���������������
  * @param  ��
  * @retval ���ؽ��յ���������
  */

uint16_t usb_Rx_Get_Num(void)
{
    return Rx_Date_Num;
}

/**
  * @brief  ����������ڽ���USB���⴮�ڵ�����
  * @param  Rx_Buffer: ���ջ�����
  * @param  num: ��Ҫ���յ���������
  * @param  overtime: ��ʱʱ��
  * @retval ������ճɹ�������1�������ʱ������0
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
  * @brief  �����������ݣ�����������ɽ��������ȫ�ֱ���Rx_status��һ������Ϊ0
  * @param  Rx_Buffer: ���ջ�����
  * @param  num: ��Ҫ���յ���������
  * @retval ��
  */
void usb_vbc_Receive_It(uint8_t* Rx_Buffer,uint16_t num)
{
    p=Rx_Buffer;
    RX_goal_num=num;
    Rx_status=0;
}
/**
  * @brief  ����������ڸ��ƽ��ջ����������ݣ�����������������λ
  * @param  Buffer_get: ��ȡ������
  * @param  Buffer_put: ���û�����
  * @param  num: Ҫ���Ƶ�Ԫ������
  * @retval ��
  */

void Rx_buffer_copy(uint8_t* Buffer_get,uint8_t* Buffer_put,uint16_t num)
{
    uint16_t i=0;
    for(i=0;i<num;i++)//��������
    {
        Buffer_get[i]=Buffer_put[i];
    }
    for(i=0;i<Rx_Date_Num-num;i++)//ʣ��������λ
    {
        Buffer_put[i]=Buffer_put[i+num];
    }
}

/**
  * @brief  ����������ڽ�һ����������ݸ��Ƶ���һ�������У������ᶪʧ���������е�ԭʼ����
  * @param  src: Դ����
  * @param  dest: Ŀ������
  * @param  n: Դ�����е�Ԫ������
  * @retval ��
  */
void Rx_date_save(uint8_t* src, uint8_t* dest, uint16_t n)
{
    uint16_t i=0,num=Rx_Date_Num;
    if(num+n>APP_RX_DATA_SIZE)
    return;//������������С������ֱ��ֹͣ��
    for(i=0;i<n;i++)
        dest[i+num]=src[i];
}


