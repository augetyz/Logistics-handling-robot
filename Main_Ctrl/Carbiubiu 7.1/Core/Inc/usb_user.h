#ifndef __USB_USER_H
#define __USB_USER_H

#include "usb_device.h"
/* 使用usb_printf函数，按照指定的格式输出数据 */
void usb_printf(const char *format, ...);

/* 使用Rx_buffer_copy函数，将Buffer_get的数据复制到Buffer_put，复制的数据量为num */
void Rx_buffer_copy(uint8_t* Buffer_get,uint8_t* Buffer_put,uint16_t num);

/* 使用usb_vbc_Receive函数，从Rx_Buffer接收数据，接收的数据量为num，超时时间为overtime */
uint8_t usb_vbc_Receive(uint8_t* Rx_Buffer,uint16_t num,uint32_t overtime);

/* 使用Rx_date_save函数，将src的数据保存到dest，保存的数据量为n */
void Rx_date_save(uint8_t* src, uint8_t* dest, uint16_t n);

/* 使用usb_vbc_Receive_It函数，从Rx_Buffer接收数据，接收的数据量为num */
void usb_vbc_Receive_It(uint8_t* Rx_Buffer,uint16_t num);

#endif

