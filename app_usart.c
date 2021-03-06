/*
 * File      : app_uart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 *2017-12-15      DQL          the first version 
 */

#include <rthw.h>
#include <rtthread.h>
#include <math.h>
#include "HDMI.H"
#include "app_uSart.h"

/* 串口接收事件标志 */
#define UART_RX_EVENT (1 << 0)

/* 事件控制块 */
static struct rt_event event1;
static struct rt_event event2;
static struct rt_event event3;
/* 串口设备句柄 */
static rt_device_t uart_device1 = RT_NULL;//用于第一个串口 USART1
static rt_device_t uart_device2 = RT_NULL;//用于第一个串口 USART2
static rt_device_t uart_device3 = RT_NULL;//用于第一个串口 USART3

/*******用于把ASCII转换为十进制的整数
j表示输入的数据，j表示**************/
float shiji(uint8_t i,uint8_t j)
{
	uint8_t i1=0;
	float kk1=1.0f;
	for(i1=j-1;i1>0;i1--)
	{
		kk1*=10.0f;
	}
	kk1*=i;
	return kk1;
}
/*******用于把ASCII转换为十进制的小数，i表示输入的数据数组，j表示离小数点的位置**************/
float Ascii_Xiaoshu(uint8_t i,uint8_t j)
{
	uint8_t i1=0;
	float kk1=1.0f;
	for(i1=0;i1<j;i1++)
	{
		kk1/=10.0f;
	}
	kk1*=i;
	return kk1;
}


/***********用于ASCII码转换为整数******************/
float Get_AutoRun(uint8_t *kk,uint8_t j)
{
	uint8_t i=0;
	uint8_t b=0;
	float total=0.0f;
//	if(j==0)
//	{
//		return() ;//当只有11个数据时，表示没有数字输入，数据保持不变
//	}else{
		for(i=0;i<j;i++)//EE B1 11 00 00 00 09 11 00 FF FC FF FF 
		{
			
			if(kk[i]==0x2e)
			{
				
				b=i;
				break;
			}else{
				
				b=0;
			}
		}
			if(b>0)//当存在小数点时
			{
				
				for(i=0;i<b;i++)
				{
					
					total+=shiji(kk[i]-0x30,b-i);//获得整数部分,b表示小数点位置的坐标，
					
//					uart1_putchar(total);
				}
				
				
				for(i=b+1;i<j;i++)
				{
					
					total+=Ascii_Xiaoshu(kk[i]-0x30,i-b);
					
				}
				
			}
		if(b==0)
		{
				for(i=0;i<j;i++)
				{
					total+=(float)(shiji(kk[i]-0x30,j-i));//获得整数部分
			}
		}
		
	return total;
		
}
	
	
/* 回调函数 */
static rt_err_t uart_intput(rt_device_t dev, rt_size_t size)
{
    /* 发送事件 */
    rt_event_send(&event1, UART_RX_EVENT);
	rt_event_send(&event2, UART_RX_EVENT);
    rt_event_send(&event3, UART_RX_EVENT);
    return RT_EOK;
}

/***********串口1获取8位数据***************/
rt_uint8_t uart1_getchar(void)
{
    rt_uint32_t e;
    rt_uint8_t ch;

    /* 读取1字节数据 */
    while (rt_device_read(uart_device1, 0, &ch, 1) != 1)
    {
         /* 接收事件 */
        rt_event_recv(&event1, UART_RX_EVENT,RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e);
				

			
		}
		return ch;
}

/***********串口2获取8位数据***************/
rt_uint8_t uart2_getchar(void)
{
    rt_uint32_t e;
    rt_uint8_t ch;

    /* 读取1字节数据 */
    while (rt_device_read(uart_device2, 0, &ch, 1) != 1)
    {
         /* 接收事件 */
        rt_event_recv(&event2, UART_RX_EVENT,RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e);
				

			
		}
		return ch;
}
/***********串口3获取8位数据***************/
rt_uint8_t uart3_getchar(void)
{
    rt_uint32_t e;
    rt_uint8_t ch1;

    /* 读取1字节数据 */
    while (rt_device_read(uart_device3, 0, &ch1, 1) != 1)
    {
         /* 接收事件 */
        rt_event_recv(&event3, UART_RX_EVENT,RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e);
				

			
		}
		return ch1;
}

/***********串口1发送8位数据***************/
void uart1_putchar(const rt_uint8_t c)
{
    rt_size_t len = 0;
    rt_uint32_t timeout = 0;
    do
    {
        len = rt_device_write(uart_device1, 0, &c, 1);
        timeout++;
    }
    while (len != 1 && timeout < 500);
}

/***********串口2发送8位数据***************/
void uart2_putchar(const rt_uint8_t c)
{
    rt_size_t len = 0;
    rt_uint32_t timeout = 0;
    do
    {
        len = rt_device_write(uart_device2, 0, &c, 1);
        timeout++;
    }
    while (len != 1 && timeout < 500);
}
/***********串口3发送8位数据***************/
void uart3_putchar(const rt_uint8_t c)
{
	 rt_size_t len = 0;
    rt_uint32_t timeout = 0;
    do
    {
        len = rt_device_write(uart_device3, 0, &c, 1);
        timeout++;
    }
    while (len != 1 && timeout < 500);
	
}

/***********串口1发送字符串***************/
void uart1_putstring(const rt_uint8_t *s)
{
    while(*s)
    {
        uart1_putchar(*s++);
    }
}
/***********串口1发送字符串***************/
void uart2_putstring( char *s)
{
    while(*s)
    {
        uart2_putchar(*s++);
    }
}

/***********串口1发送字符串***************/
void uart3_putstring( char *s)
{
    while(*s)
    {
        uart3_putchar(*s++);
    }
}

/***********打开串口1***************/
rt_err_t uart1_open(const char *name)
{
    rt_err_t res;

    /* 查找系统中的串口设备 */
    uart_device1= rt_device_find(name);   
    /* 查找到设备后将其打开 */
    if (uart_device1 != RT_NULL)
    {   
       
        res = rt_device_set_rx_indicate(uart_device1, uart_intput);
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("set %s rx indicate error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 打开设备，以可读写、中断方式 */
        res = rt_device_open(uart_device1, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );       
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("open %s device error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 初始化事件对象 */
        rt_event_init(&event1, "event1", RT_IPC_FLAG_FIFO); 
    }
    else
    {
        rt_kprintf("can't find %s device.\n",name);
        return -RT_ERROR;
    }

    return RT_EOK;
}
/***********打开串口2***************/
rt_err_t uart2_open(const char *name)
{
    rt_err_t res;

    /* 查找系统中的串口设备 */
    uart_device2 = rt_device_find(name);   
    /* 查找到设备后将其打开 */
    if (uart_device3 != RT_NULL)
    {   
       
        res = rt_device_set_rx_indicate(uart_device2, uart_intput);
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("set %s rx indicate error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 打开设备，以可读写、中断方式 */
        res = rt_device_open(uart_device2, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );       
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("open %s device error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 初始化事件对象 */
        rt_event_init(&event2, "event2", RT_IPC_FLAG_FIFO); 
    }
    else
    {
        rt_kprintf("can't find %s device.\n",name);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/***********打开串口3***************/
rt_err_t uart3_open(const char *name)
{
    rt_err_t res;

    /* 查找系统中的串口设备 */
    uart_device3 = rt_device_find(name);   
    /* 查找到设备后将其打开 */
    if (uart_device3 != RT_NULL)
    {   
       
        res = rt_device_set_rx_indicate(uart_device3, uart_intput);
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("set %s rx indicate error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 打开设备，以可读写、中断方式 */
        res = rt_device_open(uart_device3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );       
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("open %s device error.%d\n",name,res);
            return -RT_ERROR;
        }

        /* 初始化事件对象 */
        rt_event_init(&event3, "event3", RT_IPC_FLAG_FIFO); 
    }
    else
    {
        rt_kprintf("can't find %s device.\n",name);
        return -RT_ERROR;
    }

    return RT_EOK;
}


