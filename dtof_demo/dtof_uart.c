/*
 * Copyright (C) 2021 HiHope Open Source Organization .
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 *
 * limitations under the License.
 */

#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_io.h"
#include "iot_pwm.h"
#include "hi_pwm.h"

#define IOT_UART_IDX_1  (1)
#define STACK_SIZE   (1024)
#define DELAY_US     (20000)
#define IOT_GPIO_5  (5) //uart1
#define IOT_GPIO_6  (6) 
#define IOT_GPIO_2  (2) //LED
#define IOT_GPIO_0  (0)
#define IOT_GPIO_1  (1)
#define TOF_RX      (12)
#define TOF_TX      (11)
#define TOF_UART_IDX    (2)

#define IOT_PWM_PORT_PWM2   2
#define PWM_DUTY_50        (50)
#define PWM_DUTY_99        (99)
#define PWM_FREQ_4K        (4000)
#define DTOF_DATA_LEN       137
#define CRC_LEN             133

unsigned char uartReadBuff[300] = {0};
unsigned char crcBuff[300] = {0};
static unsigned int crc32table[256];

/*
 * 初始化uart1接口配置
 * Initialize uart1 interface configuration
 */
int usr_uart_config(void)
{
    IotUartAttribute g_uart_cfg = {115200, 8, 1, IOT_UART_PARITY_NONE, 500, 500, 0};
    int ret = IoTUartInit(TOF_UART_IDX, &g_uart_cfg);
    if (ret != 0) 
    {
        printf("uart init fail\r\n");
    }

    return ret;
}


static void init_crc_table(void)
{
    unsigned int c;
    unsigned int i, j;
    for (i = 0; i < 256; i++) 
    {
        c = (unsigned int)i;
        for (j = 0; j < 8; j++) 
        {
            if (c & 1)
                c = 0xedb88320L ^ (c >> 1);
            else
                c = c >> 1;
        }                                                                                                                                            
        crc32table[i] = c;
    }
};

/*
 * crc校验
 * crc checking
 */

static unsigned int crc32(const unsigned char *buf, unsigned int size)
{
    unsigned int  i, crc = 0xFFFFFFFF;

    for (i = 0; i < size; i++) 
    {
        crc = crc32table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8); /* 8: 右移8bit */
    }
    return crc ^ 0xFFFFFFFF;
}

/*
 * 配置uart1、pwm0、红灯、黄灯、绿灯的gpio管脚
 * Configure GPIO pins for uart1, wm0, red light, yellow light, and green light
 */
void gpio_init(void)
{

    IoTGpioInit(TOF_TX);  
    hi_io_set_func(TOF_TX, HI_IO_FUNC_GPIO_11_UART2_TXD);
    IoTGpioInit(TOF_RX);  
    hi_io_set_func(TOF_RX, HI_IO_FUNC_GPIO_12_UART2_RXD);

    // IoTGpioInit(IOT_GPIO_2);  
    // hi_io_set_func(IOT_GPIO_2, HI_IO_FUNC_GPIO_2_PWM2_OUT);
    // IoTGpioSetDir(IOT_GPIO_2, IOT_GPIO_DIR_OUT);
    // IoTPwmInit(IOT_PWM_PORT_PWM2);
}

/* 
 * 接收dtof数据并处理
 * Receive dtof data and process it
 */
static void* UartDemo_Task(void)
{
    static int r = 0; //统计距离小于0.2米的点云数量，满足条件亮红灯
    static int g = 0; //统计距离在0.4米到1米之间的点云数量，满足条件亮绿灯
    static int y = 0; //统计距离在0.2米到0.4米之间的点云数量，满足条件亮黄灯
    static int d = 0; 
    unsigned short int *data = NULL;
    short int i = 0;

    unsigned int len = 0; //接收到的串口数据长度
    unsigned int crc_data = 0; //接收CRC校验后的值

    gpio_init();

    usr_uart_config();

    init_crc_table();

    printf("task init successfully!\n");

    while (1) 
    {

        len += IoTUartRead(TOF_UART_IDX, uartReadBuff+len, DTOF_DATA_LEN);
        if (len >= DTOF_DATA_LEN) 
        {
            if(uartReadBuff[0] == 0xAA && uartReadBuff[1] == 0x55 && uartReadBuff[132] == 0xFF) 
            {
                memcpy(crcBuff,uartReadBuff,CRC_LEN);
                crc_data = crc32(crcBuff,CRC_LEN);
                
                if(crc_data == *(unsigned int *)(&uartReadBuff[CRC_LEN])) 
                {
                    printf("info get successfully!\n");
                    for(i = 0; i < 64; i++) 
                    {
                        data = (unsigned short int *)(&uartReadBuff[i*2+4]);
                        if((*data) < 200)
                        {
                            r += 1;
                        }
                        else if ((*data) >= 200 && (*data) < 400)
                        {
                            y += 1;
                        }
                        else if((*data) >= 400 && (*data)  < 1000)
                        {
                            g += 1;
                        }
                        else if((*data) <= 8000 && (*data) > 1000)
                        {
                            d += 1;
                        }

                    }
                    // d = d + 1;
                    // if (r == 64)
                    // { 
                    //     IoTGpioSetOutputVal(IOT_GPIO_10, IOT_GPIO_VALUE1);
                    //     IoTGpioSetOutputVal(IOT_GPIO_11, IOT_GPIO_VALUE0);
                    //     IoTGpioSetOutputVal(IOT_GPIO_12, IOT_GPIO_VALUE0);

                    //     if((*data) >= 100 && (*data) < 150)
                    //     {
                    //         IoTPwmStart(IOT_PWM_PORT_PWM0, 99, PWM_FREQ_4K);
                    //         d = 0;
                    //     }
                    //     else if((*data) >= 150 && (*data) < 200 && d >= 3)
                    //     {
                    //         IoTPwmStart(IOT_PWM_PORT_PWM0, PWM_DUTY_50, PWM_FREQ_4K);
                    //         d = 0;
                    //     }
                    // }
                    // else if(y >= 32)
                    // {
                    //     IoTGpioSetOutputVal(IOT_GPIO_10, IOT_GPIO_VALUE0);
                    //     IoTGpioSetOutputVal(IOT_GPIO_11, IOT_GPIO_VALUE0);
                    //     IoTGpioSetOutputVal(IOT_GPIO_12, IOT_GPIO_VALUE1);

                    //     if((*data) >= 200 && (*data) < 300 && d >= 6)
                    //     {
                    //         IoTPwmStart(IOT_PWM_PORT_PWM0, PWM_DUTY_50, PWM_FREQ_4K);
                    //         d = 0;
                    //     }
                    //     else if((*data) >= 300 && (*data) < 400 && d >= 9)
                    //     {
                    //         IoTPwmStart(IOT_PWM_PORT_PWM0, PWM_DUTY_50, PWM_FREQ_4K);
                    //         d = 0;
                    //     }
                    //     else if((*data) >= 400 && (*data) < 500 && d >= 12)
                    //     {
                    //         IoTPwmStart(IOT_PWM_PORT_PWM0, PWM_DUTY_50, PWM_FREQ_4K);
                    //         d = 0;
                    //     }
                    // }
                    // else if (g >= 16)
                    // {
                    //     IoTGpioSetOutputVal(IOT_GPIO_10, IOT_GPIO_VALUE0);
                    //     IoTGpioSetOutputVal(IOT_GPIO_11, IOT_GPIO_VALUE1);
                    //     IoTGpioSetOutputVal(IOT_GPIO_12, IOT_GPIO_VALUE0);
                    //     IoTPwmStop(IOT_PWM_PORT_PWM0);
                    // }
                    // else
                    // {
                    //     IoTGpioSetOutputVal(IOT_GPIO_10, IOT_GPIO_VALUE0);
                    //     IoTGpioSetOutputVal(IOT_GPIO_11, IOT_GPIO_VALUE0);
                    //     IoTGpioSetOutputVal(IOT_GPIO_12, IOT_GPIO_VALUE0);

                    //     IoTPwmStop(IOT_PWM_PORT_PWM0);
                    // }
                }
                else
                {
                    printf("crc32 fail !!!\r\n");
                }
            }
            len = 0;
            crc_data = 0;
            //printf("Uart read data:r = %d,y = %d,g = %d\r\n", r,y,g);
            printf("%d,%d,%d,%d\n", r,y,g,d);
            r = y = g = d = 0;
            data = NULL;
            memset(uartReadBuff,0,sizeof(uartReadBuff));
            memset(crcBuff,0,sizeof(crcBuff));
        }
        usleep(DELAY_US);
        // if(len == 0)
        // {
        //     crc_data = 0;
        //     //printf("Uart read data:r = %d,y = %d,g = %d\r\n", r,y,g);
        //     printf("%d,%d,%d\n", r,y,g);
        //     r = y = g = 0;
        //     data = NULL;
        //     memset(uartReadBuff,0,sizeof(uartReadBuff));
        //     memset(crcBuff,0,sizeof(crcBuff));

        //     // IoTGpioSetOutputVal(IOT_GPIO_10, IOT_GPIO_VALUE0);
        //     // IoTGpioSetOutputVal(IOT_GPIO_11, IOT_GPIO_VALUE0);
        //     // IoTGpioSetOutputVal(IOT_GPIO_12, IOT_GPIO_VALUE0);

        //     // IoTPwmStop(IOT_PWM_PORT_PWM0);
        // }

    }
    // while(1)
    // {
    //     printf("hello world\n");
    //     osDelay(100);
    // }
    return NULL;
}

/*
 * 任务入口函数
 * Task Entry Function
 */
static void UartDemo_Entry(void)
{
    osThreadAttr_t attr = {0};

    printf("[UartDemo] UartDemo_Entry()\n");

    attr.name = "UartDemo_Task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = STACK_SIZE;      
    attr.priority = osPriorityNormal;  

    if (osThreadNew((osThreadFunc_t)UartDemo_Task, NULL, &attr) == NULL) 
    {
        printf("[UartDemo] Falied to create UartDemo_Task!\n");
    }
}

/*
 * 注册模块
 * registration module
 */
// SYS_RUN(UartDemo_Entry);
APP_FEATURE_INIT(UartDemo_Entry);
