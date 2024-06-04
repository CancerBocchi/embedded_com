/*
 * Copyright (c) 2020 Nanjing Xiaoxiongpai Intelligent Technology Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <hi_stdlib.h>
#include "wheel_codec.h"
#include "iot_gpio_ex.h"
#include "compile_define.h" //define encoder pinport
#include "robot_l9110s.h"   //pwm car drive
#include "cmsis_os2.h"
#include "ohos_init.h"
#include "hi_io.h"
#include "iot_uart.h"
#include "PID.h"

#define THREAD_STACK_SIZE (1024 * 8)
#define THREAD_PRIO 25
#define THREAD_DELAY_1S 2000000
#define THREAD_DELAY_1S 2000000
#define THREAD_DELAY_500MS 500000
#define WIFI_IOT_UART_IDX_1 1
#define WIFI_IOT_UART_IDX_2 2
#define UART_BUFF_SIZE 8


int flag_pid = 0;
/**
 * @brief encoder task
 *
 */
void MotorTask(void)
{
    
    
    while (1)
    {
        
    }
}

/**
 * @brief UartTask
 *
 */
static void UartTask(void)
{
    uint8_t uart_buff_1[UART_BUFF_SIZE] = {0};
    uint8_t *uart_buff_ptr_1 = uart_buff_1;
    uint8_t uart_buff_2[UART_BUFF_SIZE] = {0};
    uint8_t *uart_buff_ptr_2 = uart_buff_2;
    uint8_t ret_1;
    uint8_t ret_2;
    hi_io_set_func(HI_IO_NAME_GPIO_5, HI_IO_FUNC_GPIO_5_UART1_RXD);
    hi_io_set_func(HI_IO_NAME_GPIO_6, HI_IO_FUNC_GPIO_6_UART1_TXD);
    hi_io_set_func(HI_IO_NAME_GPIO_11, HI_IO_FUNC_GPIO_11_UART2_TXD); /* uart2 tx */
    hi_io_set_func(HI_IO_NAME_GPIO_12, HI_IO_FUNC_GPIO_12_UART2_RXD); /* uart2 rx */
    IotUartAttribute uart_attr = {

        // baud_rate: 115200
        .baudRate = 115200,

        // data_bits: 8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    // Initialize uart driver
    ret_1 = IoTUartInit(WIFI_IOT_UART_IDX_1, &uart_attr);
    if (ret_1 != IOT_SUCCESS)
    {
        printf("Failed to init uart1! Err code = %d\n", ret_1);
        return;
    }

    // // Initialize uart driver
    ret_2 = IoTUartInit(WIFI_IOT_UART_IDX_2, &uart_attr);
    if (ret_2 != IOT_SUCCESS)
    {
        printf("Failed to init uart2! Err code = %d\n", ret_2);
        return;
    }

    while (1)
    {

        // send data through uart1
        IoTUartRead(WIFI_IOT_UART_IDX_1, uart_buff_ptr_1, UART_BUFF_SIZE);
        if (uart_buff_ptr_1[0] != 0)
        {
            if (!strcmp(uart_buff_ptr_1, "1"))
            {
                printf("stop\n");
                car_stop();
                L_pid_velocity.Ref = 0;
                // IoTPwmStart(IOT_PWM_PORT_PWM2, 50, IOT_FREQ);
                // IoTPwmStart(IOT_PWM_PORT_PWM0, 50, IOT_FREQ);
            }
            else if (!strcmp(uart_buff_ptr_1, "2"))
            {
                printf("v = 1\n");
                // IoTPwmStart(2, 70, 100000); // 左轮正转
                // PID_setTarget(&L_pid_velocity, 5);
                // car_drive(0,20);
                L_pid_velocity.Ref = 8;
            }
            else if (!strcmp(uart_buff_ptr_1, "3"))
            {
                printf("v = 2\n");
                // car_drive(0,40);
                L_pid_velocity.Ref = -8;
                // PID_setTarget(&L_pid_velocity, 5);
                // IoTPwmStart(2, 30, 100000); // 左轮反转
            }
            else if (!strcmp(uart_buff_ptr_1, "4"))
            {
                printf("v = 3\n");
                L_pid_velocity.Ref = 16;
                // car_drive(0,40);
                // PID_setTarget(&L_pid_velocity, 8);
                // IoTPwmStart(0, 70, 100000); // 左轮正转
            }
            else if (!strcmp(uart_buff_ptr_1, "5"))
            {
                printf("v = 4\n");
                car_drive(1,-20);
                // IoTPwmStart(0, 40, 100000); // 左轮正转
            }
            else if (!strcmp(uart_buff_ptr_1, "6"))
            {
                printf("not pid \n");
                flag_pid = 1;
                // IoTPwmStart(0, 40, 100000); // 左轮正转
            }
            else if (!strcmp(uart_buff_ptr_1, "7"))
            {
                printf("is pid \n");
                flag_pid = 0;
                // IoTPwmStart(0, 40, 100000); // 左轮正转
            }
            else if(uart_buff_1[0] == 'P'){
                float pvalue = atof(uart_buff_1+1);
                L_pid_velocity.Kp = pvalue;
            }
            else if(uart_buff_ptr_1[0] == 'I'){
                float ivalue = atof(uart_buff_1+1);
                L_pid_velocity.Ki = ivalue;
            }
            else
                printf("command not found!\n");
            memset(uart_buff_ptr_1, 0, 8);
        }
        else
            ;
        // receive data through uart2
        // IoTUartRead(WIFI_IOT_UART_IDX_2, uart_buff_ptr_2, UART_BUFF_SIZE);
        // if (uart_buff_ptr_2[0] != 0)
        // {
        //     // printf("Uart1 read data:%s\n", uart_buff_ptr);
        //     if (!strcmp(uart_buff_ptr_2, "1"))
        //         printf("uart2 = 1\n");
        //     else if (!strcmp(uart_buff_ptr_2, "2"))
        //         printf("uart2 = 2\n");
        //     else
        //         printf("command not found!\n");
        //     memset(uart_buff_ptr_2, 0, 8);
        // }
        // else
        //     ;
        usleep(40000);
    }
}


/**
 * @brief Main Entry of the Thread Example
 *
 */
static void ThreadExample(void)
{
    osThreadAttr_t attr;

    attr.name = "MotorTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = THREAD_STACK_SIZE;
    attr.priority = 24;

    // Create the Thread1 task
    if (osThreadNew((osTimerFunc_t)MotorTask, NULL, &attr) == NULL)
    {
        printf("Failed to create MotorTask!\n");
    }

    // Create the Thread2 task
    attr.name = "UartTask";
    attr.priority = 25;
    if (osThreadNew((osTimerFunc_t)UartTask, NULL, &attr) == NULL)
    {
        printf("Failed to create UartTask!\n");
    }
}

APP_FEATURE_INIT(ThreadExample);
