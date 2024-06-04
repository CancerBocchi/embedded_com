/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
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
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <memory.h>
#include "compile_define.h"
#include "iot_gpio.h"
#include "iot_pwm.h"
#include "robot_l9110s.h"

#define IOT_PWM_PORT_PWM0 (0)
#define IOT_PWM_PORT_PWM1 (1)
#define IOT_PWM_PORT_PWM2 (2)
#define IOT_PWM_PORT_PWM3 (3)
#define IOT_FREQ 100000

#define DRIVE_LEFT_FORWARD_PIN_NAME (IOT_IO_NAME_GPIO_2)
#define DRIVE_LEFT_FORWARD_PIN_GPIO (IOT_IO_FUNC_GPIO_2_GPIO)
#define DRIVE_LEFT_FORWARD_PWM (IOT_PWM_PORT_PWM2)
#define DRIVE_LEFT_FORWARD_PWM_FUNC (IOT_IO_FUNC_GPIO_2_PWM2_OUT)
#define DRIVE_RIGHT_FORWARD_PIN_NAME (IOT_IO_NAME_GPIO_7)
#define DRIVE_RIGHT_FORWARD_PIN_GPIO (IOT_IO_FUNC_GPIO_7_GPIO)
#define DRIVE_RIGHT_FORWARD_PWM (IOT_PWM_PORT_PWM0)
#define DRIVE_RIGHT_FORWARD_PWM_FUNC (IOT_IO_FUNC_GPIO_7_PWM0_OUT)

void init_car_drive(void)
{
    IoSetFunc(DRIVE_LEFT_FORWARD_PIN_NAME, DRIVE_LEFT_FORWARD_PWM_FUNC);
    IoTGpioSetDir(DRIVE_LEFT_FORWARD_PIN_NAME, IOT_GPIO_DIR_OUT);
    IoTPwmInit(DRIVE_LEFT_FORWARD_PWM);

    IoSetFunc(DRIVE_RIGHT_FORWARD_PIN_NAME, DRIVE_RIGHT_FORWARD_PWM_FUNC);
    IoTGpioSetDir(DRIVE_RIGHT_FORWARD_PIN_NAME, IOT_GPIO_DIR_OUT);
    IoTPwmInit(DRIVE_RIGHT_FORWARD_PWM);
}

void car_drive(int left_or_right, int value)
{
    if (left_or_right == 0)
    {
        if (value > 0)
        {
            IoTPwmStart(DRIVE_LEFT_FORWARD_PWM, (value / 2.0 + 50), IOT_FREQ); // 左轮正转
        }
        else if (value < 0)
        {
            IoTPwmStart(DRIVE_LEFT_FORWARD_PWM, (50 + value / 2.0), IOT_FREQ); // 左轮反转
        }
        else
        {
            IoTPwmStart(DRIVE_LEFT_FORWARD_PWM, 50, IOT_FREQ);
        }
    }
    else if (left_or_right == 1)
    {
        if (value > 0)
        {
            IoTPwmStart(DRIVE_RIGHT_FORWARD_PWM, (-value / 2.0 + 50), IOT_FREQ); // 右轮正转
        }
        else if (value < 0)
        {
            IoTPwmStart(DRIVE_RIGHT_FORWARD_PWM, (50 - value / 2.0), IOT_FREQ); // 右轮反转
        }
        else
        {
            IoTPwmStart(DRIVE_LEFT_FORWARD_PWM, 50, IOT_FREQ);
        }
    }
    else
    {
        car_stop();
    }
}

void car_stop(void)
{
    IoTPwmStart(DRIVE_LEFT_FORWARD_PWM, 50, IOT_FREQ);
    IoTPwmStart(DRIVE_RIGHT_FORWARD_PWM, 50, IOT_FREQ);
}
