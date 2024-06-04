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

/* 实现电机转动的编码器计数, 用于计算速率和距禄1�7
 * 电机编码器是AB信号的形弄1�7, 当A信号领先B信号旄1�7, 脉冲正计敄1�7, 反之脉冲负计敄1�7
 * 由于3861中断棢�测只能在上跳变和下跳变之间二选一, 这里采用中断中改变检测类型设定的方法
 * 实现了上下跳变连续检测的AB脉冲计数.
 *
 * Encoder counting for motor rotation, used to calculate speed and distance.
 * The motor encoder is in the form of AB signal. When A signal is ahead of B signal,
 * the pulse counts positively, and vice versa. Since 3861 interrupt detection can only
 * select one from the two options of up jump and down jump, the method of changing the
 * detection type setting in interrupt is adopted
 * The AB pulse counting of up and down jump continuous detection is realized
 */

#include <stdio.h>
#include <stdint.h>
#include <hi_stdlib.h>
#include "compile_define.h"
#include "iot_gpio_ex.h"
#include "wheel_codec.h"

/*
 * iot_gpio.h的极性设置与示波器实测是反的, 重新定义丢�个类垄1�7
 *
 * The polarity setting in iot_gpio.h is actually opposite to the measurement of the oscilloscope.
 * Redefine a type
 */
typedef enum {
    /* Interrupt at a high level or rising edge */
    IOT_GPIO_INT_EDGE_RISE = 0,
    IOT_GPIO_INT_EDGE_FALL
} GpioIntEdgePolarity;

typedef struct {
    int pin_name_a;
    int pin_name_b;
    int16_t counter;
    GpioIntEdgePolarity polar;
} WHEEL_CODEC_STRUCT;


#if WHEEL_DIRECTION_REVERT
#define COUNT_POSITVE      (-1)
#define COUNT_NEGTITVE      (1)
#else
#define COUNT_POSITVE       (1)
#define COUNT_NEGTITVE      (-1)
#endif

WHEEL_CODEC_STRUCT g_wheel_left;
WHEEL_CODEC_STRUCT g_wheel_right;

void INIT_GPIO_IN(IotIoName NAME, unsigned char FUNC)
{
    IoTGpioInit(NAME);
    IoSetFunc(NAME, FUNC);
    IoTGpioSetDir(NAME, IOT_GPIO_DIR_IN);
    IoSetPull(NAME, IOT_IO_PULL_UP);
}

void wheel_codec_svr(char *arg)
{
    IotGpioValue value_b;
    uint32_t * const reg = (uint32_t *)(0x5000603C); // reg GPIO_INT_POLARITY
    WHEEL_CODEC_STRUCT* pt = (WHEEL_CODEC_STRUCT*)(arg);

    if (IoTGpioGetInputVal(pt -> pin_name_b, &value_b) != IOT_SUCCESS) {
        printf("read wheel right SIGNAL B read fail\n");
    }

    if (value_b == 0) {
        pt -> counter += (pt -> polar == IOT_GPIO_INT_EDGE_RISE) ? COUNT_POSITVE : COUNT_NEGTITVE;
    } else {
        pt -> counter += (pt -> polar == IOT_GPIO_INT_EDGE_RISE) ? COUNT_NEGTITVE : COUNT_POSITVE;
    }
    /* 加��中断极性转换时闄1�7 */
    pt -> polar ^= 1;
    if (pt -> polar) {
        *reg |= (1 << pt->pin_name_a);
    } else {
        *reg &= ~(1 << pt->pin_name_a);
    }
}

int16_t get_wheel_cnt_right(void)
{
    return g_wheel_right.counter;
}

int16_t get_wheel_cnt_left(void)
{
    return g_wheel_left.counter;
}

void get_wheel_cnt(int16_t *left, int16_t *right)
{
    *left = g_wheel_left.counter;
    *right = g_wheel_right.counter;
    return;
}

int get_encoder_left(){
    static int left_pre;
    static int left_now;
    int return_val;
    left_now = get_wheel_cnt_left();

    if (abs(left_pre - left_now) > 100)
        return_val = 0;
    else
        return_val = left_now - left_pre;
    left_pre = left_now;

    return return_val;
}

int get_encoder_right(){
    static int right_pre;
    static int right_now;
    int return_val;
    right_now = get_wheel_cnt_left();

    if (abs(right_pre - right_now) > 100)
        return_val = 0;
    else
        return_val = right_now - right_pre;
    
    right_pre = right_now;

    return return_val;

}

void init_wheel_codec(void)
{
    memset_s(&g_wheel_left, sizeof(g_wheel_left), 0, sizeof(g_wheel_left));
    memset_s(&g_wheel_right, sizeof(g_wheel_right), 0, sizeof(g_wheel_right));
    g_wheel_left.pin_name_a = WHEEL_LEFT_CA_PIN_NAME;
    g_wheel_left.pin_name_b = WHEEL_LEFT_CB_PIN_NAME;
    g_wheel_right.pin_name_a = WHEEL_RIGHT_CA_PIN_NAME;
    g_wheel_right.pin_name_b = WHEEL_RIGHT_CB_PIN_NAME;
    
    INIT_GPIO_IN(WHEEL_LEFT_CA_PIN_NAME, WHEEL_LEFT_CA_PIN_FUNC);
    INIT_GPIO_IN(WHEEL_LEFT_CB_PIN_NAME, WHEEL_LEFT_CB_PIN_FUNC);
    INIT_GPIO_IN(WHEEL_RIGHT_CA_PIN_NAME, WHEEL_RIGHT_CA_PIN_FUNC);
    INIT_GPIO_IN(WHEEL_RIGHT_CB_PIN_NAME, WHEEL_RIGHT_CB_PIN_FUNC);

    IoTGpioRegisterIsrFunc(WHEEL_LEFT_CA_PIN_NAME, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_INT_EDGE_RISE, wheel_codec_svr, (char *)(&g_wheel_left));
    IoTGpioRegisterIsrFunc(WHEEL_RIGHT_CA_PIN_NAME, IOT_INT_TYPE_EDGE, IOT_GPIO_INT_EDGE_RISE,
                           wheel_codec_svr, (char *)(&g_wheel_right));

    printf("init_wheel_codec\n");
}

