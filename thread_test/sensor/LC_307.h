#ifndef __LC_307_H__
#define __LC_307_H__

#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_io.h"

#define LC_307_UART_RX_PIN   (1)
#define LC_307_UART_TX_PIN   (0)
#define LC_307_UART_IDX      (1)

#define LC_307_UART_BAUD     (19200)
#define LC_307_DATA_LENGHTH  (14)

#define SENSOR_IIC_ADDR      (0xdc)

typedef struct{
    int16_t Vx;
    int16_t Vy;
    int16_t dt;
    int16_t valid;//判断数据是否可用
    int8_t version;
    int16_t Groung_H;
}LC_307_Data;

extern LC_307_Data Data_Out;

void LC_307_Get_Data(LC_307_Data *Data_Out);
uint8_t LC_307_Init();

#endif