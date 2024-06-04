#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_io.h"

#include "LC_307.h"

#define STACK_SIZE 1024

static void* LC_307_Demo_Task()
{
    LC_307_Init();
    
    printf("LC_307_Demo test\n");

    while(1)
    {
        LC_307_Get_Data(&Data_Out);
        printf("%d,%d,%d,%d,%d\n",Data_Out.Vx,Data_Out.Vy,Data_Out.Groung_H,Data_Out.dt,Data_Out.version);

        osDelay(1);
    }
    
    return NULL;
}

/*
 * 任务入口函数
 * Task Entry Function
 */
static void LC_307_Demo_Entry(void)
{
    osThreadAttr_t attr = {0};

    printf("[LC_307_Demo] LC_307_Demo_Entry()\n");

    attr.name = "LC_307_Demo_Task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = STACK_SIZE;      
    attr.priority = osPriorityNormal;  

    if (osThreadNew((osThreadFunc_t)LC_307_Demo_Task, NULL, &attr) == NULL) 
    {
        printf("[LC_307_demo] Falied to create LC_307_Demo_Task!\n");
    }
}

/*
 * 注册模块
 * registration module
 */
APP_FEATURE_INIT(LC_307_Demo_Entry);