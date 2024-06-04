#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "cmsis_os2.h"
#include "iot_errno.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"
#include "iot_i2c.h"
#include "ohos_init.h"
#include "hi_io.h"

#include "mpu6050.h"

#define MPU_TASK_STACK_SIZE (1024 * 4)
#define MPU_TASK_PRIO 25

#define WIFI_IOT_I2C_BAUDRATE 400000
#define TASK_DELAY_1S 1000000


static void gpio_init()
{
    // GPIO_0 multiplexed to I2C1_SDA
    IoTGpioInit(MPU_6050_I2C_SDA);
    IoTGpioSetFunc(MPU_6050_I2C_SDA, HI_IO_FUNC_GPIO_0_I2C1_SDA);

    // GPIO_1 multiplexed to I2C1_SCL
    IoTGpioInit(MPU_6050_I2C_SCL);
    IoTGpioSetFunc(MPU_6050_I2C_SCL, HI_IO_FUNC_GPIO_1_I2C1_SCL);

    // baudrate: 400kbps
    IoTI2cInit(MPU_6050_I2C_NUM, WIFI_IOT_I2C_BAUDRATE);
}

/**
 * @brief led task output high and low levels to turn on and off LED
 *
 */
static void MpuTask(void)
{
    uint8_t ret;

    gpio_init();

    MPU6050Init();

    while (1) {

        Att_GetYaw();
        // printf("acc data:%.2f,%.2f,%.2f\n",acc_x,acc_y,acc_z);
        //printf("gyr data:%.2f,%.2f,%.2f\n",gyro_x,gyro_y,gyro_z);
        //printf("gyr data:%d,%d,%d\n",gyr_data[0],gyr_data[1],gyr_data[2]);
        
        printf("%.2f,%d,%.2f,%d,%d,%.3f\n",gyro_z,gyr_data[0],yaw,GyroOffset.z,current_t,dt);
        
        osDelay(1);
    }
}

/**
 * @brief Main Entry of the Led Example
 *
 */
static void MpuExampleEntry(void)
{
    osThreadAttr_t attr;

    attr.name = "MPU_Task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = MPU_TASK_STACK_SIZE;
    attr.priority = MPU_TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)MpuTask, NULL, &attr) == NULL) {
        printf("Failed to create LedTask!\n");
    }
}

APP_FEATURE_INIT(MpuExampleEntry);