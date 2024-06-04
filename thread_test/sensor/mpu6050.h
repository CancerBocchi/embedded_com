#ifndef __MPU6050_H
#define __MPU6050_H
 
#include "stdint.h"
 
#include "cmsis_os2.h"
#include "iot_errno.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"
#include "iot_i2c.h"
#include "ohos_init.h"

#define WIFI_IOT_I2C_BAUDRATE 400000
 
/* MPU6050 */
#define DELAY_MS                    10    // 初始化延时

#define acc_range                   16.0f //单位为单位重力加速度
#define IMU_RESOLUTION              0xFFFF//十六位ADC 一半为十五位
#define gyro_range                  4000  //单位为角度每秒
#define angle_to_rad                0.0174532925f
 
#define MPU6050_GYRO_OUT            0x43  // MPU6050陀螺仪数据寄存器地址
#define MPU6050_ACC_OUT             0x3B  // MPU6050加速度数据寄存器地址
#define MPU6050_ADDRESS             0x68  // MPU6050器件读地址
#define MPU6050_ADDRESS_AD0_LOW     0x68  // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F  // 运动检测阀值设置寄存器
#define MPU6050_RA_MOT_DUR          0x20  // 运动检测时间阀值
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_INT_PIN_CFG      0x37   // 中断/旁路设置寄存器
#define MPU6050_RA_INT_ENABLE       0x38   // 中断使能寄存器
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_DATA_2_BYTE         2
#define MPU6050_DATA_256_BYTE       256
#define MPU6050_DATA_DELAY          100

#define SENSOR_DATA_WIDTH_8_BIT     8      // 8 bit
#define ACCEL_DATA_LEN              6
#define TEMP_DATA_LEN               2

#define MPU6050_CONSTANT_1          13200
#define MPU6050_CONSTANT_2          280
#define MPU6050_CONSTANT_3          13

#define MPU_6050_I2C_NUM        (1)
#define MPU_6050_I2C_SDA        (0)
#define MPU_6050_I2C_SCL        (1)

#define RESET_DELAY_US 20000
#define READ_DATA_DELAY_US 50000

#define IMU_UNIT_TIME (1.0f/(float)osKernelGetTickFreq())
 
typedef enum {
    OFF = 0,
    ON
} E53SC2Status;

enum AccelAxisNum {
    ACCEL_X_AXIS = 0,
    ACCEL_Y_AXIS = 1,
    ACCEL_Z_AXIS = 2,
    ACCEL_AXIS_NUM = 3,
};
enum AccelAxisPart {
    ACCEL_X_AXIS_LSB = 0,
    ACCEL_X_AXIS_MSB = 1,
    ACCEL_Y_AXIS_LSB = 2,
    ACCEL_Y_AXIS_MSB = 3,
    ACCEL_Z_AXIS_LSB = 4,
    ACCEL_Z_AXIS_MSB = 5,
    ACCEL_AXIS_BUTT,
};
enum TempPart {
    TEMP_LSB = 0,
    TEMP_MSB = 1,
};
/* E53_SC2传感器数据类型定义 ------------------------------------------------------------*/
typedef struct {
    short   Temperature;
    double   Accel[3];
    double   Gyro[3];
    unsigned int  red_led;
    unsigned int  ir_led;
} E53SC2Data;

typedef struct gyro_offset{
	short x;
	short y;
	short z;

}Gyro_Offset;

extern float acc_x;
extern float acc_y;
extern float acc_z;

extern float gyro_x;
extern float gyro_y;
extern float gyro_z;

extern short acc_data[3];
extern short gyr_data[3];

extern float yaw;

extern int current_t;
extern int previous_t;
extern float dt;

extern Gyro_Offset GyroOffset;

void mpu_data_convertion();
float Att_GetYaw();

void MPU6050Init(void);
int MPU6050ReadID(void);
int MPU6050ReturnTemp(short *Temperature);
int MPU6050ReadTemp(short *tempData);
int MPU6050ReadGyro(short *gyroData);
int MPU6050ReadAcc(short *accData);

 
#endif /* __MPU6050_H */