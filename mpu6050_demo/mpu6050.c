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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "cmsis_os2.h"
#include "iot_errno.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"
#include "iot_i2c.h"
#include "iot_i2c_ex.h"
#include "mpu6050.h"

float acc_x;
float acc_y;
float acc_z;

float gyro_x;
float gyro_y;
float gyro_z;

short acc_data[3];
short gyr_data[3];

float yaw;

int current_t;
int previous_t;
float dt;

Gyro_Offset GyroOffset;
/***************************************************************
 * 函数功能: 得到对应的角度值
 * 输入参数: Addr：I2C设备地址
 *           Reg：目标寄存器
 *           Value：值
 * 返 回 值: 无
 * 说    明: 无
 **************************************************************/
void mpu_data_convertion()
{
	static float fliter_value[2];
	float a = 0.8f;

    MPU6050ReadAcc(acc_data);
    MPU6050ReadGyro(gyr_data);

	acc_x = (float)acc_data[0]*acc_range/(float)IMU_RESOLUTION;
	acc_y = (float)acc_data[1]*acc_range/(float)IMU_RESOLUTION;
	acc_z = (float)acc_data[2]*acc_range/(float)IMU_RESOLUTION;

	gyro_x = ((float)gyr_data[0] - GyroOffset.x)*(float)gyro_range/(float)IMU_RESOLUTION;
	gyro_y = ((float)gyr_data[1] - GyroOffset.y)*(float)gyro_range/(float)IMU_RESOLUTION;
	gyro_z = ((float)gyr_data[2] - GyroOffset.z)*(float)gyro_range/(float)IMU_RESOLUTION;

//一阶低通滤波器
	// fliter_value[1] = (float)((1.0f - a) * (float)fliter_value[0] + a*gyro_z);
	// fliter_value[0] = fliter_value[1];

	// gyro_z = fliter_value[1];
}

/***************************************************************
 * 函数功能: 将得到的数据转化为yaw角度
 * 输入参数: Addr：I2C设备地址
 *           Reg：目标寄存器
 *           Value：值
 * 返 回 值: 无
 * 说    明: 无
 **************************************************************/
float Att_GetYaw()
{
	mpu_data_convertion();
	current_t=osKernelGetTickCount();	//获取系统时间
	dt=(float)(current_t-previous_t) * IMU_UNIT_TIME;//计算时间差,计数器相差的值 * 计数器的单位时间
	previous_t=current_t;//更新时间

	if(fabs(gyro_z)>0.5f)//略去微小分量
		yaw += gyro_z*dt;

	yaw = yaw>360.0f? (yaw - 360.0f) : yaw;
	yaw = yaw<0.0f? (yaw + 360.0f) : yaw;

	return yaw;
}

/***************************************************************
 * 函数功能: 通过I2C写入一个值到指定寄存器内
 * 输入参数: Addr：I2C设备地址
 *           Reg：目标寄存器
 *           Value：值
 * 返 回 值: 无
 * 说    明: 无
 **************************************************************/
static int MPU6050WriteData(uint8_t Reg, uint8_t Value)
{
    uint32_t ret;
    uint8_t send_data[MPU6050_DATA_2_BYTE] = { Reg, Value };
    ret = IoTI2cWrite(MPU_6050_I2C_NUM, (MPU6050_ADDRESS << 1) | 0x00, send_data, sizeof(send_data));
    if (ret != 0) {
        printf("===== Error: I2C write ret = 0x%x! =====\r\n", ret);
        return -1;
    }
    return 0;
}
/***************************************************************
 * 函数功能: 通过I2C写入一段数据到指定寄存器内
 * 输入参数: Addr：I2C设备地址
 *           Reg：目标寄存器
 *           RegSize：寄存器尺寸(8位或者16位)
 *           pBuffer：缓冲区指针
 *           Length：缓冲区长度
 * 返 回 值: HAL_StatusTypeDef：操作结果
 * 说    明: 在循环调用是需加一定延时时间
 **************************************************************/
static int MPU6050WriteBuffer(uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
    uint32_t ret = 0;
    uint8_t send_data[MPU6050_DATA_256_BYTE] = { 0 };

    send_data[0] = Reg;
    for (int j = 0; j < Length; j++) {
        send_data[j + 1] = pBuffer[j];
    }

    ret = IoTI2cWrite(MPU_6050_I2C_NUM, (MPU6050_ADDRESS << 1) | 0x00, send_data, Length + 1);
    if (ret != 0) {
        printf("===== Error: I2C write ret = 0x%x! =====\r\n", ret);
        return -1;
    }
    return 0;
}

/***************************************************************
 * 函数功能: 通过I2C读取一段寄存器内容存放到指定的缓冲区内
 * 输入参数: Addr：I2C设备地址
 *           Reg：目标寄存器
 *           RegSize：寄存器尺寸(8位或者16位)
 *           pBuffer：缓冲区指针
 *           Length：缓冲区长度
 * 返 回 值: HAL_StatusTypeDef：操作结果
 * 说    明: 无
 **************************************************************/
static int MPU6050ReadBuffer(uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
    uint32_t ret = 0;
    IotI2cData mpu6050_i2c_data = { 0 };
    uint8_t buffer[1] = { Reg };
    mpu6050_i2c_data.sendBuf = buffer;
    mpu6050_i2c_data.sendLen = 1;
    mpu6050_i2c_data.receiveBuf = pBuffer;
    mpu6050_i2c_data.receiveLen = Length;
    ret = IoTI2cWriteread(MPU_6050_I2C_NUM, (MPU6050_ADDRESS << 1) | 0x00, &mpu6050_i2c_data);
    if (ret != 0) {
        printf("===== Error: I2C writeread ret = 0x%x! =====\r\n", ret);
        return -1;
    }
    return 0;
}
/***************************************************************
 * 函数功能: 写数据到MPU6050寄存器
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 ***************************************************************/
static void MPU6050WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
    MPU6050WriteData(reg_add, reg_dat);
}

/***************************************************************
 * 函数功能: 从MPU6050寄存器读取数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 ***************************************************************/
static int MPU6050ReadData(uint8_t reg_add, unsigned char *read, uint8_t num)
{
    return MPU6050ReadBuffer(reg_add, read, num);
}

/***************************************************************
 * 函数功能: 读取MPU6050的加速度数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 ***************************************************************/
int MPU6050ReadAcc(short *accData)
{
    int ret;
    uint8_t buf[ACCEL_DATA_LEN];
    ret = MPU6050ReadData(MPU6050_ACC_OUT, buf, ACCEL_DATA_LEN);
    if (ret != 0) {
        return -1;
    }
    accData[ACCEL_X_AXIS] = (buf[ACCEL_X_AXIS_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[ACCEL_X_AXIS_MSB];
    accData[ACCEL_Y_AXIS] = (buf[ACCEL_Y_AXIS_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[ACCEL_Y_AXIS_MSB];
    accData[ACCEL_Z_AXIS] = (buf[ACCEL_Z_AXIS_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[ACCEL_Z_AXIS_MSB];
    return 0;
}

/***************************************************************
 * 函数功能: 读取MPU6050的角速度数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 ***************************************************************/
int MPU6050ReadGyro(short *gyroData)
{
    int ret;
    uint8_t buf[ACCEL_DATA_LEN];
    ret = MPU6050ReadData(MPU6050_GYRO_OUT, buf, ACCEL_DATA_LEN);
    if (ret != 0) {
        return -1;
    }
    gyroData[ACCEL_X_AXIS] = (buf[ACCEL_X_AXIS_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[ACCEL_X_AXIS_MSB];
    gyroData[ACCEL_Y_AXIS] = (buf[ACCEL_Y_AXIS_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[ACCEL_Y_AXIS_MSB];
    gyroData[ACCEL_Z_AXIS] = (buf[ACCEL_Z_AXIS_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[ACCEL_Z_AXIS_MSB];
    return 0;
}

/***************************************************************
 * 函数功能: 读取MPU6050的原始温度数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 ***************************************************************/
int MPU6050ReadTemp(short *tempData)
{
    int ret;
    uint8_t buf[TEMP_DATA_LEN];
    ret = MPU6050ReadData(MPU6050_RA_TEMP_OUT_H, buf, TEMP_DATA_LEN); // 读取温度值
    if (ret != 0) {
        return -1;
    }
    *tempData = (buf[TEMP_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[TEMP_MSB];
    return 0;
}

/***************************************************************
 * 函数功能: 读取MPU6050的温度数据，转化成摄氏度
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 **************************************************************/
int MPU6050ReturnTemp(short *Temperature)
{
    int ret;
    short temp3;
    uint8_t buf[TEMP_DATA_LEN];

    ret = MPU6050ReadData(MPU6050_RA_TEMP_OUT_H, buf, TEMP_DATA_LEN); // 读取温度值
    if (ret != 0) {
        return -1;
    }
    temp3 = (buf[TEMP_LSB] << SENSOR_DATA_WIDTH_8_BIT) | buf[TEMP_MSB];
    *Temperature = (((double)(temp3 + MPU6050_CONSTANT_1)) / MPU6050_CONSTANT_2) - MPU6050_CONSTANT_3;
    return 0;
}

/***************************************************************
 * 函数功能: 初始化MPU6050芯片
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 ***************************************************************/
void MPU6050Init(void)
{
    MPU6050WriteReg(MPU6050_RA_PWR_MGMT_1, 0X80); // 复位MPU6050
    usleep(RESET_DELAY_US);
    MPU6050WriteReg(MPU6050_RA_PWR_MGMT_1, 0X00); // 唤醒MPU6050
    MPU6050WriteReg(MPU6050_RA_INT_ENABLE, 0X00); // 关闭所有中断
    MPU6050WriteReg(MPU6050_RA_USER_CTRL, 0X00);  // I2C主模式关闭
    MPU6050WriteReg(MPU6050_RA_FIFO_EN, 0X00);    // 关闭FIFO
    MPU6050WriteReg(MPU6050_RA_INT_PIN_CFG,0X80); // 中断的逻辑电平模式,设置为0，中断信号为高电；设置为1，中断信号为低电平时。

    MPU6050WriteReg(MPU6050_RA_CONFIG, 0x04);       // 配置外部引脚采样和DLPF数字低通滤波器
    MPU6050WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x1C); // 加速度传感器量程和高通滤波器配置
    MPU6050WriteReg(MPU6050_RA_GYRO_CONFIG,0x18);   // 陀螺仪寄存器
    // MPU6050WriteReg(MPU6050_RA_INT_PIN_CFG, 0X1C);  // INT引脚低电平平时
    // MPU6050WriteReg(MPU6050_RA_INT_ENABLE, 0x40);   // 中断使能寄存器

    unsigned char ret;
    MPU6050ReadData(MPU6050_RA_ACCEL_CONFIG,&ret,1);
    printf("MPU6050 Init:accel config = %x",ret);
    MPU6050ReadData(MPU6050_RA_GYRO_CONFIG,&ret,1);
    printf(", gyro config = %x\n",ret);

    GyroOffset.x = 0;
    GyroOffset.y = 0;
    GyroOffset.z = 0;
    for (uint16_t i = 0; i < 1000; ++i) {
        MPU6050ReadGyro(gyr_data);
        GyroOffset.x += gyr_data[0];
        GyroOffset.y += gyr_data[1];
        GyroOffset.z += gyr_data[2];
        osDelay(1);
    }

    GyroOffset.x /= 1000;
    GyroOffset.y /= 1000;
    GyroOffset.z /= 1000;
}

/***************************************************************
 * 函数功能: 读取MPU6050的ID
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 ***************************************************************/
int MPU6050ReadID(void)
{
    unsigned char Re = 0;
    MPU6050ReadData(MPU6050_RA_WHO_AM_I, &Re, 1); // 读器件地址
    if (Re != 0x68) {
        printf("MPU6050 dectected error!\r\n");
        return -1;
    } else {
        printf("MPU6050 detected!\r\n");
        return 0;
    }
}