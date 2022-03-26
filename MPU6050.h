/*
 * MPU6050.h
 *
 *  Created on: Mar 6, 2022
 *      Author: Fatih
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stdbool.h"
#include "stm32f1xx_hal.h"
#define MPU6050_ADDRESS             0xD0
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1		    0x6B
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_FF_THRESHOLD        0x1D
#define MPU6050_FF_DURATION         0x1E
#define MPU6050_INT_STATUS          0x3A
#define MPU6050_ZMOT_DURATION       0x22
#define MPU6050_ZMOT_THRESHOLD      0x21
#define MPU6050_MOT_DURATION        0x20
#define MPU6050_MOT_THRESHOLD       0x1F
#define MPU6050_INT_ENABLE          0x38

#define RAD_2_DEG                   57.29578

I2C_HandleTypeDef I2C_Handle;
bool MPU6050_Config(I2C_HandleTypeDef mpuI2cHandle);
bool I2C_Read_1Byte(uint8_t REG_ADDR,uint8_t *val);
bool I2C_Write_1Byte(uint8_t REG_ADDR,uint8_t val);
bool I2C_Read_Bytes(uint8_t REG_ADDR,uint8_t *val,uint8_t length);
void MPU6050GetAcceleration(int16_t* x, int16_t* y, int16_t* z);
void MPU6050GetGyro (int16_t* x, int16_t* y, int16_t* z);
void ResetBitInRegister(uint8_t reg, uint8_t pos);
void SetBitInRegister(uint8_t reg, uint8_t pos );
void setFreeFallDuration(uint8_t duration);
void setFreeFallThreshold(uint8_t threshold);
uint8_t GetIntStatus(void);
bool IsFreFallDetect(void);
void SetZeroMotionDetectionDuration(uint8_t duration);
void SetZeroMotionDetectionThreshold(uint8_t threshold);
void SetMotionDetectionDuration(uint8_t duration);
void SetMotionDetectionThreshold(uint8_t threshold);
void SetIntZeroMotionEnabled(void);
void ResetIntZeroMotionEnabled(void);
void SetIntMotionEnabled(void);
void ResetIntMotionEnabled(void);

#endif /* INC_MPU6050_H_ */
