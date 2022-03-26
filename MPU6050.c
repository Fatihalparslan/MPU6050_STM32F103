/*
 * MPU6050.c
 *
 *  Created on: Mar 6, 2022
 *      Author: Fatih
 */
#include "MPU6050.h"

bool I2C_Read_1Byte(uint8_t REG_ADDR,uint8_t *val)
{

	HAL_StatusTypeDef I2C_Error;


    if(HAL_I2C_Mem_Read (&I2C_Handle, MPU6050_ADDRESS,REG_ADDR,1, val, 1, 1000)==HAL_OK){
    	return true;
    }else
    	return false;

}
bool I2C_Read_Bytes(uint8_t REG_ADDR,uint8_t *val,uint8_t length)
{

	HAL_StatusTypeDef I2C_Error;


    if(HAL_I2C_Mem_Read (&I2C_Handle, MPU6050_ADDRESS,REG_ADDR,1, val, length, 1000)==HAL_OK){
    	return true;
    }else
    	return false;

}
bool I2C_Write_1Byte(uint8_t REG_ADDR,uint8_t val)
{


    uint8_t SendValue=val;
    if(HAL_I2C_Mem_Write(&I2C_Handle, MPU6050_ADDRESS, REG_ADDR, 1, &SendValue, 1, 1000)==HAL_OK){
        return true;
    }else
    	return false;
}

bool MPU6050_Config(I2C_HandleTypeDef mpuI2cHandle){

	I2C_Handle=mpuI2cHandle;
	uint8_t whoami;
	uint8_t val;
	if(I2C_Read_1Byte(MPU6050_REG_WHO_AM_I,&whoami)){
	if (whoami == 0x68){
		val=0x00;
		I2C_Write_1Byte(MPU6050_PWR_MGMT_1,val);
		/*Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		 * Gyroscope Output Rate=8KHz
		 * SMPLRT_DIV=7;
		 * Sample Rate=1KHz
		 */
		val=0x07;
		I2C_Write_1Byte(MPU6050_SMPLRT_DIV,val);
		/*
		 *  Full Scale Range
		 *   ± 2g
		 */
		val=0x00;
		I2C_Write_1Byte(MPU6050_ACCEL_CONFIG,val);
		/*
		 *Full Scale Range
		 *± 250 °/s
		 */
		val=0x00;
		I2C_Write_1Byte(MPU6050_GYRO_CONFIG,val);
	return 1;
	}else
		return 0;
	}else
		return 0;
}
void MPU6050GetAcceleration(int16_t* x, int16_t* y, int16_t* z) {
     uint8_t buffer[6];
	 I2C_Read_Bytes(MPU6050_ACCEL_XOUT_H,buffer,6);
    *x = (int16_t)((int16_t)buffer[0] << 8 | buffer [1]);
    *y = (int16_t)((int16_t)buffer[2] << 8 | buffer [3]);
    *z = (int16_t)((int16_t)buffer[4] << 8 | buffer [5]);
}
void MPU6050GetGyro (int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buffer[6];
	I2C_Read_Bytes(MPU6050_GYRO_XOUT_H,buffer,6);
	*x = (int16_t)(buffer[0] << 8 | buffer [1]);
	*y = (int16_t)(buffer[2] << 8 | buffer [3]);
	*z = (int16_t)(buffer[4] << 8 | buffer [5]);

}


void SetBitInRegister(uint8_t reg, uint8_t pos )
{
    uint8_t reg_val;

        I2C_Read_1Byte(reg,&reg_val);

    	reg_val |= (1 << pos);

    	I2C_Write_1Byte(reg, reg_val);
}
void ResetBitInRegister(uint8_t reg, uint8_t pos)
{
    uint8_t reg_val;

    I2C_Read_1Byte(reg,&reg_val);

    reg_val &= ~(1 << pos);

    I2C_Write_1Byte(reg, reg_val);
}

void setIntFreeFallEnabled(void)
{
	SetBitInRegister(MPU6050_INT_ENABLE, 7);
}

void setFreeFallThreshold(uint8_t threshold)
{
	I2C_Write_1Byte(MPU6050_FF_THRESHOLD, threshold);
}

void setFreeFallDuration(uint8_t duration)
{
	I2C_Write_1Byte(MPU6050_FF_DURATION, duration);
}

uint8_t GetIntStatus(void)
{   uint8_t IntStat;
    return I2C_Read_1Byte(MPU6050_INT_STATUS,&IntStat);
}
bool IsFreFallDetect(void){
	uint8_t IntStat;
	IntStat=GetIntStatus();
	return ((IntStat>>7)&1);
}
void SetIntZeroMotionEnabled(void)
{
	SetBitInRegister(MPU6050_INT_ENABLE, 5);
}
void ResetIntZeroMotionEnabled(void)
{
	ResetBitInRegister(MPU6050_INT_ENABLE, 5);
}

void SetIntMotionEnabled(void)
{
	SetBitInRegister(MPU6050_INT_ENABLE, 6);
}

void ResetIntMotionEnabled(void)
{
	ResetBitInRegister(MPU6050_INT_ENABLE, 6);
}
void SetMotionDetectionThreshold(uint8_t threshold)
{
	I2C_Write_1Byte(MPU6050_MOT_THRESHOLD, threshold);
}
void SetMotionDetectionDuration(uint8_t duration)
{
	I2C_Write_1Byte(MPU6050_MOT_DURATION, duration);
}
void SetZeroMotionDetectionThreshold(uint8_t threshold)
{
	I2C_Write_1Byte(MPU6050_ZMOT_THRESHOLD, threshold);
}
void SetZeroMotionDetectionDuration(uint8_t duration)
{
	I2C_Write_1Byte(MPU6050_ZMOT_DURATION, duration);
}
