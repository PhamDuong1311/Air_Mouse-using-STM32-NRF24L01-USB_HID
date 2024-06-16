/*
 * GY521.c
 *
 *  Created on: Apr 18, 2024
 *      Author: ADMIN
 */
#include "stm32f1xx_hal.h"
#include "GY521.h"
extern I2C_HandleTypeDef hi2c1;


void MPU6050_Init (void){
	uint8_t check,Data;
	// check the sensor ID (SEE WHO AM I DATASHEET)
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	if (check == 104) // the sensor is present
	{
		// setting PWR Registers
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);
		// var sample rate with SMPLRT_DIV_REG
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
		// var accelerometer config with ACCEL_CONFIG_REG
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
		// var Gyro config with GYRO_CONFIG_REG
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}
