#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

void MPU6050_init(void); //Initialize the MPU 
void MPU6050_Config_Interrupt(I2C_HandleTypeDef *hi2c); //Configure the Interrupt of MPU
void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az); //Read MPU Accelerator 
void MPU6050_Read_Gyro (float *Gx, float *Gy, float *Gz); //Read MPU Gyroscope

#endif /* MPU6050_H */