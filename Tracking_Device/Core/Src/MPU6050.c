#include "MPU6050.h"
#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


#define INT_ENABLE_REG 0x38
#define MOT_THR_REG 0x1F
#define MOT_DUR_REG 0x20

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
//float Ax, Ay, Az, Gx, Gy, Gz;

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, data;
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
    if (check == 104) {
        data = 0; // Wake up
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
        data = 0x07; // 1KHz rate
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
        data = 0x00; // Config ranges
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
    }
}

void MPU6050_Config_Interrupt(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    data = 20; // Sensitivity
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MOT_THR_REG, 1, &data, 1, 1000);
    data = 1;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MOT_DUR_REG, 1, &data, 1, 1000);
    data = 0x40; // Enable MOT_EN
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, INT_ENABLE_REG, 1, &data, 1, 1000);
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, float* Ax, float* Ay, float* Az) {
    uint8_t d[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, d, 6, 1000);
    *Ax = (int16_t)(d[0] << 8 | d[1]) / 16384.0;
    *Ay = (int16_t)(d[2] << 8 | d[3]) / 16384.0;
    *Az = (int16_t)(d[4] << 8 | d[5]) / 16384.0;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, float* Gx, float* Gy, float* Gz) {
    uint8_t d[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, d, 6, 1000);
    *Gx = (int16_t)(d[0] << 8 | d[1]) / 131.0;
    *Gy = (int16_t)(d[2] << 8 | d[3]) / 131.0;
    *Gz = (int16_t)(d[4] << 8 | d[5]) / 131.0;
}