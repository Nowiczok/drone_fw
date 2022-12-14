/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>

typedef enum{
    MPU_6050_OK,
    MPU_6050_TIMEOUT,
    MPU_6050_INPUT_ERROR,
    MPU_6050_ERROR
}mpu6050_status_t;

// MPU6050 structure
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;

    float Temperature;

    float estimated_roll;
    float estimated_pitch;
} MPU6050_t;

// Kalman structure
typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_commons_nc_t;

mpu6050_status_t MPU6050_Init(void *I2Cx);

/*void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);*/

mpu6050_status_t MPU6050_Read_All(void *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_commons_nc_t *Kalman, double newAngle, double newRate, double dt);
