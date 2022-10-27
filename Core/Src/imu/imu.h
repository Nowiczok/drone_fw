//
// Created by Michał on 23.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_IMU_H
#define DRONE_CONTROLLER_FW_IMU_H

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

typedef struct {
    float roll_speed_meas;  // raw readings from gyro
    float pith_speed_meas;
    float yaw_speed_meas;

    float roll_accum_angle;  // integrated angular speeds over fixed period of time
    float pitch_accum_angle;
    float yaw_accum_angle;

    float acc_x;  // raw accelerometer readings
    float acc_y;
    float acc_z;
} imuMessage_t;

void imu_init(QueueHandle_t output_queue, I2C_HandleTypeDef* hi2c);

#endif //DRONE_CONTROLLER_FW_IMU_H
