//
// Created by Michał on 23.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_IMU_H
#define DRONE_CONTROLLER_FW_IMU_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#define US_TIM_PERIOD 0xffff
#define TASK_EX_PERIOD_MS 1

#define ROLL_BUFFER_CAP 256  // number of chunks in buffer
#define ROLL_BUFFER_CHUNK_SIZE 5  // chunk size is number of samples that are accumulated together into one ring buffer element
#define PITCH_BUFFER_CAP 256
#define PITCH_BUFFER_CHUNK_SIZE 5
#define YAW_BUFFER_CAP 256
#define YAW_BUFFER_CHUNK_SIZE 5

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

bool imu_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim_us);

#endif //DRONE_CONTROLLER_FW_IMU_H
