//
// Created by Michał on 23.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_IMU_H
#define DRONE_CONTROLLER_FW_IMU_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#define IMU_TASK_EX_PERIOD_MS 1

#define YAW_BUFFER_CAP 256  // number of chunks in buffer
#define YAW_BUFFER_CHUNK_SIZE 5  // chunk size is number of samples that are accumulated together into one ring buffer element
#define Z_ACC_IDLE 9.81f

typedef enum{
    IMU_OK,
    IMU_ERROR,
    IMU_TIMEOUT,
    IMU_INIT_ERROR
}imu_status_t;

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float yaw_accum;

    imu_status_t status;
} imuMessage_t;

bool imu_init(QueueHandle_t output_queue, void *hi2c);

#endif //DRONE_CONTROLLER_FW_IMU_H
