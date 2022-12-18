//
// Created by Michał on 05.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_SENSOR_FUSION_H
#define DRONE_CONTROLLER_FW_SENSOR_FUSION_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>

typedef enum{
    SENS_FUS_OK,
    SENS_FUS_NO_DATA,
    SENS_FUS_DATA_ERROR
}sens_fus_status_t;

typedef struct{
    float roll;
    float pitch;
    float yaw;
    float alt;
    sens_fus_status_t status;
} fused_data_t;

bool sensor_fusion_init(QueueHandle_t imu_queue, QueueHandle_t baro_queue, QueueHandle_t magnetometer_queue,
                        QueueHandle_t output_queue);

#endif //DRONE_CONTROLLER_FW_SENSOR_FUSION_H
