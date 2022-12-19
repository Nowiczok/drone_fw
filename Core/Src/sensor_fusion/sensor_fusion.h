//
// Created by Michał on 05.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_SENSOR_FUSION_H
#define DRONE_CONTROLLER_FW_SENSOR_FUSION_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>

typedef enum{
    SENS_FUS_ANG_OK,
    SENS_FUS_ANG_NO_DATA,
    SENS_FUS_ANG_DATA_ERROR
}sens_fus_ang_status_t;

typedef enum{
    SENS_FUS_BAR_OK,
    SENS_FUS_BAR_NO_DATA,
    SENS_FUS_BAR_DATA_ERROR
}sens_fus_bar_status_t;

typedef struct{
    float roll;
    float pitch;
    float yaw;
    float alt;
    sens_fus_ang_status_t ang_status;
    sens_fus_bar_status_t bar_status;
} fused_data_t;

bool sensor_fusion_init(QueueHandle_t imu_queue, QueueHandle_t baro_queue, QueueHandle_t magnetometer_queue,
                        QueueHandle_t output_queue);

#endif //DRONE_CONTROLLER_FW_SENSOR_FUSION_H
