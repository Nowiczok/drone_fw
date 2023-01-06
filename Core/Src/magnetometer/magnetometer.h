//
// Created by Michał on 20.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_MAGNETOMETER_H
#define DRONE_CONTROLLER_FW_MAGNETOMETER_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>

typedef enum{
    MAG_OK,
    MAG_TIMEOUT,
    MAG_ERROR
}mag_status_t;

typedef struct {
    float magX;
    float magY;
    float magZ;

    mag_status_t status;
}mag_message_t;

bool magnetometer_init(QueueHandle_t output_queue, void *hi2c);

#endif //DRONE_CONTROLLER_FW_MAGNETOMETER_H
