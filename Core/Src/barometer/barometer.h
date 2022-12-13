//
// Created by Michał on 03.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_BAROMETER_H
#define DRONE_CONTROLLER_FW_BAROMETER_H

#include <stdbool.h>

#define BARO_TASK_PERIOD_MS 1
typedef enum{
    BARO_OK,
    BARO_ERROR,
    BARO_INIT_ERROR
}barometer_status_t;

typedef struct{
    float alt;
    barometer_status_t status;
}baro_message_t;

bool barometer_init(QueueHandle_t output_queue, void *hi2c);

#endif //DRONE_CONTROLLER_FW_BAROMETER_H
