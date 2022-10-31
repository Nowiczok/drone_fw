//
// Created by Michał on 29.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_CONTROLLER_H
#define DRONE_CONTROLLER_FW_CONTROLLER_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"


bool controller_init(QueueHandle_t imu_queue, QueueHandle_t motors_queue, QueueHandle_t commands_queue);

#endif //DRONE_CONTROLLER_FW_CONTROLLER_H
