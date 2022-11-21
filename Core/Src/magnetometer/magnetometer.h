//
// Created by Michał on 20.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_MAGNETOMETER_H
#define DRONE_CONTROLLER_FW_MAGNETOMETER_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include "stm32g4xx_hal.h"

bool magnetometer_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c);

#endif //DRONE_CONTROLLER_FW_MAGNETOMETER_H
