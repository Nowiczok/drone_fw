//
// Created by Michał on 03.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_BAROMETER_H
#define DRONE_CONTROLLER_FW_BAROMETER_H

#include <stdbool.h>
#include "stm32g4xx_hal.h"

bool barometer_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c);

#endif //DRONE_CONTROLLER_FW_BAROMETER_H
