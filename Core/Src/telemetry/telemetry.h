//
// Created by Michał on 19.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_TELEMETRY_H
#define DRONE_CONTROLLER_FW_TELEMETRY_H

#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include <stdbool.h>

bool telemetry_init(QueueHandle_t tel_queue, UART_HandleTypeDef *huart);

#endif //DRONE_CONTROLLER_FW_TELEMETRY_H
