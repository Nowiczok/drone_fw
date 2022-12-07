//
// Created by Michał on 30.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_CURR_MEAS_H
#define DRONE_CONTROLLER_FW_CURR_MEAS_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

#define ADC_BUFF_LEN 128

bool curr_meas_init(QueueHandle_t output_queue);

#endif //DRONE_CONTROLLER_FW_CURR_MEAS_H
