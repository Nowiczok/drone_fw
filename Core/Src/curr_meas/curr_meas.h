//
// Created by Michał on 30.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_CURR_MEAS_H
#define DRONE_CONTROLLER_FW_CURR_MEAS_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#define ADC_BUFF_LEN 128
#define SHUNT_VAL 0.001f
#define AMP_GAIN 8.25f
#define INT_OPAMP_GAIN 16
#define VOL_BIAS 0.1236f
#define MEAS_PERIOD_MS 50
#define REFERENCE_VOL 3.3f
#define ADC_NUM_QUANTS 4096

typedef struct{
    float current;
    float drawn_charge;
}curr_meas_message_t;

bool curr_meas_init(QueueHandle_t output_queue, ADC_HandleTypeDef *hadc, OPAMP_HandleTypeDef *hopamp);

#endif //DRONE_CONTROLLER_FW_CURR_MEAS_H
