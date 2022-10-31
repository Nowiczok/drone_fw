//
// Created by Michał on 29.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_CONTROLLER_H
#define DRONE_CONTROLLER_FW_CONTROLLER_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#define ROLL_PID_P 0.1f
#define ROLL_PID_I 0.0f
#define ROLL_PID_D 0.0f
#define ROLL_SLEW_RATE 25

#define PITCH_PID_P 0.1f
#define PITCH_PID_I 0.0f
#define PITCH_PID_D 0.0f
#define PITCH_SLEW_RATE 25

#define YAW_PID_P 0.1f
#define YAW_PID_I 0.0f
#define YAW_PID_D 0.0f
#define YAW_SLEW_RATE 25

#define ALT_PID_P 0.1f
#define ALT_PID_I 0.0f
#define ALT_PID_D 0.0f
#define ALT_SLEW_RATE 25

bool controller_init(QueueHandle_t imu_queue, QueueHandle_t motors_queue, QueueHandle_t commands_queue);

#endif //DRONE_CONTROLLER_FW_CONTROLLER_H
