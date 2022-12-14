//
// Created by Michał on 27.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_MOTORS_H
#define DRONE_CONTROLLER_FW_MOTORS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define MOTOR_NUM 4

typedef struct{
    float throttle;
    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;
} motorsMessage_t;

void motors_init(QueueHandle_t input_queue, QueueHandle_t output_queue, void *htim);

#endif //DRONE_CONTROLLER_FW_MOTORS_H
