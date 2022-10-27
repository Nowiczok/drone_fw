//
// Created by Michał on 25.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_DRONE_H
#define DRONE_CONTROLLER_FW_DRONE_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#define MAIN_TASK_STACK_DEPTH 128
//mutexes
extern SemaphoreHandle_t i2c_mutex;

//queues
extern QueueHandle_t imu_queue;
extern QueueHandle_t motors_queue;

void droneInit();

#endif //DRONE_CONTROLLER_FW_DRONE_H
