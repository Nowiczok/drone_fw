//
// Created by Michał on 29.10.2022.
//

#include "controller.h"

#include "pid.h"
#include "commands.h"

//private variables
TaskHandle_t controller_task_handle;
QueueHandle_t imu_queue_local;
QueueHandle_t motors_queue_local;
QueueHandle_t commands_queue_local;

//private functions
void controller_task(void* params);

bool controller_init(QueueHandle_t imu_queue, QueueHandle_t motors_queue, QueueHandle_t commands_queue)
{
    imu_queue_local = imu_queue;
    motors_queue_local = motors_queue;
    commands_queue_local = commands_queue;
    xTaskCreate(controller_task,
                "controller_task",
                128,
                NULL,
                3,
                &controller_task_handle);
}

void controller_task(void* params)
{

    while(1)
    {

    }
}
