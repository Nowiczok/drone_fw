//
// Created by Michał on 25.10.2022.
//

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "i2c.h"
#include "tim.h"

#include "imu.h"
#include "barometer.h"
#include "motors.h"
#include "commands.h"
#include "controller.h"
#include "drone.h"


QueueHandle_t imu_queue;
QueueHandle_t motors_queue;
QueueHandle_t commands_queue;
QueueHandle_t altitude_queue;

void droneInit()
{
    //create mutexes
    i2c_mutex = xSemaphoreCreateMutex();

    //create queues
    imu_queue = xQueueCreate((UBaseType_t) 10,
                             (UBaseType_t) sizeof(imuMessage_t));
    motors_queue = xQueueCreate((UBaseType_t) 10,
                                (UBaseType_t) sizeof(motorsMessage_t));
    commands_queue = xQueueCreate((UBaseType_t) 10,
                                (UBaseType_t) sizeof(command_hover_mode_t));
    altitude_queue = xQueueCreate((UBaseType_t) 10,
                                  (UBaseType_t) sizeof(float));

    //initialize hardware
    HAL_TIM_Base_Start(&htim7);  // start us timer

    //initialize modules
    imu_init(imu_queue, &hi2c1);
    motors_init(motors_queue, &htim2);
    commands_init(commands_queue);
    controller_init(imu_queue, motors_queue, commands_queue);
    barometer_init(altitude_queue, &hi2c1);
}