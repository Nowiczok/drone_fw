//
// Created by Michał on 25.10.2022.
//

#include "drone.h"
#include "../common/hal_wrappers.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "imu.h"
#include "motors.h"
#include "i2c.h"
#include "tim.h"

void droneInit()
{
    //create mutexes
    i2c_mutex = xSemaphoreCreateMutex();

    //create queues
    imu_queue = xQueueCreate((UBaseType_t) 10,
                             (UBaseType_t) sizeof(imuMessage_t));
    motors_queue = xQueueCreate((UBaseType_t) 10,
                                (UBaseType_t) sizeof(motorsMessage_t));

    //initialize hardware
    HAL_TIM_Base_Start(&htim7);

    //initialize modules
    imu_init(imu_queue, &hi2c1, NULL);
    motors_init(motors_queue, &htim2);
}