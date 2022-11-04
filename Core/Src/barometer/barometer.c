//
// Created by Michał on 03.11.2022.
//

#include "FreeRTOS.h"
#include "task.h"
#include "barometer.h"

// private functions
void barometer_task(void* params);

bool barometer_init()
{
    BaseType_t result;
    result = xTaskCreate(barometer_task,
                         "baro_task",
                         128,
                         NULL,
                         3,
                         NULL);
    return result == pdPASS;
}

void barometer_task(void* params)
{

    while(1)
    {

    }
}