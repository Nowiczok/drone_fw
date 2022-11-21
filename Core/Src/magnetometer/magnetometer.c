//
// Created by Michał on 20.11.2022.
//

#include "magnetometer.h"
#include "MMC5983.h"

static void magnetometer_task(void* params);

static QueueHandle_t output_queue_local;
static I2C_HandleTypeDef *i2c_handle_ptr;

bool magnetometer_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c)
{
    bool result = false;
    if(hi2c != NULL)
    {
        output_queue_local = output_queue;
        i2c_handle_ptr = hi2c;
        BaseType_t rtos_rslt;
        rtos_rslt = xTaskCreate(magnetometer_task,
                                "mag_task",
                                128,
                                NULL,
                                3,
                                NULL);
        result = rtos_rslt == pdPASS;
    }
    return result;
}

static void magnetometer_task(void* params)
{
    mmc5983_dev_t mag;
    mmc5983_init(&mag, i2c_handle_ptr);
    while(1)
    {
        vTaskDelay(1);
        mmc5983_read_all(&mag);
        //xQueueSendToFront(output_queue_local, )
    }
}