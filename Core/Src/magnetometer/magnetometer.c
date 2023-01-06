//
// Created by Michał on 20.11.2022.
//

#include "magnetometer.h"
#include "MMC5983.h"

static void magnetometer_task(void* params);

static QueueHandle_t output_queue_local;
static void *i2c_handle_ptr;

bool magnetometer_init(QueueHandle_t output_queue, void *hi2c)
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

void magnetometer_task(void* params)
{
    mag_message_t new_message;
    mmc5983_dev_t mag;
    mmc5983_status_t res;

    res = mmc5983_init(&mag, i2c_handle_ptr);
    while(1)
    {
        vTaskDelay(1);
        res = mmc5983_read_all(&mag);
        if(res == MMC5983_OK){
            new_message.magX = mag.mag_x;
            new_message.magY = mag.mag_y;
            new_message.magZ = mag.mag_z;

            new_message.status = MAG_OK;
        }else if(res == MMC5983_TIMEOUT){
            new_message.status = MAG_TIMEOUT;
        }else{
            new_message.status = MAG_ERROR;
        }
        xQueueSendToFront(output_queue_local, &new_message, 10);
    }
}