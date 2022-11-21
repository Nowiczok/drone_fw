//
// Created by Michał on 19.11.2022.
//

#include "telemetry.h"
#include "imu.h"

typedef struct __attribute__((packed)){
    uint8_t start1;
    uint8_t start2;
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;
}imu_tel_t;

UART_HandleTypeDef *uart_handle_ptr;
static QueueHandle_t imu_queue_local;
static void telemetry_task(void* params);

bool telemetry_init(QueueHandle_t imu_queue, UART_HandleTypeDef *huart)
{
    bool result = false;
    if(huart != NULL)
    {
        uart_handle_ptr = huart;
        imu_queue_local = imu_queue;
        BaseType_t task_creation_res;
        task_creation_res = xTaskCreate(telemetry_task,
                                        "tel_task",
                                        128,
                                        NULL,
                                        3,
                                        NULL);
        result = task_creation_res == pdPASS;
    }
    return result;
}

static void telemetry_task(void* params)
{
    imuMessage_t imu_message;
    while(1)
    {
        xQueuePeek(imu_queue_local, &imu_message, 100);
        //HAL_UART_Transmit();
        vTaskDelay(100);
    }
}