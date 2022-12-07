//
// Created by Michał on 19.11.2022.
//

#include "telemetry.h"
#include "sensor_fusion.h"
#include "hal_wrappers.h"
#include <string.h>

typedef struct __attribute__((packed)){
    uint8_t protocol_length;
    uint8_t command_code;
    float roll;
    float pitch;
    float yaw;
    float alt;
    uint16_t checksum;
} telemetry_data_t;

union ibus_frame{
    telemetry_data_t fields;
    uint8_t bytes[sizeof(telemetry_data_t)];
};

static UART_HandleTypeDef *uart_handle_ptr;
static QueueHandle_t act_data_queue_local;
static void telemetry_task(void* params);
static void ibus_checksum(uint8_t frame_bytes[sizeof(union ibus_frame)]);

bool telemetry_init(QueueHandle_t tel_queue, UART_HandleTypeDef *huart)
{
    bool result = false;
    if(huart != NULL)
    {
        uart_handle_ptr = huart;
        act_data_queue_local = tel_queue;
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
    union ibus_frame tel_message;
    tel_message.fields.protocol_length = sizeof(union ibus_frame);
    tel_message.fields.command_code = 0x21;  // random number
    fused_data_t fused_data;
    while(1)
    {
        //xQueuePeek(act_data_queue_local, &imu_message, 100);
        xQueueReceive(act_data_queue_local, &fused_data, 100);  // receive only for now, should be peek
        tel_message.fields.roll = fused_data.roll;
        tel_message.fields.pitch = fused_data.pitch;
        tel_message.fields.yaw = fused_data.yaw;
        tel_message.fields.alt = fused_data.alt;
        ibus_checksum(tel_message.bytes);

        WrapperRTOS_UART_Transmit_DMA(uart_handle_ptr, tel_message.bytes, sizeof(union ibus_frame));
        vTaskDelay(100);
    }
}

void ibus_checksum(uint8_t frame_bytes[sizeof(union ibus_frame)])
{
    uint16_t checksum_cal = 0xffff;
    for(uint32_t i = 0; i < (sizeof(telemetry_data_t) - 2); i++)  // -2 because last 2 bytes are checksum
    {
        checksum_cal -= frame_bytes[i];
    }
    memcpy(&frame_bytes[sizeof(telemetry_data_t)-2], &checksum_cal, sizeof(checksum_cal));
}