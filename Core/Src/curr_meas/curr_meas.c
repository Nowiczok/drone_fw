//
// Created by Michał on 30.11.2022.
//

#include "curr_meas.h"
#include "stm32g4xx_hal.h"

uint8_t adc_buff[ADC_BUFF_LEN];
static QueueHandle_t output_queue_local;

static void curr_meas_task(void* params);

bool curr_meas_init(QueueHandle_t output_queue)
{
    output_queue_local = output_queue;
    xTaskCreate(curr_meas_task,
                "curr_t",
                128,
                NULL,
                3,
                NULL);
}

static void curr_meas_task(void* params)
{
    //HAL_ADC_Start_DMA()
    while(1)
    {

    }
}