//
// Created by Michał on 30.11.2022.
//

#include "curr_meas.h"
#include "stm32g4xx_hal.h"

//uint16_t adc_buff[ADC_BUFF_LEN];
ADC_HandleTypeDef* hadc_local;
OPAMP_HandleTypeDef* hopamp_local;
static QueueHandle_t output_queue_local;

static void curr_meas_task(void* params);

bool curr_meas_init(QueueHandle_t output_queue, ADC_HandleTypeDef *hadc, OPAMP_HandleTypeDef *hopamp)
{
    output_queue_local = output_queue;
    hadc_local = hadc;
    hopamp_local = hopamp;
    xTaskCreate(curr_meas_task,
                "curr_t",
                128,
                NULL,
                3,
                NULL);
}

static void curr_meas_task(void* params)
{
    taskENTER_CRITICAL();
    HAL_ADCEx_Calibration_Start(hadc_local, ADC_SINGLE_ENDED);
    taskEXIT_CRITICAL();
    vTaskDelay(200);
    HAL_OPAMP_Start(hopamp_local);
    uint16_t raw;
    curr_meas_message_t message = {.drawn_charge = 0.0f};
    while(1)
    {
        HAL_ADC_Start(hadc_local);
        HAL_ADC_PollForConversion(hadc_local, HAL_MAX_DELAY);
        raw = HAL_ADC_GetValue(hadc_local);
        float vol = REFERENCE_VOL/(ADC_NUM_QUANTS-1) * (float)raw;
        message.current = (vol - VOL_BIAS) / (SHUNT_VAL * AMP_GAIN * INT_OPAMP_GAIN);
        message.drawn_charge += message.current * MEAS_PERIOD_MS/1000;
        xQueueSendToFront(output_queue_local, &message, 1);
        vTaskDelay(MEAS_PERIOD_MS);
    }
}

/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

}*/

