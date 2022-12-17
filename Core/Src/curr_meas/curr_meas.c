//
// Created by Michał on 30.11.2022.
//

#include "hal_wrappers.h"
#include "curr_meas.h"

//uint16_t adc_buff[ADC_BUFF_LEN];
void* hadc_local;
void* hopamp_local;
static QueueHandle_t output_queue_local;

static void curr_meas_task(void* params);

bool curr_meas_init(QueueHandle_t output_queue, void *hadc, void *hamp)
{
    output_queue_local = output_queue;
    hadc_local = hadc;
    hopamp_local = hamp;
    xTaskCreate(curr_meas_task,
                "curr_t",
                128,
                NULL,
                3,
                NULL);
}

static void curr_meas_task(void* params)
{
    uint16_t raw;
    float vol;
    curr_meas_message_t message = {.drawn_charge = 0.0f};
    uint32_t curr_tim;
    uint32_t prev_tim = 0;
    float delta_tim_s;

    WrapperRTOS_ADC_init(hadc_local, hopamp_local);
    while(1)
    {
        Wrapper_RTOS_status_t adc_res = WrapperRTOS_ADC_read_blocking(hadc_local, &raw, 100);
        if(adc_res == WrRTOS_OK){
            // in case of failure, delta tim will be longer and accuracy will not drop so significantly
            curr_tim = WrapperRTOS_read_t_1ms();
            delta_tim_s = (float)(curr_tim - prev_tim)/1000.0f;
            prev_tim = curr_tim;
            vol = REFERENCE_VOL/(ADC_NUM_QUANTS-1) * (float)raw;
            message.current = (vol - VOL_BIAS) / (SHUNT_VAL * AMP_GAIN * INT_OPAMP_GAIN);
            message.drawn_charge += message.current * delta_tim_s;
            message.status = CURR_MEAS_OK;
        }else{
            message.status = CURR_MEAS_ADC_ERROR;
        }
        xQueueSendToFront(output_queue_local, &message, 1);
        vTaskDelay(MEAS_PERIOD_MS);
    }
}

