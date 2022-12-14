//
// Created by Michał on 25.10.2022.
//

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "opamp.h"

#include "imu.h"
#include "barometer.h"
#include "magnetometer.h"
#include "motors.h"
#include "sensor_fusion.h"
#include "commands.h"
#include "controller.h"
#include "drone.h"
#include "telemetry.h"
#include "curr_meas.h"
#include "hal_wrappers.h"

extern uint8_t uart3_rx_buff;

QueueHandle_t imu_queue;
QueueHandle_t motors_queue;
QueueHandle_t commands_queue;
QueueHandle_t altitude_queue;
QueueHandle_t mag_queue;
QueueHandle_t sens_fus_queue;
QueueHandle_t curr_meas_queue;

void blink_task(void* params);

bool drone_init()
{
    //TODO check success
    WrapperRTOS_init();

    //create queues
    imu_queue = xQueueCreate((UBaseType_t) 10,
                             (UBaseType_t) sizeof(imuMessage_t));
    vQueueAddToRegistry(imu_queue, "imu_q");

    motors_queue = xQueueCreate((UBaseType_t) 10,
                                (UBaseType_t) sizeof(motorsMessage_t));
    vQueueAddToRegistry(motors_queue, "mot_q");

    commands_queue = xQueueCreate((UBaseType_t) 10,
                                (UBaseType_t) sizeof(command_hover_mode_t));
    vQueueAddToRegistry(commands_queue, "com_q");

    altitude_queue = xQueueCreate((UBaseType_t) 10,
                                  (UBaseType_t) sizeof(baro_message_t));
    vQueueAddToRegistry(altitude_queue, "alt_q");

    mag_queue = xQueueCreate((UBaseType_t) 10,
                             (UBaseType_t) sizeof(float));
    vQueueAddToRegistry(mag_queue, "mag_q");

    sens_fus_queue = xQueueCreate((UBaseType_t) 10,
                                  (UBaseType_t) sizeof(fused_data_t));
    vQueueAddToRegistry(sens_fus_queue, "sf_q");

    curr_meas_queue = xQueueCreate((UBaseType_t) 10,
                                  (UBaseType_t) sizeof(curr_meas_message_t));
    vQueueAddToRegistry(curr_meas_queue, "curr_q");

    //initialize hardware
    HAL_TIM_Base_Start_IT(&htim7);  // start high resolution timer
    HAL_UART_Receive_IT(&huart3, &uart3_rx_buff, sizeof(uint8_t));

    BaseType_t result_led;
    result_led = xTaskCreate(blink_task,
                             "blink_task",
                             128,
                             NULL,
                             3,
                             NULL);
    if(result_led != pdPASS)
    {
        return false;
    }

    //initialize modules
    bool res = true;
    res &= imu_init(imu_queue, &hi2c1);
    res &= barometer_init(altitude_queue, &hi2c1);
    res &= sensor_fusion_init(imu_queue, altitude_queue, mag_queue, sens_fus_queue);
    //telemetry_init(sens_fus_queue, &huart3);
    //magnetometer_init(mag_queue, &hi2c1);
    motors_init(motors_queue, NULL, &htim2);
    commands_init(commands_queue);
    controller_init(sens_fus_queue, motors_queue, commands_queue);
    curr_meas_init(curr_meas_queue, &hadc1, &hopamp1);

    if(!res)
    {
        return false;
    }else
        return true;
}

void blink_task(void* params)
{
    for(uint8_t i = 0; i < 30; i++)
    {
        vTaskDelay(100);
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    }
    while(1)
    {
        vTaskDelay(1000);
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    }
}

