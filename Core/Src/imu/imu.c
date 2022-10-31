//
// Created by Michał on 23.10.2022.
//
#include "imu.h"
#include "mpu6050.h"
#include "hal_wrappers.h"
#include "ring_buffer.h"

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>

// this struct contains parameters needed to calculate moving accumulated value of angle
typedef struct{
    float accum;
    uint8_t chunking_progress;
    float int_chunk;
    uint8_t chunk_size;
}accum_params_t;

static MPU6050_t mpu6050_status;

static ring_buffer buffer_yaw;  // ring buffers that are used to implement accumulation
static float yaw_buffer_mem[YAW_BUFFER_CAP] = {0};

static TaskHandle_t imu_taskHandle;
QueueHandle_t output_queue_local;  // reference to a handle created by app

static I2C_HandleTypeDef* i2c_handle_ptr;  // reference to i2c handel, passed by app

//private functions
void imu_task(void* parameters);
bool calculate_accum(float sample, ring_buffer *buffer, accum_params_t *accum_params);

bool imu_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c)
{
    i2c_handle_ptr = hi2c;
    output_queue_local = output_queue;
    bool task_creation_res;
    task_creation_res = xTaskCreate(imu_task,
                                     "imu_task",
                                     128,
                                     NULL,
                                     3,
                                     &imu_taskHandle);
    return task_creation_res;
}

void imu_task(void* parameters)
{
    MPU6050_Init(i2c_handle_ptr);
    ring_buffer_init(&buffer_yaw, yaw_buffer_mem, YAW_BUFFER_CAP);

    float alt = 0.0f;
    float vel = 0.0f;
    imuMessage_t new_message;
    accum_params_t accum_yaw = {.accum = 0.0f, .chunking_progress = 0, .int_chunk = 0.0f, .chunk_size = YAW_BUFFER_CHUNK_SIZE};
    uint32_t prev_tim_us = 0;
    uint32_t curr_tim_us = 0;
    float delta_tim_s = 0.0f;

    while(1)
    {
        curr_tim_us = WrapperRTOS_read_t_us();
        delta_tim_s = (float)(calculate_delta_t(curr_tim_us, prev_tim_us)) * 1.0E-6f;
        prev_tim_us = curr_tim_us;

        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status, delta_tim_s);  // get data from accel and gyro
        calculate_accum(mpu6050_status.Gz * delta_tim_s, &buffer_yaw, &accum_yaw);  // accumulate yaw value

        new_message.roll = mpu6050_status.estimated_roll;
        new_message.pitch = mpu6050_status.estimated_pitch;

        new_message.yaw_accum_angle = accum_yaw.accum;

        alt = delta_tim_s*vel + (delta_tim_s*delta_tim_s)*(mpu6050_status.Az - Z_ACC_IDLE); //estimate altitude
        vel = delta_tim_s*(mpu6050_status.Az - Z_ACC_IDLE);  // update velocity
        new_message.alt = alt;

        xQueueSendToFront(output_queue_local, &new_message, 100);
        vTaskDelay(TASK_EX_PERIOD_MS);
    }

}

bool calculate_accum(float sample, ring_buffer *buffer, accum_params_t *accum_params)
{
    bool result = false;
    if(buffer != NULL && accum_params != NULL)
    {
        float chunk_to_delete = 0.0f;
        (accum_params->chunking_progress)++;
        accum_params->int_chunk += sample;
        if(accum_params->chunking_progress >= accum_params->chunk_size)
        {
            accum_params->chunking_progress = 0;
            ring_buffer_push(buffer, accum_params->int_chunk);
        }
        ring_buffer_pull(buffer, &chunk_to_delete);
        accum_params->accum += accum_params->int_chunk - chunk_to_delete;
        result = true;
    }
    return result;
}

