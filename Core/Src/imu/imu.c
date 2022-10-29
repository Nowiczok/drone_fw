//
// Created by Michał on 23.10.2022.
//
#include "imu.h"
#include "mpu6050.h"
#include "../common/hal_wrappers.h"
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

static ring_buffer buffer_roll;
static ring_buffer buffer_pitch;
static ring_buffer buffer_yaw;  // ring buffers that are used to implement accumulation
static float roll_buffer_mem[ROLL_BUFFER_CAP] = {0};
static float pitch_buffer_mem[PITCH_BUFFER_CAP] = {0};
static float yaw_buffer_mem[YAW_BUFFER_CAP] = {0};

static TaskHandle_t imu_taskHandle;
QueueHandle_t output_queue_local;  // reference to a handle created by app

static I2C_HandleTypeDef* i2c_handle_ptr;  // reference to i2c handel, passed by app
static TIM_HandleTypeDef* us_meas_handle_tim_ptr;

//private functions
void imu_task(void* parameters);
bool calculate_accum(float sample, ring_buffer *buffer, accum_params_t *accum_params);
int32_t calculate_delta_t(int32_t curr, int32_t prev);

bool imu_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim_us)
{
    i2c_handle_ptr = hi2c;
    us_meas_handle_tim_ptr = htim_us;
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
    ring_buffer_init(&buffer_roll, roll_buffer_mem, ROLL_BUFFER_CAP);
    ring_buffer_init(&buffer_pitch, pitch_buffer_mem, PITCH_BUFFER_CAP);
    ring_buffer_init(&buffer_yaw, yaw_buffer_mem, YAW_BUFFER_CAP);

    imuMessage_t newMessage;
    accum_params_t accum_roll = {.accum = 0.0f, .chunking_progress = 0, .int_chunk = 0.0f, .chunk_size = ROLL_BUFFER_CHUNK_SIZE};
    accum_params_t accum_pitch = {.accum = 0.0f, .chunking_progress = 0, .int_chunk = 0.0f, .chunk_size = PITCH_BUFFER_CHUNK_SIZE};
    accum_params_t accum_yaw = {.accum = 0.0f, .chunking_progress = 0, .int_chunk = 0.0f, .chunk_size = YAW_BUFFER_CHUNK_SIZE};
    uint32_t prev_tim_us = 0;
    uint32_t curr_tim_us = 0;
    uint32_t delta_tim_s = 0;

    while(1)
    {
        curr_tim_us = WrapperRTOS_read_t_us(us_meas_handle_tim_ptr);
        delta_tim_s = calculate_delta_t(curr_tim_us, prev_tim_us) * 1.0E-6;
        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status);
        calculate_accum(mpu6050_status.Gx * delta_tim_s, &buffer_roll, &accum_roll);
        calculate_accum(mpu6050_status.Gy * delta_tim_s, &buffer_pitch, &accum_pitch);
        calculate_accum(mpu6050_status.Gz * delta_tim_s, &buffer_yaw, &accum_yaw);

        newMessage.pith_speed_meas = (float)mpu6050_status.Gx;
        newMessage.pith_speed_meas = (float)mpu6050_status.Gy;
        newMessage.yaw_speed_meas = (float)mpu6050_status.Gz;

        newMessage.acc_x = (float)mpu6050_status.Ax;
        newMessage.acc_y = (float)mpu6050_status.Ay;
        newMessage.acc_z = (float)mpu6050_status.Az;

        newMessage.roll_accum_angle = accum_roll.accum;
        newMessage.pitch_accum_angle = accum_pitch.accum;
        newMessage.yaw_accum_angle = accum_yaw.accum;

        xQueueSendToFront(output_queue_local, &newMessage, 100);
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

int32_t calculate_delta_t(int32_t curr, int32_t prev)
{
    int32_t delta;
    if(curr >= prev)  // overflow detection
    {
        delta = curr - prev;
    }else
    {
        delta = US_TIM_PERIOD - prev + curr;
    }

    return delta;
}