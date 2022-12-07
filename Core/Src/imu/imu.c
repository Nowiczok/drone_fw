//
// Created by Michał on 23.10.2022.
//
#include "imu.h"
#include "mpu6050.h"
#include "hal_wrappers.h"
#include "ring_buffer.h"
#include "misc_utils.h"

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include <math.h>

#define GYRO_BIAS_X 0.95f
#define GYRO_BIAS_Y (-2.90f)
#define GYRO_BIAS_Z (-0.93f)

// this struct contains parameters needed to calculate moving accumulated value of angle
typedef struct{
    float accum;
    uint8_t chunking_progress;
    float int_chunk;
    uint8_t chunk_size;
}accum_params_t;

typedef struct{
    float mean_acc_x;
    float mean_acc_y;
    float mean_acc_z;
    float var_acc_x;
    float var_acc_y;
    float var_acc_z;

    float mean_gyro_x;
    float mean_gyro_y;
    float mean_gyro_z;
    float var_gyro_x;
    float var_gyro_y;
    float var_gyro_z;
}imu_basic_calib_data_t;

static MPU6050_t mpu6050_status;

static ring_buffer buffer_yaw;  // ring buffers that are used to implement accumulation
static float yaw_buffer_mem[YAW_BUFFER_CAP] = {0};

static TaskHandle_t imu_taskHandle;
static QueueHandle_t output_queue_local;  // reference to a handle created by app

static I2C_HandleTypeDef* i2c_handle_ptr;  // reference to i2c handel, passed by app

//private functions
void imu_task(void* parameters);
static bool calculate_accum(float sample, ring_buffer *buffer, accum_params_t *accum_params);
static void get_calib_data(imu_basic_calib_data_t* data);

bool imu_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c)
{
    i2c_handle_ptr = hi2c;
    output_queue_local = output_queue;
    BaseType_t task_creation_res;
    task_creation_res = xTaskCreate(imu_task,
                                     "imu_task",
                                     128,
                                     NULL,
                                     3,
                                     &imu_taskHandle);
    return task_creation_res == pdPASS;
}

void imu_task(void* parameters)
{
    MPU6050_Init(i2c_handle_ptr);
    ring_buffer_init(&buffer_yaw, yaw_buffer_mem, YAW_BUFFER_CAP, sizeof(float));

    float alt = 0.0f;
    float vel = 0.0f;
    imuMessage_t new_message = {.yaw_accum = 0.0f};
    accum_params_t accum_yaw = {.accum = 0.0f, .chunking_progress = 0, .int_chunk = 0.0f, .chunk_size = YAW_BUFFER_CHUNK_SIZE};
    uint32_t prev_tim_s = 0;
    uint32_t curr_tim_s = 0;
    float delta_tim_s = 0.0f;
    imu_basic_calib_data_t calib_data;

    //debug only
    //get_calib_data(&calib_data);

    while(1)
    {
        curr_tim_s = HAL_GetTick();
        delta_tim_s = (float)(curr_tim_s - prev_tim_s)/1000.0f;;
        prev_tim_s = curr_tim_s;

        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status, delta_tim_s);  // get data from accel and gyro
        calculate_accum((mpu6050_status.Gz - GYRO_BIAS_Z) * delta_tim_s, &buffer_yaw, &accum_yaw);  // accumulate yaw value

        new_message.acc_x = mpu6050_status.Ax;
        new_message.acc_y = mpu6050_status.Ay;
        new_message.acc_z = mpu6050_status.Az;

        new_message.gyro_x = mpu6050_status.Gx - GYRO_BIAS_X;
        new_message.gyro_y = mpu6050_status.Gy - GYRO_BIAS_Y;
        new_message.gyro_z = mpu6050_status.Gz - GYRO_BIAS_Z;

        new_message.yaw_accum += (mpu6050_status.Gz - GYRO_BIAS_Z) * delta_tim_s;

        xQueueSendToFront(output_queue_local, &new_message, 100);
        vTaskDelay(TASK_EX_PERIOD_MS);
    }
}

// helper procedure that calculates means and variances of gyro and accel, mean needed to compensate for bias error
// and variance needed for Kalman filter operation
void get_calib_data(imu_basic_calib_data_t* data)
{
    uint16_t num_of_samples = 5000;
    for(uint16_t i = 1; i <= num_of_samples; i++)
    {
        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status, 0);
        update_mean(&data->mean_acc_x, mpu6050_status.Ax, i);
        update_mean(&data->mean_acc_y, mpu6050_status.Ay, i);
        update_mean(&data->mean_acc_z, mpu6050_status.Az, i);
    }

    for(uint16_t i = 1; i <= num_of_samples; i++)
    {
        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status, 0);
        update_variance(&data->var_acc_x, data->mean_acc_x, mpu6050_status.Ax, i);
        update_variance(&data->var_acc_y, data->mean_acc_y, mpu6050_status.Ay, i);
        update_variance(&data->var_acc_z, data->mean_acc_z, mpu6050_status.Az, i);
    }

    for(uint16_t i = 1; i <= num_of_samples; i++)
    {
        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status, 0);
        update_mean(&data->mean_gyro_x, mpu6050_status.Gx, i);
        update_mean(&data->mean_gyro_y, mpu6050_status.Gy, i);
        update_mean(&data->mean_gyro_z, mpu6050_status.Gz, i);
    }

    for(uint16_t i = 1; i <= num_of_samples; i++)
    {
        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status, 0);
        update_variance(&data->var_gyro_x, data->mean_gyro_x, mpu6050_status.Gx, i);
        update_variance(&data->var_gyro_y, data->mean_gyro_y, mpu6050_status.Gy, i);
        update_variance(&data->var_gyro_z, data->mean_gyro_z, mpu6050_status.Gz, i);
    }
}

bool calculate_accum(float sample, ring_buffer *buffer, accum_params_t *accum_params)
{
    bool result = false;
    if(buffer != NULL && accum_params != NULL)
    {
        float chunk_to_delete = 0.0f;
        (accum_params->chunking_progress)++;  // increase number of samples in chunk
        accum_params->int_chunk += sample;  // accumulate sample
        if(accum_params->chunking_progress >= accum_params->chunk_size)  // if enough samples were chunked, push it into buffer
        {
            accum_params->chunking_progress = 0;
            ring_buffer_push(buffer, &accum_params->int_chunk);
        }
        if(ring_buffer_get_len(buffer) == YAW_BUFFER_CAP)
        {
            ring_buffer_pull(buffer, &chunk_to_delete);  // if buffer is full, take oldest chunk
        }
        accum_params->accum += sample - chunk_to_delete;
        result = true;
    }
    return result;
}

