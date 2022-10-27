//
// Created by Michał on 23.10.2022.
//
#include "imu.h"
#include "mpu6050.h"
#include "../common/hal_wrappers.h"

#include "FreeRTOS.h"
#include "queue.h"

MPU6050_t mpu6050_status;

TaskHandle_t imu_taskHandle;
static QueueHandle_t output_queue_local;  // reference to a handle created by app

I2C_HandleTypeDef* i2c_handle_ptr;

//private functions
void imu_task(void* parameters);
float calculate_accum(float accum);

void imu_init(QueueHandle_t output_queue, I2C_HandleTypeDef* hi2c)
{
    i2c_handle_ptr = hi2c;
    output_queue_local = output_queue;
    xTaskCreate(imu_task,
                "imu_task",
                128,
                NULL,
                3,
                &imu_taskHandle);
}

void imu_task(void* parameters)
{
    MPU6050_Init(i2c_handle_ptr);
    imuMessage_t newMessage;

    while(1)
    {
        MPU6050_Read_All(i2c_handle_ptr, &mpu6050_status);

        newMessage.pith_speed_meas = (float)mpu6050_status.Gx;
        newMessage.pith_speed_meas = (float)mpu6050_status.Gy;
        newMessage.yaw_speed_meas = (float)mpu6050_status.Gz;

        newMessage.acc_x = (float)mpu6050_status.Ax;
        newMessage.acc_y = (float)mpu6050_status.Ay;
        newMessage.acc_z = (float)mpu6050_status.Az;



        xQueueSendToFront(output_queue_local, &newMessage, 100);
        vTaskDelay(1);
    }

}

float calculate_accum(float accum)
{

}
