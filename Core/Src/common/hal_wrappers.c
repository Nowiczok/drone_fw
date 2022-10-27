//
// Created by Michał on 25.10.2022.
//

#include "hal_wrappers.h"
#include "semphr.h"

SemaphoreHandle_t i2c_mutex;

QueueHandle_t imu_queue;
QueueHandle_t motors_queue;

// wrappers on standard HAL i2c mem read/write functions, to make them more FreeRTOS friendly
HAL_StatusTypeDef WrapperRTOS_i2cMemRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                        uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    if(i2c_mutex != NULL)  // check whether mutex is valid
    {
        if(xSemaphoreTake(i2c_mutex, (TickType_t)Timeout) == pdTRUE) // use HAL i2c only if none other task uses it
        {
            //TODO: change it to nonblocking
            status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
            xSemaphoreGive(i2c_mutex);
        }
    }
    return status;
}

HAL_StatusTypeDef WrapperRTOS_i2cMemWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                         uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    if(i2c_mutex != NULL)  // check whether mutex is valid
    {
        if(xSemaphoreTake(i2c_mutex, (TickType_t)Timeout) == pdTRUE) // use HAL i2c only if none other task uses it
        {
            //TODO: change it to nonblocking
            status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
            xSemaphoreGive(i2c_mutex);
        }
    }
    return status;
}