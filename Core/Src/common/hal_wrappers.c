//
// Created by Michał on 25.10.2022.
//

#include "hal_wrappers.h"
#include "semphr.h"
#include "tim.h"

SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t tim_mutex;

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
extern uint64_t high_res_time;
uint32_t WrapperRTOS_read_t_10us()
{
    uint32_t time_10us = 0;
    if(xSemaphoreTake(tim_mutex, (TickType_t)100) == pdTRUE) // use timer only if none other task uses it
    {
        time_10us =  high_res_time + htim7.Instance->CNT;
        xSemaphoreGive(tim_mutex);
    }
    return time_10us;
}

uint32_t calculate_delta_t(uint32_t curr, uint32_t prev)
{
    uint32_t delta;
    if(curr >= prev)  // overflow detection
    {
        delta = curr - prev;
    }else
    {
        delta = US_TIM_PERIOD - prev + curr;
    }

    return delta;
}