//
// Created by Michał on 25.10.2022.
//

#include "hal_wrappers.h"
#include "semphr.h"
#include "stm32g4xx_hal.h"

#include "tim.h"
#include "adc.h"

static SemaphoreHandle_t i2c_mutex = NULL;
static SemaphoreHandle_t uart_mutex = NULL;
static SemaphoreHandle_t tim_mutex = NULL;

bool WrapperRTOS_init()
{
    //create mutexes
    //TODO check if mutexes where successfully created
    i2c_mutex = xSemaphoreCreateMutex();
    uart_mutex = xSemaphoreCreateMutex();
    tim_mutex = xSemaphoreCreateMutex();
}

// wrappers on standard HAL i2c mem read/write functions, to make them more FreeRTOS friendly
Wrapper_RTOS_status_t WrapperRTOS_i2cMemRead(void *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                         uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    Wrapper_RTOS_status_t out_res;
    HAL_StatusTypeDef hal_res;
    if(i2c_mutex != NULL && hi2c != NULL && pData != NULL){
        if(xSemaphoreTake(i2c_mutex, (TickType_t)Timeout) == pdTRUE){  // use HAL i2c only if none other task uses it
            //TODO: change it to nonblocking
            hal_res = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
            if(hal_res == HAL_OK){
                out_res = WrRTOS_OK;
            }else{
                out_res = WrRTOS_ERROR;  // i2c transaction unsuccessful
            }
            xSemaphoreGive(i2c_mutex);
        }else{
            out_res = WrRTOS_TIMEOUT;  // semaphore taking timeout
        }
    }else{
        out_res = WrRTOS_INPUT_ERROR;  // mutex or one of pointers uninitialized
    }
    return out_res;
}

Wrapper_RTOS_status_t WrapperRTOS_i2cMemWrite(void *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                              uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    Wrapper_RTOS_status_t out_res;
    HAL_StatusTypeDef hal_res;
    if(i2c_mutex != NULL && pData != NULL)  // check whether mutex is valid
    {
        if(xSemaphoreTake(i2c_mutex, (TickType_t)Timeout) == pdTRUE) // use HAL i2c only if none other task uses it
        {
            //TODO: change it to nonblocking
            hal_res = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
            if(hal_res == HAL_OK){
                out_res = WrRTOS_OK;
            }else{
                out_res = WrRTOS_ERROR;
            }
            xSemaphoreGive(i2c_mutex);
        }else{
            out_res = WrRTOS_TIMEOUT;  // semaphore taking timeout
        }
    }else{
        out_res = WrRTOS_INPUT_ERROR;  // mutex or one of pointers uninitialized
    }
    return out_res;
}

Wrapper_RTOS_status_t WrapperRTOS_UART_Receive_IT_fromISR(void *huart, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    BaseType_t task_woken_take = pdFALSE;
    BaseType_t task_woken_give = pdFALSE;
    if(uart_mutex != NULL)  // check whether mutex is valid
    {
        if (xSemaphoreTakeFromISR(uart_mutex, &task_woken_take))
        {
            status = HAL_UART_Receive_IT(huart, pData, Size);
            xSemaphoreGiveFromISR(uart_mutex, &task_woken_give);
            portYIELD_FROM_ISR(task_woken_give || task_woken_give);
        }
    }
    return status;
}

Wrapper_RTOS_status_t WrapperRTOS_UART_Transmit_DMA(void *huart, const uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    if(uart_mutex != NULL)  // check whether mutex is valid
    {
        if (xSemaphoreTake(uart_mutex, (TickType_t) 100) == pdTRUE)
        {
            status = HAL_UART_Transmit_DMA(huart, pData, Size);
            xSemaphoreGive(uart_mutex);
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

uint32_t WrapperRTOS_read_t_1ms()
{
    return HAL_GetTick();
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

Wrapper_RTOS_status_t WrapperRTOS_ADC_init(void* hadc, void* amp)
{
    taskENTER_CRITICAL();
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
    taskEXIT_CRITICAL();
    vTaskDelay(200);
    HAL_OPAMP_Start(amp);
}

Wrapper_RTOS_status_t WrapperRTOS_ADC_read_blocking(void* hadc, uint16_t* data, uint32_t timeout){
    Wrapper_RTOS_status_t out_res;
    HAL_StatusTypeDef hal_res;

    hal_res = HAL_ADC_Start(hadc);
    if(hal_res == HAL_OK){
        hal_res = HAL_ADC_PollForConversion(hadc, timeout);
        if(hal_res == HAL_OK){
            *data = HAL_ADC_GetValue(hadc);
            out_res = WrRTOS_OK;  // polling successfull
        }else if(hal_res == HAL_TIMEOUT){
            out_res = WrRTOS_TIMEOUT;  // timeout of polling for ADC conversion
        }else{
            out_res = WrRTOS_ERROR;  // polling for conversions returned something different
        }
    }else{
        out_res = WrRTOS_ERROR;  // starting of ADC unsuccessful, HAL_ERROR
    }
    return out_res;
}