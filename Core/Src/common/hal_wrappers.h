//
// Created by Michał on 25.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_HAL_WRAPPERS_H
#define DRONE_CONTROLLER_FW_HAL_WRAPPERS_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>

typedef enum{
    WrRTOS_OK          = 0x00U,
    WrRTOS_ERROR       = 0x01U,
    WrRTOS_TIMEOUT     = 0x03U,
    WrRTOS_INPUT_ERROR = 0x04U,
}Wrapper_RTOS_status_t;

#define US_TIM_PERIOD 0xffff
bool WrapperRTOS_init();
Wrapper_RTOS_status_t WrapperRTOS_i2cMemRead(void *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                             uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
Wrapper_RTOS_status_t WrapperRTOS_i2cMemWrite(void *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                              uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
Wrapper_RTOS_status_t WrapperRTOS_UART_Receive_IT(void *huart, uint8_t *pData, uint16_t Size);
Wrapper_RTOS_status_t WrapperRTOS_UART_Receive_IT_fromISR(void *huart, uint8_t *pData, uint16_t Size);
Wrapper_RTOS_status_t WrapperRTOS_UART_Transmit_IT(void *huart, const uint8_t *pData, uint16_t Size);
Wrapper_RTOS_status_t WrapperRTOS_UART_Transmit_DMA(void *huart, const uint8_t *pData, uint16_t Size);

Wrapper_RTOS_status_t WrapperRTOS_ADC_init(void* hadc, void* amp);
Wrapper_RTOS_status_t WrapperRTOS_ADC_read_blocking(void* hadc, uint16_t* data, uint32_t timeout);

uint32_t WrapperRTOS_read_t_10us();
uint32_t WrapperRTOS_read_t_1ms();
uint32_t calculate_delta_t(uint32_t curr, uint32_t prev);
#endif //DRONE_CONTROLLER_FW_HAL_WRAPPERS_H
