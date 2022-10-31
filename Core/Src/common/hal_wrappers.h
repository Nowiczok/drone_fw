//
// Created by Michał on 25.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_HAL_WRAPPERS_H
#define DRONE_CONTROLLER_FW_HAL_WRAPPERS_H

#include "stm32g4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"



HAL_StatusTypeDef WrapperRTOS_i2cMemRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                        uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef WrapperRTOS_i2cMemWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                          uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint32_t WrapperRTOS_read_t_us(TIM_HandleTypeDef *htim);
#endif //DRONE_CONTROLLER_FW_HAL_WRAPPERS_H