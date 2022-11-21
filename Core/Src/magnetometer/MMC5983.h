//
// Created by Michał on 02.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_MMC5983_H
#define DRONE_CONTROLLER_FW_MMC5983_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

typedef struct{
    uint32_t mag_x;
    uint32_t mag_y;
    uint32_t mag_z;
    uint8_t id;
    I2C_HandleTypeDef* hi2c;
}mmc5983_dev_t;

bool mmc5983_init(mmc5983_dev_t *dev, I2C_HandleTypeDef *hi2c);
bool mmc5983_read_all(mmc5983_dev_t *dev);
bool mmc5983_read_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data);
bool mmc5983_read_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count);
bool mmc5983_write_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t data);
bool mmc5983_write_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count);

#endif //DRONE_CONTROLLER_FW_MMC5983_H
