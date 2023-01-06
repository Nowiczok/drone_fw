//
// Created by Michał on 02.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_MMC5983_H
#define DRONE_CONTROLLER_FW_MMC5983_H

#include <stdint.h>
#include <stdbool.h>

typedef enum{
    MMC5983_OK,
    MMC5983_TIMEOUT,
    MMC5983_INPUT_ERROR,
    MMC5983_ERROR
}mmc5983_status_t;

typedef struct{
    uint32_t mag_x;
    uint32_t mag_y;
    uint32_t mag_z;
    uint8_t id;
    void* hi2c;
}mmc5983_dev_t;

mmc5983_status_t mmc5983_init(mmc5983_dev_t *dev, void *hi2c);
mmc5983_status_t mmc5983_read_all(mmc5983_dev_t *dev);
mmc5983_status_t mmc5983_read_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data);
mmc5983_status_t mmc5983_read_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count);
mmc5983_status_t mmc5983_write_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t data);
mmc5983_status_t mmc5983_write_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count);

#endif //DRONE_CONTROLLER_FW_MMC5983_H
