//
// Created by Michał on 20.11.2022.
//
#include "MMC5983.h"
#include "hal_wrappers.h"
#include "MMC5983_defines.h"

static mmc5983_status_t mmc5983_read_id(mmc5983_dev_t *dev);

mmc5983_status_t mmc5983_init(mmc5983_dev_t *dev, void *hi2c)
{
    mmc5983_status_t result;
    if(dev != NULL && hi2c != NULL)
    {
        dev->hi2c = hi2c;
        result = mmc5983_read_id(dev);
        if(result == MMC5983_OK) {  // proceed only if ID was read properly
            uint8_t internal_control[3] = {0};  // fourth register (internal control 3) is not used
            internal_control[0] |= AUTO_SR_EN;  // automatic set/reset enable
            internal_control[1] |= BW0 | BW1;  // bandwidth 800 Hz
            internal_control[2] |= CM_FREQ_0 | CM_FREQ_1 | CM_FREQ_2 |  // 1 kHz sampling rate
                                   CMM_EN |  // continuous conversion enabled
                                   PRD_SET_0 | PRD_SET_1 |  // set every 100 measurements
                                   EN_PRD_SET;  // enable periodic set
            result = mmc5983_write_mul_regs(dev, INT_CTRL_0_REG, internal_control, 3);
        }
    }else{
        result = MMC5983_INPUT_ERROR;
    }
    return result;
}

mmc5983_status_t mmc5983_read_id(mmc5983_dev_t *dev)
{
    mmc5983_status_t result;
    if(dev != NULL){
        result = mmc5983_read_one_reg(dev, PROD_ID_REG, &dev->id);
    }else{
        result = MMC5983_INPUT_ERROR;
    }
    return result;
}

mmc5983_status_t mmc5983_read_all(mmc5983_dev_t *dev)
{
    mmc5983_status_t result;
    if(dev != NULL)
    {
        uint8_t output_regs[7];
        result = mmc5983_read_mul_regs(dev, X_OUT_0_REG, output_regs, 7);
        if(result == MMC5983_OK) {
            dev->mag_x = output_regs[0];
            dev->mag_x = (dev->mag_x << 8) | output_regs[1];
            dev->mag_x = (dev->mag_x << 2) | output_regs[6] >> 6;

            dev->mag_y = output_regs[0];
            dev->mag_y = (dev->mag_y << 8) | output_regs[1];
            dev->mag_y = (dev->mag_y << 2) | (output_regs[6] >> 4 & 0x03);

            dev->mag_z = output_regs[0];
            dev->mag_z = (dev->mag_z << 8) | output_regs[1];
            dev->mag_z = (dev->mag_z << 2) | (output_regs[6] >> 2 & 0x03);
        }
    }else{
        result = MMC5983_INPUT_ERROR;
    }
    return result;
}

mmc5983_status_t mmc5983_read_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data)
{
    return mmc5983_read_mul_regs(dev, addr, data, 1);
}

mmc5983_status_t mmc5983_read_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count)
{
    mmc5983_status_t result;
    if(dev != NULL && data != NULL)
    {
        Wrapper_RTOS_status_t hal_rslt;
        hal_rslt = WrapperRTOS_i2cMemRead(dev->hi2c, I2C_ADDR<<1,addr,
                                          1, data, count, 100);
        if(hal_rslt == WrRTOS_OK){
            result = MMC5983_OK;
        }else if(hal_rslt == WrRTOS_TIMEOUT){
            result = MMC5983_TIMEOUT;
        }else{
            result = MMC5983_ERROR;
        }
    }else{
        result = MMC5983_INPUT_ERROR;
    }
    return result;
}

mmc5983_status_t mmc5983_write_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t data)
{
    return mmc5983_write_mul_regs(dev, addr, &data, 1);
}

mmc5983_status_t mmc5983_write_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count)
{
    mmc5983_status_t result;
    if(dev != NULL && data != NULL)
    {
        Wrapper_RTOS_status_t hal_rslt;
        hal_rslt = WrapperRTOS_i2cMemWrite(dev->hi2c, I2C_ADDR<<1, addr,
                                           1, data, count, 100);
        if(hal_rslt == WrRTOS_OK){
            result = MMC5983_OK;
        }else if(hal_rslt == WrRTOS_TIMEOUT){
            result = MMC5983_TIMEOUT;
        }else{
            result = MMC5983_ERROR;
        }
    }else{
        result = MMC5983_INPUT_ERROR;
    }
    return result;
}
