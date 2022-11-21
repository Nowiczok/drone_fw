//
// Created by Michał on 20.11.2022.
//
#include "MMC5983.h"
#include "hal_wrappers.h"
#include "MMC5983_defines.h"

static bool mmc5983_read_id(mmc5983_dev_t *dev);

bool mmc5983_init(mmc5983_dev_t *dev, I2C_HandleTypeDef *hi2c)
{
    bool result = false;
    if(dev != NULL && hi2c != NULL)
    {
        dev->hi2c = hi2c;
        mmc5983_read_id(dev);

        uint8_t internal_control[3] = {0};  // fourth register (internal control 3) is not used
        internal_control[0] |= AUTO_SR_EN;  // automatic set/reset enable
        internal_control[1] |= BW0 | BW1;  // bandwidth 800 Hz
        internal_control[2] |= CM_FREQ_0 | CM_FREQ_1 | CM_FREQ_2 |  // 1 kHz sampling rate
                               CMM_EN |  // continuous conversion enabled
                               PRD_SET_0 | PRD_SET_1 |  // set every 100 measurements
                               EN_PRD_SET;  // enable periodic set
        result = mmc5983_write_mul_regs(dev, INT_CTRL_0_REG, internal_control, 3);
    }
    return result;
}

bool mmc5983_read_id(mmc5983_dev_t *dev)
{
    bool result = false;
    if(dev != NULL)
    {
        result = mmc5983_read_one_reg(dev, PROD_ID_REG, &dev->id);
    }
    return result;
}

bool mmc5983_read_all(mmc5983_dev_t *dev)
{
    bool result = false;
    if(dev != NULL)
    {
        uint8_t output_regs[7];
        result = mmc5983_read_mul_regs(dev, X_OUT_0_REG, output_regs, 7);
        dev->mag_x = output_regs[0];
        dev->mag_x = (dev->mag_x<<8) | output_regs[1];
        dev->mag_x = (dev->mag_x<<2) | output_regs[6]>>6;

        dev->mag_y = output_regs[0];
        dev->mag_y = (dev->mag_y<<8) | output_regs[1];
        dev->mag_y = (dev->mag_y<<2) | (output_regs[6]>>4 & 0x03);

        dev->mag_z = output_regs[0];
        dev->mag_z = (dev->mag_z<<8) | output_regs[1];
        dev->mag_z = (dev->mag_z<<2) | (output_regs[6]>>2 & 0x03);
    }
    return result;
}

bool mmc5983_read_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data)
{
    return mmc5983_read_mul_regs(dev, addr, data, 1);
}

bool mmc5983_read_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count)
{
    bool result = false;
    if(dev != NULL)
    {
        HAL_StatusTypeDef hal_rslt;
        hal_rslt = WrapperRTOS_i2cMemRead(dev->hi2c,
                                          I2C_ADDR<<1,
                                          addr,
                                          1,
                                          data,
                                          count,
                                          100);
        result = hal_rslt == HAL_OK;
    }
    return result;
}

bool mmc5983_write_one_reg(mmc5983_dev_t *dev, uint8_t addr, uint8_t data)
{
    return mmc5983_write_mul_regs(dev, addr, &data, 1);
}

bool mmc5983_write_mul_regs(mmc5983_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count)
{
    bool result = false;
    if(dev != NULL)
    {
        HAL_StatusTypeDef hal_rslt;
        hal_rslt = WrapperRTOS_i2cMemWrite(dev->hi2c,
                                           I2C_ADDR<<1,
                                           addr,
                                           1,
                                           data,
                                           count,
                                           100);
        result = hal_rslt == HAL_OK;
    }
    return result;
}
