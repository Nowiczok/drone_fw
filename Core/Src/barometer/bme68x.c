
#include "bme68x.h"
#include "bme68x_defs.h"
#include "hal_wrappers.h"

bool bme68x_init(bme68x_t *sensor, bme68x_init_t *init)
{
    bool result = true;
    if(sensor != NULL && init != NULL)
    {
        uint8_t aux;
        // perform soft reset if necessary
        if(init->perform_soft_reset)
            result &= bme68x_write_regs(sensor, BME68X_REG_SOFT_RESET, BME68X_SOFT_RESET_CMD);

        // read chip ID
        result &= bme68x_read_regs(sensor, BME68X_REG_CHIP_ID, &sensor->chip_id);
        result &= sensor->chip_id == BME68X_CHIP_ID;  // compare id with reference value

        // set humidity oversampling and disable interrupt in 3-wire spi mode
        aux = init->hum_over_sampling << BME68X_OSH_POS | 0 << BME68X_SPI3WINTEN_POS;
        result &= bme68x_write_regs(sensor, BME68X_REG_CTRL_HUM, aux);

        // set forced mode, temperature and pressure oversampling
        aux = init->temp_over_sampling << BME68X_OST_POS |
              init->pres_over_sampling << BME68X_OSP_POS |
              1 << BME68X_MODE_POS;
        result &= bme68x_write_regs(sensor, BME68X_REG_CTRL_MEAS, aux);

        // turn off gas sensor


    }else
        result = false;

    return result;
}

bool bme68x_read_regs(bme68x_t *sensor, uint8_t addr, uint8_t *data)
{
    HAL_StatusTypeDef funct_results;
    funct_results = WrapperRTOS_i2cMemRead(sensor->I2Cx,
                                           BME68X_I2C_ADDR << 1,
                                           addr,
                                           1,
                                           data,
                                           1,
                                           100);
    return funct_results == HAL_OK;
}

bool bme68x_write_regs(bme68x_t *sensor, uint8_t addr, uint8_t data)
{
    HAL_StatusTypeDef funct_results;
    funct_results = WrapperRTOS_i2cMemRead(sensor->I2Cx,
                                           BME68X_I2C_ADDR << 1,
                                           addr,
                                           1,
                                           &data,
                                           1,
                                           100);
    return funct_results == HAL_OK;
}

bool bme68x_read_all(bme68x_t *sensor)
{
    bool result = true;
    if(sensor != NULL)
    {

    }else
        result = false;
    return result;
}

bool bme68x_read_temp(bme68x_t *sensor)
{
    bool result = true;
    if(sensor != NULL)
    {

    }else
        result = false;
    return result;
}

bool bme68x_read_press(bme68x_t *sensor)
{
    bool result = true;
    if(sensor != NULL)
    {

    }else
        result = false;
    return result;
}

bool bme68x_read_hum(bme68x_t *sensor)
{
    bool result = true;
    if(sensor != NULL)
    {

    }else
        result = false;
    return result;
}
