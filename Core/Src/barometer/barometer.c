//
// Created by Michał on 03.11.2022.
//

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "barometer.h"
#include "bme68x.h"
#include "hal_wrappers.h"
#include "i2c.h"
#include <math.h>

#define REF_ALT          0.0f
#define REF_PRESS        101325.0f
#define GAS_CONSTANT     8.3144598f
#define GRAV_ACC         9.80665f
#define MOLAR_MASS_AIR   0.0289644f

typedef struct{
    I2C_HandleTypeDef* hi2c;
    uint8_t dev_addr;
}bme_i2c_params_t;

QueueHandle_t output_queue_local;
bme_i2c_params_t intf = {.hi2c = NULL, .dev_addr = BME280_I2C_ADDR_PRIM};

// private functions
static void barometer_task(void* params);
static BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void bme280_delay_us(uint32_t period, void *intf_ptr);
float calculate_altitude(float ref_alt, float ref_press, float press, float temp);

bool barometer_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c)
{
    BaseType_t result;
    output_queue_local = output_queue;
    intf.hi2c = hi2c;
    result = xTaskCreate(barometer_task,
                         "baro_task",
                         256,
                         NULL,
                         3,
                         NULL);
    return result == pdPASS;
}

void barometer_task(void* params)
{
    volatile int8_t rslt = BME280_OK;
    struct bme280_dev dev;
    uint8_t settings_sel = 0;
    uint32_t req_delay;
    struct bme280_data comp_data;
    float alt = 0.0f;
    float ref_press = 0.0f;
    float ref_temp = 0.0f;

    // pass HW dependent functions
    dev.intf = BME280_I2C_INTF;
    dev.intf_ptr = &intf;
    dev.write = bme280_i2c_write;
    dev.read = bme280_i2c_read;
    dev.delay_us = bme280_delay_us;
    rslt = bme280_init(&dev);

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    /* Set the sensor settings */
    rslt = bme280_set_sensor_settings(settings_sel, &dev);

    req_delay = bme280_cal_meas_delay(&dev.settings);

    while(1)
    {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
        dev.delay_us(req_delay, dev.intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        float temp_K = (float)comp_data.temperature + 273.0f;
        alt = calculate_altitude(REF_ALT, REF_PRESS, (float) comp_data.pressure, temp_K);
        xQueueSendToFront(output_queue_local, &alt, 100);
    }
}

// environment specific functions that bme680 driver needs to communicate with HW
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bme_i2c_params_t* params = (bme_i2c_params_t*)intf_ptr;
    HAL_StatusTypeDef res = WrapperRTOS_i2cMemRead(params->hi2c,
                                                   params->dev_addr<<1,
                                                   reg_addr,
                                                   1,
                                                   reg_data,
                                                   len,
                                                   100);
    return (res == HAL_OK) ? BME280_OK : 1;
}

BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bme_i2c_params_t* params = (bme_i2c_params_t*)intf_ptr;
    HAL_StatusTypeDef res = WrapperRTOS_i2cMemWrite(params->hi2c,
                                                    params->dev_addr<<1,
                                                    reg_addr,
                                                    1,
                                                    reg_data,
                                                    len,
                                                    100);
    return (res == HAL_OK) ? BME280_OK : 1;
}

void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    vTaskDelay(period/1000);  // this lib doesn't use resolution greater than 1 ms
}

float calculate_altitude(float ref_alt, float ref_press, float press, float temp)
{
    // transformed formula from https://en.wikipedia.org/wiki/Barometric_formula
    return ref_alt - GAS_CONSTANT*temp/(GRAV_ACC*MOLAR_MASS_AIR) * logf(press/ref_press);
}