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
#define GAS_CONSTANT     8.3144598f
#define GRAV_ACC         9.80665f
#define MOLAR_MASS_AIR   0.0289644f

typedef struct{
    I2C_HandleTypeDef* hi2c;
    uint8_t dev_addr;
}bme_i2c_params_t;

QueueHandle_t output_queue_local;
bme_i2c_params_t intf = {.hi2c = NULL, .dev_addr = BME68X_I2C_ADDR_LOW};

// private functions
static void barometer_task(void* params);
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void bme68x_delay_us(uint32_t period, void *intf_ptr);
float calculate_altitude(float ref_alt, float ref_press, float ref_temp, float press, float temp);

bool barometer_init(QueueHandle_t output_queue, I2C_HandleTypeDef *hi2c)
{
    BaseType_t result;
    output_queue_local = output_queue;
    intf.hi2c = hi2c;
    result = xTaskCreate(barometer_task,
                         "baro_task",
                         128,
                         NULL,
                         3,
                         NULL);
    return result == pdPASS;
}

void barometer_task(void* params)
{
    struct bme68x_dev bme;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data;
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 1;
    float alt;
    float ref_press;
    float ref_temp;

    // interface initialization
    bme.read = bme68x_i2c_read;
    bme.write = bme68x_i2c_write;
    bme.delay_us = bme68x_delay_us;
    bme.intf_ptr = &intf;
    bme.intf = BME68X_I2C_INTF;
    bme.amb_temp = 25;
    bme68x_init(&bme);

    // temp, press, hum configuration
    conf.filter = BME68X_FILTER_SIZE_127;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_16X;
    bme68x_set_conf(&conf, &bme);

    // gas sensor configuration
    heatr_conf.enable = BME68X_DISABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);

    // get reference values of pressure and temperature (altitude related to place of start)
    bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
    bme.delay_us(del_period, bme.intf_ptr);
    bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    ref_press = data.pressure;
    ref_temp = data.temperature;

    while(1)
    {
        bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);
        bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        alt = calculate_altitude(REF_ALT, ref_press, ref_temp, data.pressure, data.temperature);
        xQueueSendToFront(output_queue_local, &alt, 100);
    }
}

// environment specific functions that bme680 driver needs to communicate with HW
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bme_i2c_params_t* params = (bme_i2c_params_t*)intf_ptr;
    HAL_StatusTypeDef res = WrapperRTOS_i2cMemRead(params->hi2c,
                                                   params->dev_addr,
                                                   reg_addr,
                                                   1,
                                                   reg_data,
                                                   len,
                                                   100);
    return (res == HAL_OK) ? BME68X_INTF_RET_SUCCESS : 1;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bme_i2c_params_t* params = (bme_i2c_params_t*)intf_ptr;
    HAL_StatusTypeDef res = WrapperRTOS_i2cMemWrite(params->hi2c,
                                                    params->dev_addr,
                                                    reg_addr,
                                                    1,
                                                    reg_data,
                                                    len,
                                                    100);
    return (res == HAL_OK) ? BME68X_INTF_RET_SUCCESS : 1;
}

void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    vTaskDelay(period/1000);  // this lib doesn't use resolution greater than 1 ms
}

float calculate_altitude(float ref_alt, float ref_press, float ref_temp, float press, float temp)
{
    // transformed formula from https://en.wikipedia.org/wiki/Barometric_formula
    return ref_alt - GAS_CONSTANT*ref_temp/(GRAV_ACC*MOLAR_MASS_AIR) * logf(press/ref_press);
}