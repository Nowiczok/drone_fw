//
// Created by Michał on 03.11.2022.
//

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "barometer.h"
#include "bme68x.h"
#include "hal_wrappers.h"
#include "misc_utils.h"
#include <math.h>

#define REF_ALT          0.0f
#define REF_PRESS        101325.0f
#define GAS_CONSTANT     8.3144598f
#define GRAV_ACC         9.80665f
#define MOLAR_MASS_AIR   0.0289644f

typedef struct{
    void* hi2c;
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

bool barometer_init(QueueHandle_t output_queue, void *hi2c)
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

    struct bme280_dev dev;
    uint8_t settings_sel = 0;
    uint32_t req_delay;
    struct bme280_data comp_data;
    baro_message_t new_message;
    float ref_press = 0.0f;
    BME280_INTF_RET_TYPE bme_init_res;
    BME280_INTF_RET_TYPE bme_settings_res;
    BME280_INTF_RET_TYPE bme_mode_res;
    BME280_INTF_RET_TYPE bme_res;

    // pass HW dependent functions
    dev.intf = BME280_I2C_INTF;
    dev.intf_ptr = &intf;
    dev.write = bme280_i2c_write;
    dev.read = bme280_i2c_read;
    dev.delay_us = bme280_delay_us;
    bme_init_res = bme280_init(&dev);

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;
    bme_settings_res = bme280_set_sensor_settings(settings_sel, &dev);
    bme_mode_res = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

    //REVISIT: when uncommented causes barometer to return same wrong value all the time 
    /*if(bme_init_res != BME280_OK || bme_settings_res != BME280_OK || bme_mode_res != BME280_OK){
        new_message.status = BARO_INIT_ERROR;
        while(1){
            xQueueSendToFront(output_queue_local, &new_message, 100);
        }
    }*/

    vTaskDelay(100);
    bme_res = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    ref_press = (float)comp_data.pressure;
    while(1)
    {
        dev.delay_us(1000, dev.intf_ptr);
        bme_res = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if(bme_res == BME280_OK) {
            float temp_K = (float) comp_data.temperature + 273.0f;
            new_message.alt = calculate_altitude(0.0f, ref_press, (float) comp_data.pressure, temp_K);
            new_message.status = BARO_OK;
        }else{
            new_message.status = BARO_ERROR;
        }
        BaseType_t rslt = xQueueSendToFront(output_queue_local, &new_message, 100);
        vTaskDelay(BARO_TASK_PERIOD_MS);
    }
}

// environment specific functions that bme680 driver needs to communicate with HW
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bme_i2c_params_t* params = (bme_i2c_params_t*)intf_ptr;
    Wrapper_RTOS_status_t res = WrapperRTOS_i2cMemRead(params->hi2c,
                                                   params->dev_addr<<1,
                                                   reg_addr,
                                                   1,
                                                   reg_data,
                                                   len,
                                                   100);
    return (res == WrRTOS_OK) ? BME280_OK : 1;
}

BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bme_i2c_params_t* params = (bme_i2c_params_t*)intf_ptr;
    Wrapper_RTOS_status_t res = WrapperRTOS_i2cMemWrite(params->hi2c,
                                                    params->dev_addr<<1,
                                                    reg_addr,
                                                    1,
                                                    reg_data,
                                                    len,
                                                    100);
    return (res == WrRTOS_OK) ? BME280_OK : 1;
}

void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    float period_ms = ceilf((float)period/1000);
    vTaskDelay( (TickType_t)period_ms);  // this lib doesn't use resolution greater than 1 ms
}

float calculate_altitude(float ref_alt, float ref_press, float press, float temp)
{
    // transformed formula from https://en.wikipedia.org/wiki/Barometric_formula
    return ref_alt - GAS_CONSTANT*temp/(GRAV_ACC*MOLAR_MASS_AIR) * logf(press/ref_press);
}