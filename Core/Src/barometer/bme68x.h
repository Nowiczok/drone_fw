#ifndef BME68X_H_
#define BME68X_H_

#include <stdbool.h>
#include "stm32g4xx_hal.h"

typedef struct{
    uint8_t chip_id;
    I2C_HandleTypeDef *I2Cx;
    float temp;
    float press;
    float hum;
    float gas;
} bme68x_t;

typedef struct{
    bool perform_soft_reset;
    uint8_t temp_over_sampling;
    uint8_t hum_over_sampling;
    uint8_t pres_over_sampling;
    uint8_t filter;
} bme68x_init_t;

bool bme68x_init(bme68x_t *sensor, bme68x_init_t *init);

bool bme68x_read_regs(bme68x_t *sensor, uint8_t addr, uint8_t *data);

bool bme68x_write_regs(bme68x_t *sensor, uint8_t addr, uint8_t data);
bool bme68x_read_all(bme68x_t *sensor);
bool bme68x_read_temp(bme68x_t *sensor);
bool bme68x_read_press(bme68x_t *sensor);
bool bme68x_read_hum(bme68x_t *sensor);
bool bme68x_read_altitude(bme68x_t *sensor);

#endif /* BME68X_H_ */
