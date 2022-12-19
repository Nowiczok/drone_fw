//
// Created by Michał on 24.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_MISC_UTILS_H
#define DRONE_CONTROLLER_FW_MISC_UTILS_H

#include "misc_utils.h"
#include "stdint.h"
#include <math.h>
#include <stdbool.h>

typedef struct {
    float mean;
    float var;
    uint32_t iter_num;
    bool calc_var;
} sens_fus_var_t;

void update_variance(float* variance, float mean, float new_sample, uint64_t n);
void update_mean(float* mean, float new_sample, uint64_t n);
bool calculate_vars(sens_fus_var_t *vars, float sample, uint32_t iter_num);

#endif //DRONE_CONTROLLER_FW_MISC_UTILS_H
