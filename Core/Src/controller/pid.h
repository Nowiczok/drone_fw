//
// Created by Michał on 29.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_PID_H
#define DRONE_CONTROLLER_FW_PID_H
#include "stm32g4xx_hal.h"
#include <stdbool.h>

typedef struct {

    float Kp, Ki, Kd;
    float max_output, min_output;
    float error_sum;
    float last_error;
    float set_point;
    float process_val;
    float prev_output;
    float slew_rate;
} PidParam_t;

float pid(PidParam_t *pid, float delta_t_s, float process_val_in, float set_point_in);
bool pid_set_params(PidParam_t *param, float P, float I, float D, float slew_rate);
bool pid_set_sample_min_out(PidParam_t *param, float minOut);
bool pid_set_sample_max_out(PidParam_t *param, float maxOut);
void pid_reset(PidParam_t *param);

#endif //DRONE_CONTROLLER_FW_PID_H
