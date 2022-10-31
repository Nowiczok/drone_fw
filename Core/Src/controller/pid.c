//
// Created by Michał on 29.10.2022.
//

#include "pid.h"

#define ABS_FLOAT(x) (x>=0? x:(x)*(-1))
#define SIGN(x) (x<0 ? (-1) : 1)

float pid(PidParam_t *pid, float delta_t_s, float process_val_in, float set_point_in)
{
    float out, error;

    error = set_point_in - process_val_in;

    // integrate error
    if(pid->Ki > 0)
    {
        pid->error_sum += error;
    }
    else
        pid->error_sum =0;

    out = error * pid->Kp + pid->error_sum * pid->Ki * delta_t_s
          + (error - pid->last_error) / delta_t_s * pid->Kd;

    if (pid->Ki > 0)
    {
        if (out > pid->max_output)
        {
            pid->error_sum -= (out - pid->max_output) / (delta_t_s * pid->Ki);
            out = pid->max_output;
        } else if (out < pid->min_output)
        {
            pid->error_sum -= (out - pid->min_output) / (delta_t_s * pid->Ki);
            out = pid->min_output;
        }
    }
    else if (ABS_FLOAT(out) > pid->max_output)
    {
        out = pid->max_output * SIGN(out);
    }

    pid->last_error = error;

    float lastOutput;
    lastOutput = pid->prev_output;
    if(out - lastOutput > pid->slew_rate)
        out = lastOutput + pid->slew_rate;

    if(out - lastOutput < -pid->slew_rate)
        out = lastOutput - pid->slew_rate;

    pid->prev_output = out;
    return out;
}

bool pid_set_params(PidParam_t *param, float P, float I, float D, float slew_rate)
{
    if(P<0 || I<0 || D<0)
        return false;

    param->Kp = P;
    param->Ki = I;
    param->Kd = D;

    return true;
}

bool pid_set_sample_max_out(PidParam_t *param, float maxOut)
{
    if(maxOut < 0)
        return false;

    param->max_output = maxOut;

    return true;
}

bool pid_set_sample_min_out(PidParam_t *param, float minOut)
{
//	if(minOut < 0)
//		return false;

    param->min_output = minOut;
    return true;
}

void pid_reset(PidParam_t *param)
{
    param->error_sum = 0;
    param->prev_output = 0;
    param->last_error = 0;
}
