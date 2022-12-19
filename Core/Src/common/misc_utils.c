//
// Created by Michał on 24.11.2022.
//

#include "misc_utils.h"

void update_variance(float* variance, float mean, float new_sample, uint64_t n)
{
    update_mean(variance, powf(new_sample - mean, 2), n-1);
}

void update_mean(float* mean, float new_sample, uint64_t n)
{
    if(mean != NULL)
    {
        if(n != 0)
            *mean = *mean + (new_sample - *mean)/(float)(n);
        else
            *mean += new_sample;
    }
}

bool calculate_vars(sens_fus_var_t *vars, float sample, uint32_t iter_num)
{
    bool res;
    if(!vars->calc_var)
    {
        if(vars->iter_num < iter_num)
        {
            update_mean(&vars->mean, sample, vars->iter_num);
            ++vars->iter_num;
            res = false;
        }else
        {
            vars->calc_var = true;
            vars->iter_num = 0;
            res = false;
        }
    }else
    {
        if(vars->iter_num < iter_num)
        {
            update_variance(&vars->var, vars->mean, sample, vars->iter_num);
            ++vars->iter_num;
            res = false;
        }else
        {
            res = true;
        }
    }
    return res;
}