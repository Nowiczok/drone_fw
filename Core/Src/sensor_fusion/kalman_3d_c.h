//
// Created by Michał on 17.12.2022.
//

#ifndef DRONE_CONTROLLER_FW_KALMAN_3D_C_H
#define DRONE_CONTROLLER_FW_KALMAN_3D_C_H

#include <stdbool.h>
typedef struct
{
    float acc_var;
    float F[2][2];  // state-transition model
    float G[2][1];  // control matrix
    float H[1][2];  // observation model
    float Q[2][2];  // covariance of process noise
    float R;  // covariance of observation noise
} Kalman_commons_c_t;

typedef struct
{
    float X[2][1];  // state, needs to be initialized before first usage, after that updated by software
    float Z;  // measure, manually updated
    float K[2][1];  // Kalman Gain, updated by software
    float P[2][2];  // covariance, needs to be initialized before first usage, after that updated by software
    float a;
} Kalman_variables_c_t;

bool kalman_3D_alg_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part);

#endif //DRONE_CONTROLLER_FW_KALMAN_3D_C_H
