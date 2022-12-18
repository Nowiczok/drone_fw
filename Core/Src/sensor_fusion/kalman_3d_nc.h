//
// Created by Michał on 17.12.2022.
//

#ifndef DRONE_CONTROLLER_FW_KALMAN_3D_NC_H
#define DRONE_CONTROLLER_FW_KALMAN_3D_NC_H

#include <stdbool.h>

// this struct contains elements common for all Kalman filters that are processing angles (roll, pitch, yaw)
typedef struct
{
    float acc_var;
    float F[3][3];  // state-transition model
    float H[2][3];  // observation model
    float Q[3][3];  // covariance of process noise
    float R[2][2];  // covariance of observation noise
} Kalman_commons_nc_t;

typedef struct
{
    float X[3][1];  // state, needs to be initialized before first usage, after that updated by software
    float Z[2][1];  // measure, manually updated
    float K[3][2];  // Kalman Gain, updated by software
    float P[3][3];  // covariance, needs to be initialized before first usage, after that updated by software
} Kalman_variables_nc_t;

bool kalman_3D_alg_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part);

#endif //DRONE_CONTROLLER_FW_KALMAN_3D_NC_H

