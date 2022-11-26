//
// Created by Michał on 26.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_KALMAN_FILTER_H
#define DRONE_CONTROLLER_FW_KALMAN_FILTER_H

#include <stdbool.h>

// this struct contains elements common for all Kalman filters that are processing angles (roll, pitch, yaw)
typedef struct
{
    float acc_var;
    float F[3][3];  // state-transition model
    float H[2][3];  // observation model
    float Q[3][3];  // covariance of process noise
    float R[2][2];  // covariance of observation noise
} Kalman_no_control_commons_t;

typedef struct
{
    float X[3][1];  // state, needs to be initialized before first usage, after that updated by software
    float Z[2][1];  // measure, manually updated
    float K[3][2];  // Kalman Gain, updated by software
    float P[3][3];  // covariance, needs to be initialized before first usage, after that updated by software
} Kalman_no_control_variables_t;

typedef struct
{
    float acc_var;
    float F[2][2];  // state-transition model
    float G[2][1];  // control matrix
    float H[1][2];  // observation model
    float Q[2][2];  // covariance of process noise
    float R;  // covariance of observation noise
} Kalman_control_commons_t;

typedef struct
{
    float X[2][1];  // state, needs to be initialized before first usage, after that updated by software
    float Z;  // measure, manually updated
    float K[2][1];  // Kalman Gain, updated by software
    float P[2][2];  // covariance, needs to be initialized before first usage, after that updated by software
    float a;
} Kalman_control_variables_t;

bool kalman_3D_no_control_alg(const Kalman_no_control_commons_t *comm_part, Kalman_no_control_variables_t *var_part);
bool kalman_3D_control_alg(const Kalman_control_commons_t *comm_part, Kalman_control_variables_t *var_part);

#endif //DRONE_CONTROLLER_FW_KALMAN_FILTER_H
