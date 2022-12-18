//
// Created by Michał on 26.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_KALMAN_FILTER_H
#define DRONE_CONTROLLER_FW_KALMAN_FILTER_H

#include <stdbool.h>
#include "kalman_3d_c.h"
#include "kalman_3d_nc.h"

bool kalman_3D_no_control_alg_ref(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part);
bool kalman_3D_control_alg_ref(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part);

#endif //DRONE_CONTROLLER_FW_KALMAN_FILTER_H
