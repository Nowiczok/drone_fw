//
// Created by Michał on 17.12.2022.
//

#include "kalman_3d_c.h"
#include "linear_algebra.h"

static void kalman_3d_state_prediction_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part);
static void kalman_3d_cov_prediction_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part);
static void kalman_3d_kalman_gain_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part);
static void kalman_3d_state_update_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part);
static void kalman_3d_state_cov_update_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part);

bool kalman_3D_alg_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part){
    bool res;
    if(comm_part != NULL && var_part != NULL) {
        kalman_3d_state_prediction_c(comm_part, var_part);
        kalman_3d_cov_prediction_c(comm_part, var_part);
        kalman_3d_kalman_gain_c(comm_part, var_part);
        kalman_3d_state_update_c(comm_part, var_part);
        kalman_3d_state_cov_update_c(comm_part, var_part);
        res = true;
    }else
        res = false;
    return res;
}

static void kalman_3d_state_prediction_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part){
    // prediction of state
    float FX[2][1];
    mul((float*)comm_part->F, (float*)var_part->X, false,  // F * X
        (float*)FX, 2, 2, 1);
    var_part->X[0][0] = comm_part->G[0][0] * var_part->a + FX[0][0];  // (F*X) + G * a
    var_part->X[1][0] = comm_part->G[1][0] * var_part->a + FX[1][0];
}

static void kalman_3d_cov_prediction_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part){
    float F_t[2][2];
    tran_const_in((float*)comm_part->F, (float*)F_t, 2, 2);
    // prediction of covariance
    float FP[2][2];
    float FPF_t[2][2];
    mul((float*)comm_part->F, (float*)var_part->P, false,  // F * P, stored in FP
        (float*)FP, 2, 2, 2);
    mul((float*)FP, (float*)F_t, false,  // (F*P) * F_t, stored in FPF_t, FP free
        (float*)FPF_t, 2, 2, 2);
    add((float*)FPF_t, (float*)comm_part->Q, (float*)var_part->P,  //  (F*P*F_t) + Q, FPF_t free
        2, 2, 2);
}

static void kalman_3d_kalman_gain_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part){
    float H_t[2][1];
    float PH_t[2][1];
    float aux;

    tran_const_in((float*)comm_part->H, (float*)H_t, 1, 2);
    mul((float*)var_part->P, (float*)H_t, false,  // P * H_t, stored in PH_t
        (float*)PH_t, 2, 2, 1);
    mul((float*)comm_part->H, (float*)PH_t, false,  // H * (P*H_t), stored in aux
        &aux, 1, 2, 1);
    aux += comm_part->R; // (H*P*H_t) + R, stored in aux
    aux = 1/aux;  // (H*P*H_t+R)^(-1), stored in aux22
    mul((float*)PH_t, &aux, false,  // [P*H_t] * [(H*P*H_t+R)^(-1)], PH_t and aux free
        (float*)var_part->K, 2, 1, 1);
}

static void kalman_3d_state_update_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part){
    float aux;
    float K_times_Z_minus_HX[2][1];
    float aux21[2][1];

    mul((float*)comm_part->H, (float*)var_part->X, false,  // H*X, stored in aux
        &aux, 1, 2, 1);
    aux = var_part->Z - aux;  // Z - (H*X), stored in aux
    mul((float*)var_part->K, &aux, false,  // K * (Z-H*X), stored in K_times_Z_minus_HX, aux free
        (float*)K_times_Z_minus_HX, 2, 1, 1);
    add((float*)K_times_Z_minus_HX, (float*)var_part->X, (float*)aux21, 2, 1, 1);
    memcpy((float*)var_part->X, (float*)aux21, 2 * sizeof(float));  // X + [K*(Z-H*X)], stored in aux21, then copied to X
}

static void kalman_3d_state_cov_update_c(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part){
    // estimate uncertainty update
    float K_t[3][2];
    float aux22[2][2];
    float I_minus_KH[2][2];
    float aux22_3[2][2];
    float aux22_4[2][2];
    float I[2][2] = {{1.0f, 0.0f},
                     {0.0f, 1.0f}};

    tran_const_in((float*)var_part->K, (float*)K_t, 2, 1);
    mul((float*)var_part->K, (float*)K_t, false,  // K * K_t, stored in aux22_3
        (float*)aux22_3, 2, 1, 2);
    aux22_3[0][0] *= comm_part->R;
    aux22_3[0][1] *= comm_part->R;
    aux22_3[1][0] *= comm_part->R;
    aux22_3[1][1] *= comm_part->R;
    mul((float*)var_part->K, (float*)comm_part->H, false,  // K*H, stored in aux22
        (float*)aux22, 2, 1, 2);
    sub((float*)I, (float*)aux22,  // I - (K*H), stored in I_minus_KH, aux22 free
        (float*)I_minus_KH, 2, 2, 2);
    tran_const_in((float*)I_minus_KH, (float*)aux22, 2, 2);  // (I-K*H)^T, stored in aux22
    mul((float*)var_part->P, (float*)aux22, false,  // P * [(I-K*H)^T], stored in aux22_4, aux22 free
        (float*)aux22_4, 2, 2, 2);
    mul((float*)I_minus_KH, (float*)aux22_4, false,  // [I-K*H] * [P*(I-K*H)^T], stored in aux22, I_minus_KH and aux22_4 free
        (float*)aux22, 2, 2, 2);
    add((float*)aux22, (float*)aux22_3,   // [(I-K*H)*P*(I-K*H)^T] + [K*R*K_t]
        (float*)var_part->P, 2, 2, 2);
}
