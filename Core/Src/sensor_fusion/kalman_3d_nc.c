//
// Created by Michał on 17.12.2022.
//

#include "kalman_3d_nc.h"
#include "kalman_filter.h"
#include "linear_algebra.h"

static void kalman_3d_state_prediction_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part);
static void kalman_3d_cov_prediction_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part);
static void kalman_3d_kalman_gain_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part);
static void kalman_3d_state_update_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part);
static void kalman_3d_state_cov_update_c(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part);

bool kalman_3D_alg_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part)
{
    kalman_3d_state_prediction_nc(comm_part, var_part);
    kalman_3d_cov_prediction_nc(comm_part, var_part);
    kalman_3d_kalman_gain_nc(comm_part, var_part);
    kalman_3d_state_update_nc(comm_part, var_part);
    kalman_3d_state_cov_update_c(comm_part, var_part);
}

// implementation of state prediction equation for algorithm without control matrix
void kalman_3d_state_prediction_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part){
    float X_pred[1][3];

    mul((float*)comm_part->F, (float*)var_part->X, false,
        (float*)X_pred, 3, 3, 1);
    memcpy(var_part->X, X_pred, 3 * sizeof(float));  // put predicted value into struct
}

void kalman_3d_cov_prediction_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part){
    float F_t[3][3];
    float FP[3][3];
    float FPF_t[3][3];

    tran_const_in((float*)comm_part->F, (float*)F_t, 3, 3);  // prepare transposed F matrix
    mul((float*)comm_part->F, (float*)var_part->P, false,  // F * P, stored in FP
        (float*)FP, 3, 3, 3);
    mul((float*)FP, (float*)F_t, false,
        (float*)FPF_t, 3, 3, 3);  // (F*P) * F_t, FP free
    add((float*)FPF_t, (float*)comm_part->Q, (float*)var_part->P,  //  (F*P*F_t) + Q
        3, 3, 3);
}

void kalman_3d_kalman_gain_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part){
    float H_t[3][2];
    float aux22[2][2];  // auxiliary matrix, used to store more than one intermediate value
    float HPH_t_plus_R[2][2];
    float PH_T[3][2];

    tran_const_in((float*)comm_part->H, (float*)H_t, 2, 3);
    mul((float*)var_part->P, (float*)H_t, false,  // P * H_t, stored in PH_T
        (float*)PH_T, 3, 3, 2);
    mul((float*)comm_part->H, (float*)PH_T, false,  // H * (P*H_t), stored in aux22
        (float*)aux22, 2, 3, 2);
    add((float*)aux22, (float*)comm_part->R, (float*)HPH_t_plus_R, 2, 2, 2);  // (H*P*H_t) + R, stored in HPH_t_plus_R, aux22 free
    inv_2x2((float*)HPH_t_plus_R, (float*)aux22);  // (H*P*H_t+R)^(-1), stored in aux22, HPH_t_plus_R free
    mul((float*)PH_T, (float*)aux22, false,  // [P*H_t] * [(H*P*H_t+R)^(-1)], PH_T and aux22 free
        (float*)var_part->K, 3, 2, 2);
}

void kalman_3d_state_update_nc(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part){
    // state update
    float HX[2][1];
    float Z_minus_HX[2][1];
    float X_updated[3][1];
    float K_times_Z_minus_HX[3][1];

    mul((float*)comm_part->H, (float*)var_part->X, false,  // H*X, stored in HX
        (float*)HX, 2, 3, 1);
    sub((float*)var_part->Z, (float*)HX,  // Z - (H*X), stored in Z_minus_HX, HX free
        (float*)Z_minus_HX, 2, 1, 1);
    mul((float*)var_part->K, (float*)Z_minus_HX, false,  // K * (Z-H*X), stored in K_times_Z_minus_HX, Z_minus_HX free
        (float*)K_times_Z_minus_HX, 3, 2, 1);
    add((float*)K_times_Z_minus_HX, (float*)var_part->X, (float*)X_updated, 3, 1, 1);
    memcpy((float*)var_part->X, (float*)X_updated, 3 * sizeof(float));  // X + [K*(Z-H*X)]
}

void kalman_3d_state_cov_update_c(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part){
    // estimate uncertainty update
    float K_t[3][2];
    float RK_T[2][3];
    float KRK_T[3][3];
    float aux33_2[3][3];
    float I_minus_KH[3][3];
    float aux33_4[3][3];
    float I[3][3] = {{1.0f, 0.0f, 0.0f},
                     {0.0f, 1.0f, 0.0f},
                     {0.0f, 0.0f, 1.0f}};

    tran_const_in((float*)var_part->K, (float*)K_t, 3, 2);
    mul((float*)comm_part->R, (float*)K_t, false,  // R * K_t, stored in RK_T
        (float*)RK_T, 2, 2, 3);
    mul((float*)var_part->K, (float*)RK_T, false,  //K * (R*K_t), stored in KRK_T, RK_T free
        (float*)KRK_T, 3, 2, 3);
    mul((float*)var_part->K, (float*)comm_part->H, false,  // K*H, sored in aux33_2
        (float*)aux33_2, 3, 2, 3);
    sub((float*)I, (float*)aux33_2,  // I - (K*H), stored in I_minus_KH, aux33_2 free
        (float*)I_minus_KH, 3, 3, 3);
    tran_const_in((float*)I_minus_KH, (float*)aux33_2, 3, 3);  // (I-K*H)^T, stored in aux33_2
    mul((float*)var_part->P, (float*)aux33_2, false,  // P * [(I-K*H)^T], stored in aux33_4, aux33_2 free
        (float*)aux33_4, 3,3,3);
    mul((float*)I_minus_KH, (float*)aux33_4, false,  // [I-K*H] * [P*(I-K*H)^T], stored in aux33_2, I_minus_KH and aux33_4 free
        (float*)aux33_2, 3, 3, 3);
    add((float*)aux33_2, (float*)KRK_T,   // [(I-K*H)*P*(I-K*H)^T] + [K*R*K_t]
        (float*)var_part->P, 3, 3, 3);
}