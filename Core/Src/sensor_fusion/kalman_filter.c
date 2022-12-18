//
// Created by Michał on 26.11.2022.
//

#include "kalman_filter.h"
#include "linear_algebra.h"

bool kalman_3D_no_control_alg_ref(const Kalman_commons_nc_t *comm_part, Kalman_variables_nc_t *var_part)
{
    bool result = false;
    if(comm_part != NULL && var_part != NULL)
    {
        // prediction of state
        float X_int_roll[1][3];
        mul((float*)comm_part->F, (float*)var_part->X, false,
            (float*)X_int_roll, 3, 3, 1);
        memcpy(var_part->X, X_int_roll, 3*sizeof(float));  // put predicted value into struct

        float F_t[3][3];
        tran_const_in((float*)comm_part->F, (float*)F_t, 3, 3);
        // prediction of covariance
        float aux33[3][3];
        float aux33_2[3][3];
        mul((float*)comm_part->F, (float*)var_part->P, false,  // F * X, stored in aux33
            (float*)aux33, 3, 3, 3);
        mul((float*)aux33, (float*)F_t, false,
            (float*)aux33_2, 3, 3, 3);  // (F*X) * F_t, aux33 free
        add((float*)aux33_2, (float*)comm_part->Q, (float*)var_part->P,  //  (F*X*F_t) + Q
            3, 3, 3);

        float H_t[3][2];
        tran_const_in((float*)comm_part->H, (float*)H_t, 2, 3);
        // Kalman gain calculation
        float aux22[2][2];
        float aux22_2[2][2];
        float aux32[3][2];
        mul((float*)var_part->P, (float*)H_t, false,  // P * H_t, stored in aux32
            (float*)aux32, 3,3,2);
        mul((float*)comm_part->H, (float*)aux32, false,  // H * (P*H_t), stored in aux22
            (float*)aux22, 2, 3, 2);
        add((float*)aux22, (float*)comm_part->R, (float*)aux22_2, 2, 2, 2);  // (H*P*H_t) + R, stored in aux22_2, aux22 free
        inv_2x2((float*)aux22_2, (float*)aux22);  // (H*P*H_t+R)^(-1), stored in aux22
        mul((float*)aux32, (float*)aux22, false,  // [P*H_t] * [(H*P*H_t+R)^(-1)], aux32 and aux22_2 free
            (float*)var_part->K, 3, 2, 2);

        // state update
        float aux21[2][1];
        float aux21_2[2][1];
        float aux31[3][1];
        float aux31_2[3][1];
        mul((float*)comm_part->H, (float*)var_part->X, false,  // H*X, stored in aux21
            (float*)aux21, 2, 3, 1);
        sub((float*)var_part->Z, (float*)aux21,  // Z - (H*X), stored in aux21_2, aux21 free
            (float*)aux21_2, 2, 1, 1);
        mul((float*)var_part->K, (float*)aux21_2, false,  // K * (Z-H*X), stored in aux31_2, aux21_2 free
            (float*)aux31_2, 3, 2, 1);
        add((float*)aux31_2, (float*)var_part->X, (float*)aux31, 3, 1, 1);
        memcpy((float*)var_part->X, (float*)aux31, 3*sizeof(float));  // X + [K*(Z-H*X)]

        // estimate uncertainty update
        float K_t[3][2];
        float aux23[2][3];
        float aux33_3[3][3];
        float aux33_4[3][3];
        float I[3][3] = {{1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, 1.0f}};
        tran_const_in((float*)var_part->K, (float*)K_t, 3, 2);
        mul((float*)comm_part->R, (float*)K_t, false,  // R * K_t, stored in aux23
            (float*)aux23, 2, 2, 3);
        mul((float*)var_part->K, (float*)aux23, false,  //K * (R*K_t), stored in aux33, aux23 free
            (float*)aux33, 3, 2, 3);
        mul((float*)var_part->K, (float*)comm_part->H, false,  // K*H, sored in aux33_2
            (float*)aux33_2, 3, 2, 3);
        sub((float*)I, (float*)aux33_2,  // I - (K*H), stored in aux33_3, aux33_2 free
            (float*)aux33_3, 3, 3, 3);
        tran_const_in((float*)aux33_3, (float*)aux33_2, 3, 3);  // (I-K*H)^T, stored in aux33_2
        mul((float*)var_part->P, (float*)aux33_2, false,  // P * [(I-K*H)^T], stored in aux33_4, aux33_2 free
            (float*)aux33_4, 3,3,3);
        mul((float*)aux33_3, (float*)aux33_4, false,  // [I-K*H] * [P*(I-K*H)^T], stored in aux33_2, aux33_3 and aux33_4 free
            (float*)aux33_2, 3, 3, 3);
        add((float*)aux33_2, (float*)aux33,   // [(I-K*H)*P*(I-K*H)^T] + [K*R*K_t]
            (float*)var_part->P, 3, 3, 3);
        result = true;
    }
    return result;
}

bool kalman_3D_control_alg_ref(const Kalman_commons_c_t *comm_part, Kalman_variables_c_t *var_part)
{
    bool result = false;
    if(comm_part != NULL && var_part != NULL)
    {
        // prediction of state
        float aux21[2][1];
        float aux21_2[2][1];
        mul((float*)comm_part->F, (float*)var_part->X, false,  // F * X
            (float*)aux21, 2, 2, 1);
        var_part->X[0][0] = comm_part->G[0][0] * var_part->a + aux21[0][0];  // (F*X) + G * a
        var_part->X[1][0] = comm_part->G[1][0] * var_part->a + aux21[1][0];

        float F_t[2][2];
        tran_const_in((float*)comm_part->F, (float*)F_t, 2, 2);
        // prediction of covariance
        float aux22[2][2];
        float aux22_2[2][2];
        mul((float*)comm_part->F, (float*)var_part->P, false,  // F * P, stored in aux22
            (float*)aux22, 2, 2, 2);
        mul((float*)aux22, (float*)F_t, false,  // (F*P) * F_t, stored in aux22_2, aux22 free
            (float*)aux22_2, 2, 2, 2);
        add((float*)aux22_2, (float*)comm_part->Q, (float*)var_part->P,  //  (F*P*F_t) + Q, aux22_2 free
            2, 2, 2);

        float H_t[2][1];
        tran_const_in((float*)comm_part->H, (float*)H_t, 1, 2);
        // Kalman gain calculation
        float aux;
        //float aux22_2[2][2];
        mul((float*)var_part->P, (float*)H_t, false,  // P * H_t, stored in aux21
            (float*)aux21, 2,2,1);
        mul((float*)comm_part->H, (float*)aux21, false,  // H * (P*H_t), stored in aux
            &aux, 1, 2, 1);
        aux += comm_part->R; // (H*P*H_t) + R, stored in aux
        aux = 1/aux;  // (H*P*H_t+R)^(-1), stored in aux22
        mul((float*)aux21, &aux, false,  // [P*H_t] * [(H*P*H_t+R)^(-1)], aux21 and aux free
            (float*)var_part->K, 2, 1, 1);

        // state update
        mul((float*)comm_part->H, (float*)var_part->X, false,  // H*X, stored in aux
            &aux, 1, 2, 1);
        aux = var_part->Z - aux;  // Z - (H*X), stored in aux
        mul((float*)var_part->K, &aux, false,  // K * (Z-H*X), stored in aux21, aux
            (float*)aux21, 2, 1, 1);
        add((float*)aux21, (float*)var_part->X, (float*)aux21_2, 2, 1, 1);
        memcpy((float*)var_part->X, (float*)aux21_2, 2*sizeof(float));  // X + [K*(Z-H*X)], stored in aux21_2, then copied to X

        // estimate uncertainty update
        float K_t[3][2];
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
        sub((float*)I, (float*)aux22,  // I - (K*H), stored in aux22_2, aux22 free
            (float*)aux22_2, 2, 2, 2);
        tran_const_in((float*)aux22_2, (float*)aux22, 2, 2);  // (I-K*H)^T, stored in aux22
        mul((float*)var_part->P, (float*)aux22, false,  // P * [(I-K*H)^T], stored in aux22_4, aux22 free
            (float*)aux22_4, 2, 2, 2);
        mul((float*)aux22_2, (float*)aux22_4, false,  // [I-K*H] * [P*(I-K*H)^T], stored in aux22, aux22_2 and aux22_4 free
            (float*)aux22, 2, 2, 2);
        add((float*)aux22, (float*)aux22_3,   // [(I-K*H)*P*(I-K*H)^T] + [K*R*K_t]
            (float*)var_part->P, 2, 2, 2);
        result = true;
    }
    return result;
}

