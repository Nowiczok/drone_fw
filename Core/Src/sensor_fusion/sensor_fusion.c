//
// Created by Michał on 05.11.2022.
//

#include "sensor_fusion.h"
#include "imu.h"
#include "barometer.h"
#include "linear_algebra.h"
#include "hal_wrappers.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.295779513082320876798154814105f

// this struct contains elements common for all Kalman filters that are processing angles (roll, pitch, yaw)
typedef struct
{
    float acc_std_dev;
    float F[3][3];  // state-transition model
    float H[2][3];  // observation model
    float Q[3][3];  // covariance of process noise
    float R[2][2];  // covariance of observation noise
} Kalman_angles_common_t;

typedef struct
{
    float X[3][1];  // state, needs to be initialized before first usage, after that updated by software
    float Z[2][1];  // measure, manually updated
    float K[3][2];  // Kalman Gain, updated by software
    float P[3][3];  // covariance, needs to be initialized before first usage, after that updated by software
} Kalman_angles_variables_t;

static QueueHandle_t imu_queue_local;
static QueueHandle_t baro_queue_local;
static QueueHandle_t magnetometer_queue_local;
static QueueHandle_t output_queue_local;

Kalman_angles_common_t comm_part_angles;
Kalman_angles_common_t comm_part_alt;
Kalman_angles_variables_t variable_part_roll;
Kalman_angles_variables_t variable_part_pitch;
Kalman_angles_variables_t variable_part_yaw;
Kalman_angles_variables_t variable_part_alt;
fused_data_t output_data;

// private functions
static void sensor_fusion_task(void* params);
static bool kalman_angles(float delta_t);
static bool kalman_3D_alg(const Kalman_angles_common_t *comm_part, Kalman_angles_variables_t *var_part);
static void kalman_alt_init(float alt_std_dev, float vel_std_dev, float acc_std_dev);
static bool kalman_alt(float delta_t);


static void kalman_angles_init(float angle_std_dev, float vel_std_dev, float acc_std_dev);

bool sensor_fusion_init(QueueHandle_t imu_queue, QueueHandle_t baro_queue, QueueHandle_t magnetometer_queue,
                        QueueHandle_t output_queue)
{
    imu_queue_local = imu_queue;
    baro_queue_local = baro_queue;
    magnetometer_queue_local = magnetometer_queue;
    BaseType_t task_creation_res;
    output_queue_local = output_queue;
    task_creation_res = xTaskCreate(sensor_fusion_task,
                                    "sens_fus_task",
                                    128,
                                    NULL,
                                    3,
                                    NULL);
    return task_creation_res == pdPASS;
}

void sensor_fusion_task(void* params)
{
    imuMessage_t imu_data;
    float baro_alt;
    kalman_angles_init(1.0f, 1.0f, 1.0f);
    kalman_alt_init(1.0f, 1.0f, 1.0f);
    uint32_t prev_time = 0;
    uint32_t curr_time = 0;
    float delta_t_s = 0.0f;

    while(1)
    {
        curr_time = WrapperRTOS_read_t_us();
        delta_t_s = (float)calculate_delta_t(curr_time, prev_time)/10e6f;
        xQueueReceive(imu_queue_local, &imu_data, 100);
        xQueueReceive(baro_queue_local, &baro_alt, 100);

        // enter measurements
        variable_part_pitch.Z[0][0] = atan2f(imu_data.acc_x, imu_data.acc_z) * RAD_TO_DEG;
        variable_part_pitch.Z[1][0] = imu_data.gyro_x;  // TODO: check whether it really should be gyro_x
        variable_part_roll.Z[0][0] = atan2f(imu_data.acc_y, imu_data.acc_z);
        variable_part_roll.Z[1][0] = imu_data.gyro_y;  // TODO: check whether it really should be gyro_y
        variable_part_alt.Z[0][0] = baro_alt;
        variable_part_alt.Z[1][0] = sqrtf(powf(imu_data.acc_x, 2) + powf(imu_data.acc_y, 2) + powf(imu_data.acc_z, 2));
        kalman_angles(delta_t_s);
        kalman_alt(delta_t_s);
        output_data.roll = variable_part_roll.X[0][0];
        output_data.pitch = variable_part_pitch.X[0][0];
        output_data.alt = variable_part_alt.X[0][0];

        xQueueSendToFront(output_queue_local, &output_data, 100);
    }
}

void kalman_angles_init(float angle_std_dev, float vel_std_dev, float acc_std_dev)
{
    // initialize constant values in F matrix
    comm_part_angles.F[0][0] = 1.0f;
    comm_part_angles.F[1][0] = 0.0f;
    comm_part_angles.F[1][1] = 1.0f;
    comm_part_angles.F[2][0] = 0.0f;
    comm_part_angles.F[2][1] = 0.0f;
    comm_part_angles.F[2][2] = 1.0f;

    // initializing H matrix
    comm_part_angles.H[0][0] = 1.0f; comm_part_angles.H[0][1] = 0.0f; comm_part_angles.H[0][2] = 0.0f;
    comm_part_angles.H[1][0] = 0.0f; comm_part_angles.H[1][1] = 1.0f; comm_part_angles.H[1][2] = 0.0f;

    // initializing R matrix
    comm_part_angles.R[0][0] = powf(angle_std_dev, 2); comm_part_angles.R[0][1] = 0.0f;
    comm_part_angles.R[1][0] = 0.0f; comm_part_angles.R[1][1] = powf(vel_std_dev, 2);

    // initialize constant values in Q matrix
    comm_part_angles.Q[1][2] = acc_std_dev;
    comm_part_angles.Q[2][1] = acc_std_dev;
    comm_part_angles.Q[2][2] = acc_std_dev;
}

void kalman_alt_init(float alt_std_dev, float vel_std_dev, float acc_std_dev)
{
    // initialize constant values in F matrix
    comm_part_alt.F[0][0] = 1.0f;
    comm_part_alt.F[1][0] = 0.0f;
    comm_part_alt.F[1][1] = 1.0f;
    comm_part_alt.F[2][0] = 0.0f;
    comm_part_alt.F[2][1] = 0.0f;
    comm_part_alt.F[2][2] = 1.0f;

    // initializing H matrix
    comm_part_alt.H[0][0] = 1.0f; comm_part_alt.H[0][1] = 0.0f; comm_part_alt.H[0][2] = 0.0f;
    comm_part_alt.H[1][0] = 0.0f; comm_part_alt.H[1][1] = 0.0f; comm_part_alt.H[1][2] = 1.0f;

    // initializing R matrix
    comm_part_alt.R[0][0] = powf(alt_std_dev, 2); comm_part_alt.R[0][1] = 0.0f;
    comm_part_alt.R[1][0] = 0.0f; comm_part_alt.R[1][1] = powf(vel_std_dev, 2);

    // initialize constant values in Q matrix
    comm_part_alt.Q[1][2] = acc_std_dev;
    comm_part_alt.Q[2][1] = acc_std_dev;
    comm_part_alt.Q[2][2] = acc_std_dev;
}

bool kalman_angles(float delta_t)
{
    // set variable values in F matrix
    comm_part_angles.F[0][1] = delta_t;
    comm_part_angles.F[0][2] = 0.5f * powf(delta_t, 2);
    comm_part_angles.F[1][2] = delta_t;

    // prepare Q matrix
    float acc_std_dev = comm_part_angles.acc_std_dev;
    comm_part_angles.Q[0][0] = 0.25f * acc_std_dev * powf(delta_t, 4);
    comm_part_angles.Q[0][1] = 0.5f * acc_std_dev * powf(delta_t, 3);
    comm_part_angles.Q[0][2] = 0.5f * acc_std_dev * powf(delta_t, 2);

    comm_part_angles.Q[1][0] = 0.5f * acc_std_dev * powf(delta_t, 3);
    comm_part_angles.Q[1][1] = acc_std_dev * powf(delta_t, 2);

    comm_part_angles.Q[2][0] = 0.5f * acc_std_dev * powf(delta_t, 2);

    kalman_3D_alg(&comm_part_angles, &variable_part_roll);
    kalman_3D_alg(&comm_part_angles, &variable_part_pitch);
}

bool kalman_alt(float delta_t)
{
    // set variable values in F matrix
    comm_part_alt.F[0][1] = delta_t;
    comm_part_alt.F[0][2] = 0.5f * powf(delta_t, 2);
    comm_part_alt.F[1][2] = delta_t;

    // prepare Q matrix
    float acc_std_dev = comm_part_alt.acc_std_dev;
    comm_part_alt.Q[0][0] = 0.25f * acc_std_dev * powf(delta_t, 4);
    comm_part_alt.Q[0][1] = 0.5f * acc_std_dev * powf(delta_t, 3);
    comm_part_alt.Q[0][2] = 0.5f * acc_std_dev * powf(delta_t, 2);

    comm_part_alt.Q[1][0] = 0.5f * acc_std_dev * powf(delta_t, 3);
    comm_part_alt.Q[1][1] = acc_std_dev * powf(delta_t, 2);

    comm_part_alt.Q[2][0] = 0.5f * acc_std_dev * powf(delta_t, 2);

    kalman_3D_alg(&comm_part_alt, &variable_part_alt);
}

bool kalman_3D_alg(const Kalman_angles_common_t *comm_part, Kalman_angles_variables_t *var_part)
{
    bool result = false;
    if(comm_part != NULL && var_part != NULL)
    {
        // prediction of state
        float X_int_roll[1][3];
        mul((float*)comm_part->F, (float*)var_part->X, false,
            (float*)X_int_roll, 3, 3, 1);
        memcpy(var_part->X, X_int_roll, 3*3*sizeof(float));  // put predicted value into struct

        float F_t[3][3];
        tran_const_in((float*)comm_part->F, (float*)F_t, 3, 3);
        // prediction of covariance
        float aux33[3][3];
        mul((float*)comm_part->F, (float*)var_part->P, false,  // F * X, stored in aux33
            (float*)aux33, 3, 3, 3);
        mul((float*)aux33, (float*)F_t, false,
            (float*)var_part->P, 3, 3, 3);  // (F*X) * F_t, aux33 free

        float H_t[3][2];
        tran_const_in((float*)comm_part->H, (float*)H_t, 2, 3);
        // Kalman gain calculation
        float aux22[2][2];
        float aux22_2[2][2];
        float aux32[3][2];
        mul((float*)var_part->P, (float*)H_t, false,  // P * H_t, stored in aux33
            (float*)aux32, 3,3,2);
        mul((float*)comm_part->H, (float*)aux33, false,  // H * (P*H_t), stored in aux22
            (float*)aux22, 2, 3, 2);
        add((float*)aux22, (float*)comm_part->R, (float*)aux22_2, 2, 2, 2);  // (H*P*H_t) + R, stored in aux22_2, aux22 free
        inv_2x2((float*)aux22_2, (float*)aux22);  // (H*P*H_t+R)^(-1), stored in aux22
        mul((float*)aux32, (float*)aux22_2, false,  // [P*H_t] * [(H*P*H_t+R)^(-1)], aux32 and aux22_2 free
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
        memcpy((float*)var_part->X, (float*)aux31, 3);  // X + [K*(Z-H*X)]

        // estimate uncertainty update
        float K_t[3][2];
        float aux23[2][3];
        float aux33_2[3][3];
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

