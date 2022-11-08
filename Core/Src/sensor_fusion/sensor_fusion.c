//
// Created by Michał on 05.11.2022.
//

#include "sensor_fusion.h"
#include "imu.h"
#include "barometer.h"
#include "linear_algebra.h"
#include <math.h>
#include <string.h>

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
    float X[1][3];  // state, needs to be initialized before first usage, after that updated by software
    float Z[1][2];  // measure, manually updated
    float K[1][3];  // Kalman Gain, updated by software
    float P[3][3];  // covariance, needs to be initialized before first usage, after that updated by software
} Kalman_angles_variables_t;

QueueHandle_t imu_queue_local;
QueueHandle_t baro_queue_local;
QueueHandle_t magnetometer_queue_local;

// private functions
static void sensor_fusion_task(void* params);
static bool kalman_angles(Kalman_angles_common_t *common_part, Kalman_angles_variables_t *var_part_roll,
                          Kalman_angles_variables_t *var_part_pitch, float delta_t);

static bool matrix_mul(uint32_t mat_1_rows, uint32_t mat_1_cols,
           uint32_t mat_2_rows, uint32_t mat_2_cols,
           float product[mat_1_rows][mat_2_cols],
           float mat_1[mat_1_rows][mat_1_cols],
           float mat_2[mat_2_rows][mat_2_cols]);

static bool matrix_tran(uint32_t mat_rows, uint32_t mat_cols,
        float mat_tran[mat_rows][mat_cols],
        float mat[mat_rows][mat_cols]);

static bool kalman_angles_init(Kalman_angles_common_t *common_part, float angle_std_dev, float vel_std_dev, float acc_std_dev);

bool sensor_fusion_init(QueueHandle_t imu_queue, QueueHandle_t baro_queue, QueueHandle_t magnetometer_queue)
{
    imu_queue_local = imu_queue;
    baro_queue_local = baro_queue;
    magnetometer_queue_local = magnetometer_queue;
    BaseType_t task_creation_res;
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

    while(1)
    {
        xQueueReceive(imu_queue_local, &imu_data, 100);
        xQueueReceive(baro_queue_local, &baro_alt, 100);


    }
}

bool kalman_angles_init(Kalman_angles_common_t *common_part, float angle_std_dev, float vel_std_dev, float acc_std_dev)
{
    bool result = false;
    if(common_part != NULL)
    {
        //initializing constant part of H matrix
        common_part->H[0][0] = 1.0f; common_part->H[0][1] = 0.0f; common_part->H[0][2] = 0.0f;
        common_part->H[1][0] = 0.0f; common_part->H[1][1] = 1.0f; common_part->H[1][2] = 0.0f;

        //initializing constant part of R matrix
        common_part->R[0][0] = powf(angle_std_dev, 2); common_part->R[0][1] = 0.0f;
        common_part->R[1][0] = 0.0f; common_part->R[1][1] = powf(vel_std_dev, 2);

        result = true;
    }
    return result;
}

bool kalman_angles(Kalman_angles_common_t *common_part, Kalman_angles_variables_t *var_part_roll,
                   Kalman_angles_variables_t *var_part_pitch, float delta_t)
{
    bool result = false;
    if(common_part != NULL)
    {
        // prepare F matrix
        common_part->F[0][0] = 1.0f;
        common_part->F[0][1] = delta_t;
        common_part->F[0][2] = 0.5f * powf(delta_t, 2);

        common_part->F[1][0] = 0.0f;
        common_part->F[1][1] = 1.0f;
        common_part->F[1][2] = delta_t;

        common_part->F[2][0] = 0.0f;
        common_part->F[2][1] = 0.0f;
        common_part->F[2][2] = 1.0f;

        // prepare Q matrix
        float acc_std_dev = common_part->acc_std_dev;
        common_part->Q[0][0] = 0.25f * acc_std_dev * powf(delta_t, 4);
        common_part->Q[0][1] = 0.5f * acc_std_dev * powf(delta_t, 3);
        common_part->Q[0][2] = 0.5f * acc_std_dev * powf(delta_t, 2);

        common_part->Q[1][0] = 0.5f * acc_std_dev * powf(delta_t, 3);
        common_part->Q[1][1] = acc_std_dev * powf(delta_t, 2);
        common_part->Q[1][2] = acc_std_dev;

        common_part->Q[2][0] = 0.5f * acc_std_dev * powf(delta_t, 2);
        common_part->Q[2][1] = acc_std_dev;
        common_part->Q[2][2] = acc_std_dev;

        // prediction of state
        float X_int_roll[1][3];
        matrix_mul(3, 3,
                   3, 1,
                   X_int_roll, common_part->F, var_part_roll->X);
        memcpy(var_part_roll->X, X_int_roll, 3*3*sizeof(float));  // put predicted value into struct

        // prediction of covariance
        float aux33[3][3];
        float P_int_roll[3][3];
        matrix_mul(3, 3,
                   3, 3,
                   aux33, common_part->F, var_part_roll->P);

        // Kalman gain calculation


        result = true;
    }
    return result;
}

//TODO: test in debug mode this matrix arithmetics!!!!!!!!!!!!!!!!!!!!!!!!!
bool
matrix_mul(uint32_t mat_1_rows, uint32_t mat_1_cols,
           uint32_t mat_2_rows, uint32_t mat_2_cols,
           float product[mat_1_rows][mat_2_cols],
           float mat_1[mat_1_rows][mat_1_cols],
           float mat_2[mat_2_rows][mat_2_cols])
{
    bool result = false;
    if(mat_1 != NULL && mat_2 != NULL && product != NULL && mat_1_cols != mat_2_rows)
    {
        for(uint32_t i = 0; i < mat_1_rows; i++)
        {
            for(uint32_t j = 0; j < mat_2_cols; j++)
            {
                float int_sum = 0.0f;
                for(uint32_t k = 0; k < mat_2_rows; k++)
                {
                    int_sum += mat_1[i][k] * mat_2[k][j];
                }
                product[i][j] = int_sum;
            }
        }
        result = true;
    }
    return result;
}

static bool matrix_tran(uint32_t mat_rows, uint32_t mat_cols,
                        float mat_tran[mat_cols][mat_rows],
                        float mat[mat_rows][mat_cols])
{
    bool result = false;
    if(mat_tran != NULL && mat != NULL)
    {
        for(uint32_t i = 0; i < mat_cols; i++)
        {
            for(uint32_t j = 0; j < mat_rows; j++)
            {
                mat_tran[i][j] = mat[j][i];
            }
        }
        result = true;
    }
    return result;
}

