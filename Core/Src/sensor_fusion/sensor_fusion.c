//
// Created by Michał on 05.11.2022.
//

#include "sensor_fusion.h"
#include "imu.h"
#include "barometer.h"
#include "hal_wrappers.h"
#include "misc_utils.h"
#include "kalman_filter.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.295779513082320876798154814105f

static QueueHandle_t imu_queue_local;
static QueueHandle_t baro_queue_local;
static QueueHandle_t magnetometer_queue_local;
static QueueHandle_t output_queue_local;

static Kalman_no_control_commons_t comm_part_angles;
static Kalman_control_commons_t comm_part_alt;
static Kalman_no_control_variables_t variable_part_roll;
static Kalman_no_control_variables_t variable_part_pitch;
static Kalman_no_control_variables_t variable_part_yaw;
static Kalman_control_variables_t variable_part_alt;
static fused_data_t output_data;

// private functions
static void sensor_fusion_task(void* params);
static bool kalman_angles(float delta_t);
static void kalman_alt_init(float alt_var, float vel_var, float acc_var);
static bool kalman_alt(float delta_t);
static void kalman_angles_init(float angle_var, float vel_var, float acc_var);

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
                                    256,
                                    NULL,
                                    3,
                                    NULL);
    return task_creation_res == pdPASS;
}

void sensor_fusion_task(void* params)
{
    imuMessage_t imu_data;
    float baro_alt;
    kalman_angles_init(0.017f, 0.02f, 0.05f);
    kalman_alt_init(0.017f, 0.02f, 0.05f);
    uint32_t prev_time = 0;
    uint32_t curr_time = 0;
    float delta_t_s = 0.0f;

    sens_fus_var_t var_acc = {0};
    sens_fus_var_t no_fus_var_alt = {0};
    sens_fus_var_t fus_var_alt = {0};
    sens_fus_var_t no_fus_var_roll = {0};
    sens_fus_var_t fus_var_roll = {0};

    volatile uint32_t curr_time_ref = 0;
    volatile uint32_t prev_time_ref = 0;
    volatile float delta_time_ref = 0;

    volatile uint32_t start;
    volatile uint32_t stop;
    volatile uint32_t delta;

    while(1)
    {
        BaseType_t queue_rec_rslt = pdTRUE;
        queue_rec_rslt &= xQueueReceive(imu_queue_local, &imu_data, 10);
        queue_rec_rslt &= xQueueReceive(baro_queue_local, &baro_alt, 10);
        if(queue_rec_rslt == pdTRUE) // proceed only if sensors provided data
        {
            curr_time = WrapperRTOS_read_t_10us();
            curr_time_ref = HAL_GetTick();
            delta_t_s = (float)(curr_time - prev_time)/10e5f;
            delta_time_ref = (float)(curr_time_ref - prev_time_ref)/1000.0f;
            prev_time = curr_time;
            prev_time_ref = curr_time_ref;

            //!!!!!!!!!!!!!!!!!!!!!!!!!
            delta_t_s = (float)delta_time_ref;

            float roll = atan2f(imu_data.acc_y, imu_data.acc_z) * RAD_TO_DEG;
            float pitch = atan2f(imu_data.acc_x, imu_data.acc_z) * RAD_TO_DEG;
            // enter measurements
            variable_part_pitch.Z[0][0] = pitch;
            variable_part_pitch.Z[1][0] = imu_data.gyro_y;  // TODO: check whether it really should be gyro_x
            variable_part_roll.Z[0][0] = roll;
            variable_part_roll.Z[1][0] = imu_data.gyro_x;  // TODO: check whether it really should be gyro_y
            variable_part_alt.Z = baro_alt;
            float acc_raw = sqrtf(powf(imu_data.acc_x, 2) + powf(imu_data.acc_y, 2) + powf(imu_data.acc_z, 2));;
            variable_part_alt.a = acc_raw - 1.1342f;
            kalman_angles(delta_t_s);
            kalman_alt(delta_t_s);
            output_data.roll = variable_part_roll.X[0][0];
            output_data.pitch = variable_part_pitch.X[0][0];
            output_data.alt = variable_part_alt.X[0][0];

            calculate_vars(&var_acc, variable_part_alt.a, 3000);
            calculate_vars(&no_fus_var_alt, baro_alt, 3000);
            calculate_vars(&fus_var_alt, output_data.alt, 3000);
            calculate_vars(&no_fus_var_roll, roll, 3000);
            calculate_vars(&fus_var_roll, output_data.roll, 3000);
            //xQueueSendToFront(output_queue_local, &output_data, 100);
        }
    }
}

void kalman_angles_init(float angle_var, float vel_var, float acc_var)
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
    comm_part_angles.R[0][0] = angle_var; comm_part_angles.R[0][1] = 0.0f;
    comm_part_angles.R[1][0] = 0.0f; comm_part_angles.R[1][1] = vel_var;

    // initialize constant values in Q matrix
    comm_part_angles.Q[1][2] = acc_var;
    comm_part_angles.Q[2][1] = acc_var;
    comm_part_angles.Q[2][2] = acc_var;

    comm_part_angles.acc_var = acc_var;

    //roll initial guess
    variable_part_roll.X[0][0] = 0.0f;
    variable_part_roll.X[1][0] = 0.0f;
    variable_part_roll.X[2][0] = 0.0f;

    variable_part_roll.P[0][0] = 5.0f; variable_part_roll.P[0][1] = 0; variable_part_roll.P[0][2] = 0;
    variable_part_roll.P[1][0] = 0; variable_part_roll.P[1][1] = 5.0f; variable_part_roll.P[1][2] = 0;
    variable_part_roll.P[2][0] = 0; variable_part_roll.P[2][1] = 0; variable_part_roll.P[2][2] = 5.0f;

    //pitch initial guess
    variable_part_pitch.X[0][0] = 0.0f;
    variable_part_pitch.X[1][0] = 0.0f;
    variable_part_pitch.X[2][0] = 0.0f;

    variable_part_pitch.P[0][0] = 5.0f; variable_part_pitch.P[0][1] = 0; variable_part_pitch.P[0][2] = 0;
    variable_part_pitch.P[1][0] = 0; variable_part_pitch.P[1][1] = 5.0f; variable_part_pitch.P[1][2] = 0;
    variable_part_pitch.P[2][0] = 0; variable_part_pitch.P[2][1] = 0; variable_part_pitch.P[2][2] = 5.0f;
}

bool kalman_angles(float delta_t)
{
    // set variable values in F matrix
    comm_part_angles.F[0][1] = delta_t;
    comm_part_angles.F[0][2] = 0.5f * powf(delta_t, 2);
    comm_part_angles.F[1][2] = delta_t;

    // prepare Q matrix
    float acc_var = comm_part_angles.acc_var;
    comm_part_angles.Q[0][0] = 0.25f * acc_var * powf(delta_t, 4);
    comm_part_angles.Q[0][1] = 0.5f * acc_var * powf(delta_t, 3);
    comm_part_angles.Q[0][2] = 0.5f * acc_var * powf(delta_t, 2);

    comm_part_angles.Q[1][0] = 0.5f * acc_var * powf(delta_t, 3);
    comm_part_angles.Q[1][1] = acc_var * powf(delta_t, 2);
    comm_part_angles.Q[1][2] = acc_var * delta_t;

    comm_part_angles.Q[2][0] = 0.5f * acc_var * powf(delta_t, 2);
    comm_part_angles.Q[2][1] = delta_t * acc_var;
    comm_part_angles.Q[2][2] = acc_var;

    kalman_3D_no_control_alg(&comm_part_angles, &variable_part_roll);
    kalman_3D_no_control_alg(&comm_part_angles, &variable_part_pitch);
}

void kalman_alt_init(float alt_var, float vel_var, float acc_var)
{
    // initialize constant values in F matrix
    comm_part_alt.F[0][0] = 1.0f;
    comm_part_alt.F[1][0] = 0.0f;
    comm_part_alt.F[1][1] = 1.0f;

    // initializing H matrix
    comm_part_alt.H[0][0] = 1.0f; comm_part_alt.H[0][1] = 0.0f;

    // initializing R
    comm_part_alt.R = alt_var;
    comm_part_alt.acc_var = acc_var;

    // initial guess
    variable_part_alt.X[0][0] = 0.0f;
    variable_part_alt.X[1][0] = 0.0f;

    variable_part_alt.P[0][0] = 5.0f;  variable_part_alt.P[0][1] = 0.0f;
    variable_part_alt.P[1][0] = 0.0f;  variable_part_alt.P[1][1] = 5.0f;
}

bool kalman_alt(float delta_t)
{
    // set variable values in F matrix
    comm_part_alt.F[0][1] = delta_t;

    // set G matrix
    comm_part_alt.G[0][0] = 0.5f * powf(delta_t, 2);
    comm_part_alt.G[1][0] = delta_t;

    // prepare Q matrix
    float acc_std_dev = comm_part_alt.acc_var;
    comm_part_alt.Q[0][0] = 0.25f * acc_std_dev * powf(delta_t, 4); comm_part_alt.Q[0][1] = 0.5f * acc_std_dev * powf(delta_t, 3);
    comm_part_alt.Q[1][0] = 0.5f * acc_std_dev * powf(delta_t, 3);  comm_part_alt.Q[1][1] = acc_std_dev * powf(delta_t, 2);

    kalman_3D_control_alg(&comm_part_alt, &variable_part_alt);
}

