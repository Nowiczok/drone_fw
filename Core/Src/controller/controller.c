//
// Created by Michał on 29.10.2022.
//

#include "controller.h"

#include "pid.h"
#include "commands.h"
#include "motors.h"
#include "sensor_fusion.h"
#include "hal_wrappers.h"

//private variables
static TaskHandle_t controller_task_handle;
static QueueHandle_t act_data_queue_local;
static QueueHandle_t motors_queue_local;
static QueueHandle_t commands_queue_local;

//private functions
static void controller_task(void* params);

bool controller_init(QueueHandle_t act_data_queue, QueueHandle_t motors_queue, QueueHandle_t commands_queue)
{
    act_data_queue_local = act_data_queue;
    motors_queue_local = motors_queue;
    commands_queue_local = commands_queue;
    xTaskCreate(controller_task,
                "controller_task",
                256,
                NULL,
                3,
                &controller_task_handle);
}

void controller_task(void* params)
{
    uint32_t prev_tim_us = 0;
    uint32_t curr_tim_us = 0;
    float delta_tim_s = 0.0f;
    command_hover_mode_t command;
    fused_data_t act_data_message;
    PidParam_t pid_roll;
    PidParam_t pid_pitch;
    PidParam_t pid_yaw;
    PidParam_t pid_alt;
    motorsMessage_t motors_message;

    pid_set_params(&pid_roll, ROLL_PID_P, ROLL_PID_I, ROLL_PID_D, ROLL_SLEW_RATE, -500.0f, 500.0f);
    pid_set_params(&pid_pitch, PITCH_PID_P, PITCH_PID_I, PITCH_PID_D, PITCH_SLEW_RATE, -500.0f, 500.0f);
    pid_set_params(&pid_yaw, YAW_PID_P, YAW_PID_I, YAW_PID_D, YAW_SLEW_RATE, -500.0f, 500.0f);
    pid_set_params(&pid_alt, ALT_PID_P, ALT_PID_I, ALT_PID_D, ALT_SLEW_RATE, -500.0f, 500.0f);

    uint32_t curr_time_ref = 0;
    uint32_t prev_time_ref = 0;
    float delta_time_ref = 0;

    while(1)
    {
        curr_tim_us = WrapperRTOS_read_t_10us();
        delta_tim_s = (float)(calculate_delta_t(curr_tim_us, prev_tim_us)) * 1.0E-6f;
        prev_tim_us = curr_tim_us;

        curr_time_ref = HAL_GetTick();
        delta_time_ref = (float)(curr_time_ref - prev_time_ref)/1000.0f;
        prev_time_ref = curr_time_ref;
        //!!!!!!!!!!!!!!!!!!!!!!!!!
        delta_tim_s = (float)delta_time_ref;

        xQueueReceive(commands_queue_local, &command, 1);
        xQueueReceive(act_data_queue_local, &act_data_message, 10);


        //motors_message.throttle = pid(&pid_alt, delta_tim_s, act_data_message.alt, command.alt);
        if(!command.timeout)
        {
            motors_message.rollSpeed =  pid(&pid_roll, delta_tim_s, act_data_message.roll, 0);
            motors_message.pitchSpeed =  pid(&pid_pitch, delta_tim_s, act_data_message.pitch, 0);
            motors_message.yawSpeed =  pid(&pid_yaw, delta_tim_s, act_data_message.yaw, 0);
            motors_message.throttle = command.alt;
        }
        else
            motors_message.throttle = 0;
                    xQueueSendToFront(motors_queue_local, &motors_message, 100);
    }
}
