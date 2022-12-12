//
// Created by Michał on 27.10.2022.
//
#include "motors.h"

#define PWM_DUTY_OFFSET 500

static xQueueHandle input_queue_local;
static xQueueHandle output_queue_local;
TIM_HandleTypeDef* pwm_timer_handle_ptr;

void motors_task(void* parameters);
void control_motors(motorsMessage_t input_message);
static float line_approximation(float x, const float *reference_x, const float *reference_y, uint8_t num_of_ref_pairs);
static float pwm_torque_translation(float pwm, const float *reference_pwms, const float *reference_forces, uint8_t ref_num);

// 1------------2
//       |
//       |
//       |
// 3------------4

void motors_init(QueueHandle_t input_queue, QueueHandle_t output_queue, TIM_HandleTypeDef *htim)
{
    pwm_timer_handle_ptr = htim;
    input_queue_local = input_queue;
    output_queue_local = output_queue;
    xTaskCreate(motors_task,
                "motors_task",
                128,
                NULL,
                3,
                NULL);
}

void motors_task(void* parameters)
{
    motorsMessage_t input_message;

    input_message.throttle = 0;
    input_message.yawSpeed = 0;
    input_message.pitchSpeed = 0;
    input_message.rollSpeed = 0;

    //init timer that generates PWM
    HAL_TIM_Base_Start(pwm_timer_handle_ptr);
    HAL_TIM_PWM_Start(pwm_timer_handle_ptr, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(pwm_timer_handle_ptr, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(pwm_timer_handle_ptr, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(pwm_timer_handle_ptr, TIM_CHANNEL_4);

    vTaskDelay(5000);

    while(1)
    {
        xQueueReceive(input_queue_local, &input_message,100);
        control_motors(input_message);
        vTaskDelay(10);
    }
}

void control_motors(motorsMessage_t input_message)
{
    float pwm_vals[MOTOR_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};

    //apply throttle, all motors should spin the same
    pwm_vals[0] += input_message.throttle;
    pwm_vals[1] += input_message.throttle;
    pwm_vals[2] += input_message.throttle;
    pwm_vals[3] += input_message.throttle;

    //apply roll
    pwm_vals[0] += input_message.rollSpeed;
    pwm_vals[1] -= input_message.rollSpeed;
    pwm_vals[2] += input_message.rollSpeed;
    pwm_vals[3] -= input_message.rollSpeed;

    //apply pitch
    pwm_vals[0] -= input_message.pitchSpeed;
    pwm_vals[1] -= input_message.pitchSpeed;
    pwm_vals[2] += input_message.pitchSpeed;
    pwm_vals[3] += input_message.pitchSpeed;

    //apply yaw
    //pwm_vals[0] -= input_message.yawSpeed;
    //pwm_vals[1] += input_message.yawSpeed;
    //pwm_vals[2] += input_message.yawSpeed;
    //pwm_vals[3] -= input_message.yawSpeed;


    __HAL_TIM_SET_COMPARE(pwm_timer_handle_ptr, TIM_CHANNEL_1, pwm_vals[0] + PWM_DUTY_OFFSET + 191);  // 201
    __HAL_TIM_SET_COMPARE(pwm_timer_handle_ptr, TIM_CHANNEL_2, pwm_vals[1] + PWM_DUTY_OFFSET + 150);  // 168
    __HAL_TIM_SET_COMPARE(pwm_timer_handle_ptr, TIM_CHANNEL_3, pwm_vals[2] + PWM_DUTY_OFFSET + 156);  // 166
    __HAL_TIM_SET_COMPARE(pwm_timer_handle_ptr, TIM_CHANNEL_4, pwm_vals[3] + PWM_DUTY_OFFSET + 161);  //171
}

static float line_approximation(float x, const float *reference_x, const float *reference_y, uint8_t num_of_ref_pairs)
{
    float force = 0.0f;
    for(uint8_t i = 0; i < num_of_ref_pairs - 1; i++)
    {
        if(x > reference_x[i] || i == num_of_ref_pairs - 2)
        {
            float a = (reference_y[i + 1] - reference_y[i]) / (reference_x[i + 1] - reference_x[i]);
            float b = reference_y[i] - a * reference_x[i];
            force = a * x + b;  // approximation with linear function
            break;
        }
    }
    return force;
}

