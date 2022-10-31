//
// Created by Michał on 29.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_PID_H
#define DRONE_CONTROLLER_FW_PID_H
#include "stm32g4xx_hal.h"
#include <stdbool.h>

typedef struct {

    float Kp, Ki, Kd;
    float sampleTime;
    float frequency;

    float maxOutput, minOutput;

    float errorSum;
    float lastError;
    float SetPoint,ProcessVal;
    float output;

} PidParam_t;

typedef struct {

    uint32_t Kp, Ki, Kd;
    uint32_t KpDiv, KiDiv, KdDiv;
    uint32_t KiDivXfreq;
    uint16_t frequency;
    uint32_t maxOutput;
    int64_t errorSum;
    int32_t lastError;
    int32_t SetPoint, ProcessVal;
    int32_t output;

} PidParamInt_t;


void pid(PidParam_t *param);
void pidInt(PidParamInt_t *param);
bool pidSetGain(PidParam_t *param, float P, float I, float D);
void pidSetGainP(PidParam_t *param, float P);
bool pidIntSetGain(PidParamInt_t *param, uint32_t P, uint32_t Pdiv, uint32_t I, uint32_t Idiv, uint32_t D, uint32_t Ddiv);
bool pidSetFreq(PidParam_t *param, float freq);
bool pidIntSetFreq(PidParamInt_t *param, uint16_t freq);
bool pidSetSampleTime(PidParam_t *param, float sampleTime);
bool pidSetSampleMinOut(PidParam_t *param, float minOut);
bool pidSetSampleMaxOut(PidParam_t *param, float maxOut);
bool pidIntSetSampleMaxOut(PidParamInt_t *param, uint32_t maxOut);
void pidReset(PidParam_t *param);
void pidIntReset(PidParamInt_t *param);

#endif //DRONE_CONTROLLER_FW_PID_H
