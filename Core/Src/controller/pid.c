//
// Created by Michał on 29.10.2022.
//

#include "pid.h"

#define ABS_FLOAT(x) (x>=0? x:(x)*(-1))
#define SIGN(x) (x<0 ? (-1) : 1)

void pid(PidParam_t *param) {
    float out, error;

    error = param->SetPoint - param->ProcessVal;

    if(param->Ki>0){
        param->errorSum += error;
    }
    else
        param->errorSum =0;


    out = error * param->Kp + param->errorSum * param->Ki * param->sampleTime
          + (error - param->lastError) * param->frequency * param->Kd;



    if (param->Ki > 0) {
        if (out > param->maxOutput) {
            param->errorSum -= (out - param->maxOutput)
                               / (param->sampleTime * param->Ki);
            out = param->maxOutput;
        } else if (out < param->minOutput) {
            param->errorSum -= (out - param->minOutput)
                               / (param->sampleTime * param->Ki);
            out = param->minOutput;
        }
    }
    else if (ABS_FLOAT(out) > param->maxOutput){
        out = param->maxOutput * SIGN(out);
    }



    param->lastError = error;

    float lastOutput;
    lastOutput = param->output;
    if(out - lastOutput >25)
        out = lastOutput + 25;

    if(out - lastOutput < -25)
        out = lastOutput - 25;


    param->output = out;

}

void pidInt(PidParamInt_t *param) {

    int32_t out, error;

    error = param->SetPoint - param->ProcessVal;
    param->errorSum += error;

    out = (error * (int32_t)param->Kp) / (int32_t)param->KpDiv
          + (param->errorSum * (int32_t)param->Ki) / (int32_t)(param->KiDivXfreq);

    if (ABS_FLOAT(out) > param->maxOutput) {
        param->errorSum -= (((int64_t)out - (SIGN(out) * param->maxOutput))
                            * (param->KiDivXfreq)) / param->Ki;
        out = param->maxOutput * SIGN(out);
    }

    param->lastError = error;
    param->output = out;

}

bool pidSetGain(PidParam_t *param, float P, float I, float D){

    if(P < 0 || I<0 || D<0)
        return false;

    param->Kp = P;
    param->Ki = I;
    param->Kd = D;

    return true;
}

void pidSetGainP(PidParam_t *param, float P){
    param->Kp = P;
}

bool pidIntSetGain(PidParamInt_t *param, uint32_t P, uint32_t Pdiv, uint32_t I, uint32_t Idiv, uint32_t D, uint32_t Ddiv){

    if(Pdiv==0 || Idiv==0 || Ddiv==0)
        return false;

    param->Kp = P;
    param->KpDiv = Pdiv;

    param->Ki = I;
    param->KiDiv = Idiv;
    param->KiDivXfreq = param->KiDiv* param->frequency;

    param->Kd = D;
    param->KdDiv = Ddiv;

    return true;

}

bool pidSetFreq(PidParam_t *param, float freq){

    if(freq<=0)
        return false;

    param->frequency = freq;
    param->sampleTime = 1/freq;


    return true;

}

bool pidIntSetFreq(PidParamInt_t *param, uint16_t freq) {

    param->frequency = freq;
    param->KiDivXfreq = param->KiDiv* param->frequency;

    return true;

}


bool pidSetSampleTime(PidParam_t *param, float sampleTime){

    if(sampleTime<=0)
        return false;

    param->frequency = 1 / sampleTime;
    param->sampleTime = sampleTime;


    return true;

}

bool pidSetSampleMaxOut(PidParam_t *param, float maxOut){

    if(maxOut < 0)
        return false;

    param->maxOutput = maxOut;

    return true;

}

bool pidSetSampleMinOut(PidParam_t *param, float minOut){

//	if(minOut < 0)
//		return false;

    param->minOutput = minOut;

    return true;

}

bool pidIntSetSampleMaxOut(PidParamInt_t *param, uint32_t maxOut){



    param->maxOutput = maxOut;

    return true;

}
void pidReset(PidParam_t *param){

    param->errorSum = 0;
    param->output = 0;
    param->lastError = 0;

}

void pidIntReset(PidParamInt_t *param){

    param->errorSum = 0;
    param->output = 0;
    param->lastError = 0;

}