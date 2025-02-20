/*
 * pid.c
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */
#include "pid.h"

void PID_Init(PID *pid) {
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void PID_Init_x4(PID_x4 *pid) {
    PID_Init(&pid->wheel_FL);
    PID_Init(&pid->wheel_FR);
    PID_Init(&pid->wheel_RL);
    PID_Init(&pid->wheel_RR);
}

void PID_Update(PID *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    
    // Proportional term
    float P_out = PID_KP * error;
    
    // Integral term
    pid->integral += error;
    float I_out = PID_KI * pid->integral;
    
    // Derivative term
    float derivative = error - pid->last_error;
    float D_out = PID_KD * derivative;
    
    // PID output before clamping
    pid->output = P_out + I_out + D_out;
    
    // 记录当前误差，用于下次计算Derivative term
    pid->last_error = error;
    
    // Clamp output to max/min limits
    if (pid->output > PID_MAX_OUTPUT)
        pid->output = PID_MAX_OUTPUT;
    else if (pid->output < PID_MIN_OUTPUT)
        pid->output = PID_MIN_OUTPUT;
}

