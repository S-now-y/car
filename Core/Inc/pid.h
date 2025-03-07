/*
 * pid.h
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */

#ifndef INC_PID_H
#define INC_PID_H

// PID tuning constants defined as macros.
#define PID_KP      3.21f
#define PID_KI      0.0044f
#define PID_KD      0.01f

// Output limits
#define PID_MAX_OUTPUT   100
#define PID_MIN_OUTPUT  -100

#include "encoder.h"

// 定义 PID 结构体
typedef struct {
    float last_error;
    float integral;
    float output;
} PID;

typedef struct {
    PID wheel_FL;
    PID wheel_FR;
    PID wheel_RL;
    PID wheel_RR;
} PID_x4;

/**
 * @brief  重置/初始化单一路 PID 控制器
 * @param  pid: 指向 PID 结构体的指针
 */
void PID_Init(PID *pid);

/**
 * @brief  同时初始化四路 PID 控制器
 * @param  pid: 指向 PID_x4 结构体的指针
 */
void PID_Init_x4(PID_x4 *pid);

/**
 * @brief  使用新测量值更新 PID 控制器
 * @param  pid: 指向 PID 结构体的指针
 * @param  setpoint: 目标值
 * @param  measured: 实际测量值
 */
void PID_Update(PID *pid, float setpoint, float measured);

#endif // INC_PID_H
