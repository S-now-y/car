/*
 * pid.h
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define VELOCITY_KP 1.0f
#define VELOCITY_KI 0.5f
#define VELOCITY_KD 0.1f
#define LIMIT_VALUE 1000

#include "encoder.h"
// 定义 PID 结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float last_error;  // 上一次偏差
    float prev_error;  // 上上次偏差

    int limit;   // 限制输出幅值
    int pwm_add; // 输出的 PWM 值
} PID;

void PID_Init(PID *p);
void PID_Cal(int targetSpeed,int currentSpeed,PID *p);

#endif /* INC_PID_H_ */
