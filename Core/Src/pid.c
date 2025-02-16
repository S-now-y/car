/*
 * pid.c
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */
#include "pid.h"



void PID_Init(PID *p)
{
    p->Kp = Velocity_Kp;
    p->Ki = Velocity_Ki;
    p->Kd = Velocity_Kd;
    p->last_error = 0;
    p->prev_error = 0;
    p->limit = limit_value;
    p->pwm_add = 0;
}
/**
 * @brief  PID相关参数的初始化
 * @param  PID的结构体指针
 */
void PID_Cal(int targetSpeed,int currentSpeed,PID *p)
{
  int error = targetSpeed - currentSpeed; // 得到目标速度与当前速度的误差
  p->pwm_add += p->Kp*(error - p->last_error) + p->Ki*error + p->Kd*(error - 2*p->last_error+p->prev_error);               // 根据增量PID公式计算得到输出的增量

  p->prev_error = p->last_error;          // 记录上次误差
  p->last_error = error;                  // 记录本次误差

  if(p->pwm_add>p->limit) p->pwm_add=p->limit; // 限制最大输出值
  if(p->pwm_add<-p->limit) p->pwm_add=-p->limit;
}
