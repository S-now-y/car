/*
 * decoder.c
 *
 *  Created on: Feb 17, 2025
 *      Author: fabbo
 */

// decoder.c

#include "decoder.h"
#include <math.h>

// 定义机器人参数，根据实际情况调整
#define ROBOT_LENGTH    1.0f      // 机器人前后轮距参数
#define ROBOT_WIDTH     1.0f      // 机器人左右轮距参数

// 定义轮速最大限幅值
#define WHEEL_SPEED_MAX 100.0f

// 计算麦克纳姆轮速度的函数实现
WheelSpeeds calculateMecanumWheelSpeeds(float Vx, float Vy, float omega) {
    WheelSpeeds speeds;
    float r = ROBOT_LENGTH + ROBOT_WIDTH;
    
    // 基于标准麦克纳姆轮解算公式
    float wheel_FL = Vx + Vy - omega * r;
    float wheel_FR = -Vx + Vy + omega * r;
    float wheel_RL = -Vx + Vy - omega * r;
    float wheel_RR = Vx + Vy + omega * r;
    
    // 找到最大绝对值
    float max_val = fabs(wheel_FL);
    if (fabs(wheel_FR) > max_val) max_val = fabs(wheel_FR);
    if (fabs(wheel_RL) > max_val) max_val = fabs(wheel_RL);
    if (fabs(wheel_RR) > max_val) max_val = fabs(wheel_RR);
    
    // 如果最大值超过 WHEEL_SPEED_MAX，则对所有轮速进行归一化处理
    if (max_val > WHEEL_SPEED_MAX) {
        float scale = WHEEL_SPEED_MAX / max_val;
        wheel_FL *= scale;
        wheel_FR *= scale;
        wheel_RL *= scale;
        wheel_RR *= scale;
    }
    
    // 将计算结果转换为整数
    speeds.wheel_FL = (int)wheel_FL;
    speeds.wheel_FR = (int)wheel_FR;
    speeds.wheel_RL = (int)wheel_RL;
    speeds.wheel_RR = (int)wheel_RR;
    
    return speeds;
}
