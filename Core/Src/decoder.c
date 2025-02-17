/*
 * decoder.c
 *
 *  Created on: Feb 17, 2025
 *      Author: fabbo
 */

// decoder.c

#include "decoder.h"


// 计算麦克纳姆轮速度的函数实现
WheelSpeeds calculateMecanumWheelSpeeds(float Vx, float Vy, float omega) {
    WheelSpeeds speeds;

    // 定义底盘尺寸，可以根据需要调整
    float L = CAR_LENGTH; //chassis_length;
    float W = CAR_WIDTH; //chassis_width;

    // 计算每个轮子的速度
    speeds.wheel_FL = Vx - Vy - omega * (L + W);
    speeds.wheel_FR = Vx + Vy + omega * (L + W);
    speeds.wheel_RL = Vx + Vy - omega * (L + W);
    speeds.wheel_RR = Vx - Vy + omega * (L + W);

    // 如果需要，可以添加速度限制或归一化处理

    //	     // 输出结果
    //	     printf("左前轮速度: %f\n", speeds.wheel_FL);
    //	     printf("右前轮速度: %f\n", speeds.wheel_FR);
    //	     printf("左后轮速度: %f\n", speeds.wheel_RL);
    //	     printf("右后轮速度: %f\n", speeds.wheel_RR);

    return speeds;
}
