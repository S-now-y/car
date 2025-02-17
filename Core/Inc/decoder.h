/*
 * decoder.h
 *
 *  Created on: Feb 17, 2025
 *      Author: fabbo
 */

#ifndef INC_DECODER_H_
#define INC_DECODER_H_

#define CAR_WIDTH 1.0f
#define CAR_LENGTH 0.5f

// 定义轮子速度结构体
typedef struct {
    float wheel_FL; // 左前轮速度
    float wheel_FR; // 右前轮速度
    float wheel_RL; // 左后轮速度
    float wheel_RR; // 右后轮速度
} WheelSpeeds;

// 计算麦克纳姆轮速度的函数声明
WheelSpeeds calculateMecanumWheelSpeeds(float Vx, float Vy, float omega);

#endif /* INC_DECODER_H_ */
