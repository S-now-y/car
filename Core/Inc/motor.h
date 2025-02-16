/*
 * motor.h
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal_conf.h"//基于HAL库!!!!!!!!!!!!conf我暂时不知道删不删，删了有warning！

//宏定义（暂时未完善）之后添加
#define MAX_SPEED 100
#define MOTOR_TIM_HANDLE htim1

//比较寄存器关键设置法 __HAL_TIM_SET_cOMPARE(句柄，通道，值)

////句柄
//#define MOTOR_TIM_CHANNEL1 TIM_CHANNEL_1
//#define MOTOR_TIM_CHANNEL2 TIM_CHANNEL_2
//#define MOTOR_TIM_CHANNEL3 TIM_CHANNEL_3
//#define MOTOR_TIM_CHANNEL3 TIM_CHANNEL_4
//快慢衰减枚举
typedef enum{
	SLOW_DECAY,
	FAST_DECAY
}DecayMode;//暂时默认快衰减

////电机正反转，暂时不用。
//typedef enum{
//	FORWARD=0,
//	BACKWARD,
//	BRAKE
//
//}Direction;


//函数声明

void Motor_Init(void);
void Motor_Stop(void);

void Motor1_Forward(uint8_t speed);
void Motor2_Forward(uint8_t speed);
void Motor3_Forward(uint8_t speed);
void Motor4_Forward(uint8_t speed);
void Motor1_Backward(uint8_t speed);
void Motor2_Backward(uint8_t speed);
void Motor3_Backward(uint8_t speed);
void Motor4_Backward(uint8_t speed);

void Motor1_Brake(void);
void Motor2_Brake(void);
void Motor3_Brake(void);
void Motor4_Brake(void);





#endif /* INC_MOTOR_H_ */
