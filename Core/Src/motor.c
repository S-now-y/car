/*
 * motor.c
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */
/*
 注意，在幻尔ROS Controller上，对应关系：
 *插槽-Motor函数
 *M1-Motor3,
 *M2-Motor1,
 *M3-Motor4,
 *M4-Motor3
 */
#include "motor.h"
#include "tim.h"
//初始化
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);

}
void Motor_Stop(void)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);

	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);

	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);

	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,0);
}
//先默认都用快衰减，实现正反滑行转
//void Motor_Decaymode(void)
//{
//
//}

//电机正反转，暂定左前右前左后右后1234
void Motor1_Forward(uint8_t speed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
}
void Motor2_Forward(uint8_t speed)
{
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);

	//幻尔板子需要翻转 F&R
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,speed);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
}
void Motor3_Forward(uint8_t speed)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,speed);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
}
void Motor4_Forward(uint8_t speed)
{
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,speed);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,0);
}

void Motor1_Backward(uint8_t speed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
}
void Motor2_Backward(uint8_t speed)
{
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,speed);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

	//幻尔板子需要翻转 F&R
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
}
void Motor3_Backward(uint8_t speed)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,speed);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
}
void Motor4_Backward(uint8_t speed)
{
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,speed);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
}
void Motor1_Brake(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,MAX_SPEED);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,MAX_SPEED);

}
void Motor2_Brake(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,MAX_SPEED);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,MAX_SPEED);

}
void Motor3_Brake(void)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,MAX_SPEED);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,MAX_SPEED);

}
void Motor4_Brake(void)
{
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MAX_SPEED);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MAX_SPEED);

}



