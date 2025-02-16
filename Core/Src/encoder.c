/*
 * encoder.c
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */

#include "encoder.h"
#include "tim.h"

void GetEncoderPulse(void)
{
	 encoderPulse[0] = -((short)__HAL_TIM_GET_COUNTER(&htim2));
	  encoderPulse[1] = -((short)__HAL_TIM_GET_COUNTER(&htim3));
	  encoderPulse[2] = ((short)__HAL_TIM_GET_COUNTER(&htim4));
	  encoderPulse[3] = ((short)__HAL_TIM_GET_COUNTER(&htim5));

	    __HAL_TIM_GET_COUNTER(&htim2) = 0;   //计数值重新清零
	    __HAL_TIM_GET_COUNTER(&htim3) = 0;
	    __HAL_TIM_GET_COUNTER(&htim4) = 0;   //计数值重新清零
	    __HAL_TIM_GET_COUNTER(&htim5) = 0;
}

//速度计算，里面的值该速度为每个麦克纳姆轮的转速，并非小车实际的运行速度，车轮周长*规定时间内得到的脉冲数/(电机转动一圈的脉冲数*定时器规定时间 )
float CalculatePulse(int pulse)
{
	return (float)(0.00 * pulse);
}
