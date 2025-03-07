/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "decoder.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//volatile short encoderPulse[4] = {0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint8_t CMD[8] = {'0','0','0','0','0','0','0','0',};
	volatile int Vx, Vy, omega;
	volatile bool Speedupdateflag = 0, Pidupdateflag = 0, debug = 0;
	PID_x4 pid;
	WheelSpeeds Targetspeeds, Currentspeeds;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//毫秒延时 //要延时的毫秒数不要超过1500ms
void Delay_ms(uint16_t nms)
{
 uint32_t temp;
 SysTick->LOAD = 9000*nms;
 SysTick->VAL=0X00;//清空计数�?????????????????????????????????????
 SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
 do
 {
  temp=SysTick->CTRL;//读取当前倒计数�??
 }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
    SysTick->CTRL=0x00; //关闭计数�?????????????????????????????????????
    SysTick->VAL =0X00; //清空计数�?????????????????????????????????????
}
void MotorRun(int left_front_MotorPWM, int right_front_MotorPWM,int left_behind_MotorPWM, int right_behind_MotorPWM)
{
    // 左前轮 (FL)
    if (left_front_MotorPWM == 0) {
        Motor1_Brake();
    } else if (left_front_MotorPWM > 0) {
        Motor1_Forward((uint8_t)left_front_MotorPWM);
    } else {
        Motor1_Backward((uint8_t)(-left_front_MotorPWM));
    }

    // 右前轮 (FR)
    if (right_front_MotorPWM == 0) {
        Motor2_Brake();
    } else if (right_front_MotorPWM > 0) {
        Motor2_Forward((uint8_t)right_front_MotorPWM);
    } else {
        Motor2_Backward((uint8_t)(-right_front_MotorPWM));
    }

    // 左后轮 (RL)
    if (left_behind_MotorPWM == 0) {
        Motor3_Brake();
    } else if (left_behind_MotorPWM > 0) {
        Motor3_Forward((uint8_t)left_behind_MotorPWM);
    } else {
        Motor3_Backward((uint8_t)(-left_behind_MotorPWM));
    }

    // 右后轮 (RR)
    if (right_behind_MotorPWM == 0) {
        Motor4_Brake();
    } else if (right_behind_MotorPWM > 0) {
        Motor4_Forward((uint8_t)right_behind_MotorPWM);
    } else {
        Motor4_Backward((uint8_t)(-right_behind_MotorPWM));
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  PID_Init_x4(&pid);
  HAL_UART_Receive_IT(&huart1, CMD, 8);
//  HAL_TIM_Base_Start(&htim4);
//  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_UART_Transmit(&huart1, (uint8_t*)"Reseted", 7, 50);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 //Motor4_Backward(100);
	 // 调用函数计算每个轮子速度
	  //Targetspeeds.wheel_FL=60;Targetspeeds.wheel_FR=60;Targetspeeds.wheel_RL=60;Targetspeeds.wheel_RR=60;
	 if(Speedupdateflag == 1){
		 //-------------目前使用short值的输入数据，需统一-------------------
		 Targetspeeds = calculateMecanumWheelSpeeds((float)Vx, (float)Vy, (float)omega);
	     Speedupdateflag = 0;
	 }
	 if(1){
		 //-------------演示模式-------------------
		 if(__HAL_TIM_GET_COUNTER(&htim7)<=10000)
		 Targetspeeds = calculateMecanumWheelSpeeds(60, 0, 0);
		 else if(__HAL_TIM_GET_COUNTER(&htim7)<=20000)
		 Targetspeeds = calculateMecanumWheelSpeeds(0, 60, 0);
		 else if(__HAL_TIM_GET_COUNTER(&htim7)<=30000)
		 Targetspeeds = calculateMecanumWheelSpeeds(-60, 0, 0);
		 else
		 Targetspeeds = calculateMecanumWheelSpeeds(0, -60, 0);
	 }
	 if (Pidupdateflag == 1) {
	     
	     // 使用新写的 PID 算法，使用目标速度（speeds）和当前速度（编码器测量值）
	     //speeds.wheel_FL=0;
	     PID_Update(&pid.wheel_FL, Targetspeeds.wheel_FL, Currentspeeds.wheel_FL);
	     PID_Update(&pid.wheel_FR, Targetspeeds.wheel_FR, Currentspeeds.wheel_FR);
	     PID_Update(&pid.wheel_RL, Targetspeeds.wheel_RL, Currentspeeds.wheel_RL);
	     PID_Update(&pid.wheel_RR, Targetspeeds.wheel_RR, Currentspeeds.wheel_RR);

	     // 根据计算的输出值驱动电机
	     MotorRun((int)pid.wheel_FL.output, (int)pid.wheel_FR.output, (int)pid.wheel_RL.output, (int)pid.wheel_RR.output);

		 //MotorRun(-100, 0, 80, 100);
	     
	     Pidupdateflag = 0;
	 }
	 if(debug == 1){

         /* ------------------调试用------------------*/
//	        // 利用 sprintf 将结果附加到 info 数组后面
//	     	 char info[100]="Target:";
//	        sprintf(info + strlen(info), " FL: %6.2f", Targetspeeds.wheel_FL);
//	        sprintf(info + strlen(info), " FR: %6.2f", Targetspeeds.wheel_FR);
//	        sprintf(info + strlen(info), " RL: %6.2f", Targetspeeds.wheel_RL);
//	        sprintf(info + strlen(info), " RR: %6.2f\n", Targetspeeds.wheel_RR);
//	        HAL_UART_Transmit(&huart1, (uint8_t*)info, strlen(info), 50);
	     // 调试输出当前脉冲数据
	     char info1[100] = "Pace:";
	     sprintf(info1 + strlen(info1), " FL: %6.2f", Currentspeeds.wheel_FL);
	     sprintf(info1 + strlen(info1), " FR: %6.2f", Currentspeeds.wheel_FR);
	     sprintf(info1 + strlen(info1), " RL: %6.2f", Currentspeeds.wheel_RL);
	     sprintf(info1 + strlen(info1), " RR: %6.2f\n", Currentspeeds.wheel_RR);
	     HAL_UART_Transmit(&huart1, (uint8_t*)info1, strlen(info1), 100);
	     debug = 0;
	 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//可能需要检查是哪一个huart
	if (CMD[0] == 'c' && CMD[7] == 'c') {
	    // 将字符转换为对应的数字值，并计算出最终的 Vx, Vy, omega
	    Vx = (short)((CMD[1] - '0') * 10 + (CMD[2] - '0'));
	    Vy = (short)((CMD[3] - '0') * 10 + (CMD[4] - '0'));
	    omega = (short)((CMD[5] - '0') * 10 + (CMD[6] - '0'));

		Speedupdateflag = 1;
	}
	else if(CMD[0]=='#'&&CMD[1]=='#'&&CMD[2]=='#'){
			   __set_FAULTMASK(1);//禁止所有的可屏蔽中断
			   NVIC_SystemReset();//软件复位
	}
	HAL_UART_Receive_IT(&huart1, CMD, 8);
}
//此处添加了定时器六更新中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&htim6) && Pidupdateflag == 0)
  {
		 Currentspeeds.wheel_FL = CalculatePulse(GetEncoderPulse0());
		 Currentspeeds.wheel_FR = -CalculatePulse(GetEncoderPulse2());
		 Currentspeeds.wheel_RL = CalculatePulse(GetEncoderPulse1());
		 Currentspeeds.wheel_RR = -CalculatePulse(GetEncoderPulse3());
	  Pidupdateflag = 1;
  }
  else if(htim == (&htim7))
  {
	  debug = 1;

  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
