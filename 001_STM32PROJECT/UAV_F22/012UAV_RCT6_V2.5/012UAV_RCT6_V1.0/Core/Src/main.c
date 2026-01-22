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
  *	作者							XPQH CQX LSL		2025.10.17
  *								UAVF22无人机悬停，在17号首次实现悬停，注意要系绳子，悬停时间4s，可自行设置时间
  *全新改版优化V3.0 作为独立文件，从3.0开始命名		2025.11.8
  *新增: 增量式pid文件
  *优化内容1：  修改了角速度读取的位置，改为了中断里面读取(原在while循坏)
  *优化内容2：  修改角速度pid函数，从原先的普通pid改为了增量pid函数
  *优化内容3：  中断时间从10ms改为1ms(1KHz), 中断计时标志相应修改
  *优化内容4：  对角度pid进行了分频(5分频, 200Hz)， 并且对调角度环和角速度环的上下位置关系
  *
  *小幅优化内容5：pid限幅直接在初始化pid_init时定义
  *小幅优化内容6：移除了安全机制的while循环里面的pwm函数
  *
  *								STM32F103RCT6+MPU6050+DMP+串级PID
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "MPU6050.h"
#include "pid_study.h"
#include "pid_incremental.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 中断PID控制变量
volatile float roll_isr, pitch_isr, yaw_isr;
volatile float gyro_x_isr, gyro_y_isr, gyro_z_isr;
volatile uint8_t sensor_data_ready = 0;  // 传感器数据就绪标志
volatile uint8_t pid_calc_done = 0;      // PID计算完成标志

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

 /*——————————————————————————————————————————————————PID调参入口————————————————————————————————————————————————————————————*/
	//串级PID就是:第一级输出, 当作第二级输入用, 二级响应速度尽可能大于一级即可当串级PID
	//角度为第一级 角速度为第二级
	//可选优化：占空比设置最大5000,将调节精度由原来的1/1000 提高到1/5000

#define PX0_OFFSET 0		//预设0 			初始角度补偿，根据传感器实际误差进行补偿，每一个6050的初始偏移不一样
#define	RY0_OFFSET 0		//预设0

#define GX0_OFFSET -1.4922	//预设+1.4922	同理
#define GY0_OFFSET -0.682	//预设-0.682
#define GZ0_OFFSET -2		//预设0

  //								角度	参数
  //KGain代表K增益，也就是我们调的参数
  PID_KpidGain pid_KGain_roll 	= {.Kp = 3.3, .Ki = 0.10, .Kd = 0.02};
  PID_KpidGain pid_KGain_pitch 	= {.Kp = 3.3, .Ki = 0.10, .Kd = 0.02};
  PID_KpidGain pid_KGain_yaw 	= {.Kp = 0.00, .Ki = 0.00, .Kd = 0.00};
  PID_State pid_state_roll, pid_state_pitch, pid_state_yaw;

  //								角速度参数
  PID_KpidGain_Inc pid_KGain_GyroX 	= {.Kp = 2.1415926, .Ki = 0.122769, .Kd = 0.0269125};
  PID_KpidGain_Inc pid_KGain_GyroY 	= {.Kp = 3.1415926, .Ki = 0.122769, .Kd = 0.0269125};
  PID_KpidGain_Inc pid_KGain_GyroZ 	= {.Kp = 2.5, .Ki = 0.1, .Kd = 0.01};
  PID_State_Inc pid_stateInc_GyroX, pid_stateInc_GyroY, pid_stateInc_GyroZ;

/*——————————————————————————————————————————————————PID调参结束————————————————————————————————————————————————————————————*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(2000);
  int ret = 0;
  do {
	ret = MPU6050_DMP_init();
  } while (ret);

  //初始化PID
  //Y坐标轴 横滚		0目标角度，200是上限
  PID_Init(&pid_state_roll, 0, 200, -200);
  //X坐标轴 俯仰
  PID_Init(&pid_state_pitch, 0, 200, -200);
  //Z坐标轴
  PID_Init(&pid_state_yaw, 0, 200, -200);

  //角速度反馈初始化
  PID_Inc_Init(&pid_stateInc_GyroX, pid_state_pitch.output, 100, -100);
  PID_Inc_Init(&pid_stateInc_GyroY, pid_state_roll.output, 100, -100);
  PID_Inc_Init(&pid_stateInc_GyroZ, pid_state_yaw.output, 100, -100);

  //启动电机PWM 占空比满载999
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);


  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //	横滚	  偏航  俯仰
  float roll, yaw, pitch;
  char message[4096];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //获取角度数据(单位：度)
	  MPU6050_DMP_Get_Date(&roll, &pitch, &yaw);

	    // 更新中断使用的全局变量
	    roll_isr = roll;
	    pitch_isr = pitch;
	    yaw_isr = yaw;

	    sensor_data_ready = 1;  // 设置数据就绪标志

#if 1		//串口调试信息输出到串口，使用Vofa+软件，方便.   		不调试，		令if 0     需要调试if 1
	    sprintf(message, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",gyro_x_isr, gyro_y_isr, gyro_z_isr, pitch, roll, yaw);
	    HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));
#endif
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//在定时器里面确保PID更新时间是严格的1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

    if (htim->Instance == TIM4)  // 确认是PID定时器
    {
        static uint32_t pid_counter = 0;
        static uint32_t safety_timer = 0;      // 安全计时器
        static uint8_t safety_triggered = 0;   // 安全触发标志
        pid_counter++;

        // 安全计时：每次中断增加1ms
          safety_timer++;

          // 检查是否触发安全机制（4秒超时）
          if (safety_timer >= 8000) {  // 4000 * 1ms = 4秒
              safety_triggered = 1;
          }

          // 如果安全机制已触发，立即停止所有电机并进入死循环
          if (safety_triggered) {
              // 立即停止所有电机
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

              // 死循环 - 系统停止
              while(1) {}
          }


        // 检查传感器数据是否就绪
        if (sensor_data_ready && !safety_triggered)
        {
            // 重置标志
            sensor_data_ready = 0;

            /*========== PID计算核心部分 ==========*/

            // 角速度环PID - 内环 2025.11.8改至中断内读取
            //由于角度进行了5分频，此时角速度应该放在角度上方
            MPU6050_Get_Gyro(&gyro_x_isr, &gyro_y_isr, &gyro_z_isr);
            /*MPU6050_Get_Accel(&accel_x_isr, &accel_y_isr, &accel_z_isr);*/

            pid_stateInc_GyroX.target = -pid_state_pitch.output;  // 俯仰角度输出 -> X轴角速度目标
            pid_stateInc_GyroY.target = -pid_state_roll.output;   // 横滚角度输出 -> Y轴角速度目标
            pid_stateInc_GyroZ.target = -pid_state_yaw.output;

            float gyro_x_calib = gyro_x_isr + GX0_OFFSET;
            float gyro_y_calib = gyro_y_isr + GY0_OFFSET;
            float gyro_z_calib = gyro_z_isr + GZ0_OFFSET;

            pid_stateInc_GyroX.actual = gyro_x_calib;
            pid_stateInc_GyroY.actual = gyro_y_calib;
            pid_stateInc_GyroZ.actual = gyro_z_calib;

            pid_stateInc_GyroX = PID_Inc_Iterate(pid_KGain_GyroX, pid_stateInc_GyroX);
            pid_stateInc_GyroY = PID_Inc_Iterate(pid_KGain_GyroY, pid_stateInc_GyroY);
            pid_stateInc_GyroZ = PID_Inc_Iterate(pid_KGain_GyroZ, pid_stateInc_GyroZ);

            // 角度环PID - 外环  对角度pid频率进行5分频，非常容易实现
            static uint32_t pid_angle_count = 0;
            static uint32_t pid_angle_FD = 5;
            pid_angle_count++;
            if((pid_angle_count % pid_angle_FD) == 0){
    			pid_state_roll.target = 1;
    			pid_state_pitch.target = 5;

                float roll_calib = roll_isr + RY0_OFFSET;
    			float pitch_calib = pitch_isr + PX0_OFFSET;

                pid_state_roll.actual = roll_calib;
                pid_state_pitch.actual = pitch_calib;

                pid_state_roll = PID_Iterate(pid_KGain_roll, pid_state_roll);
                pid_state_pitch = PID_Iterate(pid_KGain_pitch, pid_state_pitch);

            }
      	  /*
      	   * 			  Roll(横滚角)				逆时针为数据正
      	   * 					Y^+					陀螺仪逆时针反而是负
      	   * 					|
      	   * 			FL	 	|	 	FR
      	   * 					|
      	   * 	---------Z(垂直于纸面向外)---------------->X Pitch(俯仰角)
      	   * 					|							+
      	   * 					|
      	   * 			BL		|	 	BR
      	   * 					|
      	   * 		PWM控制区 UAVF22四旋翼无人机
      	   * */

            /*========== PWM输出计算 ==========*/
            int PID_GyroX_OUT = (int)pid_stateInc_GyroX.output;
            int PID_GyroY_OUT = (int)pid_stateInc_GyroY.output;
            int	PID_GyroZ_OUT = (int)pid_stateInc_GyroZ.output;

            // 基础PWM值
            uint16_t base_pwm_value = 710;

            //易知，电机最终PWM，就是XYZ三者角速度PID输出的叠加的结果
            uint16_t PWM_FL = base_pwm_value + PID_GyroX_OUT + PID_GyroY_OUT + PID_GyroZ_OUT;
            uint16_t PWM_FR = base_pwm_value + PID_GyroX_OUT - PID_GyroY_OUT - PID_GyroZ_OUT;
            uint16_t PWM_BL = base_pwm_value - PID_GyroX_OUT + PID_GyroY_OUT - PID_GyroZ_OUT;
            uint16_t PWM_BR = base_pwm_value - PID_GyroX_OUT - PID_GyroY_OUT + PID_GyroZ_OUT;

            // PWM限幅
            if(PWM_FL > 1000) PWM_FL = 999;
            if(PWM_FL < 0) PWM_FL = 0;
            if(PWM_FR > 1000) PWM_FR = 999;
            if(PWM_FR < 0) PWM_FR = 0;
            if(PWM_BL > 1000) PWM_BL = 999;
            if(PWM_BL < 0) PWM_BL = 0;
            if(PWM_BR > 1000) PWM_BR = 999;
            if(PWM_BR < 0) PWM_BR = 0;

            // 更新PWM输出
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_FL);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_FR);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_BR);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_BL);

            pid_calc_done = 1;  // 设置计算完成标志


        }

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
#ifdef USE_FULL_ASSERT
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
