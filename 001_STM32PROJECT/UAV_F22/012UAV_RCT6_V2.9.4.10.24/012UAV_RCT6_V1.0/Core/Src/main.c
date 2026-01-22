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
  *
  *
  * 							XPQH CQX LSL 2025.10.20
  *
  *对无用部分进行了删除和精简					 2025.11.3
  *可以看出，截止11.3日，基本功能已经完成，但是程序并不完美，存在一些特殊问题，比如在第二次启动等待期间，电机会似动非动
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 中断PID控制变量(volatile可以防止被编译器优化，全局变量确保在中断和while循环都可用)
volatile float roll_isr, pitch_isr, yaw_isr;
volatile float gyro_x_isr, gyro_y_isr, gyro_z_isr;
volatile uint8_t sensor_data_ready = 0;  // 传感器数据就绪标志
volatile uint8_t pid_calc_done = 0;      // PID计算完成标志

//按键控制第二次起飞启动标志
volatile uint8_t Key1 = 0;

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

#define PX0_OFFSET 0		//预设0 			初始角度补偿，根据实际情况预设，每一个6050的初始偏差都不一样
#define	RY0_OFFSET 0		//预设0

#define GX0_OFFSET 1.4922	//预设+1.4922	同理
#define GY0_OFFSET -0.985	//预设-0.682
#define GZ0_OFFSET -1.4		//预设0

  //								角度	参数
  //KGain代表K增益，也就是我们调的参数
  PID_KpidGain pid_KGain_roll 	= {.Kp = 3.3, .Ki = 0.10, .Kd = 0.02};
  PID_KpidGain pid_KGain_pitch 	= {.Kp = 3.3, .Ki = 0.10, .Kd = 0.02};
  PID_KpidGain pid_KGain_yaw 	= {.Kp = 0, .Ki = 0, .Kd = 0};
  PID_State pid_state_roll, pid_state_pitch, pid_state_yaw;

  //								角速度参数
  PID_KpidGain pid_KGain_GyroY 	= {.Kp = 2.3415926, .Ki = 0.222769, .Kd = 0.0669125};
  PID_KpidGain pid_KGain_GyroX 	= {.Kp = 2.1415926, .Ki = 0.222769, .Kd = 0.0669125};
  PID_KpidGain pid_KGain_GyroZ 	= {.Kp = 2.5, .Ki = 0.1, .Kd = 0.01};
  PID_State pid_state_GyroX, pid_state_GyroY, pid_state_GyroZ;

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
  //Y坐标轴 横滚
  PID_Init(&pid_state_roll, 0);
  //X坐标轴 俯仰
  PID_Init(&pid_state_pitch, 0);
  //Z坐标轴
  PID_Init(&pid_state_yaw, 0);

  //角速度反馈初始化
  PID_Init(&pid_state_GyroX, pid_state_pitch.output);
  PID_Init(&pid_state_GyroY, pid_state_roll.output);
  PID_Init(&pid_state_GyroZ, pid_state_yaw.output);

  //启动电机PWM 占空比满载999
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //指示灯闪烁 代表6050启动成功
  for(int i = 0; i<3; i++){
	  //指示灯未安装
	  HAL_Delay(1000);

  }

  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //	横滚	  偏航  俯仰
  float roll, yaw, pitch;
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //获取角度数据(单位：度)
	  MPU6050_DMP_Get_Date(&roll, &pitch, &yaw);

	  // 获取角速度数据(单位：dps)  2025.10.9 基本成功
	  MPU6050_Get_Gyro(&gyro_x, &gyro_y, &gyro_z);
	  MPU6050_Get_Accel(&accel_x, &accel_y, &accel_z);

	    // 更新中断使用的全局变量,令传感器数据传入中断
	    roll_isr = roll;
	    pitch_isr = pitch;
	    yaw_isr = yaw;
	    gyro_x_isr = gyro_x;
	    gyro_y_isr = gyro_y;
	    gyro_z_isr = gyro_z;
	    sensor_data_ready = 1;  // 设置数据就绪标志

	    //串口调试信息输出,可以使用VOFA+软件查看数据,注意波特率115200 如果不使用，if 0屏蔽掉即可,如果使用if 1打开
#if 0
	    sprintf(message, "%.4f,%.4f\n",gyro_x,gyro_y);
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

//在定时器里面确保PID更新时间是严格的10ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

    if (htim->Instance == TIM4)  // 确认是PID定时器
    {
        static uint32_t pid_counter = 0;
        static uint32_t safety_timer = 0;      // 安全计时器
        static uint8_t safety_triggered = 0;   // 安全触发标志
        pid_counter++;

        // 安全计时：每次中断增加10ms
          safety_timer++;

          // 检查是否触发安全机制（4秒超时）
          if (safety_timer >= 3000) {  // 400 * 10ms = 4秒
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
              while(1) {
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

              }
          }

          static uint16_t Flight_timer = 0;      	// 飞行时间计数
          Flight_timer++;							//	每次中断增加1,等价于时间加10ms
          static uint16_t FLight_base_pwm = 0;		// 飞行基础PWM值
          static uint8_t Key1_Strat = 0;
          static uint8_t Up2_Start = 0;	//飞向大圆开始标志
        // 检查传感器数据是否就绪
        if (sensor_data_ready && !safety_triggered) {

            // 重置标志
            sensor_data_ready = 0;
/*----------------------------------------飞行逻辑运行段-----------------------------------------------------*/
            //其实不难从角度看出，要控制无人机运动也就控制角度偏的多少，偏多久就可以了，主要还是精度不好控制
            //水平提高之后尽量不用简单if判断	2025.10.20
            //XY由角度控制，Z由角速度控制	注意区分
//            FLight_base_pwm代表每一个状态使用的pwm的大小
#if 1
            static uint16_t	Up_Time 	= 40;	//起步结束时间，150*10ms = 1.5s
			static uint16_t	F1_Time 	= 114;	//第1次前进结束时间 - 起步结束时间 = 前进花费时间,往后以此类推

			static uint16_t	STOP		=700;	//STOP - F1_Time = 按键按下到开始起飞花费时间(预留时间尽量5s以上)
			//程序修改为按键控制之后，STOP可以认为是飞向大圆开始时间

			static uint16_t	Up2_Time 	= 900;	//飞向大圆结束时间825



	        static uint16_t Flight_PWM 	= 670;	//	电机使用的pwm的大小

/*---------注意，改完static uint16_t 	Up2_Time 	= 900;飞向大圆结束时间，还需要修改277行的安全停止时间！！！-----------*/

            //起步
            if (Flight_timer >= 0 && Flight_timer < Up_Time) {

            	//角度
            	pid_state_roll.target 	= 0;
            	pid_state_pitch.target 	= 0;

            	//角速度
            	pid_state_GyroZ.target 	= 0;
            	FLight_base_pwm	= Flight_PWM;

			}

            //第1次前进
            else if (Flight_timer >= Up_Time && Flight_timer < F1_Time){

            	//角度
            	pid_state_roll.target 	= -13;
            	pid_state_pitch.target 	= 0;

            	//角速度
            	pid_state_GyroZ.target 	= 60;
            	FLight_base_pwm	= Flight_PWM;

			}

								/*视觉传感器预留区*/
            else if(Flight_timer >= F1_Time && Flight_timer < STOP && Key1 == 1){

            	Key1_Strat = 1;
            	Up2_Start = 1;	//飞向大圆开始标志

            }

            //第2次前进
            else if (Flight_timer >= STOP && Flight_timer < Up2_Time && Key1_Strat){

            	//角度
            	pid_state_roll.target 	= -5;
            	pid_state_pitch.target 	= -8;

            	//角速度
            	pid_state_GyroZ.target 	= 50;
            	FLight_base_pwm	= Flight_PWM;



			}

#endif

/*----------------------------------------飞行逻辑运行段-----------------------------------------------------*/

            /*========== PID计算核心部分 ==========*/
#if 0
            // 角度环PID - 外环 进行飞行逻辑时，主要控制的就是目标角度，所以当注释
			pid_state_roll.target = 0;
			pid_state_pitch.target = 0;
#endif
            float roll_calib = roll_isr + RY0_OFFSET;
			float pitch_calib = pitch_isr + PX0_OFFSET;
			float yaw_calib = yaw_isr;

			//新增PID死区控制2025.10.21
			//横滚死区
			if (roll_calib > -0.2 && roll_calib < 0.2){

				roll_calib = 0;

			}
			//俯仰死区
			if(pitch_calib > -0.2 && pitch_calib < 0.2){

				pitch_calib = 0;

			}
			//偏航死区
			if(yaw_calib > -0.2 && yaw_calib < 0.2){

				yaw_calib = 0;

			}

            pid_state_roll.actual = roll_calib;
            pid_state_pitch.actual = pitch_calib;


            pid_state_roll = PID_Iterate(pid_KGain_roll, pid_state_roll);
            pid_state_pitch = PID_Iterate(pid_KGain_pitch, pid_state_pitch);



            // 角速度环PID - 内环
            // 将角度环的输出设置为角速度环的目标值
            pid_state_GyroX.target = -pid_state_pitch.output;  // 俯仰角度输出 -> X轴角速度目标
            pid_state_GyroY.target = -pid_state_roll.output;   // 横滚角度输出 -> Y轴角速度目标

#if	0
            //由于进行飞行逻辑时，Z主要控制的就是角速度，所以当注释
            pid_state_GyroZ.target = -pid_state_yaw.output;
#endif
            float gyro_x_calib = gyro_x_isr + GX0_OFFSET;
            float gyro_y_calib = gyro_y_isr + GY0_OFFSET;
            float gyro_z_calib = gyro_z_isr + GZ0_OFFSET;

            //X角速度死区	2025.10.21
            if(gyro_x_calib > -0.2 && gyro_x_calib < 0.2){

            	gyro_x_calib = 0;

			}
            //Y角速度死区
            if(gyro_y_calib > -0.2 && gyro_y_calib < 0.2){
            	gyro_y_calib = 0;
            }
            //Z角速度死区
            if(gyro_z_calib > -0.2 && gyro_z_calib < 0.2){
            	gyro_z_calib = 0;
			}

            pid_state_GyroX.actual = gyro_x_calib;
            pid_state_GyroY.actual = gyro_y_calib;
            pid_state_GyroZ.actual = gyro_z_calib;

            pid_state_GyroX = PID_Iterate(pid_KGain_GyroX, pid_state_GyroX);
            pid_state_GyroY = PID_Iterate(pid_KGain_GyroY, pid_state_GyroY);
            pid_state_GyroZ = PID_Iterate(pid_KGain_GyroZ, pid_state_GyroZ);

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
            int PID_GyroX_OUT = (int)pid_state_GyroX.output;
            int PID_GyroY_OUT = (int)pid_state_GyroY.output;
            int	PID_GyroZ_OUT = (int)pid_state_GyroZ.output;

            uint16_t base_pwm_value = FLight_base_pwm;

            //每一个电机最终由基本PWM值+PID输出值组成
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

            //停止在小圆
            if(Flight_timer >= F1_Time && !Up2_Start){

            	Flight_timer = F1_Time + 1;
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

            }
            if(Flight_timer >= F1_Time && Flight_timer < STOP){


                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

             }



            //结束(停止在大圆)
             if (Flight_timer >= Up2_Time && Up2_Start){

                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

             	while(1){
                     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
             	}

 			}
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
