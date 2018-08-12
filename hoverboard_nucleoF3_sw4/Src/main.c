
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdlib.h>
#include "Uart.h"
#include "BLDC_Motors.h"
#include "tm_stm32_delay.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include "PID_regulator.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//UART
extern uint8_t str[50];
extern uint16_t size;

//MPU
TM_MPU6050_t MPU6050_Data0;
TM_MPU6050_t MPU6050_Data1;
uint8_t sensor0 = 0;
uint8_t sensor1 = 0;

//KALMAN
float sensor0_kalman_result = 0;
float sensor1_kalman_result = 0;
int32_t d0_kal_result = 0;	//for Uart
int32_t d1_kal_result = 0;	//for Uart
float angle0_result = 0;
float angle1_result = 0;
int32_t d0_angle_result = 0;	//for Uart
int32_t d1_angle_result = 0;	//for Uart

//PID
float pid0_result = 0;
float pid1_result = 0;
int32_t d0_pid_result = 0;	//for Uart
int32_t d1_pid_result = 0;	//for Uart

//TEMPORARY & GARBAGE
int32_t temp = 0;
int32_t lastTick = 0;
int32_t interval = 0;
uint8_t allow = 1;


//uint8_t Received[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); //odbieranie komendy o ró¿nej d³ugoœci

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//cykliczne pomiary MPU, filtr KALMANa, regulator PID

 //if( HAL_GPIO_ReadPin(BUTTON_START_GPIO_Port, BUTTON_START_Pin) == GPIO_PIN_RESET) {

	if(htim->Instance == TIM6){ // Je¿eli przerwanie pochodzi od timera 6

		  //Reading UART command
		  UART_Command_Reading();
		if(allow) {

			interval = HAL_GetTick()-lastTick;
			lastTick = HAL_GetTick();


				TM_MPU6050_ReadAll(&MPU6050_Data0);	//RIGHT

				sensor0_kalman_result = kalman_filter_get_est(&K_MPU6050_0, MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_X);

				d0_kal_result = (int8_t) sensor0_kalman_result;	//for Uart

				angle0_result = angle_before_kalman(MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z);
				d0_angle_result = (int8_t) angle0_result;	//for Uart

				pid0_result = PID_calculate(0,sensor0_kalman_result);
				d0_pid_result = (int16_t) pid0_result;	//for Uart



				TM_MPU6050_ReadAll(&MPU6050_Data1);	//LEFT
				sensor1_kalman_result = -kalman_filter_get_est(&K_MPU6050_1, MPU6050_Data1.Accelerometer_Y, MPU6050_Data1.Accelerometer_Z, MPU6050_Data1.Gyroscope_X);

				d1_kal_result = (int8_t) sensor1_kalman_result;	//for Uart

				angle1_result = -angle_before_kalman(MPU6050_Data1.Accelerometer_Y, MPU6050_Data1.Accelerometer_Z);
				d1_angle_result = (int8_t) angle1_result;	//for Uart

				pid1_result = PID_calculate(0,sensor1_kalman_result);
				d1_pid_result = (int16_t) pid1_result;	//for Uart


				size = sprintf(str, "%d %d %d      %d %d %d    %d %d\r\n", d0_pid_result, d0_angle_result, d0_kal_result, d1_pid_result, d1_angle_result, d1_kal_result, 100, -100);
				HAL_UART_Transmit_IT(&huart2, str, size);


			//	sensor0_kalman_result = kalman_filter_get_est(&K_MPU6050_0, MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_X);



/*
				// Format data
				size = sprintf(str, "Sensor0: Angle:%d   Kalman:%d   PID:%d   Acc: X:%d   Y:%d   Z:%d         Gyr: X:%d   Y:%d   Z:%d \r\n",
				d0_angle_result,
				d0_kal_result,
				d0_pid_result,
				MPU6050_Data0.Accelerometer_X,
				MPU6050_Data0.Accelerometer_Y,
				MPU6050_Data0.Accelerometer_Z,
				MPU6050_Data0.Gyroscope_X,
				MPU6050_Data0.Gyroscope_Y,
				MPU6050_Data0.Gyroscope_Z
				);


				size = sprintf(str, "%d  %d   %d %d      %d %d %d \r\n", sensor0, d0_pid_result, d0_angle_result, d0_kal_result, 100, -100, interval);
				HAL_UART_Transmit_IT(&huart2, str, size);



				//-------------




				// Format data
				size = sprintf(str, "Sensor1: Angle:%d   Kalman:%d   PID:%d   Acc: X:%d   Y:%d   Z:%d         Gyr: X:%d   Y:%d   Z:%d \r\n",
				d1_angle_result,
				d1_kal_result,
				d1_pid_result,
				MPU6050_Data1.Accelerometer_X,
				MPU6050_Data1.Accelerometer_Y,
				MPU6050_Data1.Accelerometer_Z,
				MPU6050_Data1.Gyroscope_X,
				MPU6050_Data1.Gyroscope_Y,
				MPU6050_Data1.Gyroscope_Z
				);


				size = sprintf(str, "%d  %d   %d %d      %d %d %d \r\n", sensor1, d1_pid_result, d1_angle_result, d1_kal_result, 100, -100, interval);
				HAL_UART_Transmit_IT(&huart2, str, size);
*/

		}

	}

 //}
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //Uart init
  HAL_UART_Receive_IT(&huart2, Rx_data, 1);

  //BLDC_Motors init
  Initialize_LR_Motors(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  Stop_LR_Motors();

  //==================


  //Accelerometr MPU6050 init

  // Initialize MPU6050 sensor 0, address = 0xD0, AD0 pin on sensor is low
  if (TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s) == TM_MPU6050_Result_Ok) {
		// Display message to user
		size = sprintf(str, "MPU6050 sensor 0 is ready to use! (AD0:low)\n");
		HAL_UART_Transmit(&huart2, str, size, 1000);
		sensor0 = 1;		// Sensor 0 OK	//RIGHT
  }

  // Initialize MPU6050 sensor 1, address = 0xD2, AD0 pin on sensor is high
  if (TM_MPU6050_Init(&MPU6050_Data1, TM_MPU6050_Device_1, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s) == TM_MPU6050_Result_Ok) {
		// Display message to user
		size = sprintf(str, "MPU6050 sensor 1 is ready to use! (AD0:high)\n");
		HAL_UART_Transmit(&huart2, str, size, 1000);
		sensor1 = 1;		// Sensor 1 OK	//LEFT
  }
  //==================

  HAL_Delay(200);
  //Kalman filter init

  TM_MPU6050_ReadAll(&MPU6050_Data0);
  kalman_filter_init(&K_MPU6050_0, MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z);

  TM_MPU6050_ReadAll(&MPU6050_Data1);
  kalman_filter_init(&K_MPU6050_1, MPU6050_Data1.Accelerometer_Y, MPU6050_Data1.Accelerometer_Z);


  //Init cyclic timer (MPU, KALMAN, PID) - after Kalman init!
  HAL_TIM_Base_Start_IT(&htim6);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  /*
	  size = sprintf(str, "kutaczan %d\r\n", 22);
	  HAL_UART_Transmit_IT(&huart2, str, size);
	  */

	//------------------------------------

/*
		TM_MPU6050_ReadAll(&MPU6050_Data0);	//RIGHT
		sensor0_kalman_result = kalman_filter_get_est(&K_MPU6050_0, MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_X);

		d0_kal_result = sensor0_kalman_result;	//for Uart

		angle0_result = angle_before_kalman(MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z);
		d0_angle_result = angle0_result;	//for Uart

		pid0_result = PID_calculate(0,sensor0_kalman_result);
		d0_pid_result = pid0_result;	//for Uart



		TM_MPU6050_ReadAll(&MPU6050_Data1);	//LEFT
		sensor1_kalman_result = -kalman_filter_get_est(&K_MPU6050_1, MPU6050_Data1.Accelerometer_Y, MPU6050_Data1.Accelerometer_Z, MPU6050_Data1.Gyroscope_X);

		d1_kal_result = sensor1_kalman_result;	//for Uart

		angle1_result = -angle_before_kalman(MPU6050_Data1.Accelerometer_Y, MPU6050_Data1.Accelerometer_Z);
		d1_angle_result = angle1_result;	//for Uart

		pid1_result = PID_calculate(0,sensor1_kalman_result);
		d1_pid_result = pid1_result;	//for Uart


		size = sprintf(str, "%d %d %d      %d %d %d    %d %d\r\n", d0_pid_result, d0_angle_result, d0_kal_result, d1_pid_result, d1_angle_result, d1_kal_result, MPU6050_Data0.Gyroscope_X, MPU6050_Data1.Gyroscope_X);
		HAL_UART_Transmit_IT(&huart2, str, size);


	  	  HAL_Delay(50);

*/

	//--------------------------------


/*
      if (sensor0) {
           // Read all data from sensor 0
           TM_MPU6050_ReadAll(&MPU6050_Data0);
           sensor0_kalman_result = kalman_filter_get_est(&K_MPU6050_0, MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_X);
 	  	   d0_kal_result = sensor0_kalman_result;	//for Uart

 		   angle0_result = angle_before_kalman(MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z);
 		   d0_angle_result = angle0_result;	//for Uart

           // Format data
           size = sprintf(str, "Sensor0: Angle:%d   Kalman:%d      Acc: X:%d   Y:%d   Z:%d         Gyr: X:%d   Y:%d   Z:%d \r\n",
        	   d0_angle_result,
			   d0_kal_result,
               MPU6050_Data0.Accelerometer_X,
               MPU6050_Data0.Accelerometer_Y,
               MPU6050_Data0.Accelerometer_Z,
               MPU6050_Data0.Gyroscope_X,
               MPU6050_Data0.Gyroscope_Y,
               MPU6050_Data0.Gyroscope_Z
           );

       //    size = sprintf(str, "%d %d      %d %d \r\n", d_angle_result, d_kal_result, 100, -100);
           HAL_UART_Transmit(&huart2, str, size, 1000);
      }

  	  HAL_Delay(500);


      if (sensor1) {
           // Read all data from sensor 1
           TM_MPU6050_ReadAll(&MPU6050_Data1);
           sensor1_kalman_result = kalman_filter_get_est(&K_MPU6050_1, MPU6050_Data1.Accelerometer_Y, MPU6050_Data1.Accelerometer_Z, MPU6050_Data1.Gyroscope_X);
 	  	   d1_kal_result = sensor1_kalman_result;	//for Uart

 		   angle1_result = angle_before_kalman(MPU6050_Data1.Accelerometer_Y, MPU6050_Data1.Accelerometer_Z);
 		   d1_angle_result = angle1_result;	//for Uart

           // Format data

           size = sprintf(str, "Sensor1: Angle:%d   Kalman:%d      Acc: X:%d   Y:%d   Z:%d         Gyr: X:%d   Y:%d   Z:%d \r\n",
               d1_angle_result,
    	       d1_kal_result,
               MPU6050_Data1.Accelerometer_X,
               MPU6050_Data1.Accelerometer_Y,
               MPU6050_Data1.Accelerometer_Z,
               MPU6050_Data1.Gyroscope_X,
               MPU6050_Data1.Gyroscope_Y,
               MPU6050_Data1.Gyroscope_Z
           );

          //     size = sprintf(str, "%d %d      %d %d \r\n", d_angle_result, d_kal_result, 100, -100);

           HAL_UART_Transmit(&huart2, str, size, 1000);
      }


  	  HAL_Delay(500);
*/

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 36000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin|LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_MOTOR_DIR_Pin LEFT_MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = RIGHT_MOTOR_DIR_Pin|LEFT_MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_START_Pin */
  GPIO_InitStruct.Pin = BUTTON_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_START_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //odbieranie komendy o ró¿nej d³ugoœci
{
	if (huart->Instance == USART2)  //current UART
	{
		Uart_Receive();
		HAL_UART_Receive_IT(&huart2, Rx_data, 1); //activate UART receive interrupt every time
	}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
