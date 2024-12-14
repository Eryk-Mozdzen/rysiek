/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MOTOR_PIN_SET(signal, state) HAL_GPIO_WritePin((signal).port, (signal).pin, (state))
#define MOTOR_NUM 3

typedef enum {
	MOTOR_DIR_FORWARD = -1,
	MOTOR_DIR_BACKWARD = 1,
} motor_dit_t;

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} motor_signal_t;

typedef struct {
	motor_signal_t a1;
	motor_signal_t a2;
	motor_signal_t b1;
	motor_signal_t b2;
	int8_t state;
} motor_t;

motor_t motors[MOTOR_NUM];

void drive_motor(motor_t *motor, const motor_dit_t dir) {
	switch(motor->state) {
	    case 0: {
        MOTOR_PIN_SET(motor->b2, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->a2, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->b1, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->a1, GPIO_PIN_SET);
	      motor->state +=dir;
	    } break;
	    case 1: {
        MOTOR_PIN_SET(motor->a1, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->b2, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->a2, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->b1, GPIO_PIN_SET);
	      motor->state +=dir;
	    } break;
	    case 2: {
        MOTOR_PIN_SET(motor->b1, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->a1, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->b2, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->a2, GPIO_PIN_SET);
	    motor->state +=dir;
      } break;
	    case 3: {
        MOTOR_PIN_SET(motor->a2, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->b1, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->a1, GPIO_PIN_RESET);
        MOTOR_PIN_SET(motor->b2, GPIO_PIN_SET);
	      motor->state +=dir;
      } break;
	    default: {
	      motor->state = 0;
      } break;
   }

   if(motor->state > 3) {
      motor->state = 0;
    }

   if(motor->state < 0) {
      motor->state = 3;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim==&htim7) {
		for(uint8_t i=0; i<MOTOR_NUM; i++) {
			drive_motor(&motors[i], MOTOR_DIR_FORWARD);
		}
	}
}

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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  motors[0].a1 = (motor_signal_t){.port = M1_A1_GPIO_Port, .pin = M1_A1_Pin};
  motors[0].a2 = (motor_signal_t){.port = M1_A2_GPIO_Port, .pin = M1_A2_Pin};
  motors[0].b1 = (motor_signal_t){.port = M1_B1_GPIO_Port, .pin = M1_B1_Pin};
  motors[0].b2 = (motor_signal_t){.port = M1_B2_GPIO_Port, .pin = M1_B2_Pin};

  motors[1].a1 = (motor_signal_t){.port = M2_A1_GPIO_Port, .pin = M2_A1_Pin};
  motors[1].a2 = (motor_signal_t){.port = M2_A2_GPIO_Port, .pin = M2_A2_Pin};
  motors[1].b1 = (motor_signal_t){.port = M2_B1_GPIO_Port, .pin = M2_B1_Pin};
  motors[1].b2 = (motor_signal_t){.port = M2_B2_GPIO_Port, .pin = M2_B2_Pin};

  motors[2].a1 = (motor_signal_t){.port = M3_A1_GPIO_Port, .pin = M3_A1_Pin};
  motors[2].a2 = (motor_signal_t){.port = M3_A2_GPIO_Port, .pin = M3_A2_Pin};
  motors[2].b1 = (motor_signal_t){.port = M3_B1_GPIO_Port, .pin = M3_B1_Pin};
  motors[2].b2 = (motor_signal_t){.port = M3_B2_GPIO_Port, .pin = M3_B2_Pin};

  HAL_Delay(6000);
  HAL_GPIO_WritePin(Mx_EN_GPIO_Port, Mx_EN_Pin, 1);
  HAL_TIM_Base_Start_IT(&htim7);

  while(1) {
	/*for(uint8_t i=0; i<MOTOR_NUM; i++) {
	  drive_motor(&motors[i], MOTOR_DIR_FORWARD);
	}

	HAL_Delay(1);*/
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 80-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M2_B2_Pin|M2_B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M3_A1_Pin|Mx_EN_Pin|M1_A1_Pin|M1_A2_Pin
                          |M1_B1_Pin|M1_B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M3_B1_Pin|M3_B2_Pin|M3_A2_Pin|M2_A1_Pin
                          |M2_A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M2_B2_Pin M2_B1_Pin */
  GPIO_InitStruct.Pin = M2_B2_Pin|M2_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_A1_Pin Mx_EN_Pin M1_A1_Pin M1_A2_Pin
                           M1_B1_Pin M1_B2_Pin */
  GPIO_InitStruct.Pin = M3_A1_Pin|Mx_EN_Pin|M1_A1_Pin|M1_A2_Pin
                          |M1_B1_Pin|M1_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_B1_Pin M3_B2_Pin M3_A2_Pin M2_A1_Pin
                           M2_A2_Pin */
  GPIO_InitStruct.Pin = M3_B1_Pin|M3_B2_Pin|M3_A2_Pin|M2_A1_Pin
                          |M2_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
