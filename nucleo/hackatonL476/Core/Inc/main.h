/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MATRIX_COL3_Pin GPIO_PIN_13
#define MATRIX_COL3_GPIO_Port GPIOC
#define MATRIX_COL4_Pin GPIO_PIN_14
#define MATRIX_COL4_GPIO_Port GPIOC
#define MATRIX_ROW2_Pin GPIO_PIN_15
#define MATRIX_ROW2_GPIO_Port GPIOC
#define MATRIX_ROW3_Pin GPIO_PIN_0
#define MATRIX_ROW3_GPIO_Port GPIOH
#define MATRIX_ROW4_Pin GPIO_PIN_1
#define MATRIX_ROW4_GPIO_Port GPIOH
#define MATRIX_COL5_Pin GPIO_PIN_0
#define MATRIX_COL5_GPIO_Port GPIOC
#define MATRIX_ROW7_Pin GPIO_PIN_1
#define MATRIX_ROW7_GPIO_Port GPIOC
#define MATRIX_ROW5_Pin GPIO_PIN_2
#define MATRIX_ROW5_GPIO_Port GPIOC
#define MATRIX_ROW6_Pin GPIO_PIN_3
#define MATRIX_ROW6_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_0
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_1
#define SERVO2_GPIO_Port GPIOA
#define MATRIX_COL7_Pin GPIO_PIN_4
#define MATRIX_COL7_GPIO_Port GPIOA
#define U0_ECHO_Pin GPIO_PIN_5
#define U0_ECHO_GPIO_Port GPIOA
#define U0_ECHO_EXTI_IRQn EXTI9_5_IRQn
#define M2_B2_Pin GPIO_PIN_6
#define M2_B2_GPIO_Port GPIOA
#define M2_B1_Pin GPIO_PIN_7
#define M2_B1_GPIO_Port GPIOA
#define M3_A1_Pin GPIO_PIN_4
#define M3_A1_GPIO_Port GPIOC
#define Mx_EN_Pin GPIO_PIN_5
#define Mx_EN_GPIO_Port GPIOC
#define MATRIX_COL6_Pin GPIO_PIN_0
#define MATRIX_COL6_GPIO_Port GPIOB
#define BEAM_CLK_Pin GPIO_PIN_1
#define BEAM_CLK_GPIO_Port GPIOB
#define BEAM_DOUT_Pin GPIO_PIN_2
#define BEAM_DOUT_GPIO_Port GPIOB
#define M3_B1_Pin GPIO_PIN_13
#define M3_B1_GPIO_Port GPIOB
#define M3_B2_Pin GPIO_PIN_14
#define M3_B2_GPIO_Port GPIOB
#define M1_A1_Pin GPIO_PIN_6
#define M1_A1_GPIO_Port GPIOC
#define M1_A2_Pin GPIO_PIN_7
#define M1_A2_GPIO_Port GPIOC
#define M1_B1_Pin GPIO_PIN_8
#define M1_B1_GPIO_Port GPIOC
#define M1_B2_Pin GPIO_PIN_9
#define M1_B2_GPIO_Port GPIOC
#define U0_TRIG_Pin GPIO_PIN_8
#define U0_TRIG_GPIO_Port GPIOA
#define U1_ECHO_Pin GPIO_PIN_9
#define U1_ECHO_GPIO_Port GPIOA
#define U1_ECHO_EXTI_IRQn EXTI9_5_IRQn
#define U1_TRIG_Pin GPIO_PIN_10
#define U1_TRIG_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_12
#define SW_GPIO_Port GPIOA
#define MATRIX_ROW1_Pin GPIO_PIN_15
#define MATRIX_ROW1_GPIO_Port GPIOA
#define MATRIX_COL0_Pin GPIO_PIN_10
#define MATRIX_COL0_GPIO_Port GPIOC
#define MATRIX_COL1_Pin GPIO_PIN_11
#define MATRIX_COL1_GPIO_Port GPIOC
#define MATRIX_COL2_Pin GPIO_PIN_12
#define MATRIX_COL2_GPIO_Port GPIOC
#define MATRIX_ROW0_Pin GPIO_PIN_2
#define MATRIX_ROW0_GPIO_Port GPIOD
#define M3_A2_Pin GPIO_PIN_3
#define M3_A2_GPIO_Port GPIOB
#define M2_A1_Pin GPIO_PIN_4
#define M2_A1_GPIO_Port GPIOB
#define M2_A2_Pin GPIO_PIN_5
#define M2_A2_GPIO_Port GPIOB
#define STRIPE_R_Pin GPIO_PIN_6
#define STRIPE_R_GPIO_Port GPIOB
#define STRIPE_G_Pin GPIO_PIN_7
#define STRIPE_G_GPIO_Port GPIOB
#define STRIPE_B_Pin GPIO_PIN_8
#define STRIPE_B_GPIO_Port GPIOB
#define STRIPE_W_Pin GPIO_PIN_9
#define STRIPE_W_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
