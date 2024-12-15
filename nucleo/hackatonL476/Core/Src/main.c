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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define SIGNAL_SET(signal, state) HAL_GPIO_WritePin((signal).port, (signal).pin, (state))
#define SIGNAL_GET(signal)        HAL_GPIO_ReadPin((signal).port, (signal).pin)

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} signal_t;

#define MOTOR_NUM 3

typedef enum {
	MOTOR_DIR_FORWARD = -1,
	MOTOR_DIR_BACKWARD = 1,
} motor_dir_t;

typedef struct {
	signal_t a1;
	signal_t a2;
	signal_t b1;
	signal_t b2;
	int8_t state;
} motor_t;

static motor_t motors[MOTOR_NUM];

void drive_motor(motor_t *motor, const motor_dir_t dir) {
	switch(motor->state) {
	    case 0: {
	    	SIGNAL_SET(motor->b2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a1, GPIO_PIN_SET);
	    	motor->state +=dir;
	    } break;
	    case 1: {
	    	SIGNAL_SET(motor->a1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b1, GPIO_PIN_SET);
	    	motor->state +=dir;
	    } break;
	    case 2: {
	    	SIGNAL_SET(motor->b1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a2, GPIO_PIN_SET);
	    	motor->state +=dir;
      } break;
	    case 3: {
	    	SIGNAL_SET(motor->a2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b2, GPIO_PIN_SET);
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


#define MATRIX_COLS 8
#define MATRIX_ROWS 8

typedef struct {
	signal_t cols[MATRIX_COLS];
	signal_t rows[MATRIX_ROWS];
	volatile uint8_t current_row;
} matrix_t;

const uint8_t heart[8][8] = {
	{0, 1, 1, 0, 0, 1, 1, 0},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{0, 1, 1, 1, 1, 1, 1, 0},
	{0, 0, 1, 1, 1, 1, 0, 0},
	{0, 0, 0, 1, 1, 0, 0, 0},
};

void matrix_clear(matrix_t *matrix) {
	for(uint8_t i=0; i<MATRIX_COLS; i++) {
		SIGNAL_SET(matrix->cols[i], 0);
	}
	for(uint8_t i=0; i<MATRIX_ROWS; i++) {
		SIGNAL_SET(matrix->rows[i], 0);
	}
}

void matrix_heart(matrix_t *matrix) {
	for(uint8_t i=0; i<MATRIX_ROWS; i++) {
		SIGNAL_SET(matrix->rows[i], 1);
	}
	for(uint8_t i=0; i<MATRIX_COLS; i++) {
		SIGNAL_SET(matrix->cols[i], 0);
	}

	matrix->current_row++;
	matrix->current_row %=MATRIX_ROWS;

	SIGNAL_SET(matrix->rows[matrix->current_row], 0);
	for(uint8_t i=0; i<MATRIX_COLS; i++) {
		SIGNAL_SET(matrix->cols[i], heart[matrix->current_row][i]);
	}
}

static matrix_t matrix = {0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim==&htim7) {
		for(uint8_t i=0; i<MOTOR_NUM; i++) {
			drive_motor(&motors[i], MOTOR_DIR_FORWARD);
		}

		matrix_heart(&matrix);
	}
}

#define ULTRA_NUM            1
#define ULTRA_TRIG_TIM       htim1
#define ULTRA_TO_SEC(delta)  (0.000002f*(delta))
#define ULTRA_SPEED_OF_SOUND 343.f

typedef struct {
	signal_t echo;
	uint32_t start;
	uint32_t end;
	uint32_t delta;
	bool valid;
	float dist;
} ultrasonic_t;

static ultrasonic_t ultras[ULTRA_NUM] = {0};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	for(uint8_t i=0; i<ULTRA_NUM; i++) {
		if(GPIO_Pin==ultras[i].echo.pin) {
			if(HAL_GPIO_ReadPin(ultras[i].echo.port, ultras[i].echo.pin)) {
				ultras[i].valid = false;
				ultras[i].start = __HAL_TIM_GET_COUNTER(&ULTRA_TRIG_TIM);
			} else {
				ultras[i].end = __HAL_TIM_GET_COUNTER(&ULTRA_TRIG_TIM);
				ultras[i].delta = ultras[i].end - ultras[i].start;
				ultras[i].dist = ULTRA_TO_SEC(ultras[i].delta)*ULTRA_SPEED_OF_SOUND*0.5f;
				ultras[i].valid = true;
			}
		}
	}
}

static void logger(const char *format, ...) {
    va_list args;
    va_start(args, format);

	char buffer[256];
    const uint16_t len = vsprintf(buffer, format, args);

    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

#define LOAD_CALIB_0KG 800000
#define LOAD_CALIB_1KG 900000
#define LOAD_TIMEOUT   10

typedef struct {
  signal_t dout;
  signal_t sck;
  int32_t data;
  float load;
} beam_t;

void beam_init(beam_t *beam) {
	//HAL_GPIO_WritePin(LOAD_RATE_GPIO_Port, LOAD_RATE_Pin, GPIO_PIN_RESET);

	SIGNAL_SET(beam->sck, GPIO_PIN_SET);
	HAL_Delay(10);
	SIGNAL_SET(beam->sck, GPIO_PIN_RESET);
	HAL_Delay(10);
}

bool beam_read(beam_t *beam) {
	beam->data = 0;

	uint32_t start_time = HAL_GetTick();

	while(SIGNAL_GET(beam->dout) == GPIO_PIN_SET) {
		if((HAL_GetTick() - start_time)>LOAD_TIMEOUT) {
			return false;
		}
	}

	for(uint8_t i=0; i<24; i++) {
		SIGNAL_SET(beam->sck, GPIO_PIN_SET);
		for(volatile uint32_t i=0; i<100; i++) {}
		SIGNAL_SET(beam->sck, GPIO_PIN_RESET);
		for(volatile uint32_t i=0; i<100; i++) {}

		beam->data <<=1;

		if(SIGNAL_GET(beam->dout) == GPIO_PIN_SET) {
			beam->data |=0x01;
		}
	}

	beam->data ^=0x800000;

	SIGNAL_SET(beam->sck, GPIO_PIN_SET);
	for(volatile uint32_t i=0; i<100; i++) {}
	SIGNAL_SET(beam->sck, GPIO_PIN_RESET);
	for(volatile uint32_t i=0; i<100; i++) {}

	beam->load = ((float)(beam->load - LOAD_CALIB_0KG))/((float)(LOAD_CALIB_1KG - LOAD_CALIB_0KG));

	return true;
}

static beam_t beam = {0};

#define STRIPE_TIM   htim4
#define STRIPE_CHN_R TIM_CHANNEL_1
#define STRIPE_CHN_G TIM_CHANNEL_4
#define STRIPE_CHN_B TIM_CHANNEL_2
#define STRIPE_CHN_W TIM_CHANNEL_3

#define ARRAY_SIZE 8
#define ROBOT_READY_LED_COUNTER_MAX		((uint16_t)240)

const uint8_t array[ARRAY_SIZE][3] = {
  	{  0,   0, 255},
  	{127,   0, 255},
  	{255,   0, 255},
  	{127,   0, 255},
  	{  0,   0, 255},
  	{  0, 127, 255},
  	{  0, 255, 255},
  	{  0, 127, 255}
};

void stripe_init() {
	HAL_TIM_Base_Start(&STRIPE_TIM);
	HAL_TIM_PWM_Start(&STRIPE_TIM, STRIPE_CHN_R);
	HAL_TIM_PWM_Start(&STRIPE_TIM, STRIPE_CHN_G);
	HAL_TIM_PWM_Start(&STRIPE_TIM, STRIPE_CHN_B);
	HAL_TIM_PWM_Start(&STRIPE_TIM, STRIPE_CHN_W);
}

void stripe_set(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
	r = (r>250) ? 250 : r;
	g = (g>250) ? 250 : g;
	b = (b>250) ? 250 : b;
	w = (w>250) ? 250 : w;

	__HAL_TIM_SET_COMPARE(&STRIPE_TIM, STRIPE_CHN_R, r);
	__HAL_TIM_SET_COMPARE(&STRIPE_TIM, STRIPE_CHN_G, g);
	__HAL_TIM_SET_COMPARE(&STRIPE_TIM, STRIPE_CHN_B, b);
	__HAL_TIM_SET_COMPARE(&STRIPE_TIM, STRIPE_CHN_W, w);
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_Delay(6000);

  motors[0].a1 = (signal_t){.port = M1_A1_GPIO_Port, .pin = M1_A1_Pin};
  motors[0].a2 = (signal_t){.port = M1_A2_GPIO_Port, .pin = M1_A2_Pin};
  motors[0].b1 = (signal_t){.port = M1_B1_GPIO_Port, .pin = M1_B1_Pin};
  motors[0].b2 = (signal_t){.port = M1_B2_GPIO_Port, .pin = M1_B2_Pin};

  motors[1].a1 = (signal_t){.port = M2_A1_GPIO_Port, .pin = M2_A1_Pin};
  motors[1].a2 = (signal_t){.port = M2_A2_GPIO_Port, .pin = M2_A2_Pin};
  motors[1].b1 = (signal_t){.port = M2_B1_GPIO_Port, .pin = M2_B1_Pin};
  motors[1].b2 = (signal_t){.port = M2_B2_GPIO_Port, .pin = M2_B2_Pin};

  motors[2].a1 = (signal_t){.port = M3_A1_GPIO_Port, .pin = M3_A1_Pin};
  motors[2].a2 = (signal_t){.port = M3_A2_GPIO_Port, .pin = M3_A2_Pin};
  motors[2].b1 = (signal_t){.port = M3_B1_GPIO_Port, .pin = M3_B1_Pin};
  motors[2].b2 = (signal_t){.port = M3_B2_GPIO_Port, .pin = M3_B2_Pin};

  matrix.rows[1] = (signal_t){.port = MATRIX_ROW0_GPIO_Port, .pin = MATRIX_ROW0_Pin};
  matrix.rows[0] = (signal_t){.port = MATRIX_COL1_GPIO_Port, .pin = MATRIX_COL1_Pin};
  matrix.rows[7] = (signal_t){.port = MATRIX_COL2_GPIO_Port, .pin = MATRIX_COL2_Pin};
  matrix.rows[2] = (signal_t){.port = MATRIX_COL3_GPIO_Port, .pin = MATRIX_COL3_Pin};
  matrix.rows[4] = (signal_t){.port = MATRIX_COL4_GPIO_Port, .pin = MATRIX_COL4_Pin};
  matrix.rows[3] = (signal_t){.port = MATRIX_COL5_GPIO_Port, .pin = MATRIX_COL5_Pin};
  matrix.rows[5] = (signal_t){.port = MATRIX_COL6_GPIO_Port, .pin = MATRIX_COL6_Pin};
  matrix.rows[6] = (signal_t){.port = MATRIX_COL7_GPIO_Port, .pin = MATRIX_COL7_Pin};

  matrix.cols[1] = (signal_t){.port = MATRIX_COL0_GPIO_Port, .pin = MATRIX_COL0_Pin};
  matrix.cols[3] = (signal_t){.port = MATRIX_ROW1_GPIO_Port, .pin = MATRIX_ROW1_Pin};
  matrix.cols[0] = (signal_t){.port = MATRIX_ROW2_GPIO_Port, .pin = MATRIX_ROW2_Pin};
  matrix.cols[4] = (signal_t){.port = MATRIX_ROW3_GPIO_Port, .pin = MATRIX_ROW3_Pin};
  matrix.cols[6] = (signal_t){.port = MATRIX_ROW4_GPIO_Port, .pin = MATRIX_ROW4_Pin};
  matrix.cols[5] = (signal_t){.port = MATRIX_ROW5_GPIO_Port, .pin = MATRIX_ROW5_Pin};
  matrix.cols[2] = (signal_t){.port = MATRIX_ROW6_GPIO_Port, .pin = MATRIX_ROW6_Pin};
  matrix.cols[7] = (signal_t){.port = MATRIX_ROW7_GPIO_Port, .pin = MATRIX_ROW7_Pin};

  //ultras[0].echo = (signal_t){.port = U1_ECHO_GPIO_Port, .pin = U1_ECHO_Pin};

  //beam.dout = (signal_t){.port = , .pin = };
  //beam.clk = (signal_t){.port = , .pin = };

  HAL_GPIO_WritePin(Mx_EN_GPIO_Port, Mx_EN_Pin, 1);
  HAL_TIM_Base_Start_IT(&htim7); // motors steps & matrix

  //HAL_TIM_Base_Start(&htim1); // ultrasonic trigger
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  beam_init(&beam);
  stripe_init();

  uint32_t rgbw_time = 0;
  uint32_t counter = 0;

  while(1) {
	  const uint32_t time = HAL_GetTick();

	  /*if(ultras[0].valid) {
		  logger("dist = %10.3fm\n\r", ultras[0].dist);
		  HAL_Delay(100);
	  }*/

	  if(beam_read(&beam)) {
		  logger("load = %10.3fkg\n\r", beam.load);
		  HAL_Delay(100);
	  }

	  if((time - rgbw_time)>=1) {
			rgbw_time = time;

			const uint8_t (*color1)[3] = &array[((ARRAY_SIZE*counter)/ROBOT_READY_LED_COUNTER_MAX)%ARRAY_SIZE];
			const uint8_t (*color2)[3] = &array[(((ARRAY_SIZE*counter)/ROBOT_READY_LED_COUNTER_MAX) + 1)%ARRAY_SIZE];

			const uint16_t fraq = (ARRAY_SIZE*counter)%ROBOT_READY_LED_COUNTER_MAX;

			const uint8_t r = (((uint16_t)(*color2)[0])*fraq + ((uint16_t)(*color1)[0])*(ROBOT_READY_LED_COUNTER_MAX - fraq))/ROBOT_READY_LED_COUNTER_MAX;
			const uint8_t g = (((uint16_t)(*color2)[1])*fraq + ((uint16_t)(*color1)[1])*(ROBOT_READY_LED_COUNTER_MAX - fraq))/ROBOT_READY_LED_COUNTER_MAX;
			const uint8_t b = (((uint16_t)(*color2)[2])*fraq + ((uint16_t)(*color1)[2])*(ROBOT_READY_LED_COUNTER_MAX - fraq))/ROBOT_READY_LED_COUNTER_MAX;

			stripe_set(r/4, g/4, b/4, 0);

			counter++;
			counter %=ROBOT_READY_LED_COUNTER_MAX;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 320-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 250;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MATRIX_COL3_Pin|MATRIX_COL4_Pin|MATRIX_ROW2_Pin|MATRIX_COL5_Pin
                          |MATRIX_ROW7_Pin|MATRIX_ROW5_Pin|MATRIX_ROW6_Pin|M3_A1_Pin
                          |Mx_EN_Pin|M1_A1_Pin|M1_A2_Pin|M1_B1_Pin
                          |M1_B2_Pin|MATRIX_COL0_Pin|MATRIX_COL1_Pin|MATRIX_COL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, MATRIX_ROW3_Pin|MATRIX_ROW4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MATRIX_COL7_Pin|M2_B2_Pin|M2_B1_Pin|MATRIX_ROW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MATRIX_COL6_Pin|M3_B1_Pin|M3_B2_Pin|M3_A2_Pin
                          |M2_A1_Pin|M2_A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MATRIX_ROW0_GPIO_Port, MATRIX_ROW0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MATRIX_COL3_Pin MATRIX_COL4_Pin MATRIX_ROW2_Pin MATRIX_COL5_Pin
                           MATRIX_ROW7_Pin MATRIX_ROW5_Pin MATRIX_ROW6_Pin M3_A1_Pin
                           Mx_EN_Pin M1_A1_Pin M1_A2_Pin M1_B1_Pin
                           M1_B2_Pin MATRIX_COL0_Pin MATRIX_COL1_Pin MATRIX_COL2_Pin */
  GPIO_InitStruct.Pin = MATRIX_COL3_Pin|MATRIX_COL4_Pin|MATRIX_ROW2_Pin|MATRIX_COL5_Pin
                          |MATRIX_ROW7_Pin|MATRIX_ROW5_Pin|MATRIX_ROW6_Pin|M3_A1_Pin
                          |Mx_EN_Pin|M1_A1_Pin|M1_A2_Pin|M1_B1_Pin
                          |M1_B2_Pin|MATRIX_COL0_Pin|MATRIX_COL1_Pin|MATRIX_COL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MATRIX_ROW3_Pin MATRIX_ROW4_Pin */
  GPIO_InitStruct.Pin = MATRIX_ROW3_Pin|MATRIX_ROW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : MATRIX_COL7_Pin M2_B2_Pin M2_B1_Pin MATRIX_ROW1_Pin */
  GPIO_InitStruct.Pin = MATRIX_COL7_Pin|M2_B2_Pin|M2_B1_Pin|MATRIX_ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MATRIX_COL6_Pin M3_B1_Pin M3_B2_Pin M3_A2_Pin
                           M2_A1_Pin M2_A2_Pin */
  GPIO_InitStruct.Pin = MATRIX_COL6_Pin|M3_B1_Pin|M3_B2_Pin|M3_A2_Pin
                          |M2_A1_Pin|M2_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MATRIX_ROW0_Pin */
  GPIO_InitStruct.Pin = MATRIX_ROW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MATRIX_ROW0_GPIO_Port, &GPIO_InitStruct);

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
