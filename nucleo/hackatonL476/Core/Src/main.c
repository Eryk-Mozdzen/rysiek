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
#include <math.h>

#include "protocol.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
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
	MOTOR_DIR_NO_MOTION = 0,
	MOTOR_DIR_FORWARD = -1,
	MOTOR_DIR_BACKWARD = 1,
} motor_dir_t;

typedef struct {
	signal_t a1;
	signal_t a2;
	signal_t b1;
	signal_t b2;
	int8_t state;
	motor_dir_t dir;
	uint8_t counter;
	uint8_t prescaler;
} motor_t;

static motor_t motors[MOTOR_NUM];
static signal_t motor_sw;

void drive_step(motor_t *motor) {
	if(motor->dir==MOTOR_DIR_NO_MOTION) {
		return;
	}

	motor->counter++;
	if(motor->counter!=(motor->prescaler+1)) {
		return;
	}
	motor->counter %=(motor->prescaler+1);

	switch(motor->state) {
	    case 0: {
	    	SIGNAL_SET(motor->b2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a1, GPIO_PIN_SET);
	    	motor->state +=motor->dir;
	    } break;
	    case 1: {
	    	SIGNAL_SET(motor->a1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b1, GPIO_PIN_SET);
	    	motor->state +=motor->dir;
	    } break;
	    case 2: {
	    	SIGNAL_SET(motor->b1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a2, GPIO_PIN_SET);
	    	motor->state +=motor->dir;
      } break;
	    case 3: {
	    	SIGNAL_SET(motor->a2, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->a1, GPIO_PIN_RESET);
	    	SIGNAL_SET(motor->b2, GPIO_PIN_SET);
	    	motor->state +=motor->dir;
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

void motor_vel(motor_t *motor, int8_t vel) {
	vel = (vel>100) ? 100 : (vel<-100) ? -100 : vel;

	if(vel>0) {
		motor->dir = MOTOR_DIR_FORWARD;
		motor->prescaler = (100 - vel)/10;
	} else if(vel<0) {
		motor->dir = MOTOR_DIR_BACKWARD;
		motor->prescaler = (100 + vel)/10;
	} else {
		motor->dir = MOTOR_DIR_NO_MOTION;
		motor->prescaler = 0;
	}
}

#define MATRIX_COLS 8
#define MATRIX_ROWS 8

typedef struct {
	signal_t cols[MATRIX_COLS];
	signal_t rows[MATRIX_ROWS];
	volatile uint8_t current_row;
  uint8_t bitmap[8][8];
} matrix_t;

uint8_t bitmap_empty[8][8] = {
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
};

uint8_t bitmap_full[8][8] = {
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
};

uint8_t bitmap_smile[8][8] = {
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 1, 1, 0, 0, 1, 1, 0},
	{0, 1, 1, 0, 0, 1, 1, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 1, 0, 0, 0, 0, 1, 0},
	{0, 1, 1, 0, 0, 1, 1, 0},
	{0, 0, 1, 1, 1, 1, 0, 0},
};

uint8_t bitmap_heart[8][8] = {
	{0, 1, 1, 0, 0, 1, 1, 0},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{0, 1, 1, 1, 1, 1, 1, 0},
	{0, 0, 1, 1, 1, 1, 0, 0},
	{0, 0, 0, 1, 1, 0, 0, 0},
};

uint8_t bitmap_konar[8][8] = {
	{0, 0, 1, 1, 1, 0, 0, 0},
	{0, 1, 1, 1, 0, 1, 1, 0},
	{1, 1, 1, 1, 1, 0, 1, 1},
	{1, 1, 0, 1, 1, 1, 0, 0},
	{1, 1, 1, 1, 0, 1, 0, 1},
	{0, 1, 0, 1, 0, 0, 1, 1},
	{0, 0, 0, 1, 0, 0, 0, 0},
	{0, 0, 0, 1, 0, 0, 0, 0},
};

void matrix_step(matrix_t *matrix) {
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
		SIGNAL_SET(matrix->cols[i], matrix->bitmap[MATRIX_COLS - i - 1][matrix->current_row]);
	}
}

static matrix_t matrix = {0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim==&htim7) {
		for(uint8_t i=0; i<MOTOR_NUM; i++) {
			drive_step(&motors[i]);
		}

		matrix_step(&matrix);
	}
}

#define ULTRA_NUM            2
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

/*static void logger(const char *format, ...) {
    va_list args;
    va_start(args, format);

	char buffer[256];
    const uint16_t len = vsprintf(buffer, format, args);

    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}*/

#define LOAD_CALIB_0KG   8488400.f
#define LOAD_CALIB_XKG   8433700.f
#define LOAD_CALIB_VAL   0.261
#define LOAD_TIMEOUT     10

typedef struct {
  signal_t dout;
  signal_t clk;
  int32_t data;
  float load;
} beam_t;

void beam_init(beam_t *beam) {
	//HAL_GPIO_WritePin(LOAD_RATE_GPIO_Port, LOAD_RATE_Pin, GPIO_PIN_RESET);

	SIGNAL_SET(beam->clk, GPIO_PIN_SET);
	HAL_Delay(10);
	SIGNAL_SET(beam->clk, GPIO_PIN_RESET);
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
		SIGNAL_SET(beam->clk, GPIO_PIN_SET);
		for(volatile uint32_t i=0; i<100; i++) {}
		SIGNAL_SET(beam->clk, GPIO_PIN_RESET);
		for(volatile uint32_t i=0; i<100; i++) {}

		beam->data <<=1;

		if(SIGNAL_GET(beam->dout) == GPIO_PIN_SET) {
			beam->data |=0x01;
		}
	}

	beam->data ^=0x800000;

	SIGNAL_SET(beam->clk, GPIO_PIN_SET);
	for(volatile uint32_t i=0; i<100; i++) {}
	SIGNAL_SET(beam->clk, GPIO_PIN_RESET);
	for(volatile uint32_t i=0; i<100; i++) {}

	beam->load = LOAD_CALIB_VAL*((float)(beam->data) - LOAD_CALIB_0KG)/(LOAD_CALIB_XKG - LOAD_CALIB_0KG);

	return true;
}

static beam_t beam = {0};

#define STRIPE_TIM   htim4
#define STRIPE_CHN_R TIM_CHANNEL_1
#define STRIPE_CHN_G TIM_CHANNEL_4
#define STRIPE_CHN_B TIM_CHANNEL_2
#define STRIPE_CHN_W TIM_CHANNEL_3

#define ARRAY_SIZE 12

const uint8_t array[ARRAY_SIZE][3] = {
  	{  0,   0, 255},
  	{127,   0, 255},
  	{255,   0, 255},
  	{255,   0, 127},
	{255,   0,   0},
	{255, 127,   0},
	{255, 255,   0},
	{127, 255,   0},
	{  0, 255,   0},
	{  0, 255, 127},
  	{  0, 255, 255},
  	{  0, 127, 255},
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

void servos_set(const float angle1, const float angle2) {
	const uint32_t compare1 = ((angle1/90.f) + 1.f)*1000;
	const uint32_t compare2 = ((angle2/90.f) + 1.f)*1000;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare2);
}

static protocol_t protocol = {0};
static protocol_frame_t frame = {0};

#define UART_BUFFER_SIZE	16

static uint8_t uart_buffer[UART_BUFFER_SIZE] = {0};
static volatile bool uart_ready = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart3) {
		uart_ready = true;
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
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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

  motor_sw = (signal_t){.port = SW_GPIO_Port, .pin = SW_Pin};

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

  ultras[0].echo = (signal_t){.port = U0_ECHO_GPIO_Port, .pin = U0_ECHO_Pin};
  ultras[1].echo = (signal_t){.port = U1_ECHO_GPIO_Port, .pin = U1_ECHO_Pin};

  beam.clk  = (signal_t){.port = BEAM_CLK_GPIO_Port, .pin = BEAM_CLK_Pin};
  beam.dout = (signal_t){.port = BEAM_DOUT_GPIO_Port, .pin = BEAM_DOUT_Pin};

  HAL_GPIO_WritePin(Mx_EN_GPIO_Port, Mx_EN_Pin, 1);
  HAL_TIM_Base_Start_IT(&htim7); // motors steps & matrix

  HAL_TIM_Base_Start(&htim1); // ultrasonic trigger
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim2); // servos
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_UART_Receive_DMA(&huart3, uart_buffer, UART_BUFFER_SIZE);

  beam_init(&beam);
  stripe_init();

  uint32_t servos_time = 0;

  uint32_t last_msg = HAL_GetTick();
  uint32_t last_color = HAL_GetTick();
  uint32_t last_color_inc = HAL_GetTick();
  uint8_t color_i = 0;

  bool first_time = true;
  uint32_t mass_time = 0;
  float current_mass = 0;

  if(beam_read(&beam)) {
	  current_mass = beam.load;
  }

  while(1) {
	  const uint32_t time = HAL_GetTick();

	  if(uart_ready) {
		  uart_ready = false;
		  for(uint8_t i=0; i<UART_BUFFER_SIZE; i++) {
			  if(protocol_consume(&protocol, uart_buffer[i])) {
				  memcpy(&frame, &protocol.frame, sizeof(frame));
				  last_msg = time;

				  if(frame.motor_left>0) {
					  frame.motor_left = 90;
				  }

				  if(frame.motor_left<0) {
					  frame.motor_left = -90;
				  }

				  if(frame.motor_right>0) {
					  frame.motor_right = 90;
				  }

				  if(frame.motor_right<0) {
					  frame.motor_right = -90;
				  }

				  motor_vel(&motors[1], frame.motor_left);
				  motor_vel(&motors[2], frame.motor_right);

				  switch(frame.matrix) {
					  case PROTOCOL_MATRIX_EMPTY: {
						  memcpy(matrix.bitmap, bitmap_empty, 64);
					  } break;
					  case PROTOCOL_MATRIX_FULL: {
						  memcpy(matrix.bitmap, bitmap_full, 64);
					  } break;
					  case PROTOCOL_MATRIX_HEART: {
						  memcpy(matrix.bitmap, bitmap_heart, 64);
					  } break;
					  case PROTOCOL_MATRIX_SMILE: {
						  memcpy(matrix.bitmap, bitmap_smile, 64);
					  } break;
					  case PROTOCOL_MATRIX_KONAR: {
						  memcpy(matrix.bitmap, bitmap_konar, 64);
					  } break;
				  }
			  }
		  }
	  }

	  if((time - last_msg)>=5000) {
		  last_msg = time;
		  HAL_UART_Receive_DMA(&huart3, uart_buffer, UART_BUFFER_SIZE);
		  motors[1].dir = MOTOR_DIR_NO_MOTION;
		  motors[2].dir = MOTOR_DIR_NO_MOTION;
	  }

	  if((time - mass_time)>=1000) {
		  mass_time = time;

		if(beam_read(&beam)) {
			const float delta = beam.load - current_mass;
			current_mass = beam.load;

			if(delta<-0.010f && !first_time) {
				memcpy(matrix.bitmap, bitmap_heart, 64);
				stripe_set(0, 255, 0, 0);
				servos_set(30, 0);
				motors[0].dir = MOTOR_DIR_NO_MOTION;
				motors[1].dir = MOTOR_DIR_NO_MOTION;
				motors[2].dir = MOTOR_DIR_NO_MOTION;
				HAL_Delay(3000);
				stripe_set(0, 0, 0, 0);
				memcpy(matrix.bitmap, bitmap_empty, 64);
			}

			first_time = false;
		}
	  }

	  if((time - last_color_inc)>=1000) {
		  last_color_inc = time;

		  color_i++;
	  }

	  if((time - last_color)>=10) {
		  const uint8_t *color1 = array[(color_i+0)%ARRAY_SIZE];
		  const uint8_t *color2 = array[(color_i+1)%ARRAY_SIZE];

		  const float frac = (time - last_color_inc)/1000.f;
		  const float r = color1[0]*(1 - frac) + color2[0]*frac;
		  const float g = color1[1]*(1 - frac) + color2[1]*frac;
		  const float b = color1[2]*(1 - frac) + color2[2]*frac;

		  stripe_set(r, g, b, 0);

		  last_color = time;
	  }

	  /*if(ultras[0].valid && ultras[1].valid) {
		  logger("left %10.3fm      right %10.3fm\n\r", ultras[0].dist, ultras[1].dist);
		  HAL_Delay(100);
	  }

	  if(beam_read(&beam)) {
		  logger("load = %10.3fkg raw = %lu\n\r", beam.load, beam.data);
		  HAL_Delay(100);
	  }*/

	  if((time - servos_time)>=60) {
		  servos_time = time;

		  const float angle = sinf(2.f*3.1415f*0.2f*0.001f*time)*30.f;

		  servos_set(angle, -angle + 60);
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, MATRIX_COL6_Pin|BEAM_CLK_Pin|M3_B1_Pin|M3_B2_Pin
                          |M3_A2_Pin|M2_A1_Pin|M2_A2_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : U0_ECHO_Pin U1_ECHO_Pin */
  GPIO_InitStruct.Pin = U0_ECHO_Pin|U1_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MATRIX_COL6_Pin BEAM_CLK_Pin M3_B1_Pin M3_B2_Pin
                           M3_A2_Pin M2_A1_Pin M2_A2_Pin */
  GPIO_InitStruct.Pin = MATRIX_COL6_Pin|BEAM_CLK_Pin|M3_B1_Pin|M3_B2_Pin
                          |M3_A2_Pin|M2_A1_Pin|M2_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BEAM_DOUT_Pin */
  GPIO_InitStruct.Pin = BEAM_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BEAM_DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MATRIX_ROW0_Pin */
  GPIO_InitStruct.Pin = MATRIX_ROW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MATRIX_ROW0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
