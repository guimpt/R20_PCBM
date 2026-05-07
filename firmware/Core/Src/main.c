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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include "mct8316.h"
#include "pid.h"
#include "tusb.h"
#include "cli.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VALID_HALL_STATES ((1 << 1) | (1 << 3) | (1 << 2) | (1 << 6) | (1 << 4) | (1 << 5))
#define ALPHA_HALL_SPEED	0.2f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
MCT8316 mct8316;
volatile int32_t hall_count = 0;
volatile int32_t last_hall_count = 0;
volatile float hall_speed = 0;
volatile uint8_t last_hall_state = 0;
volatile uint32_t last_tick = 0;

PID posPID;
PID velPID;
ctrl_mode_t ctrl_mode = MODE_POSITION;
volatile int32_t vel_prev_count = 0;
volatile float vel_filtered = 0;
int32_t action_k;
int32_t action_km1;
uint16_t timer_counter = 0;
uint8_t flag_100hz = RESET;
uint8_t flag_10hz = RESET;
uint8_t flag_1hz = RESET;
uint8_t enable_flag = SET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void USB_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static inline uint8_t is_valid_hall_state(uint8_t state);
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
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	/* Initialize MCT8316ZR */
	mct8316.hspi=&hspi1;
	MCT8316_Init(&mct8316);

	/* Initialize PWM generation */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	/* Initialize PID timer */
	HAL_TIM_Base_Start_IT(&htim3);
	//	TIM2->CCR4 = 50;
	TIM2->CCR4 = 0;
	MCT8316_SetDirection(&mct8316,1);

	posPID.Umax = 50;
	posPID.Umin = -50;
	posPID.kd = 0.001;
	posPID.kp = 1;
	posPID.ki = 0;
	posPID.loop_freq = 1000; //[Hz]
	posPID.w_cutoff = 5; //[Hz]
	PID_Initialize(&posPID);

	velPID.Umax = 50;
	velPID.Umin = -50;
	velPID.kd = 0.001;
	velPID.kp = 0.1;
	velPID.ki = 2;
	velPID.loop_freq = 100; //[Hz]
	velPID.w_cutoff = 5; //[Hz]
	PID_Initialize(&velPID);

	posPID.x_k = 0;

	USB_Init();
	CLI_Init();


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		tud_task();
		CLI_Process();
		if(ctrl_mode == MODE_POSITION && posPID.update_flag && enable_flag){
			posPID.update_flag = RESET;
			posPID.h_k = hall_count * posPID.multiplier;
			PID_Update(&posPID);
			action_k = posPID.y_k / ((int32_t)posPID.multiplier);
			TIM2->CCR4 = abs(action_k)+18;
		}
		if(ctrl_mode == MODE_VELOCITY && velPID.update_flag && enable_flag){
			velPID.update_flag = RESET;
			int32_t delta = hall_count - vel_prev_count;
			vel_prev_count = hall_count;
			float vel_raw = (float)delta * 100.0f;
			vel_filtered = 0.3f * vel_raw + 0.7f * vel_filtered;
			velPID.h_k = (int32_t)(vel_filtered * (float)velPID.multiplier);
			PID_Update(&velPID);
			action_k = velPID.y_k / ((int32_t)velPID.multiplier);
			TIM2->CCR4 = abs(action_k)+18;
		}
		if(flag_100hz && enable_flag){
			flag_100hz = RESET;
			if(__SIGN(action_k) != __SIGN(action_km1)){
				if(__SIGN(action_k) == 1) MCT8316_SetDirection(&mct8316,1);
				else if(__SIGN(action_k) == -1) MCT8316_SetDirection(&mct8316,0);
			}
			action_km1 = action_k;
		}
		if(flag_10hz){
			flag_10hz = RESET;
			hall_speed = ALPHA_HALL_SPEED*(((float)(hall_count - last_hall_count)) * 10.f / HALL_COUNTS_PER_REV) +
					(1.f - ALPHA_HALL_SPEED) * hall_speed;
			last_hall_count = hall_count;
			MCT8316_UpdateStatus(&mct8316); // TODO check errors and clear if needed
		}
		if(flag_1hz){
			flag_1hz = RESET;
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 239;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 48000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void USB_Init(void)
{
	__HAL_RCC_USB_CLK_ENABLE();
	HAL_NVIC_SetPriority(USB_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(USB_IRQn);

	const tusb_rhport_init_t rh_init = {
		.role = TUSB_ROLE_DEVICE,
		.speed = TUSB_SPEED_FULL
	};
	tud_rhport_init(BOARD_TUD_RHPORT, &rh_init);
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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : HALL_A_Pin HALL_B_Pin HALL_C_Pin */
	GPIO_InitStruct.Pin = HALL_A_Pin|HALL_B_Pin|HALL_C_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Function to check if a Hall state is valid using bitmasking
static inline uint8_t is_valid_hall_state(uint8_t state)
{
	return (VALID_HALL_STATES & (1 << state)) != 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t gpio_read = HALL_A_GPIO_Port->IDR;
	uint8_t hall_state;

	// Read the current Hall state
	hall_state = (__HALL_READ(gpio_read, HALL_A_Pin) << 2) |
			(__HALL_READ(gpio_read, HALL_B_Pin) << 1) |
			(__HALL_READ(gpio_read, HALL_C_Pin) << 0);

	// Ignore noise: process only if it's a valid Hall state
	if (!is_valid_hall_state(hall_state))
	{
		return;  // Skip invalid states (caused by noise)
	}

	// Determine direction based on the Hall sequence
	if (((last_hall_state == 0b001) && (hall_state == 0b011)) ||
			((last_hall_state == 0b011) && (hall_state == 0b010)) ||
			((last_hall_state == 0b010) && (hall_state == 0b110)) ||
			((last_hall_state == 0b110) && (hall_state == 0b100)) ||
			((last_hall_state == 0b100) && (hall_state == 0b101)) ||
			((last_hall_state == 0b101) && (hall_state == 0b001)))
	{
		hall_count++;
	}
	else if (((last_hall_state == 0b001) && (hall_state == 0b101)) ||
			((last_hall_state == 0b101) && (hall_state == 0b100)) ||
			((last_hall_state == 0b100) && (hall_state == 0b110)) ||
			((last_hall_state == 0b110) && (hall_state == 0b010)) ||
			((last_hall_state == 0b010) && (hall_state == 0b011)) ||
			((last_hall_state == 0b011) && (hall_state == 0b001)))
	{
		hall_count--;
	}

	// Update last state
	last_hall_state = hall_state;

}


/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
  posPID.update_flag = SET;
  CLI_Tick();
  if(++timer_counter >= 1000) {
	  timer_counter = 0;
	  flag_1hz = SET;
  }
  if((timer_counter%10) == 0) { // 100Hz
	  flag_100hz = SET;
	  velPID.update_flag = SET;
	  if((timer_counter%100) == 0) flag_10hz = SET; // 10Hz
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
