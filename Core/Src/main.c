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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_hal_uart.h"
#include <string.h>
#include <stdio.h>

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

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId consoleTaskHandle;
osSemaphoreId time_semHandle;
/* USER CODE BEGIN PV */

typedef struct {
	uint16_t msec;
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
} time_t;

static volatile time_t time = {
	.hour = 0,
	.min = 0,
	.sec = 0,
	.msec = 0,
};
static volatile time_t on_time = {
	.hour = 6,
	.min = 0,
	.sec = 0,
	.msec = 0,
};
static volatile time_t off_time = {
	.hour = 18,
	.min = 0,
	.sec = 0,
	.msec = 0,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void start_console_task(void const * argument);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of time_sem */
  osSemaphoreDef(time_sem);
  time_semHandle = osSemaphoreCreate(osSemaphore(time_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreRelease(time_semHandle);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of consoleTask */
  osThreadDef(consoleTask, start_console_task, osPriorityLow, 0, 256);
  consoleTaskHandle = osThreadCreate(osThread(consoleTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|RELAY_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin RELAY_ON_Pin */
  GPIO_InitStruct.Pin = LED_Pin|RELAY_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#define BUF_SIZE 64
static char rx_buffer[BUF_SIZE];
static volatile size_t rx_index = 0;
// static const char resp[8] = "12 34\n";

void handle_rx_complete(void)
{
	char tx[16] = "";
	int r = 0;

	//if (strncmp(rx_buffer, "st ", 3) == 0) {}
	if (rx_buffer[0] == 'g' && rx_buffer[1] == 't') {
		osSemaphoreWait(time_semHandle, 0);
		snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", time.hour, time.min, time.sec);
		HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
		osSemaphoreRelease(time_semHandle);
	}
	else if (rx_buffer[0] == 'g' && rx_buffer[1] == 'o' && rx_buffer[2] == 'n') {
		osSemaphoreWait(time_semHandle, 0);
		snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", on_time.hour, on_time.min, on_time.sec);
		HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
		osSemaphoreRelease(time_semHandle);
	}
	else if (rx_buffer[0] == 'g' && rx_buffer[1] == 'o' && rx_buffer[2] == 'f' && rx_buffer[3] == 'f') {
		osSemaphoreWait(time_semHandle, 0);
		snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", off_time.hour, off_time.min, off_time.sec);
		HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
		osSemaphoreRelease(time_semHandle);
	}
	else if (rx_buffer[0] == 's' && rx_buffer[1] == 't') {
		unsigned int hours, mins, secs;
		r = sscanf(rx_buffer, "st %u:%u:%u", &hours, &mins, &secs);
		if (r != 3 || hours > 23 || mins > 59 || secs > 59) {
			// Bad
			snprintf(tx, sizeof(tx), "Bad time\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
			return;
		}

		osSemaphoreWait(time_semHandle, 0);
		time.hour = hours;
		time.min = mins;
		time.sec = secs;
		time.msec = 0;
		snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", time.hour, time.min, time.sec);
		HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
		osSemaphoreRelease(time_semHandle);
	}
	else if (rx_buffer[0] == 's' && rx_buffer[1] == 'o' && rx_buffer[2] == 'n') {
		unsigned int hours, mins, secs;
		r = sscanf(rx_buffer, "son %u:%u:%u", &hours, &mins, &secs);
		if (r != 3 || hours > 23 || mins > 59 || secs > 59) {
			// Bad
			snprintf(tx, sizeof(tx), "Bad time\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
			return;
		}

		osSemaphoreWait(time_semHandle, 0);
		on_time.hour = hours;
		on_time.min = mins;
		on_time.sec = secs;
		on_time.msec = 0;
		snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", on_time.hour, on_time.min, on_time.sec);
		HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
		osSemaphoreRelease(time_semHandle);
	}
	else if (rx_buffer[0] == 's' && rx_buffer[1] == 'o' && rx_buffer[2] == 'f' && rx_buffer[3] == 'f') {
		unsigned int hours, mins, secs;
		r = sscanf(rx_buffer, "soff %u:%u:%u", &hours, &mins, &secs);
		if (r != 3 || hours > 23 || mins > 59 || secs > 59) {
			// Bad
			snprintf(tx, sizeof(tx), "Bad time\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
			return;
		}

		osSemaphoreWait(time_semHandle, 0);
		off_time.hour = hours;
		off_time.min = mins;
		off_time.sec = secs;
		off_time.msec = 0;
		snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", off_time.hour, off_time.min, off_time.sec);
		HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
		osSemaphoreRelease(time_semHandle);
	}
	/*
	else if (rx_buffer[0]) {

		HAL_UART_Transmit(&huart2, (uint8_t *)"gt\n", 3, 1000);
		HAL_UART_Transmit(&huart2, (uint8_t *)"st hh:mm:ss\n", 12, 1000);
		HAL_UART_Transmit(&huart2, (uint8_t *)"gon\n", 4, 1000);
		HAL_UART_Transmit(&huart2, (uint8_t *)"goff\n", 5, 1000);
		HAL_UART_Transmit(&huart2, (uint8_t *)"son hh:mm:ss\n", 13, 1000);
		HAL_UART_Transmit(&huart2, (uint8_t *)"soff hh:mm:ss\n", 14, 1000);

	}
	*/

	HAL_UART_Transmit(&huart2, (uint8_t *)"> ", 2, 1000);
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t last_timestamp = HAL_GetTick();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  // HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
	// HAL_GPIO_TogglePin(GPIOA, RELAY_ON_Pin);

	osSemaphoreWait(time_semHandle, 0);

	uint32_t new_timestamp = HAL_GetTick();
	uint32_t time_ms = time.msec + (time.sec * 1000) + (time.min * 60 * 1000) + (time.hour * 60 * 60 * 1000);
	uint32_t time_diff_ms = new_timestamp - last_timestamp;
	time_ms = time_ms + time_diff_ms;
	last_timestamp = new_timestamp;
	time.msec = time_ms % 1000;
	time.sec = (time_ms / 1000) % 60;
	time.min = (time_ms / (60 * 1000)) % 60;
	time.hour = (time_ms / (60 * 60 * 1000)) % 24;

	uint32_t on_time_ms = on_time.msec + (on_time.sec * 1000) + (on_time.min * 60 * 1000) + (on_time.hour * 60 * 60 * 1000);
	uint32_t off_time_ms = off_time.msec + (off_time.sec * 1000) + (off_time.min * 60 * 1000) + (off_time.hour * 60 * 60 * 1000);

	// Check if passed on/off time
	if ((on_time_ms <= off_time_ms) && (on_time_ms <= time_ms && time_ms < off_time_ms)) {
		// Same day on/off
		// Turn on
		HAL_GPIO_WritePin(GPIOA, RELAY_ON_Pin, GPIO_PIN_SET);
	}
	else if ((on_time_ms > off_time_ms) && (on_time_ms <= time_ms || time_ms < off_time_ms)) {
		// Different days on/off
		// Turn on
		HAL_GPIO_WritePin(GPIOA, RELAY_ON_Pin, GPIO_PIN_SET);
	}
	else {
		// Turn off
		HAL_GPIO_WritePin(GPIOA, RELAY_ON_Pin, GPIO_PIN_RESET);
	}

	osSemaphoreRelease(time_semHandle);
	osDelay(5000);

	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
	osDelay(1);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

	//char tx[16] = "hi\n\r";
	//HAL_UART_Transmit(&huart2, (uint8_t *)tx, 4, 0);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_console_task */
/**
* @brief Function implementing the consoleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_console_task */
void start_console_task(void const * argument)
{
  /* USER CODE BEGIN start_console_task */
  HAL_StatusTypeDef ret;
  uint8_t rx_byte = 0;

  HAL_UART_Transmit(&huart2, (const uint8_t *)"****\nBoot\n", 10, 1000);

  /* Infinite loop */
  for(;;)
  {
    ret = HAL_UART_Receive(&huart2, &rx_byte, 1, 1000);
    switch (ret) {
    case HAL_TIMEOUT:
    case HAL_ERROR:
    case HAL_BUSY:
    	break;
    case HAL_OK:
    	HAL_UART_Transmit(&huart2, &rx_byte, 1, 1000); // Echo
    	if (rx_byte == '\n' || rx_byte == '\r') {
    		rx_buffer[rx_index] = 0; // Terminate string
    		handle_rx_complete();
    		rx_index = 0;
    	} else {
    		rx_buffer[rx_index] = rx_byte;
    		rx_index++;
    		if (rx_index >= (BUF_SIZE - 1)) {
    			// Too long command...
    			// TODO: print error
    			rx_index = 0;
    		}
    	}
    }

    osDelay(1);
  }
  /* USER CODE END start_console_task */
}

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
