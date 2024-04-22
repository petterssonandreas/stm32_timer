/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "adc.h"
#include "stm32f0xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


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
/* USER CODE BEGIN Variables */

#define BUF_SIZE 64
static char rx_buffer[BUF_SIZE];
static volatile size_t rx_index = 0;

typedef struct {
  uint16_t msec;
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
} timer_time_t;

static volatile timer_time_t time = {
    .hour = 0,
    .min = 0,
    .sec = 0,
    .msec = 0,
};
static volatile timer_time_t on_time = {
    .hour = 6,
    .min = 0,
    .sec = 0,
    .msec = 0,
};
static volatile timer_time_t off_time = {
    .hour = 18,
    .min = 0,
    .sec = 0,
    .msec = 0,
};

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId consoleTaskHandle;
osSemaphoreId time_semHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void handle_rx_complete(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void start_console_task(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of time_sem */
  osSemaphoreDef(time_sem);
  time_semHandle = osSemaphoreCreate(osSemaphore(time_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
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
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  UNUSED(argument);
  uint32_t last_timestamp = HAL_GetTick();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  /* Infinite loop */
  // HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for (;;) {
    // HAL_GPIO_TogglePin(GPIOA, RELAY_ON_Pin);

    osSemaphoreWait(time_semHandle, 0);

    uint32_t new_timestamp = HAL_GetTick();
    uint32_t time_ms = time.msec + (time.sec * 1000) + (time.min * 60 * 1000) +
                       (time.hour * 60 * 60 * 1000);
    uint32_t time_diff_ms = new_timestamp - last_timestamp;
    time_ms = time_ms + time_diff_ms;
    last_timestamp = new_timestamp;
    time.msec = time_ms % 1000;
    time.sec = (time_ms / 1000) % 60;
    time.min = (time_ms / (60 * 1000)) % 60;
    time.hour = (time_ms / (60 * 60 * 1000)) % 24;

    uint32_t on_time_ms = on_time.msec + (on_time.sec * 1000) +
                          (on_time.min * 60 * 1000) +
                          (on_time.hour * 60 * 60 * 1000);
    uint32_t off_time_ms = off_time.msec + (off_time.sec * 1000) +
                           (off_time.min * 60 * 1000) +
                           (off_time.hour * 60 * 60 * 1000);

    // Check if passed on/off time
    if ((on_time_ms <= off_time_ms) &&
        (on_time_ms <= time_ms && time_ms < off_time_ms)) {
      // Same day on/off
      // Turn on
      HAL_GPIO_WritePin(GPIOA, RELAY_ON_Pin, GPIO_PIN_SET);
    } else if ((on_time_ms > off_time_ms) &&
               (on_time_ms <= time_ms || time_ms < off_time_ms)) {
      // Different days on/off
      // Turn on
      HAL_GPIO_WritePin(GPIOA, RELAY_ON_Pin, GPIO_PIN_SET);
    } else {
      // Turn off
      HAL_GPIO_WritePin(GPIOA, RELAY_ON_Pin, GPIO_PIN_RESET);
    }

    osSemaphoreRelease(time_semHandle);
    osDelay(5000);

    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

    // char tx[16] = "hi\n\r";
    // HAL_UART_Transmit(&huart2, (uint8_t *)tx, 4, 0);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_start_console_task */
/**
 * @brief Function implementing the consoleTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_console_task */
void start_console_task(void const *argument) {
  /* USER CODE BEGIN start_console_task */
  UNUSED(argument);
  HAL_StatusTypeDef ret;
  uint8_t rx_byte = 0;

  HAL_UART_Transmit(&huart2, (const uint8_t *)"****\nBoot\n", 10, 1000);

  /* Infinite loop */
  for (;;) {
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void handle_rx_complete(void) {
  char tx[16] = "";
  int r = 0;
  uint16_t raw;

  // if (strncmp(rx_buffer, "st ", 3) == 0) {}
  if (rx_buffer[0] == 'g' && rx_buffer[1] == 't') {
    osSemaphoreWait(time_semHandle, 0);
    snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", time.hour, time.min, time.sec);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
    osSemaphoreRelease(time_semHandle);
  } else if (rx_buffer[0] == 'g' && rx_buffer[1] == 'o' &&
             rx_buffer[2] == 'n') {
    osSemaphoreWait(time_semHandle, 0);
    snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", on_time.hour, on_time.min,
             on_time.sec);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
    osSemaphoreRelease(time_semHandle);
  } else if (rx_buffer[0] == 'g' && rx_buffer[1] == 'o' &&
             rx_buffer[2] == 'f' && rx_buffer[3] == 'f') {
    osSemaphoreWait(time_semHandle, 0);
    snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", off_time.hour, off_time.min,
             off_time.sec);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
    osSemaphoreRelease(time_semHandle);
  } else if (rx_buffer[0] == 's' && rx_buffer[1] == 't') {
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
  } else if (rx_buffer[0] == 's' && rx_buffer[1] == 'o' &&
             rx_buffer[2] == 'n') {
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
    snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", on_time.hour, on_time.min,
             on_time.sec);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
    osSemaphoreRelease(time_semHandle);
  } else if (rx_buffer[0] == 's' && rx_buffer[1] == 'o' &&
             rx_buffer[2] == 'f' && rx_buffer[3] == 'f') {
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
    snprintf(tx, sizeof(tx), "%02u:%02u:%02u\n", off_time.hour, off_time.min,
             off_time.sec);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx, 9, 1000);
    osSemaphoreRelease(time_semHandle);
  } else if (rx_buffer[0] == 'b' && rx_buffer[1] == 'a' &&
             rx_buffer[2] == 't') {

    // Get ADC value
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 3000);
    raw = HAL_ADC_GetValue(&hadc);

    // Convert to string and print

    // Take x(3300 / 4096) to get mV
    // Take x2 to get the battery voltage (have divider by with 100k and 100k,
    // to be inside of 3.3V)

    uint32_t val = ((uint32_t)raw * 3300 * 2) / 4095;

    snprintf(tx, sizeof(tx), "raw: %hu\n", raw);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx, strlen(tx), 1000);
    snprintf(tx, sizeof(tx), "vol: %lu mV\n", val);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx, strlen(tx), 1000);
    HAL_ADC_Stop(&hadc);
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

/* USER CODE END Application */
