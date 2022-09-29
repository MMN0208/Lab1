/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
typedef enum {
	NO_COLOR,
	RED,
	GREEN,
	YELLOW
} trafficLightColor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define		DELAY_TIME	 1000 // 1 second delay
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static trafficLightColor trafficLight1, trafficLight2;
static uint8_t secCounter = 0; // second counter for exercises below
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
void turnLedOn(GPIO_TypeDef* port, uint16_t pin);
void turnLedOff(GPIO_TypeDef* port, uint16_t pin);
void toggleLed(GPIO_TypeDef* port, uint16_t pin);
void exercise1(void);
void exercise2(void);
void trafficLight1State(trafficLightColor trafficLight1);
void trafficLight2State(trafficLightColor trafficLight2);
void exercise3(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void turnLedOn(GPIO_TypeDef* port, uint16_t pin) {
	HAL_GPIO_WritePin (port , pin, GPIO_PIN_RESET);
}

void turnLedOff(GPIO_TypeDef* port, uint16_t pin) {
	HAL_GPIO_WritePin (port , pin, GPIO_PIN_SET);
}

void toggleLed(GPIO_TypeDef* port, uint16_t pin) {
	HAL_GPIO_TogglePin(port, pin);
}

void exercise1(void) {
	if(secCounter == 0) { // initial state
		turnLedOn(LED_RED_GPIO_Port, LED_RED_Pin);
		turnLedOff(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	}
	else if(secCounter % 2 == 0) { // toggle both LEDs every 2s
		toggleLed(LED_RED_GPIO_Port, LED_RED_Pin);
		toggleLed(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	}
	secCounter++;
	if(secCounter >= 255) secCounter = 1; // prevent the 8-bit counter from overflowing
}

void trafficLight1State(trafficLightColor trafficLight1) {
	switch(trafficLight1) {
	case RED: {
		HAL_GPIO_WritePin (LED_RED_GPIO_Port , LED_RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (LED_YELLOW_GPIO_Port , LED_YELLOW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		break;
	}
	case YELLOW: {
		HAL_GPIO_WritePin (LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		break;
	}
	case GREEN: {
		HAL_GPIO_WritePin (LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		break;
	}
	case NO_COLOR: {
		HAL_GPIO_WritePin (LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		break;
	}
	}
}

void trafficLight2State(trafficLightColor trafficLight2) {
	switch(trafficLight2) {
	case RED: {
		HAL_GPIO_WritePin (LED_RED_2_GPIO_Port , LED_RED_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (LED_YELLOW_2_GPIO_Port , LED_YELLOW_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, GPIO_PIN_SET);
		break;
	}
	case YELLOW: {
		HAL_GPIO_WritePin (LED_RED_2_GPIO_Port, LED_RED_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_YELLOW_2_GPIO_Port, LED_YELLOW_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, GPIO_PIN_SET);
		break;
	}
	case GREEN: {
		HAL_GPIO_WritePin (LED_RED_2_GPIO_Port, LED_RED_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_YELLOW_2_GPIO_Port, LED_YELLOW_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, GPIO_PIN_RESET);
		break;
	}
	case NO_COLOR: {
		HAL_GPIO_WritePin (LED_RED_2_GPIO_Port, LED_RED_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_YELLOW_2_GPIO_Port, LED_YELLOW_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, GPIO_PIN_SET);
		break;
	}
	}
}

void exercise2(void) {
	switch(trafficLight1) {
	case NO_COLOR: {
		trafficLight1 = RED;
		break;
	}
	case RED: {
		trafficLight1State(trafficLight1);
		HAL_Delay(5000);
		trafficLight1 = GREEN;
		break;
	}
	case GREEN: {
		trafficLight1State(trafficLight1);
		HAL_Delay(3000);
		trafficLight1 = YELLOW;
		break;
	}
	case YELLOW: {
		trafficLight1State(trafficLight1);
		HAL_Delay(2000);
		trafficLight1 = RED;
		break;
	}
	}
}

void exercise3(void) {
	switch(trafficLight1) {
	case NO_COLOR: {
		trafficLight1 = RED;
		trafficLight2 = GREEN;
		break;
	}
	case RED: {
		trafficLight1State(trafficLight1);
		trafficLight2State(trafficLight2);
		HAL_Delay(3000);
		trafficLight2 = YELLOW;
		trafficLight2State(trafficLight2);
		HAL_Delay(2000);
		trafficLight1 = GREEN;
		break;
	}
	case GREEN: {
		trafficLight1State(trafficLight1);
		trafficLight2 = RED;
		trafficLight2State(trafficLight2);
		HAL_Delay(3000);
		trafficLight1 = YELLOW;
		break;

	}
	case YELLOW: {
		trafficLight1State(trafficLight1);
		trafficLight2State(trafficLight2);
		HAL_Delay(2000);
		trafficLight1 = RED;
		trafficLight2 = GREEN;
		break;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* Exercise 1 */
	exercise1();
	HAL_Delay(DELAY_TIME);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_RED_2_Pin
                          |LED_YELLOW_2_Pin|LED_GREEN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin LED_GREEN_Pin LED_RED_2_Pin
                           LED_YELLOW_2_Pin LED_GREEN_2_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_RED_2_Pin
                          |LED_YELLOW_2_Pin|LED_GREEN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
