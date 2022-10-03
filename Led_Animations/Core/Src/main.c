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
	RED = 0x06,
	GREEN = 0x03,
	YELLOW = 0x05,
} trafficLightColor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Delay time
#define	DELAY_TIME			1000 // 1 second delay

// Toggle every period (in seconds)
#define	TOGGLE_PERIOD		2

// Traffic light configuration (in seconds)
#define	GREEN_LIGHT_TIME	3
#define	YELLOW_LIGHT_TIME	2
#define	RED_LIGHT_TIME		GREEN_LIGHT_TIME + YELLOW_LIGHT_TIME

// Seven segment
#define SEVEN_SEG_VALUES	10

// Counter max value
#define COUNTER_MAX			255 // 8-bit counter
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t secCounter = 0; // second counter for exercises below

// Two traffic lights' state for 4-way configuration

const uint8_t sevenSegValue[SEVEN_SEG_VALUES] = {
	0x40, // 0
	0x79, // 1
	0x24, // 2
	0x30, // 3
	0x19, // 4
	0x12, // 5
	0x02, // 6
	0x78, // 7
	0x00, // 8
	0x10, // 9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void turnLedOn(GPIO_TypeDef* port, uint16_t pin);
void turnLedOff(GPIO_TypeDef* port, uint16_t pin);
void toggleLed(GPIO_TypeDef* port, uint16_t pin);
void display7SEG(int num);
void exercise1(void);
void exercise2(void);
void trafficLight1Operation(trafficLightColor trafficLight1);
void trafficLight2Operation(trafficLightColor trafficLight2);
void exercise3(void);
void exercise4(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void turnLedOn(GPIO_TypeDef* port, uint16_t pin) {
	HAL_GPIO_WritePin (port , pin, GPIO_PIN_RESET);
}

void turnLedOff(GPIO_TypeDef* port, uint16_t pin) {
	HAL_GPIO_WritePin (port , pin, GPIO_PIN_SET);
}

void display7SEG(int num) {
	uint8_t index = 0;
	HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, (sevenSegValue[num] >> index++) & 0x01);
	HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, (sevenSegValue[num] >> index++) & 0x01);
	HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, (sevenSegValue[num] >> index++) & 0x01);
	HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, (sevenSegValue[num] >> index++) & 0x01);
	HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, (sevenSegValue[num] >> index++) & 0x01);
	HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, (sevenSegValue[num] >> index++) & 0x01);
	HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, (sevenSegValue[num] >> index) & 0x01);
}

void exercise1(void) {
	if(secCounter == 0) { // initial state
		turnLedOn(LED_RED_GPIO_Port, LED_RED_Pin);
		turnLedOff(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	}
	else if(secCounter % TOGGLE_PERIOD == 0) { // toggle both LEDs every period
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	}
	secCounter++;
	if(secCounter >= COUNTER_MAX) secCounter = 1; // prevent the 8-bit counter from overflowing
}

void trafficLight1Operation(trafficLightColor trafficLight1) {
	uint8_t index = 0;
	HAL_GPIO_WritePin(LED_RED_GPIO_Port , LED_RED_Pin, (trafficLight1 >> index++) & 0x01);
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port , LED_YELLOW_Pin, (trafficLight1 >> index++) & 0x01);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, (trafficLight1 >> index) & 0x01);
}

void trafficLight2Operation(trafficLightColor trafficLight2) {
	uint8_t index = 0;
	HAL_GPIO_WritePin(LED_RED_2_GPIO_Port , LED_RED_2_Pin, (trafficLight2 >> index++) & 0x01);
	HAL_GPIO_WritePin(LED_YELLOW_2_GPIO_Port , LED_YELLOW_2_Pin, (trafficLight2 >> index++) & 0x01);
	HAL_GPIO_WritePin(LED_GREEN_2_GPIO_Port, LED_GREEN_2_Pin, (trafficLight2 >> index) & 0x01);
}

void exercise2(void) {
	if(secCounter < RED_LIGHT_TIME) {
		trafficLight1Operation(RED);
	}
	else if(secCounter < RED_LIGHT_TIME + GREEN_LIGHT_TIME) {
		trafficLight1Operation(GREEN);
	}
	else {
		trafficLight1Operation(YELLOW);
	}
	secCounter = (secCounter + 1) % (RED_LIGHT_TIME + RED_LIGHT_TIME); // RED + GRN + YEL = RED + RED
}

void exercise3(void) {
	if(secCounter < RED_LIGHT_TIME) {
		trafficLight1Operation(RED);
		if(secCounter < GREEN_LIGHT_TIME) {
			trafficLight2Operation(GREEN);
		}
		else {
			trafficLight2Operation(YELLOW);
		}
	}
	else if(secCounter < RED_LIGHT_TIME + GREEN_LIGHT_TIME) {
		trafficLight1Operation(GREEN);
		trafficLight2Operation(RED);
	}
	else {
		trafficLight1Operation(YELLOW);
		trafficLight2Operation(RED);
	}
	secCounter = (secCounter + 1) % (RED_LIGHT_TIME + RED_LIGHT_TIME);
}

void exercise4(void) {
	if(secCounter >= 10) secCounter = 0;
	display7SEG(secCounter++);
}

void exercise5(void) {
	if(secCounter < RED_LIGHT_TIME) {
		trafficLight1Operation(RED);
		if(secCounter < GREEN_LIGHT_TIME) {
			trafficLight2Operation(GREEN);
		}
		else {
			trafficLight2Operation(YELLOW);
		}
		display7SEG(RED_LIGHT_TIME - secCounter);
	}
	else if(secCounter < RED_LIGHT_TIME + GREEN_LIGHT_TIME) {
		trafficLight1Operation(GREEN);
		trafficLight2Operation(RED);
		display7SEG(RED_LIGHT_TIME + GREEN_LIGHT_TIME - secCounter);
	}
	else {
		trafficLight1Operation(YELLOW);
		trafficLight2Operation(RED);
		display7SEG(RED_LIGHT_TIME + RED_LIGHT_TIME - secCounter);
	}
	secCounter = (secCounter + 1) % (RED_LIGHT_TIME + RED_LIGHT_TIME);
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
	/* Exercise 4 */
	exercise5();
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_RED_2_Pin
                          |LED_YELLOW_2_Pin|LED_GREEN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin LED_GREEN_Pin LED_RED_2_Pin
                           LED_YELLOW_2_Pin LED_GREEN_2_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_RED_2_Pin
                          |LED_YELLOW_2_Pin|LED_GREEN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_C_Pin SEG_D_Pin
                           SEG_E_Pin SEG_F_Pin SEG_G_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
