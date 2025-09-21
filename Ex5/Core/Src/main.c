/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t segmentBinary[10] = {
	0b00111111, // 0
	0b00000110, // 1
	0b01011011, // 2
	0b01001111, // 3
	0b01100110, // 4
	0b01101101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01101111  // 9
};
void display7SEG1(int num) {
	uint8_t display = segmentBinary[num];

	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, (display & 0b00000001) ? RESET : SET);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, (display & 0b00000010) ? RESET : SET);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, (display & 0b00000100) ? RESET : SET);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, (display & 0b00001000) ? RESET : SET);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, (display & 0b00010000) ? RESET : SET);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, (display & 0b00100000) ? RESET : SET);
	HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, (display & 0b01000000) ? RESET : SET);
}

void display7SEG2(int num) {
	uint8_t display = segmentBinary[num];

	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, (display & 0b00000001) ? RESET : SET);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, (display & 0b00000010) ? RESET : SET);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, (display & 0b00000100) ? RESET : SET);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, (display & 0b00001000) ? RESET : SET);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, (display & 0b00010000) ? RESET : SET);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, (display & 0b00100000) ? RESET : SET);
	HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, (display & 0b01000000) ? RESET : SET);
}
void TrafficLightState(GPIO_PinState redA, GPIO_PinState yellowA, GPIO_PinState greenA,
					   GPIO_PinState redB, GPIO_PinState yellowB, GPIO_PinState greenB) {
	HAL_GPIO_WritePin(RED_A_GPIO_Port, RED_A_Pin, redA);
	HAL_GPIO_WritePin(YELLOW_A_GPIO_Port, YELLOW_A_Pin, yellowA);
	HAL_GPIO_WritePin(GREEN_A_GPIO_Port, GREEN_A_Pin, greenA);
	HAL_GPIO_WritePin(RED_B_GPIO_Port, RED_B_Pin, redB);
	HAL_GPIO_WritePin(YELLOW_B_GPIO_Port, YELLOW_B_Pin, yellowB);
	HAL_GPIO_WritePin(GREEN_B_GPIO_Port, GREEN_B_Pin, greenB);
}
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
  static int timer = 0;
  static int state = 0;
  while (1)
  {
	  switch (state) {
		  case 0:
			  TrafficLightState(SET, RESET, RESET, RESET, RESET, SET);
			  if (timer == 0) timer = 3;
			  display7SEG1(timer + 2);
			  display7SEG2(timer);
			  break;
		  case 1:
			  TrafficLightState(SET, RESET, RESET, RESET, SET, RESET);
			  if (timer == 0) timer = 2;
			  display7SEG1(timer);
			  display7SEG2(timer);
			  break;
		  case 2:
			  TrafficLightState(RESET, RESET, SET, SET, RESET, RESET);
			  if (timer == 0) timer = 3;
			  display7SEG1(timer);
			  display7SEG2(timer + 2);
			  break;
		  case 3:
			  TrafficLightState(RESET, SET, RESET, SET, RESET, RESET);
			  if (timer == 0) timer = 2;
			  display7SEG1(timer);
			  display7SEG2(timer);
			  break;
	  }
	  timer--;
	  if (timer <= 0) {
		  state++;
		  if (state > 3) state = 0;
	  }
	  HAL_Delay(1000);
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
  HAL_GPIO_WritePin(GPIOA, RED_A_Pin|YELLOW_A_Pin|GREEN_A_Pin|RED_B_Pin
                          |YELLOW_B_Pin|GREEN_B_Pin|A1_Pin|B1_Pin
                          |C1_Pin|D1_Pin|E1_Pin|F1_Pin
                          |G1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A2_Pin|B2_Pin|C2_Pin|D2_Pin
                          |E2_Pin|F2_Pin|G2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_A_Pin YELLOW_A_Pin GREEN_A_Pin RED_B_Pin
                           YELLOW_B_Pin GREEN_B_Pin A1_Pin B1_Pin
                           C1_Pin D1_Pin E1_Pin F1_Pin
                           G1_Pin */
  GPIO_InitStruct.Pin = RED_A_Pin|YELLOW_A_Pin|GREEN_A_Pin|RED_B_Pin
                          |YELLOW_B_Pin|GREEN_B_Pin|A1_Pin|B1_Pin
                          |C1_Pin|D1_Pin|E1_Pin|F1_Pin
                          |G1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A2_Pin B2_Pin C2_Pin D2_Pin
                           E2_Pin F2_Pin G2_Pin */
  GPIO_InitStruct.Pin = A2_Pin|B2_Pin|C2_Pin|D2_Pin
                          |E2_Pin|F2_Pin|G2_Pin;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
