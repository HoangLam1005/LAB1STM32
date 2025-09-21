/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RED_A_Pin GPIO_PIN_2
#define RED_A_GPIO_Port GPIOA
#define YELLOW_A_Pin GPIO_PIN_3
#define YELLOW_A_GPIO_Port GPIOA
#define GREEN_A_Pin GPIO_PIN_4
#define GREEN_A_GPIO_Port GPIOA
#define RED_B_Pin GPIO_PIN_5
#define RED_B_GPIO_Port GPIOA
#define YELLOW_B_Pin GPIO_PIN_6
#define YELLOW_B_GPIO_Port GPIOA
#define GREEN_B_Pin GPIO_PIN_7
#define GREEN_B_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_0
#define A2_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_1
#define B2_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_2
#define C2_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_8
#define A1_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_9
#define B1_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_10
#define C1_GPIO_Port GPIOA
#define D1_Pin GPIO_PIN_11
#define D1_GPIO_Port GPIOA
#define E1_Pin GPIO_PIN_12
#define E1_GPIO_Port GPIOA
#define F1_Pin GPIO_PIN_13
#define F1_GPIO_Port GPIOA
#define G1_Pin GPIO_PIN_14
#define G1_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_3
#define D2_GPIO_Port GPIOB
#define E2_Pin GPIO_PIN_4
#define E2_GPIO_Port GPIOB
#define F2_Pin GPIO_PIN_5
#define F2_GPIO_Port GPIOB
#define G2_Pin GPIO_PIN_6
#define G2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
