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
#define One_Pin GPIO_PIN_4
#define One_GPIO_Port GPIOA
#define Two_Pin GPIO_PIN_5
#define Two_GPIO_Port GPIOA
#define Three_Pin GPIO_PIN_6
#define Three_GPIO_Port GPIOA
#define Four_Pin GPIO_PIN_7
#define Four_GPIO_Port GPIOA
#define Five_Pin GPIO_PIN_8
#define Five_GPIO_Port GPIOA
#define Six_Pin GPIO_PIN_9
#define Six_GPIO_Port GPIOA
#define Seven_Pin GPIO_PIN_10
#define Seven_GPIO_Port GPIOA
#define Eight_Pin GPIO_PIN_11
#define Eight_GPIO_Port GPIOA
#define Nine_Pin GPIO_PIN_12
#define Nine_GPIO_Port GPIOA
#define Ten_Pin GPIO_PIN_13
#define Ten_GPIO_Port GPIOA
#define Eleven_Pin GPIO_PIN_14
#define Eleven_GPIO_Port GPIOA
#define Zero_Pin GPIO_PIN_15
#define Zero_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
