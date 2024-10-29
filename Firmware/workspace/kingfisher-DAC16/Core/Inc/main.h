/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define DAC_nCS_Pin GPIO_PIN_4
#define DAC_nCS_GPIO_Port GPIOA
#define DAC_TOGGLE0_Pin GPIO_PIN_0
#define DAC_TOGGLE0_GPIO_Port GPIOB
#define DAC_TOGGLE1_Pin GPIO_PIN_1
#define DAC_TOGGLE1_GPIO_Port GPIOB
#define DAC_TOGGLE2_Pin GPIO_PIN_2
#define DAC_TOGGLE2_GPIO_Port GPIOB
#define DAC_nLDAC_Pin GPIO_PIN_5
#define DAC_nLDAC_GPIO_Port GPIOB
#define DAC_nRESET_Pin GPIO_PIN_6
#define DAC_nRESET_GPIO_Port GPIOB
#define DAC_nCLR_Pin GPIO_PIN_7
#define DAC_nCLR_GPIO_Port GPIOB
#define DAC_nALMOUT_Pin GPIO_PIN_8
#define DAC_nALMOUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
