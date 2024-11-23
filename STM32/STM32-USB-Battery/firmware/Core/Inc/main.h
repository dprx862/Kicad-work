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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
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
#define HSE_IN_Pin GPIO_PIN_0
#define HSE_IN_GPIO_Port GPIOH
#define HSE_OUT_Pin GPIO_PIN_1
#define HSE_OUT_GPIO_Port GPIOH
#define STATUS_LED_Pin GPIO_PIN_3
#define STATUS_LED_GPIO_Port GPIOA
#define SUPPLY_MON_Pin GPIO_PIN_5
#define SUPPLY_MON_GPIO_Port GPIOA
#define BC_STAT_PG_Pin GPIO_PIN_7
#define BC_STAT_PG_GPIO_Port GPIOA
#define BC_STAT_2_Pin GPIO_PIN_5
#define BC_STAT_2_GPIO_Port GPIOC
#define BC_STAT_1_Pin GPIO_PIN_1
#define BC_STAT_1_GPIO_Port GPIOB
#define BC_CE_Pin GPIO_PIN_10
#define BC_CE_GPIO_Port GPIOB
#define BC_SEL_Pin GPIO_PIN_12
#define BC_SEL_GPIO_Port GPIOB
#define BC_PROG2_Pin GPIO_PIN_14
#define BC_PROG2_GPIO_Port GPIOB
#define USB_D__Pin GPIO_PIN_11
#define USB_D__GPIO_Port GPIOA
#define USB_D_A12_Pin GPIO_PIN_12
#define USB_D_A12_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
