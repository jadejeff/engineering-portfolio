/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32u5xx_hal.h"

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
#define air_Pin GPIO_PIN_2
#define air_GPIO_Port GPIOC
#define R_Pin GPIO_PIN_3
#define R_GPIO_Port GPIOC
#define PWR1_Pin GPIO_PIN_8
#define PWR1_GPIO_Port GPIOB
#define RB12_Pin GPIO_PIN_12
#define RB12_GPIO_Port GPIOB
#define RB13_Pin GPIO_PIN_13
#define RB13_GPIO_Port GPIOB
#define PWR2_Pin GPIO_PIN_9
#define PWR2_GPIO_Port GPIOB
#define PWR3_Pin GPIO_PIN_15
#define PWR3_GPIO_Port GPIOB
#define PWR1_BTN_Pin GPIO_PIN_6
#define PWR1_BTN_GPIO_Port GPIOC
#define PWR2_BTN_Pin GPIO_PIN_7
#define PWR2_BTN_GPIO_Port GPIOC
#define PWR3_BTN_Pin GPIO_PIN_8
#define PWR3_BTN_GPIO_Port GPIOC
#define LIGHT_SENSOR_Pin GPIO_PIN_9
#define LIGHT_SENSOR_GPIO_Port GPIOC
#define buzzer_Pin GPIO_PIN_5
#define buzzer_GPIO_Port GPIOC
#define light_Pin GPIO_PIN_3
#define light_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
