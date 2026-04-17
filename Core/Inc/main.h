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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
    MODE_TASK2_GENERATOR = 0,
    MODE_TASK3_GENERATOR,
    MODE_TASK4_FILTER,
    MODE_ADVANCED_MODELING
} SystemMode_t;
extern SystemMode_t CurrentMode;

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
#define KEY_1_Pin GPIO_PIN_1
#define KEY_1_GPIO_Port GPIOC
#define KEY_1_EXTI_IRQn EXTI1_IRQn
#define KEY_2_Pin GPIO_PIN_2
#define KEY_2_GPIO_Port GPIOC
#define KEY_2_EXTI_IRQn EXTI2_IRQn
#define AUTO_KEY_Pin GPIO_PIN_3
#define AUTO_KEY_GPIO_Port GPIOC
#define AUTO_KEY_EXTI_IRQn EXTI3_IRQn
#define ZERO_CROSS_Pin GPIO_PIN_0
#define ZERO_CROSS_GPIO_Port GPIOA
#define ZERO_CROSS_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
