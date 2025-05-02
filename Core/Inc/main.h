/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
#define PWM_EXP1_Pin GPIO_PIN_0
#define PWM_EXP1_GPIO_Port GPIOC
#define PWM_EXP2_Pin GPIO_PIN_13
#define PWM_EXP2_GPIO_Port GPIOC
#define EXP_START_Pin GPIO_PIN_3
#define EXP_START_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define LED_SEQ_Pin GPIO_PIN_1
#define LED_SEQ_GPIO_Port GPIOB
#define LAUNCH_SIG_Pin GPIO_PIN_2
#define LAUNCH_SIG_GPIO_Port GPIOB
#define LAUNCH_SIG_EXTI_IRQn EXTI2_3_IRQn
#define RLD_EXP_Pin GPIO_PIN_15
#define RLD_EXP_GPIO_Port GPIOA
#define RLD_EXP_EXTI_IRQn EXTI4_15_IRQn
#define RLD_PARA_Pin GPIO_PIN_3
#define RLD_PARA_GPIO_Port GPIOB
#define RLD_PARA_EXTI_IRQn EXTI2_3_IRQn
#define PWM_PARACH_Pin GPIO_PIN_9
#define PWM_PARACH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
