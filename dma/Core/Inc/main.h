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
#include "stm32f0xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BTN_KNOB1_Pin GPIO_PIN_4
#define BTN_KNOB1_GPIO_Port GPIOA
#define BTN_KNOB1_EXTI_IRQn EXTI4_15_IRQn
#define BTN_KNOB_VS_Pin GPIO_PIN_5
#define BTN_KNOB_VS_GPIO_Port GPIOA
#define BTN_KNOB_VS_EXTI_IRQn EXTI4_15_IRQn
#define KNOB_VS_A_Pin GPIO_PIN_6
#define KNOB_VS_A_GPIO_Port GPIOA
#define KNOB_VS_B_Pin GPIO_PIN_7
#define KNOB_VS_B_GPIO_Port GPIOA
#define BTN_KNOB_ALT_Pin GPIO_PIN_0
#define BTN_KNOB_ALT_GPIO_Port GPIOB
#define BTN_KNOB_ALT_EXTI_IRQn EXTI0_1_IRQn
#define KNOB_ALT_A_Pin GPIO_PIN_8
#define KNOB_ALT_A_GPIO_Port GPIOA
#define KNOB_ALT_B_Pin GPIO_PIN_9
#define KNOB_ALT_B_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
