/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define USER_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define UART_SFE_TX_Pin GPIO_PIN_0
#define UART_SFE_TX_GPIO_Port GPIOA
#define UART_SFE_RX_Pin GPIO_PIN_1
#define UART_SFE_RX_GPIO_Port GPIOA
#define UART_DEBUG_TX_Pin GPIO_PIN_2
#define UART_DEBUG_TX_GPIO_Port GPIOA
#define UART_DEBUG_RX_Pin GPIO_PIN_3
#define UART_DEBUG_RX_GPIO_Port GPIOA
#define USER_LED_Pin GPIO_PIN_5
#define USER_LED_GPIO_Port GPIOA
#define UART_SIE_RX_Pin GPIO_PIN_5
#define UART_SIE_RX_GPIO_Port GPIOC
#define UART_SIE_TX_Pin GPIO_PIN_10
#define UART_SIE_TX_GPIO_Port GPIOB
#define UART_TF_TX_Pin GPIO_PIN_9
#define UART_TF_TX_GPIO_Port GPIOA
#define UART_TF_RX_Pin GPIO_PIN_10
#define UART_TF_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define UART_EFE_TX_Pin GPIO_PIN_12
#define UART_EFE_TX_GPIO_Port GPIOC
#define UART_EFE_RX_Pin GPIO_PIN_2
#define UART_EFE_RX_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
