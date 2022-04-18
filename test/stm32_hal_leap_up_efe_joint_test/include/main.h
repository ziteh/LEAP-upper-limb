/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <leap_up_efe_joint.h>

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

  void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

/* D13. */
#define AS5047P_SCK_Pin GPIO_PIN_5
#define AS5047P_SCK_GPIO_Port GPIOA

/* D12. */
#define AS5047P_MISO_Pin GPIO_PIN_6
#define AS5047P_MISO_GPIO_Port GPIOA

/* D11. */
#define AS5047P_MOSI_Pin GPIO_PIN_7
#define AS5047P_MOSI_GPIO_Port GPIOA

/* D10. */
#define AS5047P_CS_Pin GPIO_PIN_6
#define AS5047P_CS_GPIO_Port GPIOB

/* D8. */
#define AS5047P_ENCODER_B_Pin GPIO_PIN_9
#define AS5047P_ENCODER_B_GPIO_Port GPIOA
#define AS5047P_ENCODER_B_EXTI_IRQn EXTI9_5_IRQn

/* D9. */
#define AS5047P_ENCODER_A_Pin GPIO_PIN_7
#define AS5047P_ENCODER_A_GPIO_Port GPIOC
#define AS5047P_ENCODER_A_EXTI_IRQn EXTI9_5_IRQn

#define ESCON_PWM_Pin GPIO_PIN_4
#define ESCON_PWM_GPIO_Port GPIOB
#define ESCON_Enable_Pin GPIO_PIN_10
#define ESCON_Enable_GPIO_Port GPIOB
#define ESCON_Direction_Pin GPIO_PIN_8
#define ESCON_Direction_GPIO_Port GPIOA
#define ESCON_Ready_Pin GPIO_PIN_5
#define ESCON_Ready_GPIO_Port GPIOB

#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
