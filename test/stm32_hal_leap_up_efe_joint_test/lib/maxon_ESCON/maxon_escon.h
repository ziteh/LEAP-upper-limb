/**
 * @file maxon_escon.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief A library for maxon ESCON motor controller.
 * @copyright MIT License, Copyright (c) 2022 ZiTe
 * @remark Enable State: Low(0) --> Disable, High(1) --> Enable.
 *         Dircetion: Low(0) --> CW, High(1) --> CCW.
 *         Readye State: Low(0) --> Ready, High(1) --> Fault.
 */

#ifndef __MAXON_ESCON_H
#define __MAXON_ESCON_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#define ESCON_PWM_Timer htim3
#define ESCON_PWM_Channel TIM_CHANNEL_1
#define ESCON_PWM_TIM TIM3
#define ESCON_PWM_CCR CCR1
#define ESCON_PWM_Pin GPIO_PIN_4
#define ESCON_PWM_GPIO_Port GPIOB

#define ESCON_Enable_Pin GPIO_PIN_10
#define ESCON_Enable_GPIO_Port GPIOB

#define ESCON_Direction_Pin GPIO_PIN_8
#define ESCON_Direction_GPIO_Port GPIOA

#define ESCON_Ready_Pin GPIO_PIN_5
#define ESCON_Ready_GPIO_Port GPIOB

  typedef enum
  {
    CW = 0,
    CCW = 1
  } ESCON_Direction_t;

  typedef enum
  {
    Disable = 0,
    Enable = 1
  } ESCON_FunctionalState_t;

  /**
   * @brief ESCON Initialization, set dircetion to CW, set speed to 0, and disable the controller.
   *
   * @return 0 for Ready, 1 for Error.
   */
  int ESCON_Init(void);

  void ESCON_SetFunctionState(ESCON_FunctionalState_t state);
  void ESCON_SetDirection(ESCON_Direction_t direction);
  void ESCON_SetPwmDutyCycle(float duty_cycle);

  /**
   * @brief
   *
   * @return 0 for Ready, 1 for Fault.
   */
  int ESCON_GetReadyState(void);
  ESCON_FunctionalState_t ESCON_GetFunctionState(void);
  ESCON_Direction_t ESCON_GetDirection(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAXON_ESCON_H */