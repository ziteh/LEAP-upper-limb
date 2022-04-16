/**
 * @file maxon_escon.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief A library for maxon ESCON motor controller.
 * @copyright MIT License, Copyright (c) 2022 ZiTe
 *
 */

#include "maxon_escon.h"

extern TIM_HandleTypeDef ESCON_PWM_Timer;

int ESCON_Init(void)
{
  ESCON_SetFunctionState(Disable);
  ESCON_SetDirection(CW);
  ESCON_SetPwmDutyCycle(0);
  HAL_TIM_PWM_Start(&ESCON_PWM_Timer, ESCON_PWM_Channel);

  if (ESCON_GetFunctionState() != Disable)
  {
    return 1; /* Error. */
  }

  if (ESCON_GetDirection() != CW)
  {
    return 1; /* Error. */
  }

  return ESCON_GetReadyState();
}

void ESCON_SetFunctionState(ESCON_FunctionalState_t state)
{
  if (state == Enable)
  {
    HAL_GPIO_WritePin(ESCON_Enable_GPIO_Port, ESCON_Enable_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(ESCON_Enable_GPIO_Port, ESCON_Enable_Pin, GPIO_PIN_RESET);
  }
}

void ESCON_SetDirection(ESCON_Direction_t direction)
{
  if (direction == CW)
  {
    HAL_GPIO_WritePin(ESCON_Direction_GPIO_Port, ESCON_Direction_Pin, GPIO_PIN_RESET);
  }
  else if (direction == CCW)
  {
    HAL_GPIO_WritePin(ESCON_Direction_GPIO_Port, ESCON_Direction_Pin, GPIO_PIN_SET);
  }
}

void ESCON_SetPwmDutyCycle(float duty_cycle)
{
  /* Limit. */
  if (duty_cycle > 100)
  {
    duty_cycle = 100;
  }
  else if (duty_cycle < 0)
  {
    duty_cycle = 0;
  }

  ESCON_PWM_TIM->ESCON_PWM_CCR = (ESCON_PWM_Timer.Init.Period * (duty_cycle / 100.0) + 1);
}

int ESCON_GetReadyState(void)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(ESCON_Ready_GPIO_Port, ESCON_Ready_Pin);
  if (state == GPIO_PIN_RESET)
  {
    return 0; /* Ready. */
  }
  else
  {
    return 1; /* Fault. */
  }
}

ESCON_FunctionalState_t ESCON_GetFunctionState(void)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(ESCON_Enable_GPIO_Port, ESCON_Enable_Pin);
  if (state != GPIO_PIN_SET)
  {
    return Disable;
  }
  else
  {
    return Enable;
  }
}

ESCON_Direction_t ESCON_GetDirection(void)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(ESCON_Direction_GPIO_Port, ESCON_Direction_Pin);
  if (state == GPIO_PIN_SET)
  {
    return CCW;
  }
  else
  {
    return CW;
  }
}