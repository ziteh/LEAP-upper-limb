/**
 * @file	  pwm.cpp
 * @author	ZiTe (honmonoh@gmail.com)
 * @date    Sep 30, 2021
 */

#include "pwm.hpp"

PWM::PWM(GPIO_PortPinTypeDef portPin)
{
  GPIO gpio(portPin, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);

  TIM_TimeBaseStructInit(&_TIM_TimeBaseStructure);
  _TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  _TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_OCStructInit(&_TIM_OCInitStructure);
  _TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  _TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  _TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  this->init();
}

void PWM::Enable(void)
{
  if (_TIM_OCInitStructure.TIM_OutputState != TIM_OutputState_Enable)
  {
    _TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    this->init();
  }

  // TIMx peripheral Preload register on CCRx
  switch (_channel)
  {
  case CH1:
    TIM_OC1PreloadConfig(_timer, TIM_OCPreload_Enable);
    break;
  case CH2:
    TIM_OC2PreloadConfig(_timer, TIM_OCPreload_Enable);
    break;
  case CH3:
    TIM_OC3PreloadConfig(_timer, TIM_OCPreload_Enable);
    break;
  case CH4:
    TIM_OC4PreloadConfig(_timer, TIM_OCPreload_Enable);
    break;
  default:
    break;
  }
  TIM_ARRPreloadConfig(_timer, ENABLE); // TIMx peripheral Preload register on ARR
  TIM_Cmd(_timer, ENABLE);              // The specified TIM peripheral
}

void PWM::Disable(void)
{
  if (_TIM_OCInitStructure.TIM_OutputState != TIM_OutputState_Disable)
  {
    _TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    this->init();
  }

  // TIMx peripheral Preload register on CCRx
  switch (_channel)
  {
  case CH1:
    TIM_OC1PreloadConfig(_timer, TIM_OCPreload_Disable);
    break;
  case CH2:
    TIM_OC2PreloadConfig(_timer, TIM_OCPreload_Disable);
    break;
  case CH3:
    TIM_OC3PreloadConfig(_timer, TIM_OCPreload_Disable);
    break;
  case CH4:
    TIM_OC4PreloadConfig(_timer, TIM_OCPreload_Disable);
    break;
  default:
    break;
  }
  TIM_ARRPreloadConfig(_timer, DISABLE); // TIMx peripheral Preload register on ARR
  TIM_Cmd(_timer, DISABLE);              // The specified TIM peripheral
}

// FIXME Value of frequency error.
void PWM::setFrequency(uint16_t frequency)
{
  /**
   *  TIM_Period = ((System_Frequency / TIM_Prescaler) / PWM_Frequency) - 1
   *
   *  The Maximum of System_Frequency = 72MHz (STM32F103RB)
   */

  extern RCC_ClocksTypeDef RCC_Clocks;

  _TIM_TimeBaseStructure.TIM_Prescaler = 20; // !! or 100
  _TIM_TimeBaseStructure.TIM_Period = ((RCC_Clocks.SYSCLK_Frequency / _TIM_TimeBaseStructure.TIM_Prescaler) / frequency) - 1;

  this->init();
}

void PWM::setDutyCycle(uint16_t dutyCycle)
{
  if (_TIM_OCInitStructure.TIM_Pulse != this->convertDutyCycleToPulse(dutyCycle))
  {
    _TIM_OCInitStructure.TIM_Pulse = this->convertDutyCycleToPulse(dutyCycle);
    this->init();
  }
}
uint16_t PWM::getFrequency(void)
{
  /**
   * PWM_Frequency = (System_Frequency / TIM_Prescaler) / (TIM_Period + 1)
   *
   * The Maximum of System_Frequency = 72MHz (STM32F103RB)
   */

  extern RCC_ClocksTypeDef RCC_Clocks;

  return (RCC_Clocks.SYSCLK_Frequency / _TIM_TimeBaseStructure.TIM_Prescaler) / (_TIM_TimeBaseStructure.TIM_Period + 1);
}

uint16_t PWM::getDutyCycle(void)
{
  /**
   *  PWM_Duty Cycle % = (TIM_Pulse * 100%) / TIM_Period
   *
   *  TIM_Pulse = CCRx
   *  TIM_Period = ARR
   */

  switch (this->_channel)
  {
  case CH1:
    return ((_timer->CCR1) * 100.0) / (_timer->ARR);
    break;
  case CH2:
    return ((_timer->CCR2) * 100.0) / (_timer->ARR);
    break;
  case CH3:
    return ((_timer->CCR3) * 100.0) / (_timer->ARR);
    break;
  case CH4:
    return ((_timer->CCR4) * 100.0) / (_timer->ARR);
    break;
  default:
    break;
  }
}

void PWM::init(void)
{
  TIM_TimeBaseInit(_timer, &_TIM_TimeBaseStructure);
  switch (_channel)
  {
  case CH1:
    TIM_OC1Init(_timer, &_TIM_OCInitStructure);
    break;
  case CH2:
    TIM_OC2Init(_timer, &_TIM_OCInitStructure);
    break;
  case CH3:
    TIM_OC3Init(_timer, &_TIM_OCInitStructure);
    break;
  case CH4:
    TIM_OC4Init(_timer, &_TIM_OCInitStructure);
    break;
  default:
    break;
  }
}

uint16_t PWM::convertDutyCycleToPulse(uint16_t dutyCycle)
{
  /* TIM_Pulse = (PWM_Duty Cycle % * TIM_Period) / 100% */
  return (dutyCycle * (_TIM_TimeBaseStructure.TIM_Period) / 100.0);
}

