/**
 * @file	  pwm.hpp
 * @author	ZiTe (honmonoh@gmail.com)
 * @date    Sep 30, 2021
 */

#ifndef PWM_HPP_
#define PWM_HPP_

extern "C"
{
#include "stm32f10x.h"
}
#include "gpio.hpp"

typedef enum
{
  CH1 = 1,
  CH2,
  CH3,
  CH4
} PWM_TimerChannelTypeDef;

class PWM
{
public:
  PWM(GPIO_PortPinTypeDef portPin);

  void Enable(void);
  void Disable(void);

  void setFrequency(uint16_t frequency);
  void setDutyCycle(uint16_t dutyCycle);

  uint16_t getFrequency(void);
  uint16_t getDutyCycle(void);

private:
  GPIO_PortPinTypeDef _piortPin;
  TIM_TypeDef *_timer;
  PWM_TimerChannelTypeDef _channel;
  TIM_TimeBaseInitTypeDef _TIM_TimeBaseStructure;
  TIM_OCInitTypeDef _TIM_OCInitStructure;

  void init(void);
  uint16_t convertDutyCycleToPulse(uint16_t dutyCycle);
};

#endif /* PWM_HPP_ */
