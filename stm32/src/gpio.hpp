/**
 * @file    gpio.hpp 
 * @author	ZiTe (honmonoh@gmail.com)
 * @date	  Sep 30, 2021
 */

#ifndef GPIO_HPP_
#define GPIO_HPP_

extern "C"
{
#include "stm32f10x.h"
#include "stm32f103rb_gpio_mapping.h"
}

typedef enum
{
  Low = 0,
  High = !Low
} GPIO_ValueTypeDef;

class GPIO
{
public:
  GPIO(GPIO_PortPinTypeDef portPin,
       GPIOMode_TypeDef mode,
       GPIOSpeed_TypeDef speed = GPIO_Speed_2MHz);
  void setValue(GPIO_ValueTypeDef value);
  void setValue(uint8_t value);
  void toggleValue(void);

  GPIO_ValueTypeDef getValue(void);
  GPIO_ValueTypeDef getInputValue(void);
  GPIO_ValueTypeDef getOutputValue(void);

  GPIO_TypeDef *getPort(void);
  uint16_t getPin(void);

private:
  GPIO_PortPinTypeDef _portPin;
  GPIOMode_TypeDef _mode;
  GPIOSpeed_TypeDef _speed;

  void init(void);
  GPIO_ValueTypeDef uint8_t_to_GPIO_Value_TypeDef(uint8_t value);
};

#endif /* GPIO_HPP_ */
