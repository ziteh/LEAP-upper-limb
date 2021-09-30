/**
 * @file	  gpio.cpp
 * @author	ZiTe (honmonoh@gmail.com)
 * @date    Sep 30, 2021
 */

#include "gpio.hpp"

GPIO::GPIO(GPIO_PortPinTypeDef portPin,
           GPIOMode_TypeDef mode,
           GPIOSpeed_TypeDef speed)
{
  this->_portPin = portPin;
  this->_mode = mode;
  this->_speed = speed;
  this->init();
}

void GPIO::init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = this->_mode;
  GPIO_InitStructure.GPIO_Speed = this->_speed;
  GPIO_InitStructure.GPIO_Pin = this->getPin();
  GPIO_Init(this->getPort(), &GPIO_InitStructure);
}

void GPIO::setValue(GPIO_ValueTypeDef value)
{
  switch (value)
  {
  case Low:
    (this->getPort())->BRR |= (this->getPin()); // Set value LOW
    break;

  case High:
    (this->getPort())->BSRR |= (this->getPin()); // Set value HIGH
    break;
  }
}

void GPIO::setValue(uint8_t value)
{
  this->setValue(this->uint8_t_to_GPIO_Value_TypeDef(value));
}

void GPIO::toggleValue(void)
{
  (this->getPort())->ODR ^= (this->getPin());
}

GPIO_ValueTypeDef GPIO::getValue(void)
{
  uint8_t value;

  switch (this->_mode)
  {
  case GPIO_Mode_AIN:
  case GPIO_Mode_IN_FLOATING:
  case GPIO_Mode_IPD:
  case GPIO_Mode_IPU:
    // This GPIO is input.
    value = GPIO_ReadInputDataBit(this->getPort(), this->getPin());
    break;

  case GPIO_Mode_Out_OD:
  case GPIO_Mode_Out_PP:
  case GPIO_Mode_AF_OD:
  case GPIO_Mode_AF_PP:
    // This GPIO is output.
    value = GPIO_ReadOutputDataBit(this->getPort(), this->getPin());
    break;
  }

  return this->uint8_t_to_GPIO_Value_TypeDef(value);
}

GPIO_ValueTypeDef GPIO::getInputValue(void)
{
  GPIO_ValueTypeDef value;

  if (((this->getPort())->IDR & (this->getPin())) != (uint32_t)Bit_RESET)
  {
    value = High;
  }
  else
  {
    value = Low;
  }

  return value;
}

GPIO_ValueTypeDef GPIO::getOutputValue(void)
{
  GPIO_ValueTypeDef value;

  if (((this->getPort())->ODR & (this->getPin())) != (uint32_t)Bit_RESET)
  {
    value = High;
  }
  else
  {
    value = Low;
  }

  return value;
}

GPIO_TypeDef *GPIO::getPort(void)
{
  if (((uint8_t)this->_portPin) <= ((uint8_t)PA15)) // Port-A:  0~15
    return GPIOA;
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PB15)) // Port-B: 16~31
    return GPIOB;
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PC15)) // Port-C: 32~47
    return GPIOC;
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PD15)) // Port-D: 48~63
    return GPIOD;
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PE15)) // Port-E: 64~79
    return GPIOE;
}

uint16_t GPIO::getPin(void)
{
  uint8_t offset = 0;

  if (((uint8_t)this->_portPin) <= ((uint8_t)PA15)) // Port-A:  0~15
    offset = ((uint8_t)PA0);
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PB15)) // Port-B: 16~31
    offset = ((uint8_t)PB0);
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PC15)) // Port-C: 32~47
    offset = ((uint8_t)PC0);
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PD15)) // Port-D: 48~63
    offset = ((uint8_t)PD0);
  else if (((uint8_t)this->_portPin) <= ((uint8_t)PE15)) // Port-E: 64~79
    offset = ((uint8_t)PE0);

  return ((uint16_t)(0x0001 << (((uint8_t)this->_portPin) - offset)));
}

/**
 * @brief   Convert uint8_t to GPIO_Value_TypeDef.
 * @param   value: The value in uint8_t. This parameter should be 0 or 1.
 * @return  The converted GPIO_Value_TypeDef value.
 */
GPIO_ValueTypeDef GPIO::uint8_t_to_GPIO_Value_TypeDef(uint8_t value)
{
  GPIO_ValueTypeDef gpioValue;
  if (value == 0)
  {
    gpioValue = (GPIO_ValueTypeDef)Low;
  }
  else
  {
    gpioValue = (GPIO_ValueTypeDef)High;
  }
  return gpioValue;
}
