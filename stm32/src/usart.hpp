/**
 * @file	  usart.hpp
 * @author	ZiTe (honmonoh@gmail.com)
 * @date    Sep 30, 2021
 */

#ifndef USART_HPP_
#define USART_HPP_

extern "C"
{
#include "stm32f10x.h"
}
#include <string>
#include "gpio.hpp"

/**
 * @brief Only for USART2
 */
class USART
{
private:
public:
  USART();
  void send(uint8_t *data);
  void send(std::string data);
};

#endif /* USART_HPP_ */
