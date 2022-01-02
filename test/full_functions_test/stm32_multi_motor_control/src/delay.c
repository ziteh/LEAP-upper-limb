/**
 * @file   delay.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Delay function.
 */

#include "delay.h"

inline void delay(volatile unsigned int value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}
