/**
 * @file   printf.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Implement printf() function by USART.
 * @remark Reference: https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/usart_printf/usart_printf.c
 */

#include "printf.h"

int _write(int file, char *ptr, int len)
{
  int i;

  if (file == 1)
  {
    HAL_UART_Transmit(&PRINTF_USART_INSTANCE, ptr, len, HAL_MAX_DELAY);
    return i;
  }

  errno = EIO;
  return -1;
}