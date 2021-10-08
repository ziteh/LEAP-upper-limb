/**
 * @file    main.cpp
 * @author  ZiTe <honmonoh@gmail.com>
 * @brief   Main program body
 */

extern "C"
{
#include "main.h"
}
#include "f103rb_lib.hpp"

static __IO uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

int main(void)
{
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

  while (1)
  {
  }
}

extern "C"
{
  /**
   * @brief  Inserts a delay time.
   * @param  nTime: specifies the delay time length, in 1 ms.
   * @retval None
   */
  void Delay(__IO uint32_t nTime)
  {
    TimingDelay = nTime;

    while (TimingDelay != 0)
    {
      /* Null */
    }
  }

  /**
   * @brief  Decrements the TimingDelay variable.
   * @param  None
   * @retval None
   */
  void TimingDelay_Decrement(void)
  {
    if (TimingDelay != 0x00)
    {
      TimingDelay--;
    }
  }
}
