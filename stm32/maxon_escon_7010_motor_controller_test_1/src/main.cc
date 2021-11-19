/**
 * @file   main.cc
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  maxon mmc ESCON 70/10 motor controller test.
 */

#include "main.h"

#define PWM_FREQUENCY (1000) /* PWM frequency in Hz. */
// #define PWM_DUTY_CYCLE (72.5) /* PWM duty-cycle in %. */

/*
 * f_pwm = f_tim / [(PRS + 1) * (PER + 1)]
 * So,
 * PER = {f_tim / [(PRS + 1) * f_pwm]} - 1
 * 
 * f_pwm: PWM frequency.
 * f_tim: Timer frequency. The value is 'rcc_apb1_frequency * 2' equal 48MHz in this case.
 * PRS:   PWM timer prescaler.
 * PER:   PWM timer period.
 */
#define PWM_TIMER_PRESCALER (48 - 1)
#define PWM_TIMER_PERIOD (((rcc_apb1_frequency * 2) / ((PWM_TIMER_PRESCALER + 1) * PWM_FREQUENCY)) - 1)

int main(void)
{
  setup_clock();
  setup_led();
  setup_pwm();

  float dutycycle = 0;
  bool dir_up = true;

  while (1)
  {
    set_dutycycle(dutycycle);

    if (dir_up && dutycycle >= 95)
    {
      dir_up = false;
    }
    else if (!dir_up && dutycycle <= 5)
    {
      dir_up = true;
    }

    if (dir_up)
    {
      dutycycle += 0.1;
    }
    else
    {
      dutycycle -= 0.1;
    }

    delay(100000);
    gpio_toggle(GPIOA, GPIO5);
  }

  return 0;
}

void inline setup_clock(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_TIM3);
}

void setup_pwm(void)
{
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO7);

  timer_set_mode(TIM3,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIM3);
  timer_continuous_mode(TIM3);

  timer_set_prescaler(TIM3, PWM_TIMER_PRESCALER);
  timer_set_period(TIM3, PWM_TIMER_PERIOD);

  timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_value(TIM3, TIM_OC2, 0); /* Set duty cycle to 0%. */

  timer_enable_oc_output(TIM3, TIM_OC2);
  timer_enable_counter(TIM3);
}

void setup_led(void)
{
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO5);
}

void inline set_dutycycle(float value)
{
  timer_set_oc_value(TIM3,
                     TIM_OC2,
                     PWM_TIMER_PERIOD * (value / 100.0));
}

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}
