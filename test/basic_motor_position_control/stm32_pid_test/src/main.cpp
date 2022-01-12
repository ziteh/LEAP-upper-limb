/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic motor position control with PID.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

/* User LED. */
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

/* User button. */
#define BUTTON_PORT (GPIOC)
#define BUTTON_PIN (GPIO13)

/* A0 = PA0. */
#define HALL_SENSOR_A_PORT (GPIOA)
#define HALL_SENSOR_A_PIN (GPIO0)

/* A1 = PA1. */
#define HALL_SENSOR_B_PORT (GPIOA)
#define HALL_SENSOR_B_PIN (GPIO1)

/* A2 = PA4. */
#define HALL_SENSOR_C_PORT (GPIOA)
#define HALL_SENSOR_C_PIN (GPIO4)

#define CW (false)
#define CCW (!CW)

void clock_init(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_SYSCFG); // Request for EXTI.
}

void led_init(void)
{
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
  gpio_set_output_options(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, LED_PIN);
}

void button_init(void)
{
  gpio_mode_setup(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BUTTON_PIN);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);
  exti_select_source(EXTI13, GPIOC);
  exti_set_trigger(EXTI13, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI13);
}

void hall_sensor_init(void)
{
  gpio_mode_setup(HALL_SENSOR_A_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, HALL_SENSOR_A_PIN);
  gpio_mode_setup(HALL_SENSOR_B_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, HALL_SENSOR_B_PIN);
  gpio_mode_setup(HALL_SENSOR_C_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, HALL_SENSOR_C_PIN);

  nvic_enable_irq(NVIC_EXTI0_IRQ);
  nvic_enable_irq(NVIC_EXTI1_IRQ);
  nvic_enable_irq(NVIC_EXTI4_IRQ);

  exti_select_source(EXTI0, GPIOA);
  exti_select_source(EXTI1, GPIOA);
  exti_select_source(EXTI4, GPIOA);

  exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);
  exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH);
  exti_set_trigger(EXTI4, EXTI_TRIGGER_BOTH);

  exti_enable_request(EXTI0);
  exti_enable_request(EXTI1);
  exti_enable_request(EXTI4);
}

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); // Do nothing.
  }
}

int main(void)
{
  clock_init();
  led_init();
  button_init();
  hall_sensor_init();

  while (1)
  {
    gpio_toggle(LED_PORT, LED_PIN);
    delay(2000000);
  }

  return 0;
}

void exti0_isr(void)
{
  exti_reset_request(EXTI0);
  gpio_toggle(LED_PORT, LED_PIN);
}

void exti1_isr(void)
{
  exti_reset_request(EXTI1);
  gpio_toggle(LED_PORT, LED_PIN);
}

void exti4_isr(void)
{
  exti_reset_request(EXTI4);
  gpio_toggle(LED_PORT, LED_PIN);
}

void exti15_10_isr(void)
{
  exti_reset_request(EXTI13);
  gpio_toggle(LED_PORT, LED_PIN);
}