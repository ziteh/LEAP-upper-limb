/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic 3-axis force sensor test.
 */

#include "printf.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>

#define USART_BAUD_RATE (115200)
#define MAIN_DELAY_VALUE (0) // 500000

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)

#define GPIO_UART_TX_PORT (GPIOA)
#define GPIO_UART_TX_PIN (GPIO2)

#define GPIO_ADC_A0_PORT (GPIOA)
#define GPIO_ADC_A0_PIN (GPIO0)
#define ADC_A0_CHANNEL (0)

#define GPIO_ADC_A1_PORT (GPIOA)
#define GPIO_ADC_A1_PIN (GPIO1)
#define ADC_A1_CHANNEL (1)

#define GPIO_ADC_A2_PORT (GPIOA)
#define GPIO_ADC_A2_PIN (GPIO4)
#define ADC_A2_CHANNEL (4)

#define GPIO_ADC_A3_PORT (GPIOB)
#define GPIO_ADC_A3_PIN (GPIO0)
#define ADC_A3_CHANNEL (8)

#define FORCE_SENSOR_VALUE_HEADER (0x8d)
#define EOT_SYMBOL (0xff)

void delay(uint32_t volatile value);

void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);

  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_USART2);
}

void led_setup(void)
{
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LED_PIN);
}

void adc_setup(void)
{
  gpio_mode_setup(GPIO_ADC_A0_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ADC_A0_PIN);
  gpio_mode_setup(GPIO_ADC_A1_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ADC_A1_PIN);
  gpio_mode_setup(GPIO_ADC_A2_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ADC_A2_PIN);
  gpio_mode_setup(GPIO_ADC_A3_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ADC_A3_PIN);

  adc_power_off(ADC1);

  adc_disable_scan_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_56CYC);

  adc_power_on(ADC1);
  delay(800000); /* Wait a bit. */
}

void usart_setup(void)
{
  gpio_mode_setup(GPIO_UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_UART_TX_PIN);
  gpio_set_af(GPIO_UART_TX_PORT, GPIO_AF7, GPIO_UART_TX_PIN);

  usart_set_baudrate(USART2, USART_BAUD_RATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  usart_enable(USART2);
}

inline void delay(uint32_t volatile value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

uint16_t get_adc_value(uint32_t adc, uint8_t channel)
{
  uint8_t adc_channel[16];
  adc_channel[0] = channel;
  adc_set_regular_sequence(adc, 1, adc_channel);
  adc_start_conversion_regular(adc);

  /* Wait for ADC convert complete. */
  while (!adc_eoc(adc))
  {
    /* Do nothing. */
  }

  return adc_read_regular(adc);
}

void send_force_sensor_value(uint8_t id)
{
  uint16_t adc_value_0 = get_adc_value(ADC1, ADC_A0_CHANNEL);
  uint16_t adc_value_1 = get_adc_value(ADC1, ADC_A1_CHANNEL);
  uint16_t adc_value_2 = get_adc_value(ADC1, ADC_A2_CHANNEL);
  uint16_t adc_value_3 = get_adc_value(ADC1, ADC_A3_CHANNEL);

  int16_t x = adc_value_1 - adc_value_0;
  int16_t y = adc_value_3 - adc_value_2;
  int16_t z = 0;

  if (x < 0)
  {
    x = (0xfff + x) + 1;
  }

  if (y < 0)
  {
    y = (0xfff + y) + 1;
  }

  usart_send_blocking(USART2, FORCE_SENSOR_VALUE_HEADER);
  usart_send_blocking(USART2, (int)id);
  usart_send_blocking(USART2, x & 0x3f);
  usart_send_blocking(USART2, (x >> 6) & 0x3f);
  usart_send_blocking(USART2, y & 0x3f);
  usart_send_blocking(USART2, (y >> 6) & 0x3f);
  usart_send_blocking(USART2, z & 0x3f);
  usart_send_blocking(USART2, (z >> 6) & 0x3f);
  usart_send_blocking(USART2, EOT_SYMBOL);
}

int main(void)
{
  rcc_setup();
  led_setup();
  usart_setup();
  adc_setup();

  printf("\r\nReady\r\n");

  while (1)
  {
    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN);
    send_force_sensor_value(0);
    delay(MAIN_DELAY_VALUE);
  }

  return 0;
}