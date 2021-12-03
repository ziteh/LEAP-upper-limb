/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Sending data packge test.
 */

#include "main.h"

volatile bool sending_data = false;

int main(void)
{
  setup_clock();
  setup_usart();
  setup_exti();
  setup_others_gpio();

  printf("Ready\r\n");

  while (1)
  {
    if (sending_data)
    {
      gpio_set(LED_PORT, LED_PIN);

      send_fake_data();
      delay(100000);
    }
    else
    {
      gpio_clear(LED_PORT, LED_PIN);
      __asm__("nop"); /* Do nothing. */
    }
  }

  return 0;
}

void exti15_10_isr(void)
{
  exti_reset_request(EXTI13);
  sending_data = !sending_data;
}

void send_fake_data(void)
{
  usart_send_blocking(USART, FORCE_SENSOR_VALUE_HEADER);
  usart_send_blocking(USART, 0x00);
  usart_send_blocking(USART, 0x01);
  usart_send_blocking(USART, 0x02);
  usart_send_blocking(USART, 0x03);
  usart_send_blocking(USART, 0x04);
  usart_send_blocking(USART, 0x05);
  usart_send_blocking(USART, 0x06);
  usart_send_blocking(USART, EOT);
}

void inline setup_clock(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_AFIO); /* For EXTI. */
}

void setup_usart(void)
{
  /* Setup Tx pin. */
  gpio_set_mode(GPIO_BANK_USART2_TX,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART2_TX);

  /* Setup USART. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  /* Enable. */
  usart_enable(USART2);
}

void setup_exti(void)
{
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

  exti_select_source(EXTI13, BUTTON_PORT);
  exti_set_trigger(EXTI13, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI13);
}

void setup_others_gpio(void)
{
  /* Button. */
  gpio_set_mode(BUTTON_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                BUTTON_PIN);

  /* LED. */
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
}

void delay(unsigned int value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}
