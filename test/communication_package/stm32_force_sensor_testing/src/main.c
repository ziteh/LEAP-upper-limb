/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Send testing force sensor data package.
 */

#include "main.h"

volatile bool sending = true;

int main(void)
{
  setup_clock();
  setup_usart();
  setup_adc();
  setup_exti();
  setup_others_gpio();

  printf("Ready\r\n");

  while (1)
  {
    if (sending)
    {
      gpio_set(LED_PORT, LED_PIN);
      send_force_sensor_value(0);
      delay(100000);
    }
    else
    {
      gpio_clear(LED_PORT, LED_PIN);
    }
  }

  return 0;
}

void exti15_10_isr(void)
{
  exti_reset_request(EXTI13);
  sending = !sending;
}

void send_force_sensor_value(uint8_t id)
{
  uint16_t value_x = get_adc_value(FORCE_SENSOR_X_ADC_CHANNEL);
  uint16_t value_y = get_adc_value(FORCE_SENSOR_Y_ADC_CHANNEL);
  uint16_t value_z = get_adc_value(FORCE_SENSOR_Z_ADC_CHANNEL);

  usart_send_blocking(USART2, FORCE_SENSOR_VALUE_HEADER);
  usart_send_blocking(USART2, id);
  usart_send_blocking(USART2, (value_x & 0x3f));
  usart_send_blocking(USART2, ((value_x >> 6) & 0x3f));
  usart_send_blocking(USART2, (value_y & 0x3f));
  usart_send_blocking(USART2, ((value_y >> 6) & 0x3f));
  usart_send_blocking(USART2, (value_z & 0x3f));
  usart_send_blocking(USART2, ((value_z >> 6) & 0x3f));
  usart_send_blocking(USART2, EOT_SYMBOL);
}

uint16_t get_adc_value(int channel)
{
  uint8_t adc_channel[16];
  adc_channel[0] = channel;
  adc_set_regular_sequence(ADC1, 1, adc_channel);

  adc_start_conversion_direct(ADC1);

  /* Wait for ADC. */
  while (!adc_get_flag(ADC1, ADC_SR_EOC))
  {
    /* Do nothing. */
  }

  return ADC_DR(ADC1);
}

void setup_clock(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_AFIO);
}

void setup_exti(void)
{
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

  exti_select_source(EXTI13, BUTTON_PORT);
  exti_set_trigger(EXTI13, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI13);
}


void setup_adc(void)
{
  gpio_set_mode(FORCE_SENSOR_X_ADC_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                FORCE_SENSOR_X_ADC_PIN);

  gpio_set_mode(FORCE_SENSOR_Y_ADC_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                FORCE_SENSOR_Y_ADC_PIN);

  gpio_set_mode(FORCE_SENSOR_Z_ADC_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                FORCE_SENSOR_Z_ADC_PIN);

  adc_power_off(ADC1);

  adc_disable_scan_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_55DOT5CYC);

  adc_power_on(ADC1);
  delay(800000); /* Wait a bit. */
  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);

  uint8_t adc_channel[16];
  adc_channel[0] = 0;
  adc_set_regular_sequence(ADC1, 1, adc_channel);
}

void setup_usart(void)
{
  /* Enable USART IRQ. */
  nvic_enable_irq(NVIC_USART2_IRQ);

  /* Setup Tx pin. */
  gpio_set_mode(GPIO_BANK_USART2_TX,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART2_TX);

  /* Setup Rx pin. */
  gpio_set_mode(GPIO_BANK_USART2_RX,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO_USART2_RX);

  /* Setup USART. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  /* Enable Rx interrupt. */
  usart_enable_rx_interrupt(USART2);

  /* Enable. */
  usart_enable(USART2);
}

void setup_others_gpio(void)
{
  /* LED. */
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);

  gpio_set_mode(BUTTON_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                BUTTON_PIN);
}

void inline delay(unsigned int value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}
