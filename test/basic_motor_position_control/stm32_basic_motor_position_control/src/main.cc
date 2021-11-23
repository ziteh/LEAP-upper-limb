/**
 * @file   main.cc
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Basic motor position control.
 */

#include "main.h"

int main(void)
{
  setup_clock();
  setup_others_gpio();
  setup_adc();
  setup_pwm();
  setup_usart();

  printf("Ready\r\n");

  /* Disable motor. */
  gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
  set_dutycycle(15);

  while (1)
  {
    __asm__("nop");
  }

  return 0;
}

void move(float position)
{
  gpio_set(LED_PORT, LED_PIN);

  float goal = ((MAX_POSITION - MIN_POSITION) * position * 0.01) + MIN_POSITION;
  uint16_t now_position = get_adc_value();
  if (now_position < goal)
  {
    /* Set motor CCW. */
    gpio_set(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);

    /* Enable motor. */
    gpio_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);

    /* Wait. */
    while ((goal - now_position) > ALLOWABLE_POSITION_ERROR)
    {
      now_position = get_adc_value();
      if (now_position > MAX_POSITION || now_position < MIN_POSITION)
      {
        break;
      }
    }
  }
  else if (now_position > goal)
  {
    /* Set motor CW. */
    gpio_clear(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);

    /* Enable motor. */
    gpio_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);

    /* Wait. */
    while ((now_position - goal) > ALLOWABLE_POSITION_ERROR)
    {
      now_position = get_adc_value();
      if (now_position > MAX_POSITION || now_position < MIN_POSITION)
      {
        break;
      }
    }
  }
  /* Disable motor. */
  gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);

  gpio_clear(LED_PORT, LED_PIN);
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  uint16_t data = usart_recv(USART2);

  /* MSB = 1: is command, else position %. */
  if ((data & (1 << 7)) == (1 << 7))
  {
    switch (data)
    {
    case 0x80:
      /* Disable. */
      gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
      break;

    case 0x81:
      /* Enable. */
      gpio_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
      break;

    case 0x82:
      /* Dir: CW. */
      gpio_clear(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
      break;

    case 0x83:
      /* Dir: CCW. */
      gpio_set(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
      break;

    default:
      usart_send_blocking(USART2, '?');
      usart_send_blocking(USART2, '\r');
      usart_send_blocking(USART2, '\n');
      break;
    }
  }
  else
  {
    move(data);
  }

  /* 
   * Clear RXNE(Read data register not empty) flag of
   * USART SR(Status register).
   */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

uint16_t get_adc_value(void)
{
  adc_start_conversion_direct(ADC1);

  /* Wait for ADC. */
  while (!adc_get_flag(ADC1, ADC_SR_EOC))
  {
    /* Do nothing. */
  }

  return ADC_DR(ADC1);
}

void inline set_dutycycle(float value)
{
  timer_set_oc_value(TIM3,
                     TIM_OC2,
                     PWM_TIMER_PERIOD * (value / 100.0));
}

void inline setup_clock(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_ADC1);
}

void setup_adc(void)
{
  gpio_set_mode(ADC_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                ADC_PIN);

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

void setup_pwm(void)
{
  gpio_set_mode(PWM_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                PWM_PIN);

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
  gpio_clear(LED_PORT, LED_PIN);

  /* Motor enable pin. */
  gpio_set_mode(MOTOR_ENABLE_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                MOTOR_ENABLE_PIN);
  gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);

  /* Motor direction pin. */
  gpio_set_mode(MOTOR_DIRECTION_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                MOTOR_DIRECTION_PIN);
  gpio_clear(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
}

void delay(uint32_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}
