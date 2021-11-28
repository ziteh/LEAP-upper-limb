/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Communication packge test.
 */

#include "main.h"

int data_byte_number;
uint8_t info_byte;
uint8_t buffer[BUFFER_LENGTH];

int main(void)
{
  setup_clock();
  setup_usart();
  setup_pwm();
  setup_adc();
  setup_others_gpio();

  clear_communication_variable();

  printf("Ready\r\n");

  while (1)
  {
    __asm__("nop"); /* Do nothing. */
  }

  return 0;
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  uint16_t data = usart_recv(USART2);

  if (IS_INFO_BYTE(data))
  {
    info_byte = data;
    switch (info_byte)
    {
    /* Motor control. */
    case MOTOR_CONTROL_INFO_BYTE:
      data_byte_number = MOTOR_CONTROL_DATA_BYTE_NUMBER;
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
    if (data_byte_number > 0)
    {
      switch (info_byte)
      {
      case MOTOR_CONTROL_INFO_BYTE:
        buffer[data_byte_number - 1] = data;
        break;

      default:
        break;
      }
    }

    data_byte_number--;
    if (data_byte_number == 0)
    {
      switch (info_byte)
      {
      case MOTOR_CONTROL_INFO_BYTE:
      {
        uint8_t id = buffer[3] & 0x1f;
        uint8_t enable = buffer[2] & 0x03;
        uint8_t dircetion = (buffer[2] & 0x0c) >> 2;
        uint16_t position = (buffer[1] & 0x3f) | ((buffer[0] & 0x3f) << 6);

        if (enable == 0x00)
        {
          /* Disable. */
          gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
        }
        else if (enable == 0x01)
        {
          /* Enable. */
          gpio_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
        }
        else if (enable == 0x02)
        {
          gpio_toggle(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
        }

        if (dircetion == 0x00)
        {
          /* Dir: CW. */
          gpio_clear(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
        }
        else if (dircetion == 0x01)
        {
          /* Dir: CCW. */
          gpio_set(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
        }
        else if (dircetion == 0x02)
        {
          gpio_toggle(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
        }
        move(position);
        break;
      }

      default:
        break;
      }
      clear_communication_variable();
    }
  }

  /* Clear RXNE(Read data register not empty) flag of USART SR(Status register). */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

void clear_communication_variable(void)
{
  info_byte = 0xff;
  data_byte_number = 0;

  /* Clear buffer. */
  for (int i = 0; i < BUFFER_LENGTH; i++)
  {
    buffer[i] = 0x00;
  }
}

void move(float position)
{
  gpio_set(LED_PORT, LED_PIN);
  set_dutycycle(15);

  uint16_t goal = ((MAX_POSITION - MIN_POSITION) * position * 0.01) + MIN_POSITION;
  uint16_t now_position = get_adc_value();
  if (now_position > goal)
  {
    /* Set motor CCW. */
    gpio_set(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);

    /* Enable motor. */
    gpio_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);

    /* Wait. */
    while ((now_position - goal) > ALLOWABLE_POSITION_ERROR)
    {
      now_position = get_adc_value();
      printf("E: %d, A: %d (CCW)\r\n", goal, now_position);
      if (now_position > MAX_POSITION || now_position < MIN_POSITION)
      {
        printf("BREAK\r\n");
        break;
      }
    }
  }
  else if (now_position < goal)
  {
    /* Set motor CW. */
    gpio_clear(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);

    /* Enable motor. */
    gpio_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);

    /* Wait. */
    while ((goal - now_position) > ALLOWABLE_POSITION_ERROR)
    {
      now_position = get_adc_value();
      printf("E: %d, A: %d (CW)\r\n", goal, now_position);
      if (now_position > MAX_POSITION || now_position < MIN_POSITION)
      {
        printf("BREAK\r\n");
        break;
      }
    }
  }
  /* Disable motor. */
  gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);

  gpio_clear(LED_PORT, LED_PIN);
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
  rcc_periph_clock_enable(RCC_GPIOB);
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
  gpio_clear(LED_PORT, LED_PIN); /* LED off. */

  /* Motor enable pin. */
  gpio_set_mode(MOTOR_ENABLE_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                MOTOR_ENABLE_PIN);
  gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN); /* Disable motor. */

  /* Motor direction pin. */
  gpio_set_mode(MOTOR_DIRECTION_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                MOTOR_DIRECTION_PIN);
  gpio_set(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
}

void delay(unsigned int value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}