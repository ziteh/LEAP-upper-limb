/**
 * @file   main.cc
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  maxon mmc ESCON 70/10 motor controller test.
 */

#include "main.h"

#define MOTOR_ENABLE_PORT (GPIOA)
#define MOTOR_ENABLE_PIN (GPIO6)

#define MOTOR_DIRECTION_PORT (GPIOA)
#define MOTOR_DIRECTION_PIN (GPIO8)

#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

#define PWM_PORT (GPIOA)
#define PWM_PIN (GPIO7)

#define USART_BAUDRATE (9600) /* USART baud rate. */
#define PWM_FREQUENCY (1000)  /* PWM frequency in Hz. */

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
  setup_control_pin();
  setup_pwm();
  setup_usart();

  usart_send_blocking(USART2, 'R');
  usart_send_blocking(USART2, 'e');
  usart_send_blocking(USART2, 'a');
  usart_send_blocking(USART2, 'd');
  usart_send_blocking(USART2, 'y');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  while (1)
  {
    __asm__("nop");
  }

  return 0;
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  uint16_t data = usart_recv(USART2);

  /* MSB = 1: is command, else duty cycle. */
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
      gpio_toggle(LED_PORT, LED_PIN);

      usart_send_blocking(USART2, '?');
      usart_send_blocking(USART2, '\r');
      usart_send_blocking(USART2, '\n');
      break;
    }
  }
  else
  {
    set_dutycycle(data);
  }

  /* 
   * Clear RXNE(Read data register not empty) flag of
   * USART SR(Status register).
   */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

void inline setup_clock(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_clock_enable(RCC_USART2);
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

void setup_led(void)
{
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
}

void inline set_dutycycle(float value)
{
  timer_set_oc_value(TIM3,
                     TIM_OC2,
                     PWM_TIMER_PERIOD * (value / 100.0));
}

void setup_control_pin(void)
{
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
