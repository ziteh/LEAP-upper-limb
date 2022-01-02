/**
 * @file   setup.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Initialize each function.
 */

#include "setup.h"

void setup_all(void)
{
  setup_clock();
  setup_usart();
  setup_pwm();
  setup_adc();
  setup_others_gpio();
}

void setup_clock(void)
{
  rcc_clock_setup_in_hsi_out_48mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_TIM3);
  // rcc_periph_clock_enable(RCC_TIM2);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_ADC1);
}

void setup_adc(void)
{
  /* EFE joint. */
  gpio_set_mode(EFE_JOINT_POSITION_ADC_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                EFE_JOINT_POSITION_ADC_PIN);

  adc_power_off(EFE_JOINT_POSITION_ADC);

  adc_disable_scan_mode(EFE_JOINT_POSITION_ADC);
  adc_disable_external_trigger_regular(EFE_JOINT_POSITION_ADC);
  adc_set_single_conversion_mode(EFE_JOINT_POSITION_ADC);
  adc_set_right_aligned(EFE_JOINT_POSITION_ADC);
  adc_set_sample_time(EFE_JOINT_POSITION_ADC,
                      EFE_JOINT_POSITION_ADC_CHANNEL,
                      ADC_SMPR_SMP_55DOT5CYC);

  adc_power_on(EFE_JOINT_POSITION_ADC);
  delay(800000); // Wait a bit.
  adc_reset_calibration(EFE_JOINT_POSITION_ADC);
  adc_calibrate(EFE_JOINT_POSITION_ADC);

  /* SFE joint. */
  gpio_set_mode(SFE_JOINT_POSITION_ADC_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                SFE_JOINT_POSITION_ADC_PIN);

  adc_power_off(SFE_JOINT_POSITION_ADC);

  adc_disable_scan_mode(SFE_JOINT_POSITION_ADC);
  adc_disable_external_trigger_regular(SFE_JOINT_POSITION_ADC);
  adc_set_single_conversion_mode(SFE_JOINT_POSITION_ADC);
  adc_set_right_aligned(SFE_JOINT_POSITION_ADC);
  adc_set_sample_time(SFE_JOINT_POSITION_ADC,
                      SFE_JOINT_POSITION_ADC_CHANNEL,
                      ADC_SMPR_SMP_55DOT5CYC);

  adc_power_on(SFE_JOINT_POSITION_ADC);
  delay(800000); // Wait a bit.
  adc_reset_calibration(SFE_JOINT_POSITION_ADC);
  adc_calibrate(SFE_JOINT_POSITION_ADC);
}

void setup_pwm(void)
{
  /* EFE joint. */
  gpio_set_mode(EFE_MOTOR_SPEED_PWM_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                EFE_MOTOR_SPEED_PWM_PIN);

  timer_set_mode(EFE_MOTOR_SPEED_PWM_TIM,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);

  timer_disable_preload(EFE_MOTOR_SPEED_PWM_TIM);
  timer_continuous_mode(EFE_MOTOR_SPEED_PWM_TIM);

  timer_set_prescaler(EFE_MOTOR_SPEED_PWM_TIM, PWM_TIMER_PRESCALER);
  timer_set_period(EFE_MOTOR_SPEED_PWM_TIM, PWM_TIMER_PERIOD);

  timer_set_oc_mode(EFE_MOTOR_SPEED_PWM_TIM, EFE_MOTOR_SPEED_PWM_OC, TIM_OCM_PWM1);
  timer_set_oc_value(EFE_MOTOR_SPEED_PWM_TIM, EFE_MOTOR_SPEED_PWM_OC, 0); // Set duty cycle to 0%.

  timer_enable_oc_output(EFE_MOTOR_SPEED_PWM_TIM, EFE_MOTOR_SPEED_PWM_OC);
  timer_enable_counter(EFE_MOTOR_SPEED_PWM_TIM);

  /* SFE joint. */
  gpio_set_mode(SFE_MOTOR_SPEED_PWM_PORT,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                SFE_MOTOR_SPEED_PWM_PIN);

  timer_set_mode(SFE_MOTOR_SPEED_PWM_TIM,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);

  timer_disable_preload(SFE_MOTOR_SPEED_PWM_TIM);
  timer_continuous_mode(SFE_MOTOR_SPEED_PWM_TIM);

  timer_set_prescaler(SFE_MOTOR_SPEED_PWM_TIM, PWM_TIMER_PRESCALER);
  timer_set_period(SFE_MOTOR_SPEED_PWM_TIM, PWM_TIMER_PERIOD);

  timer_set_oc_mode(SFE_MOTOR_SPEED_PWM_TIM, SFE_MOTOR_SPEED_PWM_OC, TIM_OCM_PWM1);
  timer_set_oc_value(SFE_MOTOR_SPEED_PWM_TIM, SFE_MOTOR_SPEED_PWM_OC, 0); // Set duty cycle to 0%.

  timer_enable_oc_output(SFE_MOTOR_SPEED_PWM_TIM, SFE_MOTOR_SPEED_PWM_OC);
  timer_enable_counter(SFE_MOTOR_SPEED_PWM_TIM);
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
  /* Board LED. */
  gpio_set_mode(LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);
  gpio_clear(LED_PORT, LED_PIN); /* LED off. */

  /* EFE joint motor enable pin. */
  gpio_set_mode(EFE_MOTOR_ENABLE_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                EFE_MOTOR_ENABLE_PIN);
  gpio_clear(EFE_MOTOR_ENABLE_PORT, EFE_MOTOR_ENABLE_PIN); // Disable motor.

  /* EFE joint motor direction pin. */
  gpio_set_mode(EFE_MOTOR_DIRECTION_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                EFE_MOTOR_DIRECTION_PIN);
  gpio_clear(EFE_MOTOR_DIRECTION_PORT, EFE_MOTOR_DIRECTION_PIN); // Set to CW.

  /* EFE joint motor ready pin. */
  gpio_set_mode(EFE_MOTOR_READY_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN,
                EFE_MOTOR_READY_PIN);

  /* SFE joint motor enable pin. */
  gpio_set_mode(SFE_MOTOR_ENABLE_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                SFE_MOTOR_ENABLE_PIN);
  gpio_clear(SFE_MOTOR_ENABLE_PORT, SFE_MOTOR_ENABLE_PIN); // Disable motor.

  /* SFE joint motor direction pin. */
  gpio_set_mode(SFE_MOTOR_DIRECTION_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                SFE_MOTOR_DIRECTION_PIN);
  gpio_clear(SFE_MOTOR_DIRECTION_PORT, SFE_MOTOR_DIRECTION_PIN); // Set to CW.

  /* SFE joint motor ready pin. */
  gpio_set_mode(SFE_MOTOR_READY_PORT,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN,
                SFE_MOTOR_READY_PIN);
}
