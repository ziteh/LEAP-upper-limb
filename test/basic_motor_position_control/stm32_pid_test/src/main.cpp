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

/* D11 = PA7. */
#define MOTOR_ENABLE_PORT (GPIOA)
#define MOTOR_ENABLE_PIN (GPIO7)

/* D10 = PB6. */
#define MOTOR_DIRECTION_PORT (GPIOB)
#define MOTOR_DIRECTION_PIN (GPIO6)

/* D12 = PA6, PWM3-Ch1 */
#define MOTOR_SPEED_PWM_PORT (GPIOA)
#define MOTOR_SPEED_PWM_PIN (GPIO6)
#define MOTOR_SPEED_PWM_TIM (TIM3)
#define MOTOR_SPEED_PWM_OC (TIM_OC1)

#define CW (1)
#define CCW (-1)

#define PWM_FREQUENCY (1000) /* PWM frequency in Hz. */
#define PWM_TIMER_PRESCALER (48 - 1)
#define PWM_TIMER_PERIOD (((rcc_apb1_frequency * 2) / ((PWM_TIMER_PRESCALER + 1) * PWM_FREQUENCY)) - 1)

bool hall_sensor_a = false;
bool hall_sensor_b = false;
bool hall_sensor_c = false;

auto dircetion = CW;
int64_t plus_count = 0;

void set_pwm_duty_cycle(float duty_cycle)
{
  timer_set_oc_value(MOTOR_SPEED_PWM_TIM,
                     MOTOR_SPEED_PWM_OC,
                     PWM_TIMER_PERIOD * (duty_cycle / 100.0));
}

void clock_init(void)
{
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  rcc_periph_reset_pulse(RST_TIM3);
  rcc_periph_clock_enable(RCC_TIM3);

  rcc_periph_clock_enable(RCC_SYSCFG); // Request for EXTI.
}

void motor_gpio_init(void)
{
  /* PWM. */
  gpio_mode_setup(MOTOR_SPEED_PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, MOTOR_SPEED_PWM_PIN);
  gpio_set_output_options(MOTOR_SPEED_PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, MOTOR_SPEED_PWM_PIN);
  gpio_set_af(MOTOR_SPEED_PWM_PORT, GPIO_AF2, MOTOR_SPEED_PWM_PIN); // TIM3: AF02.

  timer_set_mode(MOTOR_SPEED_PWM_TIM,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_set_prescaler(MOTOR_SPEED_PWM_TIM, PWM_TIMER_PRESCALER);
  timer_set_period(MOTOR_SPEED_PWM_TIM, PWM_TIMER_PERIOD);
  timer_set_oc_mode(MOTOR_SPEED_PWM_TIM, MOTOR_SPEED_PWM_OC, TIM_OCM_PWM1);
  set_pwm_duty_cycle(50); // Set duty cycle to 50.0 %.

  timer_disable_preload(MOTOR_SPEED_PWM_TIM);
  timer_continuous_mode(MOTOR_SPEED_PWM_TIM);

  timer_enable_oc_output(MOTOR_SPEED_PWM_TIM, MOTOR_SPEED_PWM_OC);
  timer_enable_counter(MOTOR_SPEED_PWM_TIM);

  /* Enable pin. */
  gpio_mode_setup(MOTOR_ENABLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MOTOR_ENABLE_PIN);
  gpio_set_output_options(MOTOR_ENABLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, MOTOR_ENABLE_PIN);
  gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN); // Disable motor.

  /* Dircetion pin. */
  gpio_mode_setup(MOTOR_DIRECTION_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MOTOR_DIRECTION_PIN);
  gpio_set_output_options(MOTOR_DIRECTION_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, MOTOR_DIRECTION_PIN);
  gpio_clear(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN); // Set to CW.
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
  motor_gpio_init();

  delay(1000);
  hall_sensor_a = gpio_get(HALL_SENSOR_A_PORT, HALL_SENSOR_A_PIN);
  hall_sensor_b = gpio_get(HALL_SENSOR_B_PORT, HALL_SENSOR_B_PIN);
  hall_sensor_c = gpio_get(HALL_SENSOR_C_PORT, HALL_SENSOR_C_PIN);

  while (1)
  {
    // gpio_toggle(LED_PORT, LED_PIN);
    // delay(2000000);
  }

  return 0;
}

void exti0_isr(void)
{
  exti_reset_request(EXTI0);

  /* Hall sensor A. */
  // if ((hall_sensor_a && hall_sensor_b && !hall_sensor_c) ||
  //     (!hall_sensor_a && !hall_sensor_b && hall_sensor_c))
  if (hall_sensor_a != hall_sensor_c)
  {
    dircetion = CW;
    gpio_set(LED_PORT, LED_PIN);
  }
  else
  {
    dircetion = CCW;
    gpio_clear(LED_PORT, LED_PIN);
  }
  hall_sensor_a = gpio_get(HALL_SENSOR_A_PORT, HALL_SENSOR_A_PIN);
  plus_count += dircetion;
}

void exti1_isr(void)
{
  exti_reset_request(EXTI1);

  /* Hall sensor B. */
  // if ((!hall_sensor_a && hall_sensor_b && hall_sensor_c) ||
  //     (hall_sensor_a && !hall_sensor_b && !hall_sensor_c))
  if (hall_sensor_b != hall_sensor_a)
  {
    dircetion = CW;
    gpio_set(LED_PORT, LED_PIN);
  }
  else
  {
    dircetion = CCW;
    gpio_clear(LED_PORT, LED_PIN);
  }
  hall_sensor_b = gpio_get(HALL_SENSOR_B_PORT, HALL_SENSOR_B_PIN);
  plus_count += dircetion;
}

void exti4_isr(void)
{
  exti_reset_request(EXTI4);

  /* Hall sensor C. */
  // if ((hall_sensor_a && !hall_sensor_b && hall_sensor_c) ||
  //     (!hall_sensor_a && hall_sensor_b && !hall_sensor_c))
  if (hall_sensor_c != hall_sensor_b)
  {
    dircetion = CW;
    gpio_set(LED_PORT, LED_PIN);
  }
  else
  {
    dircetion = CCW;
    gpio_clear(LED_PORT, LED_PIN);
  }
  hall_sensor_c = gpio_get(HALL_SENSOR_C_PORT, HALL_SENSOR_C_PIN);
  plus_count += dircetion;
}

void exti15_10_isr(void)
{
  exti_reset_request(EXTI13);
  gpio_toggle(LED_PORT, LED_PIN);
}