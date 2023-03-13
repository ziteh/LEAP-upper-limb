/**
 * @file main.h
 * @brief
 * @author ZiTe (honmonoh@gmail.com)
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "as5047p.h"

#define TIMER_PRESCALER(f_tim, f_cnt) (f_tim / f_cnt - 1)                          /* PSC. */
#define TIMER_AUTO_RELOAD(f_goal, f_tim, psc) ((f_tim / ((psc + 1) * f_goal)) - 1) /* ARR. */
#define TIMER_PWM_CCR(arr, dc_goal) ((arr + 1) * dc_goal / 100)                    /* CCR. */

#define USART_CONSOLE_INSTANCE (USART2)
#define USART_CONSOLE_BAUDRATE (115200)
#define USART_CONSOLE_IRQ (NVIC_USART2_IRQ)
#define GPIO_USART_CONSOLE_TXRX_PORT (GPIOA)
#define GPIO_USART_CONSOLE_TX_PIN (GPIO2) /* ST-Link (D1). */
#define GPIO_USART_CONSOLE_RX_PIN (GPIO3) /* ST-Link (D0). */
#define GPIO_USART_CONSOLE_AF (GPIO_AF7)

#define SPI_AS5047_INSTANCE (SPI1)
#define GPIO_SPI_AS5047_SCK_MISO_MOSI_PORT (GPIOA)
#define GPIO_SPI_AS5047_SCK_PIN (GPIO5)  /* D13. */
#define GPIO_SPI_AS5047_MISO_PIN (GPIO6) /* D12. */
#define GPIO_SPI_AS5047_MOSI_PIN (GPIO7) /* D11. */
#define GPIO_SPI_AS5047_SS_PORT (GPIOB)
#define GPIO_SPI_AS5047_SS_PIN (GPIO6) /* D10. */
#define GPIO_SPI_AS5047_AF (GPIO_AF5)

#define GPIO_ENCODER_PORT (GPIOB)
#define GPIO_ENCODER_A_PIN (GPIO4) /* D5. */
#define GPIO_ENCODER_B_PIN (GPIO5) /* D4. */
#define GPIO_ENCODER_I_PIN (GPIO3) /* D3. */
#define EXTI_ENCODER_A (EXTI4)
#define EXTI_ENCODER_B (EXTI5)
#define EXTI_ENCODER_I (EXTI3)
#define ENCODER_A_IRQ (NVIC_EXTI4_IRQ)
#define ENCODER_B_IRQ (NVIC_EXTI9_5_IRQ)
#define ENCODER_I_IRQ (NVIC_EXTI3_IRQ)

#define TIMER_ITERATION_INSTANCE (TIM10)
#define TIMER_ITERATION_IRQ (NVIC_TIM1_UP_TIM10_IRQ)
#define TIMER_ITERATION_GOAL_FREQ (2)              /* In Hz. */
#define TIMER_ITERATION_FREQ (rcc_apb2_frequency * 2) /* In Hz. */
#define TIMER_ITERATION_COUNTER_FREQ (1e5)            /* In Hz. */

#define TIMER_PWM_INSTANCE (TIM2)
#define TIMER_PWM_GOAL_FREQ (1000)              /* In Hz. */
#define TIMER_PWM_FREQ (rcc_apb1_frequency * 2) /* In Hz. */
#define TIMER_PWM_COUNTER_FREQ (1e5)            /* In Hz. */
#define TIMER_PWM_OC (TIM_OC3)
#define GPIO_PWM_PORT (GPIOB)
#define GPIO_PWM_PIN (GPIO10) /* D6. */
#define GPIO_PWM_AF (GPIO_AF1)

#define GPIO_MOTOR_ENABLE_PORT (GPIOA)
#define GPIO_MOTOR_ENABLE_PIN (GPIO8) /* D7. */

#define GPIO_MOTOR_DIRECTION_PORT (GPIOA)
#define GPIO_MOTOR_DIRECTION_PIN (GPIO9) /* D8. */

typedef enum
{
  CW = 0,
  CCW = !CW
} direction_t;

static void setup_rcc(void);
static void setup_systick(void);
static void setup_usart(void);
static void setup_spi(void);
static void setup_timer(void);
static void setup_pwm(void);
static void setup_others_gpio(void);

static void setup_as5047(void);
static void setup_encoder_exti(void);

static void delay_ns(uint32_t ns);
static void delay_ms(uint32_t ms);

static void spi_as5047_select(void);
static void spi_as5047_deselect(void);
static void spi_as5047_send(uint16_t data);
static uint16_t spi_as5047_read(void);

static float pid_compute(float set_value,
                         float actual_value,
                         float kp,
                         float ki,
                         float kd,
                         float i_term_prev,
                         float error_prev,
                         float iteration_time,
                         float max,
                         float min,
                         float bias,
                         float *i_term,
                         float *error);

static void update_present_position(void);
static void set_pwm_duty_cycle(float duty_cycle);
static void set_motor_status(bool enable);
// static bool get_motor_status(void);
static void set_motor_direction(direction_t dir);

#endif /* MAIN_H_ */