/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Control DYNAMIXEL motor with RS-485 UART, using MAX485.
 */

#include "dynamixel_2_0.h"
#include "printf.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define MOTOR_ID (10)

/* PA5 = D13, User-LED. */
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)

/* PB3 = D3, MAX485 DE and RE pin. */
#define GPIO_ENABLE_PORT (GPIOB)
#define GPIO_ENABLE_PIN (GPIO3)

#define DYNAMIXEL_UART_BAUD_RATE (57600)
#define DYNAMIXEL_USART (USART1)
#define DYNAMIXEL_USART_IRQ (NVIC_USART1_IRQ)

/* PA9 = D8, USART1_TX, MAX485 DI pin. */
#define GPIO_DYNAMIXEL_UART_TX_PORT (GPIOA)
#define GPIO_DYNAMIXEL_UART_TX_PIN (GPIO9)

/* PA10 = D2, USART1_RX, MAX485 RO pin. */
#define GPIO_DYNAMIXEL_UART_RX_PORT (GPIOA)
#define GPIO_DYNAMIXEL_UART_RX_PIN (GPIO10)

#define CONSOLE_UART_BAUD_RATE (9600)
#define CONSOLE_USART (USART2)
#define CONSOLE_USART_IRQ (NVIC_USART2_IRQ)

#define GPIO_CONSOLE_UART_TX_PORT (GPIOA)
#define GPIO_CONSOLE_UART_TX_PIN (GPIO2)

#define GPIO_CONSOLE_UART_RX_PORT (GPIOA)
#define GPIO_CONSOLE_UART_RX_PIN (GPIO3)

#define BUFFER_LENGTH (128)

uint8_t buffer[BUFFER_LENGTH] = {0};
volatile uint16_t buffer_index = 0;

void rcc_setup(void);
void console_usart_setup(void);
void dynamixel_usart_setup(void);
void led_setup(void);
void other_gpio_setup(void);
void delay(volatile uint64_t value);

int main(void)
{
  rcc_setup();
  led_setup();
  other_gpio_setup();
  console_usart_setup();
  dynamixel_usart_setup();

  dynamixel2_set_torque_enable(MOTOR_ID, true);
  delay(100000);

  dynamixel2_clear_receive_buffer();
  printf("Ready\r\n");

  int32_t position;
  while (1)
  {
    dynamixel2_set_goal_position(MOTOR_ID, 10000);
    delay(10000000);

    position = dynamixel2_read_present_position(MOTOR_ID);
    printf("%li\r\n", position);
    delay(10000000);

    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN);

    dynamixel2_set_goal_position(MOTOR_ID, -10000);
    delay(10000000);

    position = dynamixel2_read_present_position(MOTOR_ID);
    printf("%li\r\n", position);
    delay(10000000);

    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN);
  }

  return 0;
}

void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_USART1);
  rcc_periph_clock_enable(RCC_USART2);
}

void console_usart_setup(void)
{
  /* Tx. */
  gpio_mode_setup(GPIO_CONSOLE_UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_CONSOLE_UART_TX_PIN);
  gpio_set_af(GPIO_CONSOLE_UART_TX_PORT, GPIO_AF7, GPIO_CONSOLE_UART_TX_PIN);

  /* Rx. */
  gpio_mode_setup(GPIO_CONSOLE_UART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_CONSOLE_UART_RX_PIN);
  gpio_set_af(GPIO_CONSOLE_UART_RX_PORT, GPIO_AF7, GPIO_CONSOLE_UART_RX_PIN);

  nvic_enable_irq(CONSOLE_USART_IRQ);

  usart_set_baudrate(CONSOLE_USART, CONSOLE_UART_BAUD_RATE);
  usart_set_databits(CONSOLE_USART, 8);
  usart_set_stopbits(CONSOLE_USART, USART_STOPBITS_1);
  usart_set_parity(CONSOLE_USART, USART_PARITY_NONE);
  usart_set_flow_control(CONSOLE_USART, USART_FLOWCONTROL_NONE);
  usart_set_mode(CONSOLE_USART, USART_MODE_TX_RX);

  usart_enable_rx_interrupt(CONSOLE_USART);

  usart_enable(CONSOLE_USART);
}

void dynamixel_usart_setup(void)
{
  /* Tx. */
  gpio_mode_setup(GPIO_DYNAMIXEL_UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_DYNAMIXEL_UART_TX_PIN);
  gpio_set_af(GPIO_DYNAMIXEL_UART_TX_PORT, GPIO_AF7, GPIO_DYNAMIXEL_UART_TX_PIN);

  /* Rx. */
  gpio_mode_setup(GPIO_DYNAMIXEL_UART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_DYNAMIXEL_UART_RX_PIN);
  gpio_set_af(GPIO_DYNAMIXEL_UART_RX_PORT, GPIO_AF7, GPIO_DYNAMIXEL_UART_RX_PIN);

  nvic_enable_irq(DYNAMIXEL_USART_IRQ);

  usart_set_baudrate(DYNAMIXEL_USART, DYNAMIXEL_UART_BAUD_RATE);
  usart_set_databits(DYNAMIXEL_USART, 8);
  usart_set_stopbits(DYNAMIXEL_USART, USART_STOPBITS_1);
  usart_set_parity(DYNAMIXEL_USART, USART_PARITY_NONE);
  usart_set_flow_control(DYNAMIXEL_USART, USART_FLOWCONTROL_NONE);
  usart_set_mode(DYNAMIXEL_USART, USART_MODE_TX_RX);

  usart_enable_rx_interrupt(DYNAMIXEL_USART);

  usart_enable(DYNAMIXEL_USART);
}

void led_setup(void)
{
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LED_PIN);

  gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN); /* LED off. */
}

void other_gpio_setup(void)
{
  gpio_mode_setup(GPIO_ENABLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_ENABLE_PIN);
  gpio_set_output_options(GPIO_ENABLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO_ENABLE_PIN);
}

void delay(volatile uint64_t value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void usart1_isr(void)
{
  if ((USART_SR(DYNAMIXEL_USART) & USART_SR_RXNE) != 0)
  {
    uint8_t indata = usart_recv(DYNAMIXEL_USART);
    dynamixel2_receive_callback(indata);
  }
}