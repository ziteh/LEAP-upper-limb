/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Testing CAN Bus on STM32
 */

#include <stddef.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>

#define LED_PORT (GPIOB)
#define LED_PIN (GPIO13)

#define CAN_TX_PORT (GPIOB)
#define CAN_TX_PIN (GPIO9)

#define CAN_RX_PORT (GPIOB)
#define CAN_RX_PIN (GPIO8)

#define UART_TX_PORT (GPIOA)
#define UART_TX_PIN (GPIO2)

#define UART_RX_PORT (GPIOA)
#define UART_RX_PIN (GPIO3)

#define UART_BAUDRATE (9600)

void rcc_setup(vold)
{
  rcc_clock_setup_pll(&rcc_hse8mhz_configs[RCC_CLOCK_HSE8_72MHZ]);

  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_CAN);
  rcc_periph_clock_enable(RCC_CAN1);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_SYSCFG);
}

void led_setup(void)
{
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
  gpio_set_output_options(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED_PIN);
  gpio_clear(LED_PORT, LED_PIN);
}

void can_setup(void)
{
  gpio_mode_setup(CAN_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CAN_TX_PIN);
  gpio_set_af(CAN_TX_PORT, GPIO_AF9, CAN_TX_PIN);

  gpio_mode_setup(CAN_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, CAN_RX_PIN);

  nvic_enable_irq(NVIC_USB_LP_CAN1_RX0_IRQ);
  nvic_set_priority(NVIC_USB_LP_CAN1_RX0_IRQ, 1);

  can_reset(CAN1);
  if (can_init(CAN1,
               false,
               true,
               false,
               false,
               false,
               false,
               CAN_BTR_SJW_1TQ,
               CAN_BTR_TS1_3TQ,
               CAN_BTR_TS2_4TQ,
               12,
               false,
               false))
  {
    // Die decause failed to initialize.
    gpio_set(LED_PORT, LED_PIN);
    usart_send_blocking(USART2, 'E');
    while (1)
    {
      __asm__("nop");
    }
  }

  can_filter_id_list_32bit_init(0,
                                0,
                                0,
                                0,
                                true);

  can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void uart_setup(void)
{
  gpio_mode_setup(UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_TX_PIN);
  gpio_set_af(UART_TX_PORT, GPIO_AF7, UART_TX_PIN);

  gpio_mode_setup(UART_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, UART_RX_PIN);

  usart_set_baudrate(USART2, UART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  usart_enable(USART2);
}

int main(void)
{
  rcc_setup();
  led_setup();
  uart_setup();
  can_setup();

  /* Ready. */
  usart_send_blocking(USART2, 'R');
  usart_send_blocking(USART2, 'e');
  usart_send_blocking(USART2, 'a');
  usart_send_blocking(USART2, 'd');
  usart_send_blocking(USART2, 'y');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  while (1)
  {
    /* Halt. */
  }

  return 0;
}

void sys_tick_handler(void)
{
  static int temp32 = 0;
  static uint8_t data[8] = {0, 1, 2, 0, 0, 0, 0, 0};

  if (++temp32 != 1000)
  {
    return;
  }

  temp32 = 0;
  data[0]++;
  if (can_transmit(CAN1,
                   0,
                   false,
                   false,
                   8,
                   data) == -1)
  {
    gpio_toggle(LED_PORT, LED_PIN);
  }
}

void usb_lp_can_rx0_isr(void)
{
  uint32_t id;
  bool ext, rtr;
  uint8_t fmi, length, data[8];

  can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &length, data, NULL);

  usart_send_blocking(USART2, data[0]);
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  can_fifo_release(CAN1, 0);
}