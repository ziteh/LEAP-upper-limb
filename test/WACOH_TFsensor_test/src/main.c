/**
 * @file   main.c
 * @brief  WACOH WEF-6A200-4-RCD Capacitive 6-axis force sensor test program.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define USART_DEBUG_BAUDRATE (19200)
#define USART_SENSOR_BAUDRATE (921600)

#define ZERO_POINT_OUTPUT (8192) /* 8192+-655. */

/* USART2. */
#define RCC_USART_DEBUG_TXRX_GPIO (RCC_GPIOA)
#define GPIO_USART_DEBUG_TXRX_PORT (GPIOA)
#define GPIO_USART_DEBUG_TX_PIN (GPIO2) /* D1. */
#define GPIO_USART_DEBUG_RX_PIN (GPIO3) /* D0. */
#define GPIO_USART_DEBUG_AF (GPIO_AF7)  /* Ref: Table-11 in DS10693. */

/* USART3. */
#define RCC_USART_SENSOR_TXRX_GPIO (RCC_GPIOC)
#define GPIO_USART_SENSOR_TXRX_PORT (GPIOC)
#define GPIO_USART_SENSOR_TX_PIN (GPIO10)
#define GPIO_USART_SENSOR_RX_PIN (GPIO11)
#define GPIO_USART_SENSOR_AF (GPIO_AF7)

#define RCC_LED_GPIO (RCC_GPIOA)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5) /* D13. */

uint8_t sensor_data_buffer[25];
uint8_t sensor_data_buffer_index = 0;

static uint8_t hex_ascii_to_int(uint8_t ascii)
{
  /*
   * ASCII=HEX:
   * 0=0x30, 1=0x31 ... 9=0x39
   * A=0x41, B=0x42 ... F=0x46
   */

  if (ascii <= 0x39 && ascii >= 0x30)
  {
    return ascii - 0x30;
  }
  else if (ascii <= 0x46 && ascii >= 0x41)
  {
    return (ascii - 0x41) + 10;
  }
  else
  {
    return 0xFF; /* Error. */
  }
}

static uint16_t hex_ascii_to_int_4(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
  uint8_t n1 = hex_ascii_to_int(a1); /* LSB. */
  uint8_t n2 = hex_ascii_to_int(a2);
  uint8_t n3 = hex_ascii_to_int(a3);
  uint8_t n4 = hex_ascii_to_int(a4);

  return ((n1 << 12) + (n2 << 8) + (n3 << 4) + n4) & 0x3FFF;
}

static int parse_sensor_data(uint8_t *raw_data,
                             uint8_t *N,
                             int16_t *Fx,
                             int16_t *Fy,
                             int16_t *Fz,
                             int16_t *Mx,
                             int16_t *My,
                             int16_t *Mz)
{
  *N = hex_ascii_to_int(raw_data[0]);

  *Fx = hex_ascii_to_int_4(raw_data[1], raw_data[2], raw_data[3], raw_data[4]) - ZERO_POINT_OUTPUT;
  *Fy = hex_ascii_to_int_4(raw_data[5], raw_data[6], raw_data[7], raw_data[8]) - ZERO_POINT_OUTPUT;
  *Fz = hex_ascii_to_int_4(raw_data[9], raw_data[10], raw_data[11], raw_data[12]) - ZERO_POINT_OUTPUT;

  *Mx = hex_ascii_to_int_4(raw_data[13], raw_data[14], raw_data[15], raw_data[16]) - ZERO_POINT_OUTPUT;
  *My = hex_ascii_to_int_4(raw_data[17], raw_data[18], raw_data[19], raw_data[20]) - ZERO_POINT_OUTPUT;
  *Mz = hex_ascii_to_int_4(raw_data[21], raw_data[22], raw_data[23], raw_data[24]) - ZERO_POINT_OUTPUT;

  return 0;
}

static void clear_buffer()
{
  sensor_data_buffer_index = 0;
  for (int i = 0; i < 26; i++)
  {
    sensor_data_buffer[i] = 0x00;
  }
}

static void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

static void rcc_setup(void)
{
  rcc_periph_clock_enable(RCC_LED_GPIO);
  rcc_periph_clock_enable(RCC_USART_DEBUG_TXRX_GPIO);
  rcc_periph_clock_enable(RCC_USART_SENSOR_TXRX_GPIO);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_USART3);
}

static void usart_debug_setup(void)
{
  /* Set USART-Tx & Rx pin to alternate function. */
  gpio_mode_setup(GPIO_USART_DEBUG_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_DEBUG_TX_PIN | GPIO_USART_DEBUG_RX_PIN);

  gpio_set_af(GPIO_USART_DEBUG_TXRX_PORT,
              GPIO_USART_DEBUG_AF,
              GPIO_USART_DEBUG_TX_PIN | GPIO_USART_DEBUG_RX_PIN);

  /* Config USART params. */
  usart_set_baudrate(USART2, USART_DEBUG_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  /* Setup interrupt. */
  usart_enable_rx_interrupt(USART2); /* Enable receive interrupt. */
  nvic_enable_irq(NVIC_USART2_IRQ);

  usart_enable(USART2);
}

static void usart_sensor_setup(void)
{
  /* Set USART-Tx & Rx pin to alternate function. */
  gpio_mode_setup(GPIO_USART_SENSOR_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_SENSOR_TX_PIN | GPIO_USART_SENSOR_RX_PIN);

  gpio_set_af(GPIO_USART_SENSOR_TXRX_PORT,
              GPIO_USART_SENSOR_AF,
              GPIO_USART_SENSOR_TX_PIN | GPIO_USART_SENSOR_RX_PIN);

  /* Config USART params. */
  usart_set_baudrate(USART3, USART_SENSOR_BAUDRATE);
  usart_set_databits(USART3, 8);
  usart_set_stopbits(USART3, USART_STOPBITS_1);
  usart_set_parity(USART3, USART_PARITY_NONE);
  usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART3, USART_MODE_TX_RX);

  /* Setup interrupt. */
  usart_enable_rx_interrupt(USART3); /* Enable receive interrupt. */
  nvic_enable_irq(NVIC_USART3_IRQ);

  usart_enable(USART3);
}

static void led_setup(void)
{
  /* Set LED pin to output push-pull. */
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_LED_PIN);
}

int main(void)
{
  rcc_setup();
  led_setup();
  usart_debug_setup();
  usart_sensor_setup();

  usart_send_blocking(USART2, 'R');
  usart_send_blocking(USART2, 'D');
  usart_send_blocking(USART2, 'Y');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');

  /* Halt. */
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
  uint8_t indata = usart_recv(USART2); /* Read received data. */
  usart_send_blocking(USART3, indata); /* Send back. */

  /* Clear RXNE(Read data register not empty) flag at SR(Status register). */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}

/**
 * @brief USART3 Interrupt service routine.
 */
void usart3_isr(void)
{
  uint8_t indata = usart_recv(USART3); /* Read received data. */

  if (indata == 0x0A) /* LF. */
  {
    uint8_t N;
    int16_t Fx;
    int16_t Fy;
    int16_t Fz;
    int16_t Mx;
    int16_t My;
    int16_t Mz;

    parse_sensor_data(sensor_data_buffer, &N, &Fx, &Fy, &Fz, &Mx, &My, &Mz);
    clear_buffer();
  }
  else if (indata == 0x0D) /* CR. */
  {
  }
  else
  {
    sensor_data_buffer[sensor_data_buffer_index] = indata;
    sensor_data_buffer_index++;
  }

  /* Clear RXNE(Read data register not empty) flag at SR(Status register). */
  USART_SR(USART3) &= ~USART_SR_RXNE;
}
