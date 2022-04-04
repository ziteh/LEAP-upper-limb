/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Example for MCP2515 SPI interface CAN-Bus module.
 *         Can't work.
 * @remark Reference: https://ithelp.ithome.com.tw/articles/10284376
 */

#define TX
// #define RX

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include "mcp2515.h"
#include "mcp2515_register_commands.h"

#define UART_BAUDRATE (9600)

#define GPIO_UART_TX_PORT (GPIOA)
#define GPIO_UART_TX_PIN (GPIO2)

#define GPIO_UART_RX_PORT (GPIOA)
#define GPIO_UART_RX_PIN (GPIO3)

#define GPIO_LED_PORT (GPIOB)
#define GPIO_LED_PIN (GPIO13)

#define GPIO_SPI_SCK_PORT (GPIOB)
#define GPIO_SPI_SCK_PIN (GPIO13)

#define GPIO_SPI_MISO_PORT (GPIOB)
#define GPIO_SPI_MISO_PIN (GPIO14)

#define GPIO_SPI_MOSI_PORT (GPIOB)
#define GPIO_SPI_MOSI_PIN (GPIO15)

#define GPIO_SPI_CS_PORT (GPIOB)
#define GPIO_SPI_CS_PIN (GPIO6)

void delay(uint32_t volatile value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void spi_start(void)
{
  // spi_set_nss_high(SPI3);
  // spi_set_nss_low(SPI3);
  // gpio_set(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
  gpio_clear(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
}

void spi_end(void)
{
  /* Wait for transfer finished. */
  while (!(SPI_SR(SPI2) & SPI_SR_TXE))
  { /* None. */
  }
  while (SPI_SR(SPI2) & SPI_SR_BSY)
  { /* None. */
  }

  // spi_set_nss_high(SPI3);
  // spi_set_nss_low(SPI3);
  gpio_set(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
  // gpio_clear(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
}

static void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse8mhz_configs[RCC_CLOCK_HSE8_72MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_SPI2);
}

static void led_setup(void)
{
  gpio_mode_setup(GPIO_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LED_PIN);
  gpio_set_output_options(GPIO_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LED_PIN);

  gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN); // LED off.
}

static void usart_setup(void)
{
  gpio_mode_setup(GPIO_UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_UART_TX_PIN);
  gpio_set_af(GPIO_UART_TX_PORT, GPIO_AF7, GPIO_UART_TX_PIN);

  gpio_mode_setup(GPIO_UART_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_UART_RX_PIN);

  usart_set_baudrate(USART2, UART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX);

  usart_enable(USART2);
}

static spi_setup(void)
{
  gpio_mode_setup(GPIO_SPI_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPI_CS_PIN);
  gpio_set_output_options(GPIO_SPI_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_SPI_CS_PIN);
  // gpio_mode_setup(GPIO_SPI_CS_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_SPI_CS_PIN);
  // gpio_set_af(GPIO_SPI_CS_PORT, GPIO_AF6, GPIO_SPI_CS_PIN);

  gpio_mode_setup(GPIO_SPI_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SPI_SCK_PIN);
  gpio_set_af(GPIO_SPI_SCK_PORT, GPIO_AF5, GPIO_SPI_SCK_PIN);

  gpio_mode_setup(GPIO_SPI_MOSI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SPI_MOSI_PIN);
  gpio_set_af(GPIO_SPI_MOSI_PORT, GPIO_AF5, GPIO_SPI_MOSI_PIN);

  gpio_mode_setup(GPIO_SPI_MISO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SPI_MISO_PIN);
  gpio_set_af(GPIO_SPI_MISO_PORT, GPIO_AF5, GPIO_SPI_MISO_PIN);

  spi_reset(SPI2);
  spi_init_master(SPI2,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64, // SCK: 562.4 kHz
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1,
                  SPI_CR1_MSBFIRST);
  spi_fifo_reception_threshold_8bit(SPI2);
  spi_set_data_size(SPI2, SPI_CR2_DS_8BIT);
  // spi_set_full_duplex_mode(SPI3);
  // spi_set_master_mode(SPI3);
  // spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_32);
  // spi_send_msb_first(SPI3);
  // spi_set_clock_phase_0(SPI3);
  // spi_set_clock_polarity_0(SPI3);

  // spi_enable_software_slave_management(SPI3);
  // spi_set_nss_high(SPI3);
  // spi_set_nss_low(SPI3);
  // gpio_set(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
  // gpio_clear(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN);
  // spi_end();
  // spi_enable_ss_output(SPI3);

  spi_enable(SPI2);
}

uint8_t mcp2515_read_byte(uint8_t address)
{
  uint8_t data;
  spi_start();

  spi_send8(SPI3, CAN_READ);
  spi_send8(SPI3, address);
  // spi_read8(SPI3);
  spi_send8(SPI3, 0);
  data = spi_read8(SPI3);

  spi_end();
  return data;
}

void mcp2515_write_byte(uint8_t address, uint8_t data)
{
  spi_start();

  spi_send8(SPI3, CAN_WRITE);
  spi_send8(SPI3, address);
  spi_send8(SPI3, data);

  spi_end();
}

void mcp2515_write_buffer(uint8_t *buffer, uint8_t length)
{
  while ((mcp2515_read_byte(TXB0CTRL) & 0x08) == 0x08)
  {
    __asm__("nop");
  }

  for (int i = 0; i < length; i++)
  {
    mcp2515_write_byte(TXB0D0 + i, buffer[i]);
  }

  mcp2515_write_byte(TXB0DLC, 8);
  mcp2515_write_byte(TXB0CTRL, 0x08);
}

uint8_t mcp2515_read_buffer(uint8_t *buffer)
{
  uint8_t length = 0;

  if (mcp2515_read_byte(CANINTF) & 0x01)
  {
    length = mcp2515_read_byte(RXB0DLC);
    for (int i = 0; i < length; i++)
    {
      buffer[i] = mcp2515_read_byte(RXB0D0 + i);
    }
  }

  mcp2515_write_byte(CANINTF, 0x00);
  return length;
}

int mcp2515_set_config_mode(void)
{
  mcp2515_write_byte(CANCTRL, 0x80);
  uint8_t t = 10;
  do
  {
    if ((mcp2515_read_byte(CANSTAT) & 0xE0) == 0x80)
    {
      return 1;
    }
    t--;
    delay(10000);
  } while (t > 0);
  usart_send_blocking(USART2, '0');
  return 0;
}

int mcp2515_set_normal_mode(void)
{
  mcp2515_write_byte(CANCTRL, 0x00);
  uint8_t t = 10;
  do
  {
    if ((mcp2515_read_byte(CANSTAT) & 0xE0) == 0x00)
    {
      return 1;
    }
    t--;
    delay(10000);
  } while (t > 0);
  usart_send_blocking(USART2, '0');
  return 0;
}

int mcp2515_set_loop_mode(void)
{
  mcp2515_write_byte(CANCTRL, 0x40);
  uint8_t t = 10;
  do
  {
    if ((mcp2515_read_byte(CANSTAT) & 0xE0) == 0x40)
    {
      return 1;
    }
    t--;
    delay(10000);
  } while (t > 0);
  usart_send_blocking(USART2, '0');
  return 0;
}

void mcp2515_reset(void)
{
  spi_start();
  spi_send8(SPI3, CAN_RESET);
  spi_end();
}

// int mcp2515_init(void)
// {
//   mcp2515_reset();
//   mcp2515_set_config_mode();

//   // TQ.
//   mcp2515_write_byte(CNF1, 0x03);
//   mcp2515_write_byte(CNF2, 0x80 | PHSEG1_3TQ | PRSEG_1TQ);
//   mcp2515_write_byte(CNF3, PHSEG2_3TQ);

//   // ID.
//   mcp2515_write_byte(TXB0SIDH, 0x11);
//   mcp2515_write_byte(TXB0SIDL, 0xE0);

//   // Clear Receive Register.
//   mcp2515_write_byte(RXB0SIDH, 0x00);
//   mcp2515_write_byte(RXB0SIDL, 0x00);
//   mcp2515_write_byte(RXB0EID8, 0x00);
//   mcp2515_write_byte(RXB0EID0, 0x00);
//   mcp2515_write_byte(RXB0CTRL, 0x40);
//   mcp2515_write_byte(RXB0DLC, DLC_8);

//   // Filter.
//   mcp2515_write_byte(RXF0SIDH, 0x11);
//   mcp2515_write_byte(RXF0SIDL, 0xE0);

//   // Mask.
//   mcp2515_write_byte(RXM0SIDH, 0x00);
//   mcp2515_write_byte(RXM0SIDL, 0x00);

//   // Interrupt.
//   mcp2515_write_byte(CANINTE, 0x01);
//   mcp2515_write_byte(CANINTF, 0x00);

//   uint8_t state = mcp2515_set_loop_mode();
//   return state;
// }

int main(void)
{
  rcc_setup();
  // led_setup();
  usart_setup();
  spi_setup();

  mcp2515_init();
  // int a = mcp2515_init();

  // if (a == 1)
  // {
  //   /* Ready. */
    usart_send_blocking(USART2, 'R');
    usart_send_blocking(USART2, 'e');
    usart_send_blocking(USART2, 'a');
    usart_send_blocking(USART2, 'd');
    usart_send_blocking(USART2, 'y');
  // }
  // else
  // {
  //   /* Error. */
  //   usart_send_blocking(USART2, 'E');
  //   usart_send_blocking(USART2, 'r');
  //   usart_send_blocking(USART2, 'r');
  //   usart_send_blocking(USART2, 'o');
  //   usart_send_blocking(USART2, 'r');
  //   // while (1)
  //   { /* --. */
  //   }
  // }
  // usart_send_blocking(USART2, '\r');
  // usart_send_blocking(USART2, '\n');
  //
  // uint8_t rx_data[8] = {0};
  // uint8_t tx_data[8] = {0x01,
  // 0x02,
  // 0x03,
  // 0x04,
  // 0x05,
  // 0x06,
  // 0x07,
  // 0xFF};

  struct can_frame rx_can_data;
  rx_can_data.can_dlc = 8;
  struct can_frame tx_can_data = {.can_id = 0x00,
                                  .can_dlc = 8,
                                  .data[0] = 0x00,
                                  .data[1] = 0x01,
                                  .data[2] = 0x02,
                                  .data[3] = 0x03,
                                  .data[4] = 0x04,
                                  .data[5] = 0x05,
                                  .data[6] = 0x06,
                                  .data[7] = 0x07};
  while (1)
  {
#if defined TX
    delay(100000);
    sendMessage(TXB0, &tx_can_data);
#elif defined RX
    delay(2000);
    uint8_t length = 0;
    readMessage(RXB0, &rx_can_data);
    length = rx_can_data.can_dlc;

    for (int i = 0; i < length; i++)
    {
      usart_send_blocking(USART2, rx_can_data.data[i]);
    }
#endif
  }

  return 0;
}