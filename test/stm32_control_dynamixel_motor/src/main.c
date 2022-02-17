/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Control DYNAMIXEL motor with RS-485 UART.
 */

#include "printf.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define UART_BAUD_RATE (57600)

/* PA5 = D13, User-LED. */
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN (GPIO5)

/* PB3 = D3. */
#define GPIO_ENABLE_PORT (GPIOB)
#define GPIO_ENABLE_PIN (GPIO3)

/* PA0 = A0, UART4_TX. */
#define GPIO_UART_TX_PORT (GPIOA)
#define GPIO_UART_TX_PIN (GPIO0)

/* PA1 = A1, UART4_RX. */
#define GPIO_UART_RX_PORT (GPIOA)
#define GPIO_UART_RX_PIN (GPIO1)

#define GPIO_CONSOLE_UART_TX_PORT (GPIOA)
#define GPIO_CONSOLE_UART_TX_PIN (GPIO2)

#define GPIO_CONSOLE_UART_RX_PORT (GPIOA)
#define GPIO_CONSOLE_UART_RX_PIN (GPIO3)

uint8_t buffer[32] = {0};
uint8_t buffer_index = 0;
uint16_t buffer_packet_length = 0;
bool buffer_end = false;

int32_t present_position_decoder(uint8_t id, uint8_t *packet);
void clear_buffer(void);
void delay(uint64_t volatile value)
{
  while (value--)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_180MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_UART4);
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

  nvic_enable_irq(NVIC_USART2_IRQ);

  usart_set_baudrate(USART2, UART_BAUD_RATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  usart_enable_rx_interrupt(USART2);

  usart_enable(USART2);
}

void dynamixel_usart_setup(void)
{
  /* Tx. */
  gpio_mode_setup(GPIO_UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_UART_TX_PIN);
  gpio_set_af(GPIO_UART_TX_PORT, GPIO_AF8, GPIO_UART_TX_PIN);

  /* Rx. */
  gpio_mode_setup(GPIO_UART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_UART_RX_PIN);
  gpio_set_af(GPIO_UART_RX_PORT, GPIO_AF8, GPIO_UART_RX_PIN);

  nvic_enable_irq(NVIC_UART4_IRQ);

  usart_set_baudrate(UART4, UART_BAUD_RATE);
  usart_set_databits(UART4, 8);
  usart_set_stopbits(UART4, USART_STOPBITS_1);
  usart_set_parity(UART4, USART_PARITY_NONE);
  usart_set_flow_control(UART4, USART_FLOWCONTROL_NONE);
  usart_set_mode(UART4, USART_MODE_TX_RX);

  usart_enable_rx_interrupt(UART4);

  usart_enable(UART4);
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

/* Source: https://emanual.robotis.com/docs/en/dxl/crc/ */
uint16_t update_crc(uint16_t crc_accum,
                    uint8_t *data_blk_ptr,
                    uint16_t data_blk_size)
{
  uint16_t i, j;
  uint16_t crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

void weite_dynamixel_torque_enable(uint8_t id, bool enable)
{
  uint16_t packet_length = 13;
  uint8_t packet[packet_length];
  packet[0] = 0xFF; /* Header 1. */
  packet[1] = 0xFF; /* Hedaer 2. */
  packet[2] = 0xFD; /* Hedaer 3. */
  packet[3] = 0x00; /* Reserved. */

  packet[4] = id; /* Packer ID. */

  /* Length = paras + 3. */
  packet[5] = 0x06; /* Length 1 (Low). */
  packet[6] = 0x00; /* Lenget 2 (High). */

  packet[7] = 0x03; /* Instrucion. */

  /* Address. */
  packet[8] = (562 & 0xFF);
  packet[9] = ((562 >> 8) & 0xFF);

  /* Position. */
  packet[10] = enable ? 0x01 : 0x00;

  /* Calculating CRC. */
  uint16_t crc = update_crc(0, packet, packet_length - 2);

  /* CRC. */
  packet[11] = (crc & 0xFF);        /* CRC 1 (Low). */
  packet[12] = ((crc >> 8) & 0xFF); /* CRC 2 (High). */

  gpio_set(GPIO_ENABLE_PORT, GPIO_ENABLE_PIN);
  for (int i = 0; i < packet_length; i++)
  {
    usart_send_blocking(UART4, packet[i]);
  }

  /* Wait for transmission complete. */
  while (!(USART_SR(UART4) & USART_SR_TC))
  {
    /* Do nothing. */
  }
  gpio_clear(GPIO_ENABLE_PORT, GPIO_ENABLE_PIN);
}

void write_dynamixel_position(uint8_t id, int32_t position)
{
  uint16_t packet_length = 16;
  uint8_t packet[packet_length];
  packet[0] = 0xFF; /* Header 1. */
  packet[1] = 0xFF; /* Hedaer 2. */
  packet[2] = 0xFD; /* Hedaer 3. */
  packet[3] = 0x00; /* Reserved. */

  packet[4] = id; /* Packer ID. */

  /* Length = paras + 3. */
  packet[5] = 0x09; /* Length 1 (Low). */
  packet[6] = 0x00; /* Lenget 2 (High). */

  packet[7] = 0x03; /* Instrucion. */

  /* Address. */
  packet[8] = 0x54;
  packet[9] = 0x02;

  /* Position. */
  packet[10] = (position & 0xFF);
  packet[11] = ((position >> 8) & 0xFF);
  packet[12] = ((position >> 16) & 0xFF);
  packet[13] = ((position >> 24) & 0xFF);

  /* Calculating CRC. */
  uint16_t crc = update_crc(0, packet, packet_length - 2);

  /* CRC. */
  packet[14] = (crc & 0xFF);        /* CRC 1 (Low). */
  packet[15] = ((crc >> 8) & 0xFF); /* CRC 2 (High). */

  gpio_set(GPIO_ENABLE_PORT, GPIO_ENABLE_PIN);
  for (int i = 0; i < packet_length; i++)
  {
    usart_send_blocking(UART4, packet[i]);
  }

  /* Wait for transmission complete. */
  while (!(USART_SR(UART4) & USART_SR_TC))
  {
    /* Do nothing. */
  }

  gpio_clear(GPIO_ENABLE_PORT, GPIO_ENABLE_PIN);
}

int32_t read_dynamixel_present_position(uint8_t id)
{
  uint16_t packet_length = 14;
  uint8_t packet[packet_length];
  packet[0] = 0xFF; /* Header 1. */
  packet[1] = 0xFF; /* Hedaer 2. */
  packet[2] = 0xFD; /* Hedaer 3. */
  packet[3] = 0x00; /* Reserved. */

  packet[4] = id; /* Packer ID. */

  /* Length = paras + 3. */
  packet[5] = 0x07; /* Length 1 (Low). */
  packet[6] = 0x00; /* Lenget 2 (High). */

  packet[7] = 0x02; /* Instrucion. */

  /* Address. */
  packet[8] = (611 & 0xFF);
  packet[9] = ((611 >> 8) & 0xFF);

  packet[10] = 0x04;
  packet[11] = 0x00;

  /* Calculating CRC. */
  uint16_t crc = update_crc(0, packet, packet_length - 2);

  /* CRC. */
  packet[packet_length - 2] = (crc & 0xFF);        /* CRC 1 (Low). */
  packet[packet_length - 1] = ((crc >> 8) & 0xFF); /* CRC 2 (High). */

  gpio_set(GPIO_ENABLE_PORT, GPIO_ENABLE_PIN);
  clear_buffer();
  for (int i = 0; i < packet_length; i++)
  {
    usart_send_blocking(UART4, packet[i]);
  }

  /* Wait for transmission complete. */
  while (!(USART_SR(UART4) & USART_SR_TC))
  {
    /* Do nothing. */
  }
  gpio_clear(GPIO_ENABLE_PORT, GPIO_ENABLE_PIN);

  while (!buffer_end)
  {
    /* code */
  }
  int32_t position = present_position_decoder(id, buffer);
  clear_buffer();
  return position;
}

int32_t present_position_decoder(uint8_t id, uint8_t *packet)
{
  bool header1 = (packet[0] == 0xFF);
  bool header2 = (packet[1] == 0xFF);
  bool header3 = (packet[2] == 0xFD);
  bool rsrv = (packet[3] == 0x00);
  bool packrt_id = (packet[4] == id);
  bool length1 = (packet[5] == 0x08);
  bool length2 = (packet[6] == 0x00);
  bool inst = (packet[7] == 0x55);
  bool err = (packet[8] == 0x00);
  int32_t position;

  if ((header1 && header2 && header3 && rsrv && packrt_id && length1 && length2 && inst) == true)
  {
    uint8_t p1 = packet[9];
    uint8_t p2 = packet[10];
    uint8_t p3 = packet[11];
    uint8_t p4 = packet[12];
    position = p1 | (p2 << 8) | (p3 << 16) | (p4 << 24);
  }

  return position;
}

void clear_buffer(void)
{
  for (int i = 0; i < 32; i++)
  {
    buffer[i] = 0x00;
  }
  buffer_index = 0;
  buffer_end = false;
  buffer_packet_length = 0;
}

int main(void)
{
  rcc_setup();
  led_setup();
  other_gpio_setup();
  console_usart_setup();
  dynamixel_usart_setup();

  weite_dynamixel_torque_enable(1, true);
  delay(1000000);

  printf("Ready\r\n");

  int32_t position;
  while (1)
  {
    printf(".");
    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN);
    write_dynamixel_position(1, -10000);
    delay(10000000);
    position = read_dynamixel_present_position(1);
    // printf("%ld", position);
    printf(".");
    delay(10000000);

    gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN);
    write_dynamixel_position(1, 10000);
    delay(10000000);
    position = read_dynamixel_present_position(1);
    printf("%ld", position);
    delay(10000000);
  }

  return 0;
}

void uart4_isr(void)
{
  if ((USART_SR(UART4) & USART_SR_RXNE) != 0)
  {
    uint8_t indata = usart_recv(UART4);
    if (indata == 0xFF)
    {
      if (buffer[0] != 0xFF && buffer[1] != 0xFF && buffer[2] != 0xFD)
      {
        buffer_index = 0;
        buffer_end = false;
      }
      else if (buffer[0] == 0xFF)
      {
        if (buffer[2] != 0xFD)
        {
          buffer_index = 1;
        }
      }
      else if (buffer_end)
      {
        buffer_index = 0;
        buffer_end = false;
      }
    }

    buffer[buffer_index] = indata;
    if (buffer_index == 6)
    {
      buffer_packet_length = (buffer[5] | (buffer[6] << 8)) + 7;
    }
    else if (buffer_index == buffer_packet_length-1 && buffer_index > 6)
    {
      buffer_end = true;
    }
    buffer_index++;
  }
}