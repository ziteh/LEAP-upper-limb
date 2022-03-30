/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Ref: https://visualgdb.com/tutorials/arm/stm32/spi/
 */

// #define TX
#define RX

#include <stm32f3xx_hal.h>

#define UART_BAUDRATE (9600)

#define GPIO_UART_TX_PORT (GPIOA)
#define GPIO_UART_TX_PIN (GPIO_PIN_2)

#define GPIO_UART_RX_PORT (GPIOA)
#define GPIO_UART_RX_PIN (GPIO_PIN_3)

#define GPIO_LED_PORT (GPIOB)
#define GPIO_LED_PIN (GPIO_PIN_13)

#define GPIO_SPI_SCK_PORT (GPIOB)
#define GPIO_SPI_SCK_PIN (GPIO_PIN_13)

#define GPIO_SPI_MISO_PORT (GPIOB)
#define GPIO_SPI_MISO_PIN (GPIO_PIN_14)

#define GPIO_SPI_MOSI_PORT (GPIOB)
#define GPIO_SPI_MOSI_PIN (GPIO_PIN_15)

#define GPIO_SPI_CS_PORT (GPIOB)
#define GPIO_SPI_CS_PIN (GPIO_PIN_6)

SPI_HandleTypeDef SPI_InitStruct;
UART_HandleTypeDef USART;

static void clock_init(void)
{
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __USART2_CLK_ENABLE();
  __SPI2_CLK_ENABLE();
}

static void usart_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_UART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIO_UART_TX_PORT, &GPIO_InitStruct);

  USART.Instance = USART2;
  USART.Init.BaudRate = UART_BAUDRATE;
  USART.Init.WordLength = UART_WORDLENGTH_8B;
  USART.Init.StopBits = UART_STOPBITS_1;
  USART.Init.Parity = UART_PARITY_NONE;
  USART.Init.Mode = UART_MODE_TX;
  HAL_UART_Init(&USART);
}

static void spi_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_SPI_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_SPI_CS_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIO_SPI_SCK_PORT, &GPIO_InitStruct);

  SPI_InitStruct.Instance = SPI2;
  SPI_InitStruct.Init.Mode = SPI_MODE_MASTER;
  SPI_InitStruct.Init.Direction = SPI_DIRECTION_2LINES;
  SPI_InitStruct.Init.DataSize = SPI_DATASIZE_8BIT;
  SPI_InitStruct.Init.CLKPolarity = SPI_POLARITY_LOW;
  SPI_InitStruct.Init.CLKPhase = SPI_PHASE_1EDGE;
  SPI_InitStruct.Init.NSS = SPI_NSS_SOFT;
  SPI_InitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  SPI_InitStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;
  SPI_InitStruct.Init.TIMode = SPI_TIMODE_DISABLE;
  SPI_InitStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&SPI_InitStruct);

  spi_end();
}

void mcp2515_reset()
{
  spi_start();

  uint8_t buffer[] = {0xC0};
  HAL_SPI_Transmit(&SPI_InitStruct, buffer, 1, 10);

  spi_end();
}

void mcp2515_write_byte(uint8_t address, uint8_t data)
{
  spi_start();

  uint8_t buffer[] = {0x02, address, data};
  HAL_SPI_Transmit(&SPI_InitStruct, buffer, 3, 10);

  spi_end();
}

uint8_t mcp2515_read_byte(uint8_t address)
{
  spi_start();

  uint8_t buffer[] = {0x03, address};
  HAL_SPI_Transmit(&SPI_InitStruct, buffer, 2, 10);

  uint8_t rec[] = {0x00};
  HAL_SPI_Receive(&SPI_InitStruct, rec, 1, 10);

  spi_end();
  return rec[0];
}

uint8_t mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{
  spi_start();

  uint8_t init_data[] = {0x05, address, mask, data};
  HAL_SPI_Transmit(&SPI_InitStruct, init_data, 4, 10);

  spi_end();
}

uint8_t mcp2515_set_mode(uint8_t reqop_3bit)
{
  uint8_t mask = 0xE0;
  uint8_t mode = reqop_3bit << 5;
  mcp2515_bit_modify(0x0F, mask, mode);
  uint8_t state = mcp2515_read_byte(0x0E);
  return (state & mask) == mode ? 0 : 1;
}

uint8_t mcp2515_set_normal_mode()
{
  return mcp2515_set_mode(0);
}

uint8_t mcp2515_set_configuration_mode()
{
  return mcp2515_set_mode(4);
}

uint8_t mcp2515_set_loopback_mode()
{
  return mcp2515_set_mode(2);
}

void mcp2515_write_can(uint8_t *message, uint8_t length)
{
  while ((mcp2515_read_byte(0x30) & 0x80) != 0x00)
  {
    /* Do nothing. */
  }

  for (int i = 0; i < length; i++)
  {
    mcp2515_write_byte(0x36 + i, message[i]);
  }

  mcp2515_bit_modify(0x35, 0x0F, length);
  mcp2515_bit_modify(0x30, 0x08, 0x08);
}

uint8_t mcp2515_read_can(uint8_t *message)
{
  uint8_t length = 0;
  uint8_t state = mcp2515_read_byte(0x2C);
  if ((state & 0x01) != 0x00)
  {
    length = mcp2515_read_byte(0x65) & 0x0F;
    for (int i = 0; i < length; i++)
    {
      message[i] = mcp2515_read_byte(0x66 + i);
    }
    mcp2515_write_byte(0x2C, 0x00);
  }
  return length;
}

void spi_start(void)
{
  HAL_GPIO_WritePin(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN, GPIO_PIN_RESET);
}

void spi_end(void)
{
  HAL_GPIO_WritePin(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN, GPIO_PIN_SET);
}

uint8_t rx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                      0xFF, 0xFF, 0xFF, 0xFF};
uint8_t tx_data[8] = {0x00, 0x01, 0x02, 0x03,
                      0x04, 0x05, 0x06, 0x07};

void mcp2515_init()
{
  mcp2515_reset();
  mcp2515_set_configuration_mode();

  // Clear RX reg.
  mcp2515_write_byte(0x61, 0x00);
  mcp2515_write_byte(0x62, 0x00);
  mcp2515_write_byte(0x63, 0x00);
  mcp2515_write_byte(0x64, 0x00);
  mcp2515_write_byte(0x60, 0x40);
  mcp2515_write_byte(0x65, 8);

  // ID.
  mcp2515_write_byte(0x31, 0x11);
  mcp2515_write_byte(0x32, 0xE0);

  // Fillter.
  mcp2515_write_byte(0x00, 0x11);
  mcp2515_write_byte(0x01, 0xE0);

  // Mask.
  mcp2515_write_byte(0x20, 0x00);
  mcp2515_write_byte(0x21, 0x00);

  // Interrupt.
  mcp2515_write_byte(0x2B, 0x01);
  mcp2515_write_byte(0x2C, 0x00);

  // mcp2515_set_normal_mode();
  mcp2515_set_loopback_mode();
}

int main(void)
{
  HAL_Init();
  clock_init();
  usart_init();
  spi_init();

  mcp2515_init();

  uint8_t usart_data[] = {0xFF, 0x00, 0xFF, 0x00};
  HAL_UART_Transmit(&USART, usart_data, 4, HAL_MAX_DELAY);

  HAL_Delay(50);

  while (1)
  {
    // spi_start();

    // #if defined TX
    HAL_Delay(1000);
    mcp2515_write_can(tx_data, 8);
    // #elif defined RX
    HAL_Delay(100);
    uint8_t length = mcp2515_read_can(rx_data);
    if (length > 0)
    {
      HAL_UART_Transmit(&USART, rx_data, 8, HAL_MAX_DELAY);
    }
    // #endif

    // spi_end();
  }

  return 0;
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}