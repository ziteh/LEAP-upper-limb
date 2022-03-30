/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Ref: https://visualgdb.com/tutorials/arm/stm32/spi/
 */

#define TX
// #define RX

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

static void clock_init(void)
{
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __SPI2_CLK_ENABLE();
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

void spi_start(void)
{
  HAL_GPIO_WritePin(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN, GPIO_PIN_RESET);
}

void spi_end(void)
{
  HAL_GPIO_WritePin(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN, GPIO_PIN_SET);
}

uint8_t rx_data[8] = {0};
uint8_t tx_data[8] = {0x00, 0x01, 0x02, 0x03,
                      0x04, 0x05, 0x06, 0x07};

int main(void)
{
  HAL_Init();
  clock_init();
  spi_init();

  spi_start();

  uint8_t init_data[] = {0xC0, 0x05, 0x0F, 0xE0, 0x00};
  HAL_SPI_Transmit(&SPI_InitStruct, init_data, 5, HAL_MAX_DELAY);

  uint8_t d[] = {0x03, 0x0E};
  HAL_SPI_Transmit(&SPI_InitStruct, d, 2, HAL_MAX_DELAY);
  HAL_SPI_Receive(&SPI_InitStruct, rx_data, 2, HAL_MAX_DELAY);

  spi_end();
  HAL_Delay(50);

  while (1)
  {
    spi_start();

    for (int i = 0; i < 8; i++)
    {
      uint8_t date[] = {0x02, (0x36 + i), tx_data[i]};
      HAL_SPI_Transmit(&SPI_InitStruct, date, 3, HAL_MAX_DELAY);
    }

    uint8_t d[] = {0x02, 0x35, 0x08, 0x02, 0x30, 0x08};
    HAL_SPI_Transmit(&SPI_InitStruct, d, 6, HAL_MAX_DELAY);

    spi_end();
    HAL_Delay(1000);
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