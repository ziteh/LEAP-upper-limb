
#include <stm32f3xx_hal.h>
#include "mcp2515.h"

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

void SPI_SELECT();
void SPI_DESELECT();
void spi_start(void);
void spi_end(void);
