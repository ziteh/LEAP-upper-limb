/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  3-Axis force sensor test with CAN bus.
 */

#pragma one

#include "printf.h"
#include <stm32f3xx_hal.h>

#define USART_BAUDRATE (115200)

/* Pin. */
// #define GPIO_LED_PORT (GPIOB)
// #define GPIO_LED_PIN (GPIO_PIN_13)

#define GPIO_USART_TX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO_PIN_2)

#define GPIO_USART_RX_PORT (GPIOA)
#define GPIO_USART_RX_PIN (GPIO_PIN_3)

#define GPIO_SPI_SCK_PORT (GPIOB)
#define GPIO_SPI_SCK_PIN (GPIO_PIN_13)

#define GPIO_SPI_MISO_PORT (GPIOB)
#define GPIO_SPI_MISO_PIN (GPIO_PIN_14)

#define GPIO_SPI_MOSI_PORT (GPIOB)
#define GPIO_SPI_MOSI_PIN (GPIO_PIN_15)

#define GPIO_SPI_CS_PORT (GPIOB)
#define GPIO_SPI_CS_PIN (GPIO_PIN_6)

#define GPIO_ADC_A0_PORT (GPIOA)
#define GPIO_ADC_A0_PIN (GPIO_PIN_0)
#define ADC_A0_CHANNEL (ADC_CHANNEL_1)

#define GPIO_ADC_A1_PORT (GPIOA)
#define GPIO_ADC_A1_PIN (GPIO_PIN_1)
#define ADC_A1_CHANNEL (ADC_CHANNEL_2)

#define GPIO_ADC_A2_PORT (GPIOA)
#define GPIO_ADC_A2_PIN (GPIO_PIN_4)
#define ADC_A2_CHANNEL (ADC_CHANNEL_5)

#define GPIO_ADC_A3_PORT (GPIOB)
#define GPIO_ADC_A3_PIN (GPIO_PIN_0)
#define ADC_A3_CHANNEL (ADC_CHANNEL_9)

void RCC_Init(void);
void USART_Init(void);
void SPI_Init(void);
void ADC_Init(void);

void SPI_End(void);
void SPI_Start(void);

uint16_t Get_ADC_Value(void);