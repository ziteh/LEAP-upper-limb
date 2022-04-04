/**
 * @file   main.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  3-Axis force sensor test with CAN bus.
 */

#include "main.h"

SPI_HandleTypeDef mcp2515_spi;
UART_HandleTypeDef pc_usart;
ADC_HandleTypeDef force_sensor_adc;

int main(void)
{
  HAL_Init();

  RCC_Init();
  USART_Init();
  SPI_Init();
  ADC_Init();

  printf("Ready\r\n");

  /* Halt. */
  while (1)
  {
    uint16_t value = Get_ADC_Value();
    printf("%4d\r\n", value);
    HAL_Delay(500);
  }

  return 0;
}

/* Setup. */
void RCC_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();
}

void USART_Init(void)
{
  GPIO_InitTypeDef GPIO_Tx;
  GPIO_Tx.Pin = GPIO_USART_TX_PIN;
  GPIO_Tx.Mode = GPIO_MODE_AF_PP;
  GPIO_Tx.Speed = GPIO_SPEED_HIGH;
  GPIO_Tx.Pull = GPIO_NOPULL;
  GPIO_Tx.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIO_USART_TX_PORT, &GPIO_Tx);

  GPIO_InitTypeDef GPIO_Rx;
  GPIO_Rx.Pin = GPIO_USART_RX_PIN;
  GPIO_Rx.Mode = GPIO_MODE_AF_PP;
  GPIO_Rx.Speed = GPIO_SPEED_HIGH;
  GPIO_Rx.Pull = GPIO_NOPULL;
  GPIO_Rx.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIO_USART_RX_PORT, &GPIO_Rx);

  pc_usart.Instance = USART2;
  pc_usart.Init.BaudRate = USART_BAUDRATE;
  pc_usart.Init.WordLength = UART_WORDLENGTH_8B;
  pc_usart.Init.StopBits = UART_STOPBITS_1;
  pc_usart.Init.Parity = UART_PARITY_NONE;
  pc_usart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  pc_usart.Init.Mode = USART_MODE_TX_RX;
  HAL_UART_Init(&pc_usart);
}

void SPI_Init(void)
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

  mcp2515_spi.Instance = SPI2;
  mcp2515_spi.Init.Mode = SPI_MODE_MASTER;
  mcp2515_spi.Init.Direction = SPI_DIRECTION_2LINES;
  mcp2515_spi.Init.DataSize = SPI_DATASIZE_8BIT;
  mcp2515_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
  mcp2515_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
  mcp2515_spi.Init.NSS = SPI_NSS_SOFT;
  mcp2515_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  mcp2515_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  mcp2515_spi.Init.TIMode = SPI_TIMODE_DISABLE;
  mcp2515_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  mcp2515_spi.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&mcp2515_spi);

  SPI_End();
}

void ADC_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_ADC_A0_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIO_ADC_A0_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_ADC_A1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIO_ADC_A1_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_ADC_A2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIO_ADC_A2_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_ADC_A3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIO_ADC_A3_PORT, &GPIO_InitStruct);

  force_sensor_adc.Instance = ADC1;
  force_sensor_adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  force_sensor_adc.Init.ContinuousConvMode = ENABLE;
  force_sensor_adc.Init.DiscontinuousConvMode = DISABLE;
  force_sensor_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  force_sensor_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  force_sensor_adc.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&force_sensor_adc);

  ADC_ChannelConfTypeDef ADC_ChConfig;
  // ADC_ChConfig.Channel = ADC_A0_CHANNEL | ADC_A1_CHANNEL | ADC_A2_CHANNEL | ADC_A3_CHANNEL;
  ADC_ChConfig.Channel = ADC_A0_CHANNEL;
  ADC_ChConfig.Rank = ADC_REGULAR_RANK_1;
  ADC_ChConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  HAL_ADC_ConfigChannel(&force_sensor_adc, &ADC_ChConfig);
}

uint16_t Get_ADC_Value(void)
{
  HAL_ADC_Start(&force_sensor_adc);

  while (HAL_ADC_PollForConversion(&force_sensor_adc, 5) != HAL_OK)
  {
  }

  uint16_t value = HAL_ADC_GetValue(&force_sensor_adc);
  HAL_ADC_Stop(&force_sensor_adc);
  return value;
}

void SPI_End(void)
{
  HAL_GPIO_WritePin(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN, GPIO_PIN_SET);
}

void SPI_Start(void)
{
  HAL_GPIO_WritePin(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN, GPIO_PIN_RESET);
}

/* IT Handlers. */
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