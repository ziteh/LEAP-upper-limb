/**
 * @file	  usart.cpp
 * @author	ZiTe (honmonoh@gmail.com)
 * @date    Sep 30, 2021
 */

#include "usart.hpp"

USART::USART()
{
  GPIO tx(PA2, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
  GPIO rx(PA3, GPIO_Mode_IN_FLOATING);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_InitTypeDef USART_InitStructure;
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  /* Enable "Receive data register not empty" interrupt */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  /* Enable USART */
  USART_Cmd(USART2, ENABLE);

  /* Clear "Transmission Complete" flag, else the first bit of data will lose. */
  USART_ClearFlag(USART2, USART_FLAG_TC);
}

void USART::send(uint8_t *data)
{
  for (int i = 0; data[i] != '\0'; i++)
  {
    /* Transmits single data through the USARTx peripheral */
    USART_SendData(USART2, (uint16_t)data[i]);

    /* Wait until transmission complete, use TC or TXE flag */
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {
      /* Null */
    }
  }
}

void USART::send(std::string data)
{
  for (int i = 0; data[i] != '\0'; i++)
  {
    /* Transmits single data through the USARTx peripheral */
    USART_SendData(USART2, (uint16_t)data[i]);

    /* Wait until transmission complete, use TC or TXE flag */
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {
      /* Null */
    }
  }
}
