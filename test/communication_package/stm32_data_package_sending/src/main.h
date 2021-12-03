/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Sending data packge test.
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "communication.h"
#include "printf.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

/* PA5 = D13 */
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

#define BUTTON_PORT (GPIOC)
#define BUTTON_PIN (GPIO13)

#define USART_BAUDRATE (9600) /* USART baud rate. */

void send_fake_data(void);

void setup_clock(void);
void setup_usart(void);
void setup_exti(void);
void setup_others_gpio(void);

void delay(unsigned int value);
void exti15_10_isr(void);

#endif /* MAIN_H_ */