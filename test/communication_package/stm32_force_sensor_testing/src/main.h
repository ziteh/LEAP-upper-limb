/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Send testing force sensor data package.
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "communication.h"
#include "printf.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

/* PA1 = A1 */
#define FORCE_SENSOR_X_ADC_PORT (GPIOA)
#define FORCE_SENSOR_X_ADC_PIN (GPIO1)

/* PA4 = A2 */
#define FORCE_SENSOR_Y_ADC_PORT (GPIOA)
#define FORCE_SENSOR_Y_ADC_PIN (GPIO4)

/* PB0 = A3 */
#define FORCE_SENSOR_Z_ADC_PORT (GPIOB)
#define FORCE_SENSOR_Z_ADC_PIN (GPIO0)

/* PA5 = D13 */
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

#define BUTTON_PORT (GPIOC)
#define BUTTON_PIN (GPIO13)

#define FORCE_SENSOR_X_ADC_CHANNEL (1)
#define FORCE_SENSOR_Y_ADC_CHANNEL (4)
#define FORCE_SENSOR_Z_ADC_CHANNEL (8)

#define USART_BAUDRATE (9600) /* USART baud rate. */

uint16_t get_adc_value(int channel);
void send_force_sensor_value(uint8_t id);

void setup_clock(void);
void setup_usart(void);
void setup_adc(void);
void setup_others_gpio(void);

void delay(unsigned int value);
void setup_exti(void);
void exti15_10_isr(void);

#endif /* MAIN_H_ */