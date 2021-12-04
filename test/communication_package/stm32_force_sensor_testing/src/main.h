/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Communication packge test.
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "communication.h"
#include "printf.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

#define MAX_POSITION (4095 - 1000)
#define MIN_POSITION (0 + 1000)
#define ALLOWABLE_POSITION_ERROR (35)

#define PWM_FREQUENCY (1000) /* PWM frequency in Hz. */
#define PWM_DUTYCYCLE (20)   /* PWM duty cycle in %. */

/* PA1 = A1 */
#define FORCE_SENSOR_X_ADC_PORT (GPIOA)
#define FORCE_SENSOR_X_ADC_PIN (GPIO1)

/* PA4 = A2 */
#define FORCE_SENSOR_Y_ADC_PORT (GPIOA)
#define FORCE_SENSOR_Y_ADC_PIN (GPIO4)

/* PB0 = A3 */
#define FORCE_SENSOR_Z_ADC_PORT (GPIOB)
#define FORCE_SENSOR_Z_ADC_PIN (GPIO0)

/* PA0 = A0 */
#define MOTOR_POSITION_ADC_PORT (GPIOA)
#define MOTOR_POSITION_ADC_PIN (GPIO0)

/* PA6 = D12 */
#define MOTOR_ENABLE_PORT (GPIOA)
#define MOTOR_ENABLE_PIN (GPIO6)

/* PB6 = D10 */
#define MOTOR_DIRECTION_PORT (GPIOB)
#define MOTOR_DIRECTION_PIN (GPIO6)

/* PA8 = D7 */
#define MOTOR_READY_PORT (GPIOA)
#define MOTOR_READY_PIN (GPIO8)

/* PA7 = D11 */
#define PWM_PORT (GPIOA)
#define PWM_PIN (GPIO7)

/* PA5 = D13 */
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

#define MOTOR_POSITION_ADC_CHANNEL (0)

#define FORCE_SENSOR_X_ADC_CHANNEL (1)
#define FORCE_SENSOR_Y_ADC_CHANNEL (4)
#define FORCE_SENSOR_Z_ADC_CHANNEL (8)

#define USART_BAUDRATE (9600) /* USART baud rate. */

/*
 * f_pwm = f_tim / [(PRS + 1) * (PER + 1)]
 * So,
 * PER = {f_tim / [(PRS + 1) * f_pwm]} - 1
 * 
 * f_pwm: PWM frequency.
 * f_tim: Timer frequency. The value is 'rcc_apb1_frequency * 2' equal 48MHz in this case.
 * PRS:   PWM timer prescaler.
 * PER:   PWM timer period.
 */
#define PWM_TIMER_PRESCALER (48 - 1)
#define PWM_TIMER_PERIOD (((rcc_apb1_frequency * 2) / ((PWM_TIMER_PRESCALER + 1) * PWM_FREQUENCY)) - 1)

void clear_communication_variable(void);
uint16_t get_adc_value(int channel);
void set_dutycycle(float value);
void move(uint16_t position);

void send_motor_state(uint8_t motor_id);
void send_force_sensor_value(uint8_t id);

void setup_clock(void);
void setup_usart(void);
void setup_pwm(void);
void setup_adc(void);
void setup_others_gpio(void);

void delay(unsigned int value);
void usart2_isr(void);

#endif /* MAIN_H_ */