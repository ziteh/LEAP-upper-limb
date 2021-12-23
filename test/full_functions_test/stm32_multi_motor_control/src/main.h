/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Multi motor control with communication.
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "communication.h"
#include "printf.h"
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

/* EFE Joint. */

/* PA1 = A1, ADC1-Ch1 */
#define EFE_JOINT_POSITION_ADC_PORT (GPIOA)
#define EFE_JOINT_POSITION_ADC_PIN (GPIO1)
#define EFE_JOINT_POSITION_ADC (ADC1)
#define EFE_JOINT_POSITION_ADC_CHANNEL (ADC_CHANNEL1)

/* PB10 = D6, PWM2-Ch3(Remap) */
#define EFE_MOTOR_SPEED_PWM_PORT (GPIOB)
#define EFE_MOTOR_SPEED_PWM_PIN (GPIO10)
#define EFE_MOTOR_SPEED_PWM_TIM (TIM2)
#define EFE_MOTOR_SPEED_PWM_OC (TIM_OC3)

/* PB4 = D5 */
#define EFE_MOTOR_ENABLE_PORT (GPIOB)
#define EFE_MOTOR_ENABLE_PIN (GPIO4)

/* PB5 = D4 */
#define EFE_MOTOR_DIRECTION_PORT (GPIOB)
#define EFE_MOTOR_DIRECTION_PIN (GPIO5)

/* PB3 = D3 */
#define EFE_MOTOR_READY_PORT (GPIOB)
#define EFE_MOTOR_READY_PIN (GPIO3)

#define EFE_JOINT_MAX_POSITION (4095 - 1000)
#define EFE_JOINT_MIN_POSITION (0 + 1000)
#define EFE_JOINT_ALLOWABLE_POSITION_ERROR (35)

/* SFE Joint. */

/* PA0 = A0, ADC1-Ch0 */
#define SFE_JOINT_POSITION_ADC_PORT (GPIOA)
#define SFE_JOINT_POSITION_ADC_PIN (GPIO0)
#define SFE_JOINT_POSITION_ADC (ADC1)
#define SFE_JOINT_POSITION_ADC_CHANNEL (ADC_CHANNEL0)

/* PA7 = D11, PWM3-Ch2 */
#define SFE_MOTOR_SPEED_PWM_PORT (GPIOA)
#define SFE_MOTOR_SPEED_PWM_PIN (GPIO7)
#define SFE_MOTOR_SPEED_PWM_TIM (TIM3)
#define SFE_MOTOR_SPEED_PWM_OC (TIM_OC2)

/* PA6 = D12 */
#define SFE_MOTOR_ENABLE_PORT (GPIOA)
#define SFE_MOTOR_ENABLE_PIN (GPIO6)

/* PB6 = D10 */
#define SFE_MOTOR_DIRECTION_PORT (GPIOB)
#define SFE_MOTOR_DIRECTION_PIN (GPIO6)

/* PA8 = D7 */
#define SFE_MOTOR_READY_PORT (GPIOA)
#define SFE_MOTOR_READY_PIN (GPIO8)

#define SFE_JOINT_MAX_POSITION (4095 - 1000)
#define SFE_JOINT_MIN_POSITION (0 + 1000)
#define SFE_JOINT_ALLOWABLE_POSITION_ERROR (35)

/* Others. */

/* PA5 = D13 */
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

#define USART_BAUDRATE (9600) /* USART baud rate. */

#define PWM_FREQUENCY (1000) /* PWM frequency in Hz. */
#define PWM_DUTYCYCLE (20)   /* PWM duty cycle in %. */
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

void inline delay(volatile unsigned int value);
void usart2_isr(void);

#endif /* MAIN_H_ */