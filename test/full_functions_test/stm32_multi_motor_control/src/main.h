/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Multi motor control with communication.
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "setup.h"
#include "delay.h"
#include "communication.h"
#include "printf.h"
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

void clear_communication_variable(void);
uint16_t get_adc_value(int channel);
void set_dutycycle(float value);
void move(uint16_t position);

void send_motor_state(uint8_t motor_id);
void send_force_sensor_value(uint8_t id);

void usart2_isr(void);

#endif /* MAIN_H_ */