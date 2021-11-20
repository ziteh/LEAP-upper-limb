/**
 * @file   main.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  maxon mmc ESCON 70/10 motor controller test.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

void setup_clock(void);
void setup_usart(void);
void setup_pwm(void);
void set_dutycycle(float value);
void setup_led(void);
void setup_control_pin(void);
void delay(uint32_t value);
void usart2_isr(void);
