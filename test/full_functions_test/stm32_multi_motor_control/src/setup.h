/**
 * @file   setup.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Initialize each function.
 */

#ifndef SETUP_H_
#define SETUP_H_

#include "define.h"
#include "delay.h"

void setup_all(void);

void setup_clock(void);
void setup_usart(void);
void setup_pwm(void);
void setup_adc(void);
void setup_timer(void);
void setup_others_gpio(void);

#endif /* SETUP_H_ */