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

typedef enum
{
  EFE,
  SFE
} Joints_t;

typedef enum
{
  Disable = 0,
  Enable = !Disable,
  ToggleState
} EnableState_t;

typedef enum
{
  CW = 0,
  CCW = !CW,
  ToggleDirection
} Direction_t;

uint32_t motor_enable_port[2] = {EFE_MOTOR_ENABLE_PORT,
                                 SFE_MOTOR_ENABLE_PORT};

uint16_t motor_enable_pin[2] = {EFE_MOTOR_ENABLE_PIN,
                                SFE_MOTOR_ENABLE_PIN};

uint32_t motor_direction_port[2] = {EFE_MOTOR_DIRECTION_PORT,
                                    SFE_MOTOR_DIRECTION_PORT};

uint16_t motor_direction_pin[2] = {EFE_MOTOR_DIRECTION_PIN,
                                   SFE_MOTOR_DIRECTION_PIN};

uint32_t motor_speed_pwm_tim[2] = {EFE_MOTOR_SPEED_PWM_TIM,
                                   SFE_MOTOR_SPEED_PWM_TIM};

tim_oc_id motor_speed_pwm_oc[2] = {EFE_MOTOR_SPEED_PWM_OC,
                                   SFE_MOTOR_SPEED_PWM_OC};

uint32_t joint_posiion_adc[2] = {EFE_JOINT_POSITION_ADC,
                                 SFE_JOINT_POSITION_ADC};

uint32_t joint_posiion_adc_channel[2] = {EFE_JOINT_POSITION_ADC_CHANNEL,
                                         SFE_JOINT_POSITION_ADC_CHANNEL};

void clear_communication_variable(void);
uint16_t get_adc_value(int channel);
void set_dutycycle(float value);
void move(uint16_t position);

void send_motor_state(uint8_t motor_id);
void send_force_sensor_value(uint8_t id);

void usart2_isr(void);

void set_motor_state(Joints_t joint, EnableState_t state);
void set_motor_direction(Joints_t joint, Direction_t dir);
void set_motor_speed(Joints_t joint, uint8_t speed);
uint16_t get_joint_position(Joints_t joint);

#endif /* MAIN_H_ */