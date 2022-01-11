/**
 * @file   define.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Define all the value.
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

/* PID. */
#define PID_TIMER (TIM2)
#define PID_TIMER_PRESCALER (rcc_apb1_frequency / 1000.0)
#define PID_TIMER_PERIOD (500)
#define PID_KP (1)
#define PID_KI (1)
#define PID_KD (1)

/* EFE Joint. */

/* PA1 = A1, ADC1-Ch1 */
#define EFE_JOINT_POSITION_ADC_PORT (GPIOA)
#define EFE_JOINT_POSITION_ADC_PIN (GPIO1)
#define EFE_JOINT_POSITION_ADC (ADC1)
#define EFE_JOINT_POSITION_ADC_CHANNEL (ADC_CHANNEL1)

/* PA6 = D12, PWM3-Ch1 */
#define EFE_MOTOR_SPEED_PWM_PORT (GPIOA)
#define EFE_MOTOR_SPEED_PWM_PIN (GPIO6)
#define EFE_MOTOR_SPEED_PWM_TIM (TIM3)
#define EFE_MOTOR_SPEED_PWM_OC (TIM_OC1)

/* PB10 = D6 */
#define EFE_MOTOR_ENABLE_PORT (GPIOB)
#define EFE_MOTOR_ENABLE_PIN (GPIO10)

/* PB5 = D4 */
#define EFE_MOTOR_DIRECTION_PORT (GPIOB)
#define EFE_MOTOR_DIRECTION_PIN (GPIO5)

/* PB3 = D3 */
#define EFE_MOTOR_READY_PORT (GPIOB)
#define EFE_MOTOR_READY_PIN (GPIO3)

#define EFE_JOINT_FLEXED_DEGREE (89)
#define EFE_JOINT_EXTENSION_DEGREE (3)
#define EFE_JOINT_FLEXED_ADC_VALUE (3517)    // 89 deg
#define EFE_JOINT_EXTENSION_ADC_VALUE (2620) //  3 deg
#define EFE_JOINT_ALLOWABLE_POSITION_ERROR (20)

#define EFE_FLEXED_DIRCETION (CW)

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

/* PC7 = D9 */
#define SFE_MOTOR_ENABLE_PORT (GPIOC)
#define SFE_MOTOR_ENABLE_PIN (GPIO7)

/* PB6 = D10 */
#define SFE_MOTOR_DIRECTION_PORT (GPIOB)
#define SFE_MOTOR_DIRECTION_PIN (GPIO6)

/* PA8 = D7 */
#define SFE_MOTOR_READY_PORT (GPIOA)
#define SFE_MOTOR_READY_PIN (GPIO8)

#define SFE_JOINT_FLEXED_DEGREE (50)
#define SFE_JOINT_EXTENSION_DEGREE (-70)
#define SFE_JOINT_FLEXED_ADC_VALUE (608)     //  50 deg
#define SFE_JOINT_EXTENSION_ADC_VALUE (3580) // -70 deg
#define SFE_JOINT_ALLOWABLE_POSITION_ERROR (20)

#define SFE_FLEXED_DIRCETION (CCW)

/* Force sensors. */

/* PA4 = A2, ADC1-Ch4 */
#define FORCE_SENSOR_A1_ADC_PORT (GPIOA)
#define FORCE_SENSOR_A1_ADC_PIN (GPIO4)
#define FORCE_SENSOR_A1_ADC (ADC1)
#define FORCE_SENSOR_A1_ADC_CHANNEL (ADC_CHANNEL4)

/* PB0 = A3, ADC1-Ch8 */
#define FORCE_SENSOR_A2_ADC_PORT (GPIOB)
#define FORCE_SENSOR_A2_ADC_PIN (GPIO0)
#define FORCE_SENSOR_A2_ADC (ADC1)
#define FORCE_SENSOR_A2_ADC_CHANNEL (ADC_CHANNEL8)

/* PC1 = A4, ADC1-Ch11 */
#define FORCE_SENSOR_B1_ADC_PORT (GPIOC)
#define FORCE_SENSOR_B1_ADC_PIN (GPIO1)
#define FORCE_SENSOR_B1_ADC (ADC1)
#define FORCE_SENSOR_B1_ADC_CHANNEL (ADC_CHANNEL11)

/* PC0 = A5, ADC1-Ch10 */
#define FORCE_SENSOR_B2_ADC_PORT (GPIOC)
#define FORCE_SENSOR_B2_ADC_PIN (GPIO0)
#define FORCE_SENSOR_B2_ADC (ADC1)
#define FORCE_SENSOR_B2_ADC_CHANNEL (ADC_CHANNEL10)

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

#endif /* DEFINE_H_ */