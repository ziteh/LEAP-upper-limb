/**
 * @file   dynamixel_2_0.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Robotis DYNAMIXEL Protocol 2.0 header file.
 * @remark Ref: https://emanual.robotis.com/docs/en/dxl/protocol2/
 */

#ifndef __DYNAMIXEL_2_0_H
#define __DYNAMIXEL_2_0_H

/* Select a library. */
// #define USE_LIBOPENCM3
#define USE_STM32HAL

#if defined(USE_LIBOPENCM3)
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>

#define MAX485_DE_RE_GPIO_PORT (GPIOB)
#define MAX485_DE_RE_GPIO_PIN (GPIO3)
#define MAX485_UART (USART1)
#elif defined(USE_STM32HAL)
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define MAX485_DE_RE_GPIO_PORT (GPIOB)
#define MAX485_DE_RE_GPIO_PIN (GPIO_PIN_3)
#define MAX485_UART (huart1)
#else
#error
#endif

#define DYNAMIXEL_BUFFER (buffer)
#define DYNAMIXEL_BUFFER_INDEX (buffer_index)
#define DYNAMIXEL_BUFFER_LENGTH (128)

#define DYNAMIXEL2_BROADCAST_ID ((uint8_t)0xFE)

#define H54_DEGREE_TO_POSITION(deg) ((double)deg * (250961.5 / 180.0))
#define H54_POSITION_TO_DEGREE(pos) ((int32_t)pos * (180.0 / 250961.5))

#define H42_DEGREE_TO_POSITION(deg) ((double)deg * (151875 / 180.0))
#define H42_POSITION_TO_DEGREE(pos) ((int32_t)pos * (180.0 / 151875))

typedef enum
{
  ping = 0x01,
  read = 0x02,
  write = 0x03,

  reg_write = 0x04,
  action = 0x05,

  factory_reset = 0x06,
  reboot = 0x08,
  clear = 0x10,
  control_table_backup = 0x20,

  sync_read = 0x82,
  sync_write = 0x83,
  fast_sync_read = 0x8A,

  bulk_read = 0x92,
  bulk_write = 0x93,
  fast_bulk_read = 0x9A
} dynamixel2_instruction_t;

void max485_send(uint8_t *data, uint32_t length);

void dynamixel2_write(uint8_t id, uint16_t address, uint8_t *data, uint16_t data_length);
bool dynamixel2_read(uint8_t id, uint16_t address, uint16_t data_length, uint8_t *return_data, uint16_t *return_data_length);
void dynamixel2_reset(uint8_t id);

int32_t dynamixel2_read_present_position(uint8_t id);
void dynamixel2_set_goal_position(uint8_t id, int32_t position);
void dynamixel2_set_torque_enable(uint8_t id, bool enable);

void dynamixel2_receive_callback(uint8_t received_data);
void dynamixel2_clear_receive_buffer(void);

#endif /* __DYNAMIXEL_2_0_H */