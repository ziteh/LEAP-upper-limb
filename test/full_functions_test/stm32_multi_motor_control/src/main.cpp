/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Multi motor control with communication.
 */

#include "main.h"

int receive_payload_number;
uint8_t receive_header;
uint8_t receive_buffer[BUFFER_LENGTH];

int main(void)
{
  setup_all();

  printf("Ready\r\n");

  set_motor_speed(EFE, 10);
  set_motor_speed(SFE, 90);

  set_motor_state(EFE, Enable);
  set_motor_state(SFE, Enable);
  set_motor_direction(EFE, CCW);
  set_motor_direction(SFE, CCW);

  while (1)
  {
    // auto a1 = get_joint_position(EFE);
    // auto a2 = get_joint_position(SFE);

    // printf("E: %d, S: %d\r\n", a1, a2);

    // delay(100000);
  }

  return 0;
}

void set_motor_state(Joints_t joint, EnableState_t state)
{
  auto port = motor_enable_port[joint];
  auto pin = motor_enable_pin[joint];

  switch (state)
  {
  case Enable:
    gpio_set(port, pin);
    break;

  case ToggleState:
    gpio_toggle(port, pin);
    break;

  default:
    gpio_clear(port, pin);
    break;
  }
}

void set_motor_direction(Joints_t joint, Direction_t dir)
{
  auto port = motor_direction_port[joint];
  auto pin = motor_direction_pin[joint];

  switch (dir)
  {
  case CCW:
    gpio_set(port, pin);
    break;

  case ToggleDirection:
    gpio_toggle(port, pin);
    break;

  default:
    gpio_clear(port, pin);
    break;
  }
}

void set_motor_speed(Joints_t joint, uint8_t speed)
{
  if (speed > 100)
  {
    speed = 100;
  }

  auto tim = motor_speed_pwm_tim[joint];
  auto oc = motor_speed_pwm_oc[joint];
  uint32_t value = PWM_TIMER_PERIOD * (speed / 100.0);

  timer_set_oc_value(tim, oc, value);
}

uint16_t get_joint_position(Joints_t joint)
{
  auto adc = joint_posiion_adc[joint];

  uint8_t channels[16];
  channels[0] = joint_posiion_adc_channel[joint];
  adc_set_regular_sequence(adc, 1, channels);

  adc_start_conversion_direct(adc);

  /* Wait for ADC. */
  while (!adc_get_flag(adc, ADC_SR_EOC))
  {
    __asm__("nop"); // Do nothing.
  }

  return ADC_DR(adc);
}

void clear_communication_variable(void)
{
  receive_header = 0xff;
  receive_payload_number = 0;

  /* Clear buffer. */
  for (int i = 0; i < BUFFER_LENGTH; i++)
  {
    receive_buffer[i] = 0x00;
  }
}

void data_package_parser(uint16_t data)
{
  if ((data & 0x80) == 0x80)
  {
    /* Is header or EOT symbol. */
    receive_header = data;
    switch (data)
    {
    case MOTOR_BASIC_CONTROL_HEADER:
      receive_payload_number = MOTOR_BASIC_CONTROL_PAYLOAD_NUMBER;
      break;

    case MOTOR_POSITION_CONTROL_HEADER:
      receive_payload_number = MOTOR_POSITION_CONTROL_PAYLOAD_NUMBER;
      break;

    case REQUEST_MOTOR_STATE_HEADER:
      receive_payload_number = REQUEST_MOTOR_STATE_PAYLOAD_NUMBER;
      break;

    case REQUEST_FORCE_SENSOR_VALUE_HEADER:
      receive_payload_number = REQUEST_FORCE_SENSOR_VALUE_PAYLOAD_NUMBER;
      break;

    case EOT_SYMBOL:
      clear_communication_variable();
      break;

    default:
      usart_send_blocking(USART2, '?');
      usart_send_blocking(USART2, '\r');
      usart_send_blocking(USART2, '\n');
      break;
    }
  }
  else
  {
    if (receive_payload_number > 0)
    {
      receive_buffer[receive_payload_number - 1] = data;
      receive_payload_number--;
      if (receive_payload_number == 0)
      {
        switch (receive_header)
        {
        case MOTOR_BASIC_CONTROL_HEADER:
        {
          uint8_t id = receive_buffer[3] & 0x1f;
          uint8_t enable = receive_buffer[2] & 0x03;
          uint8_t dircetion = (receive_buffer[2] & 0x0c) >> 2;
          uint16_t speed = (receive_buffer[1] & 0x3f) | ((receive_buffer[0] & 0x3f) << 6);

          set_motor_state((Joints_t)id, (EnableState_t)enable);
          set_motor_direction((Joints_t)id, (Direction_t)dircetion);
          set_motor_speed((Joints_t)id, speed * (100.0 / 4095));

          break;
        }

        case MOTOR_POSITION_CONTROL_HEADER:
        {
          uint8_t id = receive_buffer[2] & 0x1f;
          uint16_t position = (receive_buffer[1] & 0x3f) | ((receive_buffer[0] & 0x3f) << 6);
          // move(position * (100.0 / 4095));
          break;
        }

        case REQUEST_MOTOR_STATE_HEADER:
        {
          uint8_t id = receive_buffer[0] & 0x1f;
          // send_motor_state(id);
          break;
        }

        case REQUEST_FORCE_SENSOR_VALUE_HEADER:
        {
          uint8_t id = receive_buffer[0] & 0x07;
          // send_force_sensor_value(id);
          break;
        }

        default:
          break;
        }
        clear_communication_variable();
      }
    }
  }
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  uint16_t data = usart_recv(USART2);
  data_package_parser(data);

  /* Clear RXNE(Read data register not empty) flag of USART SR(Status register). */
  USART_SR(USART2) &= ~USART_SR_RXNE;
}