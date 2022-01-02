/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Multi motor control with communication.
 */

#include "main.h"

int receive_payload_number;
uint8_t receive_header;
uint8_t receive_buffer[BUFFER_LENGTH];

uint16_t joint_goal_position[2];

int main(void)
{
  setup_all();
  delay(50000);

  joint_goal_position[EFE] = get_joint_position(EFE);
  joint_goal_position[SFE] = get_joint_position(SFE);

  printf("Ready\r\n");

  while (1)
  {
    set_joint_absolute_position(EFE, joint_goal_position[EFE]);
    send_joint_position_state(EFE);
    delay(20);
    set_joint_absolute_position(SFE, joint_goal_position[SFE]);
    send_joint_position_state(SFE);
    delay(20);
    // printf("\r\n");
  }

  return 0;
}

void send_joint_position_state(Joints_t joint)
{
  auto now = convert_adc_value_to_degree(get_joint_position(joint), joint);
  auto goal = convert_adc_value_to_degree(joint_goal_position[joint], joint);
  now = now * 4095.0 / 359.0;
  goal = goal * 4095.0 / 359.0;

  usart_send_blocking(USART2, JOINT_POSITION_STATE_HEADER);
  usart_send_blocking(USART2, (int)joint);
  usart_send_blocking(USART2, now & 0x3f);
  usart_send_blocking(USART2, (now >> 6) & 0x3f);
  usart_send_blocking(USART2, goal & 0x3f);
  usart_send_blocking(USART2, (goal >> 6) & 0x3f);
  usart_send_blocking(USART2, EOT_SYMBOL);
}

int16_t convert_adc_value_to_degree(uint16_t adc_value, Joints_t joint)
{
  auto flexed_degree = joint_flexed_degree[joint];
  auto extension_degree = joint_extension_degree[joint];
  auto flexed_adc_value = joint_flexed_adc_value[joint];
  auto extension_adc_value = joint_extension_adc_value[joint];

  float m = (float)(flexed_degree - extension_degree) / (float)(flexed_adc_value - extension_adc_value);
  float degree = ((adc_value - extension_adc_value) * m) + extension_degree;

  /* 點斜式. */
  if (flexed_degree > extension_degree)
  {
    if (degree > flexed_degree)
    {
      degree = flexed_degree;
    }
    else if (degree < extension_degree)
    {
      degree = extension_degree;
    }
  }
  else
  {
    if (degree < flexed_degree)
    {
      degree = flexed_degree;
    }
    else if (degree > extension_degree)
    {
      degree = extension_degree;
    }
  }

  return (int16_t)degree;
}

uint16_t convert_degree_to_adc_value(float degree, Joints_t joint)
{
  auto flexed_degree = joint_flexed_degree[joint];
  auto extension_degree = joint_extension_degree[joint];
  auto flexed_adc_value = joint_flexed_adc_value[joint];
  auto extension_adc_value = joint_extension_adc_value[joint];

  /* 點斜式. */
  float m = (float)(flexed_degree - extension_degree) / (float)(flexed_adc_value - extension_adc_value);
  float adc_value = ((degree - extension_degree) / m) + extension_adc_value;

  if (flexed_adc_value > extension_adc_value)
  {
    if (adc_value > flexed_adc_value)
    {
      adc_value = flexed_adc_value;
    }
    else if (adc_value < extension_adc_value)
    {
      adc_value = extension_adc_value;
    }
  }
  else
  {
    if (adc_value < flexed_adc_value)
    {
      adc_value = flexed_adc_value;
    }
    else if (adc_value > extension_adc_value)
    {
      adc_value = extension_adc_value;
    }
  }

  return (uint16_t)adc_value;
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

void set_joint_absolute_position(Joints_t joint, uint16_t goal_adc_value)
{
  auto flexed_adc_value = joint_flexed_adc_value[joint];
  auto extension_adc_value = joint_extension_adc_value[joint];
  auto allowable_error = joint_allowable_position_error[joint];
  auto flexed_direction = joint_flexed_direction[joint];
  auto now_adc_value = get_joint_position(joint);
  uint16_t max_adc_value;
  uint16_t min_adc_value;
  Direction_t increase_adc_value_direction;

  if (flexed_adc_value > extension_adc_value)
  {
    max_adc_value = flexed_adc_value;
    min_adc_value = extension_adc_value;
    increase_adc_value_direction = flexed_direction;
  }
  else
  {
    max_adc_value = extension_adc_value;
    min_adc_value = flexed_adc_value;
    increase_adc_value_direction = (Direction_t)(!(bool)flexed_direction);
  }

  if ((now_adc_value - goal_adc_value) > allowable_error && now_adc_value >= min_adc_value)
  {
    auto dir = (Direction_t)(!(bool)increase_adc_value_direction);
    set_motor_direction(joint, dir);
    set_motor_speed(joint, 15);
    set_motor_state(joint, Enable);

    // printf("J%d: G: %4d, N: %4d (%d)", joint, goal_adc_value, now_adc_value, dir);
  }
  else if ((goal_adc_value - now_adc_value) > allowable_error && now_adc_value <= max_adc_value)
  {
    auto dir = increase_adc_value_direction;
    set_motor_direction(joint, dir);
    set_motor_speed(joint, 15);
    set_motor_state(joint, Enable);

    // printf("J%d: G: %4d, N: %4d (%d)", joint, goal_adc_value, now_adc_value, dir);
  }
  else
  {
    set_motor_state(joint, Disable);
    set_motor_speed(joint, 0);

    // printf("J%d: N: %4d (Done)", joint, now_adc_value);
  }
}

void set_joint_absolute_position_percentage(Joints_t joint, uint16_t position_percentage)
{
  auto max_position = joint_flexed_adc_value[joint];
  auto min_position = joint_extension_adc_value[joint];
  uint16_t goal_position = ((max_position - min_position) * position_percentage * 0.01) + min_position;

  set_joint_absolute_position(joint, goal_position);
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
          uint16_t angle_value = (receive_buffer[1] & 0x3f) | ((receive_buffer[0] & 0x3f) << 6);

          float degree = (angle_value * 359.0 / 4095.0);
          degree = degree > 180 ? -(360 - degree) : degree;

          auto adc_value = convert_degree_to_adc_value(degree, (Joints_t)id);
          joint_goal_position[(Joints_t)id] = adc_value;

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