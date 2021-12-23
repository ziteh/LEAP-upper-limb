/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Multi motor control with communication.
 */

#include "main.h"

int main(void)
{
  setup_all();

  printf("Ready\r\n");

  while (1)
  {
    set_motor_state(EFE, ToggleState);
    set_motor_state(SFE, ToggleState);
    set_motor_direction(EFE, ToggleDirection);
    set_motor_direction(SFE, ToggleDirection);

    delay(5000000);
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
