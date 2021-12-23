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

  set_motor_speed(EFE, 10);
  set_motor_speed(SFE, 90);

  set_motor_state(EFE, Enable);
  set_motor_state(SFE, Enable);
  set_motor_direction(EFE, CCW);
  set_motor_direction(SFE, CCW);

  int i = 0;
  while (1)
  {
    auto a1 = get_joint_position(EFE);
    auto a2 = get_joint_position(SFE);

    printf("E: %d, S: %d\r\n", a1, a2);

    delay(100000);
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