/**
 * @file leap_up_efe_joint.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  A library for LEAP-Up EFE-joint control.
 *
 */

#include "leap_up_efe_joint.h"

#if defined(EFE_JOINT_RIGHT)
#define EFE_JOINT_INCREASE_ANGLE_DIR (CW)
#define EFE_JOINT_DECREASE_ANGLE_DIR (CCW)
#elif defined(EFE_JOINT_LEFT)
#define EFE_JOINT_INCREASE_ANGLE_DIR (CCW)
#define EFE_JOINT_DECREASE_ANGLE_DIR (CW)
#else
#error Select right or left in .h.
#endif

int EFE_Joint_Init(void)
{
  int as5047_state = as5047p_init(0b00100101, 0b00000000);
  as5047p_set_zero(AS5047P_ZERO_POSITION_VALUE);

  int escon_state = ESCON_Init();

  if (as5047_state != 0 || escon_state != 0)
  {
    return 1; /* Error. */
  }
  else
  {
    return 0; /* OK. */
  }
}

void EFE_Joint_SetEnable(void)
{
  ESCON_SetFunctionState(Enable);
}

void EFE_Joint_SetDisable(void)
{
  ESCON_SetFunctionState(Disable);
}

void EFE_Joint_SetAngle(int32_t goal, int32_t present)
{
  /* Limit. */
  if (goal > EFE_JOINT_MAX)
  {
    goal = EFE_JOINT_MAX;
  }
  else if (goal < EFE_JOINT_MIN)
  {
    goal = EFE_JOINT_MIN;
  }

  int32_t error = goal - present;
  if (error < 0)
  {
    error = -error;
    ESCON_SetDirection(EFE_JOINT_DECREASE_ANGLE_DIR);
  }
  else
  {
    ESCON_SetDirection(EFE_JOINT_INCREASE_ANGLE_DIR);
  }

  float speed = 0;
  if (error > EFE_JOINT_ALLOWABLE_ERROR)
  {
    speed = error * (10 / 100.0);
  }
  else
  {
    ESCON_SetFunctionState(Disable);
    return;
  }

  /* Limit. */
  if (speed > EFE_JOINT_PWM_DC_MAX)
  {
    speed = EFE_JOINT_PWM_DC_MAX;
  }
  else if (speed < EFE_JOINT_PWM_DC_MIN)
  {
    speed = EFE_JOINT_PWM_DC_MIN;
  }

  ESCON_SetPwmDutyCycle(speed + EFE_JOINT_PWM_DC_OFFSET);
  ESCON_SetFunctionState(Enable);
}

double EFE_Joint_GetAngle(void)
{
  float angle;
  while (as5047p_get_angle(false, &angle) != 0)
  {
    __asm__("nop"); /* Do nothing. */
  }
  return (double)angle;
}
