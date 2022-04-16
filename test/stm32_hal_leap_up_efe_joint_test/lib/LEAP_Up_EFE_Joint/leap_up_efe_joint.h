/**
 * @file leap_up_efe_joint.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  A library for LEAP-Up EFE-joint control.
 *
 */

#ifndef __LEAP_UP_EFE_JOINT_H
#define __LEAP_UP_EFE_JOINT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include <as5047p.h>
#include <maxon_escon.h>

#define EFE_JOINT_RIGHT
  // #define EFE_JOINT_LEFT

#define EFE_JOINT_ALLOWABLE_ANGLE_ERROR (1.0)
#define EFE_JOINT_ANGLE_MAX (70)
#define EFE_JOINT_ANGLE_MIN (5)
#define EFE_JOINT_PWM_DC_MAX (30)
#define EFE_JOINT_PWM_DC_MIN (1)
#define EFE_JOINT_PWM_DC_OFFSET (10.1)

  /**
   * @brief
   *
   * @return 0 for OK, 1 for Error.
   */
  int EFE_Joint_Init(void);
  void EFE_Joint_SetEnable(void);
  void EFE_Joint_SetDisable(void);

  void EFE_Joint_SetAngle(double goal_angle);
  double EFE_Joint_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* __LEAP_UP_EFE_JOINT_H */