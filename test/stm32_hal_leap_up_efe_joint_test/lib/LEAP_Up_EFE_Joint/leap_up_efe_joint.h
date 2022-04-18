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

#define AS5047P_ZERO_POSITION_VALUE ((uint16_t)9375)
#define AS5047P_ENCODER_CPR (4096)
#define AS5047P_COUNT_TO_ANGLE(c) (c * (360.0 / AS5047P_ENCODER_CPR))
#define AS5047P_ANGLE_TO_COUNT(a) (a * (AS5047P_ENCODER_CPR / 360.0))

#define EFE_JOINT_ALLOWABLE_ERROR (5U)
#define EFE_JOINT_MAX (AS5047P_ANGLE_TO_COUNT(70))
#define EFE_JOINT_MIN (AS5047P_ANGLE_TO_COUNT(20))
#define EFE_JOINT_PWM_DC_MAX (20)
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

  void EFE_Joint_SetAngle(int32_t goal, int32_t present);
  double EFE_Joint_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* __LEAP_UP_EFE_JOINT_H */