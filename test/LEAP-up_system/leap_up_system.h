/**
 * @file leap_up_system.h
 * @brief 'LEAP-Up' upper limb powered exoskeleton control system header file.
 * @author ZiTe (honmonoh@gmail.com)
 */

#ifndef LEAP_UP_SYSTEM_H_
#define LEAP_UP_SYSTEM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

  /*
   * Joint.
   */
  typedef int8_t (*JointSetAngle_t)(float rad);
  typedef int8_t (*JointGetAngle_t)(float *rad);
  typedef int8_t (*JointGetState_t)(uint16_t *state);
  typedef int8_t (*JointEnable_t)(bool enable);

  typedef struct
  {
    JointSetAngle_t SetAngle;
    JointGetAngle_t GetAngle;
    JointGetState_t GetState;
    JointEnable_t Enable;
  } JointHandle_t;

  JointHandle_t *MakeJointHandle(JointSetAngle_t setAngleFunc,
                                 JointGetAngle_t getAngleFunc,
                                 JointGetState_t getStateFunc,
                                 JointEnable_t enableFunc);

  /*
   * FTSensor.
   */
  typedef int8_t (*FTSensorGetData_t)(int32_t *fx,
                                      int32_t *fy,
                                      int32_t *fz,
                                      int32_t *tx,
                                      int32_t *ty,
                                      int32_t *tz);
  typedef uint8_t

      /*
       * Exoskeleton.
       */
      typedef int8_t (*ExoskeletonEnable_t)(bool enable);
  typedef int8_t (*ExoskeletonGetState_t)(uint16_t *state);

  typedef struct
  {
    JointHandle_t SIE;
    JointHandle_t SFE;
    JointHandle_t EFE;
  } ExoskeletonHandle_t;

#ifdef __cplusplus
}
#endif

#endif /* LEAP_UP_SYSTEM_H_ */