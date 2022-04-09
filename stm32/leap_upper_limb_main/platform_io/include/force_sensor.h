/**
 * @file   force_sensor.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  3-Axis force sensor header.
 */

#ifndef __FORCE_SENSOR_H
#define __FORCE_SENSOR_H

#include "adc.h"
#include "printf.h"
#include "stm32f4xx_hal.h"

#define FORCE_SENSOR_ADC_HANDLE (hadc1)

#define FORCE_SENSOR_ADC_CHANNEL_0 (ADC_CHANNEL_0)
#define FORCE_SENSOR_ADC_CHANNEL_1 (ADC_CHANNEL_1)
#define FORCE_SENSOR_ADC_CHANNEL_2 (ADC_CHANNEL_4)
#define FORCE_SENSOR_ADC_CHANNEL_3 (ADC_CHANNEL_8)

typedef enum
{
  X,
  Y,
  Z
} ForceSensor_AxisTypeDef;

void ForceSensor_GetValue(double theta, double *out_x, double *out_y, double *out_z);
int16_t ForceSensor_GetRawValue(ForceSensor_AxisTypeDef axis);
void ForceSensor_PrintRawValue(void);

#endif /* __FORCE_SENSOR_H */