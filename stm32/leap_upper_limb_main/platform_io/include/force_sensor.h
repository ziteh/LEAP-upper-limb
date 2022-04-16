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

#define FORCE_SENSOR_THRESHOLD_X (250)
#define FORCE_SENSOR_THRESHOLD_Y (250)

typedef enum
{
  X,
  Y,
  Z
} ForceSensor_AxisTypeDef;

void ForceSensor_GetValue(double theta, double *out_x, double *out_y, double *out_z);
void ForceSensor_GetRawValue(int16_t *x, int16_t *y, int16_t *z);
void ForceSensor_PrintRawValue(void);
void ForceSensor_CoordinateConvert(double theta, double raw_x, double raw_y, double raw_z, double *out_x, double *out_y, double *out_z);

#endif /* __FORCE_SENSOR_H */