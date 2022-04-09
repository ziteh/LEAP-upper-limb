/**
 * @file   force_sensor.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  3-Axis force sensor.
 */

#include "force_sensor.h"
#include <math.h>

extern ADC_HandleTypeDef FORCE_SENSOR_ADC_HANDLE;

void ForceSensor_CoordinateConvert(double theta, double raw_x, double raw_y, double raw_z, double *out_x, double *out_y, double *out_z);

void ForceSensor_GetValue(double theta, double *out_x, double *out_y, double *out_z)
{
  int16_t raw_x = ForceSensor_GetRawValue(X);
  int16_t raw_y = ForceSensor_GetRawValue(Y);
  int16_t raw_z = ForceSensor_GetRawValue(Z);

  ForceSensor_CoordinateConvert(theta,
                                raw_x, raw_y, raw_z,
                                out_x, out_y, out_z);
}

int16_t ForceSensor_GetRawValue(ForceSensor_AxisTypeDef axis)
{
  uint32_t channel_a;
  uint32_t channel_b;

  switch (axis)
  {
  case X:
    channel_a = FORCE_SENSOR_ADC_CHANNEL_0;
    channel_b = FORCE_SENSOR_ADC_CHANNEL_1;
    break;

  case Y:
    channel_a = FORCE_SENSOR_ADC_CHANNEL_2;
    channel_b = FORCE_SENSOR_ADC_CHANNEL_3;
    break;

  case Z:
  default:
    // TODO.
    return 0;
    break;
  }

  uint16_t value_a = ADC_GetValue(&FORCE_SENSOR_ADC_HANDLE, channel_a);
  uint16_t value_b = ADC_GetValue(&FORCE_SENSOR_ADC_HANDLE, channel_b);
  return value_a - value_b;
}

void ForceSensor_PrintRawValue(void)
{
  int16_t value_x = ForceSensor_GetRawValue(X);
  int16_t value_y = ForceSensor_GetRawValue(Y);
  int16_t value_z = ForceSensor_GetRawValue(Z);

  printf("Force Sensor. X: %5d, Y: %5d, Z: %5d\r\n", value_x, value_y, value_z);
}

void ForceSensor_CoordinateConvert(double theta, double raw_x, double raw_y, double raw_z, double *out_x, double *out_y, double *out_z)
{
  /* Degree to Rad. */
  theta *= M_PI / 180.0;

  double x = raw_x * cos(theta) - raw_y * sin(theta);
  double y = raw_x * sin(theta) + raw_y * cos(theta);
  double z = raw_z;

  *out_x = x;
  *out_y = y;
  *out_z = z;
}