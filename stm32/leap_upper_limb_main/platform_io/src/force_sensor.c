/**
 * @file   force_sensor.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  3-Axis force sensor.
 */

#include "force_sensor.h"

extern ADC_HandleTypeDef FORCE_SENSOR_ADC_HANDLE;

int16_t ForceSensor_GetValue(ForceSensor_AxisTypeDef axis)
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

void ForceSensor_PrintValue(void)
{
  int16_t value_x = ForceSensor_GetValue(X);
  int16_t value_y = ForceSensor_GetValue(Y);
  int16_t value_z = ForceSensor_GetValue(Z);

  printf("Force Sensor. X: %5d, Y: %5d, Z: %5d\r\n", value_x, value_y, value_z);
}