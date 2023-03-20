/**
 * @file   force_sensor.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  3-Axis force sensor.
 */

#include "force_sensor.h"
#include <math.h>

// #define REVERS_X
#define REVERS_Y
// #define REVERS_Z

extern ADC_HandleTypeDef FORCE_SENSOR_ADC_HANDLE;

void ForceSensor_GetValue(double theta, double *out_x, double *out_y, double *out_z)
{
  int16_t raw_x, raw_y, raw_z;
  ForceSensor_GetRawValue(&raw_x, &raw_y, &raw_z);

  ForceSensor_CoordinateConvert(theta,
                                raw_x, raw_y, raw_z,
                                out_x, out_y, out_z);
}

void ForceSensor_GetRawValue(int16_t *x, int16_t *y, int16_t *z)
{
  uint16_t ch0_value, ch1_value, ch2_value, ch3_value;
  while (1)
  {
    int state;
    state = ADC_GetValue(&FORCE_SENSOR_ADC_HANDLE, FORCE_SENSOR_ADC_CHANNEL_0, &ch0_value);
    if (state != 0)
    {
      continue;
    }

    state = ADC_GetValue(&FORCE_SENSOR_ADC_HANDLE, FORCE_SENSOR_ADC_CHANNEL_1, &ch1_value);
    if (state != 0)
    {
      continue;
    }

    state = ADC_GetValue(&FORCE_SENSOR_ADC_HANDLE, FORCE_SENSOR_ADC_CHANNEL_2, &ch2_value);
    if (state != 0)
    {
      continue;
    }

    state = ADC_GetValue(&FORCE_SENSOR_ADC_HANDLE, FORCE_SENSOR_ADC_CHANNEL_3, &ch3_value);
    if (state == 0)
    {
      break;
    }
  }

/* X. */
#ifdef REVERS_X
  *x = ch0_value - ch1_value;
#else
  *x = ch1_value - ch0_value;
#endif

/* Y. */
#ifdef REVERS_Y
  *y = ch2_value - ch3_value;
#else
  *y = ch3_value - ch2_value;
#endif

/* Z. */
int offset = 325;
#ifdef REVERS_Z
  *z = ((((ch0_value + ch1_value) / 2) + (ch2_value + ch3_value) / 2) / 2) - (4095 / 2) + offset;
  *z *= -1;
#else
  *z = ((((ch0_value + ch1_value) / 2) + (ch2_value + ch3_value) / 2) / 2) - (4095 / 2) + offset;
#endif
}

void ForceSensor_PrintRawValue(void)
{
  int16_t value_x, value_y, value_z;
  ForceSensor_GetRawValue(&value_x, &value_y, &value_z);

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