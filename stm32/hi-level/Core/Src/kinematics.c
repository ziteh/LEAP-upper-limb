/*
 * kinematics.c
 */

#include "kinematics.h"

void IK2(float x, float y, float l1, float l2, float *out_r1, float *out_r2)
{
  if (x > (l1 + l2))
  {
    x = l1 + l2;
  }

  float xyPower = pow(x, 2) + pow(y, 2);
  //  float a = sqrt((pow(l1 + l2, 2) - xyPower))/(xyPower - pow(l1 - l2, 2));
  //  float b = 2.0f * acos(sqrt(1/(pow(a, 2)+1)));
  //  *out_r2 = abs(b);

  float a = 2.0f * atan2(sqrt(pow(l1 + l2, 2) - xyPower),
                         sqrt(xyPower - pow(l1 - l2, 2)));
  if (a < 0)
    a *= -1;

  *out_r2 = a;

  *out_r1 = (atan2(y, x) + atan2(l2 * sin(-*out_r2), l1 + l2 * cos(-*out_r2)));
}

void FK2(float r1, float r2, float l1, float l2, float *out_x, float *out_y)
{
  *out_x = l1 * cos(r1) + l2 * cos(r1 + r2);
  *out_y = l1 * sin(r1) + l2 * sin(r1 + r2);
}
