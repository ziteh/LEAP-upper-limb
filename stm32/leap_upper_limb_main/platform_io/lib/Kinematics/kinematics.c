/**
 * @file kinematics.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief
 */

#include "kinematics.h"

#define DEGREE_TO_RAD(DEGREE) (DEGREE * M_PI / 180.0)
#define RAD_TO_DEGREE(RAD) (RAD * 180.0 / M_PI)

void InverseKinematics2(double x, double y, double l1, double l2, double *out_r1, double *out_r2)
{
  double r2 = 2 * atan2(sqrt(pow(l1 + l2, 2) - pow(x, 2) + pow(y, 2)),
                        sqrt(pow(x, 2) + pow(y, 2) - pow(l1 - l2, 2)));

  double r1 = -(atan2(x, y) + atan2(l2 * sin(r2), l1 + l2 * cos(r2)));

  *out_r1 = RAD_TO_DEGREE(r1);
  *out_r2 = RAD_TO_DEGREE(r2);
}

void ForwardKinematics2(double r1, double r2, double l1, double l2, double *out_x, double *out_y)
{
  r1 = DEGREE_TO_RAD(r1);
  r2 = DEGREE_TO_RAD(r2);

  double x = l1 * cos(r1) + l2 * cos(r1 + r2);
  double y = -(l1 * sin(r1) + l2 * sin(r1 + r2));

  *out_x = x;
  *out_y = y;
}