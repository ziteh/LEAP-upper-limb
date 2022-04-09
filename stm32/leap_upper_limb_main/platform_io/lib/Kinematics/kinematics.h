/**
 * @file kinematics.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief
 */

#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <math.h>

void InverseKinematics2(double x, double y, double l1, double l2, double *out_r1, double *out_r2);
void ForwardKinematics2(double r1, double r2, double l1, double l2, double *out_x, double *out_y);

#endif /* __KINEMATICS_H */