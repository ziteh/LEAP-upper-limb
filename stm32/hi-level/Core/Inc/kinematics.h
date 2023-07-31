/*
 * kinematics.h
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <math.h>

void IK2(float x, float y, float l1, float l2, float *out_r1, float *out_r2);
void FK2(float r1, float r2, float l1, float l2, float *out_x, float *out_y);

#endif /* KINEMATICS_H_ */
