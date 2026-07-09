/*
 * quaternion.h
 *
 *  Created on: Mar 17, 2026
 *      Author: kyojin
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <stdint.h>

typedef struct {float w, x, y, z; } Quaternion;

inline Quaternion q_mul(Quaternion q1, Quaternion q2);
inline Quaternion q_scale(Quaternion q1, float scalar);
inline Quaternion q_add(Quaternion q1, Quaternion q2);
inline Quaternion q_sub(Quaternion q1, Quaternion q2);
inline Quaternion q_norm(Quaternion q1);
inline Quaternion q_star(Quaternion q1);
inline float      q_mag(Quaternion q1);
float             inv_sqrt(float f);

#endif /* QUATERNION_H_ */
