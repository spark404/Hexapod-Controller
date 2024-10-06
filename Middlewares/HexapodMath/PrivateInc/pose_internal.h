/*
 * pose_internal.h
 *
 *  Created on: Oct 5, 2024
 *      Author: hugo
 */

#ifndef HEXAPODMATH_PRIVATEINC_POSE_INTERNAL_H_
#define HEXAPODMATH_PRIVATEINC_POSE_INTERNAL_H_

#include "arm_math.h"

struct pose {
	float32_t translation[3]; // X, Y, Z
	float32_t rotation[3];    // Roll, Pitch, Yaw
};


#endif /* HEXAPODMATH_PRIVATEINC_POSE_INTERNAL_H_ */
