/*
 * pose.h
 *
 *  Created on: Oct 5, 2024
 *      Author: hugo
 */

#ifndef HEXAPODMATH_INC_POSE_H_
#define HEXAPODMATH_INC_POSE_H_

#include "arm_math.h"

typedef struct pose *pose_t;

void pose_set(pose_t pose, float32_t x, float32_t y, float32_t z, float32_t roll, float32_t pitch, float32_t yaw);
void pose_get_translation(pose_t pose, float32_t translation[3]);
void pose_get_rotation(pose_t pose, float32_t rotation[3]);

void pose_get_transformation(pose_t pose, arm_matrix_instance_f32 *T);

#endif /* HEXAPODMATH_INC_POSE_H_ */
