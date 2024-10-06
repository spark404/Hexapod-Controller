#ifndef HEXAPODMATH_INC_INVERSE_KINEMATICS_H_
#define HEXAPODMATH_INC_INVERSE_KINEMATICS_H_

#include "arm_math.h"

void calculate_leg_angles(const float32_t *origin, const float32_t *tip, float32_t *angles);

#endif /* HEXAPODMATH_INC_INVERSE_KINEMATICS_H_ */
