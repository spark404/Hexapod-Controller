//
// Created by Hugo Trippaers on 20/07/2025.
//

#ifndef CALCULATOR_H
#define CALCULATOR_H

#include "arm_math_types.h"

void calculate_motion_step(float32_t current[3], float32_t target[3], float32_t next[3], float32_t delta_t_s);

#endif //CALCULATOR_H
