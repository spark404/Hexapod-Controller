//
// Created by Hugo Trippaers on 24/07/2025.
//

#ifndef SERVOS_H
#define SERVOS_H

#include "arm_math.h"
#include "dynamixel/dynamixel.h"

void read_actual_servo_position(dynamixel_servo_t *servos, const uint8_t servo_count, float32_t *actual_servo_angles);
void write_next_servo_position(dynamixel_servo_t *servos, const uint8_t servo_count, const float32_t *next_servo_angles);

#endif //SERVOS_H
