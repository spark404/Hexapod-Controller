//
// Created by Hugo Trippaers on 20/07/2025.
//

#include "arm_math.h"
#include "hexapodmath/additional_functions.h"

static float32_t velocity = 50; // mm/s
static float32_t lift_height = 20; // mm
static float32_t lift_velocity = 20;  // mm/s

void calculate_motion_step(float32_t current[3], float32_t target[3], float32_t next[3], float32_t delta_t_s) {
    float32_t direction[2];
    float32_t unit_direction[2];
    arm_vec_sub_f32(target, current, direction, 2);
    arm_vec_normalize_f32(direction, unit_direction, 2);

    float32_t remaining_distance = arm_euclidean_distance_f32(current, target, 2);
    float32_t remaining_duration_s = remaining_distance / velocity;

    float32_t movement[2] = {0, 0};
    float32_t max_movement = velocity * delta_t_s;
    if (remaining_distance < max_movement) {
        max_movement = remaining_distance;
    }
    arm_vec_mult_scalar_f32(unit_direction, max_movement, movement, 2);

    // Compute list distance and duration
    float32_t lift_distance = target[2] - current[2] + lift_height;
    float32_t lift_duration_s = fabsf(lift_distance) / lift_velocity;
    float32_t lower_duration_s = lift_height / lift_velocity;

    // If double the lift_duration is greater than the total
    // time to move the leg we need to recalculate the lift height
    float32_t calculated_lift_height = lift_height;
    if ((2 *lift_duration_s ) > remaining_duration_s) {
        calculated_lift_height = calculated_lift_height * (remaining_duration_s / (2 * lift_duration_s));
    }

    // If the time taken to lower the leg is great than the remaining
    // movement we need to adjust the lift_height
    if (lower_duration_s > remaining_duration_s) {
        calculated_lift_height = lift_height * (remaining_duration_s / lower_duration_s);
    }

    float32_t target_height = target[2] + calculated_lift_height;

    float32_t z = current[2];
    float32_t max_z_movement = fminf(lift_velocity * delta_t_s, fabsf(target_height - current[2]));

    if (current[2] < target_height) {
        z = fminf(current[2] + max_z_movement, target[2] + calculated_lift_height);
    } else if (current[2] > target_height) {
        z = current[2] - max_z_movement;
    }

    next[0] = current[0] + movement[0];
    next[1] = current[1] + movement[1];
    next[2] = z;
}
