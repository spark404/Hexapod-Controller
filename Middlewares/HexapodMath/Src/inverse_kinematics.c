#include "inverse_kinematics.h"

void calculate_leg_angles(const float32_t *origin, const float32_t *tip, float32_t *angles)
{
	float32_t delta_x = tip[0] - origin[0];
	float32_t delta_y = tip[1] - origin[1];
	float32_t delta_z = tip[2] - origin[2];

	// TODO This should come from a robot definition file
	float32_t coxa = 24; // mm
	float32_t femur = 100; // mm
	float32_t tibia = 150; // mm

	float32_t femur_sq = femur * femur;
	float32_t tibia_sq = tibia * tibia;

	/* We can calculate the hip (tibia) angle using the Z and X
	 * difference, giving us a right sides triangle to work with.
	 */
	float32_t omega;
	arm_atan2_f32(delta_z, delta_x, &omega);

	/* The remainder of the calculations should exclude the coxa length.
	 * So we define a new origin for the calculations
	 */
	float32_t femur_origin[3];
	femur_origin[0] = origin[0] + coxa;
	femur_origin[1] = origin[1];
	femur_origin[2] = origin[2];

    /* We know three things
     *  - the distance from origin to target (pythagoras), side c
     *  - the height difference between origin and target, side a
     *  - the corner opposite to the distance is 90 deg
     */
	float32_t origin_tip_distance = arm_euclidean_distance_f32(femur_origin, tip, 3);

	// using the distance we can use the pythagoras theorem to compute the remaining side b
	float32_t b;
	arm_sqrt_f32(origin_tip_distance * origin_tip_distance - delta_z * delta_z, &b);

	float32_t alpha_accent = asinf(delta_y / b);

	/* Use the law of cosines to find the remaining angles
	 * as we know all sides, but none of the angles
	 * c**2 = a**2 + b**2 - 2ab * cos C
	 */
	float32_t alpha = acosf((femur_sq + b*b - tibia_sq) / (2 * femur * b)) + alpha_accent;
	float32_t beta = (float32_t)M_PI - acosf((tibia_sq + femur_sq - b*b) / (2 * femur * tibia));

	angles[0] = alpha;
	angles[1] = beta;
	angles[2] = omega;
}
