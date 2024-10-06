#include "conversion_2d.h"

void convert_2d_polar_to_cartesian(const float32_t *polar, float32_t *cartesian)
{
	cartesian[0] = polar[0] * arm_cos_f32(polar[1]);
	cartesian[1] = polar[0] * arm_sin_f32(polar[1]);
}
