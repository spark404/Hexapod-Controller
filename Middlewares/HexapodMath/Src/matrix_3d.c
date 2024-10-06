#include "matrix_3d.h"

typedef arm_matrix_instance_f32 *matrix;

void matrix_3d_translation_init(matrix T){
	// Initialize matrix T as transformation matrix
	for (int i = 0; i<4; i++) {
		for (int j = 0; j<4; j++) {
			T->pData[i * 4 + j] = i == j;
		}
	}
}

void matrix_3d_translation_matrix(matrix T, float32_t translation[3])
{
	matrix_3d_translation_init(T);

	// Apply the translation
	for (int i = 0; i<3; i++) {
		T->pData[i * 4 + 3] = translation[i];
	}
}

void matrix_3d_rotation_x_matrix(matrix Rx, float32_t omega)
{
	matrix_3d_translation_init(Rx);

	float32_t c = arm_cos_f32(omega);
	float32_t s = arm_sin_f32(omega);

	Rx->pData[1 * 4 + 1] = c;
	Rx->pData[1 * 4 + 2] = -s;
	Rx->pData[2 * 4 + 1] = s;
	Rx->pData[2 * 4 + 2] = c;
}

void matrix_3d_rotation_y_matrix(matrix Ry, float32_t omega)
{
	matrix_3d_translation_init(Ry);

	float32_t c = arm_cos_f32(omega);
	float32_t s = arm_sin_f32(omega);

	Ry->pData[0 * 4 + 0] = c;
	Ry->pData[0 * 4 + 2] = s;
	Ry->pData[2 * 4 + 0] = -s;
	Ry->pData[2 * 4 + 2] = c;
}

void matrix_3d_rotation_z_matrix(matrix Rz, float32_t omega)
{
	matrix_3d_translation_init(Rz);

	float32_t c = arm_cos_f32(omega);
	float32_t s = arm_sin_f32(omega);

	Rz->pData[0 * 4 + 0] = c;
	Rz->pData[0 * 4 + 1] = -s;
	Rz->pData[1 * 4 + 0] = s;
	Rz->pData[1 * 4 + 1] = c;
}