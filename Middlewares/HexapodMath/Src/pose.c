#include "pose.h"

#include "matrix_3d.h"
#include "pose_internal.h"
#include "assert.h"

void pose_set(pose_t pose, float32_t x, float32_t y, float32_t z, float32_t roll, float32_t pitch, float32_t yaw) {
    pose->translation[0] = x;
    pose->translation[1] = y;
    pose->translation[2] = z;

    pose->rotation[0] = roll;
    pose->rotation[1] = pitch;
    pose->rotation[2] = yaw;
}

void pose_get_translation(pose_t pose, float32_t translation[3]) {
    for (int i = 0; i < 3; i++) {
        translation[i] = pose->translation[i];
    }
}

void pose_get_rotation(pose_t pose, float32_t rotation[3]) {
    for (int i = 0; i < 3; i++) {
        rotation[i] = pose->rotation[i];
    }
}

void pose_get_transformation(pose_t pose, arm_matrix_instance_f32 *T) {
    assert(pose != NULL);
    assert(T != NULL);

    assert(T->numCols == 4);
    assert(T->numRows == 4);

    arm_matrix_instance_f32 Tr, Rx, Ry, Rz;
    float32_t pTrData[4 * 4];
    float32_t pRxData[4 * 4];
    float32_t pRyData[4 * 4];
    float32_t pRzData[4 * 4];

    arm_mat_init_f32(&Tr, 4, 4, pTrData);
    arm_mat_init_f32(&Rx, 4, 4, pRxData);
    arm_mat_init_f32(&Ry, 4, 4, pRyData);
    arm_mat_init_f32(&Rz, 4, 4, pRzData);

    matrix_3d_translation_matrix(&Tr, pose->translation);
    matrix_3d_rotation_x_matrix(&Rx, pose->rotation[0]);
    matrix_3d_rotation_y_matrix(&Ry, pose->rotation[1]);
    matrix_3d_rotation_z_matrix(&Rz, pose->rotation[2]);

    arm_matrix_instance_f32 I1, I2;
    float32_t pI1Data[4 * 4];
    float32_t pI2Data[4 * 4];
    arm_mat_init_f32(&I1, 4, 4, pI1Data);
    arm_mat_init_f32(&I2, 4, 4, pI2Data);

    // ZYX rotation
    arm_mat_mult_f32(&Rz, &Ry, &I1);
    arm_mat_mult_f32(&I1, &Rx, &I2);
    arm_mat_mult_f32(&Tr, &I2, T);
}
