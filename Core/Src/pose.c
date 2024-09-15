#include "pose.h"

void pose_set(pose_t *pose, double x, double y, double z, , double roll, double pitch, double yaw) {
	pose->x = x;
	pose->y = y;
	pose->z = z;
	pose->roll = roll;
	pose->pitch = pitch;
	pose->yaw = yaw;
}

void pose_translate(pose_t *pose, double x, double y, double z) {
	pose->x += x;
	pose->y += y;
	pose->z += z;
}

void pose_rotate(pose_t *pose, double roll, double pitch, double yaw) {
	pose->roll += roll;
	pose->pitch += pitch;
	pose->yaw += yaw;
}
