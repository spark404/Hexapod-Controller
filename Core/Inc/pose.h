#ifndef POSE_H
#define POSE_H

typedef struct {
	double x, y, z;
	double roll, pitch, yaw;
} pose_t;

void pose_set(pose_t *pose, double x, double y, double z, , double roll, double pitch, double yaw);
void pose_translate(pose_t *pose, double x, double y, double z);
void pose_rotate(pose_t *pose, double roll, double pitch, double yaw);

#endif /* POSE_H */
