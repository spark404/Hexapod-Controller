//
// Created by Hugo Trippaers on 30/12/2025.
//

#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#define CTRL_LOOP_DT_S        0.2f
#define CTRL_CLOSE_THRESH    3.0f
#define CTRL_MAX_YAW_RATE    0.1f
#define CTRL_LEG_RADIUS_MM   40.0f
#define CTRL_EPS_VEL         1e-3f
#define CTRL_EPS_ANG         1e-3f
#define CTRL_SERVO_Z_OFFSET  D2R(25)
#define CTRL_POWERDOWN_TIMEOUT 30  // sec
#define CTRL_DEFAULT_VEL    50.0f
#define CTRL_LIFT_VEL      20.0f
#define CTRL_LIFT_Z        50.0f
#define CTRL_BODY_Z        100.0f // mm

#endif //CONTROLLER_CONFIG_H
