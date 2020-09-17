// SPDX-License-Identifier: LGPL-3.0
#ifndef SIM2B_TYPES_BULLET_H
#define SIM2B_TYPES_BULLET_H

#ifdef __cplusplus
extern "C" {
#endif

struct sim2b_bullet;

struct sim2b_bullet_nbx
{
    // Configuration
    int nr_joints;
    const double *grav;         // XYZ [m/s^2]
    const double *jnt_pos_init; // [m] or [rad]
    const char *urdf;
    // Ports
    const int *ctrl_mode;
    double *jnt_pos_msr;        // [m] or [rad]
    double *jnt_vel_msr;        // [m/s] or [rad/s]
    const double *jnt_pos_cmd;  // [m] or [rad]
    const double *jnt_vel_cmd;  // [m/s] or [rad/s]
    const double *jnt_eff_cmd;  // [N] or [Nm]
    const double *jnt_eff_max;  // [N] or [Nm]
    // Internal state
    struct sim2b_bullet *bullet;
};

#ifdef __cplusplus
}
#endif

#endif
