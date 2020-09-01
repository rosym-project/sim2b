// SPDX-License-Identifier: LGPL-3.0
#include <sim2b/functions/bullet.h>

#include <stdio.h>

#define NR_JOINTS 2


int main(void)
{
    int ctrl_mode = 2;
    double pos_msr[NR_JOINTS];
    double vel_msr[NR_JOINTS];
    double pos_cmd[NR_JOINTS] = { 0.0, 0.0 };
    double vel_cmd[NR_JOINTS] = { 0.0, 0.0 };
    double eff_cmd[NR_JOINTS] = { 100.0, 100.0 };

    struct sim2b_bullet_nbx sim = {
        // Configuration
        .nr_joints = NR_JOINTS,
        .grav = { 0.0, 0.0, -9.81 },
        .jnt_pos_init = (double [NR_JOINTS]) { 0.0, 1.571 },
        .urdf = "2dof.urdf",
        // Connections
        .ctrl_mode = &ctrl_mode,
        .jnt_pos_msr = pos_msr,
        .jnt_vel_msr = vel_msr,
        .jnt_pos_cmd = pos_cmd,
        .jnt_vel_cmd = vel_cmd,
        .jnt_eff_cmd = eff_cmd
    };

    sim2b_bullet_configure(&sim);
    sim2b_bullet_start(&sim);

    for (int i = 0; i < 100; i++) {
        sim2b_bullet_step(&sim);
        printf("%f - %f\n", pos_msr[0], pos_msr[1]);
    }

    sim2b_bullet_cleanup(&sim);

    return 0;
}
