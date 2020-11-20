// SPDX-License-Identifier: LGPL-3.0
#pragma once

#include <ubx.h>

UBX_MODULE_LICENSE_SPDX(LGPL3)

/* Types and type metadata */
ubx_type_t sim2b_bullet_ubx_types[] = {
};

char sim2b_bullet_ubx_meta[] = "";

/* Block configuration */
ubx_proto_config_t sim2b_bullet_ubx_config[] = {
  { .name = "nr_joints", .type_name = "long", .min=1, .max=1, .doc="Number of joints" },
  { .name = "gravity", .type_name = "double", .min=0, .max=3, .doc="Acceleration vector for simulated gravity (XYZ [m/s^2])" },
  { .name = "jnt_pos_init", .type_name = "double", .min=1, .doc="Initial joint positions ([m] or [rad] depending on joint type)" },
  { .name = "urdf", .type_name = "char", .min=1, .doc="Path to URDF file containing a description of the robot that should be simulated" },
  { 0 }
};

/* Block ports */
ubx_proto_port_t sim2b_bullet_ubx_ports[] = {
  // Inputs
  { .name="ctrl_mode", .in_type_name="int", .in_data_len=1, .doc="Control mode; 0: VELOCITY_PD, 1: VELOCITY (jnt_vel_cmd port), 2: TORQUE (jnt_eff_cmd port)" },
  { .name="jnt_pos_cmd", .in_type_name="double", .doc="Commanded joint positions ([m] or [rad] depending on joint type)"},
  { .name="jnt_vel_cmd", .in_type_name="double", .doc="Commanded joint velocities ([m/s] or [rad/s] depending on joint type)"},
  { .name="jnt_eff_cmd", .in_type_name="double", .doc="Commanded joint torques ([N] or [Nm] depending on joint type)"},
  { .name="jnt_eff_max", .in_type_name="double", .doc="Maximal joint torques ([N] or [Nm] depending on joint type)"},
  // Outputs
  { .name="jnt_pos_msr", .out_type_name="double", .doc="Measured joint positions ([m] or [rad] depending on joint type)" },
  { .name="jnt_vel_msr", .out_type_name="double", .doc="Measured joint velocities ([m/s] or [rad/s] depending on joint type)" },
  { 0 }
};

/* Port cache */
struct sim2b_bullet_ubx_port_cache {
  ubx_port_t *ctrl_mode;
  ubx_port_t *jnt_pos_msr;
  ubx_port_t *jnt_vel_msr;
  ubx_port_t *jnt_pos_cmd;
  ubx_port_t *jnt_vel_cmd;
  ubx_port_t *jnt_eff_cmd;
  ubx_port_t *jnt_eff_max;
};

/* Hook forward declarations */
int sim2b_bullet_ubx_init(ubx_block_t *b);
void sim2b_bullet_ubx_cleanup(ubx_block_t *b);
void sim2b_bullet_ubx_step(ubx_block_t *b);

ubx_proto_block_t sim2b_bullet_ubx_block = {
  .name      = "sim2b/sim_bullet",
  .type      = BLOCK_TYPE_COMPUTATION,
  .meta_data = sim2b_bullet_ubx_meta,
  .configs   = sim2b_bullet_ubx_config,
  .ports     = sim2b_bullet_ubx_ports,
  /* Operations */
  .init      = sim2b_bullet_ubx_init,
  .cleanup   = sim2b_bullet_ubx_cleanup,
  .step      = sim2b_bullet_ubx_step
};

int sim2b_bullet_ubx_mod_init(ubx_node_t *n) {
  if (ubx_block_register(n, &sim2b_bullet_ubx_block) != 0) {
    return -1;
  }
  return 0;
}
UBX_MODULE_INIT(sim2b_bullet_ubx_mod_init)

void sim2b_bullet_ubx_mod_cleanup(ubx_node_t *n) {
  ubx_block_unregister(n, "sim2b/sim_bullet");
}
UBX_MODULE_CLEANUP(sim2b_bullet_ubx_mod_cleanup)
