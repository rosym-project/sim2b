// SPDX-License-Identifier: LGPL-3.0
#include "bullet_ubx.h"

#include <sim2b/types/bullet.h>
#include <sim2b/functions/bullet.h>

struct sim2b_bullet_ubx_private {
  struct sim2b_bullet_ubx_port_cache ports;
  struct sim2b_bullet_nbx nblock;
  // Buffers
  int nr_joints;
  double *grav;
  double *jnt_pos_init;
  char *urdf;
  int *ctrl_mode;
  double *jnt_pos_msr;
  double *jnt_vel_msr;
  double *jnt_pos_cmd;
  double *jnt_vel_cmd;
  double *jnt_eff_cmd;
  double *jnt_eff_max;
};

int sim2b_bullet_ubx_init(ubx_block_t *b) {
  int ret = -1;
  struct sim2b_bullet_ubx_private *private;
  if ((private = calloc(1, sizeof(struct sim2b_bullet_ubx_private))) == NULL) {
    ubx_err(b, "sim2b_bullet: failed to alloc memory");
    ret = EOUTOFMEM;
    goto out;
  }
  b->private_data = private;

  struct sim2b_bullet_nbx *nblock = &private->nblock;

  // Get configuration information.
  const int *nr_joints_temp;
  int nr_joints_len = cfg_getptr_int(b, "nr_joints", &nr_joints_temp);
  if (nr_joints_len != 1) {
    ubx_err(b, "sim2b_bullet: nr_joints must be supplied");
    goto out;
  }
  private->nr_joints = *nr_joints_temp;

  const double *gravity_temp;
  int gravity_len = cfg_getptr_double(b, "gravity", &gravity_temp);
  if (gravity_len == 3) {
    private->grav = calloc(1, gravity_len * sizeof(double));
    memcpy(private->grav, gravity_temp, gravity_len * sizeof(double));
  } else if (gravity_len == 0) {
    private->grav = calloc(1, 3 * sizeof(double));
    private->grav[0] = 0;
    private->grav[1] = 0;
    private->grav[2] = -9.81;
  } else {
    ubx_err(b, "sim2b_bullet: if supplied, gravity must consist of three doubles");
    goto out;
  }


  const double *jnt_pos_init_temp;
  int jnt_pos_init_len = cfg_getptr_double(b, "jnt_pos_init", &jnt_pos_init_temp);
  if (jnt_pos_init_len != private->nr_joints) {
    ubx_err(b, "sim2b_bullet: rank of jnt_pos_init vector must match nr_joints");
    goto out;
  }
  private->jnt_pos_init = calloc(1, jnt_pos_init_len * sizeof(double));
  memcpy(private->jnt_pos_init, jnt_pos_init_temp, jnt_pos_init_len * sizeof(double));

  const char *urdf_temp;
  int urdf_length = cfg_getptr_char(b, "urdf", &urdf_temp);
  private->urdf = calloc(1, urdf_length * sizeof(char));
  strncpy(private->urdf, urdf_temp, urdf_length);

  // Cache ports and set dimensions.
  private->ports.ctrl_mode   = ubx_port_get(b, "ctrl_mode");
  private->ports.jnt_pos_msr = ubx_port_get(b, "jnt_pos_msr");
  private->ports.jnt_pos_msr->out_data_len = private->nr_joints;
  private->ports.jnt_vel_msr = ubx_port_get(b, "jnt_vel_msr");
  private->ports.jnt_vel_msr->out_data_len = private->nr_joints;
  private->ports.jnt_pos_cmd = ubx_port_get(b, "jnt_pos_cmd");
  private->ports.jnt_pos_cmd->in_data_len = private->nr_joints;
  private->ports.jnt_vel_cmd = ubx_port_get(b, "jnt_vel_cmd");
  private->ports.jnt_vel_cmd->in_data_len = private->nr_joints;
  private->ports.jnt_eff_cmd = ubx_port_get(b, "jnt_eff_cmd");
  private->ports.jnt_eff_cmd->in_data_len = private->nr_joints;
  private->ports.jnt_eff_max = ubx_port_get(b, "jnt_eff_max");
  private->ports.jnt_eff_max->in_data_len = private->nr_joints;

  // Allocate buffers.
  private->ctrl_mode = calloc(1, sizeof(int));
  private->jnt_pos_cmd = calloc(1, private->nr_joints * sizeof(double));
  private->jnt_vel_cmd = calloc(1, private->nr_joints * sizeof(double));
  private->jnt_eff_cmd = calloc(1, private->nr_joints * sizeof(double));
  private->jnt_eff_max = calloc(1, private->nr_joints * sizeof(double));
  private->jnt_pos_msr = calloc(1, private->nr_joints * sizeof(double));
  private->jnt_vel_msr = calloc(1, private->nr_joints * sizeof(double));

  // Connect to nanoblx ports
  nblock->nr_joints = private->nr_joints;
  nblock->grav = private->grav;
  nblock->jnt_pos_init = private->jnt_pos_init;
  nblock->urdf = private->urdf;

  nblock->ctrl_mode = private->ctrl_mode;
  nblock->jnt_pos_cmd = private->jnt_pos_cmd;
  nblock->jnt_vel_cmd = private->jnt_vel_cmd;
  nblock->jnt_eff_cmd = private->jnt_eff_cmd;
  nblock->jnt_eff_max = private->jnt_eff_max;
  nblock->jnt_pos_msr = private->jnt_pos_msr;
  nblock->jnt_vel_msr = private->jnt_vel_msr;

  // Configure simulation
  sim2b_bullet_configure(&private->nblock);
  ret = 0;
 out:
  return ret;
}

void sim2b_bullet_ubx_cleanup(ubx_block_t *b) {
  struct sim2b_bullet_ubx_private *private
    = (struct sim2b_bullet_ubx_private*)b->private_data;
  sim2b_bullet_cleanup(&private->nblock);

  free(private->ctrl_mode);
  free(private->jnt_pos_cmd);
  free(private->jnt_vel_cmd);
  free(private->jnt_eff_cmd);
  free(private->jnt_eff_max);
  free(private->jnt_pos_msr);
  free(private->jnt_vel_msr);

  free(private->urdf);
  free(private->jnt_pos_init);
  free(private->grav);
  free(private);
}

void sim2b_bullet_ubx_step(ubx_block_t *b) {
  struct sim2b_bullet_ubx_private *private
    = (struct sim2b_bullet_ubx_private*)b->private_data;
  struct sim2b_bullet_nbx *nblock = &private->nblock;

  // Read ports
  read_int(private->ports.ctrl_mode, private->ctrl_mode);
  read_double_array(private->ports.jnt_pos_cmd, private->jnt_pos_cmd, nblock->nr_joints);
  read_double_array(private->ports.jnt_vel_cmd, private->jnt_vel_cmd, nblock->nr_joints);
  read_double_array(private->ports.jnt_eff_cmd, private->jnt_eff_cmd, nblock->nr_joints);
  read_double_array(private->ports.jnt_eff_max, private->jnt_eff_max, nblock->nr_joints);

  // Step simulation
  sim2b_bullet_step(&private->nblock);

  // Write ports
  write_double_array(private->ports.jnt_pos_msr, private->jnt_pos_msr, nblock->nr_joints);
  write_double_array(private->ports.jnt_vel_msr, private->jnt_vel_msr, nblock->nr_joints);
}
