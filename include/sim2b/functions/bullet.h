// SPDX-License-Identifier: LGPL-3.0
#ifndef SIM2B_FUNCTIONS_BULLET_H
#define SIM2B_FUNCTIONS_BULLET_H

#include <sim2b/types/bullet.h>


#ifdef __cplusplus
extern "C" {
#endif

void sim2b_bullet_configure(struct sim2b_bullet_nbx *b);

void sim2b_bullet_start(struct sim2b_bullet_nbx *b);

void sim2b_bullet_step(struct sim2b_bullet_nbx *b);

void sim2b_bullet_cleanup(struct sim2b_bullet_nbx *b);

#ifdef __cplusplus
}
#endif

#endif
