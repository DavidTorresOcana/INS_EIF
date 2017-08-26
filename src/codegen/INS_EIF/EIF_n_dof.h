/*
 * EIF_n_dof.h
 *
 * Code generation for function 'EIF_n_dof'
 *
 * C source code generated on: Tue Dec 16 23:30:26 2014
 *
 */

#ifndef __EIF_N_DOF_H__
#define __EIF_N_DOF_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "EIF_n_dof_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void EIF_n_dof(const real_T X[7], const real_T P[49], const real_T Q[36], const real_T sigmas[2], const real_T measures[6], real_T AT, real_T eul[3], real_T X_out[7], real_T P_out[49]);
#endif
/* End of code generation (EIF_n_dof.h) */
