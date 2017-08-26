/*
 * inv.h
 *
 * Code generation for function 'inv'
 *
 * C source code generated on: Tue Dec 16 23:30:26 2014
 *
 */

#ifndef __INV_H__
#define __INV_H__
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
extern void b_inv(const real_T x[9], real_T y[9]);
extern void inv(const real_T x[49], real_T y[49]);
#endif
/* End of code generation (inv.h) */
