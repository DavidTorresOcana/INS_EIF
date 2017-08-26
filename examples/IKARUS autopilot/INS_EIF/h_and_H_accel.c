/*
 * h_and_H_accel.c
 *
 * Code generation for function 'h_and_H_accel'
 *
 * C source code generated on: Sat Dec 13 23:33:19 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "EIF_n_dof.h"
#include "h_and_H_accel.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void h_and_H_accel(const real_T X[7], real_T h[3], real_T H[21])
{
  real_T M_q[16];
  real_T a[4];
  int32_T i2;
  int32_T i3;
  static const int8_T b_a[16] = { 0, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 0,
    0 };

  real_T b_M_q[4];
  real_T c_M_q[16];
  real_T y[4];
  int32_T i4;
  real_T b_y[16];

  /*  h */
  M_q[0] = X[0];
  M_q[4] = -X[1];
  M_q[8] = -X[2];
  M_q[12] = -X[3];
  M_q[1] = X[1];
  M_q[5] = X[0];
  M_q[9] = -X[3];
  M_q[13] = X[2];
  M_q[2] = X[2];
  M_q[6] = X[3];
  M_q[10] = X[0];
  M_q[14] = -X[1];
  M_q[3] = X[3];
  M_q[7] = -X[2];
  M_q[11] = X[1];
  M_q[15] = X[0];

  /*  Hamilton(a,b) == Matrix_4_product(a)*b */
  /*  Hamilton(qu*,Hamilton(gv,qu)) */
  for (i2 = 0; i2 < 4; i2++) {
    a[i2] = 0.0;
    for (i3 = 0; i3 < 4; i3++) {
      a[i2] += (real_T)b_a[i2 + (i3 << 2)] * X[i3];
    }
  }

  for (i2 = 0; i2 < 4; i2++) {
    b_M_q[i2] = 0.0;
    for (i3 = 0; i3 < 4; i3++) {
      b_M_q[i2] += M_q[i3 + (i2 << 2)] * a[i3];
    }
  }

  for (i2 = 0; i2 < 3; i2++) {
    h[i2] = b_M_q[i2 + 1];
  }

  /*  H */
  memset(&H[0], 0, 21U * sizeof(real_T));
  for (i2 = 0; i2 < 4; i2++) {
    y[i2] = 0.0;
    for (i3 = 0; i3 < 4; i3++) {
      y[i2] += (real_T)b_a[i2 + (i3 << 2)] * X[i3];
    }

    for (i3 = 0; i3 < 4; i3++) {
      c_M_q[i2 + (i3 << 2)] = 0.0;
      for (i4 = 0; i4 < 4; i4++) {
        c_M_q[i2 + (i3 << 2)] += M_q[i4 + (i2 << 2)] * (real_T)b_a[i4 + (i3 << 2)];
      }
    }
  }

  b_y[0] = -y[0];
  b_y[4] = -y[1];
  b_y[8] = -y[2];
  b_y[12] = -y[3];
  b_y[1] = y[1];
  b_y[5] = -y[0];
  b_y[9] = -y[3];
  b_y[13] = y[2];
  b_y[2] = y[2];
  b_y[6] = y[3];
  b_y[10] = -y[0];
  b_y[14] = -y[1];
  b_y[3] = y[3];
  b_y[7] = -y[2];
  b_y[11] = y[1];
  b_y[15] = -y[0];
  for (i2 = 0; i2 < 4; i2++) {
    for (i3 = 0; i3 < 4; i3++) {
      M_q[i3 + (i2 << 2)] = c_M_q[i3 + (i2 << 2)] + b_y[i3 + (i2 << 2)];
    }

    for (i3 = 0; i3 < 3; i3++) {
      H[i3 + 3 * i2] = M_q[(i3 + (i2 << 2)) + 1];
    }
  }
}

/* End of code generation (h_and_H_accel.c) */
