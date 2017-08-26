/*
 * h_and_H_magne.c
 *
 * Code generation for function 'h_and_H_magne'
 *
 * C source code generated on: Sat Dec 13 23:33:19 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "EIF_n_dof.h"
#include "h_and_H_magne.h"
#include "EIF_n_dof_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void h_and_H_magne(const real_T X[7], real_T h[3], real_T H[21])
{
  real_T M_q[16];
  real_T a[4];
  int32_T i;
  static const real_T dv3[3] = { 0.2102, 0.036, -0.4447 };

  real_T M_B[16];
  real_T b_M_B[4];
  int32_T i5;
  real_T b_M_q[4];
  real_T c_M_q[16];
  int32_T i6;
  real_T b_a[16];

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
  a[0] = 0.0;
  for (i = 0; i < 3; i++) {
    B_0[i] = dv3[i];
    a[i + 1] = B_0[i];
  }

  M_B[0] = 0.0;
  M_B[4] = -a[1];
  M_B[8] = -a[2];
  M_B[12] = -a[3];
  M_B[1] = a[1];
  M_B[5] = 0.0;
  M_B[9] = -a[3];
  M_B[13] = a[2];
  M_B[2] = a[2];
  M_B[6] = a[3];
  M_B[10] = 0.0;
  M_B[14] = -a[1];
  M_B[3] = a[3];
  M_B[7] = -a[2];
  M_B[11] = a[1];
  M_B[15] = 0.0;

  /*  Hamilton(qu*,Hamilton(B_0,qu)) */
  for (i = 0; i < 4; i++) {
    b_M_B[i] = 0.0;
    for (i5 = 0; i5 < 4; i5++) {
      b_M_B[i] += M_B[i + (i5 << 2)] * X[i5];
    }
  }

  for (i = 0; i < 4; i++) {
    b_M_q[i] = 0.0;
    for (i5 = 0; i5 < 4; i5++) {
      b_M_q[i] += M_q[i5 + (i << 2)] * b_M_B[i5];
    }
  }

  for (i = 0; i < 3; i++) {
    h[i] = b_M_q[i + 1];
  }

  /*  H */
  memset(&H[0], 0, 21U * sizeof(real_T));
  for (i = 0; i < 4; i++) {
    a[i] = 0.0;
    for (i5 = 0; i5 < 4; i5++) {
      a[i] += M_B[i + (i5 << 2)] * X[i5];
    }

    for (i5 = 0; i5 < 4; i5++) {
      c_M_q[i + (i5 << 2)] = 0.0;
      for (i6 = 0; i6 < 4; i6++) {
        c_M_q[i + (i5 << 2)] += M_q[i6 + (i << 2)] * M_B[i6 + (i5 << 2)];
      }
    }
  }

  b_a[0] = -a[0];
  b_a[4] = -a[1];
  b_a[8] = -a[2];
  b_a[12] = -a[3];
  b_a[1] = a[1];
  b_a[5] = -a[0];
  b_a[9] = -a[3];
  b_a[13] = a[2];
  b_a[2] = a[2];
  b_a[6] = a[3];
  b_a[10] = -a[0];
  b_a[14] = -a[1];
  b_a[3] = a[3];
  b_a[7] = -a[2];
  b_a[11] = a[1];
  b_a[15] = -a[0];
  for (i = 0; i < 4; i++) {
    for (i5 = 0; i5 < 4; i5++) {
      M_q[i5 + (i << 2)] = c_M_q[i5 + (i << 2)] + b_a[i5 + (i << 2)];
    }

    for (i5 = 0; i5 < 3; i5++) {
      H[i5 + 3 * i] = M_q[(i5 + (i << 2)) + 1];
    }
  }
}

/* End of code generation (h_and_H_magne.c) */
