/*
 * f_and_F_pred.c
 *
 * Code generation for function 'f_and_F_pred'
 *
 * C source code generated on: Tue Dec 16 23:30:26 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "EIF_n_dof.h"
#include "f_and_F_pred.h"
#include "EIF_n_dof_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void f_and_F_pred(const real_T X[7], real_T AT, real_T X_pred[7], real_T F[49],
                  real_T Gamma[42])
{
  real_T M_q[16];
  real_T y;
  real_T dv1[4];
  real_T b_M_q[4];
  real_T c_M_q[7];
  int32_T i0;
  int32_T i;
  int8_T I[16];
  real_T dv2[16];
  real_T b_y;
  real_T c_y;
  static const int8_T b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  /*  f */
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
  y = AT / 2.0;
  dv1[0] = 0.0;
  dv1[1] = X[4];
  dv1[2] = X[5];
  dv1[3] = X[6];
  for (i0 = 0; i0 < 4; i0++) {
    b_M_q[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      b_M_q[i0] += M_q[i0 + (i << 2)] * dv1[i];
    }

    c_M_q[i0] = b_M_q[i0];
  }

  c_M_q[4] = 0.0;
  c_M_q[5] = 0.0;
  c_M_q[6] = 0.0;
  for (i0 = 0; i0 < 7; i0++) {
    X_pred[i0] = X[i0] + y * c_M_q[i0];
  }

  /*  F */
  /*  Derivative of hamilton(a,b) in terms of a OR Derivative of Matrix_4_product(a)*b in terms of a */
  memset(&F[0], 0, 49U * sizeof(real_T));
  for (i = 0; i < 7; i++) {
    F[i + 7 * i] = 1.0;
  }

  for (i0 = 0; i0 < 16; i0++) {
    I[i0] = 0;
  }

  y = AT / 2.0;
  dv2[0] = 0.0;
  dv2[4] = -X[4];
  dv2[8] = -X[5];
  dv2[12] = -X[6];
  dv2[1] = X[4];
  dv2[5] = 0.0;
  dv2[9] = X[6];
  dv2[13] = -X[5];
  dv2[2] = X[5];
  dv2[6] = -X[6];
  dv2[10] = 0.0;
  dv2[14] = X[4];
  dv2[3] = X[6];
  dv2[7] = X[5];
  dv2[11] = -X[4];
  dv2[15] = 0.0;
  for (i = 0; i < 4; i++) {
    I[i + (i << 2)] = 1;
    for (i0 = 0; i0 < 4; i0++) {
      F[i0 + 7 * i] = (real_T)I[i0 + (i << 2)] + y * dv2[i0 + (i << 2)];
    }
  }

  y = AT / 2.0;

  /*  Gamma: It is the noise model */
  b_y = AT / 2.0;
  c_y = rt_powd_snf(AT, 2.0) / 2.0;
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 4; i++) {
      F[i + 7 * (4 + i0)] = y * M_q[i + ((1 + i0) << 2)];
      Gamma[i + 7 * i0] = b_y * M_q[i + ((1 + i0) << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 4; i++) {
      Gamma[i + 7 * (i0 + 3)] = c_y * M_q[i + ((1 + i0) << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      Gamma[(i + 7 * i0) + 4] = 0.0;
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      Gamma[(i + 7 * (i0 + 3)) + 4] = AT * (real_T)b[i + 3 * i0];
    }
  }
}

/* End of code generation (f_and_F_pred.c) */
