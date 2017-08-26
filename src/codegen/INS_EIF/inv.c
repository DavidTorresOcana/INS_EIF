/*
 * inv.c
 *
 * Code generation for function 'inv'
 *
 * C source code generated on: Tue Dec 16 23:30:26 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "EIF_n_dof.h"
#include "inv.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void b_inv(const real_T x[9], real_T y[9])
{
  real_T b_x[9];
  int32_T p1;
  int32_T p2;
  int32_T p3;
  real_T absx11;
  real_T absx21;
  real_T absx31;
  int32_T itmp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(x[0]);
  absx21 = fabs(x[1]);
  absx31 = fabs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = x[2];
      b_x[2] = x[0];
      b_x[3] = x[5];
      b_x[5] = x[3];
      b_x[6] = x[8];
      b_x[8] = x[6];
    }
  }

  absx11 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx21 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= absx11 * b_x[3];
  b_x[5] -= absx21 * b_x[3];
  b_x[7] -= absx11 * b_x[6];
  b_x[8] -= absx21 * b_x[6];
  if (fabs(b_x[5]) > fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx21;
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  absx11 = b_x[5] / b_x[4];
  b_x[5] /= b_x[4];
  b_x[8] -= absx11 * b_x[7];
  absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0 / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

void inv(const real_T x[49], real_T y[49])
{
  real_T A[49];
  int32_T i1;
  int8_T ipiv[7];
  int32_T j;
  int32_T c;
  int32_T pipk;
  int32_T ix;
  real_T smax;
  int32_T k;
  real_T s;
  int32_T jy;
  int32_T ijA;
  int8_T p[7];
  for (i1 = 0; i1 < 49; i1++) {
    y[i1] = 0.0;
    A[i1] = x[i1];
  }

  for (i1 = 0; i1 < 7; i1++) {
    ipiv[i1] = (int8_T)(1 + i1);
  }

  for (j = 0; j < 6; j++) {
    c = j << 3;
    pipk = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 2; k <= 7 - j; k++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        pipk = k - 1;
        smax = s;
      }
    }

    if (A[c + pipk] != 0.0) {
      if (pipk != 0) {
        ipiv[j] = (int8_T)((j + pipk) + 1);
        ix = j;
        pipk += j;
        for (k = 0; k < 7; k++) {
          smax = A[ix];
          A[ix] = A[pipk];
          A[pipk] = smax;
          ix += 7;
          pipk += 7;
        }
      }

      i1 = (c - j) + 7;
      for (jy = c + 1; jy + 1 <= i1; jy++) {
        A[jy] /= A[c];
      }
    }

    pipk = c;
    jy = c + 7;
    for (k = 1; k <= 6 - j; k++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i1 = (pipk - j) + 14;
        for (ijA = 8 + pipk; ijA + 1 <= i1; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 7;
      pipk += 7;
    }
  }

  for (i1 = 0; i1 < 7; i1++) {
    p[i1] = (int8_T)(1 + i1);
  }

  for (k = 0; k < 6; k++) {
    if (ipiv[k] > 1 + k) {
      pipk = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (int8_T)pipk;
    }
  }

  for (k = 0; k < 7; k++) {
    y[k + 7 * (p[k] - 1)] = 1.0;
    for (j = k; j + 1 < 8; j++) {
      if (y[j + 7 * (p[k] - 1)] != 0.0) {
        for (jy = j + 1; jy + 1 < 8; jy++) {
          y[jy + 7 * (p[k] - 1)] -= y[j + 7 * (p[k] - 1)] * A[jy + 7 * j];
        }
      }
    }
  }

  for (j = 0; j < 7; j++) {
    c = 7 * j;
    for (k = 6; k > -1; k += -1) {
      pipk = 7 * k;
      if (y[k + c] != 0.0) {
        y[k + c] /= A[k + pipk];
        for (jy = 0; jy + 1 <= k; jy++) {
          y[jy + c] -= y[k + c] * A[jy + pipk];
        }
      }
    }
  }
}

/* End of code generation (inv.c) */
