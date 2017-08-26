/*
 * EIF_n_dof.c
 *
 * Code generation for function 'EIF_n_dof'
 *
 * C source code generated on: Tue Dec 16 23:30:26 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "EIF_n_dof.h"
#include "inv.h"
#include "f_and_F_pred.h"
#include "EIF_n_dof_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real_T rt_atan2d_snf(real_T u0, real_T u1);

/* Function Definitions */
static real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T b_u0;
  int32_T b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((real_T)b_u0, (real_T)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void EIF_n_dof(const real_T X[7], const real_T P[49], const real_T Q[36], const
               real_T sigmas[2], const real_T measures[6], real_T AT, real_T
               eul[3], real_T X_out[7], real_T P_out[49])
{
  real_T Gamma[42];
  real_T Y_info_p[49];
  real_T X_p[7];
  real_T y[42];
  int32_T cr;
  int32_T ic;
  int32_T br;
  int32_T ar;
  int32_T ib;
  int32_T ia;
  real_T b_Y_info_p[49];
  real_T Gamma_data[42];
  real_T c_Y_info_p[49];
  real_T b_y[49];
  int8_T H_gyro[21];
  real_T R_gyro[9];
  static const int8_T b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real_T b_b[9];
  real_T c_b[9];
  real_T M_q[16];
  real_T H_accel[21];
  real_T b_M_q[16];
  real_T c_y[4];
  static const int8_T a[16] = { 0, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 0, 0
  };

  real_T d_y[16];
  real_T c_M_q[16];
  real_T G_accel[9];
  real_T x[2];
  real_T e_y;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T b_G_accel[9];
  real_T c_G_accel[9];
  real_T d_b[9];
  real_T b_H_gyro[21];
  real_T b_H_accel[21];
  real_T b_measures[3];
  real_T d_M_q[4];
  real_T c_measures[3];
  real_T dv0[7];
  real_T c_H_accel[7];
  real_T d_Y_info_p[7];

  /*  Define variables */
  /*  coder.varsize('measures', [3*number_sensors 1]); */
  /* % Pred */
  f_and_F_pred(X, AT, X_p, Y_info_p, Gamma);
  memset(&y[0], 0, 42U * sizeof(real_T));
  for (cr = 0; cr < 37; cr += 7) {
    for (ic = cr; ic + 1 <= cr + 7; ic++) {
      y[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr < 37; cr += 7) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 6; ib++) {
      if (Q[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 7; ic++) {
          ia++;
          y[ic] += Q[ib] * Gamma[ia - 1];
        }
      }

      ar += 7;
    }

    br += 6;
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 7; cr++) {
      b_Y_info_p[ar + 7 * cr] = 0.0;
      for (br = 0; br < 7; br++) {
        b_Y_info_p[ar + 7 * cr] += Y_info_p[ar + 7 * br] * P[br + 7 * cr];
      }
    }
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 6; cr++) {
      Gamma_data[cr + 6 * ar] = Gamma[ar + 7 * cr];
    }
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 7; cr++) {
      c_Y_info_p[ar + 7 * cr] = 0.0;
      for (br = 0; br < 7; br++) {
        c_Y_info_p[ar + 7 * cr] += b_Y_info_p[ar + 7 * br] * Y_info_p[cr + 7 *
          br];
      }

      b_y[ar + 7 * cr] = 0.0;
      for (br = 0; br < 6; br++) {
        b_y[ar + 7 * cr] += y[ar + 7 * br] * Gamma_data[br + 6 * cr];
      }
    }
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 7; cr++) {
      b_Y_info_p[cr + 7 * ar] = c_Y_info_p[cr + 7 * ar] + b_y[cr + 7 * ar];
    }
  }

  inv(b_Y_info_p, Y_info_p);

  /*     %% Update */
  /*  Detect which sensors we got and compute information and Matrix */
  /*  information */
  /*  Info from gyros */
  /*  h */
  /*  H */
  for (ar = 0; ar < 21; ar++) {
    H_gyro[ar] = 0;
  }

  memset(&R_gyro[0], 0, 9U * sizeof(real_T));
  for (cr = 0; cr < 3; cr++) {
    R_gyro[cr + 3 * cr] = 1.0;
    for (ar = 0; ar < 3; ar++) {
      H_gyro[ar + 3 * (4 + cr)] = (int8_T)R_gyro[ar + 3 * cr];
    }
  }

  for (ar = 0; ar < 9; ar++) {
    R_gyro[ar] = sigmas[0] * (real_T)b[ar];
  }

  b_inv(R_gyro, b_b);
  b_inv(R_gyro, c_b);

  /*  Info from acceleroemters */
  /*  h */
  M_q[0] = X_p[0];
  M_q[4] = -X_p[1];
  M_q[8] = -X_p[2];
  M_q[12] = -X_p[3];
  M_q[1] = X_p[1];
  M_q[5] = X_p[0];
  M_q[9] = -X_p[3];
  M_q[13] = X_p[2];
  M_q[2] = X_p[2];
  M_q[6] = X_p[3];
  M_q[10] = X_p[0];
  M_q[14] = -X_p[1];
  M_q[3] = X_p[3];
  M_q[7] = -X_p[2];
  M_q[11] = X_p[1];
  M_q[15] = X_p[0];

  /*  Hamilton(a,b) == Matrix_4_product(a)*b */
  /*  Hamilton(qu*,Hamilton(gv,qu)) */
  /*  H */
  memset(&H_accel[0], 0, 21U * sizeof(real_T));
  for (ar = 0; ar < 4; ar++) {
    c_y[ar] = 0.0;
    for (cr = 0; cr < 4; cr++) {
      c_y[ar] += (real_T)a[ar + (cr << 2)] * X_p[cr];
    }

    for (cr = 0; cr < 4; cr++) {
      b_M_q[ar + (cr << 2)] = 0.0;
      for (br = 0; br < 4; br++) {
        b_M_q[ar + (cr << 2)] += M_q[br + (ar << 2)] * (real_T)a[br + (cr << 2)];
      }
    }
  }

  d_y[0] = -c_y[0];
  d_y[4] = -c_y[1];
  d_y[8] = -c_y[2];
  d_y[12] = -c_y[3];
  d_y[1] = c_y[1];
  d_y[5] = -c_y[0];
  d_y[9] = -c_y[3];
  d_y[13] = c_y[2];
  d_y[2] = c_y[2];
  d_y[6] = c_y[3];
  d_y[10] = -c_y[0];
  d_y[14] = -c_y[1];
  d_y[3] = c_y[3];
  d_y[7] = -c_y[2];
  d_y[11] = c_y[1];
  d_y[15] = -c_y[0];
  for (ar = 0; ar < 4; ar++) {
    for (cr = 0; cr < 4; cr++) {
      c_M_q[cr + (ar << 2)] = b_M_q[cr + (ar << 2)] + d_y[cr + (ar << 2)];
    }

    for (cr = 0; cr < 3; cr++) {
      H_accel[cr + 3 * ar] = c_M_q[(cr + (ar << 2)) + 1];
    }
  }

  for (ar = 0; ar < 9; ar++) {
    R_gyro[ar] = sigmas[1] * (real_T)b[ar];

    /*  global Ad_param */
    /* % 1st method */
    /*  1./(0.05+exp(-(x-1).^2/0.05)) */
    /*  Ad_param(end+1) = 1./(0.02+exp(-(norm(Accelerometer)-1).^2/0.2))*(  1./(0.02+exp(-norm([omega_est(1),omega_est(3)]).^2/0.00002))  ); */
    G_accel[ar] = 0.0;
  }

  for (cr = 0; cr < 3; cr++) {
    G_accel[cr + 3 * cr] = 1.0;
  }

  x[0] = measures[0];
  x[1] = measures[2];
  e_y = 0.0;
  scale = 2.2250738585072014E-308;
  for (cr = 0; cr < 2; cr++) {
    absxk = fabs(x[cr]);
    if (absxk > scale) {
      t = scale / absxk;
      e_y = 1.0 + e_y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      e_y += t * t;
    }
  }

  e_y = scale * sqrt(e_y);
  e_y = -rt_powd_snf(e_y, 2.0) / 2.0E-7;
  G_accel[4] = 1.0 / (0.1 + exp(e_y));
  G_accel[8] = 1.0 / (0.1 + exp(e_y));

  /* % No adaptation */
  /*  G = eye(3); */
  for (ar = 0; ar < 3; ar++) {
    for (cr = 0; cr < 3; cr++) {
      c_G_accel[ar + 3 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        c_G_accel[ar + 3 * cr] += G_accel[ar + 3 * br] * R_gyro[br + 3 * cr];
      }
    }

    for (cr = 0; cr < 3; cr++) {
      b_G_accel[ar + 3 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        b_G_accel[ar + 3 * cr] += c_G_accel[ar + 3 * br] * G_accel[cr + 3 * br];
      }
    }
  }

  b_inv(b_G_accel, d_b);
  for (ar = 0; ar < 3; ar++) {
    for (cr = 0; cr < 3; cr++) {
      c_G_accel[ar + 3 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        c_G_accel[ar + 3 * cr] += G_accel[ar + 3 * br] * R_gyro[br + 3 * cr];
      }
    }

    for (cr = 0; cr < 3; cr++) {
      b_G_accel[ar + 3 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        b_G_accel[ar + 3 * cr] += c_G_accel[ar + 3 * br] * G_accel[cr + 3 * br];
      }
    }
  }

  b_inv(b_G_accel, R_gyro);

  /*  Retrieving the phisical variables */
  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 3; cr++) {
      b_H_gyro[ar + 7 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        b_H_gyro[ar + 7 * cr] += (real_T)H_gyro[br + 3 * ar] * c_b[br + 3 * cr];
      }

      b_H_accel[ar + 7 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        b_H_accel[ar + 7 * cr] += H_accel[br + 3 * ar] * R_gyro[br + 3 * cr];
      }
    }
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 7; cr++) {
      e_y = 0.0;
      for (br = 0; br < 3; br++) {
        e_y += b_H_gyro[ar + 7 * br] * (real_T)H_gyro[br + 3 * cr];
      }

      c_Y_info_p[ar + 7 * cr] = e_y;
    }
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 7; cr++) {
      b_y[ar + 7 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        b_y[ar + 7 * cr] += b_H_accel[ar + 7 * br] * H_accel[br + 3 * cr];
      }
    }
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 7; cr++) {
      b_Y_info_p[cr + 7 * ar] = Y_info_p[cr + 7 * ar] + (c_Y_info_p[cr + 7 * ar]
        + b_y[cr + 7 * ar]);
    }
  }

  inv(b_Y_info_p, P_out);
  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 3; cr++) {
      b_H_gyro[ar + 7 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        b_H_gyro[ar + 7 * cr] += (real_T)H_gyro[br + 3 * ar] * b_b[br + 3 * cr];
      }
    }
  }

  for (ar = 0; ar < 3; ar++) {
    e_y = 0.0;
    for (cr = 0; cr < 7; cr++) {
      e_y += (real_T)H_gyro[ar + 3 * cr] * X_p[cr];
    }

    b_measures[ar] = (measures[ar] - X_p[ar + 4]) + e_y;
  }

  for (ar = 0; ar < 4; ar++) {
    c_y[ar] = 0.0;
    for (cr = 0; cr < 4; cr++) {
      c_y[ar] += (real_T)a[ar + (cr << 2)] * X_p[cr];
    }
  }

  for (ar = 0; ar < 4; ar++) {
    d_M_q[ar] = 0.0;
    for (cr = 0; cr < 4; cr++) {
      d_M_q[ar] += M_q[cr + (ar << 2)] * c_y[cr];
    }
  }

  for (ar = 0; ar < 7; ar++) {
    for (cr = 0; cr < 3; cr++) {
      b_H_accel[ar + 7 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        b_H_accel[ar + 7 * cr] += H_accel[br + 3 * ar] * d_b[br + 3 * cr];
      }
    }
  }

  for (ar = 0; ar < 3; ar++) {
    e_y = 0.0;
    for (cr = 0; cr < 7; cr++) {
      e_y += H_accel[ar + 3 * cr] * X_p[cr];
    }

    c_measures[ar] = (measures[ar + 3] - d_M_q[ar + 1]) + e_y;
  }

  for (ar = 0; ar < 7; ar++) {
    e_y = 0.0;
    for (cr = 0; cr < 3; cr++) {
      e_y += b_H_gyro[ar + 7 * cr] * b_measures[cr];
    }

    dv0[ar] = e_y;
  }

  for (ar = 0; ar < 7; ar++) {
    c_H_accel[ar] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      c_H_accel[ar] += b_H_accel[ar + 7 * cr] * c_measures[cr];
    }
  }

  for (ar = 0; ar < 7; ar++) {
    e_y = 0.0;
    for (cr = 0; cr < 7; cr++) {
      e_y += Y_info_p[ar + 7 * cr] * X_p[cr];
    }

    d_Y_info_p[ar] = e_y + (dv0[ar] + c_H_accel[ar]);
  }

  for (ar = 0; ar < 7; ar++) {
    X_out[ar] = 0.0;
    for (cr = 0; cr < 7; cr++) {
      X_out[ar] += P_out[ar + 7 * cr] * d_Y_info_p[cr];
    }
  }

  /*  Required normalization */
  e_y = 0.0;
  scale = 2.2250738585072014E-308;
  for (cr = 0; cr < 4; cr++) {
    absxk = fabs(X_out[cr]);
    if (absxk > scale) {
      t = scale / absxk;
      e_y = 1.0 + e_y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      e_y += t * t;
    }
  }

  e_y = scale * sqrt(e_y);
  for (ar = 0; ar < 4; ar++) {
    X_out[ar] /= e_y;
  }

  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  eul[0] = rt_atan2d_snf(2.0 * X_out[2] * X_out[3] + 2.0 * X_out[0] * X_out[1],
                         (2.0 * rt_powd_snf(X_out[0], 2.0) + 2.0 * rt_powd_snf
    (X_out[3], 2.0)) - 1.0);

  /*  Balance phi */
  eul[1] = -asin(2.0 * X_out[1] * X_out[3] - 2.0 * X_out[0] * X_out[2]);

  /*  Angulo asiento theta */
  eul[2] = rt_atan2d_snf(2.0 * X_out[1] * X_out[2] + 2.0 * X_out[0] * X_out[3],
                         (2.0 * rt_powd_snf(X_out[0], 2.0) + 2.0 * rt_powd_snf
    (X_out[1], 2.0)) - 1.0);

  /*  Angulo de guiñada psi */
}

/* End of code generation (EIF_n_dof.c) */
