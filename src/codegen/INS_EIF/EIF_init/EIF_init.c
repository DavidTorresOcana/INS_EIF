/*
 * EIF_init.c
 *
 * Code generation for function 'EIF_init'
 *
 * C source code generated on: Tue Dec 16 23:16:58 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "EIF_init.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void EIF_init(const real_T eul_0[3], real_T X_0[7], real_T P_0[49], real_T Q[36],
              real_T sigmas[2])
{
  static const real_T dv0[49] = { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01 };

  static const real_T dv1[36] = { 0.0005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001 };

  int32_T i;
  memcpy(&P_0[0], &dv0[0], 49U * sizeof(real_T));

  /* % EIF initialization */
  /* rad */
  memcpy(&Q[0], &dv1[0], 36U * sizeof(real_T));
  Q[14] = 0.0005;
  Q[35] = 1.0E-5;

  /*  rad */
  /*  g's */
  /*  sigma_mag = 0.1;% */
  for (i = 0; i < 2; i++) {
    sigmas[i] = 0.1;
  }

  /*  R = blkdiag(R_gyro,R_accel,R_magne) ; */
  /*  options=optimset('MaxFunEvals',3000); */
  /*  q=fsolve(@QUATERNIONEULER,[0.6,0.4,0.5,0.4]); % Hay que acertar con los angulos poniendo diferentes valores iniciales del quaternion */
  /*      function F=QUATERNIONEULER(q) */
  /*          F=[ tan(phi)-(2*q(3)*q(4)-2*q(1)*q(2))/(2*q(1)^2+2*q(4)^2-1)  ; */
  /*              sin(-theta)-2*q(2)*q(4)-2*q(1)*q(3); */
  /*              tan(psi)-(2*q(2)*q(3)-2*q(1)*q(4))/(2*q(1)^2+2*q(2)^2-1); */
  /*              q(1)^2+q(2)^2+q(3)^2+q(4)^2-1]; */
  /*      end  % Este algoritmo da q* !!!!! */
  /*  Esto nos da q !!! */
  X_0[0] = cos(eul_0[2] / 2.0) * cos(eul_0[1] / 2.0) * cos(eul_0[0] / 2.0) + sin
    (eul_0[2] / 2.0) * sin(eul_0[1] / 2.0) * sin(eul_0[0] / 2.0);
  X_0[1] = cos(eul_0[2] / 2.0) * cos(eul_0[1] / 2.0) * sin(eul_0[0] / 2.0) - sin
    (eul_0[2] / 2.0) * sin(eul_0[1] / 2.0) * cos(eul_0[0] / 2.0);
  X_0[2] = cos(eul_0[2] / 2.0) * sin(eul_0[1] / 2.0) * cos(eul_0[0] / 2.0) + sin
    (eul_0[2] / 2.0) * cos(eul_0[1] / 2.0) * sin(eul_0[0] / 2.0);
  X_0[3] = sin(eul_0[2] / 2.0) * cos(eul_0[1] / 2.0) * cos(eul_0[0] / 2.0) - cos
    (eul_0[2] / 2.0) * sin(eul_0[1] / 2.0) * sin(eul_0[0] / 2.0);
  X_0[4] = 0.0;
  X_0[5] = 0.0;
  X_0[6] = 0.0;

  /*  Y_info=inv(P_0); */
  /*  y_info=Y_info*X_0; */
}

/* End of code generation (EIF_init.c) */
