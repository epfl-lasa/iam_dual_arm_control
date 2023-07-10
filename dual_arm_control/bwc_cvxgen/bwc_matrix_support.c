/* Produced by CVXGEN, 2019-04-06 15:00:00 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "bwc_solver.h"
void bwc_multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1)-rhs[6]*(1)-rhs[12]*(1);
  lhs[1] = -rhs[1]*(1)-rhs[7]*(1)-rhs[13]*(1);
  lhs[2] = -rhs[2]*(1)-rhs[8]*(1)-rhs[14]*(1);
  lhs[3] = -rhs[0]*(bwc_params.Gh_4[0])-rhs[6]*(bwc_params.Gh_4[6])-rhs[1]*(bwc_params.Gh_4[1])-rhs[7]*(bwc_params.Gh_4[7])-rhs[2]*(bwc_params.Gh_4[2])-rhs[8]*(bwc_params.Gh_4[8])-rhs[3]*(1)-rhs[9]*(1)-rhs[15]*(1);
  lhs[4] = -rhs[0]*(bwc_params.Gh_5[0])-rhs[6]*(bwc_params.Gh_5[6])-rhs[1]*(bwc_params.Gh_5[1])-rhs[7]*(bwc_params.Gh_5[7])-rhs[2]*(bwc_params.Gh_5[2])-rhs[8]*(bwc_params.Gh_5[8])-rhs[4]*(1)-rhs[10]*(1)-rhs[16]*(1);
  lhs[5] = -rhs[0]*(bwc_params.Gh_6[0])-rhs[6]*(bwc_params.Gh_6[6])-rhs[1]*(bwc_params.Gh_6[1])-rhs[7]*(bwc_params.Gh_6[7])-rhs[2]*(bwc_params.Gh_6[2])-rhs[8]*(bwc_params.Gh_6[8])-rhs[5]*(1)-rhs[11]*(1)-rhs[17]*(1);
  lhs[6] = -rhs[0]*(bwc_params.Cplh[0])-rhs[1]*(bwc_params.Cplh[1])-rhs[2]*(bwc_params.Cplh[2])-rhs[3]*(bwc_params.Cplh[3])-rhs[4]*(bwc_params.Cplh[4])-rhs[5]*(bwc_params.Cplh[5]);
  lhs[7] = -rhs[6]*(bwc_params.Cprh[0])-rhs[7]*(bwc_params.Cprh[1])-rhs[8]*(bwc_params.Cprh[2])-rhs[9]*(bwc_params.Cprh[3])-rhs[10]*(bwc_params.Cprh[4])-rhs[11]*(bwc_params.Cprh[5]);
}
void bwc_multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1)-rhs[3]*(bwc_params.Gh_4[0])-rhs[4]*(bwc_params.Gh_5[0])-rhs[5]*(bwc_params.Gh_6[0])-rhs[6]*(bwc_params.Cplh[0]);
  lhs[1] = -rhs[1]*(1)-rhs[3]*(bwc_params.Gh_4[1])-rhs[4]*(bwc_params.Gh_5[1])-rhs[5]*(bwc_params.Gh_6[1])-rhs[6]*(bwc_params.Cplh[1]);
  lhs[2] = -rhs[2]*(1)-rhs[3]*(bwc_params.Gh_4[2])-rhs[4]*(bwc_params.Gh_5[2])-rhs[5]*(bwc_params.Gh_6[2])-rhs[6]*(bwc_params.Cplh[2]);
  lhs[3] = -rhs[3]*(1)-rhs[6]*(bwc_params.Cplh[3]);
  lhs[4] = -rhs[4]*(1)-rhs[6]*(bwc_params.Cplh[4]);
  lhs[5] = -rhs[5]*(1)-rhs[6]*(bwc_params.Cplh[5]);
  lhs[6] = -rhs[0]*(1)-rhs[3]*(bwc_params.Gh_4[6])-rhs[4]*(bwc_params.Gh_5[6])-rhs[5]*(bwc_params.Gh_6[6])-rhs[7]*(bwc_params.Cprh[0]);
  lhs[7] = -rhs[1]*(1)-rhs[3]*(bwc_params.Gh_4[7])-rhs[4]*(bwc_params.Gh_5[7])-rhs[5]*(bwc_params.Gh_6[7])-rhs[7]*(bwc_params.Cprh[1]);
  lhs[8] = -rhs[2]*(1)-rhs[3]*(bwc_params.Gh_4[8])-rhs[4]*(bwc_params.Gh_5[8])-rhs[5]*(bwc_params.Gh_6[8])-rhs[7]*(bwc_params.Cprh[2]);
  lhs[9] = -rhs[3]*(1)-rhs[7]*(bwc_params.Cprh[3]);
  lhs[10] = -rhs[4]*(1)-rhs[7]*(bwc_params.Cprh[4]);
  lhs[11] = -rhs[5]*(1)-rhs[7]*(bwc_params.Cprh[5]);
  lhs[12] = -rhs[0]*(1);
  lhs[13] = -rhs[1]*(1);
  lhs[14] = -rhs[2]*(1);
  lhs[15] = -rhs[3]*(1);
  lhs[16] = -rhs[4]*(1);
  lhs[17] = -rhs[5]*(1);
}
void bwc_multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(bwc_params.CLH_1[0])-rhs[1]*(bwc_params.CLH_1[1])-rhs[2]*(bwc_params.CLH_1[2])-rhs[3]*(bwc_params.CLH_1[3])-rhs[4]*(bwc_params.CLH_1[4])-rhs[5]*(bwc_params.CLH_1[5]);
  lhs[1] = -rhs[0]*(bwc_params.CLH_2[0])-rhs[1]*(bwc_params.CLH_2[1])-rhs[2]*(bwc_params.CLH_2[2])-rhs[3]*(bwc_params.CLH_2[3])-rhs[4]*(bwc_params.CLH_2[4])-rhs[5]*(bwc_params.CLH_2[5]);
  lhs[2] = -rhs[0]*(bwc_params.CLH_3[0])-rhs[1]*(bwc_params.CLH_3[1])-rhs[2]*(bwc_params.CLH_3[2])-rhs[3]*(bwc_params.CLH_3[3])-rhs[4]*(bwc_params.CLH_3[4])-rhs[5]*(bwc_params.CLH_3[5]);
  lhs[3] = -rhs[0]*(bwc_params.CLH_4[0])-rhs[1]*(bwc_params.CLH_4[1])-rhs[2]*(bwc_params.CLH_4[2])-rhs[3]*(bwc_params.CLH_4[3])-rhs[4]*(bwc_params.CLH_4[4])-rhs[5]*(bwc_params.CLH_4[5]);
  lhs[4] = -rhs[0]*(bwc_params.CLH_5[0])-rhs[1]*(bwc_params.CLH_5[1])-rhs[2]*(bwc_params.CLH_5[2])-rhs[3]*(bwc_params.CLH_5[3])-rhs[4]*(bwc_params.CLH_5[4])-rhs[5]*(bwc_params.CLH_5[5]);
  lhs[5] = -rhs[0]*(bwc_params.CLH_6[0])-rhs[1]*(bwc_params.CLH_6[1])-rhs[2]*(bwc_params.CLH_6[2])-rhs[3]*(bwc_params.CLH_6[3])-rhs[4]*(bwc_params.CLH_6[4])-rhs[5]*(bwc_params.CLH_6[5]);
  lhs[6] = -rhs[0]*(bwc_params.CLH_7[0])-rhs[1]*(bwc_params.CLH_7[1])-rhs[2]*(bwc_params.CLH_7[2])-rhs[3]*(bwc_params.CLH_7[3])-rhs[4]*(bwc_params.CLH_7[4])-rhs[5]*(bwc_params.CLH_7[5]);
  lhs[7] = -rhs[0]*(bwc_params.CLH_8[0])-rhs[1]*(bwc_params.CLH_8[1])-rhs[2]*(bwc_params.CLH_8[2])-rhs[3]*(bwc_params.CLH_8[3])-rhs[4]*(bwc_params.CLH_8[4])-rhs[5]*(bwc_params.CLH_8[5]);
  lhs[8] = -rhs[0]*(bwc_params.CLH_9[0])-rhs[1]*(bwc_params.CLH_9[1])-rhs[2]*(bwc_params.CLH_9[2])-rhs[3]*(bwc_params.CLH_9[3])-rhs[4]*(bwc_params.CLH_9[4])-rhs[5]*(bwc_params.CLH_9[5]);
  lhs[9] = -rhs[0]*(bwc_params.CLH_10[0])-rhs[1]*(bwc_params.CLH_10[1])-rhs[2]*(bwc_params.CLH_10[2])-rhs[3]*(bwc_params.CLH_10[3])-rhs[4]*(bwc_params.CLH_10[4])-rhs[5]*(bwc_params.CLH_10[5]);
  lhs[10] = -rhs[0]*(bwc_params.CLH_11[0])-rhs[1]*(bwc_params.CLH_11[1])-rhs[2]*(bwc_params.CLH_11[2])-rhs[3]*(bwc_params.CLH_11[3])-rhs[4]*(bwc_params.CLH_11[4])-rhs[5]*(bwc_params.CLH_11[5]);
  lhs[11] = -rhs[6]*(bwc_params.CRH_1[0])-rhs[7]*(bwc_params.CRH_1[1])-rhs[8]*(bwc_params.CRH_1[2])-rhs[9]*(bwc_params.CRH_1[3])-rhs[10]*(bwc_params.CRH_1[4])-rhs[11]*(bwc_params.CRH_1[5]);
  lhs[12] = -rhs[6]*(bwc_params.CRH_2[0])-rhs[7]*(bwc_params.CRH_2[1])-rhs[8]*(bwc_params.CRH_2[2])-rhs[9]*(bwc_params.CRH_2[3])-rhs[10]*(bwc_params.CRH_2[4])-rhs[11]*(bwc_params.CRH_2[5]);
  lhs[13] = -rhs[6]*(bwc_params.CRH_3[0])-rhs[7]*(bwc_params.CRH_3[1])-rhs[8]*(bwc_params.CRH_3[2])-rhs[9]*(bwc_params.CRH_3[3])-rhs[10]*(bwc_params.CRH_3[4])-rhs[11]*(bwc_params.CRH_3[5]);
  lhs[14] = -rhs[6]*(bwc_params.CRH_4[0])-rhs[7]*(bwc_params.CRH_4[1])-rhs[8]*(bwc_params.CRH_4[2])-rhs[9]*(bwc_params.CRH_4[3])-rhs[10]*(bwc_params.CRH_4[4])-rhs[11]*(bwc_params.CRH_4[5]);
  lhs[15] = -rhs[6]*(bwc_params.CRH_5[0])-rhs[7]*(bwc_params.CRH_5[1])-rhs[8]*(bwc_params.CRH_5[2])-rhs[9]*(bwc_params.CRH_5[3])-rhs[10]*(bwc_params.CRH_5[4])-rhs[11]*(bwc_params.CRH_5[5]);
  lhs[16] = -rhs[6]*(bwc_params.CRH_6[0])-rhs[7]*(bwc_params.CRH_6[1])-rhs[8]*(bwc_params.CRH_6[2])-rhs[9]*(bwc_params.CRH_6[3])-rhs[10]*(bwc_params.CRH_6[4])-rhs[11]*(bwc_params.CRH_6[5]);
  lhs[17] = -rhs[6]*(bwc_params.CRH_7[0])-rhs[7]*(bwc_params.CRH_7[1])-rhs[8]*(bwc_params.CRH_7[2])-rhs[9]*(bwc_params.CRH_7[3])-rhs[10]*(bwc_params.CRH_7[4])-rhs[11]*(bwc_params.CRH_7[5]);
  lhs[18] = -rhs[6]*(bwc_params.CRH_8[0])-rhs[7]*(bwc_params.CRH_8[1])-rhs[8]*(bwc_params.CRH_8[2])-rhs[9]*(bwc_params.CRH_8[3])-rhs[10]*(bwc_params.CRH_8[4])-rhs[11]*(bwc_params.CRH_8[5]);
  lhs[19] = -rhs[6]*(bwc_params.CRH_9[0])-rhs[7]*(bwc_params.CRH_9[1])-rhs[8]*(bwc_params.CRH_9[2])-rhs[9]*(bwc_params.CRH_9[3])-rhs[10]*(bwc_params.CRH_9[4])-rhs[11]*(bwc_params.CRH_9[5]);
  lhs[20] = -rhs[6]*(bwc_params.CRH_10[0])-rhs[7]*(bwc_params.CRH_10[1])-rhs[8]*(bwc_params.CRH_10[2])-rhs[9]*(bwc_params.CRH_10[3])-rhs[10]*(bwc_params.CRH_10[4])-rhs[11]*(bwc_params.CRH_10[5]);
  lhs[21] = -rhs[6]*(bwc_params.CRH_11[0])-rhs[7]*(bwc_params.CRH_11[1])-rhs[8]*(bwc_params.CRH_11[2])-rhs[9]*(bwc_params.CRH_11[3])-rhs[10]*(bwc_params.CRH_11[4])-rhs[11]*(bwc_params.CRH_11[5]);
}
void bwc_multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(bwc_params.CLH_1[0])-rhs[1]*(bwc_params.CLH_2[0])-rhs[2]*(bwc_params.CLH_3[0])-rhs[3]*(bwc_params.CLH_4[0])-rhs[4]*(bwc_params.CLH_5[0])-rhs[5]*(bwc_params.CLH_6[0])-rhs[6]*(bwc_params.CLH_7[0])-rhs[7]*(bwc_params.CLH_8[0])-rhs[8]*(bwc_params.CLH_9[0])-rhs[9]*(bwc_params.CLH_10[0])-rhs[10]*(bwc_params.CLH_11[0]);
  lhs[1] = -rhs[0]*(bwc_params.CLH_1[1])-rhs[1]*(bwc_params.CLH_2[1])-rhs[2]*(bwc_params.CLH_3[1])-rhs[3]*(bwc_params.CLH_4[1])-rhs[4]*(bwc_params.CLH_5[1])-rhs[5]*(bwc_params.CLH_6[1])-rhs[6]*(bwc_params.CLH_7[1])-rhs[7]*(bwc_params.CLH_8[1])-rhs[8]*(bwc_params.CLH_9[1])-rhs[9]*(bwc_params.CLH_10[1])-rhs[10]*(bwc_params.CLH_11[1]);
  lhs[2] = -rhs[0]*(bwc_params.CLH_1[2])-rhs[1]*(bwc_params.CLH_2[2])-rhs[2]*(bwc_params.CLH_3[2])-rhs[3]*(bwc_params.CLH_4[2])-rhs[4]*(bwc_params.CLH_5[2])-rhs[5]*(bwc_params.CLH_6[2])-rhs[6]*(bwc_params.CLH_7[2])-rhs[7]*(bwc_params.CLH_8[2])-rhs[8]*(bwc_params.CLH_9[2])-rhs[9]*(bwc_params.CLH_10[2])-rhs[10]*(bwc_params.CLH_11[2]);
  lhs[3] = -rhs[0]*(bwc_params.CLH_1[3])-rhs[1]*(bwc_params.CLH_2[3])-rhs[2]*(bwc_params.CLH_3[3])-rhs[3]*(bwc_params.CLH_4[3])-rhs[4]*(bwc_params.CLH_5[3])-rhs[5]*(bwc_params.CLH_6[3])-rhs[6]*(bwc_params.CLH_7[3])-rhs[7]*(bwc_params.CLH_8[3])-rhs[8]*(bwc_params.CLH_9[3])-rhs[9]*(bwc_params.CLH_10[3])-rhs[10]*(bwc_params.CLH_11[3]);
  lhs[4] = -rhs[0]*(bwc_params.CLH_1[4])-rhs[1]*(bwc_params.CLH_2[4])-rhs[2]*(bwc_params.CLH_3[4])-rhs[3]*(bwc_params.CLH_4[4])-rhs[4]*(bwc_params.CLH_5[4])-rhs[5]*(bwc_params.CLH_6[4])-rhs[6]*(bwc_params.CLH_7[4])-rhs[7]*(bwc_params.CLH_8[4])-rhs[8]*(bwc_params.CLH_9[4])-rhs[9]*(bwc_params.CLH_10[4])-rhs[10]*(bwc_params.CLH_11[4]);
  lhs[5] = -rhs[0]*(bwc_params.CLH_1[5])-rhs[1]*(bwc_params.CLH_2[5])-rhs[2]*(bwc_params.CLH_3[5])-rhs[3]*(bwc_params.CLH_4[5])-rhs[4]*(bwc_params.CLH_5[5])-rhs[5]*(bwc_params.CLH_6[5])-rhs[6]*(bwc_params.CLH_7[5])-rhs[7]*(bwc_params.CLH_8[5])-rhs[8]*(bwc_params.CLH_9[5])-rhs[9]*(bwc_params.CLH_10[5])-rhs[10]*(bwc_params.CLH_11[5]);
  lhs[6] = -rhs[11]*(bwc_params.CRH_1[0])-rhs[12]*(bwc_params.CRH_2[0])-rhs[13]*(bwc_params.CRH_3[0])-rhs[14]*(bwc_params.CRH_4[0])-rhs[15]*(bwc_params.CRH_5[0])-rhs[16]*(bwc_params.CRH_6[0])-rhs[17]*(bwc_params.CRH_7[0])-rhs[18]*(bwc_params.CRH_8[0])-rhs[19]*(bwc_params.CRH_9[0])-rhs[20]*(bwc_params.CRH_10[0])-rhs[21]*(bwc_params.CRH_11[0]);
  lhs[7] = -rhs[11]*(bwc_params.CRH_1[1])-rhs[12]*(bwc_params.CRH_2[1])-rhs[13]*(bwc_params.CRH_3[1])-rhs[14]*(bwc_params.CRH_4[1])-rhs[15]*(bwc_params.CRH_5[1])-rhs[16]*(bwc_params.CRH_6[1])-rhs[17]*(bwc_params.CRH_7[1])-rhs[18]*(bwc_params.CRH_8[1])-rhs[19]*(bwc_params.CRH_9[1])-rhs[20]*(bwc_params.CRH_10[1])-rhs[21]*(bwc_params.CRH_11[1]);
  lhs[8] = -rhs[11]*(bwc_params.CRH_1[2])-rhs[12]*(bwc_params.CRH_2[2])-rhs[13]*(bwc_params.CRH_3[2])-rhs[14]*(bwc_params.CRH_4[2])-rhs[15]*(bwc_params.CRH_5[2])-rhs[16]*(bwc_params.CRH_6[2])-rhs[17]*(bwc_params.CRH_7[2])-rhs[18]*(bwc_params.CRH_8[2])-rhs[19]*(bwc_params.CRH_9[2])-rhs[20]*(bwc_params.CRH_10[2])-rhs[21]*(bwc_params.CRH_11[2]);
  lhs[9] = -rhs[11]*(bwc_params.CRH_1[3])-rhs[12]*(bwc_params.CRH_2[3])-rhs[13]*(bwc_params.CRH_3[3])-rhs[14]*(bwc_params.CRH_4[3])-rhs[15]*(bwc_params.CRH_5[3])-rhs[16]*(bwc_params.CRH_6[3])-rhs[17]*(bwc_params.CRH_7[3])-rhs[18]*(bwc_params.CRH_8[3])-rhs[19]*(bwc_params.CRH_9[3])-rhs[20]*(bwc_params.CRH_10[3])-rhs[21]*(bwc_params.CRH_11[3]);
  lhs[10] = -rhs[11]*(bwc_params.CRH_1[4])-rhs[12]*(bwc_params.CRH_2[4])-rhs[13]*(bwc_params.CRH_3[4])-rhs[14]*(bwc_params.CRH_4[4])-rhs[15]*(bwc_params.CRH_5[4])-rhs[16]*(bwc_params.CRH_6[4])-rhs[17]*(bwc_params.CRH_7[4])-rhs[18]*(bwc_params.CRH_8[4])-rhs[19]*(bwc_params.CRH_9[4])-rhs[20]*(bwc_params.CRH_10[4])-rhs[21]*(bwc_params.CRH_11[4]);
  lhs[11] = -rhs[11]*(bwc_params.CRH_1[5])-rhs[12]*(bwc_params.CRH_2[5])-rhs[13]*(bwc_params.CRH_3[5])-rhs[14]*(bwc_params.CRH_4[5])-rhs[15]*(bwc_params.CRH_5[5])-rhs[16]*(bwc_params.CRH_6[5])-rhs[17]*(bwc_params.CRH_7[5])-rhs[18]*(bwc_params.CRH_8[5])-rhs[19]*(bwc_params.CRH_9[5])-rhs[20]*(bwc_params.CRH_10[5])-rhs[21]*(bwc_params.CRH_11[5]);
  lhs[12] = 0;
  lhs[13] = 0;
  lhs[14] = 0;
  lhs[15] = 0;
  lhs[16] = 0;
  lhs[17] = 0;
}
void bwc_multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*bwc_params.QFh[0]);
  lhs[1] = rhs[1]*(2*bwc_params.QFh[1]);
  lhs[2] = rhs[2]*(2*bwc_params.QFh[2]);
  lhs[3] = rhs[3]*(2*bwc_params.QFh[3]);
  lhs[4] = rhs[4]*(2*bwc_params.QFh[4]);
  lhs[5] = rhs[5]*(2*bwc_params.QFh[5]);
  lhs[6] = rhs[6]*(2*bwc_params.QFh[6]);
  lhs[7] = rhs[7]*(2*bwc_params.QFh[7]);
  lhs[8] = rhs[8]*(2*bwc_params.QFh[8]);
  lhs[9] = rhs[9]*(2*bwc_params.QFh[9]);
  lhs[10] = rhs[10]*(2*bwc_params.QFh[10]);
  lhs[11] = rhs[11]*(2*bwc_params.QFh[11]);
  lhs[12] = rhs[12]*(2*bwc_params.Qw[0]);
  lhs[13] = rhs[13]*(2*bwc_params.Qw[1]);
  lhs[14] = rhs[14]*(2*bwc_params.Qw[2]);
  lhs[15] = rhs[15]*(2*bwc_params.Qw[3]);
  lhs[16] = rhs[16]*(2*bwc_params.Qw[4]);
  lhs[17] = rhs[17]*(2*bwc_params.Qw[5]);
}
void bwc_fillq(void) {
  bwc_work.q[0] = -2*bwc_params.QFh[0]*bwc_params.pFh[0];
  bwc_work.q[1] = -2*bwc_params.QFh[1]*bwc_params.pFh[1];
  bwc_work.q[2] = -2*bwc_params.QFh[2]*bwc_params.pFh[2];
  bwc_work.q[3] = -2*bwc_params.QFh[3]*bwc_params.pFh[3];
  bwc_work.q[4] = -2*bwc_params.QFh[4]*bwc_params.pFh[4];
  bwc_work.q[5] = -2*bwc_params.QFh[5]*bwc_params.pFh[5];
  bwc_work.q[6] = -2*bwc_params.QFh[6]*bwc_params.pFh[6];
  bwc_work.q[7] = -2*bwc_params.QFh[7]*bwc_params.pFh[7];
  bwc_work.q[8] = -2*bwc_params.QFh[8]*bwc_params.pFh[8];
  bwc_work.q[9] = -2*bwc_params.QFh[9]*bwc_params.pFh[9];
  bwc_work.q[10] = -2*bwc_params.QFh[10]*bwc_params.pFh[10];
  bwc_work.q[11] = -2*bwc_params.QFh[11]*bwc_params.pFh[11];
  bwc_work.q[12] = 0;
  bwc_work.q[13] = 0;
  bwc_work.q[14] = 0;
  bwc_work.q[15] = 0;
  bwc_work.q[16] = 0;
  bwc_work.q[17] = 0;
}
void bwc_fillh(void) {
  bwc_work.h[0] = 0;
  bwc_work.h[1] = 0;
  bwc_work.h[2] = 0;
  bwc_work.h[3] = 0;
  bwc_work.h[4] = 0;
  bwc_work.h[5] = 0;
  bwc_work.h[6] = 0;
  bwc_work.h[7] = 0;
  bwc_work.h[8] = 0;
  bwc_work.h[9] = 0;
  bwc_work.h[10] = 0;
  bwc_work.h[11] = 0;
  bwc_work.h[12] = 0;
  bwc_work.h[13] = 0;
  bwc_work.h[14] = 0;
  bwc_work.h[15] = 0;
  bwc_work.h[16] = 0;
  bwc_work.h[17] = 0;
  bwc_work.h[18] = 0;
  bwc_work.h[19] = 0;
  bwc_work.h[20] = 0;
  bwc_work.h[21] = 0;
}
void bwc_fillb(void) {
  bwc_work.b[0] = bwc_params.beta[0]*bwc_params.b1[0];
  bwc_work.b[1] = bwc_params.beta[0]*bwc_params.b1[1];
  bwc_work.b[2] = bwc_params.beta[0]*bwc_params.b1[2];
  bwc_work.b[3] = bwc_params.beta[0]*bwc_params.b1[3];
  bwc_work.b[4] = bwc_params.beta[0]*bwc_params.b1[4];
  bwc_work.b[5] = bwc_params.beta[0]*bwc_params.b1[5];
  bwc_work.b[6] = 0;
  bwc_work.b[7] = 0;
}
void bwc_pre_ops(void) {
  bwc_work.quad_618956689408[0] = bwc_params.pFh[0]*bwc_params.QFh[0]*bwc_params.pFh[0]+bwc_params.pFh[1]*bwc_params.QFh[1]*bwc_params.pFh[1]+bwc_params.pFh[2]*bwc_params.QFh[2]*bwc_params.pFh[2]+bwc_params.pFh[3]*bwc_params.QFh[3]*bwc_params.pFh[3]+bwc_params.pFh[4]*bwc_params.QFh[4]*bwc_params.pFh[4]+bwc_params.pFh[5]*bwc_params.QFh[5]*bwc_params.pFh[5]+bwc_params.pFh[6]*bwc_params.QFh[6]*bwc_params.pFh[6]+bwc_params.pFh[7]*bwc_params.QFh[7]*bwc_params.pFh[7]+bwc_params.pFh[8]*bwc_params.QFh[8]*bwc_params.pFh[8]+bwc_params.pFh[9]*bwc_params.QFh[9]*bwc_params.pFh[9]+bwc_params.pFh[10]*bwc_params.QFh[10]*bwc_params.pFh[10]+bwc_params.pFh[11]*bwc_params.QFh[11]*bwc_params.pFh[11];
}
