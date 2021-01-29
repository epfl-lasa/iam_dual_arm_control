/* Produced by CVXGEN, 2019-04-06 15:00:01 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: bwc_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef bwc_SOLVER_H
#define bwc_SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (bwc_testsolver.c, csolve.c or your own */
/* program) for the global variables bwc_vars, bwc_params, bwc_work and bwc_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define bwc_pm(A, m, n) bwc_printmatrix(#A, A, m, n, 1)
#endif
typedef struct bwc_Params_t {
  double QFh[12];
  double pFh[12];
  double Qw[6];
  double beta[1];
  double b1[6];
  double Gh_4[12];
  double Gh_5[12];
  double Gh_6[12];
  double Cplh[6];
  double Cprh[6];
  double CLH_1[6];
  double CLH_2[6];
  double CLH_3[6];
  double CLH_4[6];
  double CLH_5[6];
  double CLH_6[6];
  double CLH_7[6];
  double CLH_8[6];
  double CLH_9[6];
  double CLH_10[6];
  double CLH_11[6];
  double CRH_1[6];
  double CRH_2[6];
  double CRH_3[6];
  double CRH_4[6];
  double CRH_5[6];
  double CRH_6[6];
  double CRH_7[6];
  double CRH_8[6];
  double CRH_9[6];
  double CRH_10[6];
  double CRH_11[6];
  double *Gh[7];
  double *CLH[12];
  double *CRH[12];
} bwc_Params;
typedef struct bwc_Vars_t {
  double *Fh; /* 12 rows. */
  double *w; /* 6 rows. */
} bwc_Vars;
typedef struct bwc_Workspace_t {
  double h[22];
  double s_inv[22];
  double s_inv_z[22];
  double b[8];
  double q[18];
  double rhs[70];
  double x[70];
  double *s;
  double *z;
  double *y;
  double lhs_aff[70];
  double lhs_cc[70];
  double buffer[70];
  double buffer2[70];
  double KKT[264];
  double L[247];
  double d[70];
  double v[70];
  double d_inv[70];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_618956689408[1];
  int converged;
} bwc_Workspace;
typedef struct bwc_Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} bwc_Settings;
extern bwc_Vars bwc_vars;
extern bwc_Params bwc_params;
extern bwc_Workspace bwc_work;
extern bwc_Settings bwc_settings;
/* Function definitions in ldl.c: */
void bwc_ldl_bwc_solve(double *target, double *var);
void bwc_ldl_factor(void);
double bwc_check_factorization(void);
void bwc_matrix_multiply(double *result, double *source);
double bwc_check_residual(double *target, double *multiplicand);
void bwc_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void bwc_multbymA(double *lhs, double *rhs);
void bwc_multbymAT(double *lhs, double *rhs);
void bwc_multbymG(double *lhs, double *rhs);
void bwc_multbymGT(double *lhs, double *rhs);
void bwc_multbyP(double *lhs, double *rhs);
void bwc_fillq(void);
void bwc_fillh(void);
void bwc_fillb(void);
void bwc_pre_ops(void);

/* Function definitions in solver.c: */
double bwc_eval_gap(void);
void bwc_set_defaults(void);
void bwc_setup_pointers(void);
void setup_indexed_bwc_params(void);
void bwc_setup_indexing(void);
void bwc_set_start(void);
double bwc_eval_objv(void);
void bwc_fillrhs_aff(void);
void bwc_fillrhs_cc(void);
void bwc_refine(double *target, double *var);
double bwc_calc_ineq_resid_squared(void);
double bwc_calc_eq_resid_squared(void);
void bwc_better_start(void);
void bwc_fillrhs_start(void);
long bwc_solve(void);

/* Function definitions in bwc_testsolver.c: */
int bwc_main(int argc, char **argv);
void bwc_load_default_data(void);

/* Function definitions in util.c: */
void bwc_tic(void);
float bwc_toc(void);
float bwc_tocq(void);
void bwc_printmatrix(char *name, double *A, int m, int n, int sparse);
double bwc_unif(double lower, double upper);
float bwc_ran1(long*idum, int reset);
float bwc_randn_internal(long *idum, int reset);
double bwc_randn(void);
void bwc_reset_rand(void);

#endif
