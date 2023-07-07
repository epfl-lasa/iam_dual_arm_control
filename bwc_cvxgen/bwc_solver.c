/* Produced by CVXGEN, 2019-04-06 15:00:01 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "bwc_solver.h"
double bwc_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 22; i++)
    gap += bwc_work.z[i]*bwc_work.s[i];
  return gap;
}
void bwc_set_defaults(void) {
  bwc_settings.resid_tol = 1e-6;
  bwc_settings.eps = 1e-4;
  bwc_settings.max_iters = 25;
  bwc_settings.refine_steps = 1;
  bwc_settings.s_init = 1;
  bwc_settings.z_init = 1;
  bwc_settings.debug = 0;
  bwc_settings.verbose = 1;
  bwc_settings.verbose_refinement = 0;
  bwc_settings.better_start = 1;
  bwc_settings.kkt_reg = 1e-7;
}
void bwc_setup_pointers(void) {
  bwc_work.y = bwc_work.x + 18;
  bwc_work.s = bwc_work.x + 26;
  bwc_work.z = bwc_work.x + 48;
  bwc_vars.Fh = bwc_work.x + 0;
  bwc_vars.w = bwc_work.x + 12;
}
void setup_indexed_bwc_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  bwc_params.Gh[4] = bwc_params.Gh_4;
  bwc_params.Gh[5] = bwc_params.Gh_5;
  bwc_params.Gh[6] = bwc_params.Gh_6;
  bwc_params.CLH[1] = bwc_params.CLH_1;
  bwc_params.CLH[2] = bwc_params.CLH_2;
  bwc_params.CLH[3] = bwc_params.CLH_3;
  bwc_params.CLH[4] = bwc_params.CLH_4;
  bwc_params.CLH[5] = bwc_params.CLH_5;
  bwc_params.CLH[6] = bwc_params.CLH_6;
  bwc_params.CLH[7] = bwc_params.CLH_7;
  bwc_params.CLH[8] = bwc_params.CLH_8;
  bwc_params.CLH[9] = bwc_params.CLH_9;
  bwc_params.CLH[10] = bwc_params.CLH_10;
  bwc_params.CLH[11] = bwc_params.CLH_11;
  bwc_params.CRH[1] = bwc_params.CRH_1;
  bwc_params.CRH[2] = bwc_params.CRH_2;
  bwc_params.CRH[3] = bwc_params.CRH_3;
  bwc_params.CRH[4] = bwc_params.CRH_4;
  bwc_params.CRH[5] = bwc_params.CRH_5;
  bwc_params.CRH[6] = bwc_params.CRH_6;
  bwc_params.CRH[7] = bwc_params.CRH_7;
  bwc_params.CRH[8] = bwc_params.CRH_8;
  bwc_params.CRH[9] = bwc_params.CRH_9;
  bwc_params.CRH[10] = bwc_params.CRH_10;
  bwc_params.CRH[11] = bwc_params.CRH_11;
}
void bwc_setup_indexing(void) {
  bwc_setup_pointers();
  setup_indexed_bwc_params();
}
void bwc_set_start(void) {
  int i;
  for (i = 0; i < 18; i++)
    bwc_work.x[i] = 0;
  for (i = 0; i < 8; i++)
    bwc_work.y[i] = 0;
  for (i = 0; i < 22; i++)
    bwc_work.s[i] = (bwc_work.h[i] > 0) ? bwc_work.h[i] : bwc_settings.s_init;
  for (i = 0; i < 22; i++)
    bwc_work.z[i] = bwc_settings.z_init;
}
double bwc_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in bwc_work.rhs. */
  bwc_multbyP(bwc_work.rhs, bwc_work.x);
  objv = 0;
  for (i = 0; i < 18; i++)
    objv += bwc_work.x[i]*bwc_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 18; i++)
    objv += bwc_work.q[i]*bwc_work.x[i];
  objv += bwc_work.quad_618956689408[0];
  return objv;
}
void bwc_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = bwc_work.rhs;
  r2 = bwc_work.rhs + 18;
  r3 = bwc_work.rhs + 40;
  r4 = bwc_work.rhs + 62;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  bwc_multbymAT(r1, bwc_work.y);
  bwc_multbymGT(bwc_work.buffer, bwc_work.z);
  for (i = 0; i < 18; i++)
    r1[i] += bwc_work.buffer[i];
  bwc_multbyP(bwc_work.buffer, bwc_work.x);
  for (i = 0; i < 18; i++)
    r1[i] -= bwc_work.buffer[i] + bwc_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 22; i++)
    r2[i] = -bwc_work.z[i];
  /* r3 = -Gx - s + h. */
  bwc_multbymG(r3, bwc_work.x);
  for (i = 0; i < 22; i++)
    r3[i] += -bwc_work.s[i] + bwc_work.h[i];
  /* r4 = -Ax + b. */
  bwc_multbymA(r4, bwc_work.x);
  for (i = 0; i < 8; i++)
    r4[i] += bwc_work.b[i];
}
void bwc_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = bwc_work.rhs + 18;
  ds_aff = bwc_work.lhs_aff + 18;
  dz_aff = bwc_work.lhs_aff + 40;
  mu = 0;
  for (i = 0; i < 22; i++)
    mu += bwc_work.s[i]*bwc_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 22; i++)
    if (ds_aff[i] < minval*bwc_work.s[i])
      minval = ds_aff[i]/bwc_work.s[i];
  for (i = 0; i < 22; i++)
    if (dz_aff[i] < minval*bwc_work.z[i])
      minval = dz_aff[i]/bwc_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 22; i++)
    sigma += (bwc_work.s[i] + alpha*ds_aff[i])*
      (bwc_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.045454545454545456;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 18; i++)
    bwc_work.rhs[i] = 0;
  for (i = 40; i < 70; i++)
    bwc_work.rhs[i] = 0;
  for (i = 0; i < 22; i++)
    r2[i] = bwc_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void bwc_refine(double *target, double *var) {
  int i, j;
  double *residual = bwc_work.buffer;
  double norm2;
  double *new_var = bwc_work.buffer2;
  for (j = 0; j < bwc_settings.refine_steps; j++) {
    norm2 = 0;
    bwc_matrix_multiply(residual, var);
    for (i = 0; i < 70; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (bwc_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    bwc_ldl_bwc_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 70; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (bwc_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    bwc_matrix_multiply(residual, var);
    for (i = 0; i < 70; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double bwc_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  bwc_multbymG(bwc_work.buffer, bwc_work.x);
  /* Add -s + h. */
  for (i = 0; i < 22; i++)
    bwc_work.buffer[i] += -bwc_work.s[i] + bwc_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 22; i++)
    norm2_squared += bwc_work.buffer[i]*bwc_work.buffer[i];
  return norm2_squared;
}
double bwc_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  bwc_multbymA(bwc_work.buffer, bwc_work.x);
  /* Add +b. */
  for (i = 0; i < 8; i++)
    bwc_work.buffer[i] += bwc_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 8; i++)
    norm2_squared += bwc_work.buffer[i]*bwc_work.buffer[i];
  return norm2_squared;
}
void bwc_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  bwc_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 22; i++)
    bwc_work.s_inv_z[i] = 1;
  bwc_fill_KKT();
  bwc_ldl_factor();
  bwc_fillrhs_start();
  /* Borrow bwc_work.lhs_aff for the solution. */
  bwc_ldl_bwc_solve(bwc_work.rhs, bwc_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = bwc_work.lhs_aff;
  s = bwc_work.lhs_aff + 18;
  z = bwc_work.lhs_aff + 40;
  y = bwc_work.lhs_aff + 62;
  /* Just set x and y as is. */
  for (i = 0; i < 18; i++)
    bwc_work.x[i] = x[i];
  for (i = 0; i < 8; i++)
    bwc_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 22; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 22; i++)
      bwc_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 22; i++)
      bwc_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 22; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 22; i++)
      bwc_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 22; i++)
      bwc_work.z[i] = z[i] + alpha;
  }
}
void bwc_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = bwc_work.rhs;
  r2 = bwc_work.rhs + 18;
  r3 = bwc_work.rhs + 40;
  r4 = bwc_work.rhs + 62;
  for (i = 0; i < 18; i++)
    r1[i] = -bwc_work.q[i];
  for (i = 0; i < 22; i++)
    r2[i] = 0;
  for (i = 0; i < 22; i++)
    r3[i] = bwc_work.h[i];
  for (i = 0; i < 8; i++)
    r4[i] = bwc_work.b[i];
}
long bwc_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  bwc_work.converged = 0;
  bwc_setup_pointers();
  bwc_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (bwc_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  bwc_fillq();
  bwc_fillh();
  bwc_fillb();
  if (bwc_settings.better_start)
    bwc_better_start();
  else
    bwc_set_start();
  for (iter = 0; iter < bwc_settings.max_iters; iter++) {
    for (i = 0; i < 22; i++) {
      bwc_work.s_inv[i] = 1.0 / bwc_work.s[i];
      bwc_work.s_inv_z[i] = bwc_work.s_inv[i]*bwc_work.z[i];
    }
    bwc_work.block_33[0] = 0;
    bwc_fill_KKT();
    bwc_ldl_factor();
    /* Affine scaling directions. */
    bwc_fillrhs_aff();
    bwc_ldl_bwc_solve(bwc_work.rhs, bwc_work.lhs_aff);
    bwc_refine(bwc_work.rhs, bwc_work.lhs_aff);
    /* Centering plus corrector directions. */
    bwc_fillrhs_cc();
    bwc_ldl_bwc_solve(bwc_work.rhs, bwc_work.lhs_cc);
    bwc_refine(bwc_work.rhs, bwc_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 70; i++)
      bwc_work.lhs_aff[i] += bwc_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = bwc_work.lhs_aff;
    ds = bwc_work.lhs_aff + 18;
    dz = bwc_work.lhs_aff + 40;
    dy = bwc_work.lhs_aff + 62;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 22; i++)
      if (ds[i] < minval*bwc_work.s[i])
        minval = ds[i]/bwc_work.s[i];
    for (i = 0; i < 22; i++)
      if (dz[i] < minval*bwc_work.z[i])
        minval = dz[i]/bwc_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 18; i++)
      bwc_work.x[i] += alpha*dx[i];
    for (i = 0; i < 22; i++)
      bwc_work.s[i] += alpha*ds[i];
    for (i = 0; i < 22; i++)
      bwc_work.z[i] += alpha*dz[i];
    for (i = 0; i < 8; i++)
      bwc_work.y[i] += alpha*dy[i];
    bwc_work.gap = bwc_eval_gap();
    bwc_work.eq_resid_squared = bwc_calc_eq_resid_squared();
    bwc_work.ineq_resid_squared = bwc_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (bwc_settings.verbose) {
      bwc_work.optval = bwc_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, bwc_work.optval, bwc_work.gap, sqrt(bwc_work.eq_resid_squared),
          sqrt(bwc_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (bwc_work.gap < bwc_settings.eps)
        && (bwc_work.eq_resid_squared <= bwc_settings.resid_tol*bwc_settings.resid_tol)
        && (bwc_work.ineq_resid_squared <= bwc_settings.resid_tol*bwc_settings.resid_tol)
       ) {
      bwc_work.converged = 1;
      bwc_work.optval = bwc_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
