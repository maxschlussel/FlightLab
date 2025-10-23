#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>

#include "src/dynamics/trim/fmin_search.h"


/* Utility: safe allocation */
static void *xmalloc(size_t s){
    void *p = malloc(s);
    if(!p){
        fprintf(stderr,"Out of memory\n");
        exit(1);
    }
    return p;
}

/* Compare function value safe (handles NaN/Inf by treating them as very large) */
static int is_bad_double(double v){
    return !(v <= DBL_MAX && v >= -DBL_MAX); /* catches NaN and Inf */
}

/* Norm of difference x-y (infinity-norm) */
static double inf_norm_diff(const double* x, const double* y, int n){
    double m = 0.0;
    for(int i=0;i<n;i++){
        double d = fabs(x[i]-y[i]);
        if(d > m) m = d;
    }
    return m;
}

/* Initialize solver options to default */
void nm_default_options(NMOptions* o){
    if(!o) return;
    o->TolX = 1e-6;
    o->TolFun = 1e-6;
    o->MaxIter = 2000;
    o->MaxEval = 2000;
    o->InitialStep = -1.0; // triggers default
    o->rho = 1.0;
    o->chi = 2.0;
    o->psi = 0.5;
    o->sigma = 0.5;
    o->verbose = 0;
}

/**
 * @brief Finds the minimum of a function using the Nelder-Mead (fminsearch-like) algorithm.
 *
 * This function implements the unconstrained, non-linear optimization routine 
 * commonly known as fminsearch in environments like MATLAB. It requires a 
 * starting point and an objective function (cost function) to minimize.
 *
 * @param func      Pointer to the objective function (cost function) to minimize.
 *                  Signature: double func(const double *x, int n).
 * @param x0        Initial starting vector (N-dimensional array). Used to initialize the simplex.
 * @param n         Dimension of the problem (length of x0). Must be > 0.
 * @param opts_in   Optional pointer to a structure containing optimization options 
 *                  (TolX, TolFun, MaxIter, MaxEval, etc.). Use NULL for defaults.
 * @param xout      Output vector where the final solution (optimal x) will be stored.
 * @param fout      Output scalar where the final minimum function value (f(xout)) will be stored.
 * @param iter_out  Optional pointer to an integer to store the number of iterations performed.
 * @param feval_out Optional pointer to an integer to store the number of function evaluations performed.
 * @return          FminSearchStatus indicating the termination reason (convergence, limits, or error).
 */
void fminsearch(objfun_t func, const double* x0, int n, NMOptions* opts_in, NMResult* results) {
    
    if(!func || !x0 || n <= 0 || !results) results->status = FMIN_STATUS_FAILURE;

    NMOptions opts_local;
    if(opts_in){
        opts_local = *opts_in;
    } else {
        nm_default_options(&opts_local);
    }
    /* ensure defaults for unspecified fields */
    if(opts_in == NULL) nm_default_options(&opts_local);

    const double rho = opts_local.rho;
    const double chi = opts_local.chi;
    const double psi = opts_local.psi;
    const double sigma = opts_local.sigma;
    const double TolX = opts_local.TolX;
    const double TolFun = opts_local.TolFun;
    const int MaxIter = opts_local.MaxIter;
    const int MaxEval = opts_local.MaxEval;
    const int verbose = opts_local.verbose;

    /* allocate simplex: N+1 points, each length n */
    int m = n + 1;
    double **simplex = (double**) xmalloc(m * sizeof(double*));
    for(int i=0;i<m;i++) simplex[i] = (double*) xmalloc(n * sizeof(double));

    double *fvals = (double*) xmalloc(m * sizeof(double));
    for(int i=0;i<m;i++) fvals[i] = 0.0;

    /* initialize simplex */
    double *xbase = (double*) xmalloc(n * sizeof(double));
    for(int i=0;i<n;i++) xbase[i] = x0[i];

    /* default initial step */
    double init_step = opts_local.InitialStep;
    if(init_step <= 0.0){
        /* MATLAB's default initial simplex: 0.05*(1+abs(x0_i)) */
        init_step = 0.05;
    }

    /* generate simplex vertices:
       vertex 0 = x0
       vertex i (1..n): x0 with i-th component increased by step
    */
    for(int j=0;j<n;j++) simplex[0][j] = xbase[j];
    for(int i=1;i<m;i++){
        for(int j=0;j<n;j++){
            double step = init_step * (1.0 + fabs(xbase[j]));
            simplex[i][j] = xbase[j];
        }
        /* increase (i-1)-th component */
        simplex[i][i-1] = xbase[i-1] + ((init_step > 0) ? init_step * (1.0 + fabs(xbase[i-1])) : 0.05 * (1.0 + fabs(xbase[i-1])));
    }

    int feval = 0;
    /* evaluate function at simplex points */
    for(int i=0;i<m;i++){
        double val = func(simplex[i], n);
        feval++;
        if(is_bad_double(val)){
            /* treat as large */
            fvals[i] = 1e300;
        } else {
            fvals[i] = val;
        }
    }

    int iterations = 0;
    int status = FMIN_STATUS_MAX_ITER;

    double *centroid = (double*) xmalloc(n * sizeof(double));
    double *xr = (double*) xmalloc(n * sizeof(double));
    double *xe = (double*) xmalloc(n * sizeof(double));
    double *xc = (double*) xmalloc(n * sizeof(double));
    double *xnew = (double*) xmalloc(n * sizeof(double));

    /* main loop */
    while(1){
        /* sort simplex by fvals ascending (simple selection sort for clarity) */
        for(int i=0;i<m-1;i++){
            int best = i;
            for(int j=i+1;j<m;j++){
                if(fvals[j] < fvals[best]) best = j;
            }
            if(best != i){
                double *tmpv = simplex[i];
                simplex[i] = simplex[best];
                simplex[best] = tmpv;
                double tmpf = fvals[i];
                fvals[i] = fvals[best];
                fvals[best] = tmpf;
            }
        }

        /* best is simplex[0], worst is simplex[m-1] */
        if(verbose){
            printf("iter %4d fevals %4d fbest %13.8g xbest [", iterations, feval, fvals[0]);
            for(int j=0;j<n;j++){
                printf(" %g", simplex[0][j]);
            }
            printf(" ]\n");
        }

        /* termination checks: function value spread and simplex size */
        double fmean = 0.0;
        for(int i=0;i<m;i++) fmean += fvals[i];
        fmean /= m;
        double fv_std = 0.0;
        for(int i=0;i<m;i++){
            double d = fvals[i] - fmean;
            fv_std += d*d;
        }
        fv_std = sqrt(fv_std / m);

        /* max distance between vertices */
        double max_dx = 0.0;
        for(int i=1;i<m;i++){
            double d = inf_norm_diff(simplex[0], simplex[i], n);
            if(d > max_dx) max_dx = d;
        }

        if(fv_std <= TolFun && max_dx <= TolX){
            status = FMIN_STATUS_SUCCESS;
            break;
        }
        if(iterations >= MaxIter){
            status = FMIN_STATUS_MAX_ITER;
            break;
        }
        if(feval >= MaxEval){
            status = FMIN_STATUS_MAX_FUNC_EVAL;
            break;
        }

        /* compute centroid of all points except worst */
        for(int j=0;j<n;j++){
            centroid[j] = 0.0;
            for(int i=0;i<m-1;i++){
                centroid[j] += simplex[i][j];
            }
            centroid[j] /= (double)(m - 1);
        }

        /* reflection: xr = centroid + rho*(centroid - worst) = (1+rho)*centroid - rho*worst */
        for(int j=0;j<n;j++) xr[j] = centroid[j] + rho * (centroid[j] - simplex[m-1][j]);

        double fr = func(xr, n); feval++;
        if(is_bad_double(fr)) fr = 1e300;

        if(fr < fvals[0]){
            /* expansion */
            for(int j=0;j<n;j++) xe[j] = centroid[j] + chi * (xr[j] - centroid[j]);
            double fe = func(xe, n); feval++;
            if(is_bad_double(fe)) fe = 1e300;

            if(fe < fr){
                /* accept expansion */
                for(int j=0;j<n;j++) simplex[m-1][j] = xe[j];
                fvals[m-1] = fe;
            } else {
                /* accept reflection */
                for(int j=0;j<n;j++) simplex[m-1][j] = xr[j];
                fvals[m-1] = fr;
            }
        } else if(fr < fvals[m-2]){
            /* accept reflection */
            for(int j=0;j<n;j++) simplex[m-1][j] = xr[j];
            fvals[m-1] = fr;
        } else {
            /* contraction */
            if(fr < fvals[m-1]){
                /* outside contraction: x_c = centroid + psi*(xr - centroid) */
                for(int j=0;j<n;j++) xc[j] = centroid[j] + psi * (xr[j] - centroid[j]);
                double fc = func(xc, n); feval++;
                if(is_bad_double(fc)) fc = 1e300;
                if(fc <= fr){
                    for(int j=0;j<n;j++) simplex[m-1][j] = xc[j];
                    fvals[m-1] = fc;
                } else {
                    /* shrink */
                    for(int i=1;i<m;i++){
                        for(int j=0;j<n;j++){
                            simplex[i][j] = simplex[0][j] + sigma * (simplex[i][j] - simplex[0][j]);
                        }
                        fvals[i] = func(simplex[i], n); feval++;
                        if(is_bad_double(fvals[i])) fvals[i] = 1e300;
                    }
                }
            } else {
                /* inside contraction: x_c = centroid + psi*(worst - centroid) */
                for(int j=0;j<n;j++) xc[j] = centroid[j] + psi * (simplex[m-1][j] - centroid[j]);
                double fc = func(xc, n); feval++;
                if(is_bad_double(fc)) fc = 1e300;
                if(fc < fvals[m-1]){
                    for(int j=0;j<n;j++) simplex[m-1][j] = xc[j];
                    fvals[m-1] = fc;
                } else {
                    /* shrink */
                    for(int i=1;i<m;i++){
                        for(int j=0;j<n;j++){
                            simplex[i][j] = simplex[0][j] + sigma * (simplex[i][j] - simplex[0][j]);
                        }
                        fvals[i] = func(simplex[i], n); feval++;
                        if(is_bad_double(fvals[i])) fvals[i] = 1e300;
                    }
                }
            }
        }

        iterations++;
    }

    /* copy best to xout */
    for(int j=0;j<n;j++) results->x_opt[j] = simplex[0][j];
    results->f_opt = fvals[0];
    results->iterations = iterations;
    results->feval_count = feval;
    results->status = status;

    /* free */
    for(int i=0;i<m;i++) free(simplex[i]);
    free(simplex);
    free(fvals);
    free(xbase);
    free(centroid);
    free(xr); free(xe); free(xc); free(xnew);
}
