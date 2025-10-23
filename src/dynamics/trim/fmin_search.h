#pragma once


typedef enum {
    FMIN_STATUS_SUCCESS,        // Converged by tolerances
    FMIN_STATUS_MAX_ITER,       // Max iterations reached
    FMIN_STATUS_MAX_FUNC_EVAL,  // Max function evaluations reached
    FMIN_STATUS_FAILURE,        // Failed to converge: NaN/Inf or invalid input
}FminSearchStatus;


typedef struct {
    double TolX;        // termination tolerance on x (like Matlab TolX)
    double TolFun;      // termination tolerance on function values (like Matlab TolFun)
    int MaxIter;
    int MaxEval;
    double InitialStep; // relative initial simplex step (if <=0: uses default 0.05)
    double rho;         // reflection coefficient (alpha) default 1.0
    double chi;         // expansion coefficient (gamma) default 2.0
    double psi;         // contraction coefficient (beta) default 0.5
    double sigma;       // shrink coefficient default 0.5
    int verbose;        // 0 quiet, 1 progress
} NMOptions;


typedef struct {
    double* x_opt;  // Optimal result vector
    double f_opt;   // Scalar minimum function value f(x_opt)
    int iterations; // Number of iterations performed
    int feval_count;    // Number of function evaluations performed
    FminSearchStatus status;
} NMResult;


typedef double (*objfun_t)(const double* x, int n);

void nm_default_options(NMOptions* o);

void fminsearch(objfun_t func, const double *x0, int n, NMOptions* opts_in, NMResult* results);