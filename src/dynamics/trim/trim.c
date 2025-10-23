#include <stdio.h>
#include <stdlib.h>

#include "src/aircrafts/boeing_737.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/dynamics/aerodynamics.h"
#include "src/dynamics/eom.h"
#include "src/dynamics/forces.h"
#include "src/dynamics/trim/fmin_search.h"
#include "src/dynamics/trim/trim.h"


// Temporary helper funcs
double calculate_quadratic_form_trim(const double q[N_TRIM_STATES], const double H[N_TRIM_STATES][N_TRIM_STATES]);

void mat_identity_trim(double A[N_TRIM_STATES][N_TRIM_STATES]);


/**
 * @brief Solves for the aircraft trim state using the Nelder-Mead simplex method.
 *
 * @param[in] Z0        Initial guess vector for the trim state.
 * @param[in] trimRefs  Pointer to trim references struct describing the desired trim condition 
*                       (e.g., straight flight, coordinated turn).
 * @param[in] trimOpts  Pointer to the trim solving options structure defining post-solve actions 
 *                      (e.g., printing results, saving to file).
 * @param[out] Z_return Pre-allocated array where the final optimal trim state vector is stored (output).
 */
void solveTrim(double* Z0, TrimRefs* trimRefs, TrimSolveOptions* trimOpts, double* Z_return) {
    // [1] Setup cost function and params
    CostFunction costFunc = getCostFunction(trimRefs);
    // CostFuncParams costFuncParams = {.acParams = acParams};
    
    // [2] Setup solver options and result
    NMOptions nmOpts;
    nm_default_options(&nmOpts);

    nmOpts.TolX = 1e-10;
    nmOpts.TolFun = 1e-10;
    nmOpts.MaxIter = 50000;
    nmOpts.MaxEval = 50000;
    nmOpts.verbose = 0;
    
    double* result_x = (double*)malloc(N_TRIM_STATES * sizeof(double));
    NMResult results = { .x_opt = result_x};

    // [3] Perform trim solve
    fminsearch(costFunc, Z0, N_TRIM_STATES, &nmOpts, &results);

    // [4] Save/ display result
    for (int i = 0; i < N_TRIM_STATES; i++) {
        Z_return[i] = results.x_opt[i];
    }

    if (trimOpts->saveToFile) {
        // Save to file
    }

    if (trimOpts->printToScreen) {
        printf("Status: %d (0=converged). Iter %d, func evals %d\n", results.status, results.iterations, results.feval_count);
        printf("f* = %g\n", results.f_opt);
        printf("Minimum found at:\n");
        for (int i = 0; i < N_TRIM_STATES; i++){
            printf("x[%d]: %f\n", i, Z_return[i]);
        }
    }

    // [5] Clean up/ free memory
    free(result_x); 
    result_x = NULL;
    results.x_opt = NULL;
}


void mat_identity_trim(double A[N_TRIM_STATES][N_TRIM_STATES]) {
    for (int i = 0; i < N_TRIM_STATES; i++) {
        for (int j = 0; j < N_TRIM_STATES; j++) {
            A[i][j] = (i == j) ? 1.0 : 0.0; 
        }
    }
}


double calculate_quadratic_form_trim(const double q[N_TRIM_STATES], const double H[N_TRIM_STATES][N_TRIM_STATES]) {
    double total_result = 0.0;

    // Sum over i (q[i] * (H*q)[i])
    for (int i = 0; i < N_TRIM_STATES; i++) {
        double intermediate_component = 0.0; // This stores the i-th component of (H * q)

        // Calculate the i-th component of the H * q vector: sum over j (H[i][j] * q[j])
        for (int j = 0; j < N_TRIM_STATES; j++) {
            intermediate_component += H[i][j] * q[j]; 
        }

        // Add the term q[i] * (H*q)[i] to the total result
        total_result += q[i] * intermediate_component;
    }
    
    return total_result;
}


CostFunction getCostFunction(const TrimRefs* trimRefs) { //handleTrimMode
    // Module in development...
    switch (trimRefs->trimMode) {

        case TRIM_STRAIGHT_LEVEL:
            return costStraightAndLevel;
            
        case TRIM_CLIMB_RATE:
            break;

        case TRIM_GAMMA_ANGLE:
            break;

        case TRIM_COORD_TURN:
            break;

        case TRIM_ENGINE_OUT:
            break;

        case TRIM_CUSTOM:
            break;

        default:
            break;
    }
}


double costStraightAndLevel(const double* Z,int n){
    // Cost R
    // 1-9: Xdot[1-9] = 0
    // 10:  Va - 85 = 0
    // 11:  gamma = 0
    // 12:  v = 0
    // 13:  phi = 0
    // 14:  psi = 0

    StateVector X = {
        .u = Z[0],
        .v = Z[1],
        .w = Z[2],
        .p = Z[3],
        .q = Z[4],
        .r = Z[5],
        .phi    = Z[6],
        .theta  = Z[7],
        .psi    = Z[8],
        .x = 0,
        .y = 0,
        .z = 0
    };

    Actuators actuators = {
        .aileronServo  = { .pos = Z[9]},
        .elevatorServo = { .pos = Z[10]}, 
        .rudderServo   = { .pos = Z[11]}, 
        .throttle = {Z[12], Z[13]}
    };

    double Xdot[12] = {0.0};

    AeroData aeroData = {{0.0}};
    AircraftParams acParams = loadBoeing737AircraftParams();

    computeForcesAndMoments(&X, &actuators, &acParams, &aeroData);
    computeStateDerivative(&X, &acParams, &(aeroData.F_tot), &(aeroData.M_tot), Xdot);

    double Va = sqrt(Z[0]*Z[0] + Z[1]*Z[1] + Z[2]*Z[2]);

    double theta = Z[7];
    double alpha = atan2(Z[2], Z[0]);
    double gamma = theta - alpha;

    double R[14] = {
        Xdot[0],
        Xdot[1],
        Xdot[2],
        Xdot[3],
        Xdot[4],
        Xdot[5],
        Xdot[6],
        Xdot[7],
        Xdot[8],
        (Va - 85),
        gamma,
        Z[1],
        Z[6],
        Z[8],
    };

    double H[N_TRIM_STATES][N_TRIM_STATES];
    mat_identity_trim(H);

    // F0 = R` H R
    double F0 = calculate_quadratic_form_trim(R, H);
    return F0;
}
