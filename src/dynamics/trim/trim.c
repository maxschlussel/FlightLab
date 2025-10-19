#include "src/dynamics/trim/trim.h"
#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/core/control_vector.h"
#include "src/dynamics/aerodynamics.h"
#include "src/dynamics/forces.h"
#include "src/dynamics/eom.h"
#include "src/math/utils.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define N_STATES 12
#define N_CONTROLS 5
#define MAX_ITER 100
#define TOL 1e-6
#define LAMBDA_INIT 0.01


// Compute the residual (Xdot) for the trim condition
void computeTrimResidual(const StateVector *X, const ControlVector *U, const AircraftParams *acParams, AeroData *aeroData, double *residual) {
        Actuators actuators = {
        .aileronServo  = { .pos = U->da},
        .elevatorServo = { .pos = U->de}, 
        .rudderServo   = { .pos = U->dr}, 
        .throttle = {U->dthr[0], U->dthr[1]}
        };
    
    computeForcesAndMoments(X, &actuators, acParams, aeroData);
    computeStateDerivative(X, acParams, &(aeroData->F_tot), &(aeroData->M_tot), residual);

    residual[9] = 0.0; // x
    residual[10] = 0.0; // y
    residual[11] = 0.0; // z
}

void computeJacobian(const StateVector *X, const ControlVector *U, const AircraftParams *acParams, AeroData *aeroData, double *J
) {
    const double h = 1e-6;
    ControlVector U_perturbed = *U;
    double residual_plus[N_STATES], residual_minus[N_STATES];

    for (int i = 0; i < N_CONTROLS; i++) {
        // Perturb control i
        double *control_ptr = (double *)&U_perturbed + i;
        *control_ptr += h;
        computeTrimResidual(X, &U_perturbed, acParams, aeroData, residual_plus);

        *control_ptr -= 2*h;
        computeTrimResidual(X, &U_perturbed, acParams, aeroData, residual_minus);

        // Central difference
        for (int j = 0; j < N_STATES; j++) {
            J[i*N_STATES + j] = (residual_plus[j] - residual_minus[j]) / (2*h);
        }

        *control_ptr += h; // Restore
    }
}

// Solve (J^T J + lambda I) delta_U = -J^T residual
static void solveLinearSystem(
    const double *J,
    const double *residual,
    double *delta_U,
    double lambda
) {
    double JTJ[N_CONTROLS * N_CONTROLS] = {0};
    double JT_res[N_CONTROLS] = {0};

    // Compute J^T J
    for (int i = 0; i < N_CONTROLS; i++) {
        for (int j = 0; j < N_CONTROLS; j++) {
            for (int k = 0; k < N_STATES; k++) {
                JTJ[i*N_CONTROLS + j] += J[i*N_STATES + k] * J[j*N_STATES + k];
            }
        }
        JTJ[i*N_CONTROLS + i] += lambda; // Add damping
    }

    // Compute J^T residual
    for (int i = 0; i < N_CONTROLS; i++) {
        for (int k = 0; k < N_STATES; k++) {
            JT_res[i] += J[i*N_STATES + k] * residual[k];
        }
        JT_res[i] = -JT_res[i];
    }

    // Solve using Cholesky decomposition (simplified)
    // For production, replace with LAPACK or similar
    for (int i = 0; i < N_CONTROLS; i++) {
        delta_U[i] = JT_res[i] / JTJ[i*N_CONTROLS + i];
    }
}

int solveTrimLM(const StateVector *X, ControlVector *U, const AircraftParams *acParams) {
    double residual[N_STATES];
    double J[N_STATES * N_CONTROLS];
    double delta_U[N_CONTROLS];
    double lambda = LAMBDA_INIT;
    int iter = 0;

    AeroData aeroData;


    while (iter < MAX_ITER) {
        // Compute residual and Jacobian
        computeTrimResidual(X, U, acParams, &aeroData, residual);
        computeJacobian(X, U, acParams, &aeroData, J);

        // Solve for delta_U
        solveLinearSystem(J, residual, delta_U, lambda);

        // Update controls
        U->da   += delta_U[0];
        U->de   += delta_U[1];
        U->dr   += delta_U[2];
        U->dthr[0]+= delta_U[3];
        U->dthr[1]+= delta_U[4];

        // Check convergence
        double error = 0.0;
        for (int i = 0; i < N_STATES; i++) error += fabs(residual[i]);
        if (error < TOL) break;

        iter++;
    }

    return (iter < MAX_ITER);
}
