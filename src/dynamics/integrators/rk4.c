#include <string.h> 

#include "src/dynamics/eom.h"
#include "src/dynamics/forces/forces.h"
#include "src/dynamics/integrators/euler_integrator.h"
#include "src/dynamics/integrators/rk4.h"
#include "src/math/vector.h"


/**
 * @brief Integrate one simulation step using the classical 4th-order Runge–Kutta (RK4) method.
 *
 * This function advances the system state `X` by one timestep `dt_s` using RK4 integration.
 * It provides significantly better accuracy and stability than a first-order Euler integrator,
 * especially for nonlinear flight dynamics where forces and moments depend strongly on state.
 *
 * ## RK4 Algorithm
 * The RK4 method computes four derivative estimates (k1–k4) at intermediate states:
 *
 *   k1 = f( X_n,                t_n )
 *   k2 = f( X_n + (dt/2)*k1,    t_n + dt/2 )
 *   k3 = f( X_n + (dt/2)*k2,    t_n + dt/2 )
 *   k4 = f( X_n + dt*k3,        t_n + dt )
 *
 * Where X_n is the current state, dt is the timestep (h), and f(X,t) = dX/dt . In the case of a
 * 6DOF simulation, the derivative typically is independent of time: f(X) = dX/dt. Then it combines 
 * the intermediate states into a weighted average:
 *
 *   X_{n+1} = X_n + (dt/6) * ( k1 + 2*k2 + 2*k3 + k4 )
 *
 *
 * ## Implementation Notes
 * - `Xdot` is passed in for readibility in main loop (already computed derivative at current state).
 * - Forces and moments are recomputed for each intermediate state (X2, X3, X4). This ensures 
 *   nonlinear effects are captured correctly.
 * - Controls `U_cmd` are assumed constant over the timestep (`dt_s`).
 *
 * ## Parameters
 * @param[in,out] X        Pointer to current aircraft state vector (updated in-place).
 * @param[in]     U_cmd    Pointer to current control vector.
 * @param[in]     acModel Pointer to aircraft parameters struct.
 * @param[in]     Xdot     Current state derivative at f(X, t).
 * @param[in]     dt_s     Simulation timestep [s].
 */

void integrateRK4Step(StateVector* X, const Actuators* actuators, const AircraftModel* acModel, 
                      const double Xdot[12], double dt_s){

    AeroData aeroData = {0.0};

    // [1] Compute k1 = f( X_n )
    double k1[12];
    memcpy(k1, Xdot, sizeof(k1));
    
    // [2] Compute k2 = f( X_n + (dt/2)*k1 )
    StateVector X2 = *X;
    double k2[12];
    integrateEulerStep(&X2, k1, dt_s/2);
    computeForcesAndMoments(&X2, actuators, acModel, &aeroData);
    computeStateDerivative(&X2, acModel, &aeroData.F_tot, &aeroData.M_tot, k2);

    // [3] Compute k3 = f( X_n + (dt/2)*k2 )
    StateVector X3 = *X;
    double k3[12];
    integrateEulerStep(&X3, k2, dt_s/2);
    computeForcesAndMoments(&X3, actuators, acModel, &aeroData);
    computeStateDerivative(&X3, acModel, &aeroData.F_tot, &aeroData.M_tot, k3);

    // [4] Compute k4 = f( X_n + dt*k3 )
    StateVector X4 = *X;    // intermediate state y_n + h * k3
    double k4[12];          // derivative of state = f(X4)
    integrateEulerStep(&X4, k3, dt_s);
    computeForcesAndMoments(&X4, actuators, acModel, &aeroData);
    computeStateDerivative(&X4, acModel, &aeroData.F_tot, &aeroData.M_tot, k4);

    // X += (dt_s / 6) * (k1 + 2*k2 + 2*k3 + k4);
    double k_tot[12];
    for(int i = 0; i < 12; i++) k_tot[i] = k1[i] + 2*k2[i] + 2*k3[i] + k4[i];
    integrateEulerStep(X, k_tot, dt_s/6);
}