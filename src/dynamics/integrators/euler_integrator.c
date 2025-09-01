#include "src/dynamics/integrators/euler_integrator.h"

/**
 * @brief Performs a single simple Euler integration step.
 * 
 * This function updates the state vector X in-place using the Euler method:
 * \f$ X_{new} = X_{old} + \dot{X} \cdot dt \f$
 *
 * @param[in,out] X Pointer to the current state vector to be updated.
 * @param[in]     Xdot Pointer to an array of state derivatives (length 12) corresponding to
 *                the state vector elements: [u, v, w, p, q, r, phi, theta, psi, x, y, z].
 * @param[in] dt_s Integration time step in seconds.
 * 
 * @note The state vector X is modified in-place.
 * 
 * Example usage:
 * @code
 * StateVector X;
 * double Xdot[12];
 * // Compute derivatives in Xdot
 * integrateEulerStep(&X, Xdot, 0.01);
 * @endcode
 */
void integrateEulerStep(StateVector* X, const double* Xdot, double dt_s){
        X->u += dt_s * Xdot[0];
        X->v += dt_s * Xdot[1];
        X->w += dt_s * Xdot[2];
        X->p += dt_s * Xdot[3];
        X->q += dt_s * Xdot[4];  
        X->r += dt_s * Xdot[5];
        X->phi   += dt_s * Xdot[6];
        X->theta += dt_s * Xdot[7];
        X->psi   += dt_s * Xdot[8];
        X->x     += dt_s * Xdot[9];
        X->y     += dt_s * Xdot[10];
        X->z     += dt_s * Xdot[11];
}