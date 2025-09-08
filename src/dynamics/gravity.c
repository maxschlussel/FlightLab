#include "src/core/constants.h"
#include "src/dynamics/gravity.h"
#include "src/math/dcm.h"

#include "src/io/logger.h"


/**
 * @brief Compute the gravitational force acting on the aircraft in the body frame.
 *
 * This function calculates the gravity force vector in the body axes of the aircraft
 * given its current attitude and mass. .
 *
 * @param[in]  X          Pointer to the current state vector of the aircraft, containing
 *                        Euler angles (phi, theta, psi) required to rotate gravity into
 *                        the body frame.
 * @param[in]  ac_params  Pointer to the aircraft parameters structure, providing mass info.
 * @param[out] F          Pointer to a Vector3 structure where the computed gravity
 *                        force vector (in Newtons) will be stored.
 *
 * @note
 * - Assumes a constant gravitational acceleration `g = 9.80665 m/s²`.
 *
 * @see computeAerodynamicForces(), computeThrustForces(), computeStateDerivative()
 */
void computeGravityForces(StateVector* X, AircraftParams* ac_params, Vector3* F){
    Vector3 Fgrav_inertial = {0, 0, ac_params->mass * g};
    
    // Earth to body rotation matrix
    double R[3][3];
    getRotationMatrix(X->psi, X->theta, X->phi, R);

    *F = mat3_mult_vec3(R, Fgrav_inertial);

    logger.data[LOG_FORCES_F_GRAV_X] = F->x;
    logger.data[LOG_FORCES_F_GRAV_Y] = F->y;
    logger.data[LOG_FORCES_F_GRAV_Z] = F->z;
}
