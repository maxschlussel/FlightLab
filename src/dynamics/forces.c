#include "src/dynamics/aerodynamics.h"
#include "src/dynamics/forces.h"
#include "src/dynamics/gravity.h"
#include "src/dynamics/propulsion.h"

#include "src/io/logger.h"


/**
 * @brief Compute the total forces and moments acting on the aircraft.
 *
 * This function evaluates the forces and moments given the current aircraft
 * state, control inputs, and aircraft parameters. It outputs the resulting total force
 * and moment vectors expressed in the body frame.
 *
 * @param[in]  X         Pointer to the aircraft state vector.
 * @param[in]  U_cmd         Pointer to the control vector.
 * @param[in]  acParams  Pointer to the aircraft parameter struct.
 * @param[out] F_tot     Pointer to the total force vector in the body frame.
 * @param[out] M_tot     Pointer to the total moment vector in the body frame.
 */
void computeForcesAndMoments(StateVector* X, ControlVector* U_cmd, AircraftParams* acParams,
    Vector3* F_tot, Vector3* M_tot){
    
    Vector3 F_aero = {0.0}, M_aero = {0.0};
    Vector3 F_prop = {0.0}, M_prop = {0.0};
    Vector3 F_grav = {0.0};
    
    computeAerodynamicForces(X, U_cmd, acParams, &F_aero, &M_aero);

    computePropulsionForces(X, U_cmd, acParams, &F_prop, &M_prop);
    
    computeGravityForces(X, acParams, &F_grav);
    
    // Sum forces
    *F_tot = vec3_add(F_aero, vec3_add(F_prop, F_grav) );
    *M_tot = vec3_add(M_aero, M_prop);

    logger.data[LOG_FORCES_F_TOT_X] = F_tot->x;
    logger.data[LOG_FORCES_F_TOT_Y] = F_tot->y;
    logger.data[LOG_FORCES_F_TOT_Z] = F_tot->z;
    logger.data[LOG_MOMENTS_M_TOT_X] = M_tot->x;
    logger.data[LOG_MOMENTS_M_TOT_Y] = M_tot->y;
    logger.data[LOG_MOMENTS_M_TOT_Z] = M_tot->z;
}