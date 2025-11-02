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
 * @param[in]  U_cmd     Pointer to the control vector.
 * @param[in]  acModel  Pointer to the aircraft parameter struct.
 * @param[out] F_tot     Pointer to the total force vector in the body frame.
 * @param[out] M_tot     Pointer to the total moment vector in the body frame.
 */
void computeForcesAndMoments(const StateVector* X, const Actuators* actuators, const AircraftModel* acModel, AeroData* aeroData){
    
    computeAerodynamicForces(X, actuators, acModel, aeroData);

    computePropulsionForces(actuators, acModel, &aeroData->F_prop_net, &aeroData->M_prop_net);
    
    computeGravityForces(X, acModel, &aeroData->F_grav_net);
    
    // Sum forces
    aeroData->F_tot = vec3_add(aeroData->F_aero_net, vec3_add(aeroData->F_prop_net, aeroData->F_grav_net) );
    aeroData->M_tot = vec3_add(aeroData->M_aero_net, aeroData->M_prop_net);

    logger.data[LOG_FORCES_F_TOT_X] = aeroData->F_tot.x;
    logger.data[LOG_FORCES_F_TOT_Y] = aeroData->F_tot.y;
    logger.data[LOG_FORCES_F_TOT_Z] = aeroData->F_tot.z;
    logger.data[LOG_MOMENTS_M_TOT_X] = aeroData->M_tot.x;
    logger.data[LOG_MOMENTS_M_TOT_Y] = aeroData->M_tot.y;
    logger.data[LOG_MOMENTS_M_TOT_Z] = aeroData->M_tot.z;
}