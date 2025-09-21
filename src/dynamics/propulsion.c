#include "src/dynamics/propulsion.h"

#include "src/io/logger.h"


void computePropulsionForces(StateVector* X, ControlVector* U, AircraftParams* acParams, 
        Vector3* F, Vector3* M){

    double Fthrottle1 = U->dthr[0] * acParams->mass * g;
    double Fthrottle2 = U->dthr[1] * acParams->mass * g;

    Vector3 FengineOne = {Fthrottle1, 0, 0};
    Vector3 FengineTwo = {Fthrottle2, 0, 0};

    *F = vec3_add(FengineOne, FengineTwo);

    Vector3 MomentEngineOne_b = vec3_cross(acParams->r_engineOne2cg, FengineOne);
    Vector3 MomentEngineTwo_b = vec3_cross(acParams->r_engineTwo2cg, FengineTwo);

    *M = vec3_add(MomentEngineOne_b, MomentEngineTwo_b);

    logger.data[LOG_FORCES_F_PROP_X] = F->x;
    logger.data[LOG_FORCES_F_PROP_Y] = F->y;
    logger.data[LOG_FORCES_F_PROP_Z] = F->z;
    logger.data[LOG_MOMENTS_M_PROP_X] = M->x;
    logger.data[LOG_MOMENTS_M_PROP_Y] = M->y;
    logger.data[LOG_MOMENTS_M_PROP_Z] = M->z;
}
