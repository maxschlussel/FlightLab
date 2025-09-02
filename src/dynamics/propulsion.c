#include "src/dynamics/propulsion.h"

void computePropulsionForces(StateVector* X, ControlVector* U, AircraftParams* acParams, 
        Vector3* F, Vector3* M){

    double Fthrottle1 = U->dt[0] * acParams->mass * g;
    double Fthrottle2 = U->dt[1] * acParams->mass * g;

    Vector3 FengineOne = {Fthrottle1, 0, 0};
    Vector3 FengineTwo = {Fthrottle2, 0, 0};

    *F = vec3_add(FengineOne, FengineTwo);

    Vector3 MomentEngineOne_b = vec3_cross(acParams->r_engineOne2cg, FengineOne);
    Vector3 MomentEngineTwo_b = vec3_cross(acParams->r_engineOne2cg, FengineTwo);

    *M = vec3_add(MomentEngineOne_b, MomentEngineTwo_b);
}
