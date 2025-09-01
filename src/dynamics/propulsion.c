#include "src/dynamics/propulsion.h"

void computePropulsionForces(StateVector* X, ControlVector* U, AircraftParams* acParams, 
        Vector3* F, Vector3* M){

    double Fthrottle1 = U->dt[0] * acParams->mass * g;
    double Fthrottle2 = U->dt[1] * acParams->mass * g;

    Vector3 FengineOne = {Fthrottle1, 0, 0};
    Vector3 FengineTwo = {Fthrottle2, 0, 0};

    Vector3 FengineTot_b = vec3_add(FengineOne, FengineTwo);

    Vector3 MomentEngineOne_b = vec3_cross(acParams->r_engineOne2cg, FengineOne);
    Vector3 MomentEngineTwo_b = vec3_cross(acParams->r_engineOne2cg, FengineTwo);

    Vector3 MengineTot_b = vec3_add(MomentEngineOne_b, MomentEngineTwo_b);

    F->x = FengineTot_b.x;
    F->y = FengineTot_b.y;
    F->z = FengineTot_b.z;
    M->x = MengineTot_b.x;
    M->y = MengineTot_b.y;
    M->z = MengineTot_b.z;
}
