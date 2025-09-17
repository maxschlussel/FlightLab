#include "src/sim/scenarios.h"
#include "src/core/state_vector.h"



/**
* @brief Load the initial conditions of the aircraft control vector.
*/
ControlVector initControlVectorlBasic(void){
    ControlVector U_cmd = {
        .da = 0.0,
        .de = -5.0*deg2rad,
        .dr = 0.0,
        .dt[0] = 0.0,
        .dt[1] = 0.0
    };
    return U_cmd;
}

/**
* @brief Load the initial conditions of the aircraft state.
*/
StateVector initStateVectorBasicCruise(void){
    StateVector X = {
        .u = 80.0,
        .v = 0.0,
        .w = 0.0,
        .p = 0.0,
        .q = 0.0,
        .r = 0.0,
        .phi    = 0.1,
        .theta  = 0.1,
        .psi    = 0.1,
        .x = 0.0,
        .y = 0.0,
        .z = -1000.0
    };
    return X;
}
