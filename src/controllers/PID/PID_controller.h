# pragma once

#include "src/controllers/PID/PID.h"
#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/guidance/guidance.h"


typedef struct {
    ControlVector U_cmd;

    PID alt2pitchPID;
    PID pitch2elvPID;
    PID airspeed2throtPID;
} ControlSystemPID;


void computeFlightControlPID(StateVector* X_est, GuidanceRefs* guidanceRefs, AircraftParams* acParams, ControlSystemPID* controlSystemPID, double dt_s);
