#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/guidance/guidance.h"
#include "src/sensors/sensors.h"


void computeFlightControlCmd(StateVector* X_est, GuidanceRefs* guidanceRefs, AircraftParams* acParams, ControlVector* U_cmd);
