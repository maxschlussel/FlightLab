#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/actuators/actuators.h"


void integrateRK4Step(StateVector* X, Actuators* actuators, AircraftParams* acParams, const double Xdot[12], double dt_s);
