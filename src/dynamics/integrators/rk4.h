#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/actuators/actuators.h"


void integrateRK4Step(StateVector* X, const Actuators* actuators, const AircraftModel* acModel, const double Xdot[12], double dt_s);
