#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computeStateDerivative(const StateVector* X, AircraftParams* ac, Vector3* F, Vector3* M, double* Xdot);
