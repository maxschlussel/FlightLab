#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computeStateDerivative(const StateVector* X, const AircraftParams* ac, const Vector3* F, const Vector3* M, double* Xdot);
