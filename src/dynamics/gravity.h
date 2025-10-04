#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computeGravityForces(const StateVector* X, const AircraftParams* ac_parapms, Vector3* F);
