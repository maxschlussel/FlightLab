#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computeGravityForces(StateVector* X, AircraftParams* ac_parapms, Vector3* F);
