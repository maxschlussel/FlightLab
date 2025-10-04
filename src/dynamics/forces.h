#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"

void computeForcesAndMoments(const StateVector* X, const ControlVector* U, const AircraftParams* acParams, Vector3* F_tot, Vector3* M_tot);
