#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"

void computeForcesAndMoments(StateVector* X, ControlVector* U, AircraftParams* acParams, Vector3* F_tot, Vector3* M_tot);
