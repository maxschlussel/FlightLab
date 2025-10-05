#pragma once

#include "src/core/aircraft_params.h"
#include "src/actuators/actuators.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"

void computeForcesAndMoments(const StateVector* X, const Actuators* actuators, const AircraftParams* acParams, Vector3* F_tot, Vector3* M_tot);
