#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computePropulsionForces(const ControlVector* controlVector, const AircraftParams* acParams, Vector3* F, Vector3* M);
