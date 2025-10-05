#pragma once

#include "src/actuators/actuators.h"
#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computePropulsionForces(const Actuators* actuators, const AircraftParams* acParams, Vector3* F, Vector3* M);
