#pragma once

#include "src/actuators/actuators.h"
#include "src/core/aircraft_params.h"
#include "src/math/vector.h"


void computePropulsionForces(const Actuators* actuators, const AircraftModel* acModel, Vector3* F, Vector3* M);
