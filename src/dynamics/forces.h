#pragma once

#include "src/actuators/actuators.h"
#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/dynamics/aerodynamics.h"
#include "src/math/vector.h"

void computeForcesAndMoments(const StateVector* X, const Actuators* actuators, const AircraftModel* acModel, AeroData* aerodata);
