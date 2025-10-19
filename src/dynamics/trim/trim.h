#pragma once


#include "src/dynamics/trim/trim.h"
#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/core/control_vector.h"
#include "src/dynamics/aerodynamics.h"

int solveTrimLM(const StateVector *X, ControlVector *U, const AircraftParams *acParams);
