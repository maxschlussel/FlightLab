#pragma once

#include "src/core/state_vector.h"
#include "src/sensors/sensors.h"

void estimateState(Sensors* sensors, StateVector* X_est);
