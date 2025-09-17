#pragma once

#include "src/core/state_vector.h"
#include "src/sensors/sensors.h"

void estimateStateEKF(Sensors* sensors, StateVector* X_est);
