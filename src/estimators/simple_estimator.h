#pragma once

#include "src/core/state_vector.h"
#include "src/sensors/aircraft_sensors.h"

void estimateStateSimple(Sensors* sensors, StateVector* X_est, double dt);

Vector3 estimateAttitudeCF(Sensors* sensors, StateVector* X_est, double dt);

Vector3 estimateVelCF(Vector3* eulerAngles_est, Sensors* sensors, StateVector* X_est, double dt);

Vector3 estimatePosCF(Vector3* eulerAngles_est, Vector3* vel_b_est, Sensors* sensors, StateVector* X_est, double dt);
