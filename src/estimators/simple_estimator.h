#pragma once

#include "src/core/state_vector.h"
#include "src/sensors/aircraft_sensors.h"
#include "src/math/vector.h"

void estimateStateSimple(const Sensors* sensors, StateVector* X_est, double dt);

Vector3 estimateAttitudeCF(const Sensors* sensors, StateVector* X_est, double dt);

double computeHeadingFromMag(const Vector3* mag, double phi_est, double theta_est);

Vector3 estimateVelCF(const Vector3* eulerAngles_est, const Sensors* sensors, StateVector* X_est, double dt);

Vector3 estimatePosCF(const Vector3* eulerAngles_est, const Vector3* vel_b_est, const Sensors* sensors, StateVector* X_est, double dt);
