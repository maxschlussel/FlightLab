#pragma once

#include "src/sensors/sensor.h"
#include "src/core/state_vector.h"

typedef Sensor Magnetometer;

void readMagnetometer(const StateVector* X, Magnetometer* mag, double dt);
