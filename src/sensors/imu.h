#pragma once

#include "src/core/state_vector.h"
#include "src/math/vector.h"
#include "src/sensors/sensor.h"


/**
 * Basic representation of an IMU sensor containing accelerometer and gyroscope.
 */

typedef struct {
    Sensor accel;
    Sensor gyro;
} IMUSensor;

void readIMUSensor(const StateVector* X, const double* Xdot, double dt, IMUSensor* imuSensor);
