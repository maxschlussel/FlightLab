#pragma once

#include "src/core/state_vector.h"
#include "src/math/vector.h"


/**
 * Basic representation of an IMU sensor.
 */

typedef struct {
    Vector3 accel;
    Vector3 gyro;
} IMUSensor;

void readIMUSensor(const StateVector* X, IMUSensor* imuSensor);
