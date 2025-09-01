#pragma once

#include "src/core/state_vector.h"


typedef struct {
    double accel[3];
    double gyro[3];
} IMUSensor;

void readIMUSensor(const StateVector* X, IMUSensor* imuSensor);
