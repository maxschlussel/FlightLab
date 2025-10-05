#pragma once

#include "src/core/state_vector.h"
#include "src/sensors/sensor.h"

/**
 * Basic representation of a gps sensor.
 */
typedef struct {
    bool gps_valid;
    Sensor pos;
    Sensor vel;
} GPS;


void readGPS(const StateVector* X, GPS* gps, double dt);
