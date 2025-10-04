#pragma once

#include "src/core/state_vector.h"
#include "src/math/vector.h"

/**
 * Basic representation of a gps sensor.
 */
typedef struct {
    bool gps_valid;
    Vector3 pos;
    Vector3 vel;
} GPS;


void readGPS(const StateVector* X, GPS* gps);
