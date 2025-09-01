#pragma once

#include "src/core/state_vector.h"
#include "src/math/vector.h"


typedef struct {
    Vector3 pos;
} GPS;


void readGPS(const StateVector* X, GPS* gps);
