#pragma once

#include "src/core/state_vector.h"
#include "src/math/vector.h"

/**
 * Basic representation of a Pitot tube sensor.
 */

typedef struct {
    double vel;
} PitotTube;

void readPitotTube(const StateVector* X, PitotTube* pitotTube);
