#pragma once

#include "src/core/state_vector.h"
#include "src/math/vector.h"


typedef struct {
    double vel;
} PitotTube;

void readPitotTube(const StateVector* X, PitotTube* pitotTube);
