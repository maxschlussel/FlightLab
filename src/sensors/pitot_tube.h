#pragma once

#include "src/core/state_vector.h"
#include "src/math/vector.h"


typedef struct {
    Vector3 vel;
} PitotTube;

void readPitotTube(const StateVector* X, PitotTube* pitotTube);
