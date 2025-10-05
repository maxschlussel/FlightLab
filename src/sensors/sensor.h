#pragma once

#include "src/math/vector.h"

/**
 * Represents a 3 dimension hardware sensor with bias/drift and noise.
 */
typedef struct
{
    Vector3 data;       // Actual sensor data
    
    Vector3 bias;       // Moving bias (random walk)
    const double sigmaWalk;   // Sigma for random walk []
    const double sigmaNoise;  // Sigma for random noise
} Sensor;