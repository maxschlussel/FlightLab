#include <math.h>
#include <stdlib.h>

#include "src/core/constants.h"
#include "src/math/utils.h"
#include "src/sensors/pitot_tube.h"

#include "src/io/logger.h"


void readPitotTube(const StateVector* X, PitotTube* pitotTube){
    Vector3 V_b = {X->u, X->v, X->w}; // Velocities in body frame - V_b
    double velocity = vec3_norm(V_b);

    pitotTube->vel = velocity + randNoise(0.1);

    logger.data[LOG_SNS_PITOT_VEL] = pitotTube->vel;
}
