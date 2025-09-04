#include <math.h>
#include <stdlib.h>

#include "src/core/constants.h"
#include "src/math/utils.h"
#include "src/sensors/pitot_tube.h"

#include "src/io/logger.h"


void readPitotTube(const StateVector* X, PitotTube* pitotTube){
    pitotTube->vel.x = X->u + randNoise(0.001);
    pitotTube->vel.y = X->v + randNoise(0.001);
    pitotTube->vel.z = X->w + randNoise(0.001);

    logger.data[LOG_SNS_PITOT_VEL_X] = pitotTube->vel.x;
    logger.data[LOG_SNS_PITOT_VEL_Y] = pitotTube->vel.y;
    logger.data[LOG_SNS_PITOT_VEL_Z] = pitotTube->vel.z;
}
