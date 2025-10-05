#include <math.h>
#include <stdlib.h>

#include "src/core/constants.h"
#include "src/math/utils.h"
#include "src/sensors/pitot_tube.h"

#include "src/io/logger.h"


/**
 * @brief Reads the simulated Pitot tube sensor.
 *
 * @param[in]   X           Pointer to the current state vector.
 * @param[out]  pitotTube   Pointer to the Pitot tube sensor struct
 */
void readPitotTube(const StateVector* X, PitotTube* pitotTube){
    // Module in development...

    Vector3 V_b = {X->u, X->v, X->w}; // Velocities in body frame - V_b
    double velocity = vec3_norm(V_b);

    pitotTube->vel = velocity + randNoise(0.1);

    logger.data[LOG_SNS_PITOT_VEL] = pitotTube->vel;
}
