#include <math.h>
#include <stdlib.h>

#include "src/math/dcm.h"
#include "src/math/matrix.h"
#include "src/math/utils.h"
#include "src/sensors/gps.h"

#include "src/io/logger.h"


/**
 * @brief Reads the simulated GPS sensor.
 *
 * @param[in]   X       Pointer to the current state vector.
 * @param[out]  gps     Pointer to the GPS sensor struct
 */
void readGPS(const StateVector* X, GPS* gps, double dt){
    Vector3 V_b = {X->u, X->v, X->w}; // Velocities in body frame - V_b

    // Position
    gps->pos.bias.x += randNoise(gps->pos.sigmaWalk * sqrt(dt));
    gps->pos.bias.y += randNoise(gps->pos.sigmaWalk * sqrt(dt));
    gps->pos.bias.z += randNoise(gps->pos.sigmaWalk * sqrt(dt));
    
    gps->pos.data.x = X->x + gps->pos.bias.x + randNoise(gps->pos.sigmaNoise);
    gps->pos.data.y = X->y + gps->pos.bias.y + randNoise(gps->pos.sigmaNoise);
    gps->pos.data.z = X->z + gps->pos.bias.z + randNoise(gps->pos.sigmaNoise);
    
    // Velocity
    double R_e2b[3][3];  // Earth 2 body dcm
    getRotationMatrix(X->phi, X->theta, X->psi, R_e2b);
    double R_b2e[3][3];
    mat3_transpose(R_e2b, R_b2e);
    Vector3 dPos = mat3_mult_vec3(R_b2e, V_b);
    
    gps->vel.data.x = dPos.x + randNoise(gps->vel.sigmaNoise);;
    gps->vel.data.y = dPos.y + randNoise(gps->vel.sigmaNoise);;
    gps->vel.data.z = dPos.z + randNoise(gps->vel.sigmaNoise);;

    logger.data[LOG_SNS_GPS_POS_X] = gps->pos.data.x;
    logger.data[LOG_SNS_GPS_POS_Y] = gps->pos.data.y;
    logger.data[LOG_SNS_GPS_POS_Z] = gps->pos.data.z;
    logger.data[LOG_SNS_GPS_VEL_X] = gps->vel.data.x;
    logger.data[LOG_SNS_GPS_VEL_Y] = gps->vel.data.y;
    logger.data[LOG_SNS_GPS_VEL_Z] = gps->vel.data.z;
}
